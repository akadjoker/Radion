#include "PCH.h"

#include "ShadowDebugView.h"

namespace Radion
{
namespace
{

constexpr char kVertexSource[] = R"GLSL(#version 450 core
layout(location = 0) out vec2 uv;
void main()
{
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    uv = p;
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)GLSL";

// All texture shapes sit on different bindings so they can stay declared at
// once; params.x selects the one used by this draw.
constexpr char kFragmentSource[] = R"GLSL(#version 450 core
layout(binding = 0) uniform sampler2D uPlain;
layout(binding = 1) uniform sampler2DArray uArray;
layout(binding = 2) uniform samplerCube uCube;
layout(std140, binding = 0) uniform ShadowDebugBlock
{
    vec4 params; // x = 0 plain, 1 array, 2 cube; y = layer/face; z = mip
    vec4 sourceRect; // xy = source scale, zw = source offset
};
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outColor;
void main()
{
    if (params.x > 1.5)
    {
        vec2 p = uv * 2.0 - 1.0;
        int face = int(params.y + 0.5);
        vec3 direction = face == 0 ? vec3( 1.0, -p.y, -p.x) :
                         face == 1 ? vec3(-1.0, -p.y,  p.x) :
                         face == 2 ? vec3( p.x,  1.0,  p.y) :
                         face == 3 ? vec3( p.x, -1.0, -p.y) :
                         face == 4 ? vec3( p.x, -p.y,  1.0) :
                                     vec3(-p.x, -p.y, -1.0);
        // Probe faces are stored as linear HDR.  The debug target is RGBA8, so
        // map them to display space instead of clipping (or leaving dim linear
        // values looking black in the editor).
        vec3 hdr = max(textureLod(uCube, normalize(direction), params.z).rgb, vec3(0.0));
        vec3 mapped = hdr / (hdr + vec3(1.0));
        mapped = pow(mapped, vec3(1.0 / 2.2));
        outColor = vec4(mapped, 1.0);
        return;
    }
    vec2 sourceUV = uv * sourceRect.xy + sourceRect.zw;
    float value = params.x > 0.5 ? texture(uArray, vec3(sourceUV, params.y)).r
                                 : texture(uPlain, sourceUV).r;
    outColor = vec4(vec3(value), 1.0);
}
)GLSL";

} // namespace

bool ShadowDebugView::setup()
{
    GPU& gpu = GPU::getSingleton();

    PipelineDesc pipeline;
    pipeline.vs = {kVertexSource, 0, "shadow.debug.vert"};
    pipeline.fs = {kFragmentSource, 0, "shadow.debug.frag"};
    pipeline.depth.test = false;
    pipeline.depth.write = false;
    pipeline.raster.cull = CullMode::None;
    pipeline.debugName = "shadow.debug";
    mPipeline = gpu.createPipeline(pipeline);

    BufferDesc uniform;
    uniform.size = sizeof(Math::vec4) * 2;
    uniform.usage = BufferUniform;
    uniform.residency = Residency::Stream;
    uniform.debugName = "shadow.debug.uniform";
    mUniform = gpu.createBuffer(uniform);

    SamplerDesc sampler;
    sampler.filter = Filter::Linear;
    sampler.wrapU = Wrap::Clamp;
    sampler.wrapV = Wrap::Clamp;
    mSampler = gpu.createSampler(sampler);

    return mPipeline.valid() && mUniform.valid() && mSampler.valid();
}

void ShadowDebugView::shutdown()
{
    GPU& gpu = GPU::getSingleton();
    gpu.destroy(mPipeline);
    gpu.destroy(mUniform);
    gpu.destroy(mSampler);
    mPipeline = PipelineHandle();
    mUniform = BufferHandle();
    mSampler = SamplerHandle();
}

void ShadowDebugView::blit(TextureHandle texture, bool isArray, u32 layer, TargetHandle target,
                           const Viewport& viewport, const Math::vec4& sourceRect)
{
    if (!texture.valid() || !mPipeline.valid())
        return;

    const Math::vec4 values[2] = {
        Math::vec4(isArray ? 1.0f : 0.0f, static_cast<f32>(layer), 0.0f, 0.0f), sourceRect};
    GPU& gpu = GPU::getSingleton();
    gpu.setTarget(target);
    gpu.setViewport(viewport);
    gpu.setPipeline(mPipeline);
    gpu.updateBuffer(mUniform, 0, sizeof(values), values);
    gpu.bindUniform(0, mUniform);
    gpu.bindTexture(isArray ? 1 : 0, texture, mSampler);

    DrawDesc command;
    command.count = 3;
    gpu.draw(command);
}

void ShadowDebugView::drawCascades(TextureHandle cascades, u32 count, u32 windowWidth,
                                   u32 windowHeight)
{
    constexpr f32 size = 170.0f;
    constexpr f32 gap = 6.0f;
    const f32 baseY = static_cast<f32>(windowHeight) - size - 8.0f;
    for (u32 c = 0; c < count; ++c)
    {
        const Viewport viewport{8.0f + static_cast<f32>(c) * (size + gap), baseY, size, size};
        // The directional texture is now one 2D atlas. Show each split's
        // region separately so the existing four-cascade diagnostic remains
        // useful rather than displaying the whole atlas four times.
        const DirectionalShadowRegion region = directionalShadowRegion(2, count, c);
        blit(cascades, false, 0, TargetHandle(), viewport,
             Math::vec4(static_cast<f32>(region.width) * 0.5f,
                       static_cast<f32>(region.height) * 0.5f,
                       static_cast<f32>(region.x) * 0.5f,
                       static_cast<f32>(region.y) * 0.5f));
    }
    (void)windowWidth;
}

void ShadowDebugView::drawAtlas(TextureHandle atlas, u32 windowWidth, u32 windowHeight)
{
    constexpr f32 size = 220.0f;
    const Viewport viewport{static_cast<f32>(windowWidth) - size - 8.0f,
                            static_cast<f32>(windowHeight) - size - 8.0f, size, size};
    blit(atlas, false, 0, TargetHandle(), viewport);
}

void ShadowDebugView::drawTexture(TextureHandle texture, bool isArray, u32 layer,
                                  TargetHandle target, u32 width, u32 height,
                                  const Math::vec4& sourceRect)
{
    blit(texture, isArray, layer, target,
         Viewport{0.0f, 0.0f, static_cast<f32>(width), static_cast<f32>(height)}, sourceRect);
}

void ShadowDebugView::blitCubemap(TextureHandle texture, u32 face, u32 mip,
                                  TargetHandle target, const Viewport& viewport)
{
    if (!texture.valid() || !mPipeline.valid())
        return;

    const Math::vec4 values[2] = {
        Math::vec4(2.0f, static_cast<f32>(face), static_cast<f32>(mip), 0.0f),
        Math::vec4(1.0f, 1.0f, 0.0f, 0.0f)};
    GPU& gpu = GPU::getSingleton();
    gpu.setTarget(target);
    gpu.setViewport(viewport);
    gpu.setPipeline(mPipeline);
    gpu.updateBuffer(mUniform, 0, sizeof(values), values);
    gpu.bindUniform(0, mUniform);
    gpu.bindTexture(2, texture, mSampler);

    DrawDesc command;
    command.count = 3;
    gpu.draw(command);
}

void ShadowDebugView::drawCubemap(TextureHandle texture, u32 face, u32 mip,
                                  TargetHandle target, u32 width, u32 height)
{
    blitCubemap(texture, face, mip, target,
                Viewport{0.0f, 0.0f, static_cast<f32>(width), static_cast<f32>(height)});
}

} // namespace Radion
