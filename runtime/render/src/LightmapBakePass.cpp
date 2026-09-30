#include "PCH.h"

#include "LightmapBakePass.h"

#include "AssetManager.h"
#include "CameraBlock.h"
#include "Log.h"
#include "MaterialManager.h"
#include "Mesh.h"
#include "Pixmap.h"

#include "Math.h"

namespace Radion
{

namespace
{
struct BakeBlock
{
    Math::mat4 shadowViewProjection;
    Math::mat4 model;
    Math::vec4 lightDirection;
    Math::vec4 lightColor;
    // x = depth bias (already divided by the shadow map's own depth range),
    // y = ground bounce as a fraction of the sky term, z = PCF filter radius
    // in shadow texels, w = per-sample weight.
    Math::vec4 params;
    // rgb = sky light; see LightmapBakeSettings::ambient.
    Math::vec4 ambientSky;
    // xy = this sample's rasterization offset in clip space (Halton); jittering the texel position stops border texels depending on one sample, the seam artifact.
    Math::vec4 jitter;
};

f32 halton(u32 index, u32 base)
{
    f32 result = 0.0f;
    f32 fraction = 1.0f / static_cast<f32>(base);
    for (u32 i = index + 1; i > 0; i /= base)
    {
        result += static_cast<f32>(i % base) * fraction;
        fraction /= static_cast<f32>(base);
    }
    return result;
}

// Corners of `bounds` in `view` space as min/max; sizing the sun frustum from the box radius would use the diagonal and waste texels.
void projectBounds(const AABB& bounds, const Math::mat4& view, Math::vec3& minimum,
                   Math::vec3& maximum)
{
    minimum = Math::vec3(1.0e30f);
    maximum = Math::vec3(-1.0e30f);
    for (u32 corner = 0; corner < 8; ++corner)
    {
        const Math::vec3 point((corner & 1) ? bounds.max.x : bounds.min.x,
                              (corner & 2) ? bounds.max.y : bounds.min.y,
                              (corner & 4) ? bounds.max.z : bounds.min.z);
        const Math::vec3 viewPoint = Math::vec3(view * Math::vec4(point, 1.0f));
        minimum = Math::min(minimum, viewPoint);
        maximum = Math::max(maximum, viewPoint);
    }
}

// Bleed real texels outward into xatlas chart padding (alpha = coverage) so bilinear sampling at chart edges shows no dark seam.
void dilateLightmap(std::vector<f32>& pixels, u32 resolution, u32 iterations)
{
    for (u32 iteration = 0; iteration < iterations; ++iteration)
    {
        std::vector<f32> next = pixels;
        bool changed = false;
        for (u32 y = 0; y < resolution; ++y)
            for (u32 x = 0; x < resolution; ++x)
            {
                const usize i = (static_cast<usize>(y) * resolution + x) * 4;
                if (pixels[i + 3] > 0.0f)
                    continue;
                Math::vec3 sum(0.0f);
                s32 count = 0;
                for (s32 dy = -1; dy <= 1; ++dy)
                    for (s32 dx = -1; dx <= 1; ++dx)
                    {
                        if (dx == 0 && dy == 0)
                            continue;
                        const s32 nx = static_cast<s32>(x) + dx;
                        const s32 ny = static_cast<s32>(y) + dy;
                        if (nx < 0 || ny < 0 || nx >= static_cast<s32>(resolution) ||
                            ny >= static_cast<s32>(resolution))
                            continue;
                        const usize ni = (static_cast<usize>(ny) * resolution + static_cast<usize>(nx)) * 4;
                        if (pixels[ni + 3] > 0.0f)
                        {
                            sum += Math::vec3(pixels[ni + 0], pixels[ni + 1], pixels[ni + 2]);
                            ++count;
                        }
                    }
                if (count > 0)
                {
                    const Math::vec3 average = sum / static_cast<f32>(count);
                    next[i + 0] = average.x;
                    next[i + 1] = average.y;
                    next[i + 2] = average.z;
                    next[i + 3] = 1.0f;
                    changed = true;
                }
            }
        pixels.swap(next);
        if (!changed)
            break;
    }
}

// Vogel disk offset in [-1,1]^2; same distribution as lit.frag's VOGEL[] for cascade PCF.
Math::vec2 vogelDisk(u32 index, u32 count)
{
    const f32 goldenAngle = 2.39996323f;
    const f32 radius = Math::sqrt((static_cast<f32>(index) + 0.5f) / static_cast<f32>(count));
    const f32 theta = static_cast<f32>(index) * goldenAngle;
    return radius * Math::vec2(Math::cos(theta), Math::sin(theta));
}

// Jitters the direction (not the eye position): correct for an orthographic light, equals sampling a point on the sun's disk.
Math::vec3 jitterSunDirection(const Math::vec3& direction, f32 angularRadius, u32 index, u32 count)
{
    if (angularRadius <= 0.0f || count <= 1)
        return direction;
    Math::vec3 reference = Math::abs(direction.y) > 0.99f ? Math::vec3(1.0f, 0.0f, 0.0f) : Math::vec3(0.0f, 1.0f, 0.0f);
    const Math::vec3 right = Math::normalize(Math::cross(reference, direction));
    const Math::vec3 up = Math::cross(direction, right);
    const Math::vec2 offset = vogelDisk(index, count) * Math::tan(Math::radians(angularRadius));
    return Math::normalize(direction + right * offset.x + up * offset.y);
}

PipelineHandle makePipeline(const char* vertexName, const char* fragmentName,
                            const VertexLayout& layout, bool depthOnly)
{
    const std::string& vs = Assets().loadShader(vertexName);
    const std::string& fs = Assets().loadShader(fragmentName);
    if (vs.empty() || fs.empty())
        return PipelineHandle();
    PipelineDesc desc;
    desc.vs = {vs.c_str(), 0, vertexName};
    desc.fs = {fs.c_str(), 0, fragmentName};
    desc.layout = layout;
    desc.raster.cull = CullMode::None;
    desc.depth.test = depthOnly;
    desc.depth.write = depthOnly;
    desc.depth.func = Compare::LessEqual;
    desc.blend.writeRGB = !depthOnly;
    desc.blend.writeA = !depthOnly;
    // Additive blend accumulates soft-shadow samples straight into the target.
    desc.blend.mode = depthOnly ? BlendMode::Opaque : BlendMode::Additive;
    desc.debugName = depthOnly ? "lightmap.shadow" : "lightmap.bake";
    return GPU::getSingleton().createPipeline(desc);
}
}

LightmapBakePass::~LightmapBakePass()
{
    shutdown();
}

bool LightmapBakePass::setup(const VertexLayout& layout)
{
    if (mShadowPipeline.valid() && std::memcmp(&mLayout, &layout, sizeof(layout)) == 0)
        return true;
    destroyResources();
    mLayout = layout;
    GPU& gpu = GPU::getSingleton();
    BufferDesc block;
    block.size = sizeof(BakeBlock);
    block.usage = BufferUniform;
    block.residency = Residency::Dynamic;
    block.debugName = "lightmap.block";
    mBlock = gpu.createBuffer(block);
    mShadowPipeline = makePipeline("lightmap_shadow.vert", "lightmap_shadow.frag", layout, true);
    mBakePipeline = makePipeline("lightmap_bake.vert", "lightmap_bake.frag", layout, false);
    return mBlock.valid() && mShadowPipeline.valid() && mBakePipeline.valid();
}

void LightmapBakePass::destroyResources()
{
    if (!GPU::ready())
        return;
    GPU& gpu = GPU::getSingleton();
    gpu.destroy(mShadowTarget);
    gpu.destroy(mLightmapTarget);
    gpu.destroy(mShadowSampler);
    gpu.destroy(mShadowMap);
    gpu.destroy(mLightmap);
    gpu.destroy(mBlock);
    gpu.destroy(mShadowPipeline);
    gpu.destroy(mBakePipeline);
    mShadowTarget = TargetHandle();
    mLightmapTarget = TargetHandle();
    mShadowSampler = SamplerHandle();
    mShadowMap = TextureHandle();
    mLightmap = TextureHandle();
    mBlock = BufferHandle();
    mShadowPipeline = PipelineHandle();
    mBakePipeline = PipelineHandle();
    mResolution = 0;
    mShadowResolution = 0;
}

void LightmapBakePass::shutdown()
{
    destroyResources();
}

bool LightmapBakePass::bake(MeshHandle meshHandle, const Math::mat4& model, const AABB& bounds,
                            const Math::vec3& lightDirection, const Math::vec3& lightColor,
                            u32 resolution, const LightmapBakeSettings& settings)
{
    const Mesh* mesh = Assets().getMesh(meshHandle);
    if (!mesh || mesh->isSkinned() || mesh->submeshes.empty())
    {
        Log::error("LightmapBakePass: invalid or skinned mesh");
        return false;
    }
    if (!setup(mesh->colorLayout))
    {
        Log::error("LightmapBakePass: failed to create bake pipelines");
        return false;
    }

    GPU& gpu = GPU::getSingleton();
    resolution = Math::clamp(resolution, 64u, 4096u);
    // Depth only, never read back, so it may be larger than the atlas.
    const u32 shadowResolution =
        Math::clamp(settings.shadowResolution ? settings.shadowResolution : resolution, 64u, 16384u);

    if (mShadowResolution != shadowResolution)
    {
        gpu.destroy(mShadowTarget);
        gpu.destroy(mShadowSampler);
        gpu.destroy(mShadowMap);

        TextureDesc shadow;
        shadow.format = Format::Depth32F;
        shadow.width = shadowResolution;
        shadow.height = shadowResolution;
        shadow.usage = TextureSampled | TextureTarget;
        shadow.debugName = "lightmap.shadow.depth";
        mShadowMap = gpu.createTexture(shadow);
        TargetDesc shadowTarget;
        shadowTarget.depth.texture = mShadowMap;
        shadowTarget.debugName = "lightmap.shadow.target";
        mShadowTarget = gpu.createTarget(shadowTarget);

        SamplerDesc sampler;
        sampler.filter = Filter::Linear;
        sampler.wrapU = Wrap::Clamp;
        sampler.wrapV = Wrap::Clamp;
        sampler.compare = false;
        mShadowSampler = gpu.createSampler(sampler);
        mShadowResolution = shadowResolution;
    }

    if (mResolution != resolution)
    {
        gpu.destroy(mLightmapTarget);
        gpu.destroy(mLightmap);

        TextureDesc color;
        color.format = Format::RGBA16F;
        color.width = resolution;
        color.height = resolution;
        color.usage = TextureSampled | TextureTarget;
        color.debugName = "lightmap.color";
        mLightmap = gpu.createTexture(color);
        TargetDesc colorTarget;
        colorTarget.colors[0].texture = mLightmap;
        colorTarget.colorCount = 1;
        colorTarget.debugName = "lightmap.color.target";
        mLightmapTarget = gpu.createTarget(colorTarget);
        mResolution = resolution;
    }
    if (!mShadowTarget.valid() || !mLightmapTarget.valid())
    {
        Log::error("LightmapBakePass: failed to create bake targets");
        return false;
    }

    const Math::vec3 direction = Math::normalize(lightDirection);
    // `bounds` is object space; build the shadow frustum from the transformed box to match the drawn mesh.
    const AABB worldBounds = transformAABB(bounds, model);
    const Math::vec3 center = worldBounds.center();
    const Math::vec3 extents = worldBounds.extents();
    const f32 radius = Math::max(Math::length(extents), 1.0f);

    auto drawMesh = [&](PipelineHandle pipeline, TargetHandle target, bool shadowPass, bool clearColor)
    {
        ClearValue clear;
        clear.bits = shadowPass ? ClearDepth : (clearColor ? ClearColor : 0u);
        clear.depth = 1.0f;
        clear.color[0] = clear.color[1] = clear.color[2] = 0.0f;
        // Alpha starts 0 and the bake shader writes 1: a coverage mask so save() can bleed colour into padding.
        clear.color[3] = 0.0f;
        gpu.setTarget(target, clear);
        const f32 side = static_cast<f32>(shadowPass ? shadowResolution : resolution);
        gpu.setViewport({0.0f, 0.0f, side, side, 0.0f, 1.0f});
        gpu.setPipeline(pipeline);
        if (!shadowPass)
            gpu.bindTexture(1, mShadowMap, mShadowSampler);
        for (const SubMesh& submesh : mesh->submeshes)
        {
            DrawDesc draw;
            draw.vertexBuffers[0] = mesh->positionBuffer;
            draw.vertexBuffers[1] = mesh->attribBuffer;
            draw.vertexBufferCount = 2;
            draw.indexBuffer = mesh->indexBuffer;
            draw.indexType = mesh->indexType;
            draw.first = submesh.indexOffset;
            draw.count = submesh.indexCount;
            gpu.draw(draw);
        }
    };

    const u32 samples = Math::clamp(settings.sampleCount, 1u, 64u);
    const f32 weight = 1.0f / static_cast<f32>(samples);
    for (u32 sample = 0; sample < samples; ++sample)
    {
        const Math::vec3 sampleDirection =
            jitterSunDirection(direction, settings.sunAngularRadius, sample, samples);
        Math::vec3 up(0.0f, 1.0f, 0.0f);
        if (Math::abs(Math::dot(up, sampleDirection)) > 0.95f)
            up = Math::vec3(1.0f, 0.0f, 0.0f);
        const Math::mat4 shadowView = Math::lookAt(center - sampleDirection * radius * 2.5f, center, up);

        // Fitted to the geometry in the sun's view; Math::lookAt looks down -z so near/far are -max.z/-min.z, with margin so a caster on the plane is not clipped.
        Math::vec3 viewMinimum, viewMaximum;
        projectBounds(worldBounds, shadowView, viewMinimum, viewMaximum);
        const f32 margin = Math::max(radius * 0.01f, 0.01f);
        const f32 nearPlane = Math::max(-viewMaximum.z - margin, 0.01f);
        const f32 farPlane = -viewMinimum.z + margin;
        const Math::mat4 shadowProjection =
            Math::ortho(viewMinimum.x - margin, viewMaximum.x + margin, viewMinimum.y - margin,
                       viewMaximum.y + margin, nearPlane, farPlane);

        BakeBlock block;
        block.shadowViewProjection = shadowProjection * shadowView;
        block.model = model;
        block.lightDirection = Math::vec4(sampleDirection, 0.0f);
        block.lightColor = Math::vec4(lightColor, 1.0f);
        // Bias from texels to world units to ortho depth against this sample's frustum: one setting works at any scene size.
        const f32 depthRange = Math::max(farPlane - nearPlane, 0.001f);
        const f32 texelWorldSize =
            (viewMaximum.x - viewMinimum.x + 2.0f * margin) / static_cast<f32>(shadowResolution);
        const f32 worldBias =
            settings.bias > 0.0f ? settings.bias : settings.biasTexels * texelWorldSize;
        block.params = Math::vec4(worldBias / depthRange, settings.ambientGround,
                                 settings.filterRadius, weight);
        block.ambientSky = Math::vec4(settings.ambient, 0.0f);
        // Bases 2 and 3, widened slightly past one texel so samples reach neighbours a border texel lacks coverage from.
        const f32 texelWidth = 2.0f / static_cast<f32>(mResolution);
        block.jitter =
            Math::vec4((halton(sample, 2) * 2.0f - 1.0f) * texelWidth * 0.7f,
                      (halton(sample, 3) * 2.0f - 1.0f) * texelWidth * 0.7f, 0.0f, 0.0f);
        gpu.updateBuffer(mBlock, 0, sizeof(block), &block);
        gpu.bindUniform(0, mBlock);

        drawMesh(mShadowPipeline, mShadowTarget, true, true);
        drawMesh(mBakePipeline, mLightmapTarget, false, sample == 0);
    }
    return true;
}

bool LightmapBakePass::save(const std::string& filename) const
{
    if (!mLightmap.valid() || mResolution == 0)
        return false;
    std::vector<f32> pixels(static_cast<usize>(mResolution) * mResolution * 4);
    if (!GPU::getSingleton().readColorPixels(mLightmap, 0, 0, mResolution, mResolution,
                                             pixels.data(), static_cast<u32>(pixels.size())))
    {
        Log::error("LightmapBakePass: failed to read RGBA target");
        return false;
    }
    dilateLightmap(pixels, mResolution, 8);
    f32 minimum = 1.0e30f;
    f32 maximum = -1.0e30f;
    for (usize i = 0; i < pixels.size(); i += 4)
    {
        minimum = Math::min(minimum, Math::min(pixels[i], Math::min(pixels[i + 1], pixels[i + 2])));
        maximum = Math::max(maximum, Math::max(pixels[i], Math::max(pixels[i + 1], pixels[i + 2])));
    }
    Log::info("LightmapBakePass: readback range %.4f .. %.4f", static_cast<double>(minimum),
              static_cast<double>(maximum));
    Pixmap image(static_cast<int>(mResolution), static_cast<int>(mResolution), 4);
    for (u32 y = 0; y < mResolution; ++y)
        for (u32 x = 0; x < mResolution; ++x)
        {
            const usize i = (static_cast<usize>(y) * mResolution + x) * 4;
            image.set_pixel(x, y,
                            static_cast<u8>(Math::clamp(pixels[i + 0], 0.0f, 1.0f) * 255.0f),
                            static_cast<u8>(Math::clamp(pixels[i + 1], 0.0f, 1.0f) * 255.0f),
                            static_cast<u8>(Math::clamp(pixels[i + 2], 0.0f, 1.0f) * 255.0f), 255);
        }
    // No flip: row 0 stays GL's row 0, matching AssetManager::loadTexture() and the bake vertex shader (aUV2.y=0 -> row 0).
    const bool saved = image.save(filename.c_str());
    if (!saved)
        Log::error("LightmapBakePass: failed to save '%s'", filename.c_str());
    return saved;
}

void LightmapBakePass::applyToMaterials(std::vector<Material>& materials, const VertexLayout& colorLayout,
                                        TextureHandle lightmapTexture, const std::string& lightmapFile)
{
    // Clamp, not Repeat: sampling at the atlas edge must not wrap into an unrelated chart.
    SamplerDesc samplerDesc;
    samplerDesc.filter = Filter::Linear;
    samplerDesc.wrapU = Wrap::Clamp;
    samplerDesc.wrapV = Wrap::Clamp;
    const SamplerHandle sampler = Assets().getSampler(samplerDesc);

    for (Material& material : materials)
    {
        // Lit stays on: unlit.frag ignores point/spot lights. Only ReceiveShadow comes off (sun is baked; HAS_LIGHTMAP in lit.frag).
        material.flags &= ~MaterialReceiveShadow;
        material.textures[SlotLightmap].texture = lightmapTexture;
        material.textures[SlotLightmap].sampler = sampler;
        material.textures[SlotLightmap].file = lightmapFile;
        material.textures[SlotLightmap].source = TextureSource::Static;
        MaterialManager::getSingleton().resolvePipeline(material, colorLayout);
    }
}

} // namespace Radion
