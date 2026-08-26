#include "PCH.h"

#include "LightmapBakePass.h"

#include "AssetManager.h"
#include "CameraBlock.h"
#include "Log.h"
#include "MaterialManager.h"
#include "Mesh.h"
#include "Pixmap.h"

#include <glm/gtc/matrix_transform.hpp>

namespace Radion
{

namespace
{
struct BakeBlock
{
    glm::mat4 shadowViewProjection;
    glm::mat4 model;
    glm::vec4 lightDirection;
    glm::vec4 lightColor;
    // x = depth bias (already divided by the shadow map's own depth range),
    // y = ground bounce as a fraction of the sky term, z = PCF filter radius
    // in shadow texels, w = per-sample weight.
    glm::vec4 params;
    // rgb = sky light; see LightmapBakeSettings::ambient.
    glm::vec4 ambientSky;
    // xy = this sample's rasterization offset, in clip space. Jittering WHERE
    // the texel is rasterized (as opposed to jitterSunDirection(), which
    // jitters the light) is what stops a chart's border texels from being
    // decided by one arbitrary sample position - the artifact along every
    // seam. Halton, so successive samples fill the pixel evenly instead of
    // clumping the way a random pair does at low counts.
    glm::vec4 jitter;
};

// Van der Corput in `base`, the one-dimensional building block of a Halton
// sequence: index 1,2,3... reflected about the radix point.
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

// The eight corners of `bounds` in `view` space, as a min/max pair. What the
// sun's ortho frustum is fitted to: sizing it from the box's radius instead
// means sizing it to the box's DIAGONAL, which on a wide flat map is close to
// half the texels spent on empty space either side of the geometry.
void projectBounds(const AABB& bounds, const glm::mat4& view, glm::vec3& minimum,
                   glm::vec3& maximum)
{
    minimum = glm::vec3(1.0e30f);
    maximum = glm::vec3(-1.0e30f);
    for (u32 corner = 0; corner < 8; ++corner)
    {
        const glm::vec3 point((corner & 1) ? bounds.max.x : bounds.min.x,
                              (corner & 2) ? bounds.max.y : bounds.min.y,
                              (corner & 4) ? bounds.max.z : bounds.min.z);
        const glm::vec3 viewPoint = glm::vec3(view * glm::vec4(point, 1.0f));
        minimum = glm::min(minimum, viewPoint);
        maximum = glm::max(maximum, viewPoint);
    }
}

// Chart interiors are the only texels the raster pass ever touches; the
// padding xatlas reserved around each chart stays at the clear colour
// (alpha 0). Bilinear sampling right at a chart edge blends real colour
// with that empty padding, which reads as a dark seam along every chart
// boundary. Growing real texels outward into the padding by a few texels
// (alpha as the coverage mask) is what every lightmap baker does to hide
// this - not a bug in the raster itself, just a required extra step.
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
                glm::vec3 sum(0.0f);
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
                            sum += glm::vec3(pixels[ni + 0], pixels[ni + 1], pixels[ni + 2]);
                            ++count;
                        }
                    }
                if (count > 0)
                {
                    const glm::vec3 average = sum / static_cast<f32>(count);
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

// Vogel disk offset for sample `index` of `count`, in [-1, 1]^2. Same
// distribution lit.frag's own VOGEL[] table uses for cascade PCF - low
// discrepancy without the periodic banding a regular grid gives at low
// counts.
glm::vec2 vogelDisk(u32 index, u32 count)
{
    const f32 goldenAngle = 2.39996323f;
    const f32 radius = glm::sqrt((static_cast<f32>(index) + 0.5f) / static_cast<f32>(count));
    const f32 theta = static_cast<f32>(index) * goldenAngle;
    return radius * glm::vec2(glm::cos(theta), glm::sin(theta));
}

// Rotates `direction` by up to `angularRadius` degrees towards a point on
// the sun's disk, sample `index` of `count`. Jittering the direction (not
// the shadow frustum's eye position) is the correct thing to do for an
// orthographic light: it turns the whole bundle of parallel rays by a tiny
// angle, exactly what a different point on a distant, angularly-small sun
// would produce.
glm::vec3 jitterSunDirection(const glm::vec3& direction, f32 angularRadius, u32 index, u32 count)
{
    if (angularRadius <= 0.0f || count <= 1)
        return direction;
    glm::vec3 reference = glm::abs(direction.y) > 0.99f ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::vec3 right = glm::normalize(glm::cross(reference, direction));
    const glm::vec3 up = glm::cross(direction, right);
    const glm::vec2 offset = vogelDisk(index, count) * glm::tan(glm::radians(angularRadius));
    return glm::normalize(direction + right * offset.x + up * offset.y);
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
    // Soft shadows come from averaging several samples of the sun's angular
    // disk (see bake()'s sample loop below) - additive blend lets each
    // sample accumulate straight into the target instead of a separate
    // readback+average step per sample.
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

bool LightmapBakePass::bake(MeshHandle meshHandle, const glm::mat4& model, const AABB& bounds,
                            const glm::vec3& lightDirection, const glm::vec3& lightColor,
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
    resolution = glm::clamp(resolution, 64u, 4096u);
    // Only a depth texture, never read back - it can afford to be larger than
    // the atlas, which is the whole point of letting it be set apart.
    const u32 shadowResolution =
        glm::clamp(settings.shadowResolution ? settings.shadowResolution : resolution, 64u, 16384u);

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

    const glm::vec3 direction = glm::normalize(lightDirection);
    // `bounds` is in the mesh's own object space; the vertex shader draws
    // aPosition through uModel, so the shadow frustum has to be built from
    // the same transformed box or it ends up sized/placed for geometry that
    // is not where the actual (scaled, rotated, moved) mesh is.
    const AABB worldBounds = transformAABB(bounds, model);
    const glm::vec3 center = worldBounds.center();
    const glm::vec3 extents = worldBounds.extents();
    const f32 radius = glm::max(glm::length(extents), 1.0f);

    auto drawMesh = [&](PipelineHandle pipeline, TargetHandle target, bool shadowPass, bool clearColor)
    {
        ClearValue clear;
        clear.bits = shadowPass ? ClearDepth : (clearColor ? ClearColor : 0u);
        clear.depth = 1.0f;
        clear.color[0] = clear.color[1] = clear.color[2] = 0.0f;
        // Alpha starts at 0 and the bake fragment shader always writes 1 -
        // it doubles as a coverage mask so save() can tell a real chart
        // texel from empty padding and bleed colour into the padding.
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

    const u32 samples = glm::clamp(settings.sampleCount, 1u, 64u);
    const f32 weight = 1.0f / static_cast<f32>(samples);
    for (u32 sample = 0; sample < samples; ++sample)
    {
        const glm::vec3 sampleDirection =
            jitterSunDirection(direction, settings.sunAngularRadius, sample, samples);
        glm::vec3 up(0.0f, 1.0f, 0.0f);
        if (glm::abs(glm::dot(up, sampleDirection)) > 0.95f)
            up = glm::vec3(1.0f, 0.0f, 0.0f);
        const glm::mat4 shadowView = glm::lookAt(center - sampleDirection * radius * 2.5f, center, up);

        // Fitted to where the geometry actually lands in the sun's own view,
        // not to a box sized by the scene's radius. glm::lookAt looks down -z,
        // so the box's near/far are -maximum.z and -minimum.z; both get a
        // margin so a caster sitting exactly on the plane is not clipped away.
        glm::vec3 viewMinimum, viewMaximum;
        projectBounds(worldBounds, shadowView, viewMinimum, viewMaximum);
        const f32 margin = glm::max(radius * 0.01f, 0.01f);
        const f32 nearPlane = glm::max(-viewMaximum.z - margin, 0.01f);
        const f32 farPlane = -viewMinimum.z + margin;
        const glm::mat4 shadowProjection =
            glm::ortho(viewMinimum.x - margin, viewMaximum.x + margin, viewMinimum.y - margin,
                       viewMaximum.y + margin, nearPlane, farPlane);

        BakeBlock block;
        block.shadowViewProjection = shadowProjection * shadowView;
        block.model = model;
        block.lightDirection = glm::vec4(sampleDirection, 0.0f);
        block.lightColor = glm::vec4(lightColor, 1.0f);
        // Bias, from texels to world units to the ortho's own [0,1] depth,
        // against the frustum this sample actually ended up with. Doing it
        // here rather than asking the caller for a depth fraction is what
        // keeps one setting working at any scene size - and what lets the
        // frustum fit above tighten the depth range without silently changing
        // the meaning of everyone's bias.
        const f32 depthRange = glm::max(farPlane - nearPlane, 0.001f);
        const f32 texelWorldSize =
            (viewMaximum.x - viewMinimum.x + 2.0f * margin) / static_cast<f32>(shadowResolution);
        const f32 worldBias =
            settings.bias > 0.0f ? settings.bias : settings.biasTexels * texelWorldSize;
        block.params = glm::vec4(worldBias / depthRange, settings.ambientGround,
                                 settings.filterRadius, weight);
        block.ambientSky = glm::vec4(settings.ambient, 0.0f);
        // Bases 2 and 3, centred on the texel and widened a little past one
        // texel so the samples reach into the neighbour a border texel is
        // missing coverage from - the reference boosts its own by the same
        // sort of factor for the same reason.
        const f32 texelWidth = 2.0f / static_cast<f32>(mResolution);
        block.jitter =
            glm::vec4((halton(sample, 2) * 2.0f - 1.0f) * texelWidth * 0.7f,
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
        minimum = glm::min(minimum, glm::min(pixels[i], glm::min(pixels[i + 1], pixels[i + 2])));
        maximum = glm::max(maximum, glm::max(pixels[i], glm::max(pixels[i + 1], pixels[i + 2])));
    }
    Log::info("LightmapBakePass: readback range %.4f .. %.4f", static_cast<double>(minimum),
              static_cast<double>(maximum));
    Pixmap image(static_cast<int>(mResolution), static_cast<int>(mResolution), 4);
    for (u32 y = 0; y < mResolution; ++y)
        for (u32 x = 0; x < mResolution; ++x)
        {
            const usize i = (static_cast<usize>(y) * mResolution + x) * 4;
            image.set_pixel(x, y,
                            static_cast<u8>(glm::clamp(pixels[i + 0], 0.0f, 1.0f) * 255.0f),
                            static_cast<u8>(glm::clamp(pixels[i + 1], 0.0f, 1.0f) * 255.0f),
                            static_cast<u8>(glm::clamp(pixels[i + 2], 0.0f, 1.0f) * 255.0f), 255);
        }
    // No flip: AssetManager::loadTexture() never flips a PNG on load either
    // (nothing in this engine does, every other texture already relies on
    // that), so row 0 here has to stay GL's own row 0 - the same convention
    // the bake vertex shader used to write it (aUV2.y=0 -> row 0). Flipping
    // only this file would make it round-trip backwards: uv2 samples land on
    // the wrong side of the atlas, mostly the gaps between charts, which is
    // the clear colour - black.
    const bool saved = image.save(filename.c_str());
    if (!saved)
        Log::error("LightmapBakePass: failed to save '%s'", filename.c_str());
    return saved;
}

void LightmapBakePass::applyToMaterials(std::vector<Material>& materials, const VertexLayout& colorLayout,
                                        TextureHandle lightmapTexture, const std::string& lightmapFile)
{
    // Clamp, not the default Repeat: sampling past a chart at the very edge
    // of the atlas must not wrap into the opposite side's unrelated chart.
    SamplerDesc samplerDesc;
    samplerDesc.filter = Filter::Linear;
    samplerDesc.wrapU = Wrap::Clamp;
    samplerDesc.wrapV = Wrap::Clamp;
    const SamplerHandle sampler = Assets().getSampler(samplerDesc);

    for (Material& material : materials)
    {
        // Lit stays on: dropping it routed the whole material through
        // unlit.frag, which has no idea point/spot lights or their shadows
        // exist - only the sun's own real-time contribution is redundant
        // with the bake (see HAS_LIGHTMAP in lit.frag), so only its flag
        // (ReceiveShadow, which gates the cascade lookup) comes off.
        material.flags &= ~MaterialReceiveShadow;
        material.textures[SlotLightmap].texture = lightmapTexture;
        material.textures[SlotLightmap].sampler = sampler;
        material.textures[SlotLightmap].file = lightmapFile;
        material.textures[SlotLightmap].source = TextureSource::Static;
        MaterialManager::getSingleton().resolvePipeline(material, colorLayout);
    }
}

} // namespace Radion
