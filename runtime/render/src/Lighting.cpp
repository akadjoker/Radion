#include "PCH.h"

#include "Lighting.h"

#include "AssetManager.h"
#include "Decals.h"
#include "DepthPass.h"
#include "Log.h"
#include "Material.h"

#include <cmath>

namespace Radion
{

namespace
{

// Matches the cubemap-face convention CubeFaceUV() in lit.frag decodes a
// world-space direction with. The two tables have to agree, or a point
// light's shadow samples the wrong face.
const Math::Vec3 kFaceDirection[6] = {
    {1.0f, 0.0f, 0.0f},  {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
    {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},  {0.0f, 0.0f, -1.0f},
};
const Math::Vec3 kFaceUp[6] = {
    {0.0f, -1.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
    {0.0f, 0.0f, -1.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, -1.0f, 0.0f},
};

// Above every binding ForwardPass uses, so the cull never leaves a texture on
// a unit a material shader samples.
constexpr u32 kCullDepthUnit = 8;

} // namespace

bool Lighting::setup()
{
    GPU& gpu = GPU::getSingleton();

    BufferDesc entityDesc;
    entityDesc.size = static_cast<u64>(RenderList::MaxLights) * sizeof(RenderLight);
    entityDesc.usage = BufferStorage;
    entityDesc.residency = Residency::Stream;
    entityDesc.stride = sizeof(RenderLight);
    entityDesc.debugName = "lighting.entities";
    mEntityBuffer = gpu.createBuffer(entityDesc);

    // Six matrices for every light, the point-light worst case: it is the
    // simplest bound that never needs to grow.
    BufferDesc matrixDesc;
    matrixDesc.size = static_cast<u64>(RenderList::MaxLights) * 6 * sizeof(Math::Mat4);
    matrixDesc.usage = BufferStorage;
    matrixDesc.residency = Residency::Stream;
    matrixDesc.stride = sizeof(Math::Mat4);
    matrixDesc.debugName = "lighting.matrices";
    mMatrixBuffer = gpu.createBuffer(matrixDesc);

    BufferDesc lightingDesc;
    lightingDesc.size = sizeof(LightingBlock);
    lightingDesc.usage = BufferUniform;
    lightingDesc.residency = Residency::Stream;
    lightingDesc.debugName = "lighting.block";
    mLightingBlock = gpu.createBuffer(lightingDesc);

    BufferDesc cullingDesc;
    cullingDesc.size = sizeof(CullingBlock);
    cullingDesc.usage = BufferUniform;
    cullingDesc.residency = Residency::Stream;
    cullingDesc.debugName = "lighting.culling";
    mCullingBlock = gpu.createBuffer(cullingDesc);

    // Not loaded here: setup() runs during Engine::initialize(), before a demo
    // has added its asset search paths (see DepthPass::pipelineFor() and
    // MaterialManager::resolvePipeline(), which defer for the same reason).
    // ensureCullPipeline() compiles it on first use instead.

    return mEntityBuffer.valid() && mMatrixBuffer.valid() && mLightingBlock.valid() &&
           mCullingBlock.valid() && createAtlasTexture();
}

bool Lighting::ensureCullPipeline()
{
    if (mCullPipeline.valid())
        return true;

    const std::string& cullSource = Assets().loadShader("light_culling.comp");
    if (cullSource.empty())
    {
        Log::error("Lighting: light_culling.comp is missing");
        return false;
    }
    PipelineDesc cullPipeline;
    cullPipeline.cs = {cullSource.c_str(), 0, "light_culling.comp"};
    cullPipeline.debugName = "lighting.cull";
    mCullPipeline = GPU::getSingleton().createPipeline(cullPipeline);
    return mCullPipeline.valid();
}

bool Lighting::createAtlasTexture()
{
    destroyAtlasTexture();
    GPU& gpu = GPU::getSingleton();
    const u32 size = glm::max(mAtlas.settings.size, 1u);

    TextureDesc texture;
    texture.type = TextureType::Tex2D;
    texture.format = Format::Depth24;
    texture.width = size;
    texture.height = size;
    texture.usage = TextureSampled | TextureTarget;
    texture.debugName = "lighting.atlas";
    mAtlasTexture = gpu.createTexture(texture);
    if (!mAtlasTexture.valid())
        return false;

    SamplerDesc sampler;
    sampler.filter = Filter::Linear;
    sampler.wrapU = Wrap::Clamp;
    sampler.wrapV = Wrap::Clamp;
    sampler.compare = true;
    mAtlasSampler = gpu.createSampler(sampler);
    if (!mAtlasSampler.valid())
        return false;

    TargetDesc target;
    target.depth.texture = mAtlasTexture;
    target.debugName = "lighting.atlas.target";
    mAtlasTarget = gpu.createTarget(target);
    if (!mAtlasTarget.valid())
        return false;

    mAtlasSize = size;
    return true;
}

void Lighting::destroyAtlasTexture()
{
    if (!GPU::ready())
        return;
    GPU& gpu = GPU::getSingleton();
    gpu.destroy(mAtlasTarget);
    gpu.destroy(mAtlasSampler);
    gpu.destroy(mAtlasTexture);
    mAtlasTarget = TargetHandle();
    mAtlasSampler = SamplerHandle();
    mAtlasTexture = TextureHandle();
    mAtlasSize = 0;
}

void Lighting::buildEntities(ShadowCasterSource& casters, FrameContext& frame, DepthPass& depthPass,
                             bool renderShadows)
{
    mEntities.clear();
    mMatrices.clear();
    mLightsWithoutTile = 0;

    if (!frame.list)
        return;

    // The sun goes in first, by convention: it is the one directional the
    // Environment path shades, with cascades, so lit.frag's entity loop skips
    // index 0 and takes every directional after it as an unshadowed fill.
    const RenderLight* sun = frame.list->sun();
    u32 shadowedExtras = 0;
    for (const RenderLight& light : frame.list->lights())
    {
        if (light.type != RenderLightType::Directional || &light == sun)
            continue;
        if ((light.flags & RenderLightCastShadow) != 0)
            ++shadowedExtras;
    }
    if (shadowedExtras > 0 && !mExtraSunWarned)
    {
        Log::warning("Lighting: %u extra directional light(s) light the scene but cast no "
                    "shadow; only the elected sun has cascades",
                    shadowedExtras);
        mExtraSunWarned = true;
    }
    if (sun)
        mEntities.push_back(*sun);

    for (const RenderLight& light : frame.list->lights())
        if (light.type == RenderLightType::Directional && &light != sun)
            mEntities.push_back(light);

    for (const RenderLight& light : frame.list->lights())
        if (light.type != RenderLightType::Directional)
            mEntities.push_back(light);

    for (RenderLight& light : mEntities)
    {
        light.matrixIndex = -1;
        light.shadowFade = 0.0f;
        light.shadowAtlasMulAdd = Math::Vec4(0.0f);
    }

    // frame.list never holds more than MaxLights to begin with (see
    // RenderList::addLight), and the sun replaces one directional light
    // rather than adding on top of it, so this can only shrink the count.

    // Preview mode still needs the light entities for shading and tiled
    // culling, but their default matrixIndex=-1/shadowFade=0 already means
    // "unshadowed" to the shaders. Stop before atlas allocation and draws.
    if (!renderShadows)
        return;

    mAtlas.update(frame.cameraPosition, mEntities);

    if (mAtlasSize != mAtlas.settings.size && !createAtlasTexture())
        return;

    GPU& gpu = GPU::getSingleton();
    // No whole-atlas clear here: with only draws this frame reusing shared
    // tiles, that would wipe every light's shadow to redraw one. Each tile
    // below clears and is scissored to just itself instead (see clearRegion),
    // the same shape as the reference's BeginFrame/SetViewport/EndFrame.
    if (!mAtlas.tiles().empty())
    {
        gpu.setTarget(mAtlasTarget);
        gpu.setScissorEnabled(true);
    }

    for (const ShadowTile& tile : mAtlas.tiles())
    {
        RenderLight& light = mEntities[tile.lightIndex];
        light.shadowFade = glm::smoothstep(0.0f, 0.25f, tile.importance);
        const f32 atlasRcp = 1.0f / static_cast<f32>(mAtlasSize);
        light.shadowAtlasMulAdd = Math::Vec4(static_cast<f32>(tile.size) * atlasRcp,
                                            static_cast<f32>(tile.size) * atlasRcp,
                                            static_cast<f32>(tile.x) * atlasRcp,
                                            static_cast<f32>(tile.y) * atlasRcp);

        FrameContext tileFrame = frame;
        tileFrame.target = mAtlasTarget;
        tileFrame.clipPlane = Math::Vec4(0.0f);

        if (light.type == RenderLightType::Point)
        {
            light.matrixIndex = static_cast<s32>(mMatrices.size());
            const Math::Mat4 projection =
                glm::perspective(glm::radians(90.0f), 1.0f, 0.05f, glm::max(0.5f, light.range));
            // Nothing outside the light's own range is ever lit, whichever
            // face is drawing - a sphere reject is cheaper than the 90-degree
            // frustum test and correct for all six faces at once.
            const Sphere cullSphere{Math::Vec3(light.position.x, light.position.y, light.position.z), light.range};
            for (u32 face = 0; face < 6; ++face)
            {
                const Math::Mat4 viewProjection =
                    projection * glm::lookAt(light.position, light.position + kFaceDirection[face],
                                             kFaceUp[face]);
                mMatrices.push_back(viewProjection);
                casters.buildShadowList(mTileList, viewProjection, MaterialCastShadow, &cullSphere);
                tileFrame.list = &mTileList;
                tileFrame.viewProjection = viewProjection;
                tileFrame.viewport = {static_cast<f32>(tile.x + face * tile.size),
                                      static_cast<f32>(tile.y), static_cast<f32>(tile.size),
                                      static_cast<f32>(tile.size)};

                const Rect faceRect{static_cast<s32>(tile.x + face * tile.size),
                                    static_cast<s32>(tile.y), static_cast<s32>(tile.size),
                                    static_cast<s32>(tile.size)};
                gpu.setScissor(faceRect);
                ClearValue faceClear;
                faceClear.bits = ClearDepth;
                gpu.clearRegion(mAtlasTarget, faceRect, faceClear);
                // A point light's tile never gets polygon offset: depth_point.frag
                // writes gl_FragDepth by hand and bakes pointBias into that value
                // itself, so glPolygonOffset here would have no effect on it.
                depthPass.executePoint(tileFrame, light.position, light.range,
                                       mAtlas.settings.pointBias);
            }
        }
        else
        {
            light.matrixIndex = static_cast<s32>(mMatrices.size());
            const Math::Vec3 up = glm::abs(light.direction.y) > 0.99f ? Math::Vec3(0.0f, 0.0f, 1.0f)
                                                                     : Math::Vec3(0.0f, 1.0f, 0.0f);
            // Recovered from coneAngleCos: RenderLight keeps the cosine the
            // shading needs, not the degrees a shadow FOV is built from.
            const f32 outerDegrees =
                glm::degrees(std::acos(glm::clamp(light.coneAngleCos, -1.0f, 1.0f)));
            const f32 fov = light.type == RenderLightType::Rectangle
                               ? glm::radians(120.0f)
                               : glm::radians(glm::clamp(outerDegrees * 2.0f, 5.0f, 170.0f));
            const Math::Mat4 viewProjection =
                glm::perspective(fov, 1.0f, 0.05f, glm::max(0.5f, light.range)) *
                glm::lookAt(light.position, light.position + light.direction, up);
            mMatrices.push_back(viewProjection);
            casters.buildShadowList(mTileList, viewProjection, MaterialCastShadow);
            tileFrame.list = &mTileList;
            tileFrame.viewProjection = viewProjection;
            tileFrame.viewport = {static_cast<f32>(tile.x), static_cast<f32>(tile.y),
                                  static_cast<f32>(tile.size), static_cast<f32>(tile.size)};

            const Rect tileRect{static_cast<s32>(tile.x), static_cast<s32>(tile.y),
                                static_cast<s32>(tile.size), static_cast<s32>(tile.size)};
            gpu.setScissor(tileRect);
            ClearValue tileClear;
            tileClear.bits = ClearDepth;
            gpu.clearRegion(mAtlasTarget, tileRect, tileClear);
            depthPass.executeBiased(tileFrame, mAtlas.settings.biasSlope,
                                    mAtlas.settings.biasConstant);
        }
    }
    if (!mAtlas.tiles().empty())
        gpu.setScissorEnabled(false);

    for (usize i = 0; i < mEntities.size(); ++i)
    {
        const RenderLight& light = mEntities[i];
        const bool wantsShadow = (light.flags & RenderLightCastShadow) != 0 &&
                                 light.type != RenderLightType::Directional && light.range > 0.0f;
        if (wantsShadow && light.matrixIndex < 0)
            ++mLightsWithoutTile;
    }
}

void Lighting::submitDecals(const DecalSystem& decals)
{
    mDroppedDecals = 0;
    const u32 total = decals.count();
    if (total == 0 || !mEntityBuffer.valid())
        return;

    const usize entityStart = mEntities.size();
    const usize matrixStart = mMatrices.size();

    for (u32 i = 0; i < total; ++i)
    {
        const DecalSystem::Decal& decal = decals.decal(i);
        if (!decal.enabled || decal.opacity <= 0.0f)
            continue;
        if (mEntities.size() >= RenderList::MaxLights)
        {
            mDroppedDecals = total - i;
            break;
        }

        // Reuses the light fields for decal purposes, the same way the
        // ShaderEntity they land in does: coneAngleCos becomes the slope-fade
        // exponent, coneAngleScale the opacity, rectangleWidth the normal
        // strength. direction is the box's own +Z in world space - not the
        // projection's -Z - because that is the normal a surface facing the
        // decal should have, and the slope fade compares against it directly.
        RenderLight entity;
        entity.type = RenderLightType::Decal;
        entity.flags = decal.baseColorOnlyAlpha ? static_cast<u32>(RenderDecalBaseColorOnlyAlpha) : 0u;
        entity.matrixIndex = static_cast<s32>(mMatrices.size());
        entity.position = decal.position;
        entity.range = DecalSystem::boundingRadius(decal);
        entity.direction = glm::normalize(decal.rotation * Math::Vec3(0.0f, 0.0f, 1.0f));
        entity.coneAngleCos =
            decals.slopePowerOverride >= 0.0f ? decals.slopePowerOverride : decal.slopePower;
        entity.coneAngleScale = decal.opacity * decals.globalOpacity;
        entity.color = decal.color;
        entity.rectangleWidth = decal.normalStrength * decals.normalStrengthScale;

        mMatrices.push_back(DecalSystem::makeProjection(decal));
        mEntities.push_back(entity);
    }

    GPU& gpu = GPU::getSingleton();
    if (mEntities.size() > entityStart)
        gpu.updateBuffer(mEntityBuffer, entityStart * sizeof(RenderLight),
                         (mEntities.size() - entityStart) * sizeof(RenderLight),
                         mEntities.data() + entityStart);
    if (mMatrices.size() > matrixStart)
        gpu.updateBuffer(mMatrixBuffer, matrixStart * sizeof(Math::Mat4),
                         (mMatrices.size() - matrixStart) * sizeof(Math::Mat4),
                         mMatrices.data() + matrixStart);
}

void Lighting::prepare(ShadowCasterSource& casters, FrameContext& frame, DepthPass& depthPass,
                       bool renderShadows)
{
    buildEntities(casters, frame, depthPass, renderShadows);

    GPU& gpu = GPU::getSingleton();
    if (!mEntities.empty())
        gpu.updateBuffer(mEntityBuffer, 0, mEntities.size() * sizeof(RenderLight),
                         mEntities.data());
    if (!mMatrices.empty())
        gpu.updateBuffer(mMatrixBuffer, 0, mMatrices.size() * sizeof(Math::Mat4),
                         mMatrices.data());

    frame.entityBuffer = mEntityBuffer;
    frame.entityMatrixBuffer = mMatrixBuffer;
    frame.lightTileBuffer = mTileBuffer;
    frame.lightingBlock = mLightingBlock;
    frame.shadowAtlas = mAtlasTexture;
    frame.shadowAtlasSampler = mAtlasSampler;
}

void Lighting::cull(FrameContext& frame, TextureHandle sceneDepth, u32 screenWidth,
                    u32 screenHeight)
{
    GPU& gpu = GPU::getSingleton();

    mTilesX = (screenWidth + 31) / 32;
    mTilesY = (screenHeight + 31) / 32;
    const u32 tileSlots = glm::max(1u, mTilesX * mTilesY) * 8;
    const u64 tileBytes = static_cast<u64>(tileSlots) * sizeof(u32);
    if (tileSlots > mTileBufferCapacity)
    {
        gpu.destroy(mTileBuffer);
        BufferDesc desc;
        desc.size = tileBytes;
        desc.usage = BufferStorage;
        desc.residency = Residency::Stream;
        desc.stride = sizeof(u32);
        desc.debugName = "lighting.tiles";
        mTileBuffer = gpu.createBuffer(desc);
        mTileBufferCapacity = mTileBuffer.valid() ? tileSlots : 0;
        frame.lightTileBuffer = mTileBuffer;
    }

    // Bound even when tiled is off: lit.frag declares this SSBO unconditionally,
    // and an unbound storage binding a shader statically references makes
    // every following draw fail.
    //
    // `tiled` is the setting; `tiledActive` is whether the dispatch below
    // actually ran this frame. They used to be the same value everywhere -
    // block.counts.y always published `tiled` - so a missing compute shader,
    // an invalid depth, an empty entity list or a failed buffer allocation
    // left the fragment shader reading the tile path anyway, against tile
    // masks that were never written this frame.
    const bool tiledActive = tiled && mTileBuffer.valid() && sceneDepth.valid() &&
                             screenWidth > 0 && screenHeight > 0 && !mEntities.empty() &&
                             ensureCullPipeline();
    if (tiledActive)
    {
        gpu.bindStorage(0, mEntityBuffer);
        gpu.bindStorage(1, mTileBuffer);
        // Unit 8, never 0: unit 0 is where a Lit material samples its albedo,
        // and leaving the scene depth there fails every draw whose shader
        // statically samples it.
        gpu.bindTexture(kCullDepthUnit, sceneDepth);

        CullingBlock block;
        block.inverseProjection = glm::inverse(frame.projection);
        block.view = frame.view;
        block.screenSize = Math::Vec4(static_cast<f32>(screenWidth), static_cast<f32>(screenHeight),
                                     0.0f, 0.0f);
        block.tileCountEtc = Math::Vec4(static_cast<f32>(mTilesX), static_cast<f32>(mTilesY),
                                       static_cast<f32>(mEntities.size()), use25D ? 1.0f : 0.0f);
        gpu.updateBuffer(mCullingBlock, 0, sizeof(block), &block);
        gpu.bindUniform(0, mCullingBlock);

        gpu.setPipeline(mCullPipeline);
        gpu.dispatch(mTilesX, mTilesY, 1);
        gpu.barrier(BarrierStorage);
    }

    LightingBlock block;
    block.counts = Math::Vec4(static_cast<f32>(mEntities.size()), tiledActive ? 1.0f : 0.0f,
                             debugTiles ? 1.0f : 0.0f, static_cast<f32>(debugMode));
    block.tileGrid = Math::Vec4(static_cast<f32>(mTilesX), static_cast<f32>(mTilesY),
                               decalsEnabled ? 1.0f : 0.0f, 0.0f);
    gpu.updateBuffer(mLightingBlock, 0, sizeof(block), &block);
}

void Lighting::shutdown()
{
    if (GPU::ready())
    {
        GPU& gpu = GPU::getSingleton();
        gpu.destroy(mEntityBuffer);
        gpu.destroy(mMatrixBuffer);
        gpu.destroy(mTileBuffer);
        gpu.destroy(mLightingBlock);
        gpu.destroy(mCullingBlock);
        gpu.destroy(mCullPipeline);
    }
    destroyAtlasTexture();
    mEntityBuffer = BufferHandle();
    mMatrixBuffer = BufferHandle();
    mTileBuffer = BufferHandle();
    mLightingBlock = BufferHandle();
    mCullingBlock = BufferHandle();
    mCullPipeline = PipelineHandle();
    mTileBufferCapacity = 0;
    mEntities.clear();
    mMatrices.clear();
}

} // namespace Radion
