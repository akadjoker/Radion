#include "PCH.h"

#include "ShadowPass.h"

#include "DepthPass.h"
#include "GPUProfiler.h"
#include "Material.h"
#include "Profiler.h"

namespace Radion
{

bool ShadowPass::setup()
{
    BufferDesc block;
    block.size = sizeof(DirectionalShadowBlock);
    block.usage = BufferUniform;
    block.residency = Residency::Stream;
    block.debugName = "shadow.directional.block";
    mBlock = GPU::getSingleton().createBuffer(block);
    return mBlock.valid() && createResources();
}

bool ShadowPass::createResources()
{
    destroyResources();
    GPU& gpu = GPU::getSingleton();
    const u32 cascadeResolution = glm::max(mCalculator.settings.resolution, 1u);
    const u32 cascadeCount = glm::clamp(mCalculator.settings.count, 1u, MaxShadowCascades);
    // PSSM4 is four equal quadrants. Keeping the public resolution as the
    // resolution of one split preserves the old array quality: 2048 in the
    // editor becomes the same 4096 atlas layout described in the Godot code.
    const u32 resolution = cascadeCount >= 3 ? cascadeResolution * 2u : cascadeResolution;
    TextureDesc texture;
    texture.type = TextureType::Tex2D;
    texture.format = Format::Depth24;
    texture.width = resolution;
    texture.height = resolution;
    texture.usage = TextureSampled | TextureTarget;
    texture.debugName = "shadow.directional.atlas";
    mTexture = gpu.createTexture(texture);
    if (!mTexture.valid())
        return false;

    SamplerDesc sampler;
    sampler.filter = Filter::Linear;
    sampler.wrapU = Wrap::Clamp;
    sampler.wrapV = Wrap::Clamp;
    sampler.compare = true;
    mSampler = gpu.createSampler(sampler);
    if (!mSampler.valid())
        return false;

    SamplerDesc raw;
    raw.filter = Filter::Linear;
    raw.wrapU = Wrap::Clamp;
    raw.wrapV = Wrap::Clamp;
    raw.compare = false;
    mRawSampler = gpu.createSampler(raw);
    if (!mRawSampler.valid())
        return false;

    TargetDesc target;
    target.depth.texture = mTexture;
    target.debugName = "shadow.directional.atlas";
    mTarget = gpu.createTarget(target);
    if (!mTarget.valid())
        return false;
    mResolution = resolution;
    mAtlasNeedsClear = true;
    for (bool& cached : mCascadeCached)
        cached = false;
    return true;
}

void ShadowPass::destroyResources()
{
    if (!GPU::ready())
        return;
    GPU& gpu = GPU::getSingleton();
    gpu.destroy(mTarget);
    mTarget = TargetHandle();
    gpu.destroy(mSampler);
    gpu.destroy(mRawSampler);
    gpu.destroy(mTexture);
    mSampler = SamplerHandle();
    mRawSampler = SamplerHandle();
    mTexture = TextureHandle();
    mResolution = 0;
}

const char* ShadowPass::skipReasonText(ShadowSkipReason reason)
{
    switch (reason)
    {
    case ShadowSkipReason::None:
        return "rendering";
    case ShadowSkipReason::Disabled:
        return "cascade shadows are switched off";
    case ShadowSkipReason::NoSun:
        return "the render list carries no elected sun";
    case ShadowSkipReason::SunCastsNoShadow:
        return "the elected sun has shadow casting off";
    case ShadowSkipReason::ResourcesFailed:
        return "the cascade texture or targets could not be created";
    case ShadowSkipReason::CascadeFitFailed:
        return "the cascade fit rejected the camera frustum";
    }
    return "unknown";
}

void ShadowPass::reportSkip(ShadowSkipReason reason)
{
    if (reason == mSkipReason)
        return;
    mSkipReason = reason;
    if (reason == ShadowSkipReason::None)
        Log::info("ShadowPass: directional shadows rendering");
    else
        Log::warning("ShadowPass: no directional shadow - %s", skipReasonText(reason));
}

void ShadowPass::execute(ShadowCasterSource& casters, FrameContext& frame, DepthPass& depthPass)
{
    RADION_PROFILE_SCOPE("Directional shadows");

    // frame.directionalShadow* are left at their default (invalid) values
    // until every early-return below has had its chance to fail: publishing
    // mTexture/mSampler up front used to mean a resolution change that made
    // createResources() destroy and then fail to rebuild them left the frame
    // pointing at handles that were already gone. ForwardPass only binds the
    // shadow map when both the texture and the block are valid, so leaving
    // them unset here is exactly "render this frame without shadows".

    DirectionalShadowBlock block;
    if (!mCalculator.settings.enabled)
    {
        if (mResolution != 0)
            destroyResources();
        reportSkip(ShadowSkipReason::Disabled);
        GPU::getSingleton().updateBuffer(mBlock, 0, sizeof(block), &block);
        return;
    }

    const RenderLight* sun = frame.list ? frame.list->sun() : nullptr;
    const bool sunFound = sun != nullptr;
    if (sun && (sun->flags & RenderLightCastShadow) == 0)
        sun = nullptr;

    if (!sun)
    {
        reportSkip(sunFound ? ShadowSkipReason::SunCastsNoShadow : ShadowSkipReason::NoSun);
        GPU::getSingleton().updateBuffer(mBlock, 0, sizeof(block), &block);
        return;
    }
    // Compared against the normalised value, not the raw setting: mResolution
    // is always max(setting, 1) once createResources() has run, so comparing
    // it to a raw 0 setting never agrees and recreated the cascades every
    // single frame.
    const u32 cascadeResolution = glm::max(mCalculator.settings.resolution, 1u);
    const u32 cascadeCount = glm::clamp(mCalculator.settings.count, 1u, MaxShadowCascades);
    const u32 wantedResolution = cascadeCount >= 3 ? cascadeResolution * 2u : cascadeResolution;
    if (mResolution != wantedResolution && !createResources())
    {
        // createResources() destroys the old cascades before building the
        // new ones, so a failure here has nothing valid left to fall back
        // to - not even what existed before this call.
        reportSkip(ShadowSkipReason::ResourcesFailed);
        GPU::getSingleton().updateBuffer(mBlock, 0, sizeof(block), &block);
        return;
    }

    ShadowCamera camera;
    camera.view = frame.view;
    camera.fieldOfView = frame.fieldOfView;
    camera.aspect = frame.aspect;
    camera.nearPlane = frame.nearPlane;
    CascadeShadowData cascades;
    if (!mCalculator.update(camera, sun->direction, cascades))
    {
        reportSkip(ShadowSkipReason::CascadeFitFailed);
        GPU::getSingleton().updateBuffer(mBlock, 0, sizeof(block), &block);
        return;
    }
    reportSkip(ShadowSkipReason::None);

    const Math::Vec3 sunDirection = glm::normalize(sun->direction);
    if (glm::dot(sunDirection, mCachedSunDirection) < 0.9999f || cascades.count != mCachedCount)
        for (bool& cached : mCascadeCached)
            cached = false;
    mCachedSunDirection = sunDirection;
    mCachedCount = cascades.count;

    bool redraw[MaxShadowCascades] = {true, true, true, true};
    for (u32 i = 0; i < cascades.count; ++i)
    {
        redraw[i] = i < 2 || !mCalculator.settings.alternateFarCascades || !mCascadeCached[i] ||
                    ((mFrameIndex + i) & 1u) == 0u;
        if (!redraw[i])
        {
            cascades.viewProjection[i] = mCached.viewProjection[i];
            cascades.cullViewProjection[i] = mCached.cullViewProjection[i];
            cascades.shadowMatrix[i] = mCached.shadowMatrix[i];
            cascades.halfExtents[i] = mCached.halfExtents[i];
            cascades.shadowBias[i] = mCached.shadowBias[i];
            cascades.shadowNormalBias[i] = mCached.shadowNormalBias[i];
            cascades.rangeBegin[i] = mCached.rangeBegin[i];
            cascades.uvScale[i] = mCached.uvScale[i];
            cascades.texelSize[i] = mCached.texelSize[i];
        }
    }
    ++mFrameIndex;

    rebuildKernels();

    for (u32 i = 0; i < MaxShadowCascades; ++i)
    {
        block.shadowMatrix[i] = cascades.shadowMatrix[i];
        block.splits[i] = i < cascades.count ? cascades.splits[i] : 0.0f;
        block.shadowBias[i] = cascades.shadowBias[i];
        block.shadowNormalBias[i] = cascades.shadowNormalBias[i];
        block.rangeBegin[i] = cascades.rangeBegin[i];
        block.uvScale[i] = Math::Vec4(cascades.uvScale[i], 0.0f, 0.0f);
        mHalfExtents[i] = cascades.halfExtents[i];
        mSplits[i] = cascades.splits[i];
    }
    for (u32 i = 0; i < MaxShadowKernel; ++i)
    {
        block.softKernel[i] = mSoftKernel[i];
        block.penumbraKernel[i] = mPenumbraKernel[i];
    }
    block.directionAndCount =
        Math::Vec4(sunDirection, static_cast<f32>(cascades.count));
    const f32 tanAngle = mCalculator.settings.angularDiameter > 0.0f
                             ? std::tan(glm::radians(mCalculator.settings.angularDiameter))
                             : 0.0f;
    block.sampling = Math::Vec4(1.0f / static_cast<f32>(mResolution), cascades.softShadowScale,
                               tanAngle, mCalculator.settings.blend ? 1.0f : 0.0f);
    block.sampling2 =
        Math::Vec4(static_cast<f32>(mSoftSamples), static_cast<f32>(mPenumbraSamples),
                  glm::clamp(mCalculator.settings.opacity, 0.0f, 1.0f),
                  frame.temporalAA ? static_cast<f32>(mFrameIndex & 255u) : 0.0f);
    block.sampling3 = Math::Vec4(cascades.fadeFrom, cascades.fadeTo, 0.0f, 0.0f);
    GPU& gpu = GPU::getSingleton();
    gpu.updateBuffer(mBlock, 0, sizeof(block), &block);

    // Resources are confirmed valid and the block just written matches them -
    // only now is it correct to point the frame at them.
    frame.directionalShadow = mTexture;
    frame.directionalShadowSampler = mSampler;
    frame.directionalShadowRawSampler = mRawSampler;
    frame.directionalShadowBlock = mBlock;

    FrameContext shadowFrame = frame;
    shadowFrame.clipPlane = Math::Vec4(0.0f);
    ClearValue clear;
    clear.bits = ClearDepth;
    const bool fullClear = mAtlasNeedsClear;
    if (fullClear)
    {
        gpu.setTarget(mTarget, clear);
        mAtlasNeedsClear = false;
    }
    else
        gpu.setTarget(mTarget);
    static constexpr const char* cascadeScopes[MaxShadowCascades] = {
        "Shadow cascade 0", "Shadow cascade 1", "Shadow cascade 2", "Shadow cascade 3"};
    for (u32 cascade = 0; cascade < cascades.count; ++cascade)
    {
        if (!redraw[cascade])
            continue;
        GPUProfileScope cascadeScope(cascadeScopes[cascade]);
        const DirectionalShadowRegion region =
            directionalShadowRegion(mResolution, cascades.count, cascade);
        if (!fullClear)
        {
            GPUProfileScope clearScope("Shadow clear");
            gpu.clearRegion(mTarget,
                            {static_cast<s32>(region.x), static_cast<s32>(region.y),
                             static_cast<s32>(region.width), static_cast<s32>(region.height)},
                            clear);
        }
        const f32 minCasterExtent = 2.0f * cascades.texelSize[cascade];
        {
        RADION_PROFILE_SCOPE("Shadow caster list");
        casters.buildShadowList(mShadowList, cascades.cullViewProjection[cascade],
                                MaterialCastShadow, nullptr, MeshHandle(), 0, false,
                                &cascades.casterPlanes[cascade], minCasterExtent);
        }
        shadowFrame.list = &mShadowList;
        shadowFrame.viewProjection = cascades.viewProjection[cascade];
        shadowFrame.target = mTarget;
        shadowFrame.viewport = {static_cast<f32>(region.x), static_cast<f32>(region.y),
                                static_cast<f32>(region.width), static_cast<f32>(region.height),
                                0.0f, 1.0f};
        GPUProfileScope drawScope("Shadow draw");
        depthPass.executeShadow(shadowFrame, mCalculator.settings.depthBiasSlope,
                                mCalculator.settings.depthBiasConstant,
                                mCalculator.settings.cullFront);
        mCached.viewProjection[cascade] = cascades.viewProjection[cascade];
        mCached.cullViewProjection[cascade] = cascades.cullViewProjection[cascade];
        mCached.shadowMatrix[cascade] = cascades.shadowMatrix[cascade];
        mCached.halfExtents[cascade] = cascades.halfExtents[cascade];
        mCached.shadowBias[cascade] = cascades.shadowBias[cascade];
        mCached.shadowNormalBias[cascade] = cascades.shadowNormalBias[cascade];
        mCached.rangeBegin[cascade] = cascades.rangeBegin[cascade];
        mCached.uvScale[cascade] = cascades.uvScale[cascade];
        mCached.texelSize[cascade] = cascades.texelSize[cascade];
        mCascadeCached[cascade] = true;
    }
}

void ShadowPass::rebuildKernels()
{
    if (mKernelQuality == mCalculator.settings.quality)
        return;
    mKernelQuality = mCalculator.settings.quality;

    switch (glm::min(mCalculator.settings.quality, 5u))
    {
    case 0: mPenumbraSamples = 4;  mSoftSamples = 0;  break;
    case 1: mPenumbraSamples = 4;  mSoftSamples = 1;  break;
    case 2: mPenumbraSamples = 8;  mSoftSamples = 4;  break;
    case 3: mPenumbraSamples = 12; mSoftSamples = 8;  break;
    case 4: mPenumbraSamples = 24; mSoftSamples = 16; break;
    default: mPenumbraSamples = 32; mSoftSamples = 32; break;
    }

    auto buildVogel = [](Math::Vec4* kernel, u32 count)
    {
        for (u32 i = 0; i < MaxShadowKernel; ++i)
            kernel[i] = Math::Vec4(0.0f);
        const f32 goldenAngle = 2.4f;
        for (u32 i = 0; i < count; ++i)
        {
            const f32 r = std::sqrt(static_cast<f32>(i) + 0.5f) /
                          std::sqrt(static_cast<f32>(count));
            const f32 theta = static_cast<f32>(i) * goldenAngle;
            kernel[i] = Math::Vec4(std::cos(theta) * r, std::sin(theta) * r, 0.0f, 0.0f);
        }
    };
    buildVogel(mPenumbraKernel, mPenumbraSamples);
    buildVogel(mSoftKernel, mSoftSamples);
}

void ShadowPass::shutdown()
{
    destroyResources();
    if (GPU::ready())
        GPU::getSingleton().destroy(mBlock);
    mBlock = BufferHandle();
}

} // namespace Radion
