#ifndef RADION_SHADOW_PASS_H
#define RADION_SHADOW_PASS_H

#include "FrameContext.h"
#include "RenderList.h"
#include "Shadows.h"

namespace Radion
{

class DepthPass;

static constexpr u32 MaxShadowKernel = 32;

struct alignas(16) DirectionalShadowBlock
{
    // World to atlas UV per split, rect and NDC-to-UV bias folded in.
    glm::mat4 shadowMatrix[MaxShadowCascades];
    glm::vec4 splits = glm::vec4(0.0f);
    glm::vec4 shadowBias = glm::vec4(0.0f);
    glm::vec4 shadowNormalBias = glm::vec4(0.0f);
    glm::vec4 rangeBegin = glm::vec4(0.0f);
    glm::vec4 uvScale[MaxShadowCascades]{};
    glm::vec4 directionAndCount = glm::vec4(0.0f, -1.0f, 0.0f, 0.0f);
    // x = 1/atlas size, y = soft shadow scale, z = tan(angular diameter),
    // w = blend splits.
    glm::vec4 sampling = glm::vec4(0.0f);
    // x = soft samples, y = penumbra samples, z = opacity, w = frame count
    // for the rotating disk.
    glm::vec4 sampling2 = glm::vec4(0.0f);
    // x = fade from, y = fade to.
    glm::vec4 sampling3 = glm::vec4(0.0f);
    glm::vec4 softKernel[MaxShadowKernel]{};
    glm::vec4 penumbraKernel[MaxShadowKernel]{};
};

// Why a frame carries no directional shadow. Every one of these is a normal
// state, not an error, which is exactly why they used to be indistinguishable
// from each other and from a shadow that rendered but did not show.
enum class ShadowSkipReason : u32
{
    None,
    Disabled,
    NoSun,
    SunCastsNoShadow,
    ResourcesFailed,
    CascadeFitFailed,
};

class ShadowPass final
{
public:
    bool setup();
    void execute(ShadowCasterSource& casters, FrameContext& frame, DepthPass& depthPass);
    void shutdown();

    CascadeShadowSettings& cascadeSettings()
    {
        return mCalculator.settings;
    }

    // mShadowList is rebuilt (and its stats reset) for every cascade in turn,
    // so this is only ever the LAST cascade drawn this frame, not a sum
    // across all of them - still enough to see whether the cull sphere is
    // rejecting anything at all, which is what a debug panel needs.
    const RenderListStats& lastCascadeStats() const
    {
        return mShadowList.stats();
    }

    // World half-width the cascade covered this frame. Resolution divided by
    // this is what actually decides the aliasing, which the resolution alone
    // never says.
    f32 halfExtent(u32 cascade) const
    {
        return cascade < MaxShadowCascades ? mHalfExtents[cascade] : 0.0f;
    }

    f32 split(u32 cascade) const
    {
        return cascade < MaxShadowCascades ? mSplits[cascade] : 0.0f;
    }

    // The directional depth atlas, read-only. For the debug overlay - shading
    // reaches it through FrameContext::directionalShadow instead.
    TextureHandle texture() const
    {
        return mTexture;
    }

    // Why the last execute() produced no cascades. None means it rendered.
    ShadowSkipReason skipReason() const
    {
        return mSkipReason;
    }

    static const char* skipReasonText(ShadowSkipReason reason);

private:
    bool createResources();
    void destroyResources();
    void rebuildKernels();
    // Records the reason and logs it once, on the frame it changes - a
    // per-frame message for a state that persists is noise nobody reads.
    void reportSkip(ShadowSkipReason reason);

    CascadeShadowCalculator mCalculator;
    TextureHandle mTexture;
    SamplerHandle mSampler;
    SamplerHandle mRawSampler;
    TargetHandle mTarget;
    BufferHandle mBlock;
    u32 mKernelQuality = ~0u;
    u32 mSoftSamples = 4;
    u32 mPenumbraSamples = 8;
    glm::vec4 mSoftKernel[MaxShadowKernel]{};
    glm::vec4 mPenumbraKernel[MaxShadowKernel]{};
    f32 mHalfExtents[MaxShadowCascades]{};
    f32 mSplits[MaxShadowCascades]{};
    u32 mResolution = 0;
    ShadowSkipReason mSkipReason = ShadowSkipReason::None;
    CascadeShadowData mCached;
    bool mCascadeCached[MaxShadowCascades]{};
    glm::vec3 mCachedSunDirection = glm::vec3(0.0f);
    u32 mCachedCount = 0;
    u32 mFrameIndex = 0;
    bool mAtlasNeedsClear = true;

    // Rebuilt for each cascade in turn - they are drawn one at a time, never
    // concurrently, so one reused list costs less than four and behaves the
    // same: nothing reads cascade N's list once cascade N+1 starts building.
    RenderList mShadowList;
};

} // namespace Radion

#endif // RADION_SHADOW_PASS_H
