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
    Math::mat4 shadowMatrix[MaxShadowCascades];
    Math::vec4 splits = Math::vec4(0.0f);
    Math::vec4 shadowBias = Math::vec4(0.0f);
    Math::vec4 shadowNormalBias = Math::vec4(0.0f);
    Math::vec4 rangeBegin = Math::vec4(0.0f);
    Math::vec4 uvScale[MaxShadowCascades]{};
    Math::vec4 directionAndCount = Math::vec4(0.0f, -1.0f, 0.0f, 0.0f);
    // x = 1/atlas size, y = soft shadow scale, z = tan(angular diameter),
    // w = blend splits.
    Math::vec4 sampling = Math::vec4(0.0f);
    // x = soft samples, y = penumbra samples, z = opacity, w = frame count
    // for the rotating disk.
    Math::vec4 sampling2 = Math::vec4(0.0f);
    // x = fade from, y = fade to.
    Math::vec4 sampling3 = Math::vec4(0.0f);
    Math::vec4 softKernel[MaxShadowKernel]{};
    Math::vec4 penumbraKernel[MaxShadowKernel]{};
};

// Why a frame carries no directional shadow; each is a normal state, not an error.
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

    // Stats of the LAST cascade drawn (mShadowList is rebuilt per cascade); enough to see whether the cull sphere rejects anything.
    const RenderListStats& lastCascadeStats() const
    {
        return mShadowList.stats();
    }

    // World half-width the cascade covered; resolution divided by this decides aliasing.
    f32 halfExtent(u32 cascade) const
    {
        return cascade < MaxShadowCascades ? mHalfExtents[cascade] : 0.0f;
    }

    f32 split(u32 cascade) const
    {
        return cascade < MaxShadowCascades ? mSplits[cascade] : 0.0f;
    }

    // Read-only directional depth atlas, for the debug overlay.
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
    // Records the reason and logs it once, when it changes.
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
    Math::vec4 mSoftKernel[MaxShadowKernel]{};
    Math::vec4 mPenumbraKernel[MaxShadowKernel]{};
    f32 mHalfExtents[MaxShadowCascades]{};
    f32 mSplits[MaxShadowCascades]{};
    u32 mResolution = 0;
    ShadowSkipReason mSkipReason = ShadowSkipReason::None;
    u32 mFrameIndex = 0;
    bool mAtlasNeedsClear = true;

    // Rebuilt per cascade; cascades draw one at a time, so one reused list suffices.
    RenderList mShadowList;
};

} // namespace Radion

#endif // RADION_SHADOW_PASS_H
