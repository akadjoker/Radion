#ifndef RADION_LENS_FLARE_PASS_H
#define RADION_LENS_FLARE_PASS_H

#include "FrameContext.h"

namespace Radion
{

class PostProcessStack;

class LensFlarePass final
{
public:
    bool setup();
    void execute(const FrameContext& frame, PostProcessStack& post);
    void shutdown();

    bool enabled = true;

    // Texel radius of the occlusion sampling grid around the sun's screen position.
    f32 occlusionRadius = 6.0f;

    // Distance along the sun direction for projecting its screen position, in world units; must pass the far plane for a source at infinity.
    f32 sunDistance = 100000.0f;

    // Paints raw visibility (white = unobstructed) instead of the texture, to check depth occlusion.
    bool debugOcclusion = false;

    static constexpr u32 kElementCount = 7;

private:
    bool ensurePipeline();
    bool ensureTarget(PostProcessStack& post);

    PipelineHandle mPipeline;
    BufferHandle mBlock;
    SamplerHandle mDepthSampler;
    SamplerHandle mFlareSampler;
    bool mPipelineReady = false;
    bool mPipelineFailed = false;

    TextureHandle mElementTextures[kElementCount];
 
    TargetHandle mColorOnlyTarget;
    TextureHandle mAliasedColor;
};

} // namespace Radion

#endif // RADION_LENS_FLARE_PASS_H
