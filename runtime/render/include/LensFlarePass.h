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

    // Radius, in texels, of the occlusion sampling grid the vertex shader
    // walks around the sun's screen position.
    f32 occlusionRadius = 6.0f;

    // How far along the sun direction its screen position is projected from
    // the camera, in world units - has to land past the far plane for a
    // source meant to sit at infinity. The reference used sceneRadius*8; a
    // demo without that number can just pass a large constant.
    f32 sunDistance = 100000.0f;

    // Paints each element's raw visibility (white = unobstructed, black =
    // occluded) instead of its texture - to check the depth occlusion itself
    // without a small, mostly-transparent sprite in the way of seeing it.
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
