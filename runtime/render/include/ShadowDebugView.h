#ifndef RADION_SHADOW_DEBUG_VIEW_H
#define RADION_SHADOW_DEBUG_VIEW_H

#include "FrameContext.h"
#include "Shadows.h"

namespace Radion
{

// On-screen overlay for the two textures the shadow system produces: the
// cascade depth array and the shadow atlas. Both carry a comparison sampler
// for shading, which reads back a 0/1 test result rather than depth - this
// draws them through its own plain sampler instead, straight into the
// backbuffer, so the raw depth is what shows up.
class ShadowDebugView
{
public:
    bool setup();
    void shutdown();

    // One square thumbnail per cascade, bottom-left corner, growing rightward.
    void drawCascades(TextureHandle cascades, u32 count, u32 windowWidth, u32 windowHeight);

    // The whole atlas as one square, bottom-right corner.
    void drawAtlas(TextureHandle atlas, u32 windowWidth, u32 windowHeight);
    void drawTexture(TextureHandle texture, bool isArray, u32 layer, TargetHandle target,
                     u32 width, u32 height,
                     const glm::vec4& sourceRect = glm::vec4(1.0f, 1.0f, 0.0f, 0.0f));
    void drawCubemap(TextureHandle texture, u32 face, u32 mip, TargetHandle target,
                     u32 width, u32 height);

private:
    void blit(TextureHandle texture, bool isArray, u32 layer, TargetHandle target,
              const Viewport& viewport,
              const glm::vec4& sourceRect = glm::vec4(1.0f, 1.0f, 0.0f, 0.0f));
    void blitCubemap(TextureHandle texture, u32 face, u32 mip, TargetHandle target,
                     const Viewport& viewport);

    PipelineHandle mPipeline;
    BufferHandle mUniform;
    SamplerHandle mSampler;
};

} // namespace Radion

#endif // RADION_SHADOW_DEBUG_VIEW_H
