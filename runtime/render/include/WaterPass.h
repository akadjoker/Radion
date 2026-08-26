#ifndef RADION_WATER_PASS_H
#define RADION_WATER_PASS_H

#include "OffscreenTarget.h"
#include "RenderTechnique.h"

namespace Radion
{

// The scene copy the water sampled for refraction this frame, published for
// a debug view the way OceanRender publishes its own.
constexpr const char* kWaterRefractionDebugTargetName = "water.debug.scene_copy";

// What the surface needs from the frame and no other block carries: near/far
// to linearize the refraction depth it samples, time to scroll its noise.
// Bound at BindingWater.
struct WaterBlock
{
    glm::vec4 timeNearFar; // x = time, y = near, z = far, w unused
};

class WaterPass final : public RenderTechnique
{
public:
    const char* name() const override
    {
        return "Water";
    }

    bool setup() override;
    void execute(const FrameContext& frame) override;
    void shutdown() override;

private:
    BufferHandle mCameraBuffer;
    BufferHandle mReflectionCameraBuffer;
    BufferHandle mWaterBuffer;
    BufferHandle mEnvironmentBuffer;
    BufferHandle mInstanceBuffer; // one mat4 - a water plane is one instance

    // Always bound when reflection/refraction are not available this frame,
    // so the shader never inherits whatever the previous pass left in units
    // 1/2 - see finding 4 in docs/review.md.
    TextureHandle mFallbackBlack;
};

} // namespace Radion

#endif // RADION_WATER_PASS_H
