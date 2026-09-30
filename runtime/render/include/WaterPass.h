#ifndef RADION_WATER_PASS_H
#define RADION_WATER_PASS_H

#include "OffscreenTarget.h"
#include "RenderTechnique.h"

namespace Radion
{

// The scene copy water sampled for refraction this frame, published for a debug view.
constexpr const char* kWaterRefractionDebugTargetName = "water.debug.scene_copy";

// Per-frame water state no other block carries: near/far to linearize refraction depth, time to scroll noise. Bound at BindingWater.
struct WaterBlock
{
    Math::vec4 timeNearFar; // x = time, y = near, z = far, w unused
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

    // Bound when reflection/refraction are unavailable, so the shader never inherits what the previous pass left in units 1/2 (docs/review.md finding 4).
    TextureHandle mFallbackBlack;
};

} // namespace Radion

#endif // RADION_WATER_PASS_H
