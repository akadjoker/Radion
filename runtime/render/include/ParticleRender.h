#ifndef RADION_PARTICLE_RENDER_H
#define RADION_PARTICLE_RENDER_H

#include "GPU.h"

#include "Math.h"
#include <vector>

namespace Radion
{

 
class ParticleSystem
{
public:
 
    struct Emitter
    {
        Math::vec3 position = Math::vec3(0.0f);

        // Continuous emission (particles per second). 0 = bursts only.
        f32 rate = 0.0f;

        // Base direction and cone half-angle, in radians.
        // spread = PI gives a full sphere - what an explosion wants.
        Math::vec3 direction = Math::vec3(0.0f, 1.0f, 0.0f);
        f32 spread = 0.35f;

        f32 speedMin = 4.0f;
        f32 speedMax = 9.0f;

        f32 lifeMin = 1.0f;
        f32 lifeMax = 2.0f;

        f32 sizeBegin = 0.5f;
        f32 sizeEnd = 0.05f;

        Math::vec4 colorBegin = Math::vec4(1.0f, 0.85f, 0.35f, 1.0f);
        Math::vec4 colorEnd = Math::vec4(1.0f, 0.15f, 0.05f, 0.0f); // stored PER particle

        f32 mass = 1.0f;

        f32 rotationVelocity = 1.5f; // rad/s, randomised sign

        f32 startRadius = 0.0f;
    };

    // Global forces: properties of the world, not the emitter.
    Math::vec3 gravity = Math::vec3(0.0f, -9.8f, 0.0f);

    // Exponential drag; high stops the spread quickly (fireball vs flying shrapnel).
    f32 drag = 0.6f;

    // Sampled per particle in particle.frag and multiplied into the radial falloff. One texture for the whole pool (shared by all emitters); left invalid, render() binds a 1x1 white pixel.
    TextureHandle texture;

    bool create(u32 maxParticles = 262144);
    void shutdown();

    // Requests N particles at once: a burst is an emitter with rate 0 and a one-off count.
    void burst(const Emitter& emitter, u32 count);

    // Accumulates the leftover fraction so a low rate does not truncate to zero.
    void emitContinuous(const Emitter& emitter, f32 deltaTime);

    // Runs the four passes. Call once per frame.
    void update(f32 deltaTime);

    // A single indirect draw; the GPU writes the instance count.
    void render(const Math::mat4& viewProjection, const Math::vec3& cameraRight,
               const Math::vec3& cameraUp, bool additive);

    u32 maxParticles() const
    {
        return mMax;
    }

    // OPTIONAL diagnostics: reading the counter buffer back stalls the GPU, so this samples every N frames and returns the cached value otherwise.
    struct Stats
    {
        u32 alive = 0;
        s32 dead = 0;
    };
    Stats readStats() const;
    s32 statsIntervalFrames = 30; // 0 = never read

private:
    struct PendingEmit
    {
        Emitter emitter;
        u32 count = 0;
    };

    bool ensurePipelines();

    u32 mMax = 0;
    bool mValid = false;

    BufferHandle mParticleBuffer;
    BufferHandle mAliveBuffer[2];
    BufferHandle mDeadBuffer;
    BufferHandle mCounterBuffer;
    BufferHandle mIndirectBuffer;
    u32 mCurrent = 0; // which of mAliveBuffer is CURRENT this frame

    BufferHandle mEmitBlockBuffer;
    BufferHandle mSimulateBlockBuffer;
    BufferHandle mDrawBlockBuffer;

    TextureHandle mWhiteTexture; // what render() binds when texture above is invalid
    SamplerHandle mSampler;

    PipelineHandle mEmitPipeline;
    PipelineHandle mKickoffPipeline;
    PipelineHandle mSimulatePipeline;
    PipelineHandle mFinishPipeline;
    PipelineHandle mDrawPipeline;     // alpha blend
    PipelineHandle mAdditivePipeline; // additive blend
    bool mPipelinesReady = false;
    bool mPipelinesFailed = false;

    std::vector<PendingEmit> mPending;
    f32 mEmitAccumulator = 0.0f;
    u32 mFrame = 0;
    mutable Stats mStatsCache;
    mutable u32 mStatsFrame = 0xFFFFFFFFu;
};

} // namespace Radion

#endif // RADION_PARTICLE_RENDER_H
