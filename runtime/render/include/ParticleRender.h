#ifndef RADION_PARTICLE_RENDER_H
#define RADION_PARTICLE_RENDER_H

#include "GPU.h"

#include <glm/glm.hpp>
#include <vector>

namespace Radion
{

 
class ParticleSystem
{
public:
 
    struct Emitter
    {
        glm::vec3 position = glm::vec3(0.0f);

        // Continuous emission (particles per second). 0 = bursts only.
        f32 rate = 0.0f;

        // Base direction and cone half-angle, in radians.
        // spread = PI gives a full sphere - what an explosion wants.
        glm::vec3 direction = glm::vec3(0.0f, 1.0f, 0.0f);
        f32 spread = 0.35f;

        f32 speedMin = 4.0f;
        f32 speedMax = 9.0f;

        f32 lifeMin = 1.0f;
        f32 lifeMax = 2.0f;

        f32 sizeBegin = 0.5f;
        f32 sizeEnd = 0.05f;

        glm::vec4 colorBegin = glm::vec4(1.0f, 0.85f, 0.35f, 1.0f);
        glm::vec4 colorEnd = glm::vec4(1.0f, 0.15f, 0.05f, 0.0f); // stored PER particle

        f32 mass = 1.0f;

        f32 rotationVelocity = 1.5f; // rad/s, randomised sign

        // Initial position spread over a sphere of this radius.
        f32 startRadius = 0.0f;
    };

    // Global forces, the same for every particle. Properties of the WORLD,
    // not of the emitter - which is why they live here and not on Emitter.
    glm::vec3 gravity = glm::vec3(0.0f, -9.8f, 0.0f);

    // Exponential drag. High stops the spread quickly: it is what tells a
    // fireball (high drag, stays put) from flying shrapnel (low drag, keeps
    // going).
    f32 drag = 0.6f;

    // Sampled per-particle in particle.frag, multiplied into the procedural
    // radial falloff every particle already has - a soft round sprite reads
    // as an actual mote/spark instead of pure math, the same role a texture
    // plays in every particle system this one gets compared to. One texture
    // for the whole pool, same limitation gravity/drag above already have:
    // every emitter sharing this system (smoke, sparks, dust...) draws with
    // it. Left invalid, render() binds a 1x1 white pixel instead - visually
    // identical to not having a texture at all.
    TextureHandle texture;

    bool create(u32 maxParticles = 262144);
    void shutdown();

    // Requests N particles at once. This is the basis of explosions and
    // fireworks: a burst is an emitter with rate 0 and a one-off count.
    void burst(const Emitter& emitter, u32 count);

    // Continuous emission over the frame. Accumulates the leftover fraction,
    // so a low rate does not get stuck at zero from truncation.
    void emitContinuous(const Emitter& emitter, f32 deltaTime);

    // Runs the four passes. Call once per frame.
    void update(f32 deltaTime);

    // Draws. A single indirect draw, with the instance count written by the
    // GPU.
    void render(const glm::mat4& viewProjection, const glm::vec3& cameraRight,
               const glm::vec3& cameraUp, bool additive);

    u32 maxParticles() const
    {
        return mMax;
    }

    // OPTIONAL diagnostics. Reading the counter buffer back SYNCHRONISES with
    // the GPU: a full pipeline stall. Called every frame, alone, it would tank
    // the frame rate - and it would contradict the whole point of this
    // system, which exists precisely to never read anything back.
    //
    // So it samples every N frames and returns the cached value the rest of
    // the time. Plenty for a number on screen.
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
