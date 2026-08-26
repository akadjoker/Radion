#include "PCH.h"

#include "ParticleRender.h"

#include "AssetManager.h"
#include "Log.h"

namespace Radion
{

namespace
{

// Mirror of Particle in particle_common.glsl. The two definitions HAVE to
// agree - see the warning in that file.
struct alignas(16) GPUParticle
{
    Math::vec3 position;
    f32 mass;
    Math::vec3 velocity;
    f32 life;
    Math::vec3 force;
    f32 maxLife;
    Math::vec2 sizeBeginEnd;
    f32 rotation;
    f32 rotationVelocity;
    Math::vec4 color;
    Math::vec4 colorEnd;
};
static_assert(sizeof(GPUParticle) == 96, "must match the shader's std430 layout");

struct alignas(16) GPUCounters
{
    u32 aliveCount = 0;
    s32 deadCount = 0;
    u32 emitCount = 0;
    u32 aliveCountAfterSim = 0;
};

struct alignas(16) GPUIndirect
{
    u32 dispatchX = 0, dispatchY = 1, dispatchZ = 1, pad0 = 0;
    u32 vertexCount = 4, instanceCount = 0, firstVertex = 0, baseInstance = 0;
};

// Mirrors EmitBlock in particle_emit.comp.
struct alignas(16) EmitBlock
{
    Math::vec4 positionSpread;
    Math::vec4 directionRadius;
    Math::vec4 speedLife;
    Math::vec4 sizeMassRotation;
    Math::vec4 colorBegin;
    Math::vec4 colorEnd;
    s32 emitCount = 0;
    s32 seed = 0;
    s32 pad0 = 0;
    s32 pad1 = 0;
};

// Mirrors SimulateBlock in particle_simulate.comp.
struct alignas(16) SimulateBlock
{
    Math::vec4 gravityDt;
    Math::vec4 drag;
};

// Mirrors DrawBlock in particle.vert/particle.frag.
struct alignas(16) DrawBlock
{
    Math::mat4 viewProjection = Math::mat4(1.0f);
    Math::vec4 cameraRight = Math::vec4(1.0f, 0.0f, 0.0f, 0.0f);
    Math::vec4 cameraUp = Math::vec4(0.0f, 1.0f, 0.0f, 0.0f); // w = additive flag
};

// Storage bindings, identical across every particle shader - the reference's
// own numbers, kept because the C++ side binds by these too.
enum ParticleStorageBinding : u32
{
    BindingParticles = 0,
    BindingAliveCurrent = 1,
    BindingDead = 2,
    BindingCounters = 3,
    BindingIndirectStorage = 4,
    BindingAliveNew = 5,
};

constexpr u32 kUniformBinding = 0; // EmitBlock/SimulateBlock/DrawBlock each take it in turn
constexpr u32 kThreadCountEmit = 64;

} // namespace

bool ParticleSystem::create(u32 maxParticles)
{
    shutdown();
    mMax = maxParticles;

    GPU& gpu = GPU::getSingleton();

    // Particle pool, uninitialised: emit overwrites a slot before anyone reads
    // it. The only thing that defines the initial state is the dead list.
    BufferDesc particleDesc;
    particleDesc.size = static_cast<u64>(mMax) * sizeof(GPUParticle);
    particleDesc.usage = BufferStorage;
    particleDesc.residency = Residency::Static;
    particleDesc.stride = sizeof(GPUParticle);
    particleDesc.debugName = "particle.pool";
    mParticleBuffer = gpu.createBuffer(particleDesc);

    BufferDesc aliveDesc;
    aliveDesc.size = static_cast<u64>(mMax) * sizeof(u32);
    aliveDesc.usage = BufferStorage;
    aliveDesc.residency = Residency::Static;
    aliveDesc.stride = sizeof(u32);
    aliveDesc.debugName = "particle.alive0";
    mAliveBuffer[0] = gpu.createBuffer(aliveDesc);
    aliveDesc.debugName = "particle.alive1";
    mAliveBuffer[1] = gpu.createBuffer(aliveDesc);

    // Every slot starts free. The stack holds the indices in order; emit pops
    // off the top.
    {
        std::vector<u32> free(mMax);
        for (u32 i = 0; i < mMax; ++i)
            free[i] = i;
        BufferDesc deadDesc;
        deadDesc.size = static_cast<u64>(mMax) * sizeof(u32);
        deadDesc.usage = BufferStorage;
        deadDesc.residency = Residency::Static;
        deadDesc.stride = sizeof(u32);
        deadDesc.data = free.data();
        deadDesc.debugName = "particle.dead";
        mDeadBuffer = gpu.createBuffer(deadDesc);
    }

    {
        GPUCounters counters;
        counters.aliveCount = 0;
        counters.deadCount = static_cast<s32>(mMax); // full stack = everything free
        counters.emitCount = 0;
        counters.aliveCountAfterSim = 0;
        BufferDesc counterDesc;
        counterDesc.size = sizeof(GPUCounters);
        counterDesc.usage = BufferStorage;
        counterDesc.residency = Residency::Static;
        counterDesc.data = &counters;
        counterDesc.debugName = "particle.counters";
        mCounterBuffer = gpu.createBuffer(counterDesc);
    }

    {
        GPUIndirect indirect;
        BufferDesc indirectDesc;
        indirectDesc.size = sizeof(GPUIndirect);
        indirectDesc.usage = BufferStorage | BufferIndirect;
        indirectDesc.residency = Residency::Static;
        indirectDesc.data = &indirect;
        indirectDesc.debugName = "particle.indirect";
        mIndirectBuffer = gpu.createBuffer(indirectDesc);
    }

    BufferDesc emitBlockDesc;
    emitBlockDesc.size = sizeof(EmitBlock);
    emitBlockDesc.usage = BufferUniform;
    emitBlockDesc.residency = Residency::Stream;
    emitBlockDesc.debugName = "particle.emitBlock";
    mEmitBlockBuffer = gpu.createBuffer(emitBlockDesc);

    BufferDesc simulateBlockDesc;
    simulateBlockDesc.size = sizeof(SimulateBlock);
    simulateBlockDesc.usage = BufferUniform;
    simulateBlockDesc.residency = Residency::Stream;
    simulateBlockDesc.debugName = "particle.simulateBlock";
    mSimulateBlockBuffer = gpu.createBuffer(simulateBlockDesc);

    BufferDesc drawBlockDesc;
    drawBlockDesc.size = sizeof(DrawBlock);
    drawBlockDesc.usage = BufferUniform;
    drawBlockDesc.residency = Residency::Stream;
    drawBlockDesc.debugName = "particle.drawBlock";
    mDrawBlockBuffer = gpu.createBuffer(drawBlockDesc);

    {
        const unsigned char whitePixel[4] = {255, 255, 255, 255};
        TextureDesc whiteDesc;
        whiteDesc.width = 1;
        whiteDesc.height = 1;
        whiteDesc.mips = 1;
        whiteDesc.format = Format::RGBA8;
        whiteDesc.data = whitePixel;
        whiteDesc.debugName = "particle.white";
        mWhiteTexture = gpu.createTexture(whiteDesc);
    }
    SamplerDesc samplerDesc;
    samplerDesc.filter = Filter::Linear;
    samplerDesc.wrapU = Wrap::Clamp;
    samplerDesc.wrapV = Wrap::Clamp;
    mSampler = gpu.createSampler(samplerDesc);

    mValid = mParticleBuffer.valid() && mAliveBuffer[0].valid() && mAliveBuffer[1].valid() &&
             mDeadBuffer.valid() && mCounterBuffer.valid() && mIndirectBuffer.valid() &&
             mEmitBlockBuffer.valid() && mSimulateBlockBuffer.valid() && mDrawBlockBuffer.valid() &&
             mWhiteTexture.valid() && mSampler.valid();
    if (mValid)
        Log::info("ParticleSystem: pool of %u (%.1f MB), GPU-driven with indirect draw", mMax,
                  static_cast<f64>(sizeof(GPUParticle)) * mMax / (1024.0 * 1024.0));
    return mValid;
}

void ParticleSystem::shutdown()
{
    GPU& gpu = GPU::getSingleton();
    gpu.destroy(mParticleBuffer);
    gpu.destroy(mAliveBuffer[0]);
    gpu.destroy(mAliveBuffer[1]);
    gpu.destroy(mDeadBuffer);
    gpu.destroy(mCounterBuffer);
    gpu.destroy(mIndirectBuffer);
    gpu.destroy(mEmitBlockBuffer);
    gpu.destroy(mSimulateBlockBuffer);
    gpu.destroy(mDrawBlockBuffer);
    gpu.destroy(mWhiteTexture);
    gpu.destroy(mSampler);
    gpu.destroy(mEmitPipeline);
    gpu.destroy(mKickoffPipeline);
    gpu.destroy(mSimulatePipeline);
    gpu.destroy(mFinishPipeline);
    gpu.destroy(mDrawPipeline);
    gpu.destroy(mAdditivePipeline);

    mParticleBuffer = BufferHandle();
    mAliveBuffer[0] = BufferHandle();
    mAliveBuffer[1] = BufferHandle();
    mDeadBuffer = BufferHandle();
    mCounterBuffer = BufferHandle();
    mIndirectBuffer = BufferHandle();
    mEmitBlockBuffer = BufferHandle();
    mSimulateBlockBuffer = BufferHandle();
    mDrawBlockBuffer = BufferHandle();
    mWhiteTexture = TextureHandle();
    mSampler = SamplerHandle();
    mEmitPipeline = PipelineHandle();
    mKickoffPipeline = PipelineHandle();
    mSimulatePipeline = PipelineHandle();
    mFinishPipeline = PipelineHandle();
    mDrawPipeline = PipelineHandle();
    mAdditivePipeline = PipelineHandle();
    mPipelinesReady = false;
    mPipelinesFailed = false;

    mPending.clear();
    mEmitAccumulator = 0.0f;
    mFrame = 0;
    mCurrent = 0;
    mMax = 0;
    mValid = false;
}

// Built on first use, not in create(): a system can be created while the
// engine starts, before a demo has mounted its asset search paths.
bool ParticleSystem::ensurePipelines()
{
    if (mPipelinesReady)
        return true;
    if (mPipelinesFailed)
        return false;

    GPU& gpu = GPU::getSingleton();
    AssetManager& assets = Assets();

    const std::string& emit = assets.loadShader("particle_emit.comp");
    const std::string& kickoff = assets.loadShader("particle_kickoff.comp");
    const std::string& simulate = assets.loadShader("particle_simulate.comp");
    const std::string& finish = assets.loadShader("particle_finish.comp");
    const std::string& vertex = assets.loadShader("particle.vert");
    const std::string& fragment = assets.loadShader("particle.frag");
    if (emit.empty() || kickoff.empty() || simulate.empty() || finish.empty() || vertex.empty() ||
        fragment.empty())
    {
        Log::error("ParticleSystem: shaders are missing; nothing will draw");
        mPipelinesFailed = true;
        return false;
    }

    PipelineDesc emitDesc;
    emitDesc.cs = {emit.c_str(), 0, "particle_emit.comp"};
    emitDesc.debugName = "particle.emit";
    mEmitPipeline = gpu.createPipeline(emitDesc);

    PipelineDesc kickoffDesc;
    kickoffDesc.cs = {kickoff.c_str(), 0, "particle_kickoff.comp"};
    kickoffDesc.debugName = "particle.kickoff";
    mKickoffPipeline = gpu.createPipeline(kickoffDesc);

    PipelineDesc simulateDesc;
    simulateDesc.cs = {simulate.c_str(), 0, "particle_simulate.comp"};
    simulateDesc.debugName = "particle.simulate";
    mSimulatePipeline = gpu.createPipeline(simulateDesc);

    PipelineDesc finishDesc;
    finishDesc.cs = {finish.c_str(), 0, "particle_finish.comp"};
    finishDesc.debugName = "particle.finish";
    mFinishPipeline = gpu.createPipeline(finishDesc);

    PipelineDesc drawDesc;
    drawDesc.vs = {vertex.c_str(), 0, "particle.vert"};
    drawDesc.fs = {fragment.c_str(), 0, "particle.frag"};
    drawDesc.topology = Topology::TriangleStrip;
    drawDesc.depth.test = true;
    // Writes colour but NOT depth: transparent particles writing depth would
    // occlude each other depending on submission order rather than blending.
    drawDesc.depth.write = false;
    drawDesc.raster.cull = CullMode::None;
    drawDesc.blend.mode = BlendMode::Alpha;
    drawDesc.debugName = "particle.draw";
    mDrawPipeline = gpu.createPipeline(drawDesc);

    // Same shaders, different blend state: the reference toggles
    // glBlendFunc per draw, but a pipeline here bakes its blend mode in, so
    // additive gets a pipeline of its own instead of a runtime switch.
    PipelineDesc additiveDesc = drawDesc;
    additiveDesc.blend.mode = BlendMode::Additive;
    additiveDesc.debugName = "particle.draw.additive";
    mAdditivePipeline = gpu.createPipeline(additiveDesc);

    mPipelinesReady = mEmitPipeline.valid() && mKickoffPipeline.valid() &&
                      mSimulatePipeline.valid() && mFinishPipeline.valid() &&
                      mDrawPipeline.valid() && mAdditivePipeline.valid();
    mPipelinesFailed = !mPipelinesReady;
    return mPipelinesReady;
}

void ParticleSystem::burst(const Emitter& emitter, u32 count)
{
    if (count > 0)
        mPending.push_back({emitter, count});
}

void ParticleSystem::emitContinuous(const Emitter& emitter, f32 deltaTime)
{
    if (emitter.rate <= 0.0f)
        return;

    // Accumulate the fraction. rate=10 at 60 fps is 0.166 particles per
    // frame; without an accumulator truncation would give zero and the
    // emitter would never emit anything.
    mEmitAccumulator += emitter.rate * deltaTime;
    const u32 count = static_cast<u32>(mEmitAccumulator);
    if (count > 0)
    {
        mEmitAccumulator -= static_cast<f32>(count);
        mPending.push_back({emitter, count});
    }
}

void ParticleSystem::update(f32 deltaTime)
{
    if (!mValid || !ensurePipelines())
        return;
    ++mFrame;

    GPU& gpu = GPU::getSingleton();
    const u32 next = 1 - mCurrent;

    gpu.bindStorage(BindingParticles, mParticleBuffer);
    gpu.bindStorage(BindingAliveCurrent, mAliveBuffer[mCurrent]);
    gpu.bindStorage(BindingDead, mDeadBuffer);
    gpu.bindStorage(BindingCounters, mCounterBuffer);
    gpu.bindStorage(BindingIndirectStorage, mIndirectBuffer);
    gpu.bindStorage(BindingAliveNew, mAliveBuffer[next]);

    // ---- 1. EMIT ----
    // Before kickoff, on purpose: new particles enter the CURRENT list right
    // after the survivors and are already simulated this frame.
    if (!mPending.empty())
    {
        gpu.setPipeline(mEmitPipeline);
        for (usize i = 0; i < mPending.size(); ++i)
        {
            const PendingEmit& pending = mPending[i];
            const u32 count = Math::min(pending.count, mMax);
            const Emitter& e = pending.emitter;

            EmitBlock block;
            block.positionSpread = Math::vec4(e.position, e.spread);
            block.directionRadius = Math::vec4(e.direction, e.startRadius);
            block.speedLife = Math::vec4(e.speedMin, e.speedMax, e.lifeMin, e.lifeMax);
            block.sizeMassRotation =
                Math::vec4(e.sizeBegin, e.sizeEnd, e.mass, e.rotationVelocity);
            block.colorBegin = e.colorBegin;
            block.colorEnd = e.colorEnd;
            block.emitCount = static_cast<s32>(count);
            block.seed = static_cast<s32>(mFrame * 9781u + static_cast<u32>(i) * 6151u);
            gpu.updateBuffer(mEmitBlockBuffer, 0, sizeof(block), &block);
            gpu.bindUniform(kUniformBinding, mEmitBlockBuffer);

            gpu.dispatch((count + kThreadCountEmit - 1) / kThreadCountEmit, 1, 1);

            // Between bursts: they all touch the same atomic counters.
            gpu.barrier(BarrierStorage);
        }
        mPending.clear();
    }

    // ---- 2. KICKOFF ----
    gpu.barrier(BarrierStorage);
    gpu.setPipeline(mKickoffPipeline);
    gpu.dispatch(1, 1, 1);

    // The kickoff just wrote the next dispatch's arguments into this buffer,
    // and dispatchIndirect is about to read them as a command. Without the
    // indirect barrier the GPU can read last frame's arguments instead.
    gpu.barrier(BarrierStorage | BarrierIndirect);

    // ---- 3. SIMULATE (indirect) ----
    gpu.setPipeline(mSimulatePipeline);
    SimulateBlock simulateBlock;
    simulateBlock.gravityDt = Math::vec4(gravity, deltaTime);
    simulateBlock.drag = Math::vec4(drag, 0.0f, 0.0f, 0.0f);
    gpu.updateBuffer(mSimulateBlockBuffer, 0, sizeof(simulateBlock), &simulateBlock);
    gpu.bindUniform(kUniformBinding, mSimulateBlockBuffer);
    gpu.dispatchIndirect(mIndirectBuffer, offsetof(GPUIndirect, dispatchX));

    // ---- 4. FINISH ----
    gpu.barrier(BarrierStorage);
    gpu.setPipeline(mFinishPipeline);
    gpu.dispatch(1, 1, 1);

    // The finish pass's writes are read back as draw arguments.
    gpu.barrier(BarrierStorage | BarrierIndirect);

    // Swap the lists: this frame's NEW is next frame's CURRENT.
    mCurrent = next;
}

void ParticleSystem::render(const Math::mat4& viewProjection, const Math::vec3& cameraRight,
                            const Math::vec3& cameraUp, bool additive)
{
    if (!mValid || !mPipelinesReady)
        return;

    GPU& gpu = GPU::getSingleton();

    DrawBlock block;
    block.viewProjection = viewProjection;
    block.cameraRight = Math::vec4(cameraRight, 0.0f);
    block.cameraUp = Math::vec4(cameraUp, additive ? 1.0f : 0.0f);
    gpu.updateBuffer(mDrawBlockBuffer, 0, sizeof(block), &block);
    gpu.bindUniform(kUniformBinding, mDrawBlockBuffer);

    // The list to read is the one simulation just wrote. update() already
    // swapped mCurrent, so that is the one.
    gpu.bindStorage(BindingParticles, mParticleBuffer);
    gpu.bindStorage(BindingAliveNew, mAliveBuffer[mCurrent]);
    gpu.bindTexture(0, texture.valid() ? texture : mWhiteTexture, mSampler);

    gpu.setPipeline(additive ? mAdditivePipeline : mDrawPipeline);

    DrawDesc draw;
    draw.instanceCount = 0; // overwritten by the indirect args
    gpu.drawIndirect(draw, mIndirectBuffer, offsetof(GPUIndirect, vertexCount), 1);
}

ParticleSystem::Stats ParticleSystem::readStats() const
{
    if (!mValid)
        return mStatsCache;

    // Sampled sparsely: this readback stalls the GPU. See the comment on the
    // declaration.
    if (statsIntervalFrames <= 0)
        return mStatsCache;

    if (mStatsFrame == 0xFFFFFFFFu ||
        (mFrame - mStatsFrame) >= static_cast<u32>(statsIntervalFrames))
    {
        GPUCounters counters;
        if (GPU::getSingleton().readBuffer(mCounterBuffer, 0, sizeof(counters), &counters))
        {
            mStatsCache.alive = counters.aliveCountAfterSim;
            mStatsCache.dead = counters.deadCount;
            mStatsFrame = mFrame;
        }
    }
    return mStatsCache;
}

} // namespace Radion
