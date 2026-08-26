#include "PCH.h"

#include "GrassRender.h"

#include "AssetManager.h"
#include "Material.h"
#include "EnvironmentBlock.h"
#include "RenderList.h"
#include "GPUProfiler.h"
#include "Math.h"
#include "Profiler.h"

namespace Radion
{

namespace
{

// Mirrors GrassUniforms in grass_uniforms.glsl, field for field. std140 puts
// each vec3's trailing float in its fourth slot, which is why they are paired.
struct alignas(16) GrassUniforms
{
    glm::mat4 viewProjection = glm::mat4(1.0f);
    glm::mat4 viewProjectionNoJitter = glm::mat4(1.0f);
    glm::mat4 prevViewProjectionNoJitter = glm::mat4(1.0f);

    glm::vec3 cameraPosition = glm::vec3(0.0f);
    f32 time = 0.0f;

    glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
    f32 height = 1.2f;

    glm::vec3 lightDirection = glm::vec3(0.0f, -1.0f, 0.0f);
    f32 width = 0.7f;

    glm::vec3 lightColor = glm::vec3(1.0f);
    f32 wind = 1.0f;

    glm::vec3 ambient = glm::vec3(0.35f);
    f32 drawDistance = 120.0f;

    glm::vec3 viewPosition = glm::vec3(0.0f);
    f32 alphaCut = 0.35f;

    f32 cameraBend = 0.55f;
    f32 stiffness = 12.0f;
    f32 drag = 0.12f;
    f32 deltaTime = 0.0f;

    s32 rectCount = 0;
    s32 clumpCount = 0;
    s32 maxVisible = 0;
    s32 mode = 0;

    s32 reset = 0;
    s32 influencerCount = 0;
    s32 pad0 = 0;
    s32 pad1 = 0;

    glm::vec4 influencers[kGrassMaxInfluencers];
    // A float[8] in std140 is eight sixteen-byte slots, not thirty-two bytes.
    glm::vec4 influencerForces[kGrassMaxInfluencers];
};

// The tip now and one step ago, per tuft. Matches Sim in the shaders.
struct GrassSimState
{
    glm::vec4 currentTail = glm::vec4(0.0f);
    glm::vec4 previousTail = glm::vec4(0.0f);
};

// What glMultiDrawArraysIndirect reads. The compute shader owns instanceCount;
// the rest is written once per frame from here.
struct GrassIndirectArgs
{
    u32 vertexCount = 0;
    u32 instanceCount = 0;
    u32 firstVertex = 0;
    u32 firstInstance = 0;
};

// Three crossed quads, two triangles each, no index buffer.
constexpr u32 kVerticesPerClump = 18;
constexpr u32 kCullGroupSize = 64;

// Storage bindings local to this pass. Instances and Palettes own 0 and 1
// engine-wide, but nothing of theirs is bound while grass draws.
// The numbers the ported shaders declare.
enum GrassStorageBinding : u32
{
    BindingClumps = 4,
    BindingVisible = 5,
    BindingIndirect = 6,
    BindingRects = 7,
    BindingSimulation = 8,
};

class GrassPass final : public RenderTechnique
{
public:
    const char* name() const override
    {
        return "Grass";
    }

    bool setup() override
    {
        GPU& gpu = GPU::getSingleton();

        BufferDesc uniformBuffer;
        uniformBuffer.size = sizeof(GrassUniforms);
        uniformBuffer.usage = BufferUniform;
        uniformBuffer.residency = Residency::Stream;
        uniformBuffer.debugName = "grass.uniforms";
        mUniforms = gpu.createBuffer(uniformBuffer);

        BufferDesc indirect;
        indirect.size = sizeof(GrassIndirectArgs);
        indirect.usage = BufferStorage | BufferIndirect;
        indirect.residency = Residency::Stream;
        indirect.stride = sizeof(u32);
        indirect.debugName = "grass.indirect";
        mIndirect = gpu.createBuffer(indirect);

        SamplerDesc sampler;
        sampler.filter = Filter::Trilinear;
        sampler.wrapU = Wrap::Clamp;
        sampler.wrapV = Wrap::Clamp;
        mSampler = gpu.createSampler(sampler);

        return mUniforms.valid() && mIndirect.valid() && mSampler.valid();
    }

    void execute(const FrameContext& frame) override
    {
        const std::vector<GrassDrawCommand>& commands = GrassDraws().commands();
        if (commands.empty())
            return;

        if (!ensurePipelines())
            return;

        RADION_PROFILE_SCOPE("Grass");
        RADION_GPU_PROFILE_SCOPE("Grass");

        GPU& gpu = GPU::getSingleton();
        gpu.setTarget(frame.target);
        gpu.setViewport(frame.viewport);

        for (const GrassDrawCommand& command : commands)
            draw(gpu, frame, command);
    }

    void shutdown() override
    {
        GPU& gpu = GPU::getSingleton();
        gpu.destroy(mSimulatePipeline);
        gpu.destroy(mCullPipeline);
        gpu.destroy(mDrawPipeline);
        gpu.destroy(mFringePipeline);
        gpu.destroy(mUniforms);
        gpu.destroy(mIndirect);
        gpu.destroy(mClumps);
        gpu.destroy(mRects);
        gpu.destroy(mVisible);
        gpu.destroy(mSimulation);
        gpu.destroy(mSampler);
        mSampler = SamplerHandle();
        mCapacity = 0;
        mRectCapacity = 0;
        mUploadedRects = nullptr;
        mUploadedRectCount = 0;
        mRevision = 0;
        mPipelinesReady = false;
        mPipelinesFailed = false;
        GrassDraws().clear();
    }

private:
    // Built on first use, not in setup(): a technique is set up while the
    // engine starts, and the application has not mounted its assets yet.
    // MaterialManager compiles at first draw for the same reason.
    bool ensurePipelines()
    {
        if (mPipelinesReady)
            return true;
        if (mPipelinesFailed)
            return false;

        GPU& gpu = GPU::getSingleton();
        AssetManager& assets = Assets();

        const std::string& simulate = assets.loadShader("shaders/grass_simulate.comp");
        const std::string& cull = assets.loadShader("shaders/grass_cull.comp");
        const std::string& vertex = assets.loadShader("shaders/grass.vert");
        const std::string& fragment = assets.loadShader("shaders/grass.frag");
        if (simulate.empty() || cull.empty() || vertex.empty() || fragment.empty())
        {
            Log::error("Grass: shaders are missing; the field will not draw");
            mPipelinesFailed = true;
            return false;
        }

        PipelineDesc simulatePipeline;
        simulatePipeline.cs = {simulate.c_str(), 0, "grass_simulate.comp"};
        simulatePipeline.debugName = "grass.simulate";
        mSimulatePipeline = gpu.createPipeline(simulatePipeline);

        PipelineDesc cullPipeline;
        cullPipeline.cs = {cull.c_str(), 0, "grass_cull.comp"};
        cullPipeline.debugName = "grass.cull";
        mCullPipeline = gpu.createPipeline(cullPipeline);

        PipelineDesc drawPipeline;
        drawPipeline.vs = {vertex.c_str(), 0, "grass.vert"};
        drawPipeline.fs = {fragment.c_str(), 0, "grass.frag"};
        // Both faces: a quad stands for something that has no back, and half a
        // field would vanish depending on which way the wind turned it.
        drawPipeline.raster.cull = CullMode::None;
        drawPipeline.debugName = "grass.draw";
        mDrawPipeline = gpu.createPipeline(drawPipeline);

        // Same program, different state: the fringe blends and leaves depth
        // alone, which is the whole difference between the two passes.
        PipelineDesc fringePipeline = drawPipeline;
        fringePipeline.blend.mode = BlendMode::Alpha;
        fringePipeline.depth.write = false;
        fringePipeline.debugName = "grass.fringe";
        mFringePipeline = gpu.createPipeline(fringePipeline);

        mPipelinesReady = mSimulatePipeline.valid() && mCullPipeline.valid() &&
                          mDrawPipeline.valid() && mFringePipeline.valid();
        mPipelinesFailed = !mPipelinesReady;
        return mPipelinesReady;
    }

    // The clump and visible buffers only change when the field is edited, so
    // they are uploaded on a revision change rather than every frame.
    bool ensureClumps(GPU& gpu, const GrassDrawCommand& command)
    {
        if (command.clumpCount > mCapacity || !mClumps.valid() || !mVisible.valid() ||
            !mSimulation.valid())
        {
            u32 capacity = mCapacity ? mCapacity : 4096;
            while (capacity < command.clumpCount)
                capacity *= 2;

            // All three built before any of the old ones are destroyed, and
            // mCapacity only moves once every one of them exists: setting it
            // unconditionally - as this used to - let a failed allocation
            // (mSimulation, say) still raise the bar the next call's count
            // check above compares against, so a permanently invalid buffer
            // never got retried.
            BufferDesc clumps;
            clumps.size = static_cast<u64>(capacity) * sizeof(GrassClump);
            clumps.usage = BufferStorage;
            clumps.residency = Residency::Dynamic;
            clumps.stride = sizeof(GrassClump);
            clumps.debugName = "grass.clumps";
            BufferHandle nextClumps = gpu.createBuffer(clumps);

            BufferDesc visible;
            visible.size = static_cast<u64>(capacity) * sizeof(u32);
            visible.usage = BufferStorage;
            visible.residency = Residency::Static;
            visible.stride = sizeof(u32);
            visible.debugName = "grass.visible";
            BufferHandle nextVisible = gpu.createBuffer(visible);

            BufferDesc simulation;
            simulation.size = static_cast<u64>(capacity) * sizeof(GrassSimState);
            simulation.usage = BufferStorage;
            simulation.residency = Residency::Static;
            simulation.stride = sizeof(GrassSimState);
            simulation.debugName = "grass.simulation";
            BufferHandle nextSimulation = gpu.createBuffer(simulation);

            if (!nextClumps.valid() || !nextVisible.valid() || !nextSimulation.valid())
            {
                gpu.destroy(nextClumps);
                gpu.destroy(nextVisible);
                gpu.destroy(nextSimulation);
                return false;
            }

            gpu.destroy(mClumps);
            gpu.destroy(mVisible);
            gpu.destroy(mSimulation);
            mClumps = nextClumps;
            mVisible = nextVisible;
            mSimulation = nextSimulation;
            mCapacity = capacity;
            mRevision = 0; // the new buffer holds nothing yet
        }

        // A repaint moves tufts the simulation still holds tips for, so the
        // upload and the reset are the same event.
        mReset = false;
        if (command.revision != mRevision)
        {
            gpu.updateBuffer(mClumps, 0, static_cast<u64>(command.clumpCount) * sizeof(GrassClump),
                             command.clumps);
            mRevision = command.revision;
            mReset = true;
        }
        return true;
    }

    bool ensureRects(GPU& gpu, const GrassDrawCommand& command)
    {
        if (command.rectCount > mRectCapacity || !mRects.valid())
        {
            BufferDesc rects;
            rects.size = static_cast<u64>(command.rectCount) * sizeof(GrassAtlasRect);
            rects.usage = BufferStorage;
            rects.residency = Residency::Dynamic;
            rects.stride = sizeof(GrassAtlasRect);
            rects.debugName = "grass.rects";
            BufferHandle next = gpu.createBuffer(rects);
            if (!next.valid())
                return false;

            gpu.destroy(mRects);
            mRects = next;
            mRectCapacity = command.rectCount;
            mUploadedRects = nullptr;
        }

        // Regions are authored once and read every frame. Re-uploading them
        // per frame is small, but it is still a write the GPU has to order
        // against the dispatch that reads them.
        if (command.rects != mUploadedRects || command.rectCount != mUploadedRectCount)
        {
            gpu.updateBuffer(mRects, 0,
                             static_cast<u64>(command.rectCount) * sizeof(GrassAtlasRect),
                             command.rects);
            mUploadedRects = command.rects;
            mUploadedRectCount = command.rectCount;
        }
        return true;
    }

    void draw(GPU& gpu, const FrameContext& frame, const GrassDrawCommand& command)
    {
        if (!command.clumps || command.clumpCount == 0 || !command.rects ||
            command.rectCount == 0)
            return;
        if (!ensureClumps(gpu, command) || !ensureRects(gpu, command))
            return;

        GrassUniforms uniforms;
        uniforms.viewProjection = frame.viewProjection;
        uniforms.viewProjectionNoJitter = frame.viewProjectionNoJitter;
        uniforms.prevViewProjectionNoJitter = frame.prevViewProjectionNoJitter;
        uniforms.cameraPosition = frame.cameraPosition;
        uniforms.viewPosition = frame.cameraPosition;
        uniforms.time = frame.time;
        // The camera's up axis, straight out of the view matrix's second row.
        uniforms.cameraUp = glm::vec3(frame.view[0][1], frame.view[1][1], frame.view[2][1]);
        uniforms.height = command.height;
        uniforms.width = command.width;
        uniforms.wind = command.wind;
        uniforms.alphaCut = command.alphaCut;
        uniforms.cameraBend = command.cameraBend;
        uniforms.drawDistance = command.drawDistance;
        // The same sun and ambient the forward pass binds, so grass and the
        // ground it stands on move together when the time of day does. A
        // directional light in the scene overrides it, for a scene lit by one
        // instead of by a sky.
        const EnvironmentBlock environment = environmentForFrame(frame);
        uniforms.lightDirection = glm::vec3(environment.sunDirection);
        uniforms.lightColor = glm::vec3(environment.sunColor);
        uniforms.ambient = glm::vec3(environment.ambient);
        uniforms.stiffness = command.stiffness;
        uniforms.drag = command.drag;
        uniforms.deltaTime = command.deltaTime;
        uniforms.rectCount = static_cast<s32>(command.rectCount);
        uniforms.clumpCount = static_cast<s32>(command.clumpCount);
        uniforms.maxVisible = static_cast<s32>(mCapacity);
        uniforms.reset = (mReset || command.reset) ? 1 : 0;

        const u32 influencers =
            command.influencers ? glm::min(command.influencerCount, kGrassMaxInfluencers) : 0u;
        uniforms.influencerCount = static_cast<s32>(influencers);
        for (u32 i = 0; i < influencers; ++i)
        {
            const GrassInfluencer& influencer = command.influencers[i];
            uniforms.influencers[i] = glm::vec4(influencer.centre, influencer.radius);
            uniforms.influencerForces[i].x = influencer.force;
        }

        // instanceCount starts at zero for the cull shader to count into.
        const GrassIndirectArgs args{kVerticesPerClump, 0, 0, 0};
        gpu.updateBuffer(mIndirect, 0, sizeof(GrassIndirectArgs), &args);

        gpu.bindStorage(BindingClumps, mClumps);
        gpu.bindStorage(BindingVisible, mVisible);
        gpu.bindStorage(BindingIndirect, mIndirect);
        gpu.bindStorage(BindingRects, mRects);
        gpu.bindStorage(BindingSimulation, mSimulation);

        const u32 groups = (command.clumpCount + kCullGroupSize - 1) / kCullGroupSize;

        uniforms.mode = 0;
        gpu.updateBuffer(mUniforms, 0, sizeof(GrassUniforms), &uniforms);
        gpu.bindUniform(BindingCamera, mUniforms);

        gpu.setPipeline(mSimulatePipeline);
        gpu.dispatch(groups, 1, 1);

        // The draw reads the tips the simulation just wrote.
        gpu.barrier(BarrierStorage);

        gpu.setPipeline(mCullPipeline);
        gpu.dispatch(groups, 1, 1);

        // The instance count the draw reads and the list it walks are both
        // written by the dispatch above.
        gpu.barrier(BarrierIndirect | BarrierStorage);

        if (command.atlas.valid())
            gpu.bindTexture(BindingAlbedo, command.atlas, mSampler);

        DrawDesc draw;
        draw.count = kVerticesPerClump;

        // Pass one is the solid core of the leaf: alpha-tested, writes depth,
        // occludes properly. Pass two is the outline, blended and with depth
        // write off, so the fringe fades instead of stepping.
        gpu.setPipeline(mDrawPipeline);
        gpu.drawIndirect(draw, mIndirect, 0, 1);

        if (!command.softFringe)
            return;

        uniforms.mode = 1;
        gpu.updateBuffer(mUniforms, 0, sizeof(GrassUniforms), &uniforms);
        gpu.bindUniform(BindingCamera, mUniforms);
        gpu.setPipeline(mFringePipeline);
        gpu.drawIndirect(draw, mIndirect, 0, 1);
    }

    PipelineHandle mSimulatePipeline;
    PipelineHandle mCullPipeline;
    PipelineHandle mDrawPipeline;
    PipelineHandle mFringePipeline;
    BufferHandle mUniforms;
    BufferHandle mIndirect;
    BufferHandle mClumps;
    BufferHandle mRects;
    BufferHandle mVisible;
    BufferHandle mSimulation;
    SamplerHandle mSampler;
    u32 mCapacity = 0;
    u32 mRectCapacity = 0;
    const GrassAtlasRect* mUploadedRects = nullptr;
    u32 mUploadedRectCount = 0;
    u64 mRevision = 0;
    bool mReset = true;
    bool mPipelinesReady = false;
    bool mPipelinesFailed = false;
};

} // namespace

GrassRenderQueue& GrassRenderQueue::getSingleton()
{
    static GrassRenderQueue queue;
    return queue;
}

void GrassRenderQueue::clear()
{
    mCommands.clear();
}

void GrassRenderQueue::submit(const GrassDrawCommand& command)
{
    mCommands.push_back(command);
}

const std::vector<GrassDrawCommand>& GrassRenderQueue::commands() const
{
    return mCommands;
}

GrassRenderQueue& GrassDraws()
{
    return GrassRenderQueue::getSingleton();
}

RenderTechnique* createGrassPass()
{
    return new GrassPass();
}

} // namespace Radion
