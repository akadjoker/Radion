#include "PCH.h"

#include "HairRender.h"

#include "AssetManager.h"
#include "EnvironmentBlock.h"
#include "GPUProfiler.h"
#include "Material.h"
#include "Profiler.h"

#include <unordered_map>

namespace Radion
{

namespace
{

struct alignas(16) HairUniforms
{
    Math::mat4 viewProjection = Math::mat4(1.0f);
    Math::mat4 viewProjectionNoJitter = Math::mat4(1.0f);
    Math::mat4 previousViewProjectionNoJitter = Math::mat4(1.0f);
    Math::mat4 model = Math::mat4(1.0f);
    Math::mat4 previousModel = Math::mat4(1.0f);
    Math::vec4 colorRoughness = Math::vec4(1.0f);
    Math::vec4 cameraTime = Math::vec4(0.0f);
    Math::vec4 cameraUp = Math::vec4(0.0f, 1.0f, 0.0f, 0.0f);
    Math::vec4 lightDirectionWind = Math::vec4(0.0f);
    Math::vec4 lightColorGravity = Math::vec4(1.0f);
    Math::vec4 ambientDeltaTime = Math::vec4(0.0f);
    Math::vec4 physics = Math::vec4(0.0f);
    Math::vec4 appearance = Math::vec4(0.0f);
    Math::ivec4 counts = Math::ivec4(0);
    Math::ivec4 state = Math::ivec4(0);
    HairCollider colliders[kHairMaxColliders];
};

struct HairSimulationState
{
    Math::vec4 current = Math::vec4(0.0f);
    Math::vec4 previous = Math::vec4(0.0f);
};

struct HairPose
{
    Math::vec4 positionLength = Math::vec4(0.0f);
    Math::vec4 normalWidth = Math::vec4(0.0f);
    Math::vec4 previousPosition = Math::vec4(0.0f);
};

struct HairIndirectArgs
{
    u32 vertexCount = 0;
    u32 instanceCount = 0;
    u32 firstVertex = 0;
    u32 firstInstance = 0;
};

struct HairResources
{
    BufferHandle roots;
    BufferHandle visible;
    BufferHandle indirect;
    BufferHandle simulation;
    BufferHandle pose;
    BufferHandle palette;
    BufferHandle previousPalette;
    u32 rootCapacity = 0;
    u32 segmentCapacity = 0;
    u32 paletteCapacity = 0;
    u64 revision = 0;
    u64 lastSeen = 0;
};

constexpr u32 kComputeGroupSize = 64;
constexpr u64 kRetireAfterFrames = 240;

enum HairStorageBinding : u32
{
    BindingRoots = 4,
    BindingVisible = 5,
    BindingIndirect = 6,
    BindingSimulation = 7,
    BindingPose = 8,
    BindingPalette = 9,
    BindingPreviousPalette = 10,
};

u32 growCapacity(u32 current, u32 required, u32 minimum)
{
    u32 result = current ? current : minimum;
    while (result < required)
        result *= 2;
    return result;
}

class HairPass final : public RenderTechnique
{
public:
    const char* name() const override { return "Hair"; }

    bool setup() override
    {
        GPU& gpu = GPU::getSingleton();
        BufferDesc uniforms;
        uniforms.size = sizeof(HairUniforms);
        uniforms.usage = BufferUniform;
        uniforms.residency = Residency::Stream;
        uniforms.debugName = "hair.uniforms";
        mUniforms = gpu.createBuffer(uniforms);

        SamplerDesc sampler;
        sampler.filter = Filter::Trilinear;
        sampler.wrapU = Wrap::Clamp;
        sampler.wrapV = Wrap::Clamp;
        mSampler = gpu.createSampler(sampler);
        return mUniforms.valid() && mSampler.valid();
    }

    void execute(const FrameContext& frame) override
    {
        ++mFrame;
        const std::vector<HairDrawCommand>& commands = HairDraws().commands();
        if (!commands.empty() && ensurePipelines())
        {
            RADION_PROFILE_SCOPE("Hair");
            RADION_GPU_PROFILE_SCOPE("Hair");
            GPU& gpu = GPU::getSingleton();
            gpu.setTarget(frame.target);
            gpu.setViewport(frame.viewport);
            for (const HairDrawCommand& command : commands)
                draw(gpu, frame, command);
        }
        retireUnused();
    }

    void shutdown() override
    {
        GPU& gpu = GPU::getSingleton();
        for (auto& entry : mResources)
            destroyResources(gpu, entry.second);
        mResources.clear();
        gpu.destroy(mSimulatePipeline);
        gpu.destroy(mCullPipeline);
        gpu.destroy(mDrawPipeline);
        gpu.destroy(mFringePipeline);
        gpu.destroy(mUniforms);
        gpu.destroy(mSampler);
        mPipelinesReady = false;
        mPipelinesFailed = false;
        HairDraws().clear();
    }

private:
    bool ensurePipelines()
    {
        if (mPipelinesReady)
            return true;
        if (mPipelinesFailed)
            return false;

        GPU& gpu = GPU::getSingleton();
        AssetManager& assets = Assets();
        const std::string& simulate = assets.loadShader("shaders/hair_simulate.comp");
        const std::string& cull = assets.loadShader("shaders/hair_cull.comp");
        const std::string& vertex = assets.loadShader("shaders/hair.vert");
        const std::string& fragment = assets.loadShader("shaders/hair.frag");
        if (simulate.empty() || cull.empty() || vertex.empty() || fragment.empty())
        {
            Log::error("Hair: one or more shaders are missing");
            mPipelinesFailed = true;
            return false;
        }

        PipelineDesc pipeline;
        pipeline.cs = {simulate.c_str(), 0, "hair_simulate.comp"};
        pipeline.debugName = "hair.simulate";
        mSimulatePipeline = gpu.createPipeline(pipeline);

        pipeline = PipelineDesc();
        pipeline.cs = {cull.c_str(), 0, "hair_cull.comp"};
        pipeline.debugName = "hair.cull";
        mCullPipeline = gpu.createPipeline(pipeline);

        pipeline = PipelineDesc();
        pipeline.vs = {vertex.c_str(), 0, "hair.vert"};
        pipeline.fs = {fragment.c_str(), 0, "hair.frag"};
        pipeline.raster.cull = CullMode::None;
        pipeline.debugName = "hair.draw";
        mDrawPipeline = gpu.createPipeline(pipeline);

        PipelineDesc fringe = pipeline;
        fringe.blend.mode = BlendMode::Alpha;
        fringe.depth.write = false;
        fringe.debugName = "hair.fringe";
        mFringePipeline = gpu.createPipeline(fringe);

        mPipelinesReady = mSimulatePipeline.valid() && mCullPipeline.valid() &&
                          mDrawPipeline.valid() && mFringePipeline.valid();
        mPipelinesFailed = !mPipelinesReady;
        return mPipelinesReady;
    }

    void destroyResources(GPU& gpu, HairResources& resources)
    {
        gpu.destroy(resources.roots);
        gpu.destroy(resources.visible);
        gpu.destroy(resources.indirect);
        gpu.destroy(resources.simulation);
        gpu.destroy(resources.pose);
        gpu.destroy(resources.palette);
        gpu.destroy(resources.previousPalette);
        resources = HairResources();
    }

    bool ensureRoots(GPU& gpu, HairResources& resources, const HairDrawCommand& command,
                     bool& reset)
    {
        const bool resize = !resources.roots.valid() || !resources.visible.valid() ||
                            !resources.indirect.valid() || !resources.simulation.valid() ||
                            !resources.pose.valid() || command.rootCount > resources.rootCapacity ||
                            command.segments != resources.segmentCapacity;
        if (resize)
        {
            const u32 capacity = growCapacity(resources.rootCapacity, command.rootCount, 1024);
            BufferDesc desc;
            desc.usage = BufferStorage;
            desc.residency = Residency::Dynamic;
            desc.size = static_cast<u64>(capacity) * sizeof(HairRoot);
            desc.stride = sizeof(HairRoot);
            desc.debugName = "hair.roots";
            BufferHandle roots = gpu.createBuffer(desc);
            desc.residency = Residency::Static;
            desc.size = static_cast<u64>(capacity) * sizeof(u32);
            desc.stride = sizeof(u32);
            desc.debugName = "hair.visible";
            BufferHandle visible = gpu.createBuffer(desc);
            desc.size = static_cast<u64>(capacity) * command.segments * sizeof(HairSimulationState);
            desc.stride = sizeof(HairSimulationState);
            desc.debugName = "hair.simulation";
            BufferHandle simulation = gpu.createBuffer(desc);
            desc.size = static_cast<u64>(capacity) * sizeof(HairPose);
            desc.stride = sizeof(HairPose);
            desc.debugName = "hair.pose";
            BufferHandle pose = gpu.createBuffer(desc);
            desc.usage = BufferStorage | BufferIndirect;
            desc.residency = Residency::Stream;
            desc.size = sizeof(HairIndirectArgs);
            desc.stride = sizeof(u32);
            desc.debugName = "hair.indirect";
            BufferHandle indirect = gpu.createBuffer(desc);
            if (!roots.valid() || !visible.valid() || !simulation.valid() || !pose.valid() ||
                !indirect.valid())
            {
                gpu.destroy(roots); gpu.destroy(visible); gpu.destroy(simulation);
                gpu.destroy(pose); gpu.destroy(indirect);
                return false;
            }
            gpu.destroy(resources.roots); gpu.destroy(resources.visible);
            gpu.destroy(resources.simulation); gpu.destroy(resources.pose);
            gpu.destroy(resources.indirect);
            resources.roots = roots;
            resources.visible = visible;
            resources.simulation = simulation;
            resources.pose = pose;
            resources.indirect = indirect;
            resources.rootCapacity = capacity;
            resources.segmentCapacity = command.segments;
            resources.revision = 0;
            reset = true;
        }
        if (command.revision != resources.revision)
        {
            gpu.updateBuffer(resources.roots, 0,
                             static_cast<u64>(command.rootCount) * sizeof(HairRoot), command.roots);
            resources.revision = command.revision;
            reset = true;
        }
        return true;
    }

    bool ensurePalettes(GPU& gpu, HairResources& resources, const HairDrawCommand& command,
                        u32& paletteCount, u32& previousPaletteCount)
    {
        static const std::vector<Math::mat4> identity(1, Math::mat4(1.0f));
        const std::vector<Math::mat4>& palette = command.palette && !command.palette->empty()
                                                    ? *command.palette : identity;
        const std::vector<Math::mat4>& previous =
            command.previousPalette && !command.previousPalette->empty()
                ? *command.previousPalette : palette;
        paletteCount = static_cast<u32>(palette.size());
        previousPaletteCount = static_cast<u32>(previous.size());
        const u32 required = Math::max(paletteCount, previousPaletteCount);
        if (!resources.palette.valid() || !resources.previousPalette.valid() ||
            required > resources.paletteCapacity)
        {
            const u32 capacity = growCapacity(resources.paletteCapacity, required, 64);
            BufferDesc desc;
            desc.size = static_cast<u64>(capacity) * sizeof(Math::mat4);
            desc.usage = BufferStorage;
            desc.residency = Residency::Stream;
            desc.stride = sizeof(Math::mat4);
            desc.debugName = "hair.palette";
            BufferHandle current = gpu.createBuffer(desc);
            desc.debugName = "hair.previous_palette";
            BufferHandle old = gpu.createBuffer(desc);
            if (!current.valid() || !old.valid())
            {
                gpu.destroy(current); gpu.destroy(old);
                return false;
            }
            gpu.destroy(resources.palette); gpu.destroy(resources.previousPalette);
            resources.palette = current;
            resources.previousPalette = old;
            resources.paletteCapacity = capacity;
        }
        gpu.updateBuffer(resources.palette, 0,
                         static_cast<u64>(paletteCount) * sizeof(Math::mat4), palette.data());
        gpu.updateBuffer(resources.previousPalette, 0,
                         static_cast<u64>(previousPaletteCount) * sizeof(Math::mat4), previous.data());
        return true;
    }

    void draw(GPU& gpu, const FrameContext& frame, const HairDrawCommand& command)
    {
        if (!command.roots || command.rootCount == 0 || command.key == 0)
            return;
        HairResources& resources = mResources[command.key];
        resources.lastSeen = mFrame;
        bool reset = command.reset;
        if (!ensureRoots(gpu, resources, command, reset))
            return;
        u32 paletteCount = 0, previousPaletteCount = 0;
        if (!ensurePalettes(gpu, resources, command, paletteCount, previousPaletteCount))
            return;

        HairUniforms uniforms;
        uniforms.viewProjection = frame.viewProjection;
        uniforms.viewProjectionNoJitter = frame.viewProjectionNoJitter;
        uniforms.previousViewProjectionNoJitter = frame.prevViewProjectionNoJitter;
        uniforms.model = command.model;
        uniforms.previousModel = command.previousModel;
        uniforms.colorRoughness = Math::vec4(command.color, command.roughness);
        uniforms.cameraTime = Math::vec4(frame.cameraPosition, frame.time);
        uniforms.cameraUp = Math::vec4(frame.view[0][1], frame.view[1][1], frame.view[2][1], 0.0f);
        const EnvironmentBlock environment = environmentForFrame(frame);
        uniforms.lightDirectionWind = Math::vec4(Math::vec3(environment.sunDirection), command.wind);
        uniforms.lightColorGravity = Math::vec4(Math::vec3(environment.sunColor), command.gravity);
        uniforms.ambientDeltaTime = Math::vec4(Math::vec3(environment.ambient), command.deltaTime);
        uniforms.physics = Math::vec4(command.stiffness, command.drag, command.drawDistance,
                                     command.alphaCut);
        uniforms.appearance = Math::vec4(command.specularStrength, command.specularTint,
                                        command.transmission, 0.0f);
        uniforms.counts = Math::ivec4(command.rootCount, command.segments, command.followers,
                                     Math::min(command.colliderCount, kHairMaxColliders));
        uniforms.state = Math::ivec4(reset ? 1 : 0, paletteCount, previousPaletteCount, 0);
        for (u32 i = 0; i < static_cast<u32>(uniforms.counts.w); ++i)
            uniforms.colliders[i] = command.colliders[i];

        const HairIndirectArgs args{command.segments * 6u * command.followers, 0, 0, 0};
        gpu.updateBuffer(resources.indirect, 0, sizeof(args), &args);
        gpu.updateBuffer(mUniforms, 0, sizeof(uniforms), &uniforms);
        gpu.bindUniform(BindingCamera, mUniforms);
        gpu.bindStorage(BindingRoots, resources.roots);
        gpu.bindStorage(BindingVisible, resources.visible);
        gpu.bindStorage(BindingIndirect, resources.indirect);
        gpu.bindStorage(BindingSimulation, resources.simulation);
        gpu.bindStorage(BindingPose, resources.pose);
        gpu.bindStorage(BindingPalette, resources.palette);
        gpu.bindStorage(BindingPreviousPalette, resources.previousPalette);

        const u32 groups = (command.rootCount + kComputeGroupSize - 1) / kComputeGroupSize;
        gpu.setPipeline(mSimulatePipeline);
        gpu.dispatch(groups, 1, 1);
        gpu.barrier(BarrierStorage);
        gpu.setPipeline(mCullPipeline);
        gpu.dispatch(groups, 1, 1);
        gpu.barrier(BarrierIndirect | BarrierStorage);

        gpu.bindTexture(BindingAlbedo,
                        command.texture.valid() ? command.texture : Assets().getDefaultTexture(),
                        mSampler);
        DrawDesc drawDesc;
        drawDesc.count = args.vertexCount;
        gpu.setPipeline(mDrawPipeline);
        gpu.drawIndirect(drawDesc, resources.indirect, 0, 1);
        if (command.softFringe)
        {
            uniforms.state.w = 1;
            gpu.updateBuffer(mUniforms, 0, sizeof(uniforms), &uniforms);
            gpu.bindUniform(BindingCamera, mUniforms);
            gpu.setPipeline(mFringePipeline);
            gpu.drawIndirect(drawDesc, resources.indirect, 0, 1);
        }
    }

    void retireUnused()
    {
        GPU& gpu = GPU::getSingleton();
        for (auto it = mResources.begin(); it != mResources.end();)
        {
            if (mFrame - it->second.lastSeen <= kRetireAfterFrames)
            {
                ++it;
                continue;
            }
            destroyResources(gpu, it->second);
            it = mResources.erase(it);
        }
    }

    PipelineHandle mSimulatePipeline;
    PipelineHandle mCullPipeline;
    PipelineHandle mDrawPipeline;
    PipelineHandle mFringePipeline;
    BufferHandle mUniforms;
    SamplerHandle mSampler;
    std::unordered_map<u64, HairResources> mResources;
    u64 mFrame = 0;
    bool mPipelinesReady = false;
    bool mPipelinesFailed = false;
};

} // namespace

HairRenderQueue& HairRenderQueue::getSingleton()
{
    static HairRenderQueue queue;
    return queue;
}

void HairRenderQueue::clear() { mCommands.clear(); }
void HairRenderQueue::submit(const HairDrawCommand& command)
{
    // A scene can build several camera lists in one engine frame (probe,
    // reflection, game). Hair is a main-frame pass, so keep its latest pose
    // rather than simulating and drawing the same component once per list.
    for (HairDrawCommand& existing : mCommands)
        if (existing.key == command.key)
        {
            existing = command;
            return;
        }
    mCommands.push_back(command);
}
const std::vector<HairDrawCommand>& HairRenderQueue::commands() const { return mCommands; }
HairRenderQueue& HairDraws() { return HairRenderQueue::getSingleton(); }
RenderTechnique* createHairPass() { return new HairPass(); }

} // namespace Radion
