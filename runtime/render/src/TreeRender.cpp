#include "PCH.h"

#include "TreeRender.h"

#include "AssetManager.h"
#include "CameraBlock.h"
#include "EnvironmentBlock.h"
#include "Log.h"
#include "Material.h"

#include "Math.h"
#include <unordered_map>

namespace Radion
{

namespace
{

// Matches TreeBlock in tree.vert/tree.frag/tree_depth.frag. All three declare
// it identically - the leaf/trunk split and the wind both read from here, so
// the three shaders have to agree on the layout or a draw reads the wrong
// field and the tree deforms.
struct alignas(16) TreeBlock
{
    // x = model height in metres, y = time, z = wind, w = 1 for leaves.
    Math::vec4 wind = Math::vec4(1.0f, 0.0f, 1.0f, 0.0f);
    // x unused, y = alpha cut, z = bark bump, w = capture mode.
    Math::vec4 surface = Math::vec4(0.0f, 0.4f, 1.0f, 0.0f);
    // x = the time one frame ago, for the leaves' previous sway position.
    Math::vec4 temporal = Math::vec4(0.0f);
};

// Matches ImpostorBlock in impostor.vert/frag.
struct alignas(16) ImpostorBlock
{
    // x = height, y = width/height, z = angle count, w = first array layer.
    Math::vec4 shape = Math::vec4(1.0f, 0.85f, 8.0f, 0.0f);
    // x = swap distance, y = band half-width, z = alpha cut.
    Math::vec4 swap = Math::vec4(120.0f, 12.0f, 0.4f, 0.0f);
};

// Its own bindings, above what a Lit material uses, so a tree draw never
// disturbs the forward pass's own state.
constexpr u32 kTreeInstanceBinding = 6;
constexpr u32 kTreeBlockBinding = 6;
constexpr u32 kImpostorBlockBinding = 7;
constexpr u32 kTreeBarkUnit = 0;
constexpr u32 kTreeBarkNormalUnit = 1;
constexpr u32 kTreeTwigUnit = 2;

// How many photographs go round the Y axis, and how big each one is. Eight is
// the reference's own number: every 45 degrees, which at the distance an
// impostor takes over is under the angular size of the tree itself.
constexpr u32 kImpostorAngles = 8;
constexpr u32 kImpostorDimension = 256;
constexpr u32 kImpostorMaxSpecies = 8;

class TreePass final : public RenderTechnique
{
public:
    const char* name() const override
    {
        return "Trees";
    }

    bool setup() override
    {
        // Shaders are NOT loaded here. setup() runs inside Engine::initialize(),
        // and a demo registers its asset search paths after that returns - so
        // reading them now finds nothing. GrassPass defers for the same reason.
        GPU& gpu = GPU::getSingleton();

        BufferDesc cameraDesc;
        cameraDesc.size = sizeof(CameraBlock);
        cameraDesc.usage = BufferUniform;
        cameraDesc.residency = Residency::Dynamic;
        cameraDesc.debugName = "tree.camera";
        mCameraBuffer = gpu.createBuffer(cameraDesc);

        BufferDesc environmentDesc;
        environmentDesc.size = sizeof(EnvironmentBlock);
        environmentDesc.usage = BufferUniform;
        environmentDesc.residency = Residency::Dynamic;
        environmentDesc.debugName = "tree.environment";
        mEnvironmentBuffer = gpu.createBuffer(environmentDesc);

        BufferDesc blockDesc;
        blockDesc.size = sizeof(TreeBlock);
        blockDesc.usage = BufferUniform;
        blockDesc.residency = Residency::Dynamic;
        blockDesc.debugName = "tree.block";
        mBlockBuffer = gpu.createBuffer(blockDesc);

        return mCameraBuffer.valid() && mEnvironmentBuffer.valid() && mBlockBuffer.valid();
    }

    void execute(const FrameContext& frame) override
    {
        const std::vector<TreeDrawCommand>& commands = TreeDraws().commands();
        if (commands.empty())
            return;

        GPU& gpu = GPU::getSingleton();
        AssetManager& assets = Assets();

        const CameraBlock camera{frame.viewProjection,
                                 frame.clipPlane,
                                 Math::vec4(frame.cameraPosition, 1.0f),
                                 frame.view,
                                 frame.viewProjectionNoJitter,
                                 frame.prevViewProjectionNoJitter};
        gpu.updateBuffer(mCameraBuffer, 0, sizeof(camera), &camera);
        gpu.bindUniform(BindingCamera, mCameraBuffer);

        const EnvironmentBlock environment = environmentForFrame(frame);
        gpu.updateBuffer(mEnvironmentBuffer, 0, sizeof(environment), &environment);
        gpu.bindUniform(BindingEnvironment, mEnvironmentBuffer);

        for (const TreeDrawCommand& command : commands)
        {
            if (command.instanceCount == 0 || !command.instances)
                continue;
            const Mesh* mesh = assets.getMesh(command.mesh);
            if (!mesh || mesh->submeshes.empty())
                continue;
            if (!ensurePipeline(mesh->colorLayout))
                continue;
            if (!ensureInstanceCapacity(command.instanceCount))
                continue;

            gpu.updateBuffer(mInstanceBuffer, 0,
                             static_cast<u64>(command.instanceCount) * sizeof(TreeInstanceData),
                             command.instances);
            gpu.bindStorage(kTreeInstanceBinding, mInstanceBuffer);

            // Photographs first, and only when they are missing or stale: the
            // capture draws the tree sixteen times (eight angles, two maps) and
            // has to be done before anything reads the array.
            if (command.impostorsEnabled)
                ensureImpostor(command, *mesh);

            gpu.bindTexture(kTreeBarkUnit, command.bark);
            gpu.bindTexture(kTreeBarkNormalUnit, command.barkNormal);
            gpu.bindTexture(kTreeTwigUnit, command.twigTexture);
            gpu.setPipeline(mPipeline);

            // Trunk then leaves, in the order buildTree() wrote the submeshes.
            // Two draws rather than one because the block's `isLeaves` decides
            // which half of both shaders runs - and it is a uniform, so it
            // cannot change inside a draw.
            for (u32 submeshIndex = 0; submeshIndex < mesh->submeshes.size(); ++submeshIndex)
            {
                const bool leaves = submeshIndex == 1;

                TreeBlock block;
                block.wind = Math::vec4(command.modelHeight, frame.time, command.wind,
                                       leaves ? 1.0f : 0.0f);
                block.surface = Math::vec4(0.0f, command.alphaCut, command.bumpForce, 0.0f);
                block.temporal = Math::vec4(frame.time - frame.deltaTime, 0.0f, 0.0f, 0.0f);
                gpu.updateBuffer(mBlockBuffer, 0, sizeof(block), &block);
                gpu.bindUniform(kTreeBlockBinding, mBlockBuffer);

                const SubMesh& submesh = mesh->submeshes[submeshIndex];
                DrawDesc draw;
                draw.vertexBuffers[0] = mesh->positionBuffer;
                draw.vertexBuffers[1] = mesh->attribBuffer;
                draw.vertexBufferCount = 2;
                draw.indexBuffer = mesh->indexBuffer;
                draw.indexType = mesh->indexType;
                draw.first = submesh.indexOffset;
                draw.count = submesh.indexCount;
                draw.instanceCount = command.instanceCount;
                draw.firstInstance = 0;
                gpu.draw(draw);
            }
        }

        // The impostors go after every mesh, in one blended sweep: they fade in
        // OVER the geometry, so they have to be drawn once the geometry they
        // cover is already down.
        drawImpostors(commands, camera);
    }

    void shutdown() override
    {
        GPU& gpu = GPU::getSingleton();
        gpu.destroy(mCameraBuffer);
        gpu.destroy(mEnvironmentBuffer);
        gpu.destroy(mBlockBuffer);
        gpu.destroy(mInstanceBuffer);
        gpu.destroy(mPipeline);
        for (auto& entry : mImpostorTargets)
            gpu.destroy(entry.second);
        mImpostorTargets.clear();
        gpu.destroy(mImpostorAlbedo);
        gpu.destroy(mImpostorNormal);
        gpu.destroy(mImpostorDepth);
        gpu.destroy(mImpostorBlockBuffer);
        gpu.destroy(mImpostorPipeline);
        mImpostorAlbedo = TextureHandle();
        mImpostorNormal = TextureHandle();
        mImpostorDepth = TextureHandle();
        mImpostorBlockBuffer = BufferHandle();
        mImpostorPipeline = PipelineHandle();
        for (ImpostorSlot& slot : mImpostorSlots)
            slot = ImpostorSlot();
        mCameraBuffer = BufferHandle();
        mEnvironmentBuffer = BufferHandle();
        mBlockBuffer = BufferHandle();
        mInstanceBuffer = BufferHandle();
        mPipeline = PipelineHandle();
        mInstanceCapacity = 0;
    }

private:
    struct ImpostorSlot
    {
        u32 key = 0;
        u32 revision = 0xFFFFFFFFu;
        u32 baseLayer = 0;
        bool used = false;
    };

    // The array holds every species' photographs side by side: eight layers
    // each, indexed by species slot. One array rather than one texture per
    // species so the whole forest's impostors are a single bind.
    bool ensureImpostorArrays()
    {
        if (mImpostorAlbedo.valid())
            return true;
        if (mImpostorFailed)
            return false;

        GPU& gpu = GPU::getSingleton();
        TextureDesc desc;
        desc.type = TextureType::Tex2DArray;
        desc.format = Format::RGBA8;
        desc.width = kImpostorDimension;
        desc.height = kImpostorDimension;
        desc.depth = kImpostorAngles * kImpostorMaxSpecies;
        // The full chain, asked for explicitly: an impostor seen from far away
        // WITHOUT mips is the same shimmer the grass used to have.
        desc.mips = 0;
        desc.usage = TextureSampled | TextureTarget;
        desc.debugName = "tree.impostor.albedo";
        mImpostorAlbedo = gpu.createTexture(desc);

        desc.debugName = "tree.impostor.normal";
        mImpostorNormal = gpu.createTexture(desc);

        TextureDesc depthDesc;
        depthDesc.type = TextureType::Tex2D;
        depthDesc.format = Format::Depth24;
        depthDesc.width = kImpostorDimension;
        depthDesc.height = kImpostorDimension;
        depthDesc.usage = TextureTarget;
        depthDesc.debugName = "tree.impostor.depth";
        mImpostorDepth = gpu.createTexture(depthDesc);

        BufferDesc blockDesc;
        blockDesc.size = sizeof(ImpostorBlock);
        blockDesc.usage = BufferUniform;
        blockDesc.residency = Residency::Dynamic;
        blockDesc.debugName = "tree.impostor.block";
        mImpostorBlockBuffer = gpu.createBuffer(blockDesc);

        if (!mImpostorAlbedo.valid() || !mImpostorNormal.valid() || !mImpostorDepth.valid() ||
            !mImpostorBlockBuffer.valid())
        {
            Log::error("TreePass: impostor arrays unavailable");
            mImpostorFailed = true;
            return false;
        }

        Log::info("TreePass: impostors %ux%u, %u angles, up to %u species", kImpostorDimension,
                  kImpostorDimension, kImpostorAngles, kImpostorMaxSpecies);
        return true;
    }

    // -1 when there is no room left.
    s32 impostorSlotFor(u32 key)
    {
        for (u32 i = 0; i < kImpostorMaxSpecies; ++i)
            if (mImpostorSlots[i].used && mImpostorSlots[i].key == key)
                return static_cast<s32>(i);
        for (u32 i = 0; i < kImpostorMaxSpecies; ++i)
            if (!mImpostorSlots[i].used)
            {
                mImpostorSlots[i].used = true;
                mImpostorSlots[i].key = key;
                mImpostorSlots[i].revision = 0xFFFFFFFFu;
                mImpostorSlots[i].baseLayer = i * kImpostorAngles;
                return static_cast<s32>(i);
            }
        return -1;
    }

    void ensureImpostor(const TreeDrawCommand& command, const Mesh& mesh)
    {
        if (!ensureImpostorArrays())
            return;
        const s32 slot = impostorSlotFor(command.impostorKey);
        if (slot < 0)
            return;
        ImpostorSlot& entry = mImpostorSlots[static_cast<usize>(slot)];
        if (entry.revision == command.impostorRevision)
            return;

        captureImpostor(command, mesh, entry.baseLayer);
        entry.revision = command.impostorRevision;
    }

    // Photographs one species from kImpostorAngles directions around Y, twice
    // per angle: albedo, then world normals.
    void captureImpostor(const TreeDrawCommand& command, const Mesh& mesh, u32 baseLayer)
    {
        GPU& gpu = GPU::getSingleton();
        if (!ensurePipeline(mesh.colorLayout))
            return;

        const f32 height = command.modelHeight;
        const f32 half = height * 0.5f;
        // ORTHOGRAPHIC. An impostor is seen from far away, where perspective is
        // already nearly orthographic - so one photograph serves any distance
        // without the tree appearing to open up as it is approached.
        const f32 width = half * 1.15f; // slack, so the crown is not clipped
        // The vertical window is CENTRED, not 0..height. The camera sits at
        // half height looking level, so in view space the base falls at -half
        // and the top at +half. With 0..height it photographed from half height
        // up: half the image came out empty and the tree sat shrunk in the
        // bottom - the impostor appeared at half the size of the mesh.
        const Math::mat4 projection =
            Math::ortho(-width, width, -half, half, 0.01f, height * 6.0f);

        // One instance, at the origin and unrotated: the rotation is applied on
        // the impostor, when it picks its angle.
        const TreeInstanceData single;
        if (!ensureInstanceCapacity(1))
            return;
        gpu.updateBuffer(mInstanceBuffer, 0, sizeof(single), &single);
        gpu.bindStorage(kTreeInstanceBinding, mInstanceBuffer);

        gpu.bindTexture(kTreeBarkUnit, command.bark);
        gpu.bindTexture(kTreeBarkNormalUnit, command.barkNormal);
        gpu.bindTexture(kTreeTwigUnit, command.twigTexture);
        gpu.setPipeline(mPipeline);

        Viewport viewport;
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<f32>(kImpostorDimension);
        viewport.height = static_cast<f32>(kImpostorDimension);

        for (u32 angle = 0; angle < kImpostorAngles; ++angle)
        {
            const f32 theta =
                Math::two_pi<f32>() * static_cast<f32>(angle) / static_cast<f32>(kImpostorAngles);
            const Math::vec3 eye(std::sin(theta) * height * 3.0f, half,
                                std::cos(theta) * height * 3.0f);
            const Math::mat4 view =
                Math::lookAt(eye, Math::vec3(0.0f, half, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f));

            CameraBlock camera;
            camera.viewProj = projection * view;
            camera.clipPlane = Math::vec4(0.0f);
            camera.cameraPos = Math::vec4(eye, 1.0f);
            camera.view = view;
            // A still capture, not a frame: both matrices are the same one, so
            // the motion vector the shared vertex shader computes is zero.
            camera.viewProjectionNoJitter = camera.viewProj;
            camera.prevViewProjectionNoJitter = camera.viewProj;
            gpu.updateBuffer(mCameraBuffer, 0, sizeof(camera), &camera);
            gpu.bindUniform(BindingCamera, mCameraBuffer);

            // Two passes, colour then normals. No MRT here, and this runs once
            // per species - the cost does not matter.
            for (u32 target = 0; target < 2; ++target)
            {
                const TargetHandle handle =
                    impostorTarget(target == 0 ? mImpostorAlbedo : mImpostorNormal,
                                   baseLayer + angle);
                if (!handle.valid())
                    continue;

                // Cleared to alpha ZERO: the tree's silhouette comes from here.
                ClearValue clear;
                clear.bits = ClearColor | ClearDepth;
                clear.color[0] = clear.color[1] = clear.color[2] = clear.color[3] = 0.0f;
                gpu.setTarget(handle, clear);
                gpu.setViewport(viewport);

                for (u32 submeshIndex = 0; submeshIndex < mesh.submeshes.size(); ++submeshIndex)
                {
                    TreeBlock block;
                    // No wind and no time: the photograph is of the rest pose,
                    // or every instance would wear one frozen gust.
                    block.wind = Math::vec4(height, 0.0f, 0.0f,
                                           submeshIndex == 1 ? 1.0f : 0.0f);
                    block.surface = Math::vec4(0.0f, command.alphaCut, command.bumpForce,
                                              target == 0 ? 1.0f : 2.0f);
                    gpu.updateBuffer(mBlockBuffer, 0, sizeof(block), &block);
                    gpu.bindUniform(kTreeBlockBinding, mBlockBuffer);

                    const SubMesh& submesh = mesh.submeshes[submeshIndex];
                    DrawDesc draw;
                    draw.vertexBuffers[0] = mesh.positionBuffer;
                    draw.vertexBuffers[1] = mesh.attribBuffer;
                    draw.vertexBufferCount = 2;
                    draw.indexBuffer = mesh.indexBuffer;
                    draw.indexType = mesh.indexType;
                    draw.first = submesh.indexOffset;
                    draw.count = submesh.indexCount;
                    draw.instanceCount = 1;
                    draw.firstInstance = 0;
                    gpu.draw(draw);
                }
            }
        }

        gpu.generateMips(mImpostorAlbedo);
        gpu.generateMips(mImpostorNormal);
        Log::info("TreePass: species %u photographed (%u angles x 2 maps)", command.impostorKey,
                  kImpostorAngles);
    }

    // One target per (texture, layer), built on demand and kept: an FBO is
    // cheap to hold and rebuilding it every angle every capture is not.
    TargetHandle impostorTarget(TextureHandle color, u32 layer)
    {
        const u64 key = (static_cast<u64>(color.index) << 32) | layer;
        auto found = mImpostorTargets.find(key);
        if (found != mImpostorTargets.end())
            return found->second;

        TargetDesc desc;
        desc.colorCount = 1;
        desc.colors[0].texture = color;
        desc.colors[0].slice = layer;
        desc.depth.texture = mImpostorDepth;
        desc.debugName = "tree.impostor.capture";
        const TargetHandle handle = GPU::getSingleton().createTarget(desc);
        mImpostorTargets[key] = handle;
        return handle;
    }

    void drawImpostors(const std::vector<TreeDrawCommand>& commands, const CameraBlock& camera)
    {
        if (!mImpostorAlbedo.valid())
            return;

        GPU& gpu = GPU::getSingleton();
        bool restored = false;

        for (const TreeDrawCommand& command : commands)
        {
            if (!command.impostorsEnabled || command.impostorInstanceCount == 0 ||
                !command.impostorInstances)
                continue;
            const s32 slot = impostorSlotFor(command.impostorKey);
            if (slot < 0 || !ensureImpostorPipeline())
                continue;
            if (!ensureInstanceCapacity(command.impostorInstanceCount))
                continue;

            if (!restored)
            {
                // The capture left its own camera bound; put the frame's back.
                gpu.updateBuffer(mCameraBuffer, 0, sizeof(camera), &camera);
                gpu.bindUniform(BindingCamera, mCameraBuffer);
                restored = true;
            }

            gpu.updateBuffer(mInstanceBuffer, 0,
                             static_cast<u64>(command.impostorInstanceCount) *
                                 sizeof(TreeInstanceData),
                             command.impostorInstances);
            gpu.bindStorage(kTreeInstanceBinding, mInstanceBuffer);

            ImpostorBlock block;
            block.shape = Math::vec4(command.modelHeight, command.impostorWidth,
                                    static_cast<f32>(kImpostorAngles),
                                    static_cast<f32>(mImpostorSlots[static_cast<usize>(slot)]
                                                         .baseLayer));
            block.swap = Math::vec4(command.swapDistance, command.swapBand, command.alphaCut, 0.0f);
            gpu.updateBuffer(mImpostorBlockBuffer, 0, sizeof(block), &block);
            gpu.bindUniform(kImpostorBlockBinding, mImpostorBlockBuffer);

            gpu.bindTexture(0, mImpostorAlbedo);
            gpu.bindTexture(1, mImpostorNormal);
            gpu.setPipeline(mImpostorPipeline);

            // Four corners from gl_VertexID, one quad per instance: no vertex
            // buffer and no index buffer at all.
            DrawDesc draw;
            draw.count = 4;
            draw.instanceCount = command.impostorInstanceCount;
            gpu.draw(draw);
        }
    }

    bool ensureImpostorPipeline()
    {
        if (mImpostorPipeline.valid())
            return true;
        if (mImpostorPipelineFailed)
            return false;

        AssetManager& assets = Assets();
        const std::string& vertex = assets.loadShader("impostor.vert");
        const std::string& fragment = assets.loadShader("impostor.frag");
        if (vertex.empty() || fragment.empty())
        {
            Log::error("TreePass: impostor.vert/impostor.frag missing");
            mImpostorPipelineFailed = true;
            return false;
        }

        PipelineDesc desc;
        desc.vs = {vertex.c_str(), 0, "impostor.vert"};
        desc.fs = {fragment.c_str(), 0, "impostor.frag"};
        desc.topology = Topology::TriangleStrip;
        desc.depth.test = true;
        // Tests but does not WRITE: the impostor blends in over the mesh, and
        // an impostor that wrote depth would occlude the very geometry it is
        // supposed to be fading over.
        desc.depth.write = false;
        desc.depth.func = Compare::LessEqual;
        desc.blend.mode = BlendMode::Alpha;
        desc.raster.cull = CullMode::None;
        desc.debugName = "tree.impostor";
        mImpostorPipeline = GPU::getSingleton().createPipeline(desc);
        mImpostorPipelineFailed = !mImpostorPipeline.valid();
        return mImpostorPipeline.valid();
    }

    // Built on first use rather than in setup(): the layout comes from a mesh,
    // and no mesh exists yet when the pass is created.
    bool ensurePipeline(const VertexLayout& layout)
    {
        if (mPipeline.valid())
            return true;
        if (mPipelineFailed)
            return false;

        AssetManager& assets = Assets();
        const std::string& vertex = assets.loadShader("tree.vert");
        const std::string& fragment = assets.loadShader("tree.frag");
        if (vertex.empty() || fragment.empty())
        {
            Log::error("TreePass: tree.vert/tree.frag missing");
            mPipelineFailed = true;
            return false;
        }

        PipelineDesc desc;
        desc.vs = {vertex.c_str(), 0, "tree.vert"};
        desc.fs = {fragment.c_str(), 0, "tree.frag"};
        desc.layout = layout;
        desc.depth.test = true;
        desc.depth.write = true;
        desc.depth.func = Compare::Less;
        // Back-facing, not off: the twig cards come in coplanar PAIRS with
        // opposite winding (see createTwigs), so culling keeps exactly one of
        // each whichever side the camera is on. With culling off both draw in
        // the same place and fight over depth - that is the strange pattern
        // where leaves touch.
        desc.raster.cull = CullMode::Back;
        desc.debugName = "tree.draw";
        mPipeline = GPU::getSingleton().createPipeline(desc);
        mPipelineFailed = !mPipeline.valid();
        return mPipeline.valid();
    }

    bool ensureInstanceCapacity(u32 count)
    {
        if (count <= mInstanceCapacity && mInstanceBuffer.valid())
            return true;

        GPU& gpu = GPU::getSingleton();
        gpu.destroy(mInstanceBuffer);

        const u32 capacity = Math::max(count, Math::max(mInstanceCapacity * 2u, 256u));
        BufferDesc desc;
        desc.size = static_cast<u64>(capacity) * sizeof(TreeInstanceData);
        desc.usage = BufferStorage;
        desc.residency = Residency::Stream;
        desc.stride = sizeof(TreeInstanceData);
        desc.debugName = "tree.instances";
        mInstanceBuffer = gpu.createBuffer(desc);
        mInstanceCapacity = mInstanceBuffer.valid() ? capacity : 0;
        return mInstanceBuffer.valid();
    }

    BufferHandle mCameraBuffer;
    BufferHandle mEnvironmentBuffer;
    BufferHandle mBlockBuffer;
    BufferHandle mInstanceBuffer;
    PipelineHandle mPipeline;
    bool mPipelineFailed = false;
    u32 mInstanceCapacity = 0;

    TextureHandle mImpostorAlbedo;
    TextureHandle mImpostorNormal;
    TextureHandle mImpostorDepth;
    BufferHandle mImpostorBlockBuffer;
    PipelineHandle mImpostorPipeline;
    std::unordered_map<u64, TargetHandle> mImpostorTargets;
    ImpostorSlot mImpostorSlots[kImpostorMaxSpecies];
    bool mImpostorFailed = false;
    bool mImpostorPipelineFailed = false;
};

} // namespace

TreeRenderQueue& TreeRenderQueue::getSingleton()
{
    static TreeRenderQueue queue;
    return queue;
}

void TreeRenderQueue::clear()
{
    mCommands.clear();
}

void TreeRenderQueue::submit(const TreeDrawCommand& command)
{
    mCommands.push_back(command);
}

const std::vector<TreeDrawCommand>& TreeRenderQueue::commands() const
{
    return mCommands;
}

TreeRenderQueue& TreeDraws()
{
    return TreeRenderQueue::getSingleton();
}

RenderTechnique* createTreePass()
{
    return new TreePass();
}

} // namespace Radion
