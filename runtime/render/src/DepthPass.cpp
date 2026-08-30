#include "PCH.h"

#include "DepthPass.h"

#include "AssetManager.h"
#include "CameraBlock.h"
#include "GPUProfiler.h"
#include "Log.h"
#include "RenderList.h"

namespace Radion
{

namespace
{
struct PointDepthBlock
{
    glm::vec4 lightPositionAndRange;
    glm::vec4 bias;
};

CullMode shadowCullMode(const Material* material, bool forceFront)
{
    if (!material)
        return forceFront ? CullMode::Front : CullMode::Back;
    if ((material->flags & MaterialTwoSided) != 0)
        return CullMode::None;
    return forceFront ? CullMode::Front : material->cull;
}
} // namespace

bool DepthPass::setup()
{
    GPU& gpu = GPU::getSingleton();

    BufferDesc desc;
    desc.size = sizeof(CameraBlock);
    desc.usage = BufferUniform;
    desc.residency = Residency::Stream;
    desc.debugName = "depth.camera";
    mCameraBuffer = gpu.createBuffer(desc);

    BufferDesc pointDesc;
    pointDesc.size = sizeof(PointDepthBlock);
    pointDesc.usage = BufferUniform;
    pointDesc.residency = Residency::Stream;
    pointDesc.debugName = "depth.point";
    mPointDepthBuffer = gpu.createBuffer(pointDesc);

    return mCameraBuffer.valid() && mPointDepthBuffer.valid();
}

bool DepthPass::ensureInstanceCapacity(u32 count)
{
    if (count <= mInstanceCapacity && mInstanceBuffer.valid())
        return true;

    u32 capacity = mInstanceCapacity ? mInstanceCapacity : 1024u;
    while (capacity < count)
        capacity *= 2;
    BufferDesc desc;
    desc.size = static_cast<u64>(capacity) * sizeof(GPUInstance);
    desc.usage = BufferStorage;
    desc.residency = Residency::Stream;
    desc.stride = sizeof(GPUInstance);
    desc.debugName = "depth.instances";
    // The new buffer first, both handle and capacity committed together only
    // once it exists: this used to overwrite mInstanceCapacity and destroy
    // the old buffer before knowing createBuffer() would succeed, so a
    // failure left the count check above believing a capacity nothing backed.
    GPU& gpu = GPU::getSingleton();
    BufferHandle next = gpu.createBuffer(desc);
    if (!next.valid())
        return false;
    if (mInstanceBuffer.valid())
        gpu.destroy(mInstanceBuffer);
    mInstanceBuffer = next;
    mInstanceCapacity = capacity;
    return true;
}

bool DepthPass::ensurePaletteCapacity(u32 count)
{
    if (count <= mPaletteCapacity && mPaletteBuffer.valid())
        return true;

    u32 capacity = mPaletteCapacity ? mPaletteCapacity : 256u;
    while (capacity < count)
        capacity *= 2;
    BufferDesc desc;
    desc.size = static_cast<u64>(capacity) * sizeof(glm::mat4);
    desc.usage = BufferStorage;
    desc.residency = Residency::Stream;
    desc.stride = sizeof(glm::mat4);
    desc.debugName = "depth.palettes";
    GPU& gpu = GPU::getSingleton();
    BufferHandle next = gpu.createBuffer(desc);
    if (!next.valid())
        return false;
    if (mPaletteBuffer.valid())
        gpu.destroy(mPaletteBuffer);
    mPaletteBuffer = next;
    mPaletteCapacity = capacity;
    return true;
}

bool DepthPass::ensureIndirectCapacity(u32 count)
{
    if (count <= mIndirectCapacity && mIndirectBuffer.valid())
        return true;
    u32 capacity = mIndirectCapacity ? mIndirectCapacity : 1024u;
    while (capacity < count)
        capacity *= 2;
    BufferDesc desc;
    desc.size = static_cast<u64>(capacity) * sizeof(IndirectCommand);
    desc.usage = BufferIndirect;
    desc.residency = Residency::Stream;
    desc.stride = sizeof(IndirectCommand);
    desc.debugName = "depth.indirect";
    GPU& gpu = GPU::getSingleton();
    BufferHandle next = gpu.createBuffer(desc);
    if (!next.valid())
        return false;
    if (mIndirectBuffer.valid())
        gpu.destroy(mIndirectBuffer);
    mIndirectBuffer = next;
    mIndirectCapacity = capacity;
    return true;
}

// The prepass has to cull exactly as the colour pass will. Claiming depth for
// a face the forward pass then discards leaves that pixel with no one to write
// its colour: the sky fails the depth test against geometry that was never
// drawn, and whatever the target held stays on screen.
PipelineHandle DepthPass::pipelineFor(const VertexLayout& layout, bool skinned, bool alphaTest,
                                      CullMode cull, bool pancake)
{
    for (const PipelineEntry& entry : mPipelines)
        if (entry.skinned == skinned && entry.alphaTest == alphaTest && entry.cull == cull &&
            entry.pancake == pancake &&
            std::memcmp(&entry.layout, &layout, sizeof(layout)) == 0)
            return entry.pipeline;

    const std::string& source = Assets().loadShader("depth.vert");
    const std::string& fragmentSource = Assets().loadShader("depth.frag");
    if (source.empty() || fragmentSource.empty())
        return PipelineHandle();
    std::string defines = "#version 450 core\n";
    if (skinned)
        defines += "#define MATERIAL_SKINNED\n";
    if (alphaTest)
        defines += "#define MATERIAL_ALPHA_TEST\n";
    if (pancake)
        defines += "#define SHADOW_PANCAKE\n";
    const std::string vertex = (skinned || alphaTest || pancake)
                                   ? defines + source.substr(source.find('\n') + 1)
                                   : source;
    const std::string fragment = alphaTest
                                     ? "#version 450 core\n#define MATERIAL_ALPHA_TEST\n" +
                                           fragmentSource.substr(fragmentSource.find('\n') + 1)
                                     : fragmentSource;
    PipelineDesc desc;
    desc.vs = {vertex.c_str(), 0, "depth.vert"};
    desc.fs = {fragment.c_str(), 0, "depth.frag"};
    desc.layout = layout;
    desc.blend.writeRGB = false;
    desc.blend.writeA = false;
    desc.depth.test = true;
    desc.depth.write = true;
    desc.depth.func = Compare::Less;
    desc.raster.cull = cull;
    desc.debugName = alphaTest ? "depth.alpha-test" : (skinned ? "depth.skinned" : "depth.static");
    const PipelineHandle pipeline = GPU::getSingleton().createPipeline(desc);
    if (pipeline.valid())
        mPipelines.push_back({layout, skinned, alphaTest, pancake, cull, pipeline});
    return pipeline;
}

// Same shape as pipelineFor(), a separate cache because it compiles a
// different pair of shaders (depth_point.vert/frag) and depth.func matters
// less here: gl_FragDepth is written by hand, so LEQUAL versus LESS only
// differs on an exact tie, which a single undeduplicated draw never hits.
PipelineHandle DepthPass::pointPipelineFor(const VertexLayout& layout, bool skinned, bool alphaTest,
                                           CullMode cull)
{
    for (const PipelineEntry& entry : mPointPipelines)
        if (entry.skinned == skinned && entry.alphaTest == alphaTest && entry.cull == cull &&
            std::memcmp(&entry.layout, &layout, sizeof(layout)) == 0)
            return entry.pipeline;

    const std::string& source = Assets().loadShader("depth_point.vert");
    const std::string& fragmentSource = Assets().loadShader("depth_point.frag");
    if (source.empty() || fragmentSource.empty())
        return PipelineHandle();
    std::string defines = "#version 450 core\n";
    if (skinned)
        defines += "#define MATERIAL_SKINNED\n";
    if (alphaTest)
        defines += "#define MATERIAL_ALPHA_TEST\n";
    const std::string vertex = (skinned || alphaTest) ? defines + source.substr(source.find('\n') + 1)
                                                       : source;
    const std::string fragment = alphaTest
                                     ? "#version 450 core\n#define MATERIAL_ALPHA_TEST\n" +
                                           fragmentSource.substr(fragmentSource.find('\n') + 1)
                                     : fragmentSource;
    PipelineDesc desc;
    desc.vs = {vertex.c_str(), 0, "depth_point.vert"};
    desc.fs = {fragment.c_str(), 0, "depth_point.frag"};
    desc.layout = layout;
    desc.blend.writeRGB = false;
    desc.blend.writeA = false;
    desc.depth.test = true;
    desc.depth.write = true;
    desc.depth.func = Compare::Less;
    desc.raster.cull = cull;
    desc.debugName = alphaTest ? "depth.point.alpha-test"
                               : (skinned ? "depth.point.skinned" : "depth.point.static");
    const PipelineHandle pipeline = GPU::getSingleton().createPipeline(desc);
    if (pipeline.valid())
        mPointPipelines.push_back({layout, skinned, alphaTest, false, cull, pipeline});
    return pipeline;
}

bool DepthPass::collectInstances(const FrameContext& frame, RenderCategory category)
{
    const std::vector<RenderPacket>& packets = frame.list->packets(category);
    mInstances.clear();
    mPalettes.clear();
    mInstances.reserve(packets.size());
    for (const RenderPacket& packet : packets)
    {
        const RenderInstance& source = frame.list->instance(packet.instance);
        GPUInstance instance;
        instance.model = frame.list->models()[packet.instance];
        instance.paletteOffset = static_cast<u32>(mPalettes.size());
        instance.padding[0] = instance.padding[1] = instance.padding[2] = 0;
        if (source.palette)
            mPalettes.insert(mPalettes.end(), source.palette->begin(), source.palette->end());
        else if (source.material && (source.material->flags & MaterialSkinned))
        {
            // See ForwardPass::drawCategory()'s identical fallback - the
            // shadow pass compiles the same MATERIAL_SKINNED variant for a
            // skinned mesh and needs the same identity palette when there is
            // no Animator yet to pose it.
            const std::vector<glm::mat4>& identity = RenderList::identityPalette();
            mPalettes.insert(mPalettes.end(), identity.begin(), identity.end());
        }
        mInstances.push_back(instance);
    }
    if (!ensureInstanceCapacity(static_cast<u32>(mInstances.size())))
        return false;
    GPU& gpu = GPU::getSingleton();
    gpu.updateBuffer(mInstanceBuffer, 0, mInstances.size() * sizeof(GPUInstance), mInstances.data());
    gpu.bindStorage(BindingInstances, mInstanceBuffer);
    if (!mPalettes.empty())
    {
        if (!ensurePaletteCapacity(static_cast<u32>(mPalettes.size())))
            return false;
        gpu.updateBuffer(mPaletteBuffer, 0, mPalettes.size() * sizeof(glm::mat4), mPalettes.data());
        gpu.bindStorage(BindingPalettes, mPaletteBuffer);
    }
    return true;
}

DepthPass::DrawGroup& DepthPass::groupFor(MeshHandle mesh, PipelineHandle pipeline)
{
    const u64 key = (static_cast<u64>(mesh.index) << 32) | static_cast<u64>(pipeline.index);
    if (mGroupCount > 0 && mGroupKeys[mGroupCount - 1] == key)
        return mGroups[mGroupCount - 1];
    for (usize i = 0; i + 1 < mGroupCount; ++i)
        if (mGroupKeys[i] == key)
            return mGroups[i];

    if (mGroupCount == mGroups.size())
        mGroups.emplace_back();
    mGroupKeys.push_back(key);
    DrawGroup& group = mGroups[mGroupCount++];
    group.mesh = mesh;
    group.pipeline = pipeline;
    group.commands.clear();
    return group;
}

void DepthPass::drawCategory(const FrameContext& frame, RenderCategory category, f32 biasSlope,
                             f32 biasConstant, bool cullFront, bool pancake, bool forceTwoSided)
{
    if (!frame.list)
        return;
    const std::vector<RenderPacket>& packets = frame.list->packets(category);
    if (packets.empty())
        return;
    if (!collectInstances(frame, category))
        return;

    GPU& gpu = GPU::getSingleton();
    const CameraBlock camera{frame.viewProjection, frame.clipPlane,
                             glm::vec4(frame.cameraPosition, 1.0f), frame.view};
    gpu.updateBuffer(mCameraBuffer, 0, sizeof(camera), &camera);
    gpu.bindUniform(BindingCamera, mCameraBuffer);
    gpu.setTarget(frame.target);
    gpu.setViewport(frame.viewport);

    AssetManager& assets = Assets();
    const bool alphaCategory = category == RenderCategory::AlphaTest;

    // Opaque shadow/depth draws need no per-material texture binding. Gather
    // every submesh that shares one mesh and depth pipeline into a single
    // glMultiDrawElementsIndirect call. A large scene otherwise pays one GL
    // submission per visible piece, per cascade, despite all of them using
    // the same position buffer and shader.
    if (!alphaCategory)
    {
        mGroupCount = 0;
        mGroupKeys.clear();
        u32 commandCount = 0;
        for (usize packetIndex = 0; packetIndex < packets.size(); ++packetIndex)
        {
            const RenderInstance& instance = frame.list->instance(packets[packetIndex].instance);
            const Mesh* mesh = assets.getMesh(instance.mesh);
            if (!mesh || instance.submesh >= mesh->submeshes.size())
                continue;
            const CullMode cull = forceTwoSided
                                      ? (cullFront ? CullMode::Front : CullMode::None)
                                      : shadowCullMode(instance.material, cullFront);
            // depthLayout, not colorLayout: depth.vert reads position and
            // nothing else, and the layout is what drives vertex fetch, so
            // the full one costs 48 bytes a vertex of normals/tangents/UVs
            // no stage ever looks at. Skinned still needs colorLayout - the
            // joints and weights live in its third stream.
            const PipelineHandle pipeline =
                pipelineFor(mesh->isSkinned() ? mesh->colorLayout : mesh->depthLayout,
                            mesh->isSkinned(), false, cull, pancake);
            if (!pipeline.valid())
                continue;
            DrawGroup& group = groupFor(instance.mesh, pipeline);
            const SubMesh& submesh = mesh->submeshes[instance.submesh];
            group.commands.push_back({submesh.indexCount, 1u, submesh.indexOffset, 0,
                                      static_cast<u32>(packetIndex)});
            ++commandCount;
        }

        if (commandCount > 0 && ensureIndirectCapacity(commandCount))
        {
            mCommands.clear();
            mCommands.reserve(commandCount);
            for (usize i = 0; i < mGroupCount; ++i)
            {
                const DrawGroup& group = mGroups[i];
                mCommands.insert(mCommands.end(), group.commands.begin(), group.commands.end());
            }
            gpu.updateBuffer(mIndirectBuffer, 0, mCommands.size() * sizeof(IndirectCommand),
                             mCommands.data());

            u64 commandOffset = 0;
            for (usize i = 0; i < mGroupCount; ++i)
            {
                const DrawGroup& group = mGroups[i];
                const Mesh* mesh = assets.getMesh(group.mesh);
                if (!mesh || group.commands.empty())
                    continue;
                gpu.setPipeline(group.pipeline);
                if (biasSlope != 0.0f || biasConstant != 0.0f)
                    gpu.setDepthBias(biasSlope, biasConstant);
                DrawDesc draw;
                draw.vertexBuffers[0] = mesh->positionBuffer;
                draw.vertexBufferCount = 1;
                if (mesh->isSkinned())
                {
                    draw.vertexBuffers[1] = mesh->attribBuffer;
                    draw.vertexBuffers[2] = mesh->skinBuffer;
                    draw.vertexBufferCount = 3;
                }
                draw.indexBuffer = mesh->indexBuffer;
                draw.indexType = mesh->indexType;
                gpu.drawIndirect(draw, mIndirectBuffer,
                                 commandOffset * sizeof(IndirectCommand),
                                 static_cast<u32>(group.commands.size()));
                commandOffset += group.commands.size();
            }
            return;
        }
    }

    for (usize i = 0; i < packets.size();)
    {
        const RenderInstance& instance = frame.list->instance(packets[i].instance);
        const Mesh* mesh = assets.getMesh(instance.mesh);
        const CullMode cull = forceTwoSided ? (cullFront ? CullMode::Front : CullMode::None)
                                            : shadowCullMode(instance.material, cullFront);
        const bool alphaTest = alphaCategory && instance.material &&
                               instance.material->textures[SlotAlbedo].texture.valid();
        usize end = i + 1;
        while (end < packets.size())
        {
            const RenderInstance& next = frame.list->instance(packets[end].instance);
            if (next.mesh != instance.mesh || next.submesh != instance.submesh ||
                (alphaCategory && next.material != instance.material) ||
                (!forceTwoSided && shadowCullMode(next.material, cullFront) != cull))
                break;
            ++end;
        }
        if (mesh)
        {
            const PipelineHandle pipeline =
                pipelineFor(mesh->colorLayout, mesh->isSkinned(), alphaTest, cull, pancake);
            if (pipeline.valid())
            {
                gpu.setPipeline(pipeline);
                // setPipeline just reapplied this pipeline's own (bias-free)
                // raster state, so the override always has to come after it.
                if (biasSlope != 0.0f || biasConstant != 0.0f)
                    gpu.setDepthBias(biasSlope, biasConstant);
                if (alphaTest && instance.material)
                {
                    gpu.bindUniform(BindingMaterial, instance.material->paramsBuffer);
                    const MaterialTexture& albedo = instance.material->textures[SlotAlbedo];
                    gpu.bindTexture(BindingAlbedo, albedo.texture, albedo.sampler);
                }
                const SubMesh& submesh = mesh->submeshes[instance.submesh];
                DrawDesc draw;
                draw.vertexBuffers[0] = mesh->positionBuffer;
                draw.vertexBuffers[1] = mesh->attribBuffer;
                draw.vertexBufferCount = 2;
                if (mesh->isSkinned())
                {
                    draw.vertexBuffers[2] = mesh->skinBuffer;
                    draw.vertexBufferCount = 3;
                }
                draw.indexBuffer = mesh->indexBuffer;
                draw.indexType = mesh->indexType;
                draw.first = submesh.indexOffset;
                draw.count = submesh.indexCount;
                draw.instanceCount = static_cast<u32>(end - i);
                draw.firstInstance = static_cast<u32>(i);
                gpu.draw(draw);
            }
        }
        i = end;
    }
}

void DepthPass::execute(const FrameContext& frame)
{
    drawCategory(frame, RenderCategory::Opaque, 0.0f, 0.0f, false);
    drawCategory(frame, RenderCategory::AlphaTest, 0.0f, 0.0f, false);
}

void DepthPass::executeBiased(const FrameContext& frame, f32 biasSlope, f32 biasConstant,
                              bool cullFront)
{
    drawCategory(frame, RenderCategory::Opaque, biasSlope, biasConstant, cullFront);
    drawCategory(frame, RenderCategory::AlphaTest, biasSlope, biasConstant, cullFront);
}

void DepthPass::executeShadow(const FrameContext& frame)
{
    // No hardware depth bias and no reversed culling: a directional shadow is
    // biased entirely in the shader, through BIAS_FUNC's shadow_bias and
    // shadow_normal_bias (scene_forward_clustered.glsl:2346-2350), which is
    // why depth_bias_enable is never switched on anywhere in the reference.
    {
        GPUProfileScope scope("Shadow opaque");
        drawCategory(frame, RenderCategory::Opaque, 0.0f, 0.0f, false, true, true);
    }
    {
        GPUProfileScope scope("Shadow alpha test");
        drawCategory(frame, RenderCategory::AlphaTest, 0.0f, 0.0f, false, true, true);
    }
}

void DepthPass::executePoint(const FrameContext& frame, const glm::vec3& lightPosition, f32 range,
                             f32 bias)
{
    if (!frame.list)
        return;

    GPU& gpu = GPU::getSingleton();
    const CameraBlock camera{frame.viewProjection, frame.clipPlane,
                             glm::vec4(frame.cameraPosition, 1.0f), frame.view};
    gpu.updateBuffer(mCameraBuffer, 0, sizeof(camera), &camera);
    gpu.bindUniform(BindingCamera, mCameraBuffer);

    const PointDepthBlock pointBlock{glm::vec4(lightPosition, glm::max(range, 0.001f)),
                                     glm::vec4(bias, 0.0f, 0.0f, 0.0f)};
    gpu.updateBuffer(mPointDepthBuffer, 0, sizeof(pointBlock), &pointBlock);
    gpu.bindUniform(2, mPointDepthBuffer);

    gpu.setTarget(frame.target);
    gpu.setViewport(frame.viewport);

    drawPointCategory(frame, RenderCategory::Opaque);
    drawPointCategory(frame, RenderCategory::AlphaTest);
}

void DepthPass::drawPointCategory(const FrameContext& frame, RenderCategory category)
{
    const std::vector<RenderPacket>& packets = frame.list->packets(category);
    if (packets.empty() || !collectInstances(frame, category))
        return;

    GPU& gpu = GPU::getSingleton();
    AssetManager& assets = Assets();
    const bool alphaCategory = category == RenderCategory::AlphaTest;
    for (usize i = 0; i < packets.size();)
    {
        const RenderInstance& instance = frame.list->instance(packets[i].instance);
        const Mesh* mesh = assets.getMesh(instance.mesh);
        const CullMode cull = shadowCullMode(instance.material, false);
        const bool alphaTest = alphaCategory && instance.material &&
                               instance.material->textures[SlotAlbedo].texture.valid();
        usize end = i + 1;
        while (end < packets.size())
        {
            const RenderInstance& next = frame.list->instance(packets[end].instance);
            if (next.mesh != instance.mesh || next.submesh != instance.submesh ||
                (alphaCategory && next.material != instance.material) ||
                shadowCullMode(next.material, false) != cull)
                break;
            ++end;
        }
        if (mesh)
        {
            const PipelineHandle pipeline =
                pointPipelineFor(mesh->colorLayout, mesh->isSkinned(), alphaTest, cull);
            if (pipeline.valid())
            {
                gpu.setPipeline(pipeline);
                if (alphaTest)
                {
                    gpu.bindUniform(BindingMaterial, instance.material->paramsBuffer);
                    const MaterialTexture& albedo = instance.material->textures[SlotAlbedo];
                    gpu.bindTexture(BindingAlbedo, albedo.texture, albedo.sampler);
                }
                const SubMesh& submesh = mesh->submeshes[instance.submesh];
                DrawDesc draw;
                draw.vertexBuffers[0] = mesh->positionBuffer;
                draw.vertexBuffers[1] = mesh->attribBuffer;
                draw.vertexBufferCount = 2;
                if (mesh->isSkinned())
                {
                    draw.vertexBuffers[2] = mesh->skinBuffer;
                    draw.vertexBufferCount = 3;
                }
                draw.indexBuffer = mesh->indexBuffer;
                draw.indexType = mesh->indexType;
                draw.first = submesh.indexOffset;
                draw.count = submesh.indexCount;
                draw.instanceCount = static_cast<u32>(end - i);
                draw.firstInstance = static_cast<u32>(i);
                gpu.draw(draw);
            }
        }
        i = end;
    }
}

void DepthPass::shutdown()
{
    GPU& gpu = GPU::getSingleton();
    gpu.destroy(mCameraBuffer);
    gpu.destroy(mPointDepthBuffer);
    gpu.destroy(mInstanceBuffer);
    gpu.destroy(mPaletteBuffer);
    gpu.destroy(mIndirectBuffer);
    for (const PipelineEntry& entry : mPipelines)
        gpu.destroy(entry.pipeline);
    for (const PipelineEntry& entry : mPointPipelines)
        gpu.destroy(entry.pipeline);
    mPipelines.clear();
    mPointPipelines.clear();
}

} // namespace Radion
