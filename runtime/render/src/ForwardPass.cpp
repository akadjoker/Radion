#include "PCH.h"

#include "ForwardPass.h"

#include "Profiler.h"

#include "AssetManager.h"
#include "CameraBlock.h"
#include "EnvironmentBlock.h"
#include "MaterialManager.h"
#include "RenderList.h"

namespace Radion
{

bool ForwardPass::setup()
{
    GPU& gpu = GPU::getSingleton();

    BufferDesc desc;
    desc.size = sizeof(CameraBlock);
    desc.usage = BufferUniform;
    desc.residency = Residency::Stream;
    desc.debugName = "forward.camera";
    mCameraBuffer = gpu.createBuffer(desc);

    BufferDesc temporal;
    temporal.size = sizeof(TemporalCameraBlock);
    temporal.usage = BufferUniform;
    temporal.residency = Residency::Stream;
    temporal.debugName = "forward.temporal";
    mTemporalBuffer = gpu.createBuffer(temporal);

    BufferDesc environment;
    environment.size = sizeof(EnvironmentBlock);
    environment.usage = BufferUniform;
    environment.residency = Residency::Stream;
    environment.debugName = "forward.environment";
    mEnvironmentBuffer = gpu.createBuffer(environment);

    BufferDesc mirrorCamera;
    mirrorCamera.size = sizeof(glm::mat4);
    mirrorCamera.usage = BufferUniform;
    mirrorCamera.residency = Residency::Stream;
    mirrorCamera.debugName = "forward.mirror_camera";
    mMirrorCameraBuffer = gpu.createBuffer(mirrorCamera);

    const u8 white = 255;
    TextureDesc texture;
    texture.format = Format::R8;
    texture.width = 1;
    texture.height = 1;
    texture.usage = TextureSampled;
    texture.data = &white;
    texture.debugName = "forward.white_ao";
    mWhiteAO = gpu.createTexture(texture);

    // Stands in for any material sampler the shader declares and the material
    // does not have. GL treats a sampler as used if it is referenced anywhere
    // in the source, even inside a branch a #define has already made dead, and
    // draws with an unbound sampler fail outright.
    const u8 neutral[4] = {255, 255, 255, 255};
    TextureDesc neutralDesc;
    neutralDesc.format = Format::RGBA8;
    neutralDesc.width = 1;
    neutralDesc.height = 1;
    neutralDesc.usage = TextureSampled;
    neutralDesc.data = neutral;
    neutralDesc.debugName = "forward.neutral";
    mNeutral = gpu.createTexture(neutralDesc);

    TextureDesc arrayDesc;
    arrayDesc.type = TextureType::Tex2DArray;
    arrayDesc.format = Format::RGBA8;
    arrayDesc.width = 1;
    arrayDesc.height = 1;
    arrayDesc.depth = 1;
    arrayDesc.usage = TextureSampled;
    arrayDesc.data = neutral;
    arrayDesc.debugName = "forward.neutral_array";
    mNeutralArray = gpu.createTexture(arrayDesc);

    // Black, not white: this stands in for "there is no environment probe",
    // and the reflection term is additive - a white cube would add a full
    // unit of light to every lit surface in the frame. The intensity in the
    // Environment block is zeroed alongside it, so this is belt and braces.
    //
    // One pixel repeated six times, not one: a cube upload reads depth
    // (6) layers' worth of data from a single glTextureSubImage3D call, so
    // a single-pixel source here was six pixels short - the driver read
    // whatever followed it on the stack.
    const u8 black[4 * 6] = {0, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0, 255,
                             0, 0, 0, 255, 0, 0, 0, 255, 0, 0, 0, 255};
    TextureDesc cubeDesc;
    cubeDesc.type = TextureType::TexCube;
    cubeDesc.format = Format::RGBA8;
    cubeDesc.width = 1;
    cubeDesc.height = 1;
    cubeDesc.depth = 6;
    cubeDesc.usage = TextureSampled;
    cubeDesc.data = black;
    cubeDesc.debugName = "forward.neutral_cube";
    mNeutralCube = gpu.createTexture(cubeDesc);

    const u8 transparentBlack[4] = {0, 0, 0, 0};
    TextureDesc mirrorFallbackDesc;
    mirrorFallbackDesc.format = Format::RGBA8;
    mirrorFallbackDesc.width = 1;
    mirrorFallbackDesc.height = 1;
    mirrorFallbackDesc.usage = TextureSampled;
    mirrorFallbackDesc.data = transparentBlack;
    mirrorFallbackDesc.debugName = "forward.mirror_fallback";
    mMirrorFallback = gpu.createTexture(mirrorFallbackDesc);

    return mCameraBuffer.valid() && mTemporalBuffer.valid() && mEnvironmentBuffer.valid() && mMirrorCameraBuffer.valid() &&
           mWhiteAO.valid() && mNeutral.valid() && mNeutralArray.valid() &&
           mNeutralCube.valid() && mMirrorFallback.valid();
}

bool ForwardPass::ensureInstanceCapacity(u32 instances)
{
    if (instances <= mInstanceCapacity && mInstanceBuffer.valid())
        return true;

    u32 capacity = mInstanceCapacity ? mInstanceCapacity : 1024;
    while (capacity < instances)
        capacity *= 2;

    BufferDesc desc;
    desc.size = static_cast<u64>(capacity) * sizeof(GPUInstance);
    desc.usage = BufferStorage;
    desc.residency = Residency::Stream;
    desc.stride = sizeof(GPUInstance);
    desc.debugName = "forward.instances";
    // Created before the old buffer is touched: destroying it first, on the
    // handle capacity already agreed was current, used to mean a failed
    // allocation left mInstanceCapacity claiming a size no buffer backed at
    // all - the next call's early return above believed it and skipped
    // retrying.
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

bool ForwardPass::ensurePaletteCapacity(u32 matrices)
{
    if (matrices <= mPaletteCapacity && mPaletteBuffer.valid())
        return true;

    u32 capacity = mPaletteCapacity ? mPaletteCapacity : 256;
    while (capacity < matrices)
        capacity *= 2;
    BufferDesc desc;
    desc.size = static_cast<u64>(capacity) * sizeof(glm::mat4);
    desc.usage = BufferStorage;
    desc.residency = Residency::Stream;
    desc.stride = sizeof(glm::mat4);
    desc.debugName = "forward.palettes";
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

void ForwardPass::bindFrameState(const FrameContext& frame)
{
    GPU& gpu = GPU::getSingleton();
    gpu.setTarget(frame.target);
    gpu.setViewport(frame.viewport);

    const CameraBlock camera{frame.viewProjection, frame.clipPlane,
                             glm::vec4(frame.cameraPosition, 1.0f), frame.view};
    gpu.updateBuffer(mCameraBuffer, 0, sizeof(CameraBlock), &camera);
    gpu.bindUniform(BindingCamera, mCameraBuffer);
    const TemporalCameraBlock temporal{frame.viewProjectionNoJitter,
                                       frame.prevViewProjectionNoJitter};
    gpu.updateBuffer(mTemporalBuffer, 0, sizeof(temporal), &temporal);
    gpu.bindUniform(BindingTemporal, mTemporalBuffer);

    mFrameEnvironment = environmentForFrame(frame);
    gpu.updateBuffer(mEnvironmentBuffer, 0, sizeof(EnvironmentBlock), &mFrameEnvironment);
    gpu.bindUniform(BindingEnvironment, mEnvironmentBuffer);
    gpu.bindTexture(BindingAmbientOcclusion,
                    frame.ambientOcclusion.valid() ? frame.ambientOcclusion : mWhiteAO);
    gpu.bindTexture(BindingEnvironmentCube,
                    frame.environmentCube.valid() ? frame.environmentCube : mNeutralCube,
                    frame.environmentCubeSampler);

    // MaterialMirror's own reflection, bound unconditionally like the
    // environment cube above - a Lit pipeline declares uMirrorReflectionTex
    // statically once any material in the frame used HAS_MIRROR, so every
    // batch after that one needs a valid binding whether or not IT is a
    // mirror. WaterPass writes the same slots for its own draws right
    // before this pass runs the next frame; each pass rewrites both before
    // its own batches, so the two never fight over them.
    gpu.updateBuffer(mMirrorCameraBuffer, 0, sizeof(glm::mat4), &frame.reflectionViewProj);
    gpu.bindUniform(BindingReflectionCamera, mMirrorCameraBuffer);
    const TextureHandle mirrorReflection =
        Assets().resolveRenderTarget(hashName(kReflectionTargetName));
    // Trilinear, not whatever the texture's own default is: HAS_MIRROR reads
    // this with textureLod for Roughness-driven blur (mReflection.create()'s
    // mips=true), and a sampler that ignores mip level would make Roughness
    // do nothing.
    SamplerDesc mirrorSamplerDesc;
    mirrorSamplerDesc.filter = Filter::Trilinear;
    mirrorSamplerDesc.wrapU = Wrap::Clamp;
    mirrorSamplerDesc.wrapV = Wrap::Clamp;
    gpu.bindTexture(BindingMirrorReflection,
                    mirrorReflection.valid() ? mirrorReflection : mMirrorFallback,
                    Assets().getSampler(mirrorSamplerDesc));
    if (frame.directionalShadow.valid() && frame.directionalShadowBlock.valid())
    {
        gpu.bindUniform(BindingDirectionalShadow, frame.directionalShadowBlock);
        gpu.bindTexture(BindingDirectionalShadowMap, frame.directionalShadow,
                        frame.directionalShadowSampler);
        gpu.bindTexture(BindingDirectionalShadowRaw, frame.directionalShadow,
                        frame.directionalShadowRawSampler);
    }

    // Bound unconditionally, not only while a Lit material is up next: a Lit
    // pipeline's fragment shader declares these SSBOs statically, and Lighting
    // keeps the tile buffer valid (if empty) even with tiled culling off, for
    // exactly this reason - a shader-declared storage binding left unbound
    // fails every draw that follows it, silently, without the debug context on.
    if (frame.entityBuffer.valid())
    {
        gpu.bindStorage(BindingEntities, frame.entityBuffer);
        gpu.bindStorage(BindingEntityMatrices, frame.entityMatrixBuffer);
        gpu.bindStorage(BindingLightTiles, frame.lightTileBuffer);
        gpu.bindUniform(BindingLighting, frame.lightingBlock);
        gpu.bindTexture(BindingShadowAtlas, frame.shadowAtlas, frame.shadowAtlasSampler);
        gpu.bindTexture(BindingDecalAlbedo, frame.decalAlbedo.valid() ? frame.decalAlbedo : mNeutralArray);
        gpu.bindTexture(BindingDecalNormal, frame.decalNormal.valid() ? frame.decalNormal : mNeutralArray);
        gpu.bindTexture(BindingDecalSurface,
                        frame.decalSurface.valid() ? frame.decalSurface : mNeutralArray);
    }
}

void ForwardPass::execute(const FrameContext& frame)
{
    if (!frame.list)
        return;

    bindFrameState(frame);
    drawCategory(frame, RenderCategory::Opaque);
    drawCategory(frame, RenderCategory::AlphaTest);
    drawCategory(frame, RenderCategory::Transparent);
}

void ForwardPass::executeOpaque(const FrameContext& frame)
{
    if (!frame.list)
        return;

    bindFrameState(frame);
    drawCategory(frame, RenderCategory::Opaque);
    drawCategory(frame, RenderCategory::AlphaTest);
}

void ForwardPass::executeTransparent(const FrameContext& frame)
{
    if (!frame.list)
        return;

    bindFrameState(frame);
    drawCategory(frame, RenderCategory::Transparent);
}

void ForwardPass::drawCategory(const FrameContext& frame, RenderCategory category)
{
    GPU& gpu = GPU::getSingleton();
    AssetManager& assets = Assets();
    const std::vector<RenderPacket>& packets = frame.list->packets(category);
    if (packets.empty())
        return;

    // Rewritten into draw order, not submission order: instancing needs a
    // run's matrices to sit next to each other in the buffer, and the sort
    // only made the packets adjacent, not their instance indices.
    const glm::mat4* models = frame.list->models();
    const glm::mat4* prevModels = frame.list->prevModels();
    mGPUInstances.clear();
    mGPUInstances.reserve(packets.size());
    mPalettes.clear();
    RADION_PROFILE_SCOPE("Forward submit");
    for (const RenderPacket& packet : packets)
    {
        const RenderInstance& instance = frame.list->instance(packet.instance);
        GPUInstance gpuInstance;
        gpuInstance.model = models[packet.instance];
        gpuInstance.prevModel = prevModels[packet.instance];
        gpuInstance.paletteOffset = static_cast<u32>(mPalettes.size());
        gpuInstance.prevPaletteOffset = gpuInstance.paletteOffset;
        gpuInstance.padding[0] = gpuInstance.padding[1] = 0;
        if (instance.palette)
        {
            mPalettes.insert(mPalettes.end(), instance.palette->begin(), instance.palette->end());
            gpuInstance.prevPaletteOffset = static_cast<u32>(mPalettes.size());
            const std::vector<glm::mat4>* prevPalette =
                instance.prevPalette ? instance.prevPalette : instance.palette;
            mPalettes.insert(mPalettes.end(), prevPalette->begin(), prevPalette->end());
        }
        else if (instance.material && (instance.material->flags & MaterialSkinned))
        {
            // Skinned material, no Animator yet to pose it (Scene passes
            // palette=nullptr) - identity so the vertex shader's
            // MATERIAL_SKINNED path reads bind pose instead of whatever the
            // buffer held from a previous frame's draw at this offset.
            const std::vector<glm::mat4>& identity = RenderList::identityPalette();
            mPalettes.insert(mPalettes.end(), identity.begin(), identity.end());
            gpuInstance.prevPaletteOffset = static_cast<u32>(mPalettes.size());
            mPalettes.insert(mPalettes.end(), identity.begin(), identity.end());
        }
        mGPUInstances.push_back(gpuInstance);
    }

    const u32 instances = static_cast<u32>(mGPUInstances.size());
    if (!ensureInstanceCapacity(instances))
    {
        Log::error("ForwardPass: failed to allocate the instance buffer");
        return;
    }
    gpu.updateBuffer(mInstanceBuffer, 0, instances * sizeof(GPUInstance), mGPUInstances.data());
    gpu.bindStorage(BindingInstances, mInstanceBuffer);
    if (!mPalettes.empty())
    {
        if (!ensurePaletteCapacity(static_cast<u32>(mPalettes.size())))
            return;
        gpu.updateBuffer(mPaletteBuffer, 0, mPalettes.size() * sizeof(glm::mat4), mPalettes.data());
        gpu.bindStorage(BindingPalettes, mPaletteBuffer);
    }

    PipelineHandle boundPipeline;
    BufferHandle boundParams;
    TextureHandle boundEnvironmentCube =
        frame.environmentCube.valid() ? frame.environmentCube : mNeutralCube;

    for (usize i = 0; i < packets.size();)
    {
        const RenderInstance& instance = frame.list->instance(packets[i].instance);
        const Mesh* mesh = assets.getMesh(instance.mesh);
        if (!mesh)
        {
            ++i;
            continue;
        }

        // How far the run of identical draws reaches. The sort put these
        // together; this is what turns them into one call.
        usize end = i + 1;
        while (end < packets.size())
        {
            const RenderInstance& next = frame.list->instance(packets[end].instance);
            if (next.mesh != instance.mesh || next.submesh != instance.submesh ||
                next.material != instance.material || next.probe.cubemap != instance.probe.cubemap)
                break;
            ++end;
        }

        const Material& material = *instance.material;
        PipelineHandle pipeline = instance.pipeline;
        u8 pipelinePass = MaterialPipelineForward;
        if (!frame.temporalAA)
            pipelinePass |= MaterialPipelineNoTemporal;
        if (pipelinePass != MaterialPipelineForward)
            pipeline = MaterialManager::getSingleton().resolvePipeline(
                const_cast<Material&>(material), mesh->colorLayout, pipelinePass);
        if (pipeline != boundPipeline)
        {
            gpu.setPipeline(pipeline);
            boundPipeline = pipeline;
        }
        if (material.paramsBuffer != boundParams)
        {
            gpu.bindUniform(BindingMaterial, material.paramsBuffer);
            boundParams = material.paramsBuffer;
        }

        {
            // A local ReflectionProbe (scene/) nearest this instance overrides
            // the frame's single default cubemap for this run of packets - see
            // RenderProbe's comment. Runs only merge when they share the same
            // probe (the grouping check above), so every batch here really does
            // want just one binding for its whole instanced draw.
            const TextureHandle desiredCube = instance.probe.cubemap.valid()
                                                  ? instance.probe.cubemap
                                                  : (frame.environmentCube.valid() ? frame.environmentCube
                                                                                    : mNeutralCube);
            if (desiredCube != boundEnvironmentCube)
            {
                gpu.bindTexture(BindingEnvironmentCube, desiredCube,
                                instance.probe.cubemap.valid() ? instance.probe.sampler
                                                                : frame.environmentCubeSampler);
                EnvironmentBlock block = mFrameEnvironment;
                if (instance.probe.cubemap.valid())
                {
                    block.probePositionAndMips =
                        glm::vec4(instance.probe.position, glm::max(instance.probe.mipCount, 1u));
                    block.probeExtentsAndIntensity =
                        glm::vec4(instance.probe.extents, instance.probe.intensity);
                }
                gpu.updateBuffer(mEnvironmentBuffer, 0, sizeof(block), &block);
                gpu.bindUniform(BindingEnvironment, mEnvironmentBuffer);
                boundEnvironmentCube = desiredCube;
            }

            // Bound every run, not skipped on a texture-handle match: a sampler
            // is a separate GL object from the texture it wraps, so two materials
            // sharing one texture with different wrap/filter/anisotropy need
            // their own bind even when the texture side does not change. GLDevice
            // already caches texture and sampler binds independently and skips
            // the redundant GL call itself - see GLDevice::bindTexture() - so
            // there is nothing to save by duplicating that cache here, only a
            // second one that could disagree with it.
            //
            // Only meaningful when the pipeline was compiled with the matching
            // HAS_* define - see MaterialManager::resolvePipeline. Binding a slot
            // the shader never samples costs nothing it would not pay anyway.
            const MaterialTexture& albedo = material.textures[SlotAlbedo];
            gpu.bindTexture(BindingAlbedo, albedo.texture.valid() ? albedo.texture : mNeutral,
                            albedo.sampler);
            const MaterialTexture& detail = material.textures[SlotDetail];
            if (detail.texture.valid())
                gpu.bindTexture(BindingDetail, detail.texture, detail.sampler);

            // Which part of the surface is lit up. Only sampled by a pipeline
            // compiled with HAS_EMISSIVE, so a material without one pays nothing -
            // not even a branch.
            const MaterialTexture& emissive = material.textures[SlotEmissive];
            if (emissive.texture.valid())
                gpu.bindTexture(BindingEmissive, emissive.texture, emissive.sampler);
            // A landscape's colour map. Nothing else in the engine uses this
            // slot, so binding it whenever present costs nothing extra for every
            // other material - the same trade SlotDetail/BindingDetail above
            // already makes.
            const MaterialTexture& colorMap = material.textures[SlotColorMap];
            if (colorMap.texture.valid())
                gpu.bindTexture(BindingColorMap, colorMap.texture, colorMap.sampler);
            // A baked lightmap, sampled through the mesh's own uv2 - see
            // HAS_LIGHTMAP in lit.frag and OgreMeshImporter.
            const MaterialTexture& lightmap = material.textures[SlotLightmap];
            if (lightmap.texture.valid())
                gpu.bindTexture(BindingLightmap, lightmap.texture, lightmap.sampler);
            const MaterialTexture& height = material.textures[SlotHeight];
            if (height.texture.valid())
                gpu.bindTexture(BindingHeight, height.texture, height.sampler);

            const MaterialTexture& normal = material.textures[SlotNormal];
            gpu.bindTexture(BindingNormal, normal.texture.valid() ? normal.texture : mNeutral,
                            normal.sampler);
            const MaterialTexture& surface = material.textures[SlotSurface];
            gpu.bindTexture(BindingSurface, surface.texture.valid() ? surface.texture : mNeutral,
                            surface.sampler);
        }

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

        const SubMesh& submesh = mesh->submeshes[instance.submesh];
        draw.first = submesh.indexOffset;
        draw.count = submesh.indexCount;
        draw.instanceCount = static_cast<u32>(end - i);
        draw.firstInstance = static_cast<u32>(i);
        gpu.draw(draw);

        i = end;
    }
}

void ForwardPass::shutdown()
{
    GPU& gpu = GPU::getSingleton();
    gpu.destroy(mCameraBuffer);
    gpu.destroy(mTemporalBuffer);
    gpu.destroy(mEnvironmentBuffer);
    if (mInstanceBuffer.valid())
        gpu.destroy(mInstanceBuffer);
    if (mPaletteBuffer.valid())
        gpu.destroy(mPaletteBuffer);
    gpu.destroy(mWhiteAO);
    gpu.destroy(mNeutral);
    gpu.destroy(mNeutralArray);
    gpu.destroy(mNeutralCube);
    gpu.destroy(mMirrorCameraBuffer);
    gpu.destroy(mMirrorFallback);
}

} // namespace Radion
