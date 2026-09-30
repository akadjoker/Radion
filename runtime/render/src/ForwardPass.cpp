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
    mirrorCamera.size = sizeof(Math::mat4);
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

    // Stands in for any declared material sampler the material lacks: GL fails draws with an unbound sampler.
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

    // Black, not white: the reflection term is additive, so white would add a unit of light everywhere.
    // One pixel x six faces: a cube upload reads six layers of data from glTextureSubImage3D.
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
    // Create the new buffer before touching the old one, so a failed allocation leaves capacity consistent.
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
    desc.size = static_cast<u64>(capacity) * sizeof(Math::mat4);
    desc.usage = BufferStorage;
    desc.residency = Residency::Stream;
    desc.stride = sizeof(Math::mat4);
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
                             Math::vec4(frame.cameraPosition, 1.0f), frame.view};
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

    // Bound unconditionally: a Lit pipeline declares uMirrorReflectionTex once any material used HAS_MIRROR.
    gpu.updateBuffer(mMirrorCameraBuffer, 0, sizeof(Math::mat4), &frame.reflectionViewProj);
    gpu.bindUniform(BindingReflectionCamera, mMirrorCameraBuffer);
    const TextureHandle mirrorReflection =
        Assets().resolveRenderTarget(hashName(kReflectionTargetName));
    // Trilinear: HAS_MIRROR reads it with textureLod for Roughness blur.
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

    // Bound unconditionally: a shader-declared storage binding left unbound fails every following draw.
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

    // Rewritten into draw order: instancing needs a run's matrices adjacent in the buffer.
    const Math::mat4* models = frame.list->models();
    const Math::mat4* prevModels = frame.list->prevModels();
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
            const std::vector<Math::mat4>* prevPalette =
                instance.prevPalette ? instance.prevPalette : instance.palette;
            mPalettes.insert(mPalettes.end(), prevPalette->begin(), prevPalette->end());
        }
        else if (instance.material && (instance.material->flags & MaterialSkinned))
        {
            // Skinned with no Animator: identity palette so MATERIAL_SKINNED reads bind pose, not stale buffer data.
            const std::vector<Math::mat4>& identity = RenderList::identityPalette();
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
        gpu.updateBuffer(mPaletteBuffer, 0, mPalettes.size() * sizeof(Math::mat4), mPalettes.data());
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
            // A local ReflectionProbe overrides the default cubemap; runs only merge when they share a probe.
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
                        Math::vec4(instance.probe.position, Math::max(instance.probe.mipCount, 1u));
                    block.probeExtentsAndIntensity =
                        Math::vec4(instance.probe.extents, instance.probe.intensity);
                }
                gpu.updateBuffer(mEnvironmentBuffer, 0, sizeof(block), &block);
                gpu.bindUniform(BindingEnvironment, mEnvironmentBuffer);
                boundEnvironmentCube = desiredCube;
            }

            // Bound every run: a sampler is a separate GL object, so materials sharing a texture can differ in wrap/filter.
            // Only meaningful with the matching HAS_* define (MaterialManager::resolvePipeline).
            const MaterialTexture& albedo = material.textures[SlotAlbedo];
            gpu.bindTexture(BindingAlbedo, albedo.texture.valid() ? albedo.texture : mNeutral,
                            albedo.sampler);
            const MaterialTexture& detail = material.textures[SlotDetail];
            if (detail.texture.valid())
                gpu.bindTexture(BindingDetail, detail.texture, detail.sampler);

            const MaterialTexture& emissive = material.textures[SlotEmissive];
            if (emissive.texture.valid())
                gpu.bindTexture(BindingEmissive, emissive.texture, emissive.sampler);
            const MaterialTexture& colorMap = material.textures[SlotColorMap];
            if (colorMap.texture.valid())
                gpu.bindTexture(BindingColorMap, colorMap.texture, colorMap.sampler);
            // Sampled through the mesh's uv2 (HAS_LIGHTMAP in lit.frag).
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
