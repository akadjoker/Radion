#include "PCH.h"

#include "WaterPass.h"

#include "AssetManager.h"
#include "CameraBlock.h"
#include "EnvironmentBlock.h"
#include "Sky.h"
#include "Log.h"
#include "RenderList.h"

namespace Radion
{

bool WaterPass::setup()
{
    GPU& gpu = GPU::getSingleton();

    BufferDesc cameraDesc;
    cameraDesc.size = sizeof(CameraBlock);
    cameraDesc.usage = BufferUniform;
    cameraDesc.residency = Residency::Dynamic;
    cameraDesc.debugName = "water.camera";
    mCameraBuffer = gpu.createBuffer(cameraDesc);

    BufferDesc reflectionDesc;
    reflectionDesc.size = sizeof(glm::mat4);
    reflectionDesc.usage = BufferUniform;
    reflectionDesc.residency = Residency::Dynamic;
    reflectionDesc.debugName = "water.reflection_camera";
    mReflectionCameraBuffer = gpu.createBuffer(reflectionDesc);

    BufferDesc waterDesc;
    waterDesc.size = sizeof(WaterBlock);
    waterDesc.usage = BufferUniform;
    waterDesc.residency = Residency::Dynamic;
    waterDesc.debugName = "water.frame";
    mWaterBuffer = gpu.createBuffer(waterDesc);

    BufferDesc environmentDesc;
    environmentDesc.size = sizeof(EnvironmentBlock);
    environmentDesc.usage = BufferUniform;
    environmentDesc.residency = Residency::Dynamic;
    environmentDesc.debugName = "water.environment";
    mEnvironmentBuffer = gpu.createBuffer(environmentDesc);

    // One water plane's worth up front; grown the same way ForwardPass grows
    // its own if more instances ever submit.
    BufferDesc instanceDesc;
    instanceDesc.size = sizeof(glm::mat4);
    instanceDesc.usage = BufferStorage;
    instanceDesc.residency = Residency::Stream;
    instanceDesc.stride = sizeof(glm::mat4);
    instanceDesc.debugName = "water.instances";
    mInstanceBuffer = gpu.createBuffer(instanceDesc);

    const u8 black[4] = {0, 0, 0, 255};
    TextureDesc fallback;
    fallback.format = Format::RGBA8;
    fallback.width = 1;
    fallback.height = 1;
    fallback.usage = TextureSampled;
    fallback.data = black;
    fallback.debugName = "water.fallback_black";
    mFallbackBlack = gpu.createTexture(fallback);

    return mCameraBuffer.valid() && mReflectionCameraBuffer.valid() && mWaterBuffer.valid() && mEnvironmentBuffer.valid() &&
           mInstanceBuffer.valid() && mFallbackBlack.valid();
}

void WaterPass::execute(const FrameContext& frame)
{
    if (!frame.list)
        return;

    const std::vector<RenderPacket>& packets = frame.list->packets(RenderCategory::Refraction);
    if (packets.empty())
        return;

    GPU& gpu = GPU::getSingleton();
    AssetManager& assets = Assets();

    // Taken while frame.target is still exactly what Forward/Grass/Trees/Sky
    // left in it, and before this pass starts drawing into that same target -
    // see resolveSceneCopy(). Not optional: without an owned copy, refraction
    // either reads a texture nobody in the engine ever wrote (only the water
    // example published one manually) or samples the target it is currently
    // bound to.
    const TextureHandle refraction =
        assets.resolveRenderTarget(hashName(kRefractionTargetName));
    const TextureHandle refractionDepth =
        assets.resolveRenderTarget(hashName(kRefractionDepthTargetName));
    const bool hasRefraction = refraction.valid();

    gpu.setTarget(frame.target);
    gpu.setViewport(frame.viewport);

    const CameraBlock camera{frame.viewProjection, frame.clipPlane,
                             glm::vec4(frame.cameraPosition, 1.0f), frame.view};
    gpu.updateBuffer(mCameraBuffer, 0, sizeof(CameraBlock), &camera);
    gpu.bindUniform(BindingCamera, mCameraBuffer);

    gpu.updateBuffer(mReflectionCameraBuffer, 0, sizeof(glm::mat4), &frame.reflectionViewProj);
    gpu.bindUniform(BindingReflectionCamera, mReflectionCameraBuffer);

    const EnvironmentBlock environment = environmentForFrame(frame);
    gpu.updateBuffer(mEnvironmentBuffer, 0, sizeof(EnvironmentBlock), &environment);
    gpu.bindUniform(BindingEnvironment, mEnvironmentBuffer);

    // Recovered from the projection rather than taken from FrameContext,
    // which carries a near plane but no far one. Both are needed to turn the
    // refraction depth sample back into a distance.
    const glm::mat4& projection = frame.projection;
    WaterBlock water;
    water.timeNearFar.x = frame.time;
    water.timeNearFar.y = projection[3][2] / (projection[2][2] - 1.0f);
    water.timeNearFar.z = projection[3][2] / (projection[2][2] + 1.0f);
    water.timeNearFar.w = 0.0f;
    gpu.updateBuffer(mWaterBuffer, 0, sizeof(WaterBlock), &water);
    gpu.bindUniform(BindingWater, mWaterBuffer);

    // Always bind something: an unbound slot keeps whatever the previous
    // pass left there, which is how the water shader ended up reading a
    // normal map or an atlas page as if it were the sky.
    const TextureHandle reflection = assets.resolveRenderTarget(hashName(kReflectionTargetName));
    gpu.bindTexture(BindingReflection, reflection.valid() ? reflection : mFallbackBlack);
    gpu.bindTexture(BindingRefraction, hasRefraction ? refraction : mFallbackBlack);
    // Black reads as the near plane, so without a copy every pixel comes out
    // at zero depth - shallow everywhere, which switches the surface off
    // rather than leaving it wrong in some harder-to-read way.
    gpu.bindTexture(BindingRefractionDepth,
                    refractionDepth.valid() ? refractionDepth : mFallbackBlack);

    // Water is never the bulk of a scene's geometry, so this is a plain
    // one-model-per-draw loop - no run-merging like ForwardPass, which exists
    // for thousands of grouped instances, not a handful of water planes.
    for (const RenderPacket& packet : packets)
    {
        const RenderInstance& instance = frame.list->instance(packet.instance);
        const Mesh* mesh = assets.getMesh(instance.mesh);
        if (!mesh)
            continue;

        const glm::mat4 model = frame.list->models()[packet.instance];
        gpu.updateBuffer(mInstanceBuffer, 0, sizeof(glm::mat4), &model);
        gpu.bindStorage(BindingInstances, mInstanceBuffer);

        const Material& material = *instance.material;
        gpu.setPipeline(instance.pipeline);
        gpu.bindUniform(BindingMaterial, material.paramsBuffer);

        // The ripple noise is the one texture the surface owns itself, so it
        // comes off the material rather than from a target published upstream.
        const MaterialTexture& noise = material.textures[SlotAlbedo];
        gpu.bindTexture(BindingAlbedo, noise.texture.valid() ? noise.texture : mFallbackBlack,
                        noise.sampler);

        const SubMesh& submesh = mesh->submeshes[instance.submesh];
        DrawDesc draw;
        draw.vertexBuffers[0] = mesh->positionBuffer;
        draw.vertexBuffers[1] = mesh->attribBuffer;
        draw.vertexBufferCount = 2;
        draw.indexBuffer = mesh->indexBuffer;
        draw.indexType = mesh->indexType;
        draw.first = submesh.indexOffset;
        draw.count = submesh.indexCount;
        draw.instanceCount = 1;
        draw.firstInstance = 0;
        gpu.draw(draw);
    }
}

void WaterPass::shutdown()
{
    GPU& gpu = GPU::getSingleton();
    gpu.destroy(mCameraBuffer);
    gpu.destroy(mReflectionCameraBuffer);
    gpu.destroy(mWaterBuffer);
    gpu.destroy(mEnvironmentBuffer);
    gpu.destroy(mInstanceBuffer);
    if (mFallbackBlack.valid())
        gpu.destroy(mFallbackBlack);
}

} // namespace Radion
