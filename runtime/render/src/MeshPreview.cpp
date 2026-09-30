#include "PCH.h"

#include "MeshPreview.h"

#include "AssetManager.h"
#include "CameraBlock.h"
#include "EnvironmentBlock.h"
#include "Material.h"
#include "ShadowPass.h"

#include "Math.h"

namespace Radion
{

namespace
{

// Mirrors lit.vert/unlit.vert InstanceData; a preview never skins or moves, so offsets are zero and previous == current.
struct GPUInstance
{
    Math::mat4 model = Math::mat4(1.0f);
    Math::mat4 prevModel = Math::mat4(1.0f);
    u32 paletteOffset = 0;
    u32 prevPaletteOffset = 0;
    u32 padding[2] = {0, 0};
};

constexpr char kResolveVertex[] = R"GLSL(#version 450 core
layout(location = 0) out vec2 uv;
void main()
{
    vec2 position = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    uv = position;
    gl_Position = vec4(position * 2.0 - 1.0, 0.0, 1.0);
}
)GLSL";

// Same operations and constants as the post stack's last step (kPostFragment in PostProcess.cpp); they must match.
constexpr char kResolveFragment[] = R"GLSL(#version 450 core
layout(binding = 0) uniform sampler2D sourceTexture;
layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outColor;

vec3 aces(vec3 x)
{
    return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
}

void main()
{
    vec3 color = texture(sourceTexture, uv).rgb;
    color = aces(color);
    color = pow(clamp(color, 0.0, 1.0), vec3(1.0 / 2.2));
    outColor = vec4(color, 1.0);
}
)GLSL";

} // namespace

bool MeshPreview::create(u32 width, u32 height)
{
    destroy();

    // RGBA16F: Lit writes linear HDR; clamping to 8 bits before tonemap loses highlights.
    if (!mScene.create(width, height, Format::RGBA16F, Format::Depth24, "preview.scene"))
        return false;
    if (!mResolved.create(width, height, Format::RGBA8, Format::Unknown, "preview.resolved"))
    {
        destroy();
        return false;
    }

    GPU& gpu = GPU::getSingleton();

    BufferDesc cameraDesc;
    cameraDesc.size = sizeof(CameraBlock);
    cameraDesc.usage = BufferUniform;
    cameraDesc.residency = Residency::Dynamic;
    cameraDesc.debugName = "preview.camera";
    mCameraBuffer = gpu.createBuffer(cameraDesc);

    BufferDesc temporalDesc;
    temporalDesc.size = sizeof(TemporalCameraBlock);
    temporalDesc.usage = BufferUniform;
    temporalDesc.residency = Residency::Dynamic;
    temporalDesc.debugName = "preview.temporal";
    mTemporalBuffer = gpu.createBuffer(temporalDesc);

    BufferDesc instanceDesc;
    instanceDesc.size = sizeof(GPUInstance);
    instanceDesc.usage = BufferStorage;
    instanceDesc.residency = Residency::Dynamic;
    instanceDesc.stride = sizeof(GPUInstance);
    instanceDesc.debugName = "preview.instance";
    mInstanceBuffer = gpu.createBuffer(instanceDesc);

    BufferDesc environmentDesc;
    environmentDesc.size = sizeof(EnvironmentBlock);
    environmentDesc.usage = BufferUniform;
    environmentDesc.residency = Residency::Dynamic;
    environmentDesc.debugName = "preview.environment";
    mEnvironmentBuffer = gpu.createBuffer(environmentDesc);

    BufferDesc shadowDesc;
    shadowDesc.size = sizeof(DirectionalShadowBlock);
    shadowDesc.usage = BufferUniform;
    shadowDesc.residency = Residency::Dynamic;
    shadowDesc.debugName = "preview.shadow";
    mShadowBuffer = gpu.createBuffer(shadowDesc);

    if (!mCameraBuffer.valid() || !mTemporalBuffer.valid() || !mInstanceBuffer.valid() ||
        !mEnvironmentBuffer.valid() ||
        !mShadowBuffer.valid() || !ensureResolvePipeline())
    {
        destroy();
        return false;
    }
    return true;
}

bool MeshPreview::ensureResolvePipeline()
{
    if (mResolvePipeline.valid())
        return true;

    PipelineDesc desc;
    desc.vs = {kResolveVertex, 0, "preview.resolve.vert"};
    desc.fs = {kResolveFragment, 0, "preview.resolve.frag"};
    desc.depth.test = false;
    desc.depth.write = false;
    desc.raster.cull = CullMode::None;
    desc.debugName = "preview.resolve";
    mResolvePipeline = GPU::getSingleton().createPipeline(desc);
    return mResolvePipeline.valid();
}

void MeshPreview::destroy()
{
    GPU& gpu = GPU::getSingleton();
    gpu.destroy(mCameraBuffer);
    gpu.destroy(mTemporalBuffer);
    gpu.destroy(mInstanceBuffer);
    gpu.destroy(mEnvironmentBuffer);
    gpu.destroy(mShadowBuffer);
    gpu.destroy(mResolvePipeline);
    mCameraBuffer = BufferHandle();
    mTemporalBuffer = BufferHandle();
    mInstanceBuffer = BufferHandle();
    mEnvironmentBuffer = BufferHandle();
    mShadowBuffer = BufferHandle();
    mResolvePipeline = PipelineHandle();
    mScene.destroy();
    mResolved.destroy();
}

u32 MeshPreview::textureId() const
{
    return mResolved.color.valid() ? GPU::getSingleton().nativeTextureId(mResolved.color) : 0u;
}

// Uses the material's own pipeline and its own camera/environment/shadow blocks, but borrows the entity/tile SSBOs and decal arrays bound by ForwardPass: call AFTER the scene has rendered.
void MeshPreview::render(MeshHandle handle, const Material* materials, u32 materialCount, f32 yaw,
                         f32 pitch)
{
    if (!valid())
        return;

    AssetManager& assets = Assets();
    const Mesh* mesh = assets.getMesh(handle);
    if (!mesh || mesh->submeshes.empty())
        return;

    GPU& gpu = GPU::getSingleton();

    // Frame by the mesh's own bounds: sizes range from shrub to sequoia.
    const Math::vec3 centre = (mesh->bounds.min + mesh->bounds.max) * 0.5f;
    const f32 radius = Math::max(mesh->bounds.radius(), 0.001f);
    const f32 fieldOfView = Math::radians(40.0f);
    const f32 distance = radius / Math::tan(fieldOfView * 0.5f) * 1.15f;

    const Math::vec3 eye = centre + Math::vec3(Math::cos(pitch) * Math::sin(yaw) * distance,
                                             Math::sin(pitch) * distance,
                                             Math::cos(pitch) * Math::cos(yaw) * distance);

    const f32 aspect = static_cast<f32>(mScene.width) / static_cast<f32>(mScene.height);
    const Math::mat4 view = Math::lookAt(eye, centre, Math::vec3(0.0f, 1.0f, 0.0f));
    const Math::mat4 projection =
        Math::perspective(fieldOfView, aspect, distance - radius * 1.5f, distance + radius * 2.0f);

    CameraBlock camera;
    camera.viewProj = projection * view;
    camera.clipPlane = Math::vec4(0.0f);
    camera.cameraPos = Math::vec4(eye, 1.0f);
    camera.view = view;
    gpu.updateBuffer(mCameraBuffer, 0, sizeof(camera), &camera);

    const TemporalCameraBlock temporal{camera.viewProj, camera.viewProj};
    gpu.updateBuffer(mTemporalBuffer, 0, sizeof(temporal), &temporal);

    const GPUInstance instance;
    gpu.updateBuffer(mInstanceBuffer, 0, sizeof(instance), &instance);

    // Headlight aimed slightly off-axis plus generous ambient, independent of scene time of day.
    EnvironmentBlock environment;
    const Math::vec3 toCentre = Math::normalize(centre - eye);
    const Math::vec3 right = Math::normalize(Math::cross(toCentre, Math::vec3(0.0f, 1.0f, 0.0f)));
    environment.sunDirection =
        Math::vec4(Math::normalize(toCentre + right * 0.35f - Math::vec3(0.0f, 0.45f, 0.0f)), 0.0f);
    environment.sunColor = Math::vec4(1.0f, 0.98f, 0.94f, 1.0f);
    environment.ambient = Math::vec4(0.42f, 0.44f, 0.48f, 1.0f);
    gpu.updateBuffer(mEnvironmentBuffer, 0, sizeof(environment), &environment);

    // 0 cascades: lit.frag returns unshadowed. Scene cascades fit the main camera, so a mesh at the origin read fully shadowed.
    DirectionalShadowBlock shadow;
    shadow.directionAndCount = Math::vec4(Math::vec3(environment.sunDirection), 0.0f);
    gpu.updateBuffer(mShadowBuffer, 0, sizeof(shadow), &shadow);

    ClearValue clear;
    clear.bits = ClearColor | ClearDepth;
    clear.color[0] = 0.10f;
    clear.color[1] = 0.11f;
    clear.color[2] = 0.13f;
    clear.color[3] = 1.0f;
    gpu.setTarget(mScene.target, clear);

    Viewport viewport;
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = static_cast<f32>(mScene.width);
    viewport.height = static_cast<f32>(mScene.height);
    gpu.setViewport(viewport);

    gpu.bindUniform(BindingCamera, mCameraBuffer);
    gpu.bindUniform(BindingTemporal, mTemporalBuffer);
    gpu.bindUniform(BindingEnvironment, mEnvironmentBuffer);
    gpu.bindUniform(BindingDirectionalShadow, mShadowBuffer);
    gpu.bindStorage(BindingInstances, mInstanceBuffer);

    for (const SubMesh& submesh : mesh->submeshes)
    {
        const Material* material = nullptr;
        if (materials && submesh.materialSlot < materialCount)
            material = &materials[submesh.materialSlot];
        else if (submesh.materialSlot < mesh->materials.size())
            material = &mesh->materials[submesh.materialSlot];
        if (!material || !material->pipeline.valid())
            continue;

        gpu.setPipeline(material->pipeline);
        if (material->paramsBuffer.valid())
            gpu.bindUniform(BindingMaterial, material->paramsBuffer);

        // Same four slots as ForwardPass: the pipeline was compiled with HAS_* per carried slot, so each sampled slot needs a binding.
        const MaterialTexture& albedo = material->textures[SlotAlbedo];
        if (albedo.texture.valid())
            gpu.bindTexture(BindingAlbedo, albedo.texture, albedo.sampler);
        const MaterialTexture& detail = material->textures[SlotDetail];
        if (detail.texture.valid())
            gpu.bindTexture(BindingDetail, detail.texture, detail.sampler);
        const MaterialTexture& normal = material->textures[SlotNormal];
        if (normal.texture.valid())
            gpu.bindTexture(BindingNormal, normal.texture, normal.sampler);
        const MaterialTexture& surface = material->textures[SlotSurface];
        if (surface.texture.valid())
            gpu.bindTexture(BindingSurface, surface.texture, surface.sampler);

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

    // Tonemap and gamma-encode into the target ImGui samples; linear 0.5 would show as ~0.21 after display gamma.
    gpu.setTarget(mResolved.target);
    gpu.setViewport(viewport);
    gpu.setPipeline(mResolvePipeline);
    gpu.bindTexture(0, mScene.color);

    DrawDesc resolve;
    resolve.count = 3;
    resolve.instanceCount = 1;
    gpu.draw(resolve);

    // Back to the screen: the target is global state, else the ImGui pass would draw into this texture.
    gpu.setTarget(TargetHandle());
}

} // namespace Radion
