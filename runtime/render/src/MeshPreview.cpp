#include "PCH.h"

#include "MeshPreview.h"

#include "AssetManager.h"
#include "CameraBlock.h"
#include "EnvironmentBlock.h"
#include "Material.h"
#include "ShadowPass.h"

#include <glm/gtc/matrix_transform.hpp>

namespace Radion
{

namespace
{

// Mirrors lit.vert/unlit.vert's own InstanceData: the current and previous
// model matrices plus the palette offsets a skinned mesh would read. A
// preview never skins and never moves, so the offsets are zero and the
// previous matrix equals the current one - the fields still have to be here
// for the layout to match what the shader declares.
struct GPUInstance
{
    Math::Mat4 model = Math::Mat4(1.0f);
    Math::Mat4 prevModel = Math::Mat4(1.0f);
    u32 paletteOffset = 0;
    u32 prevPaletteOffset = 0;
    u32 padding[2] = {0, 0};
};

// Three vertices covering the screen from gl_VertexIndex, so the resolve needs
// no vertex buffer at all.
constexpr char kResolveVertex[] = R"GLSL(#version 450 core
layout(location = 0) out vec2 uv;
void main()
{
    vec2 position = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    uv = position;
    gl_Position = vec4(position * 2.0 - 1.0, 0.0, 1.0);
}
)GLSL";

// The same two operations the post stack's last step applies, with the same
// constants (see kPostFragment in PostProcess.cpp). They have to match: a
// preview that tonemapped differently from the scene would be a different
// answer to "what does this look like", which is the only question it exists
// to answer.
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

    // RGBA16F for the draw: a Lit shader writes linear HDR, and clamping that
    // to 8 bits before the tonemap loses the highlights the tonemap is for.
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

// Draws with the material's own pipeline, which is the point - the preview
// shows the same shader the scene does, not an approximation of it. But it
// lights the mesh ITSELF: camera, environment and shadow block are all its
// own, so the result is the same whatever the scene's sun is doing.
//
// What it still borrows from the frame: the entity/tile SSBOs and the decal
// arrays, which a Lit fragment shader declares statically and which fail every
// draw when left unbound. Binding those here would be a second copy of
// ForwardPass, so this relies on the forward pass having bound them earlier in
// the same frame - call this AFTER the scene has been rendered, never before.
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

    // Frame the mesh by its own bounds rather than a fixed distance: the
    // generator's trees range from a knee-high shrub to a sequoia, and one
    // distance cannot suit both.
    const Math::Vec3 mathCentre = (mesh->bounds.min + mesh->bounds.max) * 0.5f;
    const Math::Vec3 centre(mathCentre.x, mathCentre.y, mathCentre.z);
    const f32 radius = glm::max(mesh->bounds.radius(), 0.001f);
    const f32 fieldOfView = glm::radians(40.0f);
    const f32 distance = radius / glm::tan(fieldOfView * 0.5f) * 1.15f;

    const Math::Vec3 eye = centre + Math::Vec3(glm::cos(pitch) * glm::sin(yaw) * distance,
                                             glm::sin(pitch) * distance,
                                             glm::cos(pitch) * glm::cos(yaw) * distance);

    const f32 aspect = static_cast<f32>(mScene.width) / static_cast<f32>(mScene.height);
    const Math::Mat4 view = glm::lookAt(eye, centre, Math::Vec3(0.0f, 1.0f, 0.0f));
    const Math::Mat4 projection =
        glm::perspective(fieldOfView, aspect, distance - radius * 1.5f, distance + radius * 2.0f);

    CameraBlock camera;
    camera.viewProj = projection * view;
    camera.clipPlane = Math::Vec4(0.0f);
    camera.cameraPos = Math::Vec4(eye, 1.0f);
    camera.view = view;
    gpu.updateBuffer(mCameraBuffer, 0, sizeof(camera), &camera);

    const TemporalCameraBlock temporal{camera.viewProj, camera.viewProj};
    gpu.updateBuffer(mTemporalBuffer, 0, sizeof(temporal), &temporal);

    const GPUInstance instance;
    gpu.updateBuffer(mInstanceBuffer, 0, sizeof(instance), &instance);

    // A headlight, aimed slightly off the view axis so the shape still reads
    // instead of going flat, plus a generous ambient. This is a "what does the
    // mesh look like" view, not a lighting rehearsal - it should look the same
    // whatever time of day the scene is at.
    EnvironmentBlock environment;
    const Math::Vec3 toCentre = glm::normalize(centre - eye);
    const Math::Vec3 right = glm::normalize(glm::cross(toCentre, Math::Vec3(0.0f, 1.0f, 0.0f)));
    environment.sunDirection =
        Math::Vec4(glm::normalize(toCentre + right * 0.35f - Math::Vec3(0.0f, 0.45f, 0.0f)), 0.0f);
    environment.sunColor = Math::Vec4(1.0f, 0.98f, 0.94f, 1.0f);
    environment.ambient = Math::Vec4(0.42f, 0.44f, 0.48f, 1.0f);
    gpu.updateBuffer(mEnvironmentBuffer, 0, sizeof(environment), &environment);

    // directionAndCount.w = 0 cascades: lit.frag returns unshadowed outright
    // (see its ShadowFactor, `if (uCascadeCount <= 0)`). The scene's cascades
    // are fitted to the main camera's frustum, so a mesh drawn here at the
    // origin fell outside them and read as fully in shadow - which is what
    // made the preview black.
    DirectionalShadowBlock shadow;
    shadow.directionAndCount = Math::Vec4(Math::Vec3(environment.sunDirection), 0.0f);
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

        // The same four slots ForwardPass binds, and for the same reason: the
        // pipeline was compiled with HAS_ALBEDO/HAS_NORMAL/... according to
        // which of these the material actually carries, so a slot the shader
        // samples has to have something in it.
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

    // Tonemap and gamma-encode into the target ImGui actually samples. The
    // scene gets this from the post stack; a preview drawing straight to an
    // 8-bit target skipped it entirely, which is why it came out dark - linear
    // 0.5 shows as roughly 0.21 once the display applies its own gamma.
    gpu.setTarget(mResolved.target);
    gpu.setViewport(viewport);
    gpu.setPipeline(mResolvePipeline);
    gpu.bindTexture(0, mScene.color);

    DrawDesc resolve;
    resolve.count = 3;
    resolve.instanceCount = 1;
    gpu.draw(resolve);

    // Back to the screen. Not optional: the target is global state, and
    // whatever draws next - the ImGui pass, in practice - would otherwise land
    // inside this texture instead of the window. ImGui's own backend sets its
    // viewport from the display size, so the target alone is enough.
    gpu.setTarget(TargetHandle());
}

} // namespace Radion
