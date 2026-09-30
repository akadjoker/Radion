#ifndef RADION_MESH_PREVIEW_H
#define RADION_MESH_PREVIEW_H

#include "Mesh.h"
#include "OffscreenTarget.h"

#include "Math.h"

namespace Radion
{

struct Material;

// Renders one mesh alone into an offscreen target for an editor panel (ImGui::Image).
// Not a RenderTechnique: no FrameContext, runs when a panel asks. Uses the material's own pipeline so it matches the scene's shader.
class MeshPreview
{
public:
    bool create(u32 width, u32 height);
    void destroy();

    bool valid() const
    {
        return mScene.valid() && mResolved.valid();
    }

    // Frames the mesh by bounding sphere, orbiting `yaw` radians. `materials` is indexed by SubMesh::materialSlot; a slot without one uses the mesh's own.
    void render(MeshHandle mesh, const Material* materials, u32 materialCount, f32 yaw,
                f32 pitch = 0.35f);

    // Backend texture id for ImGui::Image; zero until create() succeeds. The image is bottom-up: pass uv0=(0,1), uv1=(1,0).
    u32 textureId() const;

    TextureHandle texture() const
    {
        return mResolved.color;
    }

private:
    bool ensureResolvePipeline();

    // mScene takes the linear HDR draw; mResolved takes the tonemapped, gamma-encoded copy that ImGui shows.
    OffscreenTarget mScene;
    OffscreenTarget mResolved;

    BufferHandle mCameraBuffer;
    // Shared vertex shaders read a motion vector from this block; a still preview holds the same camera in both matrices (unbound would divide by zero w).
    BufferHandle mTemporalBuffer;
    BufferHandle mInstanceBuffer;

    // Own lighting, not the frame's: inheriting the scene's made previews dark or fully shadowed depending on the camera.
    BufferHandle mEnvironmentBuffer;
    BufferHandle mShadowBuffer;

    PipelineHandle mResolvePipeline;
};

} // namespace Radion

#endif // RADION_MESH_PREVIEW_H
