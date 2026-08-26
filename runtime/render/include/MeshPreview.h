#ifndef RADION_MESH_PREVIEW_H
#define RADION_MESH_PREVIEW_H

#include "Mesh.h"
#include "OffscreenTarget.h"

#include "Math.h"

namespace Radion
{

struct Material;

// Renders one mesh, alone, into an offscreen target an editor panel can show
// with ImGui::Image. Without it, changing one of twenty generator parameters
// meant travelling to a real instance in the scene to see what it did.
//
// Deliberately not a RenderTechnique: it does not belong to the frame's pass
// list, has no FrameContext, and runs when a panel asks rather than once per
// frame in a fixed order. It draws with the material's own pipeline, so what
// shows is the same shader the scene uses, not an approximation of it.
class MeshPreview
{
public:
    bool create(u32 width, u32 height);
    void destroy();

    bool valid() const
    {
        return mScene.valid() && mResolved.valid();
    }

    // Frames the mesh by its bounding sphere and orbits `yaw` radians around
    // it. `materials` is indexed by SubMesh::materialSlot, the same way
    // RenderList::submit() takes overrides; a slot without one falls back to
    // the mesh's own material.
    void render(MeshHandle mesh, const Material* materials, u32 materialCount, f32 yaw,
                f32 pitch = 0.35f);

    // What ImGui::Image needs: the backend's own id, not our handle. Zero
    // until create() has succeeded. The image is bottom-up, so a panel has to
    // pass uv0=(0,1), uv1=(1,0).
    u32 textureId() const;

    TextureHandle texture() const
    {
        return mResolved.color;
    }

private:
    bool ensureResolvePipeline();

    // Two targets, for the same reason the scene has two: a Lit shader writes
    // linear HDR, and showing that straight in a panel is the washed-out dark
    // image gamma always gives. mScene takes the draw, mResolved takes the
    // tonemapped, gamma-encoded copy - which is what ImGui shows.
    OffscreenTarget mScene;
    OffscreenTarget mResolved;

    BufferHandle mCameraBuffer;
    // The vertex shaders shared with the scene compute a motion vector from
    // this block. A preview is a still frame, so both matrices hold the same
    // camera - leaving the block unbound would divide by a zero w instead.
    BufferHandle mTemporalBuffer;
    BufferHandle mInstanceBuffer;

    // Its own lighting, not the frame's. Inheriting the scene's left a preview
    // that was dark or half-lit depending on where the camera happened to be
    // standing, and fully shadowed whenever the mesh fell outside the
    // cascades the main camera's frustum fitted.
    BufferHandle mEnvironmentBuffer;
    BufferHandle mShadowBuffer;

    PipelineHandle mResolvePipeline;
};

} // namespace Radion

#endif // RADION_MESH_PREVIEW_H
