#ifndef RADION_VIEWPORT_PANEL_H
#define RADION_VIEWPORT_PANEL_H

#include "../BlenderPanel.h"
#include "MiniRenderer.h"
#include "ViewportCamera.h"
#include "Types.h"

#include "Math.h"
#include <array>
#include <vector>

namespace Radion
{

struct MeshData;
class BlenderSelection;

class ViewportPanel : public BlenderPanel
{
public:
    ViewportPanel(BlenderApplication& app);
    ~ViewportPanel() override;

    void onImGui() override;

    using ViewMode = CameraView;

    enum class LayoutMode : u8
    {
        Single,
        ThreeWay,
        FourWay
    };

    LayoutMode layoutMode() const
    {
        return mLayoutMode;
    }
    void setLayoutMode(LayoutMode mode)
    {
        mLayoutMode = mode;
    }

    ViewMode viewMode(usize index) const
    {
        return index < mViewModes.size() ? mViewModes[index] : ViewMode::Perspective;
    }
    void setViewMode(usize index, ViewMode mode);

private:
    void drawSingleView();
    void drawThreeWayLayout();
    void drawFourWayLayout();
    void drawViewportWindow(usize index, const char* name, ViewMode mode);

    void drawViewportControls();
    void drawViewModeMenu(usize viewportIndex);
    void drawToolbar();

    using CameraState = ::Radion::CameraState;

    // One offscreen colour+depth target per viewport - MiniRenderer draws
    // into this, ImGui::Image() then displays it inside the docked panel.
    // A docked panel's own screen rect is only known after ImGui's layout
    // runs, so the 3D content cannot go straight to the backbuffer the way
    // MiniRenderer's standalone test does.
    struct RenderTarget
    {
        u32 fbo = 0;
        u32 colorTexture = 0;
        u32 depthRenderbuffer = 0;
        s32 width = 0;
        s32 height = 0;

        bool ensure(s32 requestedWidth, s32 requestedHeight);
        void destroy();
    };

    void updateCameraNavigation(usize index, CameraState& camera, ViewMode mode);
    void computeMatrices(const CameraState& camera, ViewMode mode, f32 aspect, Math::mat4& view,
                         Math::mat4& projection, Math::vec3& cameraPos) const;
    void drawNavigationGizmo(CameraState& camera, const Math::mat4& view, const Math::vec2& imageMin,
                             const Math::vec2& imageSize);
    void drawSelectionOverlay(const MeshData* mesh, const Math::mat4& viewProjection);
    void drawDebugVectorOverlay(const MeshData* mesh, const Math::mat4& viewProjection);
    void drawSkeletonOverlay(const Math::mat4& viewProjection);
    void updateSelectionInput(usize index, const MeshData* mesh, const Math::mat4& view,
                              const Math::mat4& projection, const Math::vec3& cameraPos,
                              const Math::vec2& imageMin, const Math::vec2& imageSize,
                              const RenderTarget& target);
    void handleToolShortcuts();
    void uploadVertexSelection(const MeshData& mesh, const BlenderSelection& selection);
    void drawTransformGizmo(usize index, const MeshData* mesh, const Math::mat4& view,
                            const Math::mat4& projection, const Math::vec2& imageMin,
                            const Math::vec2& imageSize, bool orthographic);

    // One depth read covering the whole area a selection needs to test,
    // instead of one glReadPixels per candidate. Each of those binds the FBO
    // and stalls the pipeline waiting for the GPU, so a box select over a
    // large mesh used to cost one full sync per vertex inside the box.
    struct DepthRect
    {
        std::vector<f32> depth;
        s32 x = 0;
        s32 y = 0;
        s32 width = 0;
        s32 height = 0;
    };
    static bool readDepthRect(const RenderTarget& target, const Math::vec2& localMin,
                              const Math::vec2& localMax, DepthRect& out);
    static bool isScreenPointVisible(const DepthRect& depthRect, const Math::vec2& localPoint,
                                     const Math::vec3& worldPos,
                                     const Math::mat4& inverseViewProjection,
                                     const Math::vec3& cameraPos, const RenderTarget& target);

    LayoutMode mLayoutMode = LayoutMode::Single;
    std::array<CameraState, 4> mCameras;
    std::array<RenderTarget, 4> mTargets;
    std::array<ViewMode, 4> mViewModes = {
        {ViewMode::Perspective, ViewMode::Top, ViewMode::Front, ViewMode::Right}
    };

    // Only one viewport is ever dragged at a time - which one owns the drag
    // that started it, so releasing the mouse over a different viewport's
    // region (or none) still ends it cleanly.
    s32 mActiveViewport = -1;
    bool mOrbiting = false;
    bool mPanning = false;
    bool mLooking = false;

    s32 mBoxSelectViewport = -1;
    bool mBoxSelecting = false;
    Math::vec2 mBoxSelectStart = Math::vec2(0.0f);

    // Toolbar state. Only Grid is actually wired to a visible effect right
    // now - Move/Rotate/Scale have no gizmo to drive yet, and Snap has no
    // transform to snap. They exist because the toolbar is one row, not
    // three separate features arriving separately.
    enum class Tool : u8
    {
        Select,
        Move,
        Rotate,
        Scale
    };
    Tool mTool = Tool::Select;
    bool mSnap = false;

    // ImGuizmo keeps one set of state per frame, so only one viewport may
    // draw a gizmo - the hovered one, or whichever owns a drag in progress.
    s32 mGizmoViewport = -1;
    bool mGizmoDragging = false;
    Math::mat4 mGizmoMatrix = Math::mat4(1.0f);
    Math::mat4 mGizmoStartMatrix = Math::mat4(1.0f);
    bool mShowGrid = true;
    bool mSelectVisibleOnly = false;

    // Scratch for the vertex selection stream, and the revision it was built
    // from - the upload only happens when the selection actually changed.
    std::vector<u8> mVertexSelectionFlags;
    // One byte per vertex: 1 while the running gizmo drag is moving it.
    std::vector<u8> mSnapMoving;
    u64 mUploadedSelectionRevision = 0;
    u64 mUploadedMeshRevision = 0;
    u64 mUploadedHiddenRevision = ~u64(0);
    const MeshData* mUploadedSelectionMesh = nullptr;
    MiniRenderMode mShadingMode = MiniRenderMode::Solid;
    MiniDebugView mDebugView = MiniDebugView::None;
    bool mUnlit = false;
    bool mShowSkeleton = false;
};

} // namespace Radion

#endif // RADION_VIEWPORT_PANEL_H
