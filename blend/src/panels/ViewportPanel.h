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

    // Offscreen colour+depth target per viewport: the panel's rect is only known after ImGui layout, so MiniRenderer can't draw to the backbuffer.
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

    // One depth read for the whole selection area instead of a glReadPixels per candidate (each stalls the pipeline on the GPU).
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

    // Only one viewport drags at a time; the owner ends the drag even if the mouse is released over another viewport.
    s32 mActiveViewport = -1;
    bool mOrbiting = false;
    bool mPanning = false;
    bool mLooking = false;

    s32 mBoxSelectViewport = -1;
    bool mBoxSelecting = false;
    Math::vec2 mBoxSelectStart = Math::vec2(0.0f);

    // Toolbar state; only Grid has a visible effect so far (no gizmo for Move/Rotate/Scale, nothing for Snap to snap).
    enum class Tool : u8
    {
        Select,
        Move,
        Rotate,
        Scale
    };
    Tool mTool = Tool::Select;
    bool mSnap = false;

    // ImGuizmo keeps one state set per frame: only the hovered viewport, or the one owning a drag, draws a gizmo.
    s32 mGizmoViewport = -1;
    bool mGizmoDragging = false;
    Math::mat4 mGizmoMatrix = Math::mat4(1.0f);
    Math::mat4 mGizmoStartMatrix = Math::mat4(1.0f);
    bool mShowGrid = true;
    bool mSelectVisibleOnly = false;

    // Scratch for the vertex selection stream and the revision it was built from; upload only on change.
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
