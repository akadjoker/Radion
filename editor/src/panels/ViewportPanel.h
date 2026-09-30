#ifndef RADION_VIEWPORT_PANEL_H
#define RADION_VIEWPORT_PANEL_H

#include "EditorPanel.h"
#include "Engine.h"

namespace Radion
{
class GameObject;

class ViewportPanel final : public EditorPanel
{
public:
    explicit ViewportPanel(EditorApplication& app);

    void onImGui() override;
 
    // submeshIndex >= 0 frames just that submesh's world box.
    void focusOnObject(const GameObject& object, s32 submeshIndex = -1);

private:
    void updateNavigation();
    void drawSceneGizmos(GameObject& object);
    RenderTextureOutput mOutput;
    int mTool = 0;
    bool mSnap = false;
    bool mFastRender = false;
    bool mGrid = true;
    bool mPerspective = true;
    Math::vec3 mOrbitTarget = Math::vec3(0.0f);
    Math::vec3 mCameraPosition = Math::vec3(0.0f, 2.5f, 9.5f);
    Math::mat4 mEditorView = Math::mat4(1.0f);
    Math::mat4 mEditorProjection = Math::mat4(1.0f);
    float mOrbitDistance = 10.0f;
    float mOrbitYaw = 0.0f;
    float mOrbitPitch = -0.25f;
    bool mOrbiting = false;
    bool mPanning = false;
    bool mLooking = false;
    int mPointerNavigation = 0;
    bool mPointerNavigationActive = false;
    // Rubber-band selection: dragging on empty space with the Select tool.
    bool mRectSelecting = false;
    // Same drag with Ctrl held at start gathers the object's submeshes instead; never both.
    bool mSubmeshRectSelecting = false;
    Math::vec2 mRectStart = Math::vec2(0.0f);
    Math::vec2 mImageMin = Math::vec2(0.0f);
    Math::vec2 mImageMax = Math::vec2(0.0f);
    bool mNavigationGizmoActive = false;
    Math::vec2 mNavigationGizmoStartMouse = Math::vec2(0.0f);
    f32 mNavigationGizmoStartYaw = 0.0f;
    f32 mNavigationGizmoStartPitch = 0.0f;

    bool mTerrainPaintActive = false;
    bool mTerrainDeformActive = false;
    int mTerrainDeformMode = 0; // raise, lower, smooth, flatten
    int mTerrainPaintLayer = 0; // surface splat layer
    int mTerrainPaintChannel = 0; // vegetation channel
    bool mTerrainPaintVegetation = false;
    f32 mTerrainBrushRadius = 8.0f;
    f32 mTerrainBrushStrength = 2.0f;
    bool mTerrainStrokeUndo = false;

    void drawTransformGizmo(const Math::vec2& imageMin, const Math::vec2& imageSize);
    // Selects every object whose origin projects inside the screen rectangle; additive when `add` (Shift).
    void selectInRect(const Math::vec2& min, const Math::vec2& max, bool add);
    // Same rectangle over one object's submeshes (drag unprojected into a frustum); `subtract` removes from the selection.
    void selectSubmeshesInRect(GameObject& object, const Math::vec2& min, const Math::vec2& max,
                               bool subtract);
    // Record undo once on the press that starts a drag, not every frame Manipulate() reports a change.
    bool mGizmoDragging = false;

    // Pose-editing counterpart to drawTransformGizmo(): retargets ImGuizmo onto a bone or IK target per EditorApplication::animationPoseTarget().
    void drawBonePoseGizmo(const Math::vec2& imageMin, const Math::vec2& imageSize);
};

} // namespace Radion

#endif // RADION_VIEWPORT_PANEL_H
