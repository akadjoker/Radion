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
 
    // submeshIndex >= 0 frames just that submesh's world box instead of the
    // whole mesh's - the Inspector's per-submesh focus icon.
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
    Math::Vec3 mOrbitTarget = Math::Vec3(0.0f);
    Math::Vec3 mCameraPosition = Math::Vec3(0.0f, 2.5f, 9.5f);
    Math::Mat4 mEditorView = Math::Mat4(1.0f);
    Math::Mat4 mEditorProjection = Math::Mat4(1.0f);
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
    // The same drag with Ctrl held as it starts: gathers the selected
    // object's own submeshes instead. Never both at once.
    bool mSubmeshRectSelecting = false;
    Math::Vec2 mRectStart = Math::Vec2(0.0f);
    Math::Vec2 mImageMin = Math::Vec2(0.0f);
    Math::Vec2 mImageMax = Math::Vec2(0.0f);
    bool mNavigationGizmoActive = false;
    Math::Vec2 mNavigationGizmoStartMouse = Math::Vec2(0.0f);
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

    void drawTransformGizmo(const Math::Vec2& imageMin, const Math::Vec2& imageSize);
    // Selects every object whose origin projects inside the screen-space
    // rectangle. Additive when `add` (Shift held), otherwise replaces.
    void selectInRect(const Math::Vec2& min, const Math::Vec2& max, bool add);
    // Same rectangle, but over one object's own submeshes: the drag is
    // unprojected into a frustum and every submesh whose box meets it is
    // gathered. `subtract` removes those from the selection instead of
    // adding them - the way back from a rectangle that caught too much,
    // without starting the whole selection again.
    void selectSubmeshesInRect(GameObject& object, const Math::Vec2& min, const Math::Vec2& max,
                               bool subtract);
    // Undo has to be recorded once, on the press that starts a drag - not
    // every frame Manipulate() reports a change, or every frame of a single
    // drag becomes its own undo step.
    bool mGizmoDragging = false;

    // AnimationPanel's pose-editing counterpart to drawTransformGizmo() -
    // retargets the same ImGuizmo onto a bone (rotate, FK) or an IK chain's
    // target (translate) instead of the selected object's own transform,
    // driven by EditorApplication::animationPoseTarget().
    void drawBonePoseGizmo(const Math::Vec2& imageMin, const Math::Vec2& imageSize);
};

} // namespace Radion

#endif // RADION_VIEWPORT_PANEL_H
