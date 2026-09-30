#ifndef RADION_HIERARCHY_PANEL_H
#define RADION_HIERARCHY_PANEL_H

#include "EditorPanel.h"
#include "ImGuiFileDialog.h"

#include "Math.h"
#include <string>

namespace Radion
{
class GameObject;

// Drag payload for a Hierarchy row: the dragged GameObject's id as u64 (*static_cast<const u64*>(payload->Data)); resolve via Scene::findGameObject().
constexpr const char* kGameObjectDragPayload = "RADION_GAME_OBJECT";

class HierarchyPanel final : public EditorPanel
{
public:
    explicit HierarchyPanel(EditorApplication& app);

    void onImGui() override;

private:
    enum class PrimitiveKind : u8
    {
        Cube,
        Sphere,
        Plane,
        Cylinder,
        Cone,
        Capsule,
        Torus,
        Hills
    };

    // Primitives are queued here; the popup asks for dimensions and only OK creates the GameObject.
    struct PendingPrimitive
    {
        bool open = false;
        PrimitiveKind kind = PrimitiveKind::Cube;
        GameObject* parent = nullptr;
        Math::vec3 dimensions{1.0f};
        f32 uvTiles = 16.0f;
        int segmentsA = 0;
        int segmentsB = 0;
        // Hills only: the red channel becomes displacement (0..1 * height scale, dimensions.z).
        std::string heightmapFile;
        f32 heightScale = 5.0f;
    };
    struct PendingTerrain
    {
        bool open = false;
        GameObject* parent = nullptr;
        std::string heightmapFile;
        f32 cellSize = 1.0f;
        f32 heightScale = 32.0f;
        f32 uvTiles = 1.0f;
        int maxLod = 6;
    };
    struct PendingOcean
    {
        bool open = false;
        GameObject* parent = nullptr;
        f32 size = 100.0f;
        int segments = 128;
        f32 level = 0.0f;
        int quality = 1;
    };
    struct PendingGridDuplicate
    {
        bool open = false;
        u64 source = 0;
        int mode = 0; // 0 = grid X/Z, 1 = line X, 2 = line Z
        int countX = 10;
        int countZ = 10;
        f32 spacing = 1.0f;
    };

    void drawCreateMenu(GameObject* parent);
    void drawObjectActions();
    void drawSearchField();
    // With a filter typed, the tree becomes a flat list of matches anywhere in the hierarchy.
    void drawFilteredList(GameObject& object, u32& shown);
    bool matchesSearch(const GameObject& object) const;
    void drawNode(GameObject& object);
    void drawPendingPrimitivePopup();
    void drawPendingTerrainPopup();
    void drawPendingOceanPopup();
    void drawPendingGridDuplicatePopup();
    void queuePendingPrimitive(PrimitiveKind kind, GameObject* parent);

    // mSavePrefabSource is an object id looked up again on Accept, not a raw pointer: the object may be deleted while the dialog is open.
    void drawSavePrefabPopup();
    ImGuiFileDialog mSavePrefabDialog;
    u64 mSavePrefabSource = 0;

    PendingPrimitive mPendingPrimitive;
    PendingTerrain mPendingTerrain;
    PendingOcean mPendingOcean;
    PendingGridDuplicate mPendingGridDuplicate;
    std::string mSearch;
    // Selection commits on release, not press, so a press that becomes a drag keeps the Inspector's drop targets alive.
    u64 mPendingSelect = 0;
    // Drag across rows to select a run; lives here rather than over the 3D view.
    bool mDragSelecting = false;
};

} // namespace Radion

#endif // RADION_HIERARCHY_PANEL_H
