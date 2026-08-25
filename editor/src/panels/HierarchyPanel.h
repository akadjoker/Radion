#ifndef RADION_HIERARCHY_PANEL_H
#define RADION_HIERARCHY_PANEL_H

#include "EditorPanel.h"

#include <glm/vec3.hpp>
#include <string>

namespace Radion
{
class GameObject;

// Drag payload for a Hierarchy row: the data is the dragged GameObject's id
// (u64, not null-terminated/typed - construct via
// *static_cast<const u64*>(payload->Data)). Whoever accepts it (another
// Hierarchy row for reparenting, InspectorPanel's BoneAttachment target slot)
// resolves it back through Scene::findGameObject().
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

    // A primitive picked from Create > 3D does not land in the tree right
    // away: the shape is queued here, the popup asks for its dimensions,
    // and only its OK button actually builds the mesh and creates the
    // GameObject - Cancel (or dismissing the popup) leaves the scene
    // untouched.
    struct PendingPrimitive
    {
        bool open = false;
        PrimitiveKind kind = PrimitiveKind::Cube;
        GameObject* parent = nullptr;
        glm::vec3 dimensions{1.0f};
        f32 uvTiles = 16.0f;
        int segmentsA = 0;
        int segmentsB = 0;
        // Hills only: the red channel becomes displacement (0..1 * height
        // scale, dimensions.z below) - dropped from Assets, same drag-drop
        // slot a material's own texture fields use.
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
    // With a filter typed the tree is replaced by a flat list of whatever
    // matches, anywhere in the hierarchy - a match six levels down is no use
    // if the six parents have to be expanded by hand to reach it.
    void drawFilteredList(GameObject& object, u32& shown);
    bool matchesSearch(const GameObject& object) const;
    void drawNode(GameObject& object);
    void drawPendingPrimitivePopup();
    void drawPendingTerrainPopup();
    void drawPendingOceanPopup();
    void drawPendingGridDuplicatePopup();
    void queuePendingPrimitive(PrimitiveKind kind, GameObject* parent);

    PendingPrimitive mPendingPrimitive;
    PendingTerrain mPendingTerrain;
    PendingOcean mPendingOcean;
    PendingGridDuplicate mPendingGridDuplicate;
    std::string mSearch;
    // Selection commits on mouse release, not press: a press that turns into
    // a drag must keep the current object in the Inspector, or its drop
    // targets disappear before the payload can land on them.
    u64 mPendingSelect = 0;
    // Drag across rows to select a run of them, the way a file list does -
    // the tree is where a batch is actually picked, so the band lives here
    // rather than over the 3D view.
    bool mDragSelecting = false;
};

} // namespace Radion

#endif // RADION_HIERARCHY_PANEL_H
