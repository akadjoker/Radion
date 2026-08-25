#ifndef RADION_MESH_TOOLS_PANEL_H
#define RADION_MESH_TOOLS_PANEL_H

#include "EditorPanel.h"
#include "ImGuiFileDialog.h"
#include "Mesh.h"

#include <string>

namespace Radion
{

 
class MeshToolsPanel final : public EditorPanel
{
public:
    explicit MeshToolsPanel(EditorApplication& app);

    void onImGui() override;

private:
    u64 mCutterPlaneId = 0;
    bool mKeepPlaneNormalSide = false;
    std::string mCutStatus;
    std::string mOptimizeStatus;

       ImGuiFileDialog mSaveDialog;
    MeshHandle mSaveHandle; // which object's mesh the pending dialog is for

        enum class SaveKind
    {
        None,
        Mesh,
        Materials,
        Skeleton,
        Animation
    };
    SaveKind mSaveKind = SaveKind::None;

    std::string mSaveClipName;
};

} // namespace Radion

#endif // RADION_MESH_TOOLS_PANEL_H
