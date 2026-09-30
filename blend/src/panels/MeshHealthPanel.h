#ifndef RADION_MESH_HEALTH_PANEL_H
#define RADION_MESH_HEALTH_PANEL_H

#include "../BlenderPanel.h"
#include "AssetManager.h"

namespace Radion
{

// What the loaded mesh is made of and what is wrong with it, with a way to
// act on each fault. The analysis walks every triangle and hashes every
// position, so it runs when asked and when the mesh changes underneath it -
// not every frame.
class MeshHealthPanel : public BlenderPanel
{
public:
    MeshHealthPanel(BlenderApplication& app);
    ~MeshHealthPanel() override;

    void onImGui() override;

private:
    void refresh();
    void drawContents();
    void drawFault(bool bad, const char* label, const char* explanation);

    AssetManager::Diagnostics mDiagnostics;
    const MeshData* mAnalyzedMesh = nullptr;
    usize mAnalyzedVertexCount = 0;
    usize mAnalyzedIndexCount = 0;
    bool mAnalyzed = false;
    bool mAutoRefresh = true;
};

} // namespace Radion

#endif // RADION_MESH_HEALTH_PANEL_H
