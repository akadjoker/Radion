#ifndef RADION_HIERARCHY_PANEL_H
#define RADION_HIERARCHY_PANEL_H

#include "../BlenderPanel.h"

namespace Radion
{

class HierarchyPanel : public BlenderPanel
{
public:
    HierarchyPanel(BlenderApplication& app);
    ~HierarchyPanel() override;

    void onImGui() override;

private:
    void drawBoneTree();
    void drawObjectTree();

    bool mShowBones = true;
};

} // namespace Radion

#endif // RADION_HIERARCHY_PANEL_H
