#ifndef RADION_PROPERTIES_PANEL_H
#define RADION_PROPERTIES_PANEL_H

#include "../BlenderPanel.h"
#include "Types.h"

#include <glm/vec2.hpp>

namespace Radion
{

class PropertiesPanel : public BlenderPanel
{
public:
    PropertiesPanel(BlenderApplication& app);
    ~PropertiesPanel() override;

    void onImGui() override;

private:
    void drawMeshInfo();
    void drawFaceUVTools();
    void drawSubmeshList();

    // Held between frames so the amounts can be dialled in before anything
    // touches the mesh: every apply is a fresh edit on top of the last, and
    // an undo step of its own.
    glm::vec2 mUVScale = glm::vec2(1.0f);
    glm::vec2 mUVOffset = glm::vec2(0.0f);
    f32 mUVRotation = 0.0f;
};

} // namespace Radion

#endif // RADION_PROPERTIES_PANEL_H
