#ifndef RADION_GAME_PANEL_H
#define RADION_GAME_PANEL_H
#include "EditorPanel.h"
#include "Engine.h"
namespace Radion
{
class GamePanel final : public EditorPanel
{
public:
    explicit GamePanel(EditorApplication& app);
    void onImGui() override;

private:
    RenderTextureOutput mOutput;
    int mResolutionPreset = 0;
};
} // namespace Radion
#endif
