#ifndef RADION_SETTINGS_PANEL_H
#define RADION_SETTINGS_PANEL_H

#include "EditorPanel.h"

namespace Radion
{
class SettingsPanel final : public EditorPanel
{
public:
    explicit SettingsPanel(EditorApplication& app);
    void onImGui() override;
};
} // namespace Radion

#endif
