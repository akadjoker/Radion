#ifndef RADION_PROFILER_PANEL_H
#define RADION_PROFILER_PANEL_H

#include "EditorPanel.h"

namespace Radion
{
class ProfilerPanel final : public EditorPanel
{
public:
    explicit ProfilerPanel(EditorApplication& app) : EditorPanel("Profiler", app) {}
    void onImGui() override;
};
} // namespace Radion

#endif // RADION_PROFILER_PANEL_H
