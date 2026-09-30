#ifndef RADION_CONSOLE_PANEL_H
#define RADION_CONSOLE_PANEL_H

#include "../BlenderPanel.h"
#include "Log.h"

#include <string>
#include <vector>

namespace Radion
{

class ConsolePanel : public BlenderPanel
{
public:
    explicit ConsolePanel(BlenderApplication& app);
    ~ConsolePanel() override;

    void onImGui() override;

    // Log has one sink; BlenderApplication owns the registration and calls this.
    static void pushEntry(LogLevel level, const char* message);

private:
    struct Entry
    {
        LogLevel level;
        std::string text;
    };

    // Shared by all instances (the sink has no `this`); capped in pushEntry().
    static std::vector<Entry> sEntries;
    static constexpr usize kMaxEntries = 2000;

    bool mAutoScroll = true;
    bool mShowInfo = true;
    bool mShowWarning = true;
    bool mShowError = true;
    bool mShowDebug = false;
};

} // namespace Radion

#endif // RADION_CONSOLE_PANEL_H
