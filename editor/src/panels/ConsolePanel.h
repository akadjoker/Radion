#ifndef RADION_CONSOLE_PANEL_H
#define RADION_CONSOLE_PANEL_H

#include "EditorPanel.h"
#include "Log.h"

#include <string>
#include <vector>

namespace Radion
{
class ConsolePanel final : public EditorPanel
{
public:
    explicit ConsolePanel(EditorApplication& app);

    void onImGui() override;

    // EditorApplication owns the Log sink registration and calls this.
    static void pushEntry(LogLevel level, const char* message);

private:
    struct Entry
    {
        LogLevel level;
        std::string text;
    };

    // Shared by all instances (the sink has no `this`); capped in pushEntry() to bound growth.
    static std::vector<Entry> sEntries;
    static constexpr usize kMaxEntries = 2000;

    bool mAutoScroll = true;
    bool mShowInfo = true;
    bool mShowWarning = true;
    bool mShowError = true;
    bool mShowDebug = false;
};
} // namespace Radion
#endif
