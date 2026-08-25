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

    // Log has one sink and two things need it, so EditorApplication owns the
    // registration and calls this - see EditorApplication::logSink().
    static void pushEntry(LogLevel level, const char* message);

private:
    struct Entry
    {
        LogLevel level;
        std::string text;
    };

    // Shared across every ConsolePanel instance, of which there is only ever
    // one - the sink above has no `this` to route through, so the buffer it
    // writes into has to live at namespace/class scope instead of on an
    // instance. Capped in pushEntry(): an editor session left running for
    // hours must not grow this without bound.
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
