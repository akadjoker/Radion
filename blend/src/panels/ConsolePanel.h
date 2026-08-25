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

    // Log has one sink and both editor and blend need it - BlenderApplication
    // owns the registration and calls this, same split as EditorApplication's
    // own logSink()/ConsolePanel::pushEntry().
    static void pushEntry(LogLevel level, const char* message);

private:
    struct Entry
    {
        LogLevel level;
        std::string text;
    };

    // Shared across every ConsolePanel instance (there is only ever one) -
    // the sink has no `this` to route through. Capped in pushEntry() so a
    // session left running for hours does not grow this without bound.
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
