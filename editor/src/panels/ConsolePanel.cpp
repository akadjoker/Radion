#include "PCH.h"

#include "panels/ConsolePanel.h"

#include "EditorApplication.h"

#include <imgui.h>
#include <string>

namespace Radion
{

namespace
{
const char* prefixFor(LogLevel level)
{
    switch (level)
    {
    case LOG_WARNING: return "[warn] ";
    case LOG_ERROR: return "[error] ";
    case LOG_DEBUG: return "[debug] ";
    default: return "[info] ";
    }
}
} // namespace

std::vector<ConsolePanel::Entry> ConsolePanel::sEntries;

void ConsolePanel::pushEntry(LogLevel level, const char* message)
{
    if (sEntries.size() >= kMaxEntries)
        sEntries.erase(sEntries.begin(), sEntries.begin() + (sEntries.size() - kMaxEntries + 1));
    sEntries.push_back(Entry{level, std::string(message)});
}

ConsolePanel::ConsolePanel(EditorApplication& app) : EditorPanel("Console", app)
{
    const EditorSettings& settings = app.settings();
    mAutoScroll = settings.consoleAutoScroll;
    mShowInfo = settings.consoleShowInfo;
    mShowWarning = settings.consoleShowWarning;
    mShowError = settings.consoleShowError;
    mShowDebug = settings.consoleShowDebug;
}

void ConsolePanel::onImGui()
{
    if (ImGui::Button("Clear"))
        sEntries.clear();
    ImGui::SameLine();
    if (ImGui::Button("Copy"))
    {
        // Only what the active filters currently show - copying the raw
        // full log when Info/Debug are toggled off would hand back lines
        // the user just asked not to see.
        std::string text;
        for (const Entry& entry : sEntries)
        {
            if (entry.level == LOG_INFO && !mShowInfo)
                continue;
            if (entry.level == LOG_WARNING && !mShowWarning)
                continue;
            if (entry.level == LOG_ERROR && !mShowError)
                continue;
            if (entry.level == LOG_DEBUG && !mShowDebug)
                continue;
            text += prefixFor(entry.level);
            text += entry.text;
            text += '\n';
        }
        ImGui::SetClipboardText(text.c_str());
    }
    ImGui::SameLine();
    ImGui::Checkbox("Info", &mShowInfo);
    ImGui::SameLine();
    ImGui::Checkbox("Warning", &mShowWarning);
    ImGui::SameLine();
    ImGui::Checkbox("Error", &mShowError);
    ImGui::SameLine();
    ImGui::Checkbox("Debug", &mShowDebug);
    ImGui::SameLine();
    ImGui::Checkbox("Auto-scroll", &mAutoScroll);
    EditorSettings& settings = app().settings();
    settings.consoleAutoScroll = mAutoScroll;
    settings.consoleShowInfo = mShowInfo;
    settings.consoleShowWarning = mShowWarning;
    settings.consoleShowError = mShowError;
    settings.consoleShowDebug = mShowDebug;
    ImGui::Separator();

    ImGui::BeginChild("console.entries", ImVec2(0.0f, 0.0f), false,
                      ImGuiWindowFlags_HorizontalScrollbar);
    for (const Entry& entry : sEntries)
    {
        if (entry.level == LOG_INFO && !mShowInfo)
            continue;
        if (entry.level == LOG_WARNING && !mShowWarning)
            continue;
        if (entry.level == LOG_ERROR && !mShowError)
            continue;
        if (entry.level == LOG_DEBUG && !mShowDebug)
            continue;

        ImVec4 color(1.0f, 1.0f, 1.0f, 1.0f);
        switch (entry.level)
        {
        case LOG_WARNING: color = ImVec4(0.95f, 0.75f, 0.2f, 1.0f); break;
        case LOG_ERROR: color = ImVec4(0.95f, 0.35f, 0.3f, 1.0f); break;
        case LOG_DEBUG: color = ImVec4(0.6f, 0.6f, 0.6f, 1.0f); break;
        default: break;
        }
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::TextUnformatted((prefixFor(entry.level) + entry.text).c_str());
        ImGui::PopStyleColor();
    }
    if (mAutoScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.0f)
        ImGui::SetScrollHereY(1.0f);
    ImGui::EndChild();
}

} // namespace Radion
