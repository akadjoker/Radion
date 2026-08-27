#include "PCH.h"

#include "panels/ScriptEditorPanel.h"

#include "EditorApplication.h"
#include "FileSystem.h"
#include "Log.h"
#include "ScriptCache.h"

#include <IconsMaterialDesignIcons.h>
#include <ImGuiMinimap.h>

#include <filesystem>
#include <fstream>
#include <iterator>

namespace Radion
{

ScriptEditorPanel::ScriptEditorPanel(EditorApplication& app) : EditorPanel("Script Editor", app)
{
    mEditor.SetLanguageDefinition(TextEditor::LanguageDefinitionId::ZenScript);
    mEditor.SetPalette(TextEditor::PaletteId::VsCodeDark);
    mEditor.SetAutoIndentEnabled(true);
    mEditor.SetAutoCloseBracketsEnabled(true);
    mEditor.SetShowLineNumbersEnabled(true);
    mEditor.SetShowWhitespacesEnabled(false);
    mEditor.SetShortTabsEnabled(true);
    mEditor.SetFoldingEnabled(true);
}

void ScriptEditorPanel::openFile(const std::string& path)
{
    if (path.empty())
        return;

    if (dirty() && mPath != path)
    {
        mPendingPath = path;
        mConfirmOpen = true;
        setActive(true);
        return;
    }
    loadFile(path);
}

bool ScriptEditorPanel::loadFile(const std::string& path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file)
    {
        Log::error("ScriptEditor: could not open '%s'", path.c_str());
        app().toasts().error("Could not open script " + FileSystem::fileName(path));
        return false;
    }

    const std::string source((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    if (!file.good() && !file.eof())
    {
        Log::error("ScriptEditor: could not read '%s'", path.c_str());
        app().toasts().error("Could not read script " + FileSystem::fileName(path));
        return false;
    }

    mEditor.SetText(source);
    mPath = path;
    mSavedUndoIndex = mEditor.GetUndoIndex();
    setActive(true);
    return true;
}

bool ScriptEditorPanel::saveFile()
{
    if (mPath.empty())
        return false;

    const std::string source = mEditor.GetText();
    std::ofstream file(mPath, std::ios::binary | std::ios::trunc);
    file.write(source.data(), static_cast<std::streamsize>(source.size()));
    file.close();
    if (!file.good())
    {
        Log::error("ScriptEditor: could not save '%s'", mPath.c_str());
        app().toasts().error("Could not save script " + FileSystem::fileName(mPath));
        return false;
    }

    mSavedUndoIndex = mEditor.GetUndoIndex();
    const int refreshed = ScriptCache::getSingleton().refreshChangedFiles();
    Log::info("ScriptEditor: saved '%s'%s", mPath.c_str(),
              refreshed > 0 ? " and refreshed changed scripts" : "");
    app().toasts().success("Script saved");
    return true;
}

bool ScriptEditorPanel::dirty() const
{
    return mEditor.GetUndoIndex() != mSavedUndoIndex;
}

void ScriptEditorPanel::onImGui()
{
    if (mConfirmOpen)
    {
        ImGui::OpenPopup("Discard unsaved script changes?");
        mConfirmOpen = false;
    }
    if (ImGui::BeginPopupModal("Discard unsaved script changes?", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::TextUnformatted("The current script has unsaved changes.");
        ImGui::TextUnformatted("Open the other script and discard them?");
        if (ImGui::Button("Discard and Open"))
        {
            const std::string path = mPendingPath;
            mPendingPath.clear();
            mSavedUndoIndex = mEditor.GetUndoIndex();
            loadFile(path);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            mPendingPath.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (mPath.empty())
    {
        ImGui::TextDisabled("Open a .py file from Assets (double click) to edit it here.");
        return;
    }

    const bool hasChanges = dirty();
    ImGui::BeginDisabled(!hasChanges);
    if (ImGui::Button(ICON_MDI_CONTENT_SAVE))
        saveFile();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(hasChanges ? "Save script (Ctrl+S)" : "No changes to save");
    ImGui::SameLine();
    if (ImGui::Button(ICON_MDI_RELOAD))
    {
        if (hasChanges)
        {
            mPendingPath = mPath;
            mConfirmOpen = true;
        }
        else
        {
            loadFile(mPath);
        }
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Reload the saved file from disk");
    ImGui::SameLine();
    if (ImGui::Button(ICON_MDI_CONTENT_COPY))
        mEditor.Copy();
    ImGui::SameLine();
    if (ImGui::Button(ICON_MDI_CONTENT_PASTE))
        mEditor.Paste();
    ImGui::SameLine();
    if (ImGui::Button(ICON_MDI_FORMAT_INDENT_INCREASE))
        mEditor.Indent();
    ImGui::SameLine();
    if (ImGui::Button(ICON_MDI_FORMAT_INDENT_DECREASE))
        mEditor.Unindent();
    ImGui::SameLine();
    const bool showWhitespace = mEditor.IsShowWhitespacesEnabled();
    if (showWhitespace)
        ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    if (ImGui::Button(ICON_MDI_FORMAT_ALIGN_LEFT))
        mEditor.SetShowWhitespacesEnabled(!showWhitespace);
    if (showWhitespace)
        ImGui::PopStyleColor();
    ImGui::SameLine();
    if (mShowMinimap)
        ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    if (ImGui::Button(ICON_MDI_MAP))
        mShowMinimap = !mShowMinimap;
    if (mShowMinimap)
        ImGui::PopStyleColor();
    ImGui::SameLine();
    if (ImGui::Button(ICON_MDI_MAGNIFY_MINUS))
        mEditor.SetFontScale(mEditor.GetFontScale() - 0.1f);
    ImGui::SameLine(0.0f, 2.0f);
    ImGui::Text("%.0f%%", mEditor.GetFontScale() * 100.0f);
    ImGui::SameLine(0.0f, 2.0f);
    if (ImGui::Button(ICON_MDI_MAGNIFY_PLUS))
        mEditor.SetFontScale(mEditor.GetFontScale() + 0.1f);
    ImGui::Separator();

    const float footerHeight = ImGui::GetTextLineHeightWithSpacing();
    constexpr float minimapWidth = 110.0f;
    const ImVec2 available = ImGui::GetContentRegionAvail();
    const ImVec2 editorSize(available.x - (mShowMinimap ? minimapWidth + ImGui::GetStyle().ItemSpacing.x : 0.0f),
                            available.y - footerHeight);
    const bool focused = mEditor.Render("##zen_script_editor", false, editorSize, true);
    const bool hovered = ImGui::IsItemHovered();
    if (mShowMinimap)
    {
        ImGui::SameLine();
        ImGuiMinimap::Render("##zen_script_minimap", mEditor, minimapWidth, editorSize.y, ImGui::GetFont());
    }

    if (focused && ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false))
        saveFile();
    if (hovered && ImGui::GetIO().KeyCtrl && ImGui::GetIO().MouseWheel != 0.0f)
        mEditor.SetFontScale(mEditor.GetFontScale() + ImGui::GetIO().MouseWheel * 0.1f);

    const TextEditor::TextPosition cursor = mEditor.GetCursorPosition();
    const std::string source = mEditor.GetText();
    const std::string name = std::filesystem::path(mPath).filename().string();
    ImGui::TextDisabled("Ln %d, Col %d  |  %d lines  |  %zu bytes  |  %s", cursor.line + 1,
                        cursor.column + 1, mEditor.GetLineCount(), source.size(), name.c_str());
}

} // namespace Radion
