#ifndef RADION_SCRIPT_EDITOR_PANEL_H
#define RADION_SCRIPT_EDITOR_PANEL_H

#include "EditorPanel.h"

#include <TextEditor.h>

#include <string>

namespace Radion
{

class ScriptEditorPanel final : public EditorPanel
{
public:
    explicit ScriptEditorPanel(EditorApplication& app);

    void openFile(const std::string& path);

    void onImGui() override;

private:
    bool loadFile(const std::string& path);
    bool saveFile();
    bool dirty() const;

    std::string mPath;
    std::string mPendingPath;
    TextEditor mEditor;
    int mSavedUndoIndex = 0;
    bool mConfirmOpen = false;
    bool mShowMinimap = true;
};

} // namespace Radion

#endif // RADION_SCRIPT_EDITOR_PANEL_H
