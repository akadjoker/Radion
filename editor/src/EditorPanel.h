#ifndef RADION_EDITOR_PANEL_H
#define RADION_EDITOR_PANEL_H

#include <string>

namespace Radion
{
class EditorApplication;

 
class EditorPanel
{
public:
    EditorPanel(const char* title, EditorApplication& app) : mTitle(title), mApp(app)
    {
    }
    virtual ~EditorPanel() = default;

    virtual void onImGui() = 0;

    // ImGui window name; the .ini docking layout keys on it, keep constant across sessions.
    const std::string& title() const
    {
        return mTitle;
    }

    bool active() const
    {
        return mActive;
    }
    void setActive(bool active)
    {
        mActive = active;
    }

protected:
    EditorApplication& app()
    {
        return mApp;
    }

private:
    std::string mTitle;
    EditorApplication& mApp;
    bool mActive = true;
};

} // namespace Radion

#endif // RADION_EDITOR_PANEL_H
