#ifndef RADION_BLENDER_PANEL_H
#define RADION_BLENDER_PANEL_H

#include <string>

namespace Radion
{
class BlenderApplication;

class BlenderPanel
{
public:
    BlenderPanel(const char* title, BlenderApplication& app)
        : mTitle(title), mApp(app)
    {
    }
    virtual ~BlenderPanel() = default;

    virtual void onImGui() = 0;

    // ImGui window name and docking layout key — keep constant across sessions.
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
    BlenderApplication& app()
    {
        return mApp;
    }

private:
    std::string mTitle;
    BlenderApplication& mApp;
    bool mActive = true;
};

} // namespace Radion

#endif // RADION_BLENDER_PANEL_H
