#include "PCH.h"

#include "EditorApplication.h"
#include "EditorTheme.h"
#include "AssetPaths.h"
#include "Engine.h"
#include "EngineSettings.h"
#include "FileSystem.h"

#include <cstdio>
#include <imgui.h>

using namespace Radion;

 
int main(int, char**)
{
    Engine engine;
    EngineConfig config;
    config.title = "Radion Editor";
    config.width = 1600;
    config.height = 900;
    if (!engine.initialize(config))
        return 1;
    engine.setBuiltinPanelsVisible(false);
    // The window's default exit key is Escape - right for a demo, fatal in
    // the editor, where Escape is how a popup or drag is normally dismissed
    // and unsaved work would be gone with it. SDLK_UNKNOWN matches no key.
    engine.getWindow().setExitKey(SDLK_UNKNOWN);

 
    const std::string engineSettingsFile =
        FileSystem::getSingleton().prefPath("Radion", "Editor") + "editor.engine.settings.json";
    engine.setSettingsFile(engineSettingsFile);
    EngineSettings::load(engine, engineSettingsFile);

    // Same multi-folder search every demo registers - shaders/particle
    // system/lens flare all fail to load without it.
    FileSystem& files = FileSystem::getSingleton();
    const std::string assetDirectory = resolveAssetDirectory(RADION_ASSET_DIR);
    files.addSearchPath(assetDirectory);
    files.addSearchPath(assetDirectory + "/shaders");
    files.addSearchPath(assetDirectory + "/textures");
    files.addSearchPath(assetDirectory + "/models");

    // Radion's ImGuiLayer does not turn this on itself - every demo runs
    // without docking, and enabling it globally is not this editor's call to
    // make on their behalf. Vendor ImGui already carries the docking branch
    // (ImGui::DockSpace/DockBuilder are available), so this is enough.
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    applyEditorTheme();
    // ImGuiLayer::initialize() does not add a base font itself (it leaves
    // that to ImGui's own lazy default on first NewFrame()) - MergeMode
    // needs an existing font to merge onto, so the base font has to be added
    // explicitly here, before the atlas is ever built, or AddFont() crashes
    // trying to merge onto an empty atlas.
    io.Fonts->AddFontDefault();
    if (!loadEditorIconFont(io))
        std::fprintf(stderr, "radion_editor: failed to load the Material Design icon font\n");

    {
        EditorApplication editor(engine);
        editor.run();
    }

    engine.shutdown();
    return 0;
}
