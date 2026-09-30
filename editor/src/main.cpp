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
    // Escape must not quit: it dismisses popups/drags and would lose unsaved work. SDLK_UNKNOWN matches no key.
    engine.getWindow().setExitKey(SDLK_UNKNOWN);

 
    const std::string engineSettingsFile =
        FileSystem::getSingleton().prefPath("Radion", "Editor") + "editor.engine.settings.json";
    engine.setSettingsFile(engineSettingsFile);
    EngineSettings::load(engine, engineSettingsFile);

    // Same multi-folder search every demo registers; shaders fail to load without it.
    FileSystem& files = FileSystem::getSingleton();
    const std::string assetDirectory = resolveAssetDirectory(RADION_ASSET_DIR);
    files.addSearchPath(assetDirectory);
    files.addSearchPath(assetDirectory + "/shaders");
    files.addSearchPath(assetDirectory + "/textures");
    files.addSearchPath(assetDirectory + "/models");

    // ImGuiLayer doesn't enable docking itself; every demo runs without it.
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    applyEditorTheme();
    // MergeMode needs an existing font: add the base font before the atlas is built or AddFont() crashes.
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
