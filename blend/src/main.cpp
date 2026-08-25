#include "PCH.h"

#include "BlenderApplication.h"
#include "BlenderTheme.h"
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
    config.title = "Radion Blender";
    config.width = 1920;
    config.height = 1080;
    if (!engine.initialize(config))
        return 1;
    engine.setBuiltinPanelsVisible(false);
    // Escape dismisses popups/drags in a tool - it must not close the window
    // (the default exit key) with unsaved work in it.
    engine.getWindow().setExitKey(SDLK_UNKNOWN);

    // Load engine settings
    const std::string engineSettingsFile =
        FileSystem::getSingleton().prefPath("Radion", "Blender") + "blender.engine.settings.json";
    engine.setSettingsFile(engineSettingsFile);
    EngineSettings::load(engine, engineSettingsFile);

    // Add search paths for assets (shaders, textures, models)
    FileSystem& files = FileSystem::getSingleton();
    const std::string assetDirectory = resolveAssetDirectory(RADION_ASSET_DIR);
    files.addSearchPath(assetDirectory);
    files.addSearchPath(assetDirectory + "/shaders");
    files.addSearchPath(assetDirectory + "/textures");
    files.addSearchPath(assetDirectory + "/models");

    // Enable ImGui docking
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // Nobody sets this by default, so every ImGui app in the repo shares
    // ./imgui.ini - the editor's dozens of windows and the blender's few end
    // up in the same file, each overwriting the other's docking layout. Its
    // own file, next to the prefPath settings this app already writes to.
    static const std::string iniPath =
        FileSystem::getSingleton().prefPath("Radion", "Blender") + "blender_imgui.ini";
    io.IniFilename = iniPath.c_str();

    applyBlenderTheme();
    // Add base font before icon font merge
    io.Fonts->AddFontDefault();
    if (!loadBlenderIconFont(io))
        std::fprintf(stderr, "radion_blender: failed to load icon font\n");

    {
        BlenderApplication blender(engine);
        blender.run();
    }

    engine.shutdown();
    return 0;
}
