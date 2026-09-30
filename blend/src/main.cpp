#include "PCH.h"

#include "BlenderApplication.h"
#include "BlenderTheme.h"
#include "AssetPaths.h"
#include "Engine.h"
#include "EngineSettings.h"
#include "FileSystem.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <imgui.h>
#include <string>

using namespace Radion;

namespace
{
// The token can also come from the environment, which keeps it out of the process list.
struct ApiOptions
{
    bool enable = false;
    bool disable = false;
    int port = 0; // 0: the saved one
    std::string host = "127.0.0.1";
    std::string token;
};

void printUsage()
{
    std::printf("radion_blender [options]\n"
                "  --api                 start the local HTTP API with the editor\n"
                "  --api-port <port>     port for the API (default 7420); implies --api\n"
                "  --api-host <address>  address to listen on (default 127.0.0.1); implies --api.\n"
                "                        Anything other than loopback requires a token\n"
                "  --api-token <secret>  require 'Authorization: Bearer <secret>'\n"
                "                        (or set RADION_BLENDER_API_TOKEN)\n"
                "  --no-api              do not start the API even if it is enabled in the\n"
                "                        preferences\n"
                "  --help                this text\n");
}

bool parseArguments(int argc, char** argv, ApiOptions& options, bool& helpOnly)
{
    if (const char* token = std::getenv("RADION_BLENDER_API_TOKEN"))
        options.token = token;

    for (int i = 1; i < argc; ++i)
    {
        const char* argument = argv[i];
        auto value = [&](const char* name) -> const char*
        {
            if (i + 1 >= argc)
            {
                std::fprintf(stderr, "radion_blender: %s needs a value\n", name);
                return nullptr;
            }
            return argv[++i];
        };

        if (!std::strcmp(argument, "--api"))
        {
            options.enable = true;
        }
        else if (!std::strcmp(argument, "--no-api"))
        {
            options.disable = true;
        }
        else if (!std::strcmp(argument, "--api-port"))
        {
            const char* text = value(argument);
            if (!text)
                return false;
            char* end = nullptr;
            const long port = std::strtol(text, &end, 10);
            if (end == text || *end != '\0' || port < 1 || port > 65535)
            {
                std::fprintf(stderr, "radion_blender: '%s' is not a port (1-65535)\n", text);
                return false;
            }
            options.port = static_cast<int>(port);
            options.enable = true;
        }
        else if (!std::strcmp(argument, "--api-host"))
        {
            const char* text = value(argument);
            if (!text)
                return false;
            options.host = text;
            options.enable = true;
        }
        else if (!std::strcmp(argument, "--api-token"))
        {
            const char* text = value(argument);
            if (!text)
                return false;
            options.token = text;
        }
        else if (!std::strcmp(argument, "--help") || !std::strcmp(argument, "-h"))
        {
            helpOnly = true;
        }
        else
        {
            std::fprintf(stderr, "radion_blender: unknown option '%s' (see --help)\n", argument);
            return false;
        }
    }
    return true;
}
} // namespace

int main(int argc, char** argv)
{
    ApiOptions api;
    bool helpOnly = false;
    if (!parseArguments(argc, argv, api, helpOnly))
        return 2;
    if (helpOnly)
    {
        printUsage();
        return 0;
    }

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

    const std::string engineSettingsFile =
        FileSystem::getSingleton().prefPath("Radion", "Blender") + "blender.engine.settings.json";
    engine.setSettingsFile(engineSettingsFile);
    EngineSettings::load(engine, engineSettingsFile);

    FileSystem& files = FileSystem::getSingleton();
    const std::string assetDirectory = resolveAssetDirectory(RADION_ASSET_DIR);
    files.addSearchPath(assetDirectory);
    files.addSearchPath(assetDirectory + "/shaders");
    files.addSearchPath(assetDirectory + "/textures");
    files.addSearchPath(assetDirectory + "/models");

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // Own ini file: the shared ./imgui.ini would have the editor and blender overwrite each other's docking layout.
    static const std::string iniPath =
        FileSystem::getSingleton().prefPath("Radion", "Blender") + "blender_imgui.ini";
    io.IniFilename = iniPath.c_str();

    applyBlenderTheme();
    io.Fonts->AddFontDefault();
    if (!loadBlenderIconFont(io))
        std::fprintf(stderr, "radion_blender: failed to load icon font\n");

    {
        BlenderApplication blender(engine);

        const bool startApi = !api.disable && (api.enable || blender.settings().api().enabled);
        if (startApi)
        {
            const int port = api.port > 0 ? api.port : blender.settings().api().port;
            // A failed start is logged; the editor works without it.
            blender.startApi(api.host, port, api.token);
        }
        blender.run();
    }

    engine.shutdown();
    return 0;
}
