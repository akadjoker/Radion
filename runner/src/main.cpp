#include "Engine.h"
#include "AssetPaths.h"
#include "FileSystem.h"
#include "Input.h"
#include "Log.h"
#include "Scene.h"
#include "SceneSerializer.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include <imgui.h>

using namespace Radion;

namespace
{
struct RunTarget
{
    std::filesystem::path scene;
    std::filesystem::path projectRoot;
    std::filesystem::path assetRoot;
    std::vector<std::string> searchPaths;
};

bool safeRelativePath(const std::string& value)
{
    const std::filesystem::path path(value);
    if (path.empty() || path.is_absolute())
        return false;
    for (const std::filesystem::path& component : path.lexically_normal())
        if (component == "..")
            return false;
    return true;
}

void printDiagnostics(const SceneLoadResult& result)
{
    for (const SceneDiagnostic& diagnostic : result.diagnostics)
        std::fprintf(stderr, "%s: %s%s%s\n",
                     diagnostic.severity == SceneDiagnosticSeverity::Error ? "error" : "warning",
                     diagnostic.jsonPath.empty() ? "" : diagnostic.jsonPath.c_str(),
                     diagnostic.jsonPath.empty() ? "" : ": ", diagnostic.message.c_str());
}

bool resolveRunTarget(const char* argument, RunTarget& out, std::string& error)
{
    if (!argument || !argument[0])
    {
        error = "missing scene or project path";
        return false;
    }

    const std::filesystem::path input = std::filesystem::absolute(argument).lexically_normal();
    out.scene = input;
    out.projectRoot = input.parent_path();
    out.assetRoot = out.projectRoot / "Assets";

    // A scene does not need a preliminary JSON parse. For a project manifest
    // we need its active scene and asset search paths before SceneManager
    // starts resolving mesh/material/texture references.
    const std::string text = FileSystem::getSingleton().readText(input.string());
    if (text.empty())
        return true; // SceneManager will report the real load error with its diagnostics.

    nlohmann::json manifest;
    try
    {
        manifest = nlohmann::json::parse(text);
    }
    catch (const std::exception&)
    {
        return true; // It may be a malformed scene; let the scene loader diagnose it.
    }
    if (manifest.value("format", std::string()) != "radion-project")
        return true;

    const auto scenes = manifest.find("scenes");
    const std::string activeScene = manifest.value("activeScene", std::string());
    std::string assetRoot = manifest.value("assetRoot", std::string("Assets"));
    if (scenes == manifest.end() || !scenes->is_array() || !safeRelativePath(activeScene) ||
        !safeRelativePath(assetRoot))
    {
        error = "invalid radion-project manifest";
        return false;
    }

    const std::string normalizedActive =
        std::filesystem::path(activeScene).lexically_normal().generic_string();
    bool listed = false;
    for (const nlohmann::json& scene : *scenes)
        if (scene.is_string() &&
            std::filesystem::path(scene.get<std::string>()).lexically_normal().generic_string() ==
                normalizedActive)
            listed = true;
    if (!listed)
    {
        error = "project activeScene is not listed in scenes";
        return false;
    }

    out.projectRoot = input.parent_path();
    out.scene = out.projectRoot / normalizedActive;
    out.assetRoot = out.projectRoot / std::filesystem::path(assetRoot).lexically_normal();
    const auto searchPaths = manifest.find("searchPaths");
    if (searchPaths != manifest.end() && searchPaths->is_array())
        for (const nlohmann::json& path : *searchPaths)
            if (path.is_string() && !path.get_ref<const std::string&>().empty())
                out.searchPaths.push_back(path.get<std::string>());
    return true;
}

void addSearchPathIfPresent(FileSystem& files, const std::filesystem::path& path)
{
    std::error_code error;
    if (std::filesystem::is_directory(path, error))
        files.addSearchPath(path.string());
}

void drawProfilerOverlay(Engine& engine)
{
    ImGui::SetNextWindowBgAlpha(0.82f);
    ImGui::SetNextWindowPos(ImVec2(12.0f, 12.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(520.0f, 0.0f), ImGuiCond_Always);
    constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration |
                                       ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoSavedSettings |
                                       ImGuiWindowFlags_AlwaysAutoResize;
    if (ImGui::Begin("Runner profiler", nullptr, flags))
    {
        ImGui::TextUnformatted("Profiler (F5)");
        engine.drawProfilerContents();
    }
    ImGui::End();
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2 || argc > 3)
    {
        std::fprintf(stderr,
                     "Usage: radion_runner <scene.scene.json | project.radion-project> [--no-post]\n");
        return 1;
    }
    const bool disablePost = argc == 3 && std::string(argv[2]) == "--no-post";
    if (argc == 3 && !disablePost)
    {
        std::fprintf(stderr, "radion_runner: unknown option '%s'\n", argv[2]);
        return 1;
    }

    RunTarget target;
    std::string error;
    if (!resolveRunTarget(argv[1], target, error))
    {
        std::fprintf(stderr, "radion_runner: %s\n", error.c_str());
        return 1;
    }

    FileSystem& files = FileSystem::getSingleton();
    // Engine's own sample assets keep built-in materials/shaders available;
    // project paths then take part in normal asset lookup just like they do
    // in the editor before it opens the active scene.
    const std::filesystem::path builtInAssets = resolveAssetDirectory(RADION_ASSET_DIR);
    addSearchPathIfPresent(files, builtInAssets);
    addSearchPathIfPresent(files, builtInAssets / "shaders");
    addSearchPathIfPresent(files, builtInAssets / "textures");
    addSearchPathIfPresent(files, builtInAssets / "models");
    addSearchPathIfPresent(files, target.projectRoot);
    addSearchPathIfPresent(files, target.scene.parent_path());
    addSearchPathIfPresent(files, target.assetRoot);
    for (const std::string& searchPath : target.searchPaths)
        addSearchPathIfPresent(files, searchPath);

    Engine engine;
    EngineConfig config;
    config.title = "Radion Game";
    config.width = 1280;
    config.height = 720;
    if (!engine.initialize(config))
        return 1;
    engine.setBuiltinPanelsVisible(false);
    engine.setImGuiVisible(false);
    if (disablePost)
    {
        // Keep the game scene, lighting and shadows intact, but strip every
        // screen-space or post-lighting effect for an honest render baseline.
        const u32 disabled = RenderPassPostProcess | RenderPassAmbientOcclusion |
                             RenderPassTemporalAA | RenderPassVolumetrics |
                             RenderPassLensFlares | RenderPassPlanarReflections;
        engine.setEnabledPasses(RenderPassAll & ~disabled);
        Log::info("Runner: post effects disabled (--no-post)");
    }

    SceneLoadResult loadResult;
    if (!engine.sceneManager().load(target.scene.string(), loadResult))
    {
        printDiagnostics(loadResult);
        engine.shutdown();
        return 1;
    }
    printDiagnostics(loadResult);

    Scene* scene = engine.activeScene();
    if (!scene)
    {
        engine.shutdown();
        return 1;
    }
    scene->setRunningInEditor(false);
    scene->rebuildStaticIndex();

    bool profilerVisible = false;
    while (engine.update())
    {
        if (Input::isKeyPressed(KEY_F5))
        {
            profilerVisible = !profilerVisible;
            engine.setImGuiVisible(profilerVisible);
        }

        const f32 deltaTime = std::min(engine.getWindow().getDeltaTime(), 0.1f);
        scene->update(deltaTime);
        engine.render(*scene);
        if (profilerVisible)
            drawProfilerOverlay(engine);
        engine.flip();
    }

    engine.shutdown();
    return 0;
}
