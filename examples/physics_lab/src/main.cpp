// physics_lab - one scene per thing worth watching, all built from
// primitives so what is on screen is the simulation and not an asset.
//
// 1-4 switch scenes, R rebuilds the current one, Space pauses, F1 toggles
// the panels. Nothing here draws or steps anything itself: it picks a scene,
// feeds it time, and lets the engine run.

#include "AssetPaths.h"
#include "Engine.h"
#include "FileSystem.h"
#include "Input.h"
#include "LabScenes.h"
#include "Log.h"
#include "Scene.h"
#include "SceneManager.h"
#include "Window.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>

using namespace Radion;

namespace
{
void addSearchPathIfPresent(FileSystem& files, const std::filesystem::path& path)
{
    std::error_code error;
    if (std::filesystem::is_directory(path, error))
        files.addSearchPath(path.string());
}

Lab::LabScene requestedScene(Lab::LabScene current)
{
    if (Input::isKeyPressed(KEY_ONE))
        return Lab::LabScene::JointGallery;
    if (Input::isKeyPressed(KEY_TWO))
        return Lab::LabScene::RobotArm;
    if (Input::isKeyPressed(KEY_THREE))
        return Lab::LabScene::Vehicle;
    if (Input::isKeyPressed(KEY_FOUR))
        return Lab::LabScene::AgentCrowd;
    return current;
}
} // namespace

int main(int argc, char** argv)
{
    Log::setMode(LogMode::Verbose);

    FileSystem& files = FileSystem::getSingleton();
    const std::filesystem::path assets = resolveAssetDirectory(RADION_ASSET_DIR);
    addSearchPathIfPresent(files, assets);
    addSearchPathIfPresent(files, assets / "shaders");
    addSearchPathIfPresent(files, assets / "textures");

    Engine engine;
    EngineConfig config;
    config.title = "Radion Physics Lab";
    config.width = 1280;
    config.height = 720;
    if (!engine.initialize(config))
        return 1;
    engine.setBuiltinPanelsVisible(true);
    engine.setImGuiVisible(true);
    // The colliders are the point of this demo, so they start visible.
    engine.debugShowPhysicsShapes = true;

    Scene* scene = engine.createScene();
    if (!scene || !engine.setActiveScene(scene))
    {
        engine.shutdown();
        return 1;
    }
    scene->setRunningInEditor(false);

    // First argument opens a specific scene, so one can be iterated on
    // without clicking through the others.
    Lab::LabScene current = Lab::LabScene::JointGallery;
    if (argc > 1)
    {
        const int requested = std::atoi(argv[1]);
        if (requested >= 1 && requested <= static_cast<int>(Lab::LabScene::Count))
            current = static_cast<Lab::LabScene>(requested - 1);
    }
    Lab::buildScene(*scene, current);
    Log::info("Physics lab: %s. 1-%d switch scenes, R rebuilds, Space pauses.",
              Lab::sceneName(current), static_cast<int>(Lab::LabScene::Count));

    f32 elapsed = 0.0f;
    bool paused = false;

    while (engine.update())
    {
        const f32 deltaTime = std::min(engine.getWindow().getDeltaTime(), 0.1f);

        if (Input::isKeyPressed(KEY_F1))
            engine.setImGuiVisible(!engine.imGuiVisible());
        if (Input::isKeyPressed(KEY_SPACE))
            paused = !paused;

        const Lab::LabScene wanted = requestedScene(current);
        if (wanted != current || Input::isKeyPressed(KEY_R))
        {
            // A fresh scene rather than clearing this one: it is one call,
            // and it guarantees nothing survives - no joint still holding a
            // body, no agent still registered.
            current = wanted;
            elapsed = 0.0f;
            engine.sceneManager().unload();
            scene = engine.createScene();
            if (!scene || !engine.setActiveScene(scene))
                break;
            scene->setRunningInEditor(false);
            Lab::buildScene(*scene, current);
            Log::info("Physics lab: %s", Lab::sceneName(current));
        }

        if (!paused)
        {
            elapsed += deltaTime;
            Lab::updateScene(*scene, current, elapsed);
            scene->update(deltaTime);
        }

        engine.render(*scene);
        engine.flip();
    }

    engine.shutdown();
    return 0;
}
