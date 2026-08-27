#include "AssetManager.h"
#include "AssetPaths.h"
#include "Camera.h"
#include "CameraControllers.h"
#include "Engine.h"
#include "FileSystem.h"
#include "GameObject.h"
#include "Light.h"
#include "Material.h"
#include "Mesh.h"
#include "MeshRenderer.h"
#include "Scene.h"
#include "Thread.h"
#include "VoxelBlock.h"
#include "VoxelMesher.h"
#include "VoxelTerrain.h"
#include "VoxelWorld.h"
#include "VoxelWorldComponent.h"

#include <algorithm>
#include <cmath>
#include <filesystem>

using namespace Radion;
using namespace Radion::Voxel;

namespace
{
constexpr int kChunkRadius = 1; // chunks around the centre, per axis
constexpr u32 kSeed = 1337;
constexpr s32 kWorldMinY = 32;
constexpr s32 kWorldMaxY = 95;
const glm::vec3 kWorldCenter(0.0f, 80.0f, 0.0f);

void addSearchPathIfPresent(FileSystem& files, const std::filesystem::path& path)
{
    std::error_code error;
    if (std::filesystem::is_directory(path, error))
        files.addSearchPath(path.string());
}

} // namespace

int main(int, char**)
{
    FileSystem& files = FileSystem::getSingleton();
    const std::filesystem::path builtInAssets = resolveAssetDirectory(RADION_ASSET_DIR);
    addSearchPathIfPresent(files, builtInAssets);
    addSearchPathIfPresent(files, builtInAssets / "shaders");
    addSearchPathIfPresent(files, builtInAssets / "textures");

    Engine engine;
    EngineConfig config;
    config.title = "Radion Voxel Demo";
    config.width = 1280;
    config.height = 720;
    if (!engine.initialize(config))
        return 1;
    engine.setBuiltinPanelsVisible(false);
    engine.setImGuiVisible(false);

    Scene* scene = engine.createScene();
    if (!scene || !engine.setActiveScene(scene))
    {
        engine.shutdown();
        return 1;
    }
    scene->setRunningInEditor(false);

    // Camera: free fly starting above and in front of the generated terrain.
    GameObject* cameraObject = scene->createGameObject("Camera");
    Camera* camera = cameraObject->addComponent<Camera>();
    camera->setPerspective(60.0f, 16.0f / 9.0f, 0.1f, 1000.0f);
    cameraObject->setPosition(kWorldCenter + glm::vec3(48.0f, 112.0f, -96.0f));
    cameraObject->lookAt(kWorldCenter + glm::vec3(0.0f, 42.0f, 0.0f));
    FreeFly* fly = cameraObject->addComponent<FreeFly>();
    fly->setMoveSpeed(28.0f);
    fly->setSprintMultiplier(2.5f);
    scene->setActiveCamera(camera);

    GameObject* sunObject = scene->createGameObject("Sun");
    DirectionalLight* sun = sunObject->addComponent<DirectionalLight>();
    sun->setColor(glm::vec3(1.0f, 0.96f, 0.88f));
    sun->setIntensity(1.2f);
    sunObject->setPosition(kWorldCenter + glm::vec3(-60.0f, 80.0f, -40.0f));
    sunObject->lookAt(kWorldCenter);

    GameObject* voxelObject = scene->createGameObject("VoxelWorld");
    VoxelWorldComponent* voxelWorld = voxelObject->addComponent<VoxelWorldComponent>();
    voxelWorld->setOriginObjectId(cameraObject->id());
    voxelWorld->setSeed(kSeed);
    voxelWorld->setChunkRadius(kChunkRadius);
    voxelWorld->setMinWorldY(kWorldMinY);
    voxelWorld->setMaxWorldY(kWorldMaxY);
    voxelWorld->setWaterLevel(20);

    while (engine.update())
    {
        const f32 deltaTime = std::min(engine.getWindow().getDeltaTime(), 0.1f);
        scene->update(deltaTime);
        engine.render(*scene);
        engine.flip();
    }

    engine.shutdown();
    return 0;
}
