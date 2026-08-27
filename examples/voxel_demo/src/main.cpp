#include "AssetManager.h"
#include "AssetPaths.h"
#include "Camera.h"
#include "CameraControllers.h"
#include "DebugDraw3D.h"
#include "Engine.h"
#include "FileSystem.h"
#include "Input.h"
#include "Log.h"
#include "GameObject.h"
#include "Light.h"
#include "Material.h"
#include "Mesh.h"
#include "MeshRenderer.h"
#include "Profiler.h"
#include "RenderList.h"
#include "Shadows.h"
#include "Scene.h"
#include "Thread.h"
#include "VoxelBlock.h"
#include "VoxelMesher.h"
#include "VoxelTerrain.h"
#include "VoxelWorld.h"
#include "VoxelWorldComponent.h"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <filesystem>

using namespace Radion;
using namespace Radion::Voxel;

namespace
{
constexpr int kChunkRadius = 6; // chunks kept meshed around the camera, per axis
constexpr u32 kSeed = 1337;
constexpr s32 kWorldMinY = 0;
constexpr s32 kWorldMaxY = 95;
constexpr s32 kWaterLevel = 40;
constexpr f32 kBaseSurfaceHeight = 48.0f;
constexpr s32 kMinSurfaceHeight = 8;
constexpr s32 kMaxSurfaceHeight = 80;
constexpr f32 kContinentalAmplitude = 26.0f;
constexpr f32 kDetailAmplitude = 7.0f;
constexpr f32 kReachDistance = 96.0f;
const char* kWorldFile = "voxel_demo_world.rvox";
const glm::vec3 kWorldCenter(0.0f, kBaseSurfaceHeight, 0.0f);

void addSearchPathIfPresent(FileSystem& files, const std::filesystem::path& path)
{
    std::error_code error;
    if (std::filesystem::is_directory(path, error))
        files.addSearchPath(path.string());
}

} // namespace

int main(int argc, char** argv)
{
    // Release builds start Passive, which drops info: this demo reports what
    // each toggle did, and those lines are the measurement.
    Log::setMode(LogMode::Verbose);

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
    // The engine's own profiler panel carries FPS, frame and GPU times, draw
    // calls and the per-scope table the voxel streaming reports into.
    engine.setBuiltinPanelsVisible(true);
    engine.setImGuiVisible(true);

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
    cameraObject->setPosition(kWorldCenter + glm::vec3(0.0f, 34.0f, -48.0f));
    cameraObject->lookAt(kWorldCenter + glm::vec3(0.0f, 4.0f, 32.0f));
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
    // radion_voxel_demo [chunk radius] - so a run at a given view distance
    // needs no key presses to get there.
    const s32 radius = argc > 1 ? std::max(1, std::min(48, std::atoi(argv[1]))) : kChunkRadius;
    voxelWorld->setChunkRadius(radius);
    voxelWorld->setMinWorldY(kWorldMinY);
    voxelWorld->setMaxWorldY(kWorldMaxY);
    voxelWorld->setWaterLevel(kWaterLevel);
    voxelWorld->setMinSurfaceHeight(kMinSurfaceHeight);
    voxelWorld->setMaxSurfaceHeight(kMaxSurfaceHeight);
    voxelWorld->setBaseSurfaceHeight(kBaseSurfaceHeight);
    voxelWorld->setContinentalAmplitude(kContinentalAmplitude);
    voxelWorld->setDetailAmplitude(kDetailAmplitude);
    voxelWorld->setAmbientOcclusion(true);

    const char* paletteNames[] = {"grass", "dirt", "stone", "sand", "log", "leaves"};
    BlockId palette[6] = {};
    for (usize index = 0; index < 6; ++index)
        palette[index] = voxelWorld->blockIdByName(paletteNames[index]);
    usize selected = 2;

    Log::info("Voxel demo: left click breaks, middle click places, R swaps the block you aim "
              "at, 1-6 pick the block, "
              "hold right to look, F1 hides the panels, F2 toggles ambient occlusion, "
              "F3 cycles the shadow cascade count, F4 toggles vsync, "
              "F5 saves the edited blocks, F9 loads them back, F6/F7 shrink and grow the "
              "chunk radius, Tab walks or flies");

    f32 reportTimer = 0.0f;
    bool vsync = true;

    // Walking body: half a metre wide, 1.8 tall, eyes near the top. The
    // camera is the eye, so the body centre sits below it.
    const glm::vec3 bodyHalfExtents(0.3f, 0.9f, 0.3f);
    constexpr f32 kEyeHeight = 0.7f;
    constexpr f32 kGravity = -26.0f;
    constexpr f32 kJumpSpeed = 9.0f;
    constexpr f32 kWalkSpeed = 6.0f;
    bool walking = false;
    f32 verticalSpeed = 0.0f;

    while (engine.update())
    {
        const f32 deltaTime = std::min(engine.getWindow().getDeltaTime(), 0.1f);

        // A line a second, so a session leaves its own record of what each
        // toggle cost instead of a number read off the screen.
        reportTimer += deltaTime;
        if (reportTimer >= 1.0f)
        {
            reportTimer = 0.0f;
            const CascadeShadowSettings* shadows = engine.cascadeSettings();
            const f64 blockMegabytes =
                static_cast<f64>(voxelWorld->worldMemoryBytes()) / (1024.0 * 1024.0);
            const RenderListStats* view = engine.mainRenderListStats();
            const RenderListStats* shadowList = engine.shadowListStats();
            Log::info("%d FPS  %.2f ms  | AO %s  cascades %u  radius %d | chunks %zu (%.0f MB) "
                      "objects %zu | packets %u submitted %u culled %u/%u | shadow packets %u",
                      engine.getWindow().getFPS(),
                      Profiler::getSingleton().frameMilliseconds(),
                      voxelWorld->ambientOcclusion() ? "on" : "off",
                      shadows ? shadows->count : 0u, voxelWorld->chunkRadius(),
                      voxelWorld->loadedChunks(), blockMegabytes, voxelWorld->renderedChunks(),
                      view ? view->packets : 0u, view ? view->submitted : 0u,
                      view ? view->culledMeshes : 0u, view ? view->culledSubmeshes : 0u,
                      shadowList ? shadowList->packets : 0u);
        }

        if (Input::isKeyPressed(KEY_F1))
            engine.setImGuiVisible(!engine.imGuiVisible());
        if (Input::isKeyPressed(KEY_F2))
        {
            const bool enabled = !voxelWorld->ambientOcclusion();
            voxelWorld->setAmbientOcclusion(enabled);
            Log::info("Voxel demo: ambient occlusion %s", enabled ? "on" : "off");
        }
        if (Input::isKeyPressed(KEY_TAB))
        {
            walking = !walking;
            fly->setActive(!walking);
            verticalSpeed = 0.0f;
            Log::info("Voxel demo: %s", walking ? "walking" : "flying");
        }

        if (walking)
        {
            const glm::vec3 eye = cameraObject->globalPosition();
            glm::vec3 centre = eye - glm::vec3(0.0f, kEyeHeight, 0.0f);

            const glm::vec3 forward = cameraObject->forward();
            const glm::vec3 flatForward =
                glm::normalize(glm::vec3(forward.x, 0.0f, forward.z) + glm::vec3(1e-5f, 0.0f, 0.0f));
            const glm::vec3 right = glm::normalize(glm::cross(flatForward, glm::vec3(0, 1, 0)));

            glm::vec3 wish(0.0f);
            if (Input::isKeyDown(KEY_W)) wish += flatForward;
            if (Input::isKeyDown(KEY_S)) wish -= flatForward;
            if (Input::isKeyDown(KEY_D)) wish += right;
            if (Input::isKeyDown(KEY_A)) wish -= right;
            if (glm::dot(wish, wish) > 0.0f)
                wish = glm::normalize(wish) * kWalkSpeed;

            const bool onGround =
                voxelWorld->moveBox(centre, bodyHalfExtents, glm::vec3(0.0f)).grounded;
            if (onGround && verticalSpeed <= 0.0f)
            {
                verticalSpeed = Input::isKeyDown(KEY_SPACE) ? kJumpSpeed : 0.0f;
            }
            else
            {
                verticalSpeed += kGravity * deltaTime;
            }

            const glm::vec3 displacement(wish.x * deltaTime, verticalSpeed * deltaTime,
                                         wish.z * deltaTime);
            const VoxelMoveResult moved =
                voxelWorld->moveBox(centre, bodyHalfExtents, displacement);
            if (moved.grounded && verticalSpeed < 0.0f)
                verticalSpeed = 0.0f;
            if (moved.ceiling && verticalSpeed > 0.0f)
                verticalSpeed = 0.0f;

            cameraObject->setPosition(moved.position + glm::vec3(0.0f, kEyeHeight, 0.0f));
        }

        if (Input::isKeyPressed(KEY_F6) || Input::isKeyPressed(KEY_F7))
        {
            const s32 step = Input::isKeyPressed(KEY_F7) ? 1 : -1;
            const s32 radius = std::max(1, std::min(32, voxelWorld->chunkRadius() + step));
            voxelWorld->setChunkRadius(radius);
            Log::info("Voxel demo: chunk radius %d", radius);
        }
        if (Input::isKeyPressed(KEY_F5))
        {
            Log::info("Voxel demo: saving %zu edited blocks to %s",
                      voxelWorld->editedBlocks(), kWorldFile);
            if (!voxelWorld->saveEdits(kWorldFile))
                Log::error("Voxel demo: could not write %s", kWorldFile);
        }
        if (Input::isKeyPressed(KEY_F9))
        {
            if (voxelWorld->loadEdits(kWorldFile))
                Log::info("Voxel demo: loaded %zu edited blocks", voxelWorld->editedBlocks());
            else
                Log::error("Voxel demo: could not read %s", kWorldFile);
        }
        if (Input::isKeyPressed(KEY_F4))
        {
            // With vsync on, FPS reports the monitor and not the frame's cost:
            // any measurement of an optimisation has to run without it.
            vsync = !vsync;
            engine.getWindow().setVSync(vsync);
            Log::info("Voxel demo: vsync %s", vsync ? "on" : "off");
        }
        if (Input::isKeyPressed(KEY_F3))
        {
            if (CascadeShadowSettings* shadows = engine.cascadeSettings())
            {
                shadows->count = shadows->count > 1 ? shadows->count - 1 : 4;
                Log::info("Voxel demo: %u shadow cascades", shadows->count);
            }
        }

        for (usize index = 0; index < 6; ++index)
        {
            if (Input::isKeyPressed(static_cast<KeyCode>(KEY_ONE + index)) &&
                palette[index] != InvalidBlockId)
            {
                selected = index;
                Log::info("Voxel demo: holding %s", paletteNames[index]);
            }
        }

        // Emitted before the render, which is where the frame's debug list is
        // cleared.
        const FloatRect viewport(0.0f, 0.0f, static_cast<f32>(engine.getWindow().getWidth()),
                                 static_cast<f32>(engine.getWindow().getHeight()));
        const Ray ray = camera->rayFromMouse(static_cast<f32>(Input::getMouseX()),
                                             static_cast<f32>(Input::getMouseY()), viewport);
        VoxelRaycastHit hit;
        if (!engine.uiWantsMouse() &&
            voxelWorld->raycast(ray.origin, ray.direction, kReachDistance, hit))
        {
            DebugDraw().box(VoxelWorldComponent::blockBounds(hit.block), Color::Yellow);
            if (Input::isMousePressed(LEFT))
                voxelWorld->removeBlock(hit);
            else if (Input::isMousePressed(MIDDLE))
                voxelWorld->placeBlock(hit, palette[selected]);
            else if (Input::isKeyPressed(KEY_R))
                voxelWorld->replaceBlock(hit, palette[selected]);
        }

        scene->update(deltaTime);
        engine.render(*scene);
        engine.flip();
    }

    engine.shutdown();
    return 0;
}
