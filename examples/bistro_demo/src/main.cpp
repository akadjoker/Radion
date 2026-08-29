#include "AssetManager.h"
#include "AssetPaths.h"
#include "Camera.h"
#include "CameraControllers.h"
#include "Engine.h"
#include "FileSystem.h"
#include "GameObject.h"
#include "GPU.h"
#include "GPUProfiler.h"
#include "Input.h"
#include "Light.h"
#include "Log.h"
#include "Material.h"
#include "MaterialManager.h"
#include "Mesh.h"
#include "MeshRenderer.h"
#include "Profiler.h"
#include "RenderList.h"
#include "Scene.h"
#include "Shadows.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

using namespace Radion;

namespace
{
const char* kDefaultMesh = "/media/projectos/assets/bistro/extrior.rstm";
// Measured off the file's own bounds, not assumed: extrior.rstm spans
// 170 x 50 x 180 in its own units, so it is already in metres. The 0.01 in
// tools/lightmapbake/bistro_settings.json belongs to a centimetre export
// this .rstm is no longer one of.
constexpr f32 kSceneScale = 1.0f;

void addSearchPathIfPresent(FileSystem& files, const std::filesystem::path& path)
{
    std::error_code error;
    if (std::filesystem::is_directory(path, error))
        files.addSearchPath(path.string());
}

std::string companionMaterialFile(const std::string& meshFile)
{
    const usize dot = meshFile.find_last_of('.');
    return (dot == std::string::npos ? meshFile : meshFile.substr(0, dot)) + ".mat";
}

} // namespace

int main(int argc, char** argv)
{
    // The per-second report below is the whole point of this demo, and
    // release builds start Passive.
    Log::setMode(LogMode::Verbose);

    // radion_bistro_demo [mesh] [split target triangles] - so the same binary
    // measures the baked mesh against the one it came from, and a split
    // against no split, without a rebuild. 0 leaves the submeshes as the file
    // has them.
    const std::string meshFile = argc > 1 ? argv[1] : kDefaultMesh;
    const u32 splitTriangles = argc > 2 ? static_cast<u32>(std::max(0, std::atoi(argv[2]))) : 0u;
    // Third argument: keep this fraction of the triangles. Not a LOD chain,
    // a ceiling measurement - one level applied to the whole mesh says what
    // a per-cell chain could be worth before anyone builds one.
    const f32 simplifyRatio = argc > 3 ? static_cast<f32>(std::atof(argv[3])) : 0.0f;

    FileSystem& files = FileSystem::getSingleton();
    const std::filesystem::path builtInAssets = resolveAssetDirectory(RADION_ASSET_DIR);
    addSearchPathIfPresent(files, builtInAssets);
    addSearchPathIfPresent(files, builtInAssets / "shaders");
    addSearchPathIfPresent(files, builtInAssets / "textures");

    const std::filesystem::path meshDirectory = std::filesystem::path(meshFile).parent_path();
    addSearchPathIfPresent(files, meshDirectory);
    addSearchPathIfPresent(files, meshDirectory / "Textures");
    addSearchPathIfPresent(files, meshDirectory / "textures");

    Engine engine;
    EngineConfig config;
    config.title = "Radion Bistro Demo";
    config.width = 1280;
    config.height = 720;
    if (!engine.initialize(config))
        return 1;
    engine.setBuiltinPanelsVisible(true);
    engine.setImGuiVisible(true);

    Scene* scene = engine.createScene();
    if (!scene || !engine.setActiveScene(scene))
    {
        engine.shutdown();
        return 1;
    }
    scene->setRunningInEditor(false);

    MeshData data;
    if (!Assets().importMesh(meshFile, data))
    {
        Log::error("Bistro demo: could not load '%s'", meshFile.c_str());
        engine.shutdown();
        return 1;
    }
    // A mesh file carries material names only; the textures, colours and
    // flags live in the companion .mat, and load() replaces the list by the
    // file's order rather than matching on name - so it has to run before
    // createMesh() uploads anything.
    const std::string materialFile = companionMaterialFile(meshFile);
    if (!MaterialManager::getSingleton().load(materialFile, data.materials))
        Log::warning("Bistro demo: no material file at '%s' - the scene will have no albedo",
                     materialFile.c_str());

    const AABB sourceBounds = data.bounds;
    const usize sourceSubmeshCount = data.submeshes.size();
    const usize triangleCount = data.indices.size() / 3;

    // The whole of phase 1, borrowed for a measurement: the SceneBVH indexes
    // one entry per submesh, so cutting the file's scene-wide material groups
    // into spatially local pieces is what gives the frustum test something it
    // can reject. Done here rather than in the loader on purpose - this is
    // the demo answering a question, not the engine changing for it.
    // Before the split, not after: simplifyMesh() holds submesh borders, and
    // 701 cells worth of border would lock most of the mesh in place.
    f32 simplifyError = 0.0f;
    if (simplifyRatio > 0.0f && simplifyRatio < 1.0f)
    {
        if (Assets().simplifyMesh(data, simplifyRatio, 0.01f, &simplifyError))
            Assets().optimizeVertexFetch(data);
        else
            Log::warning("Bistro demo: simplify to %.2f failed", simplifyRatio);
    }
    const usize simplifiedTriangles = data.indices.size() / 3;

    if (splitTriangles > 0)
        Assets().splitSubMeshes(data, splitTriangles);
    const usize submeshCount = data.submeshes.size();

    MeshHandle mesh = Assets().createMesh(data);
    if (!mesh.valid())
    {
        Log::error("Bistro demo: could not upload '%s'", meshFile.c_str());
        engine.shutdown();
        return 1;
    }

    const glm::vec3 center = sourceBounds.center() * kSceneScale;
    const glm::vec3 extents = sourceBounds.extents() * kSceneScale;
    const f32 radius = std::max(extents.x, std::max(extents.y, extents.z));

    GameObject* worldObject = scene->createGameObject("Bistro");
    worldObject->setScale(glm::vec3(kSceneScale));
    // Only a static renderer enters the SceneBVH - see Scene::rebuildStaticIndex().
    // Without this the whole measurement is of the unindexed path.
    worldObject->setStatic(true);
    MeshRenderer* renderer = worldObject->addComponent<MeshRenderer>();
    renderer->setMesh(mesh);

    GameObject* cameraObject = scene->createGameObject("Camera");
    Camera* camera = cameraObject->addComponent<Camera>();
    camera->setPerspective(60.0f, 16.0f / 9.0f, 0.1f, radius * 8.0f);
    cameraObject->setPosition(center + glm::vec3(0.0f, extents.y * 0.35f, -radius * 0.9f));
    cameraObject->lookAt(center);
    FreeFly* fly = cameraObject->addComponent<FreeFly>();
    fly->setMoveSpeed(radius * 0.25f);
    fly->setSprintMultiplier(4.0f);
    scene->setActiveCamera(camera);

    GameObject* sunObject = scene->createGameObject("Sun");
    DirectionalLight* sun = sunObject->addComponent<DirectionalLight>();
    sun->setColor(glm::vec3(1.0f, 0.95f, 0.85f));
    sun->setIntensity(1.0f);
    sunObject->setPosition(center + glm::vec3(-radius, radius, -radius * 0.8f));
    sunObject->lookAt(center);

    Log::info("Bistro demo: '%s' - %zu vertices, %zu triangles, %zu submeshes, %zu materials",
              meshFile.c_str(), data.positions.size(), triangleCount, sourceSubmeshCount,
              data.materials.size());
    if (simplifyRatio > 0.0f && simplifyRatio < 1.0f)
        Log::info("Bistro demo: simplified to %.2f - %zu triangles into %zu, worst deviation %.4f",
                  simplifyRatio, triangleCount, simplifiedTriangles, simplifyError);
    if (splitTriangles > 0)
        Log::info("Bistro demo: split at %u triangles - %zu submeshes into %zu", splitTriangles,
                  sourceSubmeshCount, submeshCount);
    Log::info("Bistro demo: bounds %.1f x %.1f x %.1f metres at scale %.3f",
              extents.x * 2.0f, extents.y * 2.0f, extents.z * 2.0f, kSceneScale);
    Log::info("Bistro demo: hold right to look, F1 hides the panels, F2 toggles static culling, "
              "F3 cycles the shadow cascade count, F4 toggles vsync, F5 toggles shadows");

    // The baseline runs without vsync: with it on, FPS reports the monitor
    // and not the frame's cost.
    bool vsync = false;
    engine.getWindow().setVSync(vsync);

    f32 reportTimer = 0.0f;
    bool shadowsEnabled = true;

    while (engine.update())
    {
        const f32 deltaTime = std::min(engine.getWindow().getDeltaTime(), 0.1f);

        reportTimer += deltaTime;
        const bool reportThisFrame = reportTimer >= 1.0f;
        if (reportThisFrame)
            reportTimer = 0.0f;

        if (Input::isKeyPressed(KEY_F1))
            engine.setImGuiVisible(!engine.imGuiVisible());
        if (Input::isKeyPressed(KEY_F2))
        {
            // The lever the whole baseline exists to pull: with 132 submeshes
            // spanning the scene, on and off should read the same.
            const bool enabled = !scene->staticCullingEnabled();
            scene->setStaticCullingEnabled(enabled);
            Log::info("Bistro demo: static culling %s", enabled ? "on" : "off");
        }
        if (Input::isKeyPressed(KEY_F3))
        {
            if (CascadeShadowSettings* shadows = engine.cascadeSettings())
            {
                shadows->count = shadows->count > 1 ? shadows->count - 1 : 4;
                Log::info("Bistro demo: %u shadow cascades", shadows->count);
            }
        }
        if (Input::isKeyPressed(KEY_F4))
        {
            vsync = !vsync;
            engine.getWindow().setVSync(vsync);
            Log::info("Bistro demo: vsync %s", vsync ? "on" : "off");
        }
        if (Input::isKeyPressed(KEY_F5))
        {
            shadowsEnabled = !shadowsEnabled;
            sun->setCastShadows(shadowsEnabled);
            Log::info("Bistro demo: sun shadows %s", shadowsEnabled ? "on" : "off");
        }

        scene->update(deltaTime);
        engine.render(*scene);

        // After render(), not at the top of the loop: Engine::update() calls
        // GPU::beginFrame(), which zeroes GPUStats - reading them before the
        // frame draws anything reports zeros.
        if (reportThisFrame)
        {
            const GPUStats& gpu = GPU::getSingleton().stats();
            const RenderListStats* view = engine.mainRenderListStats();
            const RenderListStats* shadowList = engine.shadowListStats();
            const CascadeShadowSettings* shadows = engine.cascadeSettings();
            Log::info("%d FPS  CPU %.2f ms  GPU %.2f ms | draws %u  tris %u  binds %u | "
                      "culling %s cascades %u | packets %u submitted %u culled %u/%u | "
                      "shadow packets %u",
                      engine.getWindow().getFPS(),
                      Profiler::getSingleton().frameMilliseconds(), gpu.gpuMilliseconds,
                      gpu.drawCalls, gpu.triangles, gpu.textureBinds,
                      scene->staticCullingEnabled() ? "on" : "off", shadows ? shadows->count : 0u,
                      view ? view->packets : 0u, view ? view->submitted : 0u,
                      view ? view->culledMeshes : 0u, view ? view->culledSubmeshes : 0u,
                      shadowList ? shadowList->packets : 0u);

            // Where those GPU milliseconds went. Engine::renderInternal()
            // already brackets every phase with RADION_GPU_PROFILE_SCOPE; the
            // panel shows the table and this puts the same numbers in the log,
            // which is what a run measured from outside can quote.
            const GPUProfiler& gpuProfiler = GPUProfiler::getSingleton();
            std::string breakdown;
            for (u32 index = 0; index < gpuProfiler.sampleCount(); ++index)
            {
                const ProfileSample& sample = gpuProfiler.samples()[index];
                if (sample.milliseconds < 0.05f)
                    continue;
                char entry[64];
                std::snprintf(entry, sizeof(entry), "  %s %.2f", sample.name.c_str(),
                              sample.milliseconds);
                breakdown += entry;
            }
            if (!breakdown.empty())
                Log::info("        GPU:%s", breakdown.c_str());
        }

        engine.flip();
    }

    engine.shutdown();
    return 0;
}
