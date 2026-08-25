#include "PCH.h"

#include "EditorApplication.h"

#include "AssetManager.h"
#include "AsyncTextureLoader.h"
#include "Camera.h"
#include "DebugDraw3D.h"
#include "EditorPanel.h"
#include "EditorTheme.h"
#include "Engine.h"
#include "FileSystem.h"
#include "GameObject.h"
#include "Light.h"
#include "Lighting.h"
#include "Log.h"
#include "MaterialManager.h"
#include "MeshRenderer.h"
#include "ParticlePass.h"
#include "Scene.h"
#include "ScriptCache.h"
#include "Shadows.h"
#include "panels/AnimationPanel.h"
#include "panels/AssetsPanel.h"
#include "panels/ConsolePanel.h"
#include "panels/DebugPanel.h"
#include "panels/GamePanel.h"
#include "panels/HierarchyPanel.h"
#include "panels/InspectorPanel.h"
#include "panels/LightmapPanel.h"
#include "panels/MeshToolsPanel.h"
#include "panels/ProfilerPanel.h"
#include "panels/SettingsPanel.h"
#include "panels/ViewportPanel.h"
#include "panels/VolumePanel.h"

#include <IconsMaterialDesignIcons.h>
#include <cstring>
#include <filesystem>
#include <imgui.h>
#include <imgui_internal.h> // DockBuilder* - building the first-run default layout
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>
#include <unordered_map>

#ifndef _WIN32
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace Radion
{

namespace
{
// ImGui's .ini keys docking on these - constant across sessions, per
// PLANO_EDITOR.md's layout note. Never renamed casually.
constexpr const char* kHierarchyWindow = "Hierarchy";
constexpr const char* kViewportWindow = "Viewport";
constexpr const char* kGameWindow = "Game";
constexpr const char* kInspectorWindow = "Inspector";
constexpr const char* kAssetsWindow = "Assets";
constexpr const char* kConsoleWindow = "Console";
constexpr const char* kAnimationWindow = "Animation";
constexpr const char* kDebugWindow = "Debug";
constexpr const char* kProfilerWindow = "Profiler";
constexpr const char* kDockspaceId = "EditorDockspace";

void accumulateMeshBoundsRecursive(GameObject& object, AABB& bounds)
{
    if (object.active() && object.isVisibleInHierarchy())
        if (MeshRenderer* renderer = object.getComponent<MeshRenderer>())
            if (const Mesh* mesh = Assets().getMesh(renderer->mesh()))
                bounds.merge(transformAABB(mesh->bounds, object.globalTransform()));
    for (usize i = 0; i < object.childCount(); ++i)
        accumulateMeshBoundsRecursive(*object.child(i), bounds);
}

bool sameAuthoredMaterial(const Material& a, const Material& b)
{
    if (a.flags != b.flags || a.blend != b.blend || a.cull != b.cull || a.name != b.name ||
        a.nameHash != b.nameHash ||
        std::memcmp(&a.params, &b.params, sizeof(MaterialParams)) != 0 ||
        a.animCount != b.animCount)
        return false;
    for (u32 slot = 0; slot < MaterialSlotCount; ++slot)
    {
        const MaterialTexture& ta = a.textures[slot];
        const MaterialTexture& tb = b.textures[slot];
        if (ta.file != tb.file || ta.source != tb.source || ta.layers != tb.layers ||
            ta.targetName != tb.targetName)
            return false;
    }
    for (u8 i = 0; i < a.animCount; ++i)
    {
        const MaterialAnim& aa = a.anims[i];
        const MaterialAnim& ab = b.anims[i];
        if (aa.field != ab.field || aa.mask != ab.mask || aa.curve != ab.curve ||
            aa.speed != ab.speed || aa.phase != ab.phase || aa.min != ab.min || aa.max != ab.max)
            return false;
    }
    return true;
}

std::filesystem::path materialSidecarFor(const std::filesystem::path& meshPath)
{
    const std::filesystem::path stem = meshPath.parent_path() / meshPath.stem();
    const std::filesystem::path modern = stem.string() + ".material";
    const std::filesystem::path legacy = stem.string() + ".mat";
    FileSystem& files = FileSystem::getSingleton();
    if (files.exists(modern.string()))
        return modern;
    if (files.exists(legacy.string()))
        return legacy;
    return modern; // AssetMesh tries .material first too.
}

struct MaterialSaveBatch
{
    Mesh* mesh = nullptr;
    std::filesystem::path sidecar;
    std::vector<Material> original;
    std::vector<Material> merged;
    std::vector<bool> changedSlots;
    std::vector<MeshRenderer*> renderers;
};

bool collectMaterialOverridesRecursive(GameObject& object,
                                       std::unordered_map<u64, usize>& batchByMesh,
                                       std::vector<MaterialSaveBatch>& batches)
{
    bool ok = true;
    if (MeshRenderer* renderer = object.getComponent<MeshRenderer>())
    {
        const u32 overrideCount = renderer->materialOverrideCount();
        if (overrideCount > 0)
        {
            const MeshDesc& desc = Assets().meshDesc(renderer->mesh());
            Mesh* mesh = Assets().getMesh(renderer->mesh());
            if (!mesh)
            {
                Log::error(
                    "EditorApplication: cannot save material override for '%s' - mesh not loaded",
                    object.name().c_str());
                ok = false;
            }
            else if (desc.source != MeshSource::File || desc.file.empty())
            {
            }
            else if (overrideCount > mesh->materials.size())
            {
                Log::error(
                    "EditorApplication: material override on '%s' has %u slots, mesh only has %zu",
                    object.name().c_str(), overrideCount, mesh->materials.size());
                ok = false;
            }
            else
            {
                const u64 key =
                    (static_cast<u64>(renderer->mesh().index) << 32) | renderer->mesh().generation;
                auto found = batchByMesh.find(key);
                if (found == batchByMesh.end())
                {
                    const std::string resolved = FileSystem::getSingleton().resolve(desc.file);
                    const std::filesystem::path meshPath = resolved.empty()
                                                               ? std::filesystem::path(desc.file)
                                                               : std::filesystem::path(resolved);
                    MaterialSaveBatch batch;
                    batch.mesh = mesh;
                    batch.sidecar = materialSidecarFor(meshPath);
                    batch.original = mesh->materials;
                    batch.merged = mesh->materials;
                    batch.changedSlots.resize(mesh->materials.size(), false);
                    batches.push_back(std::move(batch));
                    found = batchByMesh.emplace(key, batches.size() - 1).first;
                }

                MaterialSaveBatch& batch = batches[found->second];
                const Material* overrides = renderer->materialOverrides();
                for (u32 slot = 0; slot < overrideCount; ++slot)
                {
                    if (sameAuthoredMaterial(overrides[slot], batch.original[slot]))
                        continue;
                    if (batch.changedSlots[slot] &&
                        !sameAuthoredMaterial(overrides[slot], batch.merged[slot]))
                    {
                        Log::error("EditorApplication: conflicting material edits for mesh '%s', "
                                   "slot %u; overrides kept",
                                   desc.file.c_str(), slot);
                        ok = false;
                        continue;
                    }
                    batch.merged[slot] = overrides[slot];
                    batch.changedSlots[slot] = true;
                }
                batch.renderers.push_back(renderer);
            }
        }
    }
    for (usize i = 0; i < object.childCount(); ++i)
        ok = collectMaterialOverridesRecursive(*object.child(i), batchByMesh, batches) && ok;
    return ok;
}
} // namespace

EditorApplication* EditorApplication::sInstance = nullptr;

void EditorApplication::logSink(LogLevel level, const char* message)
{
    ConsolePanel::pushEntry(level, message);
    if (!sInstance || !message)
        return;
    // Info and Debug stay in the console: a toast for each would bury the
    // ones that need answering under the running commentary of a scene load.
    if (level == LOG_ERROR)
        sInstance->mToasts.error(message);
    else if (level == LOG_WARNING)
        sInstance->mToasts.warning(message);
}

EditorApplication::EditorApplication(Engine& engine) : mEngine(engine)
{
    sInstance = this;
    Log::setMode(LogMode::Verbose);
    Log::setSink(&EditorApplication::logSink);

    // Preferences must exist before panels are constructed: their
    // constructors copy the persisted viewport/tool state into live fields.
    mSettingsFile =
        FileSystem::getSingleton().prefPath("Radion", "Editor") + "editor.settings.json";
    mSettings.load(mSettingsFile);
    mCursor3D = mSettings.cursor3D;
    mShowStatsOverlay = mSettings.showStatsOverlay;
    mShowDynamicIndexDebug = mSettings.showDynamicIndexDebug;
    mShowOcclusionDebug = mSettings.showOcclusionDebug;
    mShowSubmeshBounds = mSettings.showSubmeshBounds;
    mEngine.debugShowShadowCascades = mSettings.showShadowCascades;
    mEngine.debugShowShadowAtlas = mSettings.showShadowAtlas;
    mViewMode = mSettings.viewMode == 1 ? ViewMode::Game : ViewMode::Scene;

    mEngine.createScene();
    scene().setRunningInEditor(true);

    // Automatic coalesces scene invalidations and captures once after the
    // edits settle. Timed remains available explicitly in Settings for work
    // that really needs continuously refreshed reflections.
    EnvironmentProbe& probe = mEngine.environmentProbe();
    probe.create(64);
    probe.refresh = EnvironmentProbe::Refresh::Automatic;

    buildDefaultScene();
    buildPanels();

    if (mSettings.themeIndex < 0 || mSettings.themeIndex >= kEditorThemeCount)
        mSettings.themeIndex = 0;
    applyEditorTheme(static_cast<EditorThemeKind>(mSettings.themeIndex));
    if (!mSettings.lastProjectPath.empty())
    {
        mStartupLoadPending = true;
        mStartupLoadIsProject = true;
        mStartupLoadPath = mSettings.lastProjectPath;
    }
    else if (!mSettings.lastScenePath.empty())
    {
        mStartupLoadPending = true;
        mStartupLoadPath = mSettings.lastScenePath;
    }
}

void EditorApplication::startDeferredStartupLoad()
{
    if (!mStartupLoadPending || mStartupLoadStarted)
        return;
    mStartupLoadStarted = true;
    mStartupLoading = true;
    const bool loaded = mStartupLoadIsProject ? openProject(mStartupLoadPath)
                                              : openScene(mStartupLoadPath);
    if (!loaded)
    {
        if (mStartupLoadIsProject)
            mSettings.lastProjectPath.clear();
        else
            mSettings.lastScenePath.clear();
    }
    mStartupLoading = false;
    mStartupLoadPending = false;
}

EditorApplication::~EditorApplication()
{
    // Shutdown logs after this object is gone otherwise reach mToasts.
    sInstance = nullptr;
    mSettings.cursor3D = mCursor3D;
    mSettings.viewMode = mViewMode == ViewMode::Game ? 1 : 0;
    mSettings.showStatsOverlay = mShowStatsOverlay;
    mSettings.showDynamicIndexDebug = mShowDynamicIndexDebug;
    mSettings.showOcclusionDebug = mShowOcclusionDebug;
    mSettings.showSubmeshBounds = mShowSubmeshBounds;
    mSettings.showShadowCascades = mEngine.debugShowShadowCascades;
    mSettings.showShadowAtlas = mEngine.debugShowShadowAtlas;
    for (EditorPanel* panel : mPanels)
        mSettings.panelOpen[panel->title()] = panel->active();
    mSettings.save(mSettingsFile);
    for (EditorPanel* panel : mPanels)
        delete panel;
}

void EditorApplication::buildPanels()
{
    mPanels.push_back(new HierarchyPanel(*this));
    mPanels.push_back(new ViewportPanel(*this));
    mPanels.push_back(new GamePanel(*this));
    mPanels.push_back(new InspectorPanel(*this));
    mPanels.push_back(new SettingsPanel(*this));
    mPanels.push_back(new AssetsPanel(*this));
    mPanels.push_back(new MeshToolsPanel(*this));
    mPanels.push_back(new LightmapPanel(*this));
    mPanels.push_back(new VolumePanel(*this));
    mPanels.push_back(new ConsolePanel(*this));
    mPanels.push_back(new AnimationPanel(*this));
    mPanels.push_back(new DebugPanel(*this));
    mPanels.push_back(new ProfilerPanel(*this));
    for (EditorPanel* panel : mPanels)
    {
        const auto saved = mSettings.panelOpen.find(panel->title());
        const bool optionalTool = panel->title() == "Lightmap" || panel->title() == "Volume" ||
                                  panel->title() == "Mesh Tools";
        // The first dock-layout migration also establishes the quieter
        // default: these heavyweight tools are available from Windows, but
        // do not open automatically.
        if (mSettings.dockLayoutVersion < 1 && optionalTool)
            panel->setActive(false);
        else if (saved != mSettings.panelOpen.end())
            panel->setActive(saved->second);
        else if (optionalTool)
            // These tools remain available from Windows, but stay out of the
            // initial layout until explicitly enabled by the user.
            panel->setActive(false);
    }
}

void EditorApplication::buildDefaultScene()
{

    GameObject* cameraObject = scene().createGameObject("Camera");
    Camera* camera = cameraObject->addComponent<Camera>();
    cameraObject->setPosition(glm::vec3(0.0f, 4.0f, 10.0f));
    cameraObject->lookAt(glm::vec3(0.0f, 0.0f, 0.0f));
    camera->setPerspective(60.0f, 16.0f / 9.0f, 0.1f, 1000.0f);
    scene().setActiveCamera(camera);

    GameObject* sunObject = scene().createGameObject("Sun");
    DirectionalLight* sun = sunObject->addComponent<DirectionalLight>();
    sun->setColor(glm::vec3(1.0f, 0.96f, 0.88f));
    sun->setCastShadows(true);

    scene().update(0.0f);
}

void EditorApplication::replaceScene(Scene* replacement)
{
    replacement->setRunningInEditor(true);
    mEngine.setActiveScene(replacement);
    mSelection.clear();
    // Both of these name things in the scene being replaced - an object id
    // and submesh indices into a mesh that is about to be a different one.
    // Carried over, the next Delete acts on whatever now happens to sit at
    // those indices.
    mSubmeshSelection.object = 0;
    mSubmeshSelection.indices.clear();
    mPickedSubmesh = PickedSubmesh();
    scene().rebuildStaticIndex();
    SceneRenderSettings settings = sceneRenderSettings();
    mRenderSettingsSnapshot = mSerializer.renderSettingsToJson(settings).dump();
}

void EditorApplication::markDirty()
{
    mDirty = true;
    mEngine.environmentProbe().invalidate();

    if (Scene* active = mEngine.activeScene())
        for (ReflectionProbe* probe : active->reflectionProbes())
            probe->probe().invalidate();
}

SceneRenderSettings EditorApplication::sceneRenderSettings()
{
    return {mEngine.cascadeSettings(),
            mEngine.lighting() ? &mEngine.lighting()->atlasSettings() : nullptr,
            &mEngine.postProcess(),
            mEngine.lensFlare(),
            &mEngine.environmentProbe(),
            mEngine.lighting(),
            mEngine.volumetric(),
            &mEngine.sky(),
            &mEngine.renderResolution(),
            &ParticleDraws()};
}

void EditorApplication::recordUndo()
{
    if (mUndoPosition < mUndoStates.size())
        mUndoStates.resize(mUndoPosition);
    SceneRenderSettings settings = sceneRenderSettings();
    UndoState state;
    state.scene = mSerializer.toJson(scene(), &settings);
    mUndoStates.push_back(std::move(state));
    mUndoPosition = mUndoStates.size();
}

void EditorApplication::recordMeshUndo(MeshHandle handle)
{
    const MeshData* data = importedMeshData(handle);
    if (!data)
        return;
    recordUndo();
    UndoState& state = mUndoStates.back();
    state.mesh = handle;
    state.submeshes = data->submeshes;
    state.hasMeshEdit = true;
}

std::string EditorApplication::nextIncrementedName(const std::string& name)
{
    usize digitStart = name.size();
    while (digitStart > 0 && name[digitStart - 1] >= '0' && name[digitStart - 1] <= '9')
        --digitStart;

    std::string base = name.substr(0, digitStart);
    u64 counter = 0;
    usize width = 0;
    if (digitStart < name.size())
    {
        const std::string digits = name.substr(digitStart);
        width = digits.size();
        counter = std::strtoull(digits.c_str(), nullptr, 10);
    }
    else
    {
        base += '_';
        width = 2;
    }

    // The scene may already hold the next few numbers (duplicating the same
    // source twice, or a hand-named object sitting on the number) - walk
    // forward until one is actually free instead of handing back a duplicate.
    for (;;)
    {
        ++counter;
        std::string digits = std::to_string(counter);
        while (digits.size() < width)
            digits.insert(digits.begin(), '0');
        std::string candidate = base + digits;
        if (!scene().findGameObject(candidate))
            return candidate;
    }
}

GameObject* EditorApplication::duplicateObject(GameObject& source)
{
    recordUndo();
    SceneLoadResult result;
    GameObject* clone = mSerializer.cloneObject(source, scene(), source.parent(), result);
    if (!clone)
    {
        for (const SceneDiagnostic& diagnostic : result.diagnostics)
            Log::error("EditorApplication: duplicate failed - %s: %s", diagnostic.jsonPath.c_str(),
                       diagnostic.message.c_str());
        return nullptr;
    }

    clone->setPosition(source.position() + glm::vec3(0.5f, 0.0f, 0.5f));
    clone->setName(nextIncrementedName(source.name()));
    selection().select(clone->id());
    markDirty();
    return clone;
}

bool EditorApplication::duplicateObjectGrid(GameObject& source, bool alongX, bool alongZ,
                                            u32 countX, u32 countZ, f32 spacing)
{
    const u32 cellsX = alongX ? glm::max(countX, 1u) : 1u;
    const u32 cellsZ = alongZ ? glm::max(countZ, 1u) : 1u;
    if (cellsX * cellsZ <= 1)
        return false;

    recordUndo();
    const glm::vec3 base = source.position();
    for (u32 z = 0; z < cellsZ; ++z)
        for (u32 x = 0; x < cellsX; ++x)
        {
            if (x == 0 && z == 0)
                continue;
            SceneLoadResult result;
            GameObject* clone = mSerializer.cloneObject(source, scene(), source.parent(), result);
            if (!clone)
            {
                for (const SceneDiagnostic& diagnostic : result.diagnostics)
                    Log::error("EditorApplication: grid duplicate failed - %s: %s",
                               diagnostic.jsonPath.c_str(), diagnostic.message.c_str());
                markDirty();
                return false;
            }
            clone->setPosition(base + glm::vec3(static_cast<f32>(x) * spacing, 0.0f,
                                                static_cast<f32>(z) * spacing));
            clone->setName(nextIncrementedName(source.name()));
        }
    markDirty();
    return true;
}

void EditorApplication::restoreMeshEdit(const UndoState& state)
{
    if (!state.hasMeshEdit)
        return;
    MeshData* data = importedMeshData(state.mesh);
    if (!data)
        return;
    data->submeshes = state.submeshes;
    applyMeshEdit(state.mesh);
    // The indices it held name submeshes that have just moved (or come
    // back); keeping them would point the next Delete at the wrong pieces.
    mSubmeshSelection.indices.clear();
}

void EditorApplication::undo()
{
    if (mUndoPosition == 0)
        return;
    if (mUndoPosition == mUndoStates.size())
    {
        SceneRenderSettings settings = sceneRenderSettings();
        UndoState current;
        current.scene = mSerializer.toJson(scene(), &settings);
        // The step being left behind carries today's submesh table, so redo
        // can put the deletion back after this undo removes it.
        if (mUndoStates.back().hasMeshEdit)
            if (const MeshData* data = importedMeshData(mUndoStates.back().mesh))
            {
                current.mesh = mUndoStates.back().mesh;
                current.submeshes = data->submeshes;
                current.hasMeshEdit = true;
            }
        mUndoStates.push_back(std::move(current));
    }
    --mUndoPosition;
    Scene* restored = new Scene();
    SceneLoadResult result;
    SceneRenderSettings settings = sceneRenderSettings();
    if (mSerializer.fromJson(mUndoStates[mUndoPosition].scene, *restored, result, &settings))
    {
        replaceScene(restored);
        restoreMeshEdit(mUndoStates[mUndoPosition]);
    }
    else
        delete restored;
}

void EditorApplication::redo()
{
    if (mUndoPosition + 1 >= mUndoStates.size())
        return;
    ++mUndoPosition;
    Scene* restored = new Scene();
    SceneLoadResult result;
    SceneRenderSettings settings = sceneRenderSettings();
    if (mSerializer.fromJson(mUndoStates[mUndoPosition].scene, *restored, result, &settings))
    {
        replaceScene(restored);
        restoreMeshEdit(mUndoStates[mUndoPosition]);
    }
    else
        delete restored;
}

void EditorApplication::newScene()
{
    Scene* fresh = new Scene();
    replaceScene(fresh);
    // Without this the fresh scene has no active camera and no light - the
    // Viewport/Game panels both bail out with nothing to render ("black
    // screen") until the user adds one by hand.
    buildDefaultScene();
    mScenePath.clear();
    mDirty = false;
    mLastDiagnostics.clear();
    if (!mProjectRoot.empty())
    {
        std::filesystem::path scenePath = mProjectRoot / "Scenes" / "NewScene.scene.json";
        for (u32 index = 2; std::filesystem::exists(scenePath); ++index)
            scenePath =
                mProjectRoot / "Scenes" / ("NewScene" + std::to_string(index) + ".scene.json");
        saveSceneAs(scenePath.string());
    }
}

bool EditorApplication::openScene(const std::string& path)
{
    return loadScene(path, true);
}

bool EditorApplication::loadScene(const std::string& path, bool addToProject)
{
    SceneLoadResult result;
    if (!mEngine.sceneManager().load(path, result))
    {
        mLastDiagnostics = result.diagnostics;
        return false;
    }
    mLastDiagnostics = result.diagnostics;
    mSelection.clear();
    scene().rebuildStaticIndex();
    mScenePath = path;
    mDirty = false;
    mSettings.lastScenePath = path;
    mSettings.lastOpenDirectory = std::filesystem::path(path).parent_path().string();
    SceneRenderSettings settings = sceneRenderSettings();
    mRenderSettingsSnapshot = mSerializer.renderSettingsToJson(settings).dump();
    if (addToProject)
        addSceneToProject(path);
    return true;
}

void EditorApplication::fitShadowsToScene()
{
    CascadeShadowSettings* shadows = mEngine.cascadeSettings();
    if (!shadows)
        return;

    AABB bounds;
    accumulateMeshBoundsRecursive(scene().root(), bounds);

    const f32 radius = bounds.empty() ? 1.0f : bounds.radius();

    // Texel density depends on the camera's own field of view and on the sun's
    // angle, so the fit is solved against the real ones rather than a guess.
    ShadowCamera camera;
    if (Camera* active = scene().activeCamera(); active && active->owner())
    {
        camera.view = active->viewMatrix();
        camera.fieldOfView = active->fieldOfView();
        camera.aspect = active->aspect();
        camera.nearPlane = active->nearPlane();
    }

    glm::vec3 lightDirection = -mEngine.sky().sunDirection;
    if (DirectionalLight* sun = scene().electedSunLight(); sun && sun->owner())
        lightDirection = sun->owner()->forward();

    *shadows = CascadeShadowSettings::sizedForCamera(*shadows, radius, camera, lightDirection);
}

bool EditorApplication::saveScene()
{
    if (mScenePath.empty())
        return false; // caller should route to Save As instead
    return saveSceneAs(mScenePath);
}

void EditorApplication::play()
{
    if (mPlaying)
        return;
    SceneRenderSettings settings = sceneRenderSettings();
    mEditSnapshot = mSerializer.toJson(scene(), &settings);

    // A compiled script is kept for the life of the process, so editing a
    // .py in another window and pressing Play would otherwise run the
    // version from before the edit. Here is the one place worth paying a
    // stat per cached script for.
    if (const int reloaded = ScriptCache::getSingleton().refreshChangedFiles())
        Log::info("EditorApplication: recompiled %d changed script(s)", reloaded);

    mPlaying = true;
    scene().setRunningInEditor(false);
    mViewMode = ViewMode::Game;
    ImGui::SetWindowFocus(kGameWindow);
}

void EditorApplication::stop()
{
    if (!mPlaying)
        return;
    Scene* restored = new Scene();
    SceneLoadResult result;
    SceneRenderSettings settings = sceneRenderSettings();
    if (!mSerializer.fromJson(mEditSnapshot, *restored, result, &settings))
    {
        // The snapshot came from this same serializer moments ago - a
        // failure here means Play captured something Stop cannot rebuild,
        // which is a real bug, not a user mistake. Keep the play-mode scene
        // rather than dropping it silently into a broken restore.
        Log::error("EditorApplication: could not restore the pre-Play snapshot, staying in Play");
        delete restored;
        return;
    }
    replaceScene(restored);
    mPlaying = false;
    mEditSnapshot = nlohmann::json();
    mViewMode = ViewMode::Scene;
    ImGui::SetWindowFocus(kViewportWindow);
}

bool EditorApplication::saveSceneAs(const std::string& path)
{
    std::unordered_map<u64, usize> batchByMesh;
    std::vector<MaterialSaveBatch> batches;
    if (!collectMaterialOverridesRecursive(scene().root(), batchByMesh, batches))
    {
        // A failed sidecar write must never make the editor consider this
        // state saved.  In particular, keep every override alive so the user
        // can fix the path/permissions and retry without recreating edits.
        mDirty = true;
        Log::error(
            "EditorApplication: scene not saved because one or more material sidecars failed");
        return false;
    }

    // Apply every compatible instance edit before writing any sidecar. Slots
    // edited on different instances merge; two different edits to the same
    // shared slot fail above rather than choosing one silently.
    for (MaterialSaveBatch& batch : batches)
    {
        for (usize slot = 0; slot < batch.changedSlots.size(); ++slot)
        {
            if (!batch.changedSlots[slot])
                continue;
            MaterialManager::getSingleton().release(batch.mesh->materials[slot]);
            batch.mesh->materials[slot] = batch.merged[slot];
            batch.mesh->materials[slot].paramsBuffer = BufferHandle();
            batch.mesh->materials[slot].pipeline = PipelineHandle();
            batch.mesh->materials[slot].paramsDirty = true;
        }
    }

    bool materialsSaved = true;
    for (const MaterialSaveBatch& batch : batches)
        materialsSaved =
            MaterialManager::getSingleton().save(batch.sidecar.string(), batch.merged) &&
            materialsSaved;
    if (!materialsSaved)
    {
        mDirty = true;
        Log::error(
            "EditorApplication: scene not saved because one or more material sidecars failed");
        return false;
    }
    if (!mEngine.sceneManager().save(path))
    {
        // Sidecars may already have reached disk, but the save transaction is
        // not complete until the scene itself succeeds.  Overrides remain the
        // authoritative in-memory copy until then and are deliberately not
        // cleared here.
        mDirty = true;
        Log::error("EditorApplication: scene save failed; material overrides were kept");
        return false;
    }

    // Commit: only now have all material sidecars and the scene succeeded.
    for (const MaterialSaveBatch& batch : batches)
        for (MeshRenderer* renderer : batch.renderers)
            renderer->clearMaterialOverrides();
    // Rendering now reads mesh->materials instead of the overrides that were
    // just dropped, and those went in as plain data - a merged material can
    // be a different shader variant from the one it replaced (a baked
    // lightmap is exactly that), and a static submesh otherwise keeps the
    // pipeline the BVH cached before the swap. Without this the object turns
    // black on save, because emitSubmesh() drops a submesh whose pipeline
    // does not match. Same reason applyMeshEdit() rebuilds the index.
    if (!batches.empty())
    {
        for (const MaterialSaveBatch& batch : batches)
            if (batch.mesh)
                for (Material& material : batch.mesh->materials)
                    MaterialManager::getSingleton().resolvePipeline(material,
                                                                    batch.mesh->colorLayout);
        scene().rebuildStaticIndex();
        scene().rebuildDynamicIndex();
    }
    mScenePath = path;
    mDirty = false;
    mSettings.lastScenePath = path;
    mSettings.lastSaveDirectory = std::filesystem::path(path).parent_path().string();
    addSceneToProject(path);
    mToasts.success("Scene saved: " + FileSystem::fileName(path));
    return true;
}

bool EditorApplication::saveProject()
{
    if (mProjectManifest.empty())
        return false;
    nlohmann::json manifest;
    manifest["format"] = "radion-project";
    manifest["version"] = 1;
    manifest["name"] = mProjectRoot.filename().string();
    manifest["assetRoot"] = mProjectAssetRoot;
    manifest["scenes"] = mProjectScenes;
    manifest["activeScene"] = mProjectActiveScene;
    manifest["searchPaths"] = mExtraSearchPaths;
    return FileSystem::getSingleton().writeText(mProjectManifest.string(), manifest.dump(4) + '\n');
}

void EditorApplication::addProjectSearchPath(const std::string& path)
{
    if (path.empty())
        return;
    // Compared normalized, not as raw strings: the same folder reached with
    // or without a trailing separator (or added twice from two different
    // callers) is one search path, not two - an exact-string compare missed
    // that and let the sidebar show the same folder listed twice.
    const std::filesystem::path normalized = std::filesystem::path(path).lexically_normal();
    for (const std::string& existing : mExtraSearchPaths)
        if (std::filesystem::path(existing).lexically_normal() == normalized)
            return;
    mExtraSearchPaths.push_back(path);
    FileSystem::getSingleton().addSearchPath(path);
    if (!saveProject())
        Log::warning("EditorApplication: search path '%s' added for this session only - no "
                     "project is open to save it into",
                     path.c_str());
}

void EditorApplication::removeProjectSearchPath(usize index)
{
    if (index >= mExtraSearchPaths.size())
        return;
    FileSystem::getSingleton().removeSearchPath(mExtraSearchPaths[index]);
    mExtraSearchPaths.erase(mExtraSearchPaths.begin() + static_cast<std::ptrdiff_t>(index));
    saveProject();
}

std::string EditorApplication::assetBrowserRoot() const
{
    return mProjectAssetSearchPath.empty() ? std::string(RADION_ASSET_DIR)
                                           : mProjectAssetSearchPath;
}

void EditorApplication::browseAddSearchPath()
{
    openFileDialog(ImGuiFileDialog::Mode::ChooseFolder, FileDialogAddSearchPath,
                   mSettings.lastOpenDirectory);
}

namespace
{
u64 packMeshHandle(MeshHandle handle)
{
    return (static_cast<u64>(handle.index) << 32) | handle.generation;
}
} // namespace

MeshData* EditorApplication::importedMeshData(MeshHandle handle)
{
    const auto it = mImportedMeshData.find(packMeshHandle(handle));
    if (it != mImportedMeshData.end())
        return &it->second;

    // A mesh that arrived with a saved scene never went through the import
    // popup, so nothing put a CPU copy here - and every mesh tool, the
    // submesh delete and the lightmap unwrap all need one. Read back from
    // the recipe the scene stored, on first use rather than at load: doing
    // it for the whole scene up front would read every mesh file twice and
    // hold all of it in memory, most of which is never edited.
    const MeshDesc& desc = Assets().meshDesc(handle);
    if (desc.source != MeshSource::File || desc.file.empty())
        return nullptr;
    MeshData data;
    if (!Assets().importMeshFileData(desc.file, data))
    {
        Log::error("EditorApplication: '%s' could not be read back, so its mesh cannot be edited",
                   desc.file.c_str());
        return nullptr;
    }
    Log::info("EditorApplication: cached CPU data for '%s' (%zu vertices)", desc.file.c_str(),
              data.positions.size());
    mImportedMeshData[packMeshHandle(handle)] = std::move(data);
    return &mImportedMeshData[packMeshHandle(handle)];
}

void EditorApplication::registerImportedMesh(MeshHandle handle, MeshData data)
{
    mImportedMeshData[packMeshHandle(handle)] = std::move(data);
}

GameObject* EditorApplication::adoptGeneratedMesh(const std::string& name,
                                                  const std::string& outputBase, MeshData data)
{
    if (data.positions.empty() || data.indices.empty())
    {
        toasts().error("Nothing to build - the volume produced no geometry");
        return nullptr;
    }

    const std::string meshOutput = outputBase + ".rmesh";
    if (!Assets().saveMesh(data, meshOutput, std::string()))
    {
        Log::error("EditorApplication: could not write '%s'", meshOutput.c_str());
        toasts().error("Could not write " + FileSystem::fileName(meshOutput));
        return nullptr;
    }
    if (!data.materials.empty() &&
        !MaterialManager::getSingleton().save(outputBase + ".material", data.materials))
        Log::error("EditorApplication: could not write '%s'", (outputBase + ".material").c_str());

    addProjectSearchPath(FileSystem::directoryOf(meshOutput));

    const MeshHandle mesh = Assets().createMesh(data);
    if (!mesh.valid())
    {
        toasts().error("Could not upload " + name);
        return nullptr;
    }
    Assets().registerMeshDesc(mesh, MeshDesc::fromFile(meshOutput));
    registerImportedMesh(mesh, std::move(data));

    recordUndo();
    GameObject* object = scene().createGameObject(name, nullptr);
    if (!object)
        return nullptr;
    object->setPosition(cursor3D());
    MeshRenderer* renderer = object->addComponent<MeshRenderer>();
    renderer->setMesh(mesh);
    scene().update(0.0f);
    selection().select(object->id());
    // Same reason replaceScene() clears these: they index a mesh that is not
    // the one now in front of the user.
    mSubmeshSelection.object = 0;
    mSubmeshSelection.indices.clear();
    markDirty();
    toasts().success("Built " + FileSystem::fileName(meshOutput));
    return object;
}

bool EditorApplication::applyMeshEdit(MeshHandle handle)
{
    MeshData* data = importedMeshData(handle);
    if (!data || !Assets().replaceMesh(handle, *data))
        return false;
    // replaceMesh() hands back a brand new Mesh - same handle, but a fresh
    // Material array with every pipeline unresolved. Static geometry's
    // pipelines are only ever resolved once, up front, by rebuildStaticIndex()
    // (see its own call site's comment and Scene::buildShadowList()'s) - left
    // uncalled here, every submesh on this mesh keeps whatever pipeline
    // state the BVH cached from before the edit, which for an alpha-tested
    // material (a shader variant the old, now-gone Material object does not
    // share with the new one) is an invalid one: emitSubmesh() drops it, and
    // the submesh renders as if deleted.
    scene().rebuildStaticIndex();
    scene().rebuildDynamicIndex();
    markDirty();
    return true;
}

void EditorApplication::deleteSubmeshSelection()
{
    if (mSubmeshSelection.indices.empty())
        return;
    GameObject* object = scene().findGameObject(mSubmeshSelection.object);
    MeshRenderer* renderer = object ? object->getComponent<MeshRenderer>() : nullptr;
    MeshData* data = renderer ? importedMeshData(renderer->mesh()) : nullptr;
    if (!renderer || !data)
    {
        mSubmeshSelection.indices.clear();
        return;
    }
    recordMeshUndo(renderer->mesh());
    // Highest index first: removing one shifts every index past it down by
    // one, so the loop's own remaining indices must not be read after that
    // happens to them.
    std::vector<u32> sorted = mSubmeshSelection.indices;
    std::sort(sorted.begin(), sorted.end(), std::greater<u32>());
    u32 removed = 0;
    for (u32 index : sorted)
        if (AssetManager::getSingleton().removeSubmesh(*data, index))
            ++removed;
    renderer->setHiddenSubmeshes({});
    applyMeshEdit(renderer->mesh());
    markDirty();
    mSubmeshSelection.indices.clear();
    Log::info("EditorApplication: deleted %u selected submesh(es)", removed);
}

void EditorApplication::requestFocusObject(u64 objectId)
{
    mFocusObjectRequest = objectId;
}

u64 EditorApplication::takeFocusObjectRequest()
{
    const u64 id = mFocusObjectRequest;
    mFocusObjectRequest = 0;
    return id;
}

void EditorApplication::requestRevealAsset(const std::string& path)
{
    mRevealAssetRequest = path;
}

std::string EditorApplication::takeRevealAssetRequest()
{
    std::string path = std::move(mRevealAssetRequest);
    mRevealAssetRequest.clear();
    return path;
}

void EditorApplication::addSceneToProject(const std::string& scenePath)
{
    if (mProjectRoot.empty())
        return;
    std::error_code error;
    const std::filesystem::path relative =
        std::filesystem::relative(scenePath, mProjectRoot, error);
    if (error || relative.empty() || relative.string().find("..") == 0)
        return;
    const std::string value = relative.generic_string();
    if (std::find(mProjectScenes.begin(), mProjectScenes.end(), value) == mProjectScenes.end())
        mProjectScenes.push_back(value);
    mProjectActiveScene = value;
    saveProject();
}

bool EditorApplication::createProject(const std::string& parentDirectory, const std::string& name)
{
    const std::filesystem::path projectName(name);
    if (name.empty() || projectName.is_absolute() || projectName.has_parent_path() ||
        projectName == "." || projectName == "..")
    {
        Log::error("EditorApplication: '%s' is not a valid project name", name.c_str());
        toasts().error("'" + name + "' is not a valid project name");
        return false;
    }
    const std::filesystem::path root = std::filesystem::path(parentDirectory) / name;
    std::error_code error;
    const auto ensureDirectory = [&error](const std::filesystem::path& path)
    {
        error.clear();
        std::filesystem::create_directories(path, error);
        if (error)
            return false;
        error.clear();
        return std::filesystem::is_directory(path, error) && !error;
    };
    if (!ensureDirectory(root / "Assets") || !ensureDirectory(root / "Scenes") ||
        !ensureDirectory(root / "Scripts"))
    {
        Log::error("EditorApplication: could not create project folders under '%s' (%s)",
                   root.string().c_str(), error.message().c_str());
        toasts().error("Could not create project folders under " + root.string());
        return false;
    }

    unregisterProjectSearchPaths();
    mProjectRoot = root;
    mProjectManifest = root / "project.radion.json";
    mProjectAssetRoot = "Assets";
    mProjectAssetSearchPath = (root / "Assets").string();
    mProjectScenes.clear();
    mProjectActiveScene.clear();
    mExtraSearchPaths.clear();
    mSettings.lastProjectPath = mProjectManifest.string();
    registerProjectSearchPaths();
    newScene();
    const bool saved = saveProject();
    if (saved)
        rememberRecentProject(mProjectManifest.string());
    else
    {
        Log::error("EditorApplication: could not write '%s'", mProjectManifest.string().c_str());
        toasts().error("Could not write " + mProjectManifest.string());
    }
    return saved;
}

void EditorApplication::rememberRecentProject(const std::string& manifestPath)
{
    if (manifestPath.empty())
        return;
    std::error_code error;
    const std::filesystem::path absolutePath =
        std::filesystem::absolute(manifestPath, error).lexically_normal();
    const std::string value = (error ? std::filesystem::path(manifestPath) : absolutePath).string();
    auto& recent = mSettings.recentProjectPaths;
    recent.erase(std::remove(recent.begin(), recent.end(), value), recent.end());
    recent.insert(recent.begin(), value);
    if (recent.size() > 12)
        recent.resize(12);
}

bool EditorApplication::openProject(const std::string& manifestPath)
{
    const std::string text = FileSystem::getSingleton().readText(manifestPath);
    if (text.empty())
        return false;
    nlohmann::json manifest;
    try
    {
        manifest = nlohmann::json::parse(text);
    }
    catch (const std::exception&)
    {
        return false;
    }
    const auto scenesField = manifest.find("scenes");
    if (manifest.value("format", std::string()) != "radion-project" ||
        scenesField == manifest.end() || !scenesField->is_array())
        return false;
    const std::filesystem::path root = std::filesystem::path(manifestPath).parent_path();

    const auto safeRelativePath = [](const std::string& value)
    {
        const std::filesystem::path path(value);
        if (path.empty() || path.is_absolute())
            return false;
        for (const std::filesystem::path& component : path.lexically_normal())
            if (component == "..")
                return false;
        return true;
    };

    std::vector<std::string> scenes;
    for (const nlohmann::json& scene : *scenesField)
        if (scene.is_string() && safeRelativePath(scene.get_ref<const std::string&>()))
            scenes.push_back(std::filesystem::path(scene.get<std::string>())
                                 .lexically_normal()
                                 .generic_string());
    const std::string active = manifest.value("activeScene", std::string());
    if (!safeRelativePath(active))
        return false;
    const std::string normalizedActive =
        std::filesystem::path(active).lexically_normal().generic_string();
    if (std::find(scenes.begin(), scenes.end(), normalizedActive) == scenes.end())
        return false;

    std::string assetRoot = manifest.value("assetRoot", std::string("Assets"));
    if (!safeRelativePath(assetRoot))
        return false;
    assetRoot = std::filesystem::path(assetRoot).lexically_normal().generic_string();

    std::vector<std::string> extraSearchPaths;
    const auto searchPathsField = manifest.find("searchPaths");
    if (searchPathsField != manifest.end() && searchPathsField->is_array())
        for (const nlohmann::json& path : *searchPathsField)
            if (path.is_string() && !path.get_ref<const std::string&>().empty() &&
                std::find(extraSearchPaths.begin(), extraSearchPaths.end(),
                          path.get_ref<const std::string&>()) == extraSearchPaths.end())
                extraSearchPaths.push_back(path.get<std::string>());

    // Asset lookup is part of loading a scene, so install the candidate
    // project's paths before deserializing. If loading fails, restore the
    // previous project's paths and metadata unchanged.
    unregisterProjectSearchPaths();
    const std::string candidateAssetPath = (root / assetRoot).string();
    FileSystem::getSingleton().addSearchPath(candidateAssetPath);
    for (const std::string& path : extraSearchPaths)
        FileSystem::getSingleton().addSearchPath(path);
    if (!loadScene((root / normalizedActive).string(), false))
    {
        FileSystem::getSingleton().removeSearchPath(candidateAssetPath);
        for (const std::string& path : extraSearchPaths)
            FileSystem::getSingleton().removeSearchPath(path);
        registerProjectSearchPaths();
        return false;
    }

    mProjectRoot = root;
    mProjectManifest = manifestPath;
    mProjectAssetRoot = assetRoot;
    mProjectAssetSearchPath = candidateAssetPath;
    mProjectScenes = scenes;
    mProjectActiveScene = normalizedActive;
    mExtraSearchPaths = std::move(extraSearchPaths);
    mSettings.lastProjectPath = manifestPath;
    rememberRecentProject(manifestPath);
    return true;
}

void EditorApplication::registerProjectSearchPaths()
{
    if (!mProjectAssetSearchPath.empty())
        FileSystem::getSingleton().addSearchPath(mProjectAssetSearchPath);
    for (const std::string& path : mExtraSearchPaths)
        FileSystem::getSingleton().addSearchPath(path);
}

void EditorApplication::unregisterProjectSearchPaths()
{
    if (!mProjectAssetSearchPath.empty())
        FileSystem::getSingleton().removeSearchPath(mProjectAssetSearchPath);
    for (const std::string& path : mExtraSearchPaths)
        FileSystem::getSingleton().removeSearchPath(path);
}

void EditorApplication::closeProject()
{
    unregisterProjectSearchPaths();
    mProjectRoot.clear();
    mProjectManifest.clear();
    mProjectAssetRoot = "Assets";
    mProjectAssetSearchPath.clear();
    mProjectScenes.clear();
    mProjectActiveScene.clear();
    mExtraSearchPaths.clear();
    mSettings.lastProjectPath.clear();
}

void EditorApplication::drawDockspace()
{
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGuiWindowFlags hostFlags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar |
                                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                 ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_MenuBar;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("EditorDockspaceHost", nullptr, hostFlags);
    ImGui::PopStyleVar(3);

    drawMainMenuBar();

    const ImGuiID dockspaceId = ImGui::GetID(kDockspaceId);
    ImGui::DockSpace(dockspaceId, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_None);

    if (!mDockLayoutBuilt)
    {
        mDockLayoutBuilt = true;
        // Only on the very first run for this .ini - once the user redocks
        // anything, ImGui's own persistence takes over and this never runs
        // again (PLANO_EDITOR.md: "não reconstruir os docks a cada arranque").
        if (mSettings.dockLayoutVersion < 1 || ImGui::DockBuilderGetNode(dockspaceId) == nullptr ||
            ImGui::DockBuilderGetNode(dockspaceId)->IsSplitNode() == false)
        {
            ImGui::DockBuilderRemoveNode(dockspaceId);
            ImGui::DockBuilderAddNode(dockspaceId, ImGuiDockNodeFlags_DockSpace);
            ImGui::DockBuilderSetNodeSize(dockspaceId, viewport->WorkSize);

            ImGuiID left, center, right, centerTop, bottom, inspector, settings;
            ImGui::DockBuilderSplitNode(dockspaceId, ImGuiDir_Left, 0.20f, &left, &center);
            ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.22f / 0.80f, &right, &center);
            ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.25f, &bottom, &centerTop);
            ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.48f, &settings, &inspector);

            ImGui::DockBuilderDockWindow(kHierarchyWindow, left);

            ImGui::DockBuilderDockWindow(kGameWindow, centerTop);
            ImGui::DockBuilderDockWindow(kViewportWindow, centerTop);
            ImGui::DockBuilderDockWindow(kInspectorWindow, inspector);
            ImGui::DockBuilderDockWindow("Settings", settings);
            ImGui::DockBuilderDockWindow(kAssetsWindow, bottom);
            ImGui::DockBuilderDockWindow(kConsoleWindow, bottom);
            ImGui::DockBuilderDockWindow(kAnimationWindow, bottom);
            ImGui::DockBuilderDockWindow(kDebugWindow, bottom);
            ImGui::DockBuilderDockWindow(kProfilerWindow, bottom);
            ImGui::DockBuilderDockWindow("Mesh Tools", bottom);
            ImGui::DockBuilderDockWindow("Lightmap", bottom);
            ImGui::DockBuilderDockWindow("Volume", bottom);
            ImGui::DockBuilderFinish(dockspaceId);
            mSettings.dockLayoutVersion = 1;
        }
    }

    ImGui::End();
}

void EditorApplication::drawMainMenuBar()
{
    bool openCameraSettings = false;
    if (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z))
        undo();
    if (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Y))
        redo();
    if (!ImGui::BeginMenuBar())
        return;
    // Disabled during Play - editing the snapshot Stop will discard anyway
    // is not useful, and New/Open would abandon Play silently otherwise.
    ImGui::BeginDisabled(mPlaying);
    if (ImGui::BeginMenu("Project"))
    {
        if (ImGui::MenuItem("New Project..."))
            openFileDialog(ImGuiFileDialog::Mode::ChooseFolder, FileDialogNewProjectFolder,
                           mSettings.lastOpenDirectory);
        if (ImGui::MenuItem("Open Project..."))
            openFileDialog(ImGuiFileDialog::Mode::OpenFile, FileDialogOpenProject,
                           mSettings.lastOpenDirectory);
        if (ImGui::BeginMenu("Recent Projects", !mSettings.recentProjectPaths.empty()))
        {
            for (const std::string& path : mSettings.recentProjectPaths)
            {
                const std::filesystem::path manifest(path);
                const std::string label = manifest.stem().empty() ? manifest.string()
                                                                    : manifest.stem().string();
                if (ImGui::MenuItem((label + "##recent_project_" + path).c_str()))
                    openProject(path);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", path.c_str());
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Clear Recent Projects"))
                mSettings.recentProjectPaths.clear();
            ImGui::EndMenu();
        }
        if (ImGui::MenuItem("Save Project", nullptr, false, !mProjectManifest.empty()))
            saveProject();
        if (ImGui::MenuItem("Close Project", nullptr, false, !mProjectManifest.empty()))
            closeProject();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("File"))
    {
        if (ImGui::MenuItem("New Scene"))
            newScene();
        if (ImGui::MenuItem("Open Scene..."))
            openFileDialog(ImGuiFileDialog::Mode::OpenFile, FileDialogOpenScene,
                           mSettings.lastOpenDirectory);
        if (ImGui::MenuItem("Save Scene", nullptr, false, !mScenePath.empty()))
            saveScene();
        if (ImGui::MenuItem("Save Scene As..."))
            openFileDialog(ImGuiFileDialog::Mode::SaveFile, FileDialogSaveScene,
                           mSettings.lastSaveDirectory);
        ImGui::EndMenu();
    }
    ImGui::EndDisabled();
    if (ImGui::BeginMenu("Edit"))
    {
        if (ImGui::MenuItem("Undo", "Ctrl+Z", false, mUndoPosition > 0))
            undo();
        if (ImGui::MenuItem("Redo", "Ctrl+Y", false, mUndoPosition + 1 < mUndoStates.size()))
            redo();
        ImGui::EndMenu();
    }
    {
        GameObject* selected = mSelection.resolve(scene());
        MeshRenderer* renderer = selected ? selected->getComponent<MeshRenderer>() : nullptr;
        const Mesh* mesh = renderer ? Assets().getMesh(renderer->mesh()) : nullptr;
        const u32 submeshCount = mesh ? static_cast<u32>(mesh->submeshes.size()) : 0;
        const s32 pickedIndex =
            renderer && mPickedSubmesh.object == selected->id() ? mPickedSubmesh.index : -1;
        if (ImGui::BeginMenu("Mesh", renderer != nullptr && submeshCount > 0))
        {
            if (ImGui::MenuItem("Hide All"))
            {
                std::vector<u32> hidden(submeshCount);
                for (u32 i = 0; i < submeshCount; ++i)
                    hidden[i] = i;
                renderer->setHiddenSubmeshes(std::move(hidden));
                markDirty();
            }
            if (ImGui::MenuItem("Unhide All", nullptr, false,
                                !renderer->hiddenSubmeshes().empty()))
            {
                renderer->setHiddenSubmeshes({});
                markDirty();
            }
            if (ImGui::MenuItem("Invert Hidden"))
            {
                std::vector<u32> hidden;
                for (u32 i = 0; i < submeshCount; ++i)
                    if (renderer->submeshVisible(i))
                        hidden.push_back(i);
                renderer->setHiddenSubmeshes(std::move(hidden));
                markDirty();
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Hide Picked", nullptr, false, pickedIndex >= 0))
            {
                renderer->setSubmeshVisible(static_cast<u32>(pickedIndex), false);
                markDirty();
            }
            if (ImGui::MenuItem("Isolate Picked (hide the rest)", nullptr, false,
                                pickedIndex >= 0))
            {
                std::vector<u32> hidden;
                for (u32 i = 0; i < submeshCount; ++i)
                    if (i != static_cast<u32>(pickedIndex))
                        hidden.push_back(i);
                renderer->setHiddenSubmeshes(std::move(hidden));
                markDirty();
            }
            ImGui::EndMenu();
        }
    }
    if (ImGui::BeginMenu("Scenes"))
    {
        if (ImGui::MenuItem("New Scene", "Ctrl+N", false, !mProjectRoot.empty()))
            newScene();
        ImGui::Separator();
        for (const std::string& scene : mProjectScenes)
        {
            const bool active = scene == mProjectActiveScene;
            if (ImGui::MenuItem(scene.c_str(), nullptr, active))
                openScene((mProjectRoot / scene).string());
        }
        if (mProjectScenes.empty())
            ImGui::TextDisabled("Open or create a project to manage scenes.");
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Run"))
    {
        const bool hasTarget = !mProjectManifest.empty() || !mScenePath.empty();
        if (ImGui::MenuItem("Run in Runner", nullptr, false, hasTarget && !runnerRunning()))
            launchRunner();
        if (ImGui::MenuItem("Stop Runner", nullptr, false, runnerRunning()))
            stopRunner();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Windows"))
    {
        for (EditorPanel* panel : mPanels)
        {
            bool visible = panel->active();
            if (ImGui::MenuItem(panel->title().c_str(), nullptr, &visible))
                panel->setActive(visible);
        }
        if (ImGui::MenuItem("Reset Layout"))
        {
            ImGui::DockBuilderRemoveNode(ImGui::GetID(kDockspaceId));
            mSettings.dockLayoutVersion = 0;
            mDockLayoutBuilt = false;
        }
        ImGui::Separator();
        ImGui::MenuItem("Stats Overlay (FPS/ms)", nullptr, &mShowStatsOverlay);
        ImGui::Separator();
        if (ImGui::BeginMenu("Theme"))
        {
            int& themeIndex = mSettings.themeIndex;
            for (int i = 0; i < kEditorThemeCount; ++i)
            {
                const bool selected = i == themeIndex;
                if (ImGui::MenuItem(editorThemeName(static_cast<EditorThemeKind>(i)), nullptr,
                                    selected))
                {
                    themeIndex = i;
                    applyEditorTheme(static_cast<EditorThemeKind>(i));
                }
            }
            ImGui::EndMenu();
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View"))
    {
        if (ImGui::MenuItem("Camera Settings..."))
            openCameraSettings = true;
        ImGui::EndMenu();
    }

    const f32 buttonsWidth =
        ImGui::CalcTextSize(ICON_MDI_PLAY).x + ImGui::CalcTextSize(ICON_MDI_STOP).x +
        ImGui::GetStyle().ItemSpacing.x * 3.0f + ImGui::GetStyle().FramePadding.x * 4.0f;
    ImGui::SetCursorPosX((ImGui::GetWindowWidth() - buttonsWidth) * 0.5f);
    ImGui::BeginDisabled(mPlaying);
    if (ImGui::Button(ICON_MDI_PLAY))
        play();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Play");
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!mPlaying);
    if (ImGui::Button(ICON_MDI_STOP))
        stop();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Stop");
    ImGui::EndDisabled();

    // Which view is live. Only one renders - see ViewMode - so this is a
    // performance control as much as a navigation one, which is why it sits
    // out here beside Play/Stop instead of inside a menu.
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    const bool sceneActive = mViewMode == ViewMode::Scene;
    if (ImGui::Button(sceneActive ? "Scene" : "Game"))
    {
        mViewMode = sceneActive ? ViewMode::Game : ViewMode::Scene;
        ImGui::SetWindowFocus(sceneActive ? kGameWindow : kViewportWindow);
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Live view: %s. Click to switch - the other view stops rendering\n"
                          "and keeps its last frame, so only one scene submission is paid for.",
                          sceneActive ? "Scene" : "Game");

    ImGui::EndMenuBar();

    if (openCameraSettings)
        ImGui::OpenPopup("Camera Settings");
    drawCameraSettingsPopup();
    drawFileDialog();
    drawNewProjectPopup();
}

void EditorApplication::drawCameraSettingsPopup()
{
    ImGui::SetNextWindowSize(ImVec2(400.0f, 0.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::BeginPopupModal("Camera Settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        return;
    ImGui::TextUnformatted("Editor viewport camera navigation");
    ImGui::Separator();
    ImGui::SliderFloat("Move speed", &mSettings.cameraMoveSpeed, 0.01f, 5.0f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("WASD fly speed while the right mouse button is held in the viewport.");
    ImGui::SliderFloat("Faster speed (Shift)", &mSettings.cameraFastSpeed, 0.01f, 50.0f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Same as Move speed, but while Shift is held.");
    ImGui::Separator();
    ImGui::SliderFloat("Min view", &mSettings.cameraMinView, 0.01f, 100.0f, "%.2f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Closest the orbit camera can zoom in.");
    ImGui::SliderFloat("Max view", &mSettings.cameraMaxView, 10.0f, 500000.0f, "%.0f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Furthest the orbit camera can zoom out.");
    ImGui::Separator();
    ImGui::TextUnformatted("Viewport frustum");
    if (ImGui::DragFloat("Near clipping plane", &mSettings.cameraNearPlane, 0.01f, 0.001f,
                         mSettings.cameraFarPlane - 0.001f, "%.3f"))
        mSettings.cameraNearPlane =
            glm::clamp(mSettings.cameraNearPlane, 0.001f, mSettings.cameraFarPlane - 0.001f);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Geometry closer than this distance is clipped.");
    if (ImGui::DragFloat("Far clipping plane", &mSettings.cameraFarPlane, 1.0f,
                         mSettings.cameraNearPlane + 0.001f, 1000000.0f, "%.1f"))
        mSettings.cameraFarPlane =
            glm::max(mSettings.cameraFarPlane, mSettings.cameraNearPlane + 0.001f);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Geometry beyond this distance is clipped.");
    ImGui::Separator();
    // Applied live in the viewport (it reads these every frame) and written
    // to disk here - the periodic 5s auto-save would persist them anyway.
    if (ImGui::Button("Save"))
        mSettings.save(mSettingsFile);
    ImGui::SameLine();
    if (ImGui::Button("Close"))
        ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

void EditorApplication::openFileDialog(ImGuiFileDialog::Mode mode, int action,
                                       const std::string& directory)
{
    mFileDialog.Open(mode,
                     directory.empty() ? std::filesystem::current_path()
                                       : std::filesystem::path(directory),
                     mode == ImGuiFileDialog::Mode::SaveFile ? "NewScene.scene.json" : "");
    mFileDialogAction = static_cast<FileDialogAction>(action);
}

void EditorApplication::drawFileDialog()
{
    if (mFileDialogAction == FileDialogNone)
        return;
    const std::filesystem::path root =
        mProjectRoot.empty() ? std::filesystem::current_path() : mProjectRoot;
    if (!mFileDialog.Render(root, root / "Scripts", root / "Bin"))
        return;
    const ImGuiFileDialog::Result result = mFileDialog.ConsumeResult();
    const FileDialogAction action = mFileDialogAction;
    mFileDialogAction = FileDialogNone;
    if (!result.accepted)
        return;
    mSettings.lastOpenDirectory = result.path.parent_path().string();
    if (action == FileDialogOpenScene)
        openScene(result.path.string());
    else if (action == FileDialogSaveScene)
        saveSceneAs(result.path.string());
    else if (action == FileDialogOpenProject)
        openProject(result.path.string());
    else if (action == FileDialogAddSearchPath)
        addProjectSearchPath(result.path.string());
    else
    {
        mNewProjectParent = result.path.string();
        mShowNewProjectPopup = true;
    }
}

void EditorApplication::drawNewProjectPopup()
{
    if (mShowNewProjectPopup)
    {
        ImGui::OpenPopup("New Project");
        mShowNewProjectPopup = false;
    }
    bool open = true;
    if (!ImGui::BeginPopupModal("New Project", &open, ImGuiWindowFlags_AlwaysAutoResize))
        return;
    ImGui::SetNextItemWidth(280.0f);
    ImGui::InputText("Name", mProjectNameBuffer, sizeof(mProjectNameBuffer));
    if (ImGui::IsWindowAppearing())
        ImGui::SetKeyboardFocusHere(-1);
    const bool createRequested = ImGui::Button("Create") || ImGui::IsKeyPressed(ImGuiKey_Enter);
    if (createRequested && createProject(mNewProjectParent, mProjectNameBuffer))
    {
        toasts().success("Project created: " + std::string(mProjectNameBuffer));
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel"))
        ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

void EditorApplication::drawOpenScenePopup()
{
    if (!ImGui::BeginPopupModal("Open Scene", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        return;
    ImGui::InputText("Path", mPathBuffer, sizeof(mPathBuffer));
    const bool canOpen = mPathBuffer[0] != '\0';
    if (ImGui::Button("Open", ImVec2(120.0f, 0.0f)) && canOpen)
    {
        if (openScene(mPathBuffer))
            ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f)))
        ImGui::CloseCurrentPopup();
    if (!mLastDiagnostics.empty())
    {
        ImGui::Separator();
        for (const SceneDiagnostic& diagnostic : mLastDiagnostics)
            ImGui::TextWrapped("%s: %s", diagnostic.jsonPath.c_str(), diagnostic.message.c_str());
    }
    ImGui::EndPopup();
}

void EditorApplication::drawSaveSceneAsPopup()
{
    if (!ImGui::BeginPopupModal("Save Scene As", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        return;
    ImGui::InputText("Path", mPathBuffer, sizeof(mPathBuffer));
    const bool canSave = mPathBuffer[0] != '\0';
    if (ImGui::Button("Save", ImVec2(120.0f, 0.0f)) && canSave)
    {
        if (saveSceneAs(mPathBuffer))
            ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f)))
        ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

void EditorApplication::drawStatsOverlay(f32 deltaTime)
{
    const u32 pendingTextures = AsyncTextureLoader::getSingleton().pendingCount();
    const u32 pendingMeshes = Assets().pendingAsyncMeshLoads();
    // Streaming feedback is not behind the Stats Overlay toggle - a scene
    // load is exactly the moment the editor used to look frozen, so it has
    // to be visible without the user knowing to turn anything on first.
    if (!mShowStatsOverlay && !mStartupLoadPending && !mStartupLoading && pendingTextures == 0 &&
        pendingMeshes == 0)
        return;

    // Exponential moving average - a raw per-frame delta jumps around too
    // much to read at a glance, this reacts over roughly ten frames instead.
    mStatsSmoothedDelta =
        mStatsSmoothedDelta <= 0.0f ? deltaTime : mStatsSmoothedDelta * 0.9f + deltaTime * 0.1f;
    const f32 fps = mStatsSmoothedDelta > 0.0f ? 1.0f / mStatsSmoothedDelta : 0.0f;

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - 10.0f, viewport->WorkPos.y + 10.0f),
        ImGuiCond_Always, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.4f);
    constexpr ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoInputs;
    if (ImGui::Begin("##StatsOverlay", nullptr, flags))
    {
        if (mStartupLoadPending && !mStartupLoading)
            ImGui::Text("Preparing editor layout...");
        else if (mStartupLoading)
        {
            ImGui::Text(ICON_MDI_PROGRESS_CLOCK " Opening %s",
                        FileSystem::fileName(mStartupLoadPath).c_str());
            ImGui::TextDisabled("Meshes and textures are read on this thread - a large scene can "
                               "take a while, and the window will not answer until it is done.");
        }
        if (mShowStatsOverlay)
            ImGui::Text("%.0f FPS  %.2f ms", fps, mStatsSmoothedDelta * 1000.0f);
        if (pendingTextures > 0)
            ImGui::Text(ICON_MDI_IMAGE_MULTIPLE " Streaming %u texture(s)...", pendingTextures);
        if (pendingMeshes > 0)
            ImGui::Text(ICON_MDI_CUBE_OUTLINE " Streaming %u mesh(es)...", pendingMeshes);
    }
    ImGui::End();
}

void EditorApplication::runFrame(f32 deltaTime)
{
    // ViewportPanel raises this again after processing camera input. Reset it
    // here so a closed/hidden viewport cannot leave captures deferred.
    mEngine.setProbeCaptureDeferred(false);

    // Submesh indices only mean anything against the object they were picked
    // on. Enforced here rather than at each call site because the object
    // selection changes from several places - a Viewport click, a Hierarchy
    // row, a rubber band - and only one of them ever remembered to do it.
    if (!mSubmeshSelection.indices.empty() && mSelection.selectedId() != mSubmeshSelection.object)
    {
        mSubmeshSelection.object = 0;
        mSubmeshSelection.indices.clear();
    }
    if (mDirty && !mScenePath.empty() && !mPlaying)
    {
        mAutoSaveTimer += deltaTime;
        if (mAutoSaveTimer >= kAutoSaveInterval)
        {
            mAutoSaveTimer = 0.0f;
            if (saveScene())
                Log::info("EditorApplication: auto-saved '%s'", mScenePath.c_str());
        }
    }
    else
        mAutoSaveTimer = 0.0f;

    scene().update(deltaTime);
    drawDockspace();
    drawStatsOverlay(deltaTime);
    mToasts.update(deltaTime);
    for (EditorPanel* panel : mPanels)
    {
        if (!panel->active())
            continue;
        // Every window uses its constant title as ImGui id, matching what
        // DockBuilderDockWindow() above targeted.
        bool open = true;
        ImGuiWindowFlags flags = ImGuiWindowFlags_None;
        if (panel->title() == kViewportWindow)
            flags |= ImGuiWindowFlags_NoScrollWithMouse;
        if (ImGui::Begin(panel->title().c_str(), &open, flags))
            panel->onImGui();
        ImGui::End();
        panel->setActive(open);
    }
    // Last, so the toasts sit over every panel rather than under one.
    mToasts.draw();
    SceneRenderSettings renderSettings = sceneRenderSettings();
    const std::string currentRenderSettings =
        mSerializer.renderSettingsToJson(renderSettings).dump();
    if (mRenderSettingsSnapshot.empty())
        mRenderSettingsSnapshot = currentRenderSettings;
    else if (currentRenderSettings != mRenderSettingsSnapshot)
    {
        mRenderSettingsSnapshot = currentRenderSettings;
        markDirty();
    }
    if (mFocusViewportPending)
    {
        // SetWindowFocus() only finds a window that has had Begin() called at
        // least once - the very first runFrame() this is still null before
        // the panel loop above, a no-op. Runs after it instead, once "Viewport"
        // is guaranteed to exist, and wins over whatever tab imgui.ini
        // persisted as selected from a previous session.
        mFocusViewportPending = false;
        ImGui::SetWindowFocus(kViewportWindow);
    }
    // Preferences are cheap JSON and should survive more than a clean exit.
    // Five seconds avoids writing every camera movement frame while bounding
    // what a crash can lose.
    mSettingsSaveTimer += deltaTime;
    if (mSettingsSaveTimer >= 5.0f)
    {
        mSettingsSaveTimer = 0.0f;
        mSettings.cursor3D = mCursor3D;
        mSettings.viewMode = mViewMode == ViewMode::Game ? 1 : 0;
        mSettings.showStatsOverlay = mShowStatsOverlay;
        mSettings.showDynamicIndexDebug = mShowDynamicIndexDebug;
        mSettings.showOcclusionDebug = mShowOcclusionDebug;
        mSettings.showSubmeshBounds = mShowSubmeshBounds;
        mSettings.showShadowCascades = mEngine.debugShowShadowCascades;
        mSettings.showShadowAtlas = mEngine.debugShowShadowAtlas;
        for (EditorPanel* panel : mPanels)
            mSettings.panelOpen[panel->title()] = panel->active();
        mSettings.save(mSettingsFile);
    }
}

void EditorApplication::launchRunner()
{
#ifdef _WIN32
    Log::warning("Editor: launching the runner is not implemented on Windows yet");
#else
    if (mRunnerPid > 0)
        return;
    if (!mScenePath.empty())
        saveScene();
    if (!mProjectManifest.empty())
        saveProject();
    const std::string target =
        !mProjectManifest.empty() ? mProjectManifest.string() : mScenePath;
    if (target.empty())
        return;

    std::error_code error;
    const std::filesystem::path executable =
        std::filesystem::read_symlink("/proc/self/exe", error);
    if (error)
    {
        Log::error("Editor: could not locate the editor executable to find the runner");
        return;
    }
    const std::filesystem::path directory = executable.parent_path();
    const std::filesystem::path candidates[] = {
        directory / "radion_runner",
        directory.parent_path() / "runner" / "radion_runner",
    };
    std::filesystem::path runner;
    for (const std::filesystem::path& candidate : candidates)
    {
        if (std::filesystem::is_regular_file(candidate, error))
        {
            runner = candidate;
            break;
        }
    }
    if (runner.empty())
    {
        Log::error("Editor: radion_runner not found next to the editor - build the runner target");
        return;
    }

    const pid_t child = fork();
    if (child < 0)
    {
        Log::error("Editor: fork failed, runner not launched");
        return;
    }
    if (child == 0)
    {
        execl(runner.c_str(), "radion_runner", target.c_str(), static_cast<char*>(nullptr));
        _exit(127);
    }
    mRunnerPid = child;
    Log::info("Editor: runner started on '%s'", target.c_str());
    mEngine.getWindow().minimize();
#endif
}

void EditorApplication::stopRunner()
{
#ifndef _WIN32
    if (mRunnerPid > 0)
        kill(static_cast<pid_t>(mRunnerPid), SIGTERM);
#endif
}

void EditorApplication::updateRunner()
{
#ifndef _WIN32
    if (mRunnerPid <= 0)
        return;
    int status = 0;
    if (waitpid(static_cast<pid_t>(mRunnerPid), &status, WNOHANG) <= 0)
        return;
    mRunnerPid = 0;
    if (WIFEXITED(status) && WEXITSTATUS(status) != 0)
        Log::warning("Editor: runner exited with status %d", WEXITSTATUS(status));
    else
        Log::info("Editor: runner finished");
    mEngine.getWindow().restore();
#endif
}

void EditorApplication::run()
{
    while (mEngine.update())
    {
        const f32 deltaTime = glm::min(mEngine.getWindow().getDeltaTime(), 0.1f);
        mSceneRendered = false;
        runFrame(deltaTime);
        // Only if no panel drew the scene into its own texture this frame -
        // an older persisted layout with neither Viewport nor Game visible,
        // say. This used to run unconditionally, before the panels, which
        // meant a third full submission of the whole scene from the game
        // camera every frame, straight into a backbuffer the dockspace then
        // covered completely. Nothing was ever seen of it; it just cost a
        // scene's worth of geometry and shadows.
        if (!mSceneRendered)
            mEngine.render(scene());
        mEngine.flip();
        updateRunner();
        // While the runner owns the screen, the minimized editor idles at a
        // crawl instead of racing the game for the GPU.
        if (runnerRunning() && mEngine.getWindow().isMinimized())
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // Let the first editor frame reach the window before loading the
        // persisted project/scene. Previously this happened in the
        // constructor, so the user saw a black window until all scene work
        // completed and no loading feedback could be drawn.
        if (mStartupLoadPending && !mStartupLoadStarted)
        {
            ++mStartupFramesPresented;
            // The load is one blocking call, so the frame that says so has to
            // be on screen BEFORE it starts: raising the flag inside
            // startDeferredStartupLoad() set and cleared it without a single
            // frame drawn in between, and the message was never seen.
            if (mStartupFramesPresented == 2)
                mStartupLoading = true;
            else if (mStartupFramesPresented >= 3)
                startDeferredStartupLoad();
        }
    }
    stopRunner();
}

} // namespace Radion
