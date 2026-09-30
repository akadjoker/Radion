#ifndef RADION_EDITOR_APPLICATION_H
#define RADION_EDITOR_APPLICATION_H

#include "AssetManager.h"
#include "Containers.h"
#include "EditorSelection.h"
#include "EditorToasts.h"
#include "Log.h"
#include "EditorSettings.h"
#include "Engine.h"
#include "ImGuiFileDialog.h"
#include "SceneSerializer.h"
#include "Types.h"

#include <filesystem>
#include "Math.h"
#include <string>
#include <vector>

namespace Radion
{
class Engine;
class Scene;
class EditorPanel;
class GameObject;
class ScriptEditorPanel;

class EditorApplication
{
public:
    explicit EditorApplication(Engine& engine);
    ~EditorApplication();

    EditorApplication(const EditorApplication&) = delete;
    EditorApplication& operator=(const EditorApplication&) = delete;

    void run();

    Scene& scene()
    {
        return *mEngine.activeScene();
    }
    Engine& engine()
    {
        return mEngine;
    }
    EditorSelection& selection()
    {
        return mSelection;
    }
    // Routed here because Log takes one sink and the console needs it too; every Log error/warning becomes a toast.
    static void logSink(LogLevel level, const char* message);

    EditorToasts& toasts()
    {
        return mToasts;
    }

    // Bone/IK chain the viewport gizmo manipulates; mutually exclusive (only one of bone/ikChain is >= 0).
    struct AnimationPoseTarget
    {
        bool active = false;
        s32 bone = -1;
        s32 ikChain = -1;
    };
    AnimationPoseTarget& animationPoseTarget()
    {
        return mAnimationPoseTarget;
    }

    // Submesh last pointed at by Viewport or Inspector; carries the owner id so a stale index is never read. `justPicked` is true only the frame after a Viewport click.
    struct PickedSubmesh
    {
        s32 index = -1;
        u64 object = 0;
        bool justPicked = false;
    };
    PickedSubmesh& pickedSubmesh()
    {
        return mPickedSubmesh;
    }

    // Submesh set built by Shift-click with Pick Surface; separate from PickedSubmesh, this is what batch delete acts on.
    struct SubmeshSelection
    {
        u64 object = 0;
        std::vector<u32> indices;
    };
    SubmeshSelection& submeshSelection()
    {
        return mSubmeshSelection;
    }
    // One removal path for the Viewport Delete key and the Inspector button, so index shifting after each erase stays in sync.
    void deleteSubmeshSelection();

    struct PickedSurface
    {
        bool valid = false;
        Math::vec3 position = Math::vec3(0.0f);
        Math::vec3 normal = Math::vec3(0.0f);
        u64 object = 0;
        s32 submesh = -1;
        s32 materialSlot = -1;
        f32 viewDepth = 0.0f;
    };
    PickedSurface& pickedSurface()
    {
        return mPickedSurface;
    }
    bool& surfaceProbe()
    {
        return mSurfaceProbe;
    }

    // One shared mode (not a bool per component) keeps vegetation placement modes exclusive.
    enum class VegetationPlacementMode : u8 { None, Tree, Grass };
    VegetationPlacementMode& vegetationPlacementMode()
    {
        return mVegetationPlacementMode;
    }
    u32& vegetationPlacementSpecies()
    {
        return mVegetationPlacementSpecies;
    }

    // Only the active view renders (each is a full scene submission, ~2x cost otherwise); the other keeps its last frame.
    enum class ViewMode : u8
    {
        Scene,
        Game
    };
    ViewMode viewMode() const
    {
        return mViewMode;
    }
    void setViewMode(ViewMode mode)
    {
        mViewMode = mode;
    }
    // Set by the panel that submitted the scene this frame, so run() knows not to render to the window itself. Cleared each frame.
    void notifySceneRendered()
    {
        mSceneRendered = true;
    }

    void newScene();
    bool openScene(const std::string& path);
    void fitShadowsToScene();
    bool saveScene();
    bool saveSceneAs(const std::string& path);

    void play();
    void stop();
    bool playing() const
    {
        return mPlaying;
    }

    const std::string& scenePath() const
    {
        return mScenePath;
    }
    bool dirty() const
    {
        return mDirty;
    }

    void markDirty();
    void openScriptEditor(const std::string& path);
    const Math::vec3& cursor3D() const
    {
        return mCursor3D;
    }
    void setCursor3D(const Math::vec3& position)
    {
        mCursor3D = position;
    }
    void recordUndo();
    // recordUndo() plus the mesh's submesh table, so undo can restore deleted pieces. No-op for a mesh with no CPU-side copy.
    void recordMeshUndo(MeshHandle handle);
    void undo();
    void redo();

    // Deep-copies `source` and children beside the original via SceneSerializer::cloneObject(); records undo and selects the clone, null on failure.
    GameObject* duplicateObject(GameObject& source);
    bool duplicateObjectGrid(GameObject& source, bool alongX, bool alongZ, u32 countX, u32 countZ,
                             f32 spacing);

    // "wp_00" -> "wp_01", widening padding only when the counter overflows it; no trailing digits gets "_01"; skips taken names.
    std::string nextIncrementedName(const std::string& name);

    // Waypoint the gizmo drives, or -1 for the object itself; shared because Inspector picks it and Viewport moves it.
    s32 selectedWaypoint() const
    {
        return mSelectedWaypoint;
    }
    void setSelectedWaypoint(s32 index)
    {
        mSelectedWaypoint = index;
    }

    // Last open/save result, shown as SceneDiagnostic (not stdout).
    const std::vector<SceneDiagnostic>& lastDiagnostics() const
    {
        return mLastDiagnostics;
    }

    // Extra folders FileSystem::readBinary() falls back to beyond Assets/, for meshes whose textures live outside the project. Persisted in the manifest.
    const std::vector<std::string>& projectSearchPaths() const
    {
        return mExtraSearchPaths;
    }
    bool hasProject() const
    {
        return !mProjectManifest.empty();
    }
    void addProjectSearchPath(const std::string& path);
    void removeProjectSearchPath(usize index);
    // Where AssetsPanel starts/resets to; a starting point, not a boundary.
    std::string assetBrowserRoot() const;

    EditorSettings& settings()
    {
        return mSettings;
    }
    // Opens the folder-choose dialog whose result feeds addProjectSearchPath().
    void browseAddSearchPath();

    // CPU-side MeshData of meshes imported this session, keyed by MeshRenderer's GPU handle; absent entries (primitives, other flows) get no mesh-tools section.
    MeshData* importedMeshData(MeshHandle handle);
    void registerImportedMesh(MeshHandle handle, MeshData data);
    // In-memory geometry (volume mesher output). `outputBase` has no extension: written as .rmesh/.material first, since a MeshDesc can only name a file.
    GameObject* adoptGeneratedMesh(const std::string& name, const std::string& outputBase,
                                   MeshData data);
    // Pushes in-place edits to the GPU via AssetManager::replaceMesh(); same handle, so renderers pick it up next frame.
    bool applyMeshEdit(MeshHandle handle);

    // Request from HierarchyPanel for ViewportPanel to frame an object (panels don't reach into each other). 0 means none pending.
    void requestFocusObject(u64 objectId);
    u64 takeFocusObjectRequest();

    // Request from InspectorPanel for AssetsPanel to jump to the asset's folder. Empty means none pending.
    void requestRevealAsset(const std::string& path);
    std::string takeRevealAssetRequest();

    bool showDynamicIndexDebug() const
    {
        return mShowDynamicIndexDebug;
    }
    void setShowDynamicIndexDebug(bool show)
    {
        mShowDynamicIndexDebug = show;
    }
    bool showOcclusionDebug() const
    {
        return mShowOcclusionDebug;
    }
    void setShowOcclusionDebug(bool show)
    {
        mShowOcclusionDebug = show;
    }
    // Box per SubMesh::bounds of the selected object; shows why frustum culling rejects little (material-grouped submeshes each span the whole model).
    bool showSubmeshBounds() const
    {
        return mShowSubmeshBounds;
    }
    void setShowSubmeshBounds(bool show)
    {
        mShowSubmeshBounds = show;
    }

    void publishGameTexture(TextureHandle texture)
    {
        mGameTexture = texture;
    }
    TextureHandle gameTexture() const
    {
        return mGameTexture;
    }

private:
    SceneRenderSettings sceneRenderSettings();
    void runFrame(f32 deltaTime);
    void drawDockspace();
    void drawMainMenuBar();
    void drawCameraSettingsPopup();
    void drawStatusBar(f32 deltaTime);
    void drawStatsOverlay(f32 deltaTime);
    void drawFileDialog();
    void drawNewProjectPopup();
    void drawOpenScenePopup();
    void drawSaveSceneAsPopup();
    void openFileDialog(ImGuiFileDialog::Mode mode, int action, const std::string& directory);
    bool createProject(const std::string& parentDirectory, const std::string& name);
    bool openProject(const std::string& manifestPath);
    void rememberRecentProject(const std::string& manifestPath);
    bool saveProject();
    void closeProject();
    void addSceneToProject(const std::string& scenePath);
    void launchRunner();
    void stopRunner();
    void updateRunner();
    bool runnerRunning() const
    {
        return mRunnerPid > 0;
    }
    bool loadScene(const std::string& path, bool addToProject);
    void registerProjectSearchPaths();
    void unregisterProjectSearchPaths();
    void buildPanels();
    void buildDefaultScene();
    void applyNewSceneRenderDefaults();
    void replaceScene(Scene* replacement);
    void startDeferredStartupLoad();
    void applyPersistedDebugSettings();
    void storeRuntimeDebugSettings();

    Engine& mEngine;
    SceneSerializer mSerializer;
    EditorSelection mSelection;
    EditorToasts mToasts;
    // logSink() is a plain function pointer with no `this`.
    static EditorApplication* sInstance;
    AnimationPoseTarget mAnimationPoseTarget;
    PickedSubmesh mPickedSubmesh;
    SubmeshSelection mSubmeshSelection;
    PickedSurface mPickedSurface;
    bool mSurfaceProbe = false;
    VegetationPlacementMode mVegetationPlacementMode = VegetationPlacementMode::None;
    u32 mVegetationPlacementSpecies = 0;
    std::string mScenePath; // empty until the first Save/Save As or a successful Open
    s64 mRunnerPid = 0;
    bool mDirty = false;
    // Autosaves over mScenePath once mDirty has stood this long; never with no scenePath or mid-Play (would overwrite the edit scene with Play drift).
    f32 mAutoSaveTimer = 0.0f;
    static constexpr f32 kAutoSaveInterval = 60.0f;
    std::vector<SceneDiagnostic> mLastDiagnostics;
    s32 mSelectedWaypoint = -1;
    std::vector<EditorPanel*> mPanels; // owned
    ScriptEditorPanel* mScriptEditor = nullptr;
    bool mDockLayoutBuilt = false;
    bool mFocusScriptEditorPending = false;
    // One-shot: a persisted "Selected" tab id in imgui.ini wins over DockBuilderDockWindow(), so force Scene active on launch here.
    bool mFocusViewportPending = true;
    u64 mFocusObjectRequest = 0;
    std::string mRevealAssetRequest;
    f32 mStatsSmoothedDelta = 0.0f;
    // DebugPanel writes, ViewportPanel reads (DebugDraw3D must be emitted before Engine::render()). Off by default.
    bool mShowDynamicIndexDebug = false;
    bool mShowOcclusionDebug = false;
    bool mShowSubmeshBounds = false;
    TextureHandle mGameTexture;

    enum FileDialogAction
    {
        FileDialogNone,
        FileDialogOpenScene,
        FileDialogSaveScene,
        FileDialogOpenProject,
        FileDialogNewProjectFolder,
        FileDialogAddSearchPath
    };
    ImGuiFileDialog mFileDialog;
    FileDialogAction mFileDialogAction = FileDialogNone;
    bool mShowNewProjectPopup = false;
    char mPathBuffer[512] = "";
    char mProjectNameBuffer[128] = "NewProject";
    std::string mNewProjectParent;
    std::filesystem::path mProjectRoot;
    std::filesystem::path mProjectManifest;
    std::string mProjectAssetRoot = "Assets";
    std::string mProjectAssetSearchPath;
    std::vector<std::string> mProjectScenes;
    std::string mProjectActiveScene;
    std::vector<std::string> mExtraSearchPaths;
    // Keyed by MeshHandle index/generation packed into a u64; MeshHandle has no std::hash.
    HashMap<u64, MeshData> mImportedMeshData;

    bool mPlaying = false;
    nlohmann::json mEditSnapshot; // valid only while mPlaying

    ViewMode mViewMode = ViewMode::Scene;
    bool mSceneRendered = false;
    bool mStartupLoadPending = false;
    bool mStartupLoadStarted = false;
    bool mStartupLoading = false;
    u32 mStartupFramesPresented = 0;
    bool mStartupLoadIsProject = false;
    std::string mStartupLoadPath;

    EditorSettings mSettings;
    std::string mSettingsFile;
    f32 mSettingsSaveTimer = 0.0f;
    std::string mRenderSettingsSnapshot;
    Math::vec3 mCursor3D = Math::vec3(0.0f);
    // Undo step: scene JSON plus, for a submesh deletion, the prior submesh table. Only descriptors are kept: removeSubmesh() leaves buffers intact.
    struct UndoState
    {
        nlohmann::json scene;
        MeshHandle mesh;
        std::vector<SubMesh> submeshes;
        bool hasMeshEdit = false;
    };
    std::vector<UndoState> mUndoStates;
    usize mUndoPosition = 0;

    void restoreMeshEdit(const UndoState& state);
};

} // namespace Radion

#endif // RADION_EDITOR_APPLICATION_H
