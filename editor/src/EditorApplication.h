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
#include <glm/vec3.hpp>
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
    // Every Log::error and Log::warning in the whole engine becomes a toast,
    // rather than each call site remembering to raise one - the pairing is
    // what drifts, and an error nobody sees is the same as no error at all.
    // Routed here because Log takes one sink and the console needs it too.
    static void logSink(LogLevel level, const char* message);

    EditorToasts& toasts()
    {
        return mToasts;
    }

    // Which bone/IK chain of the selected object's Animator the viewport
    // gizmo currently manipulates - AnimationPanel sets it from its bone/IK
    // list, ViewportPanel reads it to retarget its gizmo onto a bone instead
    // of the object's own transform (mutually exclusive: only one of bone/
    // ikChain is ever >= 0 at a time). Neither panel reaches into the
    // other's fields directly, per PLANO_EDITOR.md rule 1.
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

    // The submesh the Viewport or the Inspector's own submesh list last
    // pointed at - the single thing that ties the two together in both
    // directions: a Viewport click opens/scrolls to that submesh's entry in
    // the Inspector, and clicking an entry in the Inspector highlights that
    // submesh's box back in the Viewport. The owning object's id travels
    // with it so a stale index is never read against whatever is selected
    // now. `justPicked` is the one-shot half - true only for the frame right
    // after a Viewport click, so the Inspector knows to force its entry open
    // and scroll to it instead of doing that every single frame.
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

    // Shift-clicking with the Pick Surface tool accumulates a SET of
    // submeshes on one object - stripping a building down to only its floor
    // submeshes before baking a NavMeshSurface from it, say. Separate from
    // PickedSubmesh, which stays the single "last pointed at" entry the
    // Inspector and Viewport highlight each other through; this is what a
    // batch delete acts on instead.
    struct SubmeshSelection
    {
        u64 object = 0;
        std::vector<u32> indices;
    };
    SubmeshSelection& submeshSelection()
    {
        return mSubmeshSelection;
    }
    // Shared by the Viewport's Delete key and the Inspector's own "Delete
    // Selected Submeshes" button - one removal path so both stay in sync
    // with how index shifting after each erase is handled.
    void deleteSubmeshSelection();

    struct PickedSurface
    {
        bool valid = false;
        glm::vec3 position = glm::vec3(0.0f);
        glm::vec3 normal = glm::vec3(0.0f);
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

    // Click-to-place for vegetation: while a mode is active, the Viewport's
    // click plants directly at the raycast hit on the selected object's
    // Forest/Grass instead of picking whatever is under the cursor. One
    // shared mode rather than a bool per component keeps the two exclusive -
    // ticking Grass' box while Tree is on turns Tree off, the way a radio
    // button would.
    enum class VegetationPlacementMode : u8 { None, Tree, Grass };
    VegetationPlacementMode& vegetationPlacementMode()
    {
        return mVegetationPlacementMode;
    }
    u32& vegetationPlacementSpecies()
    {
        return mVegetationPlacementSpecies;
    }

    // Which of the two scene views is allowed to render this frame. Every
    // view is a full scene submission - geometry, shadows, probes, the lot -
    // so two of them visible at once is two of everything, and the frame
    // showed it (Sponza: the second view alone was ~700k triangles and half
    // the frame). Only the active one renders; the other keeps showing the
    // last frame it drew. Play switches to Game and Stop switches back, the
    // way the reference editors do it.
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
    // Set by whichever panel actually submitted the scene this frame, so
    // run() knows it does not also have to render to the window itself to
    // keep something on screen. Cleared at the top of every frame.
    void notifySceneRendered()
    {
        mSceneRendered = true;
    }

    void newScene();
    bool openScene(const std::string& path);
    // Resizes the cascade shadow frustum around whatever is currently loaded,
    // keeping the user's own cascade count/resolution choices intact.
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
    const glm::vec3& cursor3D() const
    {
        return mCursor3D;
    }
    void setCursor3D(const glm::vec3& position)
    {
        mCursor3D = position;
    }
    void recordUndo();
    // recordUndo() plus the mesh's submesh table as it stands right now -
    // what a destructive submesh edit has to call instead, so undo can put
    // the deleted pieces back. Does nothing for a mesh with no CPU-side copy
    // (nothing to snapshot, and nothing could have edited it either).
    void recordMeshUndo(MeshHandle handle);
    void undo();
    void redo();

    // Deep-copies `source` and its children into this same scene, right
    // beside the original (SceneSerializer::cloneObject() does the actual
    // work - the same object+component read/write Save/Load and Undo
    // already use, just scoped to one subtree). Records undo and selects the
    // clone on success; null (nothing changed) if cloning failed.
    GameObject* duplicateObject(GameObject& source);
    bool duplicateObjectGrid(GameObject& source, bool alongX, bool alongZ, u32 countX, u32 countZ,
                             f32 spacing);

    // "wp_00" -> "wp_01" -> "wp_02", keeping the original zero padding and
    // widening it only when the counter runs past it ("wp_99" -> "wp_100").
    // A name with no trailing digits gets a "_01" suffix instead. Keeps
    // counting past any name already taken in the scene, so duplicating the
    // same source repeatedly walks forward rather than colliding.
    std::string nextIncrementedName(const std::string& name);

    // Which point of the selected object's Waypoints component the transform
    // gizmo drives, or -1 for the object itself. Lives here rather than in
    // either panel because the Inspector picks it and the Viewport moves it.
    s32 selectedWaypoint() const
    {
        return mSelectedWaypoint;
    }
    void setSelectedWaypoint(s32 index)
    {
        mSelectedWaypoint = index;
    }

    // What the last open/save produced, for the console/status area to show -
    // SceneDiagnostic, not stdout (see PLANO_SERIALIZACAO_CENA.md's own rule
    // on this).
    const std::vector<SceneDiagnostic>& lastDiagnostics() const
    {
        return mLastDiagnostics;
    }

    // Extra folders FileSystem::readBinary() falls back to, beyond the
    // project's own Assets/ - what a mesh imported from outside the project
    // (its textures still sitting wherever it was exported to, not copied
    // in) needs to actually find them instead of logging "file not found".
    // Persisted in the project manifest, restored on openProject().
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
    // Where AssetsPanel starts/resets to: the open project's own Assets/
    // folder, or this engine's own shipped assets when no project is open -
    // the loose scene workflow every demo main.cpp still uses. The panel
    // itself navigates freely from there - this is a starting point, not a
    // boundary.
    std::string assetBrowserRoot() const;

    // Editor-only preferences (theme, last-used paths - see EditorSettings)
    // - SettingsPanel reads/writes through this rather than owning its own
    // copy, so a change takes effect immediately and survives to the next
    // save() alongside everything else in the file.
    EditorSettings& settings()
    {
        return mSettings;
    }
    // Opens the folder-choose dialog whose result feeds addProjectSearchPath()
    // - SettingsPanel calls this rather than reaching into the file dialog
    // machinery itself (EditorPanel never draws for another panel, PLANO_
    // EDITOR.md rule 1 - this keeps that true for triggering one too).
    void browseAddSearchPath();

    // Every mesh AssetsPanel's Import/Load brought in this session keeps its
    // CPU-side MeshData here, keyed by the GPU handle MeshRenderer::mesh()
    // already holds - not a new indirection, just not throwing away the data
    // the moment it is done being useful for tools. A primitive built from
    // HierarchyPanel, or anything opened without going through that flow, has
    // no entry - importedMeshData() returns nullptr and InspectorPanel shows
    // no mesh-tools section for it, since there is nothing to run them on.
    MeshData* importedMeshData(MeshHandle handle);
    void registerImportedMesh(MeshHandle handle, MeshData data);
    // Geometry built in memory rather than read from a file - the volume
    // mesher's output. `outputBase` is a path without extension: the data is
    // written there as .rmesh (and .material) first, because a MeshDesc can
    // only name a file and without one SceneSerializer has no recipe to save.
    // Everything after that is the import flow: upload, keep the CPU copy for
    // the mesh tools, and hang a MeshRenderer on a new object.
    GameObject* adoptGeneratedMesh(const std::string& name, const std::string& outputBase,
                                   MeshData data);
    // Pushes whatever the caller already edited in-place (through the
    // pointer importedMeshData() handed back) onto the GPU via
    // AssetManager::replaceMesh() - same handle, so every MeshRenderer
    // already pointing at it just draws the new geometry next frame.
    bool applyMeshEdit(MeshHandle handle);

    // HierarchyPanel double-click asking ViewportPanel to frame an object -
    // panels never reach into one another directly (PLANO_EDITOR.md rule 1),
    // so the request sits here for ViewportPanel to pick up on its own next
    // onImGui(). 0 means none pending.
    void requestFocusObject(u64 objectId);
    u64 takeFocusObjectRequest();

    // InspectorPanel's texture slots asking AssetsPanel to jump to the
    // asset's own folder - same mediator shape as the two requests above,
    // for the same reason (PLANO_EDITOR.md rule 1). Empty string means none
    // pending.
    void requestRevealAsset(const std::string& path);
    std::string takeRevealAssetRequest();

    // DebugPanel's own checkboxes for Scene's dynamic-octree / occlusion
    // debug draw - see mShowDynamicIndexDebug's own comment for why the
    // state lives here instead of on either panel.
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
    // Every SubMesh::bounds of the selected object, drawn as its own box -
    // the one thing that answers "why does frustum culling reject almost
    // nothing on this mesh". Submeshes grouped by material rather than by
    // region each span the whole model, so every box contains the camera and
    // none can ever be culled; boxes that visibly nest inside one another
    // mean the split is spatial and the culling is simply doing its job.
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
    // Installs `replacement` through Engine's SceneManager and clears the
    // selection that belonged to the previous active scene.
    void replaceScene(Scene* replacement);
    void startDeferredStartupLoad();

    Engine& mEngine;
    SceneSerializer mSerializer;
    EditorSelection mSelection;
    EditorToasts mToasts;
    // logSink() is a plain function pointer with no `this` to route through.
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
    // Saves over mScenePath by itself once mDirty has stood for this long -
    // a Mesh Tools edit (or anything else) that never got an explicit Save
    // still survives a crash, an accidental close, or forgetting Ctrl+S.
    // Never runs with no scenePath yet (nothing to save over without asking
    // Save As's own dialog first) or mid-Play (that would overwrite the
    // saved edit-mode scene with the Play snapshot's own drift).
    f32 mAutoSaveTimer = 0.0f;
    static constexpr f32 kAutoSaveInterval = 60.0f;
    std::vector<SceneDiagnostic> mLastDiagnostics;
    s32 mSelectedWaypoint = -1;
    std::vector<EditorPanel*> mPanels; // owned
    ScriptEditorPanel* mScriptEditor = nullptr;
    bool mDockLayoutBuilt = false;
    bool mFocusScriptEditorPending = false;
    // One-shot: forces the Scene tab active on launch even when imgui.ini
    // already has a saved layout with Game selected (a persisted "Selected"
    // tab id in [Docking][Data] wins over DockBuilderDockWindow()'s call
    // order, so that fix alone never runs for a returning user).
    bool mFocusViewportPending = true;
    u64 mFocusObjectRequest = 0;
    std::string mRevealAssetRequest;
    // Smoothed frame time behind the status bar's FPS reading. The toggle
    // that used to guard a floating overlay is gone with it: the bar has a
    // place of its own and covers nothing, so there is nothing to turn off.
    f32 mStatsSmoothedDelta = 0.0f;
    // DebugPanel writes these, ViewportPanel reads them (the DebugDraw3D
    // calls have to happen there, before Engine::render() - Scene itself has
    // no per-frame hook of its own to emit them from). Off by default: a
    // debug overlay only some sessions want.
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
    // Keyed by MeshHandle::index/generation packed into one u64 - MeshHandle
    // has no std::hash of its own and packHandle() is AssetManager's private
    // convention, not something to depend on from here.
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
    glm::vec3 mCursor3D = glm::vec3(0.0f);
    // An undo step is the scene's JSON plus, when the step was a submesh
    // deletion, that mesh's submesh table as it stood before. The table is
    // all that has to be kept: removeSubmesh() only erases the SubMesh
    // descriptor and leaves the vertex/index buffers alone, so putting the
    // descriptors back restores the geometry exactly - which is what makes
    // undoing a delete on a 280MB mesh cost a few hundred bytes instead of
    // a full copy of it.
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
