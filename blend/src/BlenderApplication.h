#ifndef RADION_BLENDER_APPLICATION_H
#define RADION_BLENDER_APPLICATION_H

#include "AssetManager.h"
#include "BlenderSelection.h"
#include "BlenderSettings.h"
#include "Engine.h"
#include "ImGuiFileDialog.h"
#include "Log.h"
#include "MiniBatch.h"
#include "MiniRenderer.h"
#include "Types.h"

#include "Math.h"
#include <string>
#include <vector>

namespace Radion
{
class Engine;
class BlenderPanel;
struct MeshData;

class BlenderApplication
{
public:
    explicit BlenderApplication(Engine& engine);
    ~BlenderApplication();

    BlenderApplication(const BlenderApplication&) = delete;
    BlenderApplication& operator=(const BlenderApplication&) = delete;

    void run();

    Engine& engine()
    {
        return mEngine;
    }

    BlenderSelection& selection()
    {
        return mSelection;
    }

    BlenderSettings& settings()
    {
        return mSettings;
    }

    MiniRenderer& renderer()
    {
        return mRenderer;
    }

    MiniBatch& batch()
    {
        return mBatch;
    }

    // Mesh data access
    MeshData* currentMeshData();
    bool loadMesh(const std::string& path);
    bool importMesh(const std::string& path);
    bool applyMeshEdit();
    void recordUndo();

    // Reworks the UVs of the selected faces, or of the whole mesh when
    // nothing is selected. One undo step per call.
    void applyFaceUVTransform(const Math::vec2& scale, f32 rotationDegrees,
                              const Math::vec2& offset);

    // -- gizmo drag
    //
    // Where the gizmo sits: the median of the selected vertices, or of the
    // whole mesh when nothing is selected. Origin for an empty mesh.
    Math::vec3 transformPivot() const;
    // Takes the undo snapshot and remembers the geometry as it stands, so
    // every frame of the drag transforms the original rather than the last
    // frame's result - compounding a few hundred matrices visibly drifts.
    // False when there is nothing to drag.
    bool beginGizmoDrag();
    // `worldDelta` maps a vertex's position at the start of the drag to where
    // it belongs now; it carries its own pivot.
    void updateGizmoDrag(const Math::mat4& worldDelta);
    void endGizmoDrag();
    bool gizmoDragging() const
    {
        return mGizmoDragging;
    }
    // Drops the most recent recordUndo() snapshot - for a live-editing
    // widget bound directly to mesh/material data, where recordUndo() has to
    // run before the widget so the snapshot is the pre-edit state, but the
    // widget itself only reports afterward whether anything actually
    // changed. Called on the "nothing changed" branch to avoid piling up a
    // no-op undo step every single frame the section is open.
    void discardUndo();
    void undo();
    void redo();

    // Animation timeline
    u32 currentFrame() const
    {
        return mCurrentFrame;
    }
    void setCurrentFrame(u32 frame);
    u32 totalFrames() const
    {
        return mTotalFrames;
    }
    bool isPlaying() const
    {
        return mPlaying;
    }
    void play();
    void stop();

    // Keyframe operations
    void insertKeyframe();
    void deleteKeyframe(u32 frame);
    bool hasKeyframe(u32 frame) const;

    // Skeleton/animation - loadMesh() tries importSkeleton() on the same file
    // right after a skinned mesh loads (an FBX rig carries both together);
    // appendAnimation() is the separate "Import Animation..." flow for a
    // walk/run/etc. clip file authored against the same skeleton.
    bool hasSkeleton() const
    {
        return mHasSkeleton;
    }
    const Skeleton& skeleton() const
    {
        return mSkeleton;
    }
    const std::vector<Math::mat4>& globalPose() const
    {
        return mGlobalPose;
    }
    const std::vector<Math::mat4>& bonePalette() const
    {
        return mBonePalette;
    }
    bool appendAnimation(const std::string& path);
    // Merges another mesh file's geometry into the current one - Append on a
    // static asset in the Materials browser. AssetManager::mergeMeshes()
    // preserves each source's own submeshes/materials rather than folding
    // them together, so the result is "both meshes, one MeshData", not one
    // reduced to the other's material.
    bool appendMesh(const std::string& path);
    // What the Materials browser's context menu Append item actually calls:
    // tries the file as an animation clip against the loaded skeleton first
    // (the common one-rig/many-clips workflow), falls back to appendMesh()
    // otherwise - the same "decide from what the file has, not what the
    // extension implies" Load/Import already do.
    bool appendAsset(const std::string& path);
    // Queues the "Append Animation..." file dialog - the public entry point
    // TimelinePanel's own Add button uses, since openFileDialog() itself
    // stays private (every other panel goes through app() methods like
    // loadMesh()/importMesh(), never the dialog plumbing directly).
    void requestAppendAnimation();
    void removeAnimationClip(u32 index);
    usize animationClipCount() const
    {
        return mAnimationClips.size();
    }
    const AnimationClip& animationClip(usize index) const
    {
        return mAnimationClips[index];
    }
    s32 activeAnimationClip() const
    {
        return mActiveClip;
    }
    void setActiveAnimationClip(s32 index);

    // Operations marking dirty state
    void markDirty();
    bool isDirty() const
    {
        return mDirty;
    }

    // 3D cursor position for operations
    Math::vec3 cursor3D() const
    {
        return mCursor3D;
    }
    void setCursor3D(const Math::vec3& pos)
    {
        mCursor3D = pos;
    }

    // Save/Export
    bool saveAs(const std::string& path);
    bool exportObj(const std::string& path);

    s32 selectedSubmesh() const
    {
        return mSelectedSubmesh;
    }
    void setSelectedSubmesh(s32 index)
    {
        mSelectedSubmesh = index;
    }
    bool deleteSubmesh(u32 index);

    // Viewport-only visibility, no equivalent in SubMesh itself - toggled per
    // row in the Properties panel, read by ViewportPanel to skip a draw. Not
    // part of the saved mesh; grows to fit as submeshes are added/removed and
    // defaults every new entry to visible.
    bool isSubmeshVisible(u32 index);
    void setSubmeshVisible(u32 index, bool visible);
    void toggleSubmeshVisible(u32 index);

    // Which elements the selection is allowed to reach. Hiding a submesh is
    // not only about the viewport: staying out of the way of the selection is
    // most of what it is for, so a hidden one is unreachable by clicking, by
    // box, and by Select All alike. A vertex stays reachable while any
    // visible triangle still uses it - it is as much theirs as the hidden
    // one's. Both masks come back sized to the mesh, all true when there are
    // no submeshes to hide behind.
    void buildSelectableMask(std::vector<bool>& faceSelectable,
                             std::vector<bool>& vertexSelectable);

private:
    // logSink() is a plain function pointer with no `this` to route through -
    // see BlenderApplication.cpp.
    static void logSink(LogLevel level, const char* message);

    void buildPanels();
    void runFrame(f32 deltaTime);
    void drawDockspace();
    void drawMainMenuBar();
    void drawSelectMenu();
    void drawVertexMenu();
    void drawEdgeMenu();
    void drawFaceMenu();
    void deleteSelectedVertices();
    void deleteSelectedFaces();
    void extrudeSelectedFaces();
    void trimUndoStates();
    void handleShortcuts();
    void selectAllElements();
    void invertElementSelection();
    void deleteSelected();
    // Deselects whatever the last operation reached inside a hidden submesh.
    // Cheaper and far harder to get wrong than teaching every selection
    // operation the visibility rules of its own accord.
    void dropHiddenFromSelection();
    void growSelection();
    void shrinkSelection();
    void selectLinked();
    void selectSubmeshFaces(u32 submeshIndex);
    void groupSelectedFacesIntoSubmesh();
    // The primitives the engine already builds. Kept in the order the Add
    // menu lists them, and used to pick which parameters the popup shows.
    enum class PrimitiveType : u8
    {
        Box,
        Plane,
        Sphere,
        Cylinder,
        Cone,
        Capsule,
        Torus,
        // Not a shape on its own: a plane displaced by a heightmap image,
        // which the popup asks for before it can build anything.
        Hills
    };

    // Throws the whole document away: mesh, skeleton, clips, selection and
    // both undo stacks. Not undoable, which is why anything unsaved asks
    // first.
    void newDocument();
    void drawNewConfirmPopup();

    void drawUnwrapPopup();
    void drawBisectPopup();
    // Replaces the mesh with the convex hull of its own points. What a
    // collision proxy is built from, and it is not reversible except by undo.
    bool makeConvexHull();
    // Keeps one side of a plane, cutting the triangles that cross it. Unlike
    // the CSG path this preserves the mesh and its UVs on the side that stays.
    bool bisectMesh();
    bool extractSelectedSubmesh();
    // xatlas: a non-overlapping atlas, splitting vertices at chart seams, so
    // the mesh comes back with a different vertex count than it went in with.
    bool unwrapUVs();

    static const char* primitiveName(PrimitiveType type);
    void drawAddMenu();
    void drawPrimitivePopup();
    // `replace` starts a new mesh from the primitive; otherwise it is merged
    // into the one already loaded, keeping both sets of submeshes.
    bool createPrimitive(bool replace);

    void drawTransformMenu();
    // Bakes `matrix` into the selected vertices, or the whole mesh when
    // nothing is selected, around their own median point.
    void applyTransform(const Math::mat4& matrix, const char* verb);
    void drawMeshMenu();
    void drawToolPopups();
    void drawSaveInfoPopup();
    void drawStatusBar();
    void drawPreferencesPopup();

    // Same split as EditorApplication's own openFileDialog()/drawFileDialog():
    // Open() only queues the dialog and remembers what the result is for,
    // drawFileDialog() renders it every frame until it has one and routes it.
    enum FileDialogAction
    {
        FileDialogNone,
        FileDialogLoadMesh,
        FileDialogImportMesh,
        FileDialogSaveMesh,
        FileDialogExportObj,
        FileDialogAppendAnimation,
        // Picks the heightmap a Hills plane is shaped by, without loading
        // anything: the path goes back into the primitive popup.
        FileDialogHeightmap,
    };
    void openFileDialog(ImGuiFileDialog::Mode mode, FileDialogAction action);
    void drawFileDialog();
    // The File menu's "Open Recent" submenu - one entry per
    // BlenderSettings::GeneralSettings::recentFiles, promoting/clearing
    // through the BlenderSettings methods that own the list's rules.
    void drawOpenRecentMenu();

    Engine& mEngine;
    MiniRenderer mRenderer;
    MiniBatch mBatch;
    BlenderSelection mSelection;
    BlenderSettings mSettings;
    std::vector<BlenderPanel*> mPanels; // owned

    // Current mesh
    MeshHandle mCurrentMesh;
    MeshData* mMeshData = nullptr;

    // Timeline
    u32 mCurrentFrame = 0;
    u32 mTotalFrames = 120;
    bool mPlaying = false;
    f32 mPlaybackTimer = 0.0f;

    // Skeleton/animation - mLocalPose/mGlobalPose/mBonePalette are rebuilt by
    // updateAnimationPose() (called from runFrame()) every frame there is a
    // skeleton at all, scrubbing included, not just while mPlaying.
    static constexpr f32 kAnimationFramesPerSecond = 24.0f;
    void updateAnimationPose();
    // Shared by appendAnimation() and appendAsset(): names the clip from the
    // source file when the importer left it unnamed, pushes it, logs, and
    // selects it - the "just imported, make it the one playing" convention
    // both entry points want.
    void addAnimationClip(AnimationClip clip, const std::string& sourcePath);
    Skeleton mSkeleton;
    bool mHasSkeleton = false;
    std::vector<AnimationClip> mAnimationClips;
    s32 mActiveClip = -1;
    std::vector<LocalPose> mLocalPose;
    std::vector<Math::mat4> mGlobalPose;
    std::vector<Math::mat4> mBonePalette;

    // State
    bool mDirty = false;
    // Set from inside the File/Windows menus, consumed (OpenPopup() + reset)
    // at the top of the matching draw*Popup() - see its own comment for why
    // OpenPopup() cannot run directly at the MenuItem call site.
    bool mSaveInfoRequested = false;
    bool mNewConfirmRequested = false;
    bool mPreferencesRequested = false;
    Math::vec3 mCursor3D = Math::vec3(0.0f);
    s32 mSelectedSubmesh = -1;
    std::vector<bool> mSubmeshVisible;
    std::string mSettingsPath;

    // Undo/Redo - two plain stacks of whole-mesh snapshots, not an index into
    // one shared list: recordUndo() pushes the state *before* an edit onto
    // mUndoStates and drops mRedoStates (a new edit invalidates any redo
    // branch); undo()/redo() swap the current mesh with the top of the other
    // stack. MeshData is only vectors and strings, so copying it whole per
    // edit is a memcpy-shaped cost, not a serialize/deserialize one.
    std::vector<MeshData> mUndoStates;
    std::vector<MeshData> mRedoStates;

    f32 mWeldDistance = 0.001f;
    f32 mSmoothingStrength = 0.5f;
    f32 mExtrudeDistance = 0.5f;
    PrimitiveType mPrimitiveType = PrimitiveType::Box;
    Math::vec3 mPrimitiveSize = Math::vec3(1.0f);
    f32 mPrimitiveRadius = 0.5f;
    f32 mPrimitiveMinorRadius = 0.2f;
    f32 mPrimitiveHeight = 1.0f;
    s32 mPrimitiveRings = 16;
    s32 mPrimitiveSlices = 24;
    s32 mPrimitiveSegmentsX = 8;
    s32 mPrimitiveSegmentsZ = 8;
    f32 mPrimitiveUvTiles = 1.0f;
    f32 mPrimitiveHeightScale = 1.0f;
    std::string mPrimitiveHeightmap;
    bool mPrimitivePopupRequested = false;

    // Zero resolution is the setting that guarantees a single page, which is
    // the only kind anything downstream here copes with.
    s32 mUnwrapResolution = 0;
    s32 mUnwrapPadding = 4;
    f32 mUnwrapTexelsPerUnit = 0.0f;
    // 0 writes the result into the ordinary UVs, 1 leaves it in UV2 where a
    // lightmap bake expects to find it.
    s32 mUnwrapTarget = 0;
    bool mUnwrapPopupRequested = false;

    s32 mBisectAxis = 1;
    f32 mBisectOffset = 0.0f;
    bool mBisectKeepPositive = true;
    bool mBisectPopupRequested = false;

    f32 mScaleFactor = 1.0f;
    f32 mRotationAngle = 0.0f;
    s32 mRotationAxis = 1;

    // Geometry as it was when the gizmo drag began. Only the arrays a
    // transform touches are kept; everything else on the mesh is untouched
    // by the drag and would be dead weight to copy every time.
    bool mGizmoDragging = false;
    std::vector<u32> mGizmoIndices;
    std::vector<Math::vec3> mGizmoPositions;
    std::vector<Math::vec3> mGizmoNormals;
    std::vector<Math::vec4> mGizmoTangents;
    std::vector<u32> mGizmoWinding;

    bool mSmoothNormals = true;
    bool mAngleWeightedNormals = false;
    s32 mUvMode = 0;
    f32 mUvResolutionU = 1.0f;
    f32 mUvResolutionV = 1.0f;
    s32 mSplitTargetTriangles = 4000;
    f32 mOverdrawThreshold = 1.05f;
    f32 mSimplifyRatio = 0.5f;
    f32 mSimplifyError = 0.01f;
    std::string mMeshToolStatus;

    // Docking layout
    bool mDockLayoutBuilt = false;

    ImGuiFileDialog mFileDialog;
    FileDialogAction mFileDialogAction = FileDialogNone;
};

} // namespace Radion

#endif // RADION_BLENDER_APPLICATION_H
