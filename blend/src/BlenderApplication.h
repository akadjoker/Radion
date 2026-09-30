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
#include "mesh/MeshBoolean.h"
#include "mesh/MeshUv.h"
#include "mesh/MeshTopology.h"
#include "ViewportCamera.h"
#include "Types.h"

#include "Math.h"
#include <memory>
#include <string>
#include <vector>

namespace Radion
{
class Engine;
class BlenderPanel;
struct MeshData;

namespace BlenderApi
{
class BlenderApiHost;
}

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

    MeshData* currentMeshData();
    bool loadMesh(const std::string& path);
    bool importMesh(const std::string& path);
    // `positionsOnly` (a gizmo drag) skips the connectivity rebuild: the same vertices still stand together.
    bool applyMeshEdit(bool positionsOnly = false);
    void recordUndo();

    // UVs of the selected faces, or of the whole mesh when nothing is selected; one undo step.
    void applyFaceUVTransform(const Math::vec2& scale, f32 rotationDegrees,
                              const Math::vec2& offset);

    // Median of the selected vertices, or of the whole mesh when nothing is selected; origin for an empty mesh.
    Math::vec3 transformPivot();
    // Takes the undo snapshot and keeps the original geometry so each drag frame transforms it, not the last result
    // (compounding drifts). False when there is nothing to drag.
    bool beginGizmoDrag();
    // `worldDelta` maps a vertex's position at drag start to its current one; it carries its own pivot.
    void updateGizmoDrag(const Math::mat4& worldDelta);
    void endGizmoDrag();
    bool gizmoDragging() const
    {
        return mGizmoDragging;
    }
    // Drops the latest recordUndo() snapshot: for live widgets where recordUndo() must run before the widget but changes
    // are only known afterward.
    void discardUndo();
    void undo();
    void redo();
    bool canUndo() const
    {
        return !mUndoStates.empty();
    }
    bool canRedo() const
    {
        return !mRedoStates.empty();
    }
    usize undoDepth() const
    {
        return mUndoStates.size();
    }

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

    void insertKeyframe();
    void deleteKeyframe(u32 frame);
    bool hasKeyframe(u32 frame) const;

    // loadMesh() tries importSkeleton() on the same file after a skinned load; appendAnimation() is the separate clip-
    // import flow.
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
    // Merges another mesh file into the current one, preserving each source's submeshes/materials.
    bool appendMesh(const std::string& path);
    // Tries the file as an animation clip against the loaded skeleton first, falls back to appendMesh(): decides from
    // contents, not extension.
    bool appendAsset(const std::string& path);
    // Queues the "Append Animation..." file dialog; openFileDialog() itself stays private.
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

    void markDirty();
    bool isDirty() const
    {
        return mDirty;
    }

    Math::vec3 cursor3D() const
    {
        return mCursor3D;
    }
    void setCursor3D(const Math::vec3& pos)
    {
        mCursor3D = pos;
    }

    bool saveAs(const std::string& path);
    bool exportObj(const std::string& path);
    // Binary glTF 2.0 (.glb); static geometry only, see GltfExporter.
    bool exportGltf(const std::string& path, std::string* error = nullptr,
                    std::vector<std::string>* warnings = nullptr);

    s32 selectedSubmesh() const
    {
        return mSelectedSubmesh;
    }
    void setSelectedSubmesh(s32 index)
    {
        mSelectedSubmesh = index;
    }
    bool deleteSubmesh(u32 index);

    // Viewport-only visibility, not part of the saved mesh; grows to fit, new entries default to visible.
    bool isSubmeshVisible(u32 index);
    void setSubmeshVisible(u32 index, bool visible);
    void toggleSubmeshVisible(u32 index);

    // Which elements the selection may reach: a hidden submesh is unreachable by click, box and Select All; a vertex
    // stays reachable while a visible triangle uses it.
    // Both masks are sized to the mesh, all true when there are no submeshes.
    void buildSelectableMask(std::vector<bool>& faceSelectable,
                             std::vector<bool>& vertexSelectable);

    // Connectivity of the current mesh, rebuilt lazily after changes.
    const MeshTopology& topology();
    // Bumped by every change to the mesh; lets a cache know it is stale.
    u64 meshRevision() const
    {
        return mMeshRevision;
    }
    // The vertices an edit acts on: selected vertices, corners of selected faces, ends of selected edges, widened to
    // coincident vertices so a move never tears a seam. Empty when nothing is selected.
    std::vector<u32> editVertices();
    std::vector<u64> allEdgeKeys();

    // Rounds the selection (whole mesh when none) to multiples of `step`; returns how many vertices moved.
    u32 snapSelectionToGrid(f32 step);
    // Moves each selected point onto the nearest unselected point within `tolerance` (closes a gap exactly, unlike a
    // weld); coincident vertices move together. Returns points moved.
    u32 snapSelectionToVertices(f32 tolerance);
    // The vertices the running gizmo drag is moving (empty when none is).
    const std::vector<u32>& gizmoVertices() const
    {
        return mGizmoIndices;
    }

    // Topology edits (see mesh/MeshEdit.h): one undo step each, act on the selection, return false with `error` set on
    // failure.
    // The triangles the selection stands for: with `partial`, any with a selected corner or edge, otherwise only fully
    // selected ones (face mode: the selected faces).
    std::vector<u32> selectionFaces(bool partial);
    bool subdivideSelection(u32 levels, bool smooth, std::string* error = nullptr);
    // One selected edge at a time; returns how many were changed.
    u32 turnSelectedEdges(std::string* error = nullptr);
    u32 splitSelectedEdges(f32 t, std::string* error = nullptr);
    u32 collapseSelectedEdges(f32 t, std::string* error = nullptr);
    // Cuts along the plane dot(normal, p) = offset and selects the new line of edges (edge mode).
    bool knifeCut(const Math::vec3& normal, f32 offset, std::string* error = nullptr);
    // Adds `cuts` edge loops across the quad ring through the first selected edge, and selects them.
    bool loopCutSelected(u32 cuts, std::string* error = nullptr);
    bool insetSelection(f32 thickness, f32 depth, std::string* error = nullptr);
    bool bevelSelectedEdges(f32 width, std::string* error = nullptr);
    // Closes the open borders with a selected edge (every open border when none). Returns how many were closed.
    u32 fillHoles(u32 maxEdges, std::string* error = nullptr);
    // Joins the two open borders of the selected edges (or the mesh's only two) with a strip of triangles.
    bool bridgeBorders(std::string* error = nullptr);
    // Adds a mirror image of the selected faces (whole mesh when none) across the plane where `axis` equals `offset`;
    // geometry within `weld` of the plane is shared.
    bool mirrorGeometry(s32 axis, f32 offset, f32 weld, std::string* error = nullptr);
    // Union, difference (first minus second) or intersection of two closed solids into one new part replacing them: a
    // remesh on a grid of `resolution` cells along the longest side, not an exact cut (MeshEdit::booleanMeshes).
    bool booleanParts(MeshEdit::BooleanOp op, u32 partA, u32 partB, u32 resolution, const std::string& name,
                      s32* resultPart = nullptr, std::string* error = nullptr);
    // Joins parts (submeshes) into the lowest-numbered of them.
    bool mergeParts(const std::vector<u32>& parts, std::string* error = nullptr);
    // Makes the selected faces a part of their own. `newPart` receives its index.
    bool separateSelectedFaces(s32* newPart = nullptr, std::string* error = nullptr);

    // While on, moving/rotating/scaling vertices mirrors the change onto the vertices opposite across the plane;
    // vertices on the plane stay on it. `axis` is -1 (off), 0, 1 or 2.
    void setSymmetry(s32 axis, f32 offset);
    s32 symmetryAxis() const
    {
        return mSymmetryAxis;
    }
    f32 symmetryOffset() const
    {
        return mSymmetryOffset;
    }
    // The vertices that stand opposite `vertices` across the symmetry plane and are
    // not themselves in it or on the plane. Empty when symmetry is off.
    std::vector<u32> symmetryPartners(const std::vector<u32>& vertices) const;
    // Applies `world` (a transform in world space) to `vertices`, and its mirror
    // image to their partners when symmetry is on. An empty list is the whole mesh.
    void transformVerticesWorld(const Math::mat4& world, const std::vector<u32>& vertices);

    // Hiding works on triangles (the selected faces, or every triangle using a selected vertex or edge); a vertex or
    // edge hides when everything using it is.
    // Hidden triangles are not drawn or selectable and stay put on edits, but an edit that adds or removes triangles
    // unhides all.
    bool hideSelected();
    // Hides everything that is not entirely selected.
    bool hideUnselected();
    void unhideAll();
    bool hasHidden() const
    {
        return mHasHidden;
    }
    usize hiddenFaceCount() const;
    bool isFaceHidden(u32 face) const
    {
        return mHasHidden && face < mHiddenFaces.size() && mHiddenFaces[face] != 0;
    }
    u64 hiddenRevision() const
    {
        return mHiddenRevision;
    }
    // One byte per vertex, 1 where every triangle that uses it is hidden. Empty
    // when nothing is hidden.
    const std::vector<u8>& hiddenVertexFlags();

    // What the menus and the HTTP API both call: parameters passed in, one undo step per call, false (document
    // untouched) when there was nothing to do.

    enum class PrimitiveType : u8
    {
        Box,
        Plane,
        Sphere,
        Cylinder,
        Cone,
        Capsule,
        Torus,
        // Not a shape on its own: a plane displaced by a heightmap image.
        Hills
    };

    static const char* primitiveName(PrimitiveType type);
    static bool primitiveTypeFromName(const std::string& name, PrimitiveType& out);

    // Which fields matter depends on `type` (Box reads `size`, Sphere `radius`/`rings`/`slices`, ...).
    struct PrimitiveParams
    {
        PrimitiveType type = PrimitiveType::Box;
        Math::vec3 size = Math::vec3(1.0f);
        f32 radius = 0.5f;
        f32 minorRadius = 0.2f;
        f32 height = 1.0f;
        s32 rings = 16;
        s32 slices = 24;
        s32 segmentsX = 8;
        s32 segmentsZ = 8;
        f32 uvTiles = 1.0f;
        f32 heightScale = 1.0f;
        std::string heightmap;
    };

    // The material name doubles as the part's name (SubMesh has none); unset fields keep the material default.
    struct PartStyle
    {
        std::string name;
        bool hasColor = false;
        Math::vec4 color = Math::vec4(1.0f);
        bool hasRoughness = false;
        f32 roughness = 0.5f;
        bool hasMetallic = false;
        f32 metallic = 0.0f;
    };

    static bool buildPrimitive(const PrimitiveParams& params, MeshData& out);
    // Builds a primitive, places it and adds it as its own submesh (or starts a new mesh when `replace` or the mesh is
    // empty); `submeshOut` receives its index.
    bool createPrimitive(const PrimitiveParams& params, const Math::mat4& placement,
                         const PartStyle& style, bool replace, s32* submeshOut = nullptr);
    // The same for geometry not from a primitive; takes the part by value. `undoStep` false leaves the snapshot to the
    // caller (an op that also removes parts takes one for both).
    bool appendPart(MeshData part, const Math::mat4& placement, const PartStyle& style,
                    const char* sourceName, bool replace, s32* submeshOut = nullptr, bool undoStep = true);

    // `matrix` acts about `pivot`; a mirroring matrix has the winding turned back out.
    bool transformSubmesh(u32 index, const Math::mat4& matrix, const Math::vec3& pivot);
    bool duplicateSubmesh(u32 index, const Math::mat4& placement, s32* newIndex = nullptr);
    // Submeshes sharing the material slot get their own copy first, so only this one changes. `undoStep` false when the
    // caller already recorded the step.
    bool styleSubmesh(u32 index, const PartStyle& style, bool undoStep = true);
    // Vertex colour painting, LINEAR RGBA (MeshPaint); `opacity` blends into what is there. Each returns vertices
    // changed, one undo step.
    // paintSelection() widens to coincident vertices and fails (0, `error` set) with nothing selected; paintSphere() is
    // a soft round brush, `part` < 0 = whole mesh.
    u32 paintSelection(const Math::vec4& color, f32 opacity, std::string* error = nullptr);
    u32 paintPart(u32 part, const Math::vec4& color, f32 opacity);
    u32 paintAll(const Math::vec4& color, f32 opacity);
    u32 paintSphere(const Math::vec3& center, f32 radius, f32 hardness, const Math::vec4& color, f32 opacity, s32 part);
    // Clears the selection's, one part's (`part` >= 0) or everything's colours.
    bool clearVertexColors(s32 part, bool selectionOnly, std::string* error = nullptr);
    bool hasVertexColors() const;

    // UVs are edited per VERTEX, so two vertices at one point can hold different UVs (see mesh/MeshUv.h); one undo step
    // per call.
    enum class UvTarget
    {
        Selection, // selected faces' vertices; else selected vertices/edges (widened to coincident ones)
        Island,
        Part,
        All
    };
    // The vertices a target names. False with `error` set when there are none.
    bool uvTargetVertices(UvTarget target, s32 part, std::vector<u32>& vertices, std::string* error = nullptr);
    std::vector<u32> selectedTriangles();
    // Moves/rotates/scales UVs about `pivot`; pinned vertices stay. Returns how many moved.
    u32 transformUvs(const std::vector<u32>& vertices, const Math::vec2& pivot, const MeshUv::Transform& change);
    u32 fitUvs(const std::vector<u32>& vertices, bool keepAspect, f32 margin);
    // Fits every part into its own 0..1 square - what per-part textures need after one shared unwrap.
    u32 fitUvsPerPart(bool keepAspect, f32 margin);
    // Box projection of a target's triangles; returns vertices added by splitting.
    bool boxMapUvs(UvTarget target, s32 part, f32 tile, const Math::vec2& offset, u32* added, std::string* error);
    // Pins: transform/fit skip pinned vertices. Forgotten when the vertex count changes.
    void setUvPinned(const std::vector<u32>& vertices, bool pinned);
    void clearUvPins();
    const std::vector<u8>* uvPinned();
    u32 uvPinnedCount();

    // Puts an image on one of a part's material slots (SlotAlbedo/Normal/Surface/Emissive); an empty path clears it. The
    // part gets a private material first, like styleSubmesh().
    bool setPartTexture(u32 index, u32 slot, const std::string& path, std::string* error = nullptr);
    std::vector<u32> submeshVertices(u32 index) const;

    // Bakes `matrix` into the selected vertices (whole mesh when none) around their median point.
    void applyTransform(const Math::mat4& matrix, const char* verb);
    bool extrudeFaces(f32 distance);
    void deleteSelectedVertices();
    void deleteSelectedFaces();
    void deleteSelectedEdges();
    void deleteSelected();
    void selectAllElements();
    void invertElementSelection();
    void growSelection();
    void shrinkSelection();
    void selectLinked();
    void selectSubmeshFaces(u32 submeshIndex);
    void groupSelectedFacesIntoSubmesh();
    // Deselects whatever the last operation reached inside a hidden submesh; simpler than teaching every selection op
    // the visibility rules.
    void dropHiddenFromSelection();

    // Replaces the mesh with the convex hull of its points (for collision proxies); only undo reverses it.
    bool makeConvexHull();
    // Keeps one side of a plane, cutting triangles that cross it; unlike CSG it preserves the mesh and UVs on the kept
    // side.
    bool bisectMesh(s32 axis, f32 offset, bool keepPositive);
    bool extractSelectedSubmesh();

    // xatlas atlas: splits vertices at chart seams, so the vertex count changes.
    struct UnwrapParams
    {
        u32 resolution = 0;
        u32 padding = 4;
        f32 texelsPerUnit = 0.0f;
        // 0 writes the result into the ordinary UVs, 1 into UV2.
        s32 target = 0;
    };
    bool unwrapUVs(const UnwrapParams& params);

    // Throws away the whole document (mesh, skeleton, clips, selection, both undo stacks); not undoable, so unsaved work
    // asks first.
    void newDocument();

    // Commands arriving over HTTP are queued and run here on the frame loop, seeing exactly what the panels do.
    bool startApi(const std::string& host, int port, const std::string& token,
                  std::string* error = nullptr);
    void stopApi();
    bool apiRunning() const;
    int apiPort() const;
    bool apiHasToken() const;

    // Renders like a viewport into a private target; returns RGBA8 pixels, top row first. Must run on the GL context's
    // thread.
    struct CaptureParams
    {
        s32 width = 960;
        s32 height = 540;
        CameraView view = CameraView::Perspective;
        // Frame the whole mesh when true; otherwise use `camera` as given.
        bool frame = true;
        CameraState camera;
        MiniRenderMode shading = MiniRenderMode::Textured;
        bool wireframeOverlay = false;
        bool colorBySubmesh = false;
        bool vertexColors = true;
        bool grid = true;
    };
    bool captureViewport(const CaptureParams& params, std::vector<u8>& rgba);

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
    void trimUndoStates();
    void handleShortcuts();
    // Throws away the whole document (mesh, skeleton, clips, selection, both undo stacks); not undoable, so unsaved work
    // asks first.
    void drawNewConfirmPopup();

    void drawUnwrapPopup();
    void drawBisectPopup();
    // The popup's own forms of the public operations below, fed from the parameters it keeps between frames.
    bool bisectMesh();
    bool unwrapUVs();
    PrimitiveParams primitiveParamsFromUi() const;

    void drawAddMenu();
    void drawPrimitivePopup();
    // `replace` starts a new mesh from the primitive; otherwise it is merged
    // into the one already loaded, keeping both sets of submeshes.
    bool createPrimitive(bool replace);

    void drawTransformMenu();
    std::vector<u8> mUvPinned;
    void drawPaintMenu();
    bool hasAnySelection()
    {
        return mSelection.selectedVertexCount() > 0 || mSelection.selectedFaceCount() > 0 ||
               mSelection.selectedEdgeCount() > 0;
    }
    Math::vec3 mPaintColor = Math::vec3(0.85f, 0.2f, 0.15f); // sRGB, as the picker shows it
    f32 mPaintOpacity = 1.0f;
    f32 mPaintRadius = 0.5f;
    f32 mPaintHardness = 0.5f;
    void drawMeshMenu();
    void drawToolPopups();
    void drawSaveInfoPopup();
    void drawStatusBar();
    void drawPreferencesPopup();

    // Same split as EditorApplication's openFileDialog()/drawFileDialog(): Open() only queues and remembers the purpose.
    enum FileDialogAction
    {
        FileDialogNone,
        FileDialogLoadMesh,
        FileDialogImportMesh,
        FileDialogSaveMesh,
        FileDialogExportObj,
        FileDialogExportGltf,
        FileDialogAppendAnimation,
        // Picks the heightmap a Hills plane is shaped by, without loading
        // anything: the path goes back into the primitive popup.
        FileDialogHeightmap,
    };
    void openFileDialog(ImGuiFileDialog::Mode mode, FileDialogAction action);
    void drawFileDialog();
    void drawOpenRecentMenu();

    Engine& mEngine;
    MiniRenderer mRenderer;
    MiniBatch mBatch;
    BlenderSelection mSelection;
    BlenderSettings mSettings;
    std::vector<BlenderPanel*> mPanels; // owned
    std::unique_ptr<BlenderApi::BlenderApiHost> mApi;
    // The preferences' own copy of the port and token while they are edited.
    s32 mApiPortField = 7420;
    std::string mApiTokenField;
    std::string mApiError;

    MeshHandle mCurrentMesh;
    MeshData* mMeshData = nullptr;
    MeshTopology mTopology;
    u64 mMeshRevision = 1;
    u64 mTopologyRevision = 0;
    std::vector<u8> mHiddenFaces;
    std::vector<u8> mHiddenVertices;
    bool mHasHidden = false;
    u64 mHiddenRevision = 0;
    u64 mHiddenVerticesRevision = ~u64(0);
    void setHiddenFaces(std::vector<u8>&& faces);
    // Forgets the hidden set when the triangle count no longer matches it.
    void validateHidden();
    // Drops one part's triangles and its entry, with no undo snapshot and no refresh.
    // Gives the part its own material slot if it shares one (or has none).
    void ownMaterial(u32 index);
    void removeSubmeshData(u32 index);

    u32 mCurrentFrame = 0;
    u32 mTotalFrames = 120;
    bool mPlaying = false;
    f32 mPlaybackTimer = 0.0f;

    // mLocalPose/mGlobalPose/mBonePalette are rebuilt by updateAnimationPose() every frame there is a skeleton,
    // scrubbing included.
    static constexpr f32 kAnimationFramesPerSecond = 24.0f;
    void updateAnimationPose();
    // Names the clip from the source file if the importer left it unnamed, pushes, logs and selects it; shared by
    // appendAnimation() and appendAsset().
    void addAnimationClip(AnimationClip clip, const std::string& sourcePath);
    Skeleton mSkeleton;
    bool mHasSkeleton = false;
    std::vector<AnimationClip> mAnimationClips;
    s32 mActiveClip = -1;
    std::vector<LocalPose> mLocalPose;
    std::vector<Math::mat4> mGlobalPose;
    std::vector<Math::mat4> mBonePalette;

    bool mDirty = false;
    // Set inside the menus, consumed (OpenPopup() + reset) at the top of the matching draw*Popup(); see its comment for
    // why OpenPopup() cannot run at the MenuItem.
    bool mSaveInfoRequested = false;
    bool mNewConfirmRequested = false;
    bool mPreferencesRequested = false;
    Math::vec3 mCursor3D = Math::vec3(0.0f);
    s32 mSelectedSubmesh = -1;
    std::vector<bool> mSubmeshVisible;
    std::string mSettingsPath;

    // Two stacks of whole-mesh snapshots: recordUndo() pushes the pre-edit state and clears mRedoStates; undo()/redo()
    // swap the mesh with the other stack's top.
    std::vector<MeshData> mUndoStates;
    std::vector<MeshData> mRedoStates;

    s32 mSymmetryAxis = -1;
    f32 mSymmetryOffset = 0.0f;
    std::vector<u32> mGizmoPartners;
    s32 mSubdivideLevels = 1;
    s32 mLoopCuts = 1;
    f32 mBevelWidth = 0.1f;
    f32 mInsetThickness = 0.1f;
    f32 mInsetDepth = 0.0f;
    s32 mMirrorAxis = 0;
    f32 mMirrorOffset = 0.0f;
    s32 mKnifeAxis = 1;
    f32 mKnifeOffset = 0.0f;
    f32 mWeldDistance = 0.001f;
    f32 mSnapTolerance = 0.05f;
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

    // Zero resolution guarantees a single page, the only kind downstream copes with.
    s32 mUnwrapResolution = 0;
    s32 mUnwrapPadding = 4;
    f32 mUnwrapTexelsPerUnit = 0.0f;
    // 0 writes the result into the ordinary UVs, 1 into UV2 where a lightmap bake expects it.
    s32 mUnwrapTarget = 0;
    bool mUnwrapPopupRequested = false;

    s32 mBisectAxis = 1;
    f32 mBisectOffset = 0.0f;
    bool mBisectKeepPositive = true;
    bool mBisectPopupRequested = false;

    f32 mScaleFactor = 1.0f;
    f32 mRotationAngle = 0.0f;
    s32 mRotationAxis = 1;

    // Geometry when the gizmo drag began; only the arrays a transform touches are kept.
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

    bool mDockLayoutBuilt = false;

    ImGuiFileDialog mFileDialog;
    FileDialogAction mFileDialogAction = FileDialogNone;
};

} // namespace Radion

#endif // RADION_BLENDER_APPLICATION_H
