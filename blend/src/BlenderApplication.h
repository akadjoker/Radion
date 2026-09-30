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

#include <glm/vec3.hpp>
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

    // Mesh data access
    MeshData* currentMeshData();
    bool loadMesh(const std::string& path);
    bool importMesh(const std::string& path);
    // `positionsOnly` is for an edit that moves vertices without changing which
    // ones stand together (a gizmo drag): connectivity stays valid, so it is not
    // rebuilt.
    bool applyMeshEdit(bool positionsOnly = false);
    void recordUndo();

    // Reworks the UVs of the selected faces, or of the whole mesh when
    // nothing is selected. One undo step per call.
    void applyFaceUVTransform(const glm::vec2& scale, f32 rotationDegrees,
                              const glm::vec2& offset);

    // -- gizmo drag
    //
    // Where the gizmo sits: the median of the selected vertices, or of the
    // whole mesh when nothing is selected. Origin for an empty mesh.
    glm::vec3 transformPivot();
    // Takes the undo snapshot and remembers the geometry as it stands, so
    // every frame of the drag transforms the original rather than the last
    // frame's result - compounding a few hundred matrices visibly drifts.
    // False when there is nothing to drag.
    bool beginGizmoDrag();
    // `worldDelta` maps a vertex's position at the start of the drag to where
    // it belongs now; it carries its own pivot.
    void updateGizmoDrag(const glm::mat4& worldDelta);
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
    const std::vector<glm::mat4>& globalPose() const
    {
        return mGlobalPose;
    }
    const std::vector<glm::mat4>& bonePalette() const
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
    glm::vec3 cursor3D() const
    {
        return mCursor3D;
    }
    void setCursor3D(const glm::vec3& pos)
    {
        mCursor3D = pos;
    }

    // Save/Export
    bool saveAs(const std::string& path);
    bool exportObj(const std::string& path);
    // Binary glTF 2.0 (.glb), one primitive and PBR material per submesh. Static
    // geometry only; see GltfExporter.
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

    // Connectivity of the mesh as it is now, rebuilt lazily when the mesh has
    // changed since the last call. See MeshTopology for what "vertex" means there.
    const MeshTopology& topology();
    // Bumped by every change to the mesh; lets a cache know it is stale.
    u64 meshRevision() const
    {
        return mMeshRevision;
    }
    // The vertices an edit acts on: the selected vertices, the corners of the
    // selected faces and the ends of the selected edges, each widened to every
    // vertex standing at the same point so a move never tears a seam open. Empty
    // when nothing is selected.
    std::vector<u32> editVertices();
    // Every edge of the mesh as selection keys, for Select All in edge mode.
    std::vector<u64> allEdgeKeys();

    // -- Snap
    //
    // Rounds the positions of the selection (the whole mesh when nothing is
    // selected) to multiples of `step`. Returns how many vertices moved.
    u32 snapSelectionToGrid(f32 step);
    // Moves each selected point onto the nearest point that is not selected, when
    // one lies within `tolerance` - closing a gap exactly, where a weld would
    // merge them. Coincident vertices move together. Returns how many points moved.
    u32 snapSelectionToVertices(f32 tolerance);
    // The vertices the running gizmo drag is moving (empty when none is).
    const std::vector<u32>& gizmoVertices() const
    {
        return mGizmoIndices;
    }

    // -- Topology edits (see mesh/MeshEdit.h). Each is one undo step, acts on the
    // current selection and returns false with `error` set when it cannot.
    //
    // The triangles the selection stands for: with `partial` any triangle that
    // has a selected corner or edge, otherwise only those entirely selected.
    // Face mode gives the selected faces either way.
    std::vector<u32> selectionFaces(bool partial);
    // Subdivides the selected faces - every face when nothing is selected.
    bool subdivideSelection(u32 levels, bool smooth, std::string* error = nullptr);
    // One selected edge at a time; returns how many were changed.
    u32 turnSelectedEdges(std::string* error = nullptr);
    u32 splitSelectedEdges(f32 t, std::string* error = nullptr);
    u32 collapseSelectedEdges(f32 t, std::string* error = nullptr);
    // Cuts the mesh along the plane dot(normal, p) = offset and selects the new
    // line of edges (edge mode).
    bool knifeCut(const glm::vec3& normal, f32 offset, std::string* error = nullptr);
    // Adds `cuts` edge loops across the ring of quads through the first selected
    // edge, and selects them.
    bool loopCutSelected(u32 cuts, std::string* error = nullptr);
    // Insets the selected faces as one region and selects the shrunken region.
    bool insetSelection(f32 thickness, f32 depth, std::string* error = nullptr);
    bool bevelSelectedEdges(f32 width, std::string* error = nullptr);
    // Closes the open borders that have a selected edge (every open border when no
    // edge is selected). Returns how many were closed.
    u32 fillHoles(u32 maxEdges, std::string* error = nullptr);
    // Joins the two open borders the selected edges belong to (or the mesh's only
    // two) with a strip of triangles.
    bool bridgeBorders(std::string* error = nullptr);
    // Adds a mirror image of the selected faces (the whole mesh when none) across
    // the plane where coordinate `axis` equals `offset`; geometry within `weld` of
    // the plane is shared so the halves join.
    bool mirrorGeometry(s32 axis, f32 offset, f32 weld, std::string* error = nullptr);
    // Combines two parts as solids - union, difference (first minus second) or
    // intersection - into one new part that replaces them. A remesh on a grid
    // `resolution` cells along the longest side, not an exact cut; see
    // MeshEdit::booleanMeshes. Both parts must be closed solids.
    bool booleanParts(MeshEdit::BooleanOp op, u32 partA, u32 partB, u32 resolution, const std::string& name,
                      s32* resultPart = nullptr, std::string* error = nullptr);
    // Joins parts (submeshes) into the lowest-numbered of them.
    bool mergeParts(const std::vector<u32>& parts, std::string* error = nullptr);
    // Makes the selected faces a part of their own. `newPart` receives its index.
    bool separateSelectedFaces(s32* newPart = nullptr, std::string* error = nullptr);

    // -- Symmetry
    //
    // While it is on, moving, rotating or scaling vertices does the mirror image to
    // the vertices standing opposite them across the plane - model one half and the
    // other follows. Vertices on the plane itself are left as they are moved.
    // `axis` is -1 (off), 0, 1 or 2.
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
    void transformVerticesWorld(const glm::mat4& world, const std::vector<u32>& vertices);

    // -- Hide
    //
    // Hiding works on triangles: the selected faces, or every triangle that uses a
    // selected vertex or edge. A vertex or edge is hidden when everything that
    // uses it is. Hidden triangles are not drawn, cannot be selected, and stay
    // put when the mesh is edited - but an edit that adds or removes triangles
    // shows everything again, since the hidden set would no longer mean the same
    // triangles.
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
    // Bumped whenever the hidden set changes.
    u64 hiddenRevision() const
    {
        return mHiddenRevision;
    }
    // One byte per vertex, 1 where every triangle that uses it is hidden. Empty
    // when nothing is hidden.
    const std::vector<u8>& hiddenVertexFlags();

    // -- Modelling operations
    //
    // Everything below is what the menus call and what the HTTP API calls:
    // one implementation, parameters passed in rather than read from widget
    // state, one undo step per call, and a false return (with the document
    // untouched) when there was nothing to do.

    // The primitives the engine already builds, in the order the Add menu
    // lists them.
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

    static const char* primitiveName(PrimitiveType type);
    static bool primitiveTypeFromName(const std::string& name, PrimitiveType& out);

    // What a primitive is built from. Which fields matter depends on `type`
    // (a Box reads `size`, a Sphere `radius`/`rings`/`slices`, ...) - the same
    // split the Add popup already shows.
    struct PrimitiveParams
    {
        PrimitiveType type = PrimitiveType::Box;
        glm::vec3 size = glm::vec3(1.0f);
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

    // How a new part looks. The material name doubles as the part's name - a
    // SubMesh has none of its own. Fields left unset keep the material default.
    struct PartStyle
    {
        std::string name;
        bool hasColor = false;
        glm::vec4 color = glm::vec4(1.0f);
        bool hasRoughness = false;
        f32 roughness = 0.5f;
        bool hasMetallic = false;
        f32 metallic = 0.0f;
    };

    // The engine's own primitive, built but not added to the document.
    static bool buildPrimitive(const PrimitiveParams& params, MeshData& out);
    // Builds a primitive, places it with `placement` and adds it as its own
    // submesh (or starts a new mesh with it when `replace` or the mesh is
    // empty). `submeshOut` receives the new submesh's index.
    bool createPrimitive(const PrimitiveParams& params, const glm::mat4& placement,
                         const PartStyle& style, bool replace, s32* submeshOut = nullptr);
    // The same for geometry that did not come from a primitive - a part built
    // vertex by vertex. Takes the part by value: it is placed and styled in
    // place before it is merged.
    // `undoStep` false leaves the undo snapshot to the caller (an operation that
    // also removes parts takes one for both).
    bool appendPart(MeshData part, const glm::mat4& placement, const PartStyle& style,
                    const char* sourceName, bool replace, s32* submeshOut = nullptr, bool undoStep = true);

    // Moves/rotates/scales one submesh's vertices. `matrix` acts about `pivot`.
    // A mirroring matrix turns the submesh's winding back the right way out.
    bool transformSubmesh(u32 index, const glm::mat4& matrix, const glm::vec3& pivot);
    // Copies a submesh (and its material) with `placement` applied to the copy.
    bool duplicateSubmesh(u32 index, const glm::mat4& placement, s32* newIndex = nullptr);
    // Restyles the material of one submesh. Other submeshes that share that
    // material slot are given their own copy first, so only this one changes.
    // `undoStep` false when the caller has already recorded the step this restyle belongs to.
    bool styleSubmesh(u32 index, const PartStyle& style, bool undoStep = true);
    // Vertex colour painting. Colours are LINEAR RGBA (MeshPaint); `opacity` blends
    // the colour into what is there. Each returns how many vertices changed and
    // is one undo step. paintSelection() widens the selection to coincident
    // vertices like every other edit and fails (returns 0, `error` set) when
    // nothing is selected. paintSphere() is a soft round brush, optionally
    // limited to one part (`part` < 0 = the whole mesh).
    u32 paintSelection(const glm::vec4& color, f32 opacity, std::string* error = nullptr);
    u32 paintPart(u32 part, const glm::vec4& color, f32 opacity);
    u32 paintAll(const glm::vec4& color, f32 opacity);
    u32 paintSphere(const glm::vec3& center, f32 radius, f32 hardness, const glm::vec4& color, f32 opacity, s32 part);
    // Clears the selection's, one part's (`part` >= 0) or everything's colours.
    bool clearVertexColors(s32 part, bool selectionOnly, std::string* error = nullptr);
    bool hasVertexColors() const;

    // -- UV editing (see mesh/MeshUv.h). UVs are edited per VERTEX, so two vertices at
    // one point can hold different UVs. Each editing call is one undo step.
    enum class UvTarget
    {
        Selection, // selected faces' vertices; else selected vertices/edges (widened to coincident ones)
        Island,    // every UV island the selection touches
        Part,      // one part (`part`)
        All
    };
    // The vertices a target names. False with `error` set when there are none.
    bool uvTargetVertices(UvTarget target, s32 part, std::vector<u32>& vertices, std::string* error = nullptr);
    // Triangles the selection stands for: its faces, or every triangle using a selected vertex.
    std::vector<u32> selectedTriangles();
    // Moves/rotates/scales UVs about `pivot`; pinned vertices stay. Returns how many moved.
    u32 transformUvs(const std::vector<u32>& vertices, const glm::vec2& pivot, const MeshUv::Transform& change);
    u32 fitUvs(const std::vector<u32>& vertices, bool keepAspect, f32 margin);
    // Fits every part into its own 0..1 square - what per-part textures need after one shared unwrap.
    u32 fitUvsPerPart(bool keepAspect, f32 margin);
    // Box projection of a target's triangles; returns vertices added by splitting.
    bool boxMapUvs(UvTarget target, s32 part, f32 tile, const glm::vec2& offset, u32* added, std::string* error);
    // Pins: transform/fit skip pinned vertices. Forgotten when the vertex count changes.
    void setUvPinned(const std::vector<u32>& vertices, bool pinned);
    void clearUvPins();
    const std::vector<u8>* uvPinned();
    u32 uvPinnedCount();

    // Puts an image file on one of a part's material slots (SlotAlbedo,
    // SlotNormal, SlotSurface, SlotEmissive); an empty path clears the slot.
    // The part gets a private material first, like styleSubmesh().
    bool setPartTexture(u32 index, u32 slot, const std::string& path, std::string* error = nullptr);
    // The vertices a submesh's triangles reference, ascending.
    std::vector<u32> submeshVertices(u32 index) const;

    // Bakes `matrix` into the selected vertices, or the whole mesh when
    // nothing is selected, around their own median point.
    void applyTransform(const glm::mat4& matrix, const char* verb);
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
    // Deselects whatever the last operation reached inside a hidden submesh.
    // Cheaper and far harder to get wrong than teaching every selection
    // operation the visibility rules of its own accord.
    void dropHiddenFromSelection();

    // Replaces the mesh with the convex hull of its own points. What a
    // collision proxy is built from, and it is not reversible except by undo.
    bool makeConvexHull();
    // Keeps one side of a plane, cutting the triangles that cross it. Unlike
    // the CSG path this preserves the mesh and its UVs on the side that stays.
    bool bisectMesh(s32 axis, f32 offset, bool keepPositive);
    bool extractSelectedSubmesh();

    // xatlas: a non-overlapping atlas, splitting vertices at chart seams, so
    // the mesh comes back with a different vertex count than it went in with.
    struct UnwrapParams
    {
        u32 resolution = 0;
        u32 padding = 4;
        f32 texelsPerUnit = 0.0f;
        // 0 writes the result into the ordinary UVs, 1 into UV2.
        s32 target = 0;
    };
    bool unwrapUVs(const UnwrapParams& params);

    // Throws the whole document away: mesh, skeleton, clips, selection and
    // both undo stacks. Not undoable, which is why anything unsaved asks
    // first.
    void newDocument();

    // -- Local HTTP API
    //
    // Commands arriving over HTTP are queued and run here, on the frame loop,
    // so they see and change exactly what the panels do.
    bool startApi(const std::string& host, int port, const std::string& token,
                  std::string* error = nullptr);
    void stopApi();
    bool apiRunning() const;
    int apiPort() const;
    bool apiHasToken() const;

    // -- Offscreen capture
    //
    // Renders the document the way a viewport would, into a private target, and
    // returns the pixels top row first, RGBA8. Must run on the thread that owns
    // the GL context (the frame loop).
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
    // Throws the whole document away: mesh, skeleton, clips, selection and
    // both undo stacks. Not undoable, which is why anything unsaved asks
    // first.
    void drawNewConfirmPopup();

    void drawUnwrapPopup();
    void drawBisectPopup();
    // The popup's own forms of the public operations below, fed from the
    // parameters it keeps between frames.
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
    glm::vec3 mPaintColor = glm::vec3(0.85f, 0.2f, 0.15f); // sRGB, as the picker shows it
    f32 mPaintOpacity = 1.0f;
    f32 mPaintRadius = 0.5f;
    f32 mPaintHardness = 0.5f;
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
        FileDialogExportGltf,
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
    std::unique_ptr<BlenderApi::BlenderApiHost> mApi;
    // The preferences' own copy of the port and token while they are edited.
    s32 mApiPortField = 7420;
    std::string mApiTokenField;
    std::string mApiError;

    // Current mesh
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
    std::vector<glm::mat4> mGlobalPose;
    std::vector<glm::mat4> mBonePalette;

    // State
    bool mDirty = false;
    // Set from inside the File/Windows menus, consumed (OpenPopup() + reset)
    // at the top of the matching draw*Popup() - see its own comment for why
    // OpenPopup() cannot run directly at the MenuItem call site.
    bool mSaveInfoRequested = false;
    bool mNewConfirmRequested = false;
    bool mPreferencesRequested = false;
    glm::vec3 mCursor3D = glm::vec3(0.0f);
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
    glm::vec3 mPrimitiveSize = glm::vec3(1.0f);
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
    std::vector<glm::vec3> mGizmoPositions;
    std::vector<glm::vec3> mGizmoNormals;
    std::vector<glm::vec4> mGizmoTangents;
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
