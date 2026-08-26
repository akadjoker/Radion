#ifndef RADION_ASSET_MANAGER_H
#define RADION_ASSET_MANAGER_H

#include "Containers.h"
#include "GPU.h"
#include "Hash.h"
#include "Mesh.h"
#include "MeshLoader.h"
#include "ProcTree.h"
#include "Skeleton.h"

#include <string>
#include <future>
#include <vector>

namespace Radion
{

class Pixmap;

struct MeshMergeInput
{
    const MeshData* mesh = nullptr;
    glm::mat4 transform = glm::mat4(1.0f);
    std::string sourceName;
};

struct MeshMergeOptions
{
    bool applyTransforms = true;
    bool preserveSubmeshBoundaries = false;
    u32 maxVertices = 0;
};

enum class MeshSource : u8
{
    // Uploaded straight from MeshData, with no recipe behind it. Cannot be
    // named in a saved scene; it has to be written out as a mesh asset first
    // (exportMesh/saveMesh) and referred to by that file.
    None,
    File,
    Box,
    Plane,
    Sphere,
    Cylinder,
    Cone,
    Capsule,
    Torus,
    HillsPlane,  // + heightmap image
    Heightfield, // + heightmap image
};

// What a mesh is, said as the recipe that builds it rather than as a handle.
// A MeshHandle is an index into this process's pool and means nothing to the
// next run, so this is what a saved scene stores and what the editor shows -
// and, being the whole recipe, what rebuilds the mesh on load. Whoever wants
// a mesh describes it and gets a handle back; nothing has to remember
// afterwards what that handle was.
struct MeshDesc
{
    MeshSource source = MeshSource::None;
    // File: the mesh file. HillsPlane/Heightfield: the heightmap image.
    // Empty for the rest.
    std::string file;
    // The numbers of the recipe, in the order the matching factory below
    // takes them. Fixed-size because every recipe here fits, and because a
    // desc is copied and compared often enough not to want an allocation.
    f32 params[8]{};

    static MeshDesc fromFile(const std::string& file);
    static MeshDesc box(const glm::vec3& size);
    static MeshDesc plane(f32 width, f32 depth, u32 segX, u32 segZ, f32 uvTiles);
    static MeshDesc sphere(f32 radius, u32 rings, u32 slices);
    static MeshDesc cylinder(f32 radius, f32 height, u32 slices);
    static MeshDesc cone(f32 radius, f32 height, u32 slices);
    static MeshDesc capsule(f32 radius, f32 height, u32 rings, u32 slices);
    static MeshDesc torus(f32 majorRadius, f32 minorRadius, u32 majorSegments, u32 minorSegments);
    static MeshDesc hillsPlane(f32 width, f32 depth, u32 segX, u32 segZ,
                               const std::string& heightmapFile, f32 heightScale, f32 uvTiles);
    static MeshDesc heightfield(const std::string& heightmapFile, f32 cellSize, f32 heightScale,
                                f32 uvTiles);

    bool operator==(const MeshDesc& other) const;
    bool operator!=(const MeshDesc& other) const
    {
        return !(*this == other);
    }

    // Stable one-line form, both the cache key and what a diagnostic prints:
    // "Box|1.4,1.4,1.4", "File|sponza.rmesh".
    std::string key() const;
};

 
const char* meshSourceName(MeshSource source);
bool meshSourceFromName(const std::string& name, MeshSource& out);

 
class AssetManager
{
public:
    static AssetManager& getSingleton();

    AssetManager(const AssetManager&) = delete;
    AssetManager& operator=(const AssetManager&) = delete;

    // Releases every GPU resource held here, meshes before textures before
    // samplers. Must run while the GPU context is still alive -
    // Engine::shutdown() is what calls it.
    void shutdown();

    // ---------------------------------------------------------------- meshes

    // Builds the mesh a description asks for, and hands back the same handle
    // for a description already built - so the recipe doubles as the cache
    // key and the same file or the same box is never uploaded twice. Every
    // create* below is a wrapper over this, which is why a mesh knows what it
    // is without anything having to record it afterwards. Meshes built this
    // way are shared: destroying one destroys it for everybody holding that
    // description.
    MeshHandle createMesh(const MeshDesc& desc);

    // The geometry a MeshDesc describes, without uploading it. A physics
    // collider needs the triangles the GPU mesh was built from, and building
    // them twice from separate code is how a collider ends up disagreeing
    // with what is on screen.
    bool buildMeshData(const MeshDesc& desc, MeshData& data);

    // What the mesh is, for whoever has to write it down - a saved scene, the
    // editor's asset list. Answers a None-source desc for a mesh built
    // straight from MeshData, and for an invalid or stale handle.
    const MeshDesc& meshDesc(MeshHandle handle) const;

    // Associates `handle` with `desc` in the same lookup createMesh(MeshDesc)
    // itself fills in - for a handle built through createMesh(MeshData)
    // instead (the editor's mesh-import flow, which keeps the MeshData
    // around for Mesh Tools) that still came from a real file and has to
    // survive a save the same way a plain MeshDesc::fromFile() load does.
    // Without this, SceneSerializer::writeMeshRenderer() finds no recipe for
    // the handle and refuses to save the mesh reference at all - a mesh
    // dropped in by Import and never touched again would silently vanish the
    // next time the scene reloaded. A no-op for a source of None or an
    // invalid handle.
    void registerMeshDesc(MeshHandle handle, const MeshDesc& desc);

    // Uploads geometry the caller assembled itself. There is no recipe behind
    // the result, so it cannot be named in a saved scene - prefer
    // createMesh(MeshDesc) whenever the mesh can be described, and use this
    // for one that genuinely cannot (geometry merged, unwrapped or generated
    // at runtime), knowing it has to be written out as an asset before a
    // scene can refer to it.
    MeshHandle createMesh(const MeshData& data);
    MeshHandle createDynamicMesh(const MeshData& data);

    // Creates a stable, non-rendering handle and decodes a file mesh on a
    // worker thread. The finished MeshData is uploaded by
    // processAsyncMeshLoads() on the render thread and replaces the same
    // handle, so MeshRenderers do not need to be rewired.
    MeshHandle createMeshAsync(const MeshDesc& desc);
    u32 processAsyncMeshLoads();
    u32 pendingAsyncMeshLoads() const;

    // While set, every file mesh built from here on keeps the MeshData it
    // was uploaded from, fetchable with meshData(). For a caller that needs
    // the triangles on the CPU as well as on the GPU - a collision mesh, a
    // navmesh bake - so that asking for them is not a second identical
    // import of the same file, which on a large level costs as much as the
    // first and, unlike the first, blocks whatever is drawing.
    //
    // The data is held until releaseMeshData(), so a caller that turns this
    // on owns deciding when the copy goes away.
    void setRetainFileMeshData(bool retain);
    const MeshData* meshData(MeshHandle handle) const;
    void releaseMeshData(MeshHandle handle);

    // Rebuilds the GPU buffers behind an already-issued handle from `data`,
    // in place - same handle, same object identity, the mesh-tools
    // counterpart to GPU::replaceTexture(). What an editor mesh-fixing tool
    // (regenerate normals/tangents/UVs against a MeshData it kept from the
    // original import) uses to push the edit back onto the mesh every
    // MeshRenderer already pointing at that handle is drawing - a plain
    // createMesh() would hand back a *different* handle nothing in the scene
    // knows to switch to. False (handle untouched) if `handle` is not a live
    // mesh or the new data fails to upload.
    bool replaceMesh(MeshHandle handle, const MeshData& data);

    // Decodes a mesh file (.rmesh, .obj) through the engine's own importers -
    // registered once here, not by every demo that wants to load a file.
    // `filename` still only reaches FileSystem's search paths, same as
    // loadTexture()/loadShader(). Leaves `out` untouched on failure. For
    // loading a file to draw it, createMesh(MeshDesc::fromFile()) does this
    // and the upload in one call; this is for when the MeshData itself is
    // wanted, to measure, merge or unwrap it first.
    bool importMesh(const std::string& filename, MeshData& out);

    // Same as createMesh(MeshDesc::fromFile(file)) minus the final upload -
    // decode, apply the file's same-named .material if one exists, and
    // compute tangents/bounds, but hand back the MeshData instead of a
    // handle. buildFromDesc()'s MeshSource::File case is this plus
    // createMesh(); the editor's mesh-import flow is this plus createMesh()
    // AND keeping the MeshData around for replaceMesh() to edit later - the
    // one thing a bare MeshDesc can't do, since a recipe has no room for
    // "and then someone hand-edited the normals."
    bool importMeshFileData(const std::string& file, MeshData& data);

    // The three phases importMeshFileData() runs back to back, split out so
    // the async path can put each on the thread that may run it. Only
    // importMeshGeometry() is safe off the main thread: the other two write
    // to FileSystem's search paths and to MaterialManager's material list.
    void registerMeshSearchPaths(const std::string& file);
    bool importMeshGeometry(const std::string& file, MeshData& data);
    void applyMeshFileMaterials(const std::string& file, MeshData& data);

    // An importer reports its texture paths in MeshData's parallel arrays,
    // and only loadMeshMaterialTextures() copies them onto the per-slot
    // Material::textures[].file - on the GPU Mesh, inside createMesh().
    // Writing a .material straight from MeshData::materials therefore yields
    // a sidecar with no textures in it at all: the mesh looks right in the
    // session that imported it, and comes back untextured the next time the
    // scene is opened. This returns the same materials with the paths copied
    // across, for the sidecar to be written from. It deliberately does not
    // touch `data` - loadMeshMaterialTextures() only loads a slot whose file
    // is still empty, so filling them in place would skip the upload and
    // leave the mesh untextured in the session too.
    std::vector<Material> materialsForSidecar(const MeshData& data) const;

    // The public, MeshData-only counterpart to the private loadMeshMaterialTextures(Mesh&, const
    // MeshData&) createMesh()/replaceMesh() use - for a caller with no GPU Mesh of its own to
    // resolve textures onto, such as blend's own preview renderer, which reads straight off
    // MeshData::materials. Loads Albedo/Normal/Surface/Emissive from data's own
    // materialTextureFiles/materialNormalFiles/materialSurfaceFiles/materialEmissiveFiles into
    // data.materials[i].textures[...] in place, synchronously - unlike loadMeshMaterialTextures()
    // this never touches a slot a .material sidecar already filled (its .file is non-empty), same
    // "importer never overrides a sidecar" rule.
    void loadMeshDataMaterialTextures(MeshData& data);

    // Format-dispatching skeleton/clip decode, the counterpart to importMesh()
    // for rigs - by extension today (.fbx through FbxImporter's own
    // loadFbxSkeleton/loadFbxAnimation, .rskel/.ranim through
    // RadionSkeletonIO directly), so a caller (the editor's Add Animator
    // flow) does not need to know which importer a given file belongs to,
    // same as it does not for meshes. An unsupported extension fails and
    // logs rather than silently returning an empty skeleton/clip.
    bool importSkeleton(const std::string& file, Skeleton& skeleton);
    bool importAnimation(const std::string& file, const Skeleton& skeleton, AnimationClip& clip,
                         bool keepRootMotion = true);

    // Concatenates static meshes/submeshes into one MeshData, preserving each
    // source submesh as an output group. Material slots are remapped/deduplicated
    // only for rendering; material identity never merges geometry groups.
    bool mergeMeshes(const std::vector<MeshMergeInput>& inputs, const MeshMergeOptions& options,
                     MeshData& output, std::string* error = nullptr) const;
    bool mergeSubmeshes(MeshData& mesh, bool preserveSubmeshBoundaries = true) const;

    // Groups every submesh by materialSlot regardless of where it sits in the
    // mesh, unlike mergeSubmeshes(false) which only folds submeshes together
    // when they are already consecutive and index-contiguous. Rewrites the
    // index buffer so each material's indices land in one contiguous range;
    // vertex streams are never touched. A group whose submeshes disagree on
    // lightmapPage stays split along that disagreement instead of merging.
    bool mergeSubmeshesByMaterial(MeshData& mesh) const;

    // Drops every material slot no remaining submesh points at - what
    // removeSubmesh() leaves behind is a hole in the submesh list, not in
    // the material list, so a batch of deletions (stripping a building down
    // to its floor submeshes, say) leaves the orphaned slots' materials and
    // textures sitting in the file with nothing using them. Remaining
    // submeshes' materialSlot is remapped to stay correct; the five
    // parallel per-material file arrays (materialTextureFiles and siblings)
    // are kept in lockstep with materials itself. Returns how many slots
    // were dropped - 0 means nothing was orphaned, mesh left untouched.
    //
    // `outRemap`, when given, receives old slot -> new slot for every slot
    // that survived and kInvalidMaterialSlot for every one dropped. ANYTHING
    // ELSE holding per-slot state - a MeshRenderer's own material overrides,
    // above all - is indexed the same way and silently points at the wrong
    // material after a compaction that does not put it through this table.
    static constexpr u32 kInvalidMaterialSlot = 0xFFFFFFFFu;
    u32 compactMaterials(MeshData& mesh, std::vector<u32>* outRemap = nullptr) const;

    // Throws away the indices and vertices no surviving submesh references.
    // removeSubmesh() only erases the SubMesh descriptor - deliberately, so
    // undo can put it back for nothing - which means a mesh stripped down to
    // a few pieces still carries, and still SAVES, every vertex it ever had.
    // This is what actually makes the file smaller, and it is destructive:
    // the geometry is gone afterwards, not just unreferenced. Returns the
    // number of vertices dropped.
    u32 compactGeometry(MeshData& mesh) const;

    // Registers a Mesh whose buffers the caller already built - Landscape is
    // the one caller, for chunks that share one index buffer between them
    // (see Mesh::ownsIndexBuffer) and carry their own vertex layout instead of
    // the engine's MeshAttribs. RenderList::submit() stores a MeshHandle and
    // re-fetches through getMesh() later in the frame, so a chunk needs one of
    // these the same as any other mesh - there is no path that skips the pool.
    MeshHandle adoptMesh(const Mesh& mesh);
    bool updateMeshVertices(MeshHandle handle, u32 firstVertex, u32 vertexCount,
                            const glm::vec3* positions, const MeshAttribs* attribs);

    // Convenience for a mesh that gets rebuilt on the CPU every frame (cloth,
    // any other per-vertex simulation): packs data's normal/tangent/uv/color
    // into MeshAttribs the same way upload() does for createDynamicMesh(), and
    // pushes both buffers in one call. Vertex count must match the mesh's -
    // this updates a mesh already sized at creation, it does not resize it.
    bool updateMeshVertices(MeshHandle handle, const MeshData& data);
    bool updateMeshIndices(MeshHandle handle, const u32* indices, u32 indexCount);

    // Recomputes one submesh's bounds from freshly written positions and
    // folds the change into the mesh's overall bounds. Neither
    // updateMeshVertices() overload does this on its own - it only touches
    // the GPU buffers - so a submesh that moves after upload (a deformer, an
    // animated prop) stays boxed at its creation-time extent and eventually
    // gets frustum-culled while still visibly moving, unless the caller
    // calls this too after writing new positions. Lives here rather than on
    // Mesh itself because Mesh has no CPU copy of its own vertices to
    // recompute from (see the comment on CollisionMesh) - the caller already
    // has the positions it just wrote, so it is cheapest for it to hand
    // back their box rather than this reading the GPU buffer back.
    bool updateSubMeshBounds(MeshHandle handle, u32 submeshIndex, const AABB& bounds);
    void destroyMesh(MeshHandle handle);
    void destroyAllMeshes();

    // Writes a mesh already on the GPU back out as .rmesh via GPU::readBuffer
    // - no permanent CPU copy kept just for this. Stalls the GPU, so this is
    // for an explicit export, not a per-frame call.
    bool exportMesh(MeshHandle handle, const std::string& filename,
                    const std::string& skeletonFile = std::string()) const;

    bool saveMesh(const MeshData& mesh, const std::string& filename,
                  const std::string& skeletonFile = std::string()) const;

    Mesh* getMesh(MeshHandle handle);
    const Mesh* getMesh(MeshHandle handle) const;
    usize meshCount() const;

    // -- procedural primitives

    // Axis-aligned box centred at the origin, full extents in `size`.
    MeshHandle createBox(const glm::vec3& size);

    // XZ plane at y=0, segX/segZ subdivisions - needed for per-vertex
    // displacement or per-quad lighting detail.
    MeshHandle createPlane(f32 width, f32 depth, u32 segX = 1, u32 segZ = 1, f32 uvTiles = 1.0f);

    // UV sphere centred at the origin.
    MeshHandle createSphere(f32 radius, u32 rings = 16, u32 slices = 24);

    // Y-axis cylinder, base at y=0, capped.
    MeshHandle createCylinder(f32 radius, f32 height, u32 slices = 24);

    // Y-axis cone, base at y=0, apex at y=height, capped.
    MeshHandle createCone(f32 radius, f32 height, u32 slices = 24);

    // Y-axis capsule: a cylindrical body of `height` (centre to centre of the
    // two hemisphere caps) capped by hemispheres of `radius`. Base at y=0.
    MeshHandle createCapsule(f32 radius, f32 height, u32 rings = 8, u32 slices = 24);

    // Torus centred at the origin, ring in the XZ plane, hole facing along Y.
    MeshHandle createTorus(f32 majorRadius, f32 minorRadius, u32 majorSegments = 24,
                           u32 minorSegments = 12);

    // A plane displaced by a heightmap - a quick "hills" ground without a
    // full terrain system. The image is stretched across the plane and
    // sampled bilinearly, so the mesh keeps the width/depth and segment
    // counts asked for whatever the image's resolution is, and height comes
    // off the red channel as 0..1 scaled by `heightScale`.
    MeshHandle createHillsPlane(f32 width, f32 depth, u32 segX, u32 segZ, const Pixmap& heightmap,
                                f32 heightScale, f32 uvTiles = 1.0f);

    // Same, from a heightmap file. This is the one a saved scene can name:
    // the image path and every number are recorded, so the mesh comes back
    // identical next run - an already-loaded Pixmap has no name to write.
    MeshHandle createHillsPlane(f32 width, f32 depth, u32 segX, u32 segZ,
                                const std::string& heightmapFile, f32 heightScale,
                                f32 uvTiles = 1.0f);

    // Brute-force single-mesh terrain: one vertex per pixel, so the image's
    // resolution is the mesh's and `cellSize` is the world spacing between
    // them. No LOD/paging - for small patches, props, or previews. As above,
    // the file overload is the one a scene can name.
    MeshHandle createHeightfield(const Pixmap& heightmap, f32 cellSize, f32 heightScale,
                                 f32 uvTiles = 1.0f);
    MeshHandle createHeightfield(const std::string& heightmapFile, f32 cellSize, f32 heightScale,
                                 f32 uvTiles = 1.0f);

    // Procedural tree, base of the trunk at the origin. Submesh 0 is the bark,
    // submesh 1 the twig cards, so the cards get their own alpha-tested
    // material without costing a second object. Implemented in AssetTree.cpp.
    MeshHandle createTree(const TreeParams& params);

    // The same geometry before upload, for a caller that measures it, scales
    // it to a target height, or merges it into something bigger.
    void buildTree(MeshData& out, const TreeParams& params) const;

    // Parameter sets that produce a recognisable species, and a randomiser
    // that stays inside the bands where the generator still makes trees.
    // `state` is advanced, so one seed replays one sequence.
    static u32 treePresetCount();
    static const TreePreset& treePreset(u32 index);
    static TreeParams randomTreeParams(u32& state);

    // -- bounds

    void computeBounds(MeshData& mesh) const;

    // Area-weighted vertex normals from the current positions. A mesh whose
    // vertices move every frame - cloth, a soft body, anything deformed on
    // the CPU - keeps the normals it was built with otherwise, and lights as
    // though it never moved.
    void computeNormals(MeshData& mesh) const;

    // Per-vertex tangents from the UVs, with the handedness in w. Needed by
    // any normal-mapped material: an importer that brings none leaves the
    // shader building its basis from nothing, and the lighting comes out
    // wrong in a way that looks like a bad texture rather than missing data.
    // Requires normals and UVs; does nothing without them.
    void computeTangents(MeshData& mesh) const;

    // Makes every triangle agree with its neighbours about which way it
    // faces, and returns how many had to be flipped.
    //
    // Two triangles sharing an edge should traverse it in opposite
    // directions. Real assets often have a few that do not, and a backwards
    // triangle is invisible to a one-sided collision sweep: a character walks
    // INTO the wall, and only meets the far side once deeply inside, where
    // the push-out ejects him hard. It also lights inside out.
    //
    // Orientation is spread from one triangle per connected piece, so the
    // result is self-consistent but its overall sense is whatever that first
    // triangle had. Call flipWinding() afterwards if a piece comes out inside
    // out.
    u32 fixWinding(MeshData& mesh) const;
    void computeSubMeshBounds(MeshData& mesh) const;

    // Breaks every oversized submesh (import merged many world triangles
    // into one draw range, e.g. all walls sharing a material) into several
    // spatially local ones, so a BVH/frustum/occlusion test built per
    // submesh has boxes worth rejecting instead of one box the size of the
    // whole mesh. Reorders the indices inside each submesh's own range by a
    // uniform grid over its triangle centroids - the shared position array
    // and the index buffer's size never change, only which SubMesh entries
    // point at which slice of it. Skinned meshes (mesh.skin non-empty) are
    // left untouched: their vertices are in bind pose, so a box computed
    // here would describe a pose the mesh may never actually be in.
    void splitSubMeshes(MeshData& mesh, u32 targetTriangles = 4000) const;

    // -- selection editing

    // Drops the given triangles from the index buffer and every submesh
    // range, then compacts the vertex streams to whatever is still
    // referenced (meshopt_optimizeVertexFetchRemap, the same compaction
    // optimizeVertexFetch() uses). A submesh left with no triangles is
    // dropped entirely.
    void deleteFaces(MeshData& mesh, const std::vector<u32>& faceIndices) const;

    // Deletes every triangle that touches any of the given vertices, then
    // compacts the same way deleteFaces() does.
    void deleteVertices(MeshData& mesh, const std::vector<u32>& vertexIndices) const;

    // Pushes a region of faces out along its own normals by `distance`,
    // walling in the gap it leaves behind. The selected faces are replaced by
    // the raised copy - they do not stay behind as interior geometry - and
    // side faces are built only on the region's boundary edges, the ones used
    // by a single selected face: an edge shared by two of them is inside the
    // region and gets no wall. A vertex shared by faces at an angle travels
    // along their summed normal, so the region lifts as one piece.
    //
    // Each new vertex copies the attributes of the one it came from, and
    // every side face lands in the submesh of the face that raised it, so
    // materials survive. Normals are left alone: the raised copy keeps the
    // originals, which is right for the cap and wrong for the walls, and only
    // the caller knows whether the mesh wants smooth or flat shading -
    // follow this with recalculateNormals().
    //
    // `extrudedFaces`, when given, comes back holding the indices of the
    // raised faces, which is what makes a second extrude on the same
    // selection possible: the rebuilt index buffer renumbers everything.
    bool extrudeFaces(MeshData& mesh, const std::vector<u32>& faceIndices, f32 distance,
                      std::vector<u32>* extrudedFaces = nullptr) const;

    // -- diagnostics

    // What a mesh is made of, and what is wrong with it. A tool asks for this
    // to tell a clean import from one that will misbehave later: a degenerate
    // triangle contributes a zero normal that poisons its vertices' smoothed
    // ones, a non-manifold edge breaks anything walking the surface, and
    // missing UVs or tangents mean a normal-mapped material has nothing to
    // build its basis from. Reading only - it diagnoses, it does not repair.
    struct Diagnostics
    {
        usize vertexCount = 0;
        usize triangleCount = 0;
        usize submeshCount = 0;
        usize materialCount = 0;
        usize memoryBytes = 0;
        AABB bounds;

        bool hasNormals = false;
        bool hasTangents = false;
        bool hasUvs = false;
        bool hasUvs2 = false;
        bool hasColors = false;
        bool hasSkin = false;
        // An optional stream that is present but not one entry per vertex,
        // which every consumer reading it in parallel will run off the end of.
        bool streamsMismatched = false;

        // Indices naming a vertex the mesh does not have.
        u32 outOfRangeIndices = 0;
        // Triangles naming the same vertex twice, or with no area.
        u32 degenerateTriangles = 0;
        // Vertices no triangle refers to - dead weight carried through every
        // later pass, and what optimizeVertexFetch() exists to drop.
        u32 orphanVertices = 0;
        // Edges used by exactly one triangle. Expected on an open surface,
        // and a hole in something meant to be closed.
        u32 boundaryEdges = 0;
        // Edges used by three or more. Nothing downstream handles these.
        u32 nonManifoldEdges = 0;
        // Vertices sharing a position exactly, which is what weldVertices()
        // collapses. A lower bound: it does not look for near matches.
        u32 exactDuplicatePositions = 0;
        // Indices not a multiple of three, or a submesh range reaching past
        // the index buffer.
        bool trianglesTruncated = false;
        bool submeshRangesInvalid = false;
    };

    void analyzeMesh(const MeshData& mesh, Diagnostics& out) const;

    // -- selection queries
    //
    // None of these change the mesh; they answer "which elements" so a tool
    // can drive its own selection with them. Each writes an ascending,
    // duplicate-free list into `out`, replacing whatever was there.

    // The given vertices plus everyone an edge away - one ring wider.
    void growVertexSelection(const MeshData& mesh, const std::vector<u32>& vertexIndices,
                             std::vector<u32>& out) const;
    // The given vertices minus any that touch one left out, which peels a
    // ring off the border rather than off the whole set.
    void shrinkVertexSelection(const MeshData& mesh, const std::vector<u32>& vertexIndices,
                               std::vector<u32>& out) const;
    // The given faces plus every face sharing a vertex with one of them.
    void growFaceSelection(const MeshData& mesh, const std::vector<u32>& faceIndices,
                           std::vector<u32>& out) const;

    // Everything reachable from the seeds through shared vertices, however
    // far - the connected piece they sit on. Picking one vertex of a chair in
    // a room mesh and getting the whole chair.
    void selectLinkedVertices(const MeshData& mesh, const std::vector<u32>& seedVertices,
                              std::vector<u32>& out) const;
    void selectLinkedFaces(const MeshData& mesh, const std::vector<u32>& seedFaces,
                           std::vector<u32>& out) const;

    // Every face in one submesh - selecting by material without hunting for
    // it in the viewport.
    void submeshFaces(const MeshData& mesh, u32 submeshIndex, std::vector<u32>& out) const;

    // Pulls the given triangles out of whichever submesh range they sit in
    // and appends them as one new trailing submesh, inheriting the
    // materialSlot/lightmapPage of the first selected triangle's original
    // submesh. Vertex streams are untouched - only which SubMesh entry a
    // triangle belongs to changes. Returns false if no triangle was moved.
    bool groupFacesIntoSubmesh(MeshData& mesh, const std::vector<u32>& faceIndices) const;

    // -- normals and tangents

    // smooth false writes the face normal into all three vertices, which is
    // only right when the vertices are not shared: a vertex used by two faces
    // keeps whichever face came last. Split the mesh first for hard edges.
    void recalculateNormals(MeshData& mesh, bool smooth, bool angleWeighted = false) const;
    void recalculateTangents(MeshData& mesh) const;

    // -- uv

    // Picks, per triangle, the axis its normal points along most, and uses the
    // other two coordinates as uv.
    void makePlanarUV(MeshData& mesh, f32 resolution) const;

    // Fixed axis: 0 = X, 1 = Y, 2 = Z.
    void makePlanarUV(MeshData& mesh, f32 resolutionS, f32 resolutionT, u8 axis,
                      const glm::vec3& offset) const;

    // Wraps u once around the Y axis (atan2(x,z)/2pi) and runs v linearly
    // over the mesh's own Y extent. `resolutionU`/`resolutionV` are tile
    // counts, same convention as makePlanarUV's `resolution`. The seam where
    // u wraps from 1 back to 0 needs the same vertex-duplication makePlanarUV
    // uses at a hard edge - one shared vertex cannot hold both "just under 1"
    // and "just over 0" at once, and a naive per-vertex write leaves a
    // triangle at the seam with one corner's u torn across the whole texture.
    void makeCylindricalUV(MeshData& mesh, f32 resolutionU, f32 resolutionV) const;

    // Same seam as makeCylindricalUV along the sphere's own "date line", plus
    // the two poles: every triangle touching one fans around it with a
    // different u per triangle by construction (there is no single sensible
    // longitude for a point sitting exactly on the axis), so a pole vertex is
    // never shared across triangles here - each gets its own duplicate.
    void makeSphericalUV(MeshData& mesh, f32 resolutionU, f32 resolutionV) const;

    // Reworks the UVs already on a set of faces instead of regenerating the
    // whole mesh's: scale first, then rotate, then offset, all around the
    // centre of those faces' own UV bounds - scaling around the origin
    // instead would slide the texture off toward (0,0) by however far the
    // island sits from it.
    //
    // A vertex shared with a face left out is duplicated first, so retiling
    // one wall does not drag the texturing of everything joined to it along.
    // That splits the mesh where the selection ends, which is what a UV seam
    // is; run weldVertices() afterwards to undo it if the edit is reverted.
    //
    // An empty `faceIndices` takes the whole mesh, where nothing is shared
    // with anything left out and no vertex is split. False when the mesh
    // carries no UVs to rework - makePlanarUV() and friends make them.
    bool transformFaceUVs(MeshData& mesh, const std::vector<u32>& faceIndices,
                          const glm::vec2& scale, f32 rotationDegrees,
                          const glm::vec2& offset) const;

    // -- transforms

    void translate(MeshData& mesh, const glm::vec3& delta) const;
    void scale(MeshData& mesh, const glm::vec3& factor) const;

    // Positions go through the matrix, normals and tangents through its
    // inverse transpose, otherwise a non-uniform scale tilts them off the
    // surface.
    void transform(MeshData& mesh, const glm::mat4& matrix) const;

    // The same, restricted to the given vertices and applied around their own
    // median point instead of the origin. Scaling a selection about the
    // origin sends it away from the rest of the model by however far the
    // model sits from it; about the median it grows where it stands, which is
    // what an edit in place means. An empty `vertexIndices` transforms the
    // whole mesh, still about its own median.
    void transformVertices(MeshData& mesh, const glm::mat4& matrix,
                           const std::vector<u32>& vertexIndices = {}) const;

    // The same with the pivot named outright, for a caller whose matrix
    // already carries its own placement - a gizmo's, whose transform sits at
    // the pivot to begin with, and which the median version would place a
    // second time. Pass a zero pivot to apply `matrix` in world space.
    void transformVerticesAbout(MeshData& mesh, const glm::mat4& matrix, const glm::vec3& pivot,
                                const std::vector<u32>& vertexIndices = {}) const;

    // Moves the mesh so the centre of its box sits on the origin.
    void center(MeshData& mesh) const;

    // Centres on X and Z but drops the lowest point to y = 0, which is what a
    // character or a prop that stands on the floor wants.
    void centerOnGround(MeshData& mesh) const;

    // Reverses triangle winding. Physics back ends often want the opposite of
    // the graphics one, and a mirrored transform turns the mesh inside out.
    void flipWinding(MeshData& mesh) const;

    // Same, restricted to one submesh's own index range - turning just one
    // face of a room inside out (to see it from outside while building it,
    // say) instead of the whole mesh.
    void flipWinding(MeshData& mesh, u32 submeshIndex) const;

    // -- decomposition

    // Copies one submesh out as a mesh in its own right: only the vertices
    // its indices actually reference (a sparse subset of source's shared
    // buffers), re-indexed to a compact 0..n-1 range, every parallel
    // attribute array carried across, and its material (and name, if any)
    // copied so the result stands alone. Positions stay in source's own
    // space - nothing here knows about a GameObject transform, so the
    // caller places it. For breaking a door off a level mesh to swing it, a
    // prop to give it its own physics, or a curtain to give it cloth.
    bool extractSubmesh(const MeshData& source, u32 submeshIndex, MeshData& out) const;

    // Drops a submesh from the mesh it belongs to - just erases the SubMesh
    // entry. The other submeshes index into the same shared vertex/index
    // buffers and their indexOffset values do not move, so nothing else
    // needs fixing up; the orphaned vertices and indices stay in the
    // buffers, wasting a little memory. Compacting them would mean
    // rewriting every remaining submesh's offsets for a mesh that is about
    // to be uploaded once and (usually) never touched again - not worth it.
    bool removeSubmesh(MeshData& mesh, u32 submeshIndex) const;

    // -- optimization

    // Merges vertices whose attributes are bit-identical across every active
    // stream and drops the duplicates, rewriting the index buffer. Importers
    // that emit one vertex per face corner (OBJ, 3DS) leave meshes several
    // times larger than needed; this is the cleanup. Returns how many
    // vertices were removed.
    u32 weldVertices(MeshData& mesh) const;

    // Same merge, but by proximity instead of bit-identical attributes: any
    // two vertices no farther apart than `distance` are candidates, found
    // through a uniform grid of `distance`-sized cells rather than an
    // all-pairs test. A merged vertex's position becomes the average of its
    // group; triangles left degenerate by the merge are dropped and the
    // vertex streams compacted the same way deleteFaces() does. An empty
    // `vertexIndices` welds the whole mesh; otherwise only vertices in that
    // list are ever candidates, so unselected geometry never moves. Returns
    // how many vertices were removed.
    u32 weldVertices(MeshData& mesh, f32 distance, const std::vector<u32>& vertexIndices = {}) const;

    // Laplacian smoothing: each pass moves every eligible vertex `strength`
    // of the way toward the average of its edge-adjacent neighbors (topology
    // built once from the index buffer, not the selection). An empty
    // `vertexIndices` smooths the whole mesh.
    void smoothVertices(MeshData& mesh, f32 strength, u32 iterations,
                        const std::vector<u32>& vertexIndices = {}) const;

    // Reorders each submesh's triangles so successive triangles reuse
    // recently transformed vertices. Purely an index reorder: vertex streams
    // and submesh ranges never change.
    void optimizeVertexCache(MeshData& mesh) const;

    // Reorders each submesh's triangles to draw front-to-back within local
    // clusters, cutting overdraw on opaque geometry. Run after
    // optimizeVertexCache; `threshold` is how much cache efficiency it is
    // allowed to give back (1.05 = 5%).
    void optimizeOverdraw(MeshData& mesh, f32 threshold = 1.05f) const;

    // Reorders the vertex streams into the order the index buffer first
    // references them, so vertex memory is read sequentially. Also drops
    // vertices nothing references. Run last, after the index-reordering
    // passes above.
    void optimizeVertexFetch(MeshData& mesh) const;

    // Collapses edges until each submesh is down to about targetRatio of its
    // triangles, or the geometric deviation reaches targetError (fraction of
    // the mesh extent). Appearance-preserving: submesh borders and material
    // seams hold. Unreferenced vertices are left behind - run
    // optimizeVertexFetch afterwards to compact. resultError, when given,
    // receives the worst submesh's deviation actually reached.
    bool simplifyMesh(MeshData& mesh, f32 targetRatio, f32 targetError = 0.01f,
                      f32* resultError = nullptr) const;

    // -- collision

    void buildCollisionMesh(const MeshData& mesh, CollisionMesh& out) const;

    // Closest triangle hit, in mesh space. Returns false when nothing is hit.
    bool raycast(const CollisionMesh& mesh, const Ray& ray, f32& t, u32& triangle) const;

    // -------------------------------------------------------------- textures

    // Reads `filename` through FileSystem and uploads it, or - if the same
    // filename was already loaded in the same space - returns that handle
    // without touching disk or the GPU again. On failure (missing file,
    // unreadable format) logs an error and returns getDefaultTexture() instead
    // of an invalid handle, so a broken material path still draws something
    // recognisably wrong rather than nothing.
    //
    // `space` has no default on purpose: it is not a preference, and a wrong
    // one is invisible at the call site. When the texture is going into a
    // material slot, pass Material::colorSpaceFor(slot) rather than choosing.
    //
    // `mipLimit` caps the chain at that many levels, 0 meaning all of them.
    // An atlas needs it: past a few levels the box filter starts averaging
    // texels of neighbouring regions together and the distant result is a
    // blend of images that are not adjacent in the world at all.
    TextureHandle loadTexture(const std::string& filename, ColorSpace space,
                              bool generateMips = true, u32 mipLimit = 0);

    // Same cache/params as loadTexture(), but never blocks: hands back a
    // small solid placeholder immediately (registered under the same cache
    // key, so a second call for the same path reuses it instead of queuing a
    // duplicate decode) and queues the real decode+upload on
    // AsyncTextureLoader. The handle stays valid the whole time - Engine::
    // update() (or whoever owns the frame loop) must call
    // processAsyncTextureLoads() once a frame for the placeholder to ever
    // become the real image; nothing else drives that pump.
    TextureHandle loadTextureAsync(const std::string& filename, ColorSpace space,
                                   bool generateMips = true, u32 mipLimit = 0);

    // Uploads whatever AsyncTextureLoader finished decoding since the last
    // call - main thread only, the only one allowed to touch the GPU. A
    // no-op, cheap to call every frame regardless of whether anything is
    // actually loading.
    void processAsyncTextureLoads();

    // loadTexture() caches by filename+params, so calling it again after the
    // file on disk changed (a lightmap re-baked over the same path, say)
    // just hands back the untouched old handle. This drops that cache entry
    // first when one exists, so the reload actually re-reads the file and
    // re-uploads it - same handle-eviction path destroyTexture() uses, just
    // followed immediately by a fresh load instead of leaving it gone.
    TextureHandle reloadTexture(const std::string& filename, ColorSpace space,
                                bool generateMips = true, u32 mipLimit = 0);

    // Six explicit face paths, already in GL cube-face order (+X -X +Y -Y +Z
    // -Z). Uploads all six into one TexCube in a single call - see
    // AssetTexture.cpp for why they have to land in one contiguous buffer.
    // On any failure (a missing/unreadable face, mismatched or non-square
    // dimensions) logs the offending file and returns an invalid handle - a
    // broken sky must be visibly absent, not the checker texture, so the
    // caller can fall back to a procedural sky instead.
    TextureHandle loadCubemap(const std::string faces[6], const std::string& cacheName,
                              ColorSpace space, bool generateMips = true);

    // Resolves one of the known suffix conventions (_RT/_LF/.., _px/_nx/..,
    // SkyBox_Right/_Left/..) off `baseName`, trying each in turn and using
    // the first whose six files all exist. `baseName` may itself be a
    // directory (e.g. "skys/montains" -> "skys/montains/SkyBox_Right.jpg").
    TextureHandle loadCubemap(const std::string& baseName, ColorSpace space,
                              bool generateMips = true);

    // Every cubemap under `directory` (itself relative to a search path), as
    // base names ready to hand back to loadCubemap(). A complete set of six
    // faces counts once, whichever convention names it; anything short of
    // six is skipped rather than half-listed. For populating a picker - it
    // touches the filesystem, so call it once and keep the result.
    std::vector<std::string> listCubemaps(const std::string& directory) const;

    // Registers any GPU-created texture under `name`, so it participates in
    // the same by-name/by-index lookup as a loaded file (a runtime-generated
    // noise texture, e.g.). Returns the existing handle if `name` is already
    // registered - desc is ignored in that case.
    TextureHandle createTexture(const std::string& name, const TextureDesc& desc);

    // Small procedural magenta/black checkerboard, created once on first use
    // and cached under a reserved name. What loadTexture() falls back to when
    // a file is missing - loud on purpose, so a broken texture path is obvious
    // in the render instead of silently sampling black or white.
    TextureHandle getDefaultTexture();

    TextureHandle getTexture(const std::string& name) const;

    // Position in load/create order, 0-based. Stable across destroyTexture()
    // (the slot is tombstoned, not swapped) - check the returned handle, it is
    // invalid past the live range or on a destroyed slot.
    TextureHandle getTextureByIndex(usize index) const;
    usize textureCount() const;

    const std::string& textureName(TextureHandle handle) const;

    void destroyTexture(TextureHandle handle);
    void destroyAllTextures();

    // Samplers describe filtering/wrap, not pixel data - two textures asking
    // for the same Filter::Linear + Wrap::Repeat share one GL sampler object
    // instead of each creating their own. The cache is a linear scan on
    // purpose: a handful of distinct samplers exist in the whole engine, dedup
    // here is not a hot path. Shared, so there is no per-sampler remove - only
    // destroyAllSamplers(), which destroyAllTextures() already calls.
    SamplerHandle getSampler(const SamplerDesc& desc);
    void destroyAllSamplers();

    // ---------------------------------------------------------- named targets

    // What lets a technique that renders into a texture (ReflectionPass, a
    // depth prepass) and a material that reads it (MaterialTexture::source ==
    // TextureSource::RenderTarget) never know about each other. A technique
    // publishes its output under a name every frame it wants it visible;
    // ForwardPass (or whoever draws) resolves by the same name's hash. Not
    // ownership - the publishing technique still destroys its own texture.
    void publishRenderTarget(u32 nameHash, TextureHandle texture);
    void publishRenderTarget(const char* name, TextureHandle texture);
    TextureHandle resolveRenderTarget(u32 nameHash) const;

    // --------------------------------------------------------------- shaders

    // Reads `filename`, expanding any line of the form
    // `#include "other.glsl"` with that file's own (recursively expanded)
    // text - GLSL has no preprocessor #include of its own, so shared chunks (a
    // lighting function, a uniform block) need this to be split into their own
    // files at all. Cached by filename; editing the file on disk needs
    // reloadShader() to be picked up, a plain second loadShader() will not.
    const std::string& loadShader(const std::string& filename);

    // Drops filename's cached text, so the next loadShader() re-reads it (and
    // everything it #includes) from disk. What a shader hot-reload command
    // calls.
    void reloadShader(const std::string& filename);
    void reloadAllShaders();

private:
    AssetManager();

    bool upload(const MeshData& data, Mesh& out, Residency residency) const;
    void release(Mesh& mesh) const;
    // Shared by createMesh(MeshData)/replaceMesh(): the albedo/normal file
    // lists a MeshData carries alongside its materials only get resolved to
    // TextureHandles here, once, rather than duplicated at each call site.
    void loadMeshMaterialTextures(Mesh& mesh, const MeshData& data);

    // Appends a copy of vertex `source`'s attributes (position/normal/color/
    // skin) at a new index and returns it - what every UV generator here
    // uses to split a vertex at a seam or hard edge, since a shared vertex
    // can only ever hold one UV.
    u32 duplicateMeshVertex(MeshData& mesh, u32 source) const;

    // Files an already-created GPU texture into the cache/lookup tables
    // loadTexture()/loadTextureAsync() both share, under `cacheKey`. Reused
    // by the async path for its placeholder handle too, since it belongs
    // under the same key from the caller's very first frame - a second
    // loadTextureAsync() for the same path before the first one finishes
    // has to find this and hand back the same pending handle, not start a
    // duplicate decode job.
    TextureHandle registerTextureEntry(const std::string& filename, const std::string& cacheKey,
                                       TextureHandle handle);

    std::string expandShader(const std::string& filename, int depth);

    struct TextureEntry
    {
        std::string name;
        std::string cacheKey;
        TextureHandle handle;
    };

    struct SamplerEntry
    {
        SamplerDesc desc;
        SamplerHandle handle;
    };

    // Builds the geometry for a description not in the cache yet - the one
    // place that knows how each MeshSource is made.
    MeshHandle buildFromDesc(const MeshDesc& desc);

    Pool<Mesh, MeshHandle> mMeshes;
    MeshLoader mMeshLoader;
    // Both directions of the same fact: what a handle is, and whether a
    // description has already been built.
    HashMap<u64, MeshDesc> mMeshDescs;           // packHandle() -> description
    HashMap<std::string, MeshHandle> mMeshByKey; // MeshDesc::key() -> handle

    // Queued file meshes, and the single import in flight. One at a time on
    // purpose: the decode itself is all a worker may safely do, because
    // everything around it - registering the mesh's directory as a search
    // path, replacing the material list from the .material sidecar - writes
    // to singletons the worker's own file reads are walking. Serialising the
    // imports lets that work happen on the main thread with no worker
    // running, which is the only arrangement that is race-free without
    // rewriting FileSystem's and MaterialManager's interfaces. Parsing two
    // large meshes at once buys little anyway; both are bound by the same
    // disk.
    struct PendingMesh
    {
        MeshHandle handle;
        std::string file;
        std::future<MeshData> result;
    };
    std::vector<PendingMesh> mQueuedMeshes; // waiting for their turn
    PendingMesh mMeshInFlight;              // handle.valid() while one runs

    bool mRetainFileMeshData = false;
    HashMap<u64, MeshData> mRetainedMeshData; // packHandle() -> source data

    std::vector<TextureEntry> mTextures;
    HashMap<std::string, usize> mTexturesByName;
    HashMap<std::string, usize> mLoadedTextures;
    HashMap<u64, usize> mTexturesByHandle;
    TextureHandle mDefaultTexture;
    std::vector<SamplerEntry> mSamplers;

    HashMap<std::string, std::string> mShaderSources;

    HashMap<u32, TextureHandle> mNamedTargets;
};

// Shorthand for AssetManager::getSingleton(), which is otherwise most of the
// line at every call site.
AssetManager& Assets();

} // namespace Radion

#endif // RADION_ASSET_MANAGER_H
