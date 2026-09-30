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
    Math::mat4 transform = Math::mat4(1.0f);
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
    // Uploaded straight from MeshData, no recipe; must be exported as a mesh asset before a scene can name it.
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

// Recipe that builds a mesh. Saved scenes store this because a MeshHandle is only valid within one process.
struct MeshDesc
{
    MeshSource source = MeshSource::None;
    // File: the mesh file. HillsPlane/Heightfield: the heightmap image.
    std::string file;
    // Recipe numbers, in the order the matching factory takes them.
    f32 params[8]{};

    static MeshDesc fromFile(const std::string& file);
    static MeshDesc box(const Math::vec3& size);
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

    // Stable one-line form used as cache key: "Box|1.4,1.4,1.4", "File|sponza.rmesh".
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

    // Must run while the GPU context is alive; releases meshes, then textures, then samplers.
    void shutdown();

    // Returns the existing handle for an already-built desc (the desc is the cache key).
    // Meshes are shared: destroying one destroys it for every holder.
    MeshHandle createMesh(const MeshDesc& desc);

    // Geometry a desc describes without uploading, so colliders use the same triangles as the GPU mesh.
    bool buildMeshData(const MeshDesc& desc, MeshData& data);

    // Answers a None-source desc for MeshData-built meshes and for invalid or stale handles.
    const MeshDesc& meshDesc(MeshHandle handle) const;

    // Registers a recipe for a handle built via createMesh(MeshData), so SceneSerializer can save the reference.
    // No-op for a None source or invalid handle.
    void registerMeshDesc(MeshHandle handle, const MeshDesc& desc);

    // No recipe behind the result, so it cannot be named in a saved scene; prefer createMesh(MeshDesc).
    MeshHandle createMesh(const MeshData& data);
    MeshHandle createDynamicMesh(const MeshData& data);
    // Voxel chunk mesh: float3 positions as usual plus one packed u32 per vertex from MeshData::colors; voxel.vert unpacks it.
    MeshHandle createVoxelMesh(const MeshData& data, const Material* materials, u32 count);
    bool replaceVoxelMesh(MeshHandle handle, const MeshData& data, const Material* materials,
                          u32 count);
    bool uploadVoxel(const MeshData& data, const Material* materials, u32 count, Mesh& out) const;

    // Creates a stable handle and decodes on a worker thread; processAsyncMeshLoads() uploads on the render thread and replaces the same handle.
    MeshHandle createMeshAsync(const MeshDesc& desc);
    u32 processAsyncMeshLoads();
    u32 pendingAsyncMeshLoads() const;

    // While set, file meshes keep the MeshData they were uploaded from (see meshData()), so CPU triangles need no second import.
    // Held until releaseMeshData().
    void setRetainFileMeshData(bool retain);
    const MeshData* meshData(MeshHandle handle) const;
    void releaseMeshData(MeshHandle handle);

    // Rebuilds the GPU buffers behind an existing handle in place. False (handle untouched) if not a live mesh or the upload fails.
    bool replaceMesh(MeshHandle handle, const MeshData& data);

    // Decodes a mesh file (.rmesh, .obj) through the engine importers. Leaves `out` untouched on failure.
    bool importMesh(const std::string& filename, MeshData& out);

    // createMesh(MeshDesc::fromFile(file)) minus the upload: decode, apply the same-named .material, compute tangents/bounds.
    bool importMeshFileData(const std::string& file, MeshData& data);

    // Only importMeshGeometry() is safe off the main thread; the others write FileSystem search paths and MaterialManager's list.
    void registerMeshSearchPaths(const std::string& file);
    bool importMeshGeometry(const std::string& file, MeshData& data);
    void applyMeshFileMaterials(const std::string& file, MeshData& data);

    // Materials with texture paths copied from MeshData's parallel arrays, for writing a sidecar.
    // Does not modify `data`: loadMeshMaterialTextures() only loads slots whose file is empty.
    std::vector<Material> materialsForSidecar(const MeshData& data) const;

    // MeshData-only counterpart of the private loadMeshMaterialTextures(); loads in place, synchronously.
    // Never overrides a slot a .material sidecar already filled.
    void loadMeshDataMaterialTextures(MeshData& data);

    // Format-dispatching skeleton/clip decode (.fbx, .rskel/.ranim). An unsupported extension fails and logs.
    bool importSkeleton(const std::string& file, Skeleton& skeleton);
    bool importAnimation(const std::string& file, const Skeleton& skeleton, AnimationClip& clip,
                         bool keepRootMotion = true);

    // Concatenates meshes into one MeshData, keeping each source submesh as an output group.
    bool mergeMeshes(const std::vector<MeshMergeInput>& inputs, const MeshMergeOptions& options,
                     MeshData& output, std::string* error = nullptr) const;
    bool mergeSubmeshes(MeshData& mesh, bool preserveSubmeshBoundaries = true) const;

    // Groups submeshes by materialSlot even when not consecutive; rewrites the index buffer, vertex streams untouched.
    bool mergeSubmeshesByMaterial(MeshData& mesh) const;

    // Drops material slots no submesh uses and remaps materialSlot; returns how many were dropped.
    // `outRemap` gets old slot -> new slot (kInvalidMaterialSlot if dropped); any per-slot state elsewhere must go through it.
    static constexpr u32 kInvalidMaterialSlot = 0xFFFFFFFFu;
    u32 compactMaterials(MeshData& mesh, std::vector<u32>* outRemap = nullptr) const;

    // Drops indices/vertices no surviving submesh references. Destructive. Returns vertices dropped.
    u32 compactGeometry(MeshData& mesh) const;

    // Registers a Mesh whose buffers the caller built (Landscape chunks sharing an index buffer); RenderList re-fetches via getMesh().
    MeshHandle adoptMesh(const Mesh& mesh);
    bool updateMeshVertices(MeshHandle handle, u32 firstVertex, u32 vertexCount,
                            const Math::vec3* positions, const MeshAttribs* attribs);

    // For CPU-rebuilt meshes (cloth): packs data like upload() and pushes both buffers. Vertex count must match the mesh's.
    bool updateMeshVertices(MeshHandle handle, const MeshData& data);
    bool updateMeshIndices(MeshHandle handle, const u32* indices, u32 indexCount);

    // Recomputes one submesh's bounds and folds them into the mesh bounds. updateMeshVertices() does not, so call this after moving vertices or culling goes stale.
    bool updateSubMeshBounds(MeshHandle handle, u32 submeshIndex, const AABB& bounds);
    void destroyMesh(MeshHandle handle);
    void destroyAllMeshes();

    // Reads back via GPU::readBuffer; stalls the GPU, so for explicit export only.
    bool exportMesh(MeshHandle handle, const std::string& filename,
                    const std::string& skeletonFile = std::string()) const;

    bool saveMesh(const MeshData& mesh, const std::string& filename,
                  const std::string& skeletonFile = std::string()) const;

    Mesh* getMesh(MeshHandle handle);
    const Mesh* getMesh(MeshHandle handle) const;
    usize meshCount() const;

    // Axis-aligned box centred at the origin, full extents in `size`.
    MeshHandle createBox(const Math::vec3& size);

    // XZ plane at y=0.
    MeshHandle createPlane(f32 width, f32 depth, u32 segX = 1, u32 segZ = 1, f32 uvTiles = 1.0f);

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

    // Plane displaced by the heightmap's red channel (0..1 * heightScale), sampled bilinearly.
    MeshHandle createHillsPlane(f32 width, f32 depth, u32 segX, u32 segZ, const Pixmap& heightmap,
                                f32 heightScale, f32 uvTiles = 1.0f);

    // Same from a file; this is the one a saved scene can name.
    MeshHandle createHillsPlane(f32 width, f32 depth, u32 segX, u32 segZ,
                                const std::string& heightmapFile, f32 heightScale,
                                f32 uvTiles = 1.0f);

    // One vertex per pixel, `cellSize` world spacing. No LOD/paging; for small patches.
    MeshHandle createHeightfield(const Pixmap& heightmap, f32 cellSize, f32 heightScale,
                                 f32 uvTiles = 1.0f);
    MeshHandle createHeightfield(const std::string& heightmapFile, f32 cellSize, f32 heightScale,
                                 f32 uvTiles = 1.0f);

    // Submesh 0 is the bark, submesh 1 the alpha-tested twig cards. Implemented in AssetTree.cpp.
    MeshHandle createTree(const TreeParams& params);

    void buildTree(MeshData& out, const TreeParams& params) const;

    // randomTreeParams stays inside the bands where the generator makes trees; `state` is advanced.
    static u32 treePresetCount();
    static const TreePreset& treePreset(u32 index);
    static TreeParams randomTreeParams(u32& state);

    void computeBounds(MeshData& mesh) const;

    // Area-weighted vertex normals; needed for CPU-deformed meshes.
    void computeNormals(MeshData& mesh) const;

    // Per-vertex tangents from the UVs, handedness in w. Requires normals and UVs; does nothing without them.
    void computeTangents(MeshData& mesh) const;

    // Makes triangle winding consistent across shared edges; returns how many were flipped.
    // Orientation spreads from one triangle per connected piece; call flipWinding() if a piece ends up inside out.
    u32 fixWinding(MeshData& mesh) const;
    void computeSubMeshBounds(MeshData& mesh) const;

    // Splits oversized submeshes into spatially local ones (grid over centroids) so culling boxes are useful.
    // Skinned meshes are left untouched (bind pose).
    void splitSubMeshes(MeshData& mesh, u32 targetTriangles = 4000) const;

    // Drops triangles from the index buffer and submesh ranges, then compacts vertex streams. Emptied submeshes are dropped.
    void deleteFaces(MeshData& mesh, const std::vector<u32>& faceIndices) const;

    // Deletes every triangle touching the vertices, then compacts like deleteFaces().
    void deleteVertices(MeshData& mesh, const std::vector<u32>& vertexIndices) const;

    // Pushes faces out along their normals by `distance`, walling the boundary edges. Normals are left alone; follow with recalculateNormals().
    // `extrudedFaces` receives the raised faces' indices (the index buffer is renumbered).
    bool extrudeFaces(MeshData& mesh, const std::vector<u32>& faceIndices, f32 distance,
                      std::vector<u32>* extrudedFaces = nullptr) const;

    // Reports degenerate triangles, non-manifold edges, missing UVs/tangents etc. Read-only.
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
        // An optional stream present but not one entry per vertex.
        bool streamsMismatched = false;

        u32 outOfRangeIndices = 0;
        // Triangles naming the same vertex twice, or with no area.
        u32 degenerateTriangles = 0;
        u32 orphanVertices = 0;
        // Edges used by exactly one triangle.
        u32 boundaryEdges = 0;
        u32 nonManifoldEdges = 0;
        // Vertices sharing a position exactly; a lower bound (no near matches).
        u32 exactDuplicatePositions = 0;
        // Indices not a multiple of three, or a submesh range reaching past the index buffer.
        bool trianglesTruncated = false;
        bool submeshRangesInvalid = false;
    };

    void analyzeMesh(const MeshData& mesh, Diagnostics& out) const;

    // Selection queries never change the mesh; each writes an ascending, duplicate-free list into `out`.
    void growVertexSelection(const MeshData& mesh, const std::vector<u32>& vertexIndices,
                             std::vector<u32>& out) const;
    void shrinkVertexSelection(const MeshData& mesh, const std::vector<u32>& vertexIndices,
                               std::vector<u32>& out) const;
    void growFaceSelection(const MeshData& mesh, const std::vector<u32>& faceIndices,
                           std::vector<u32>& out) const;

    // Everything reachable from the seeds through shared vertices (the connected piece).
    void selectLinkedVertices(const MeshData& mesh, const std::vector<u32>& seedVertices,
                              std::vector<u32>& out) const;
    void selectLinkedFaces(const MeshData& mesh, const std::vector<u32>& seedFaces,
                           std::vector<u32>& out) const;

    void submeshFaces(const MeshData& mesh, u32 submeshIndex, std::vector<u32>& out) const;

    // Moves the triangles into one new trailing submesh (inherits materialSlot/lightmapPage of the first triangle's submesh). Returns false if none moved.
    bool groupFacesIntoSubmesh(MeshData& mesh, const std::vector<u32>& faceIndices) const;

    // smooth=false writes the face normal into all three vertices, wrong for shared vertices (last face wins); split the mesh first for hard edges.
    void recalculateNormals(MeshData& mesh, bool smooth, bool angleWeighted = false) const;
    void recalculateTangents(MeshData& mesh) const;

    // Per triangle, projects along the axis its normal is most aligned with.
    void makePlanarUV(MeshData& mesh, f32 resolution) const;

    // Fixed axis: 0 = X, 1 = Y, 2 = Z.
    void makePlanarUV(MeshData& mesh, f32 resolutionS, f32 resolutionT, u8 axis,
                      const Math::vec3& offset) const;

    // u wraps once around Y (atan2(x,z)/2pi), v runs over the mesh's Y extent; resolutions are tile counts.
    // The u seam needs vertex duplication, as in makePlanarUV.
    void makeCylindricalUV(MeshData& mesh, f32 resolutionU, f32 resolutionV) const;

    // Same seam as makeCylindricalUV; pole vertices are duplicated per triangle since they have no single longitude.
    void makeSphericalUV(MeshData& mesh, f32 resolutionU, f32 resolutionV) const;

    // Scale, then rotate, then offset around the centre of the faces' UV bounds.
    // Vertices shared with unselected faces are duplicated first (creating a UV seam); weldVertices() undoes it.
    // Empty `faceIndices` takes the whole mesh. False when the mesh has no UVs.
    bool transformFaceUVs(MeshData& mesh, const std::vector<u32>& faceIndices,
                          const Math::vec2& scale, f32 rotationDegrees,
                          const Math::vec2& offset) const;

    void translate(MeshData& mesh, const Math::vec3& delta) const;
    void scale(MeshData& mesh, const Math::vec3& factor) const;

    // Normals and tangents use the inverse transpose so non-uniform scale keeps them on the surface.
    void transform(MeshData& mesh, const Math::mat4& matrix) const;

    // Restricted to `vertexIndices`, applied about their median point; empty means the whole mesh.
    void transformVertices(MeshData& mesh, const Math::mat4& matrix,
                           const std::vector<u32>& vertexIndices = {}) const;

    // Explicit pivot, for a matrix that already carries its placement (a gizmo). Zero pivot = world space.
    void transformVerticesAbout(MeshData& mesh, const Math::mat4& matrix, const Math::vec3& pivot,
                                const std::vector<u32>& vertexIndices = {}) const;

    void center(MeshData& mesh) const;

    // Centres on X and Z, lowest point at y = 0.
    void centerOnGround(MeshData& mesh) const;

    // Reverses triangle winding. Physics back ends often want the opposite of
    // the graphics one, and a mirrored transform turns the mesh inside out.
    void flipWinding(MeshData& mesh) const;

    // Restricted to one submesh's index range.
    void flipWinding(MeshData& mesh, u32 submeshIndex) const;

    // Copies one submesh out as a standalone mesh: referenced vertices only, re-indexed, material carried. Positions stay in source space.
    bool extractSubmesh(const MeshData& source, u32 submeshIndex, MeshData& out) const;

    // Only erases the SubMesh entry; orphaned vertices/indices stay in the buffers (compaction not worth it).
    bool removeSubmesh(MeshData& mesh, u32 submeshIndex) const;

    // Merges bit-identical vertices across all streams and rewrites the index buffer. Returns vertices removed.
    u32 weldVertices(MeshData& mesh) const;

    // Merges by proximity (uniform grid of `distance` cells); merged position is the group average, degenerate triangles dropped.
    // Empty `vertexIndices` welds the whole mesh; otherwise only listed vertices are candidates. Returns vertices removed.
    u32 weldVertices(MeshData& mesh, f32 distance, const std::vector<u32>& vertexIndices = {}) const;

    // Laplacian smoothing: each pass moves vertices `strength` toward their edge-neighbour average. Empty `vertexIndices` = whole mesh.
    void smoothVertices(MeshData& mesh, f32 strength, u32 iterations,
                        const std::vector<u32>& vertexIndices = {}) const;

    // Index reorder only, for vertex-cache reuse.
    void optimizeVertexCache(MeshData& mesh) const;

    // Reorders triangles front-to-back within clusters to cut overdraw; run after optimizeVertexCache. `threshold` is the cache efficiency it may give back (1.05 = 5%).
    void optimizeOverdraw(MeshData& mesh, f32 threshold = 1.05f) const;

    // Reorders vertex streams to first-reference order and drops unreferenced vertices. Run last.
    void optimizeVertexFetch(MeshData& mesh) const;

    // Collapses edges to about targetRatio of triangles or until deviation reaches targetError (fraction of mesh extent); borders and material seams hold.
    // Leaves unreferenced vertices (run optimizeVertexFetch). resultError receives the worst deviation reached.
    bool simplifyMesh(MeshData& mesh, f32 targetRatio, f32 targetError = 0.01f,
                      f32* resultError = nullptr) const;

    void buildCollisionMesh(const MeshData& mesh, CollisionMesh& out) const;

    // Closest triangle hit, in mesh space. Returns false when nothing is hit.
    bool raycast(const CollisionMesh& mesh, const Ray& ray, f32& t, u32& triangle) const;

    // Returns the cached handle if the filename was already loaded in the same space. On failure logs and returns getDefaultTexture().
    // `space` has no default on purpose; for material slots pass Material::colorSpaceFor(slot).
    // `mipLimit` caps the chain (0 = all); atlases need it because deep mips blend neighbouring regions.
    TextureHandle loadTexture(const std::string& filename, ColorSpace space,
                              bool generateMips = true, u32 mipLimit = 0);

    // Never blocks: returns a placeholder (cached under the same key) and queues the decode on AsyncTextureLoader.
    // processAsyncTextureLoads() must be called once per frame or the placeholder never becomes the real image.
    TextureHandle loadTextureAsync(const std::string& filename, ColorSpace space,
                                   bool generateMips = true, u32 mipLimit = 0);

    // Uploads finished async decodes; main thread only.
    void processAsyncTextureLoads();

    // Drops the cache entry first so a file changed on disk is re-read and re-uploaded.
    TextureHandle reloadTexture(const std::string& filename, ColorSpace space,
                                bool generateMips = true, u32 mipLimit = 0);

    // Six face paths in GL cube-face order (+X -X +Y -Y +Z -Z); see AssetTexture.cpp for why they upload in one buffer.
    // On failure returns an invalid handle (not the checker) so the caller can fall back to a procedural sky.
    TextureHandle loadCubemap(const std::string faces[6], const std::string& cacheName,
                              ColorSpace space, bool generateMips = true);

    // Tries the known suffix conventions (_RT/_LF, _px/_nx, SkyBox_Right/..) and uses the first with all six files. `baseName` may be a directory.
    TextureHandle loadCubemap(const std::string& baseName, ColorSpace space,
                              bool generateMips = true);

    // Base names of every complete cubemap under `directory`; touches the filesystem, so call once and cache.
    std::vector<std::string> listCubemaps(const std::string& directory) const;

    // Registers a GPU-created texture under `name`; returns the existing handle if already registered (desc ignored).
    TextureHandle createTexture(const std::string& name, const TextureDesc& desc);

    // Magenta/black checkerboard, loud on purpose; loadTexture() falls back to it for missing files.
    TextureHandle getDefaultTexture();

    TextureHandle getTexture(const std::string& name) const;

    // Load/create order, 0-based; destroyed slots are tombstoned, so check the returned handle.
    TextureHandle getTextureByIndex(usize index) const;
    usize textureCount() const;

    const std::string& textureName(TextureHandle handle) const;

    void destroyTexture(TextureHandle handle);
    void destroyAllTextures();

    // Samplers with the same Filter/Wrap share one GL sampler. Linear-scan cache on purpose (few samplers).
    // No per-sampler remove; destroyAllTextures() calls destroyAllSamplers().
    SamplerHandle getSampler(const SamplerDesc& desc);
    void destroyAllSamplers();

    // A render-to-texture technique publishes its output under a name each frame; ForwardPass resolves it by name hash. Not ownership.
    void publishRenderTarget(u32 nameHash, TextureHandle texture);
    void publishRenderTarget(const char* name, TextureHandle texture);
    TextureHandle resolveRenderTarget(u32 nameHash) const;

    // Expands `#include "other.glsl"` lines recursively (GLSL has none). Cached by filename; edits need reloadShader().
    const std::string& loadShader(const std::string& filename);

    // Drops the cached text (and includes) so the next loadShader() re-reads from disk.
    void reloadShader(const std::string& filename);
    void reloadAllShaders();

private:
    AssetManager();

    bool upload(const MeshData& data, Mesh& out, Residency residency) const;
    void release(Mesh& mesh) const;
    // Resolves MeshData's per-material texture file lists to TextureHandles, once.
    void loadMeshMaterialTextures(Mesh& mesh, const MeshData& data);

    // Appends a copy of vertex `source` (position/normal/color/skin); UV generators use it to split at seams.
    u32 duplicateMeshVertex(MeshData& mesh, u32 source) const;

    // Files a GPU texture into the lookup tables under `cacheKey`; also used for the async placeholder so a second request finds the pending handle.
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

    // The one place that knows how each MeshSource is made.
    MeshHandle buildFromDesc(const MeshDesc& desc);

    Pool<Mesh, MeshHandle> mMeshes;
    MeshLoader mMeshLoader;
    HashMap<u64, MeshDesc> mMeshDescs;           // packHandle() -> description
    HashMap<std::string, MeshHandle> mMeshByKey; // MeshDesc::key() -> handle

    // Queued file meshes plus the single import in flight, one at a time on purpose: only decode may run on a worker; search-path and material registration touch singletons and run on the main thread.
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

AssetManager& Assets();

} // namespace Radion

#endif // RADION_ASSET_MANAGER_H
