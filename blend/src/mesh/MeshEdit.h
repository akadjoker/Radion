#ifndef RADION_BLENDER_MESH_EDIT_H
#define RADION_BLENDER_MESH_EDIT_H

#include "Mesh.h"
#include "Types.h"
#include "mesh/MeshTopology.h"

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

namespace Radion::MeshEdit
{

// Topological edits on a triangle MeshData: pure functions, no engine, no GL. Per-vertex arrays stay in step with `positions`; new vertices get interpolated (not recomputed) normals and tangents.
// Edges are named by MeshTopology::edgeKey (canonical ids). Each function returns false with `error` set and the mesh untouched on failure.

// An edge to cut; `t` runs from the edge's lower canonical vertex (0) to its higher (1).
struct EdgeSplit
{
    u64 key = 0;
    f32 t = 0.5f;
};

// Two triangles forming a quad, cut by a line joining the midpoints of two opposite sides; `corner` is p,q,r,s and sides p-q and r-s must be among the cut edges.
struct QuadCut
{
    u32 faceA = 0;
    u32 faceB = 0;
    std::array<u32, 4> corner = {0, 0, 0, 0};
};

struct RefineResult
{
    struct Midpoint
    {
        u32 vertex = 0;
        u64 edge = 0;
    };
    std::vector<Midpoint> midpoints;
    std::vector<u32> origin;
    std::unordered_map<u64, u32> byVertexPair;
    static u64 pairKey(u32 a, u32 b)
    {
        if (a > b)
            std::swap(a, b);
        return (static_cast<u64>(a) << 32) | b;
    }
};

// Cuts the given edges, splitting every triangle that has one (no T-junctions). Vertices are shared only between triangles sharing both ends; seams stay seams.
bool refineEdges(MeshData& mesh, const std::vector<EdgeSplit>& splits, RefineResult* result = nullptr,
                 std::string* error = nullptr);
// Same, cutting some triangle pairs as quads so the line between the cut sides becomes a real edge.
bool refineEdges(MeshData& mesh, const std::vector<EdgeSplit>& splits, const std::vector<QuadCut>& quads,
                 RefineResult* result = nullptr, std::string* error = nullptr);

// Subdivides `faces` (all when empty) `levels` times; `smooth` uses Loop's rules with a selection's border as a boundary. Neighbouring triangles are cut just enough to stay watertight.
bool subdivide(MeshData& mesh, const std::vector<u32>& faces, u32 levels, bool smooth,
               std::string* error = nullptr);

// Flips a quad's diagonal. Needs exactly two triangles, no seam, and a convex quad.
bool turnEdge(MeshData& mesh, u64 edgeKey, std::string* error = nullptr);

// Pulls an edge's ends together at `t`, removing collapsed triangles; the vertices stay separate but coincide.
bool collapseEdge(MeshData& mesh, u64 edgeKey, f32 t, std::string* error = nullptr);

// Cuts with the plane dot(normal, p) == offset; vertices within `epsilon` count as on it. `cutEdges` receives the edges now in the plane.
bool knife(MeshData& mesh, const Math::vec3& normal, f32 offset, f32 epsilon, std::vector<u64>* cutEdges = nullptr,
           std::string* error = nullptr);

// Adds `cuts` evenly spaced edge loops across the quad ring containing `edgeKey`, ending at a non-quad. `newEdges` receives the new loop edges.
bool loopCut(MeshData& mesh, u64 edgeKey, u32 cuts, std::vector<u64>* newEdges = nullptr,
             std::string* error = nullptr);

// Insets `faces` as one region by `thickness` across the surface; `depth` moves the inner region along its normal. `innerFaces` receives the shrunken region.
bool inset(MeshData& mesh, const std::vector<u32>& faces, f32 thickness, f32 depth,
           std::vector<u32>* innerFaces = nullptr, std::string* error = nullptr);

// Chamfers edges into strips `width` wide. Each edge needs exactly two triangles and no two may share a vertex; old vertices are dropped, so indices change.
bool bevel(MeshData& mesh, const std::vector<u64>& edges, f32 width, std::string* error = nullptr);

// Triangulates a simple polygon (concave ok, no holes) by ear clipping; triples index `points` and keep its winding.
std::vector<std::array<u32, 3>> triangulatePolygon(const std::vector<Math::vec3>& points);

// Closes open borders (those with an edge in `edges`, all when empty); borders longer than `maxEdges` stay open. `filled` receives the count.
bool fillHoles(MeshData& mesh, const std::vector<u64>& edges, u32 maxEdges, u32* filled = nullptr,
               std::string* error = nullptr);

// Joins two borders with a triangle strip; `edges` picks them (empty means the mesh must have exactly two); the strip starts where they lie closest.
bool bridge(MeshData& mesh, const std::vector<u64>& edges, std::string* error = nullptr);

// Copies `faces` mirrored across coordinate `axis` == `offset` with winding restored; vertices within `weld` of the plane are shared.
bool mirror(MeshData& mesh, s32 axis, f32 offset, f32 weld, const std::vector<u32>& faces,
            std::string* error = nullptr);

// Joins submeshes into one (the lowest-numbered of them; its material stays).
bool mergeSubmeshes(MeshData& mesh, const std::vector<u32>& submeshes, std::string* error = nullptr);

// Drops the vertices no triangle uses, renumbering the rest (the order is kept).
// Returns how many went.
u32 removeUnusedVertices(MeshData& mesh);

// A copy of vertex `vertex` appended at the end, every attribute array carried
// along - for a caller that wants to give one corner of a triangle its own UV.
u32 duplicateVertex(MeshData& mesh, u32 vertex);

// Largest triangle count an edit here will produce.
constexpr usize kMaxTriangles = 4000000;

} // namespace Radion::MeshEdit

#endif // RADION_BLENDER_MESH_EDIT_H
