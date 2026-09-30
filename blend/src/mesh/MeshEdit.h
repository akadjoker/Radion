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

// Topological edits on a triangle MeshData: pure functions, no engine, no GL.
//
// They keep every per-vertex array (normals, tangents, UVs, colours, skin) the
// same length as `positions`, give new vertices interpolated attributes, and
// keep each submesh's triangles together with its range and bounds up to date.
// Normals and tangents of new vertices are interpolated, not recomputed: an edit
// that moves the surface (smooth subdivision) leaves that to the caller, which
// owns the smoothing choice.
//
// Edges are named by MeshTopology::edgeKey (canonical vertex ids), so an edit
// reaches every vertex standing at the same point and a UV seam never becomes a
// crack. Each function returns false, with `error` set and the mesh untouched,
// when it cannot do what was asked.

// An edge to cut. `t` runs from the edge's lower canonical vertex (0) to its
// higher one (1), whatever way the triangles happen to walk it.
struct EdgeSplit
{
    u64 key = 0;
    f32 t = 0.5f;
};

// Two triangles that make a quad, cut across by a line joining the middle of two
// opposite sides. `corner` lists the quad's vertices in order p, q, r, s (so the
// sides p-q and r-s are the ones cut, and the new edge runs between their
// midpoints) and both sides must be among the edges being cut.
struct QuadCut
{
    u32 faceA = 0;
    u32 faceB = 0;
    std::array<u32, 4> corner = {0, 0, 0, 0};
};

struct RefineResult
{
    // A vertex created on an edge, with the canonical edge it lies on.
    struct Midpoint
    {
        u32 vertex = 0;
        u64 edge = 0;
    };
    std::vector<Midpoint> midpoints;
    // For each triangle of the result, the triangle of the input it came from.
    std::vector<u32> origin;
    // The vertex made on the side between two vertices (by vertex index, in
    // either order), for callers that need the one a particular triangle uses.
    std::unordered_map<u64, u32> byVertexPair;
    static u64 pairKey(u32 a, u32 b)
    {
        if (a > b)
            std::swap(a, b);
        return (static_cast<u64>(a) << 32) | b;
    }
};

// Cuts the given edges, splitting every triangle that has one. A triangle with
// one cut edge becomes two, with two becomes three, with three becomes four, so
// no T-junction is left where a cut edge meets a triangle that was not chosen.
// A vertex is shared between the triangles that share both ends of the edge,
// and kept apart where they do not (a UV or normal seam stays a seam).
bool refineEdges(MeshData& mesh, const std::vector<EdgeSplit>& splits, RefineResult* result = nullptr,
                 std::string* error = nullptr);
// The same, with some pairs of triangles cut as quads: the line between the two
// cut sides becomes a real edge instead of the diagonal they were made of.
bool refineEdges(MeshData& mesh, const std::vector<EdgeSplit>& splits, const std::vector<QuadCut>& quads,
                 RefineResult* result = nullptr, std::string* error = nullptr);

// Subdivides `faces` (every triangle when empty), `levels` times. Flat, each
// triangle becomes four with the new vertices on the old edges; `smooth` moves
// the surface by Loop's rules (a selection's border counts as a boundary, so
// the region stays joined to what it was joined to). Triangles next to the
// region are cut just enough to stay watertight.
bool subdivide(MeshData& mesh, const std::vector<u32>& faces, u32 levels, bool smooth,
               std::string* error = nullptr);

// Flips the diagonal of the quad two triangles make. Needs an edge with exactly
// two triangles, no seam along it, and a convex quad - a flip that would fold the
// surface over is refused.
bool turnEdge(MeshData& mesh, u64 edgeKey, std::string* error = nullptr);

// Pulls the two ends of an edge together at `t` along it and removes the
// triangles that collapse. The vertices stay separate but coincide.
bool collapseEdge(MeshData& mesh, u64 edgeKey, f32 t, std::string* error = nullptr);

// Cuts the mesh with the plane `dot(normal, p) == offset`: every edge that
// crosses it gets a vertex where it does, and the triangles are split along the
// line between them. Vertices within `epsilon` of the plane count as on it and
// are not cut. `cutEdges`, when given, receives the edges that now lie in the
// plane.
bool knife(MeshData& mesh, const glm::vec3& normal, f32 offset, f32 epsilon, std::vector<u64>* cutEdges = nullptr,
           std::string* error = nullptr);

// Adds `cuts` evenly spaced edge loops across the ring of quads that contains
// `edgeKey`. A ring is followed through pairs of triangles that make a quad,
// both ways, until it closes on itself or reaches something that is not a quad -
// a triangle, a border - where the last triangle is just split. `newEdges`, when
// given, receives the edges that make up the new loops.
bool loopCut(MeshData& mesh, u64 edgeKey, u32 cuts, std::vector<u64>* newEdges = nullptr,
             std::string* error = nullptr);

// Insets `faces` (as one region): the region shrinks away from its border by
// `thickness`, measured across the surface, leaving a ring of triangles between
// the old border and the new; `depth` then moves the inner region along its
// normal (negative sinks it). `innerFaces` receives the triangles of the
// shrunken region, so it can be inset or extruded again.
bool inset(MeshData& mesh, const std::vector<u32>& faces, f32 thickness, f32 depth,
           std::vector<u32>* innerFaces = nullptr, std::string* error = nullptr);

// Chamfers edges: each becomes a flat strip `width` wide on either side of where
// it was, and the two vertices at its ends are replaced by the ends of the strip.
// Every edge needs exactly two triangles, and no two of the edges may share a
// vertex (bevel those one after another). The old vertices are dropped from the
// mesh, so vertex numbers change.
bool bevel(MeshData& mesh, const std::vector<u64>& edges, f32 width, std::string* error = nullptr);

// Triangulates a simple polygon (concave allowed, no holes) by ear clipping, in
// the plane it lies nearest to. `points` are in order; the triples returned index
// them and are wound the same way as the points are - counter-clockwise seen from
// the side the outline turns counter-clockwise towards.
std::vector<std::array<u32, 3>> triangulatePolygon(const std::vector<glm::vec3>& points);

// Closes open borders with triangles. `edges` picks the borders to close (every
// border that has one of those edges); empty means every border. A border
// longer than `maxEdges` is left open. Concave outlines are triangulated by ear
// clipping in the plane they lie nearest to. `filled` receives how many borders
// were closed.
bool fillHoles(MeshData& mesh, const std::vector<u64>& edges, u32 maxEdges, u32* filled = nullptr,
               std::string* error = nullptr);

// Joins two borders with a strip of triangles, the way a tube joins two rings.
// `edges` picks the borders (exactly two must be chosen); empty means the mesh
// must have exactly two. The borders may have different numbers of edges; the
// strip starts where the two lie closest.
bool bridge(MeshData& mesh, const std::vector<u64>& edges, std::string* error = nullptr);

// Copies `faces` (every triangle when empty) mirrored across the plane where
// coordinate `axis` (0 = x, 1 = y, 2 = z) equals `offset`, with the winding
// turned back so the copy faces outward. With `weld` greater than zero, vertices
// within that distance of the plane are shared by the copy instead of doubled,
// so the two halves join without a seam.
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
