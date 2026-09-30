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
};

// Cuts the given edges, splitting every triangle that has one. A triangle with
// one cut edge becomes two, with two becomes three, with three becomes four, so
// no T-junction is left where a cut edge meets a triangle that was not chosen.
// A vertex is shared between the triangles that share both ends of the edge,
// and kept apart where they do not (a UV or normal seam stays a seam).
bool refineEdges(MeshData& mesh, const std::vector<EdgeSplit>& splits, RefineResult* result = nullptr,
                 std::string* error = nullptr);

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

// Largest triangle count an edit here will produce.
constexpr usize kMaxTriangles = 4000000;

} // namespace Radion::MeshEdit

#endif // RADION_BLENDER_MESH_EDIT_H
