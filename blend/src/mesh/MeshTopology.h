#ifndef RADION_BLENDER_MESH_TOPOLOGY_H
#define RADION_BLENDER_MESH_TOPOLOGY_H

#include "Mesh.h"
#include "Types.h"

#include <array>
#include <unordered_map>
#include <vector>

namespace Radion
{

// Connectivity of a triangle mesh, read off the index buffer.
//
// A MeshData's vertices are not points of the surface but corners with their own
// normal and UV: a cube has 24, a sphere repeats its seam. Edit tools care about
// the surface, so this works on *canonical* vertices - every vertex is mapped to
// the lowest-numbered vertex standing at the same position - and an edge is a
// pair of those. Two triangles that meet along a UV seam are neighbours here,
// which is what a person editing the shape expects.
//
// Canonical ids are vertex indices, not a renumbering, so an id (and an edge
// key) means the same thing every time the same mesh is analysed.
class MeshTopology
{
public:
    struct Edge
    {
        u32 a = 0; // canonical ids, a < b
        u32 b = 0;
        std::vector<u32> faces; // every triangle that has this edge
    };

    // Vertices closer than `epsilon` count as the same point.
    void build(const MeshData& mesh, f32 epsilon = 1.0e-5f);

    usize vertexCount() const
    {
        return mCanonical.size();
    }
    usize faceCount() const
    {
        return mFaceEdges.size();
    }

    u32 canonical(u32 vertex) const
    {
        return mCanonical[vertex];
    }
    // Every vertex standing at the same point as `vertex` (itself included),
    // ascending.
    std::vector<u32> coincident(u32 vertex) const;

    const std::vector<Edge>& edges() const
    {
        return mEdges;
    }
    static u64 edgeKey(u32 canonicalA, u32 canonicalB);
    // Index into edges(), or -1.
    s32 findEdge(u32 canonicalA, u32 canonicalB) const;

    // The edge at each corner: entry i joins corner i to corner (i+1)%3 of the
    // triangle, -1 when that side has no length (a collapsed triangle).
    const std::array<s32, 3>& faceEdges(u32 face) const
    {
        return mFaceEdges[face];
    }

    bool isBoundary(u32 edge) const
    {
        return mEdges[edge].faces.size() == 1;
    }
    bool isNonManifold(u32 edge) const
    {
        return mEdges[edge].faces.size() > 2;
    }

    // Triangles that share an edge with `face`, each once.
    void faceNeighbors(const MeshData& mesh, u32 face, std::vector<u32>& out) const;

    // The open boundary as loops of canonical ids, walked in the direction the
    // existing triangles traverse their own edges - so a face closing a loop must
    // run the other way round. Boundary edges that do not join up into a closed
    // loop (a pinched or non-manifold border) are left out.
    std::vector<std::vector<u32>> boundaryLoops(const MeshData& mesh) const;

private:
    std::vector<u32> mCanonical;
    std::vector<Edge> mEdges;
    std::vector<std::array<s32, 3>> mFaceEdges;
    std::unordered_map<u64, u32> mEdgeIndex;
};

} // namespace Radion

#endif // RADION_BLENDER_MESH_TOPOLOGY_H
