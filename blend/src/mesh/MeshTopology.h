#ifndef RADION_BLENDER_MESH_TOPOLOGY_H
#define RADION_BLENDER_MESH_TOPOLOGY_H

#include "Mesh.h"
#include "Types.h"

#include <array>
#include <unordered_map>
#include <vector>

namespace Radion
{

//
// Works on *canonical* vertices (each mapped to the lowest-numbered vertex at the same position), since MeshData vertices are corners with their own normal/UV.
// Triangles meeting along a UV seam are neighbours. Ids are indices, not a renumbering, so they are stable across analyses.
class MeshTopology
{
public:
    struct Edge
    {
        u32 a = 0;
        u32 b = 0;
        std::vector<u32> faces;
    };

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
    std::vector<u32> coincident(u32 vertex) const;

    const std::vector<Edge>& edges() const
    {
        return mEdges;
    }
    static u64 edgeKey(u32 canonicalA, u32 canonicalB);
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

    void faceNeighbors(const MeshData& mesh, u32 face, std::vector<u32>& out) const;

    // Open boundary as loops of canonical ids, walked the way existing triangles traverse their edges (a closing face runs the other way). Non-closing borders are left out.
    std::vector<std::vector<u32>> boundaryLoops(const MeshData& mesh) const;

private:
    std::vector<u32> mCanonical;
    std::vector<Edge> mEdges;
    std::vector<std::array<s32, 3>> mFaceEdges;
    std::unordered_map<u64, u32> mEdgeIndex;
};

} // namespace Radion

#endif // RADION_BLENDER_MESH_TOPOLOGY_H
