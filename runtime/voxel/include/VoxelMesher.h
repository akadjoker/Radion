#ifndef RADION_VOXEL_MESHER_H
#define RADION_VOXEL_MESHER_H

#include "Mesh.h"
#include "VoxelNeighbourhood.h"
#include "VoxelWorld.h"

namespace Radion
{
namespace Voxel
{

// One vertex stream, one submesh per pass (opaque, alpha-cutout, transparent, in that order, only when non-empty):
// one mesh per chunk is one scene object instead of three. Positions are local to the chunk origin.
struct VoxelMeshData
{
    static constexpr u32 OpaqueSlot = 0;
    static constexpr u32 CutoutSlot = 1;
    static constexpr u32 TransparentSlot = 2;

    MeshData mesh;

    void clear();
    bool empty() const { return mesh.indices.empty(); }
    bool hasSlot(u32 materialSlot) const;
};

class VoxelMesher
{
public:
    struct Settings
    {
        // Atlas dimensions in tiles, never texels; one means the face uses the full UV range.
        u16 atlasColumns = 1;
        u16 atlasRows = 1;
        // Per-vertex AO baked into mesh colours. It enters the greedy key (differently occluded cells must not merge),
        // costing about +75% vertices at chunk radius six. Off leaves every corner fully lit.
        bool ambientOcclusion = true;
    };

    // Builds only faces visible from outside their block; neighbour lookups use the gathered shell, so chunk borders add no hidden faces.
    static VoxelMeshData buildChunk(const VoxelNeighbourhood& neighbourhood,
                                    const BlockRegistry& blocks, Settings settings);
    static VoxelMeshData buildChunk(const VoxelWorld& world, const VoxelChunk& chunk,
                                    const BlockRegistry& blocks, Settings settings);
    static VoxelMeshData buildChunk(const VoxelWorld& world, const VoxelChunk& chunk,
                                    const BlockRegistry& blocks)
    {
        return buildChunk(world, chunk, blocks, Settings{});
    }
};

} // namespace Voxel
} // namespace Radion

#endif // RADION_VOXEL_MESHER_H
