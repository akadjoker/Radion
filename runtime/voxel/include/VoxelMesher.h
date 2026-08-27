#ifndef RADION_VOXEL_MESHER_H
#define RADION_VOXEL_MESHER_H

#include "Mesh.h"
#include "VoxelNeighbourhood.h"
#include "VoxelWorld.h"

namespace Radion
{
namespace Voxel
{

// One vertex stream for the chunk, and one submesh per render pass: opaque,
// alpha-cutout, then transparent, in that order and only when they have
// geometry. The passes need separate materials, not separate meshes - the
// render list already picks a material per submesh - and one mesh per chunk is
// one scene object instead of three. Positions are local to the chunk origin.
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
        // Atlas dimensions in tiles, never texels.  A value of one means the
        // face material occupies the complete UV range.
        u16 atlasColumns = 1;
        u16 atlasRows = 1;
        // Per-vertex ambient occlusion, baked into the mesh colours. It also
        // enters the greedy key - two cells whose corners are occluded
        // differently must not merge - and that costs geometry: measured at
        // +75% vertices over a chunk radius of six. Off merges as before and
        // leaves every corner fully lit.
        bool ambientOcclusion = true;
    };

    // Builds only faces visible from outside their block.  Neighbour lookups
    // come from the gathered shell, so a shared chunk boundary does not
    // produce a hidden internal face.
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
