#ifndef RADION_VOXEL_MESHER_H
#define RADION_VOXEL_MESHER_H

#include "Mesh.h"
#include "VoxelWorld.h"

namespace Radion
{
namespace Voxel
{

// Kept separate because opaque, alpha-cutout and transparent geometry require
// different render passes.  Every mesh is local to its chunk's origin.
struct VoxelMeshData
{
    MeshData opaque;
    MeshData cutout;
    MeshData transparent;

    void clear();
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
        // Insets face UVs by half a texel when sampling an atlas with linear
        // filtering, preventing bilinear sampling from crossing tile edges.
        u16 atlasTilePixels = 0;
    };

    // Builds only faces visible from outside their block.  Neighbour lookups
    // go through VoxelWorld, so a shared chunk boundary does not produce a
    // hidden internal face.
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
