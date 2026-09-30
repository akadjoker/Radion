#ifndef RADION_VOXEL_NEIGHBOURHOOD_H
#define RADION_VOXEL_NEIGHBOURHOOD_H

#include "VoxelChunk.h"

#include <vector>

namespace Radion
{
namespace Voxel
{

class VoxelWorld;

// One chunk plus a one-block shell copied out of the world: a worker meshes from this copy without reading the chunk map,
// so the main thread can keep loading and unloading chunks.
class VoxelNeighbourhood
{
public:
    static constexpr s32 Size = VoxelChunk::Size + 2;
    static constexpr usize Volume = static_cast<usize>(Size) * Size * Size;

    void gather(const VoxelWorld& world, ChunkCoord coordinate);

    ChunkCoord coordinate() const { return mCoordinate; }
    bool valid() const { return mBlocks.size() == Volume; }

    // Chunk-local coordinates, valid from -1 to VoxelChunk::Size inclusive.
    BlockId block(s32 x, s32 y, s32 z) const;
    BlockId block(VoxelCoord local) const { return block(local.x, local.y, local.z); }

private:
    static usize paddedIndex(s32 x, s32 y, s32 z);

    ChunkCoord mCoordinate;
    std::vector<BlockId> mBlocks;
};

} // namespace Voxel
} // namespace Radion

#endif // RADION_VOXEL_NEIGHBOURHOOD_H
