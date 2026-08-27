#ifndef RADION_VOXEL_NEIGHBOURHOOD_H
#define RADION_VOXEL_NEIGHBOURHOOD_H

#include "VoxelChunk.h"

#include <vector>

namespace Radion
{
namespace Voxel
{

class VoxelWorld;

// One chunk plus the single-block shell around it, copied out of the world in
// one go. A worker thread meshes from this copy and never reads the chunk map,
// which is what lets the main thread keep loading and unloading chunks while
// meshing is in flight.
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
