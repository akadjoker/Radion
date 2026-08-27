#ifndef RADION_VOXEL_WORLD_H
#define RADION_VOXEL_WORLD_H

#include "VoxelChunk.h"

#include <unordered_map>

namespace Radion
{
namespace Voxel
{

// Sparse voxel world.  A missing chunk is air, which keeps streaming and
// generation policy out of the basic block access contract.
class VoxelWorld
{
public:
    static ChunkCoord chunkFor(VoxelCoord world);
    static VoxelCoord localFor(VoxelCoord world);
    static VoxelCoord worldFor(ChunkCoord chunk, VoxelCoord local);

    VoxelChunk& ensureChunk(ChunkCoord coordinate);
    VoxelChunk* findChunk(ChunkCoord coordinate);
    const VoxelChunk* findChunk(ChunkCoord coordinate) const;
    template <class Function> void forEachChunk(Function&& function)
    {
        for (auto& entry : mChunks)
            function(entry.first, entry.second);
    }
    template <class Function> void forEachChunk(Function&& function) const
    {
        for (const auto& entry : mChunks)
            function(entry.first, entry.second);
    }
    bool removeChunk(ChunkCoord coordinate);
    void clear();
    usize chunkCount() const { return mChunks.size(); }

    BlockId block(VoxelCoord world) const;
    // Returns false when the block already had this value.  Border edits also
    // invalidate an already-loaded neighbouring chunk.
    bool setBlock(VoxelCoord world, BlockId block);

private:
    void markLoadedNeighboursDirty(ChunkCoord chunk, VoxelCoord local);

    std::unordered_map<ChunkCoord, VoxelChunk, ChunkCoordHash> mChunks;
};

} // namespace Voxel
} // namespace Radion

#endif // RADION_VOXEL_WORLD_H
