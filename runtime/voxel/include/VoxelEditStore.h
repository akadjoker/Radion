#ifndef RADION_VOXEL_EDIT_STORE_H
#define RADION_VOXEL_EDIT_STORE_H

#include "VoxelChunk.h"

#include <unordered_map>
#include <vector>

namespace Radion
{
namespace Voxel
{

struct VoxelEditRecord
{
    u16 index = 0;
    BlockId block = AirBlockId;
};

// Every player/designer edit, kept per chunk apart from generated terrain: a world saves as seed plus this store,
// and a chunk streaming back in is regenerated then replayed from here.
class VoxelEditStore
{
public:
    void record(VoxelCoord position, BlockId block);
    // Replays this chunk's edits over freshly generated terrain.
    bool apply(VoxelChunk& chunk) const;
    bool contains(ChunkCoord coordinate) const;

    usize chunkCount() const { return mChunks.size(); }
    usize recordCount() const;
    bool empty() const { return mChunks.empty(); }
    void clear();

    void write(std::vector<u8>& bytes) const;
    bool read(const std::vector<u8>& bytes);

private:
    std::unordered_map<ChunkCoord, std::vector<VoxelEditRecord>, ChunkCoordHash> mChunks;
};

} // namespace Voxel
} // namespace Radion

#endif // RADION_VOXEL_EDIT_STORE_H
