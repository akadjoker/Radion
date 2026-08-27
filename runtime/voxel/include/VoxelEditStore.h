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

// Every block a player or a designer changed, kept per chunk and apart from
// the generated terrain. A world saves as its seed plus this store, and a
// chunk that streams back in is regenerated and then replayed from here, so
// edits survive unloading without keeping the chunk in memory.
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
