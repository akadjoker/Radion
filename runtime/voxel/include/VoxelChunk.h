#ifndef RADION_VOXEL_CHUNK_H
#define RADION_VOXEL_CHUNK_H

#include "VoxelBlock.h"

#include <vector>

namespace Radion
{
namespace Voxel
{

struct VoxelCoord
{
    s32 x = 0;
    s32 y = 0;
    s32 z = 0;
};

inline bool operator==(const VoxelCoord& a, const VoxelCoord& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

struct ChunkCoord
{
    s32 x = 0;
    s32 y = 0;
    s32 z = 0;
};

inline bool operator==(const ChunkCoord& a, const ChunkCoord& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

struct ChunkCoordHash
{
    usize operator()(const ChunkCoord& coord) const;
};

// A fixed-size, CPU-only block store; editing changes only this authoritative data.
// A chunk holding one block everywhere keeps no array and answers from `mUniform` (64 KB each is what a large view radius runs out of first);
// the array appears on the first write that disagrees.
class VoxelChunk
{
public:
    static constexpr s32 Size = 32;
    static constexpr usize Volume = static_cast<usize>(Size) * Size * Size;

    explicit VoxelChunk(ChunkCoord coordinate = {});

    ChunkCoord coordinate() const { return mCoordinate; }

    static bool isLocal(VoxelCoord local);
    static usize localIndex(VoxelCoord local);

    BlockId block(VoxelCoord local) const;
    bool setBlock(VoxelCoord local, BlockId block);
    void fill(BlockId block);

    // Drops the array when every block is the same; worth one scan after generation.
    void compact();
    // Back to empty air at a new coordinate, keeping reserved storage so a recycling streamer stops allocating.
    void reset(ChunkCoord coordinate);

    bool uniform() const { return mBlocks.empty(); }
    BlockId uniformBlock() const { return mUniform; }
    // Null while uniform, which callers reading a run of blocks have to
    // handle - see VoxelNeighbourhood::gather().
    const BlockId* blocks() const { return mBlocks.empty() ? nullptr : mBlocks.data(); }
    bool empty() const { return mBlocks.empty() && mUniform == AirBlockId; }
    usize memoryBytes() const { return mBlocks.capacity() * sizeof(BlockId); }

    bool dirty() const { return mDirty; }
    void markDirty() { mDirty = true; }
    void clearDirty() { mDirty = false; }
    u64 revision() const { return mRevision; }

private:
    void expand();

    ChunkCoord mCoordinate;
    std::vector<BlockId> mBlocks;
    BlockId mUniform = AirBlockId;
    bool mDirty = true;
    u64 mRevision = 0;
};

} // namespace Voxel
} // namespace Radion

#endif // RADION_VOXEL_CHUNK_H
