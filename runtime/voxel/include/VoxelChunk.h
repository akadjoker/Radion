#ifndef RADION_VOXEL_CHUNK_H
#define RADION_VOXEL_CHUNK_H

#include "VoxelBlock.h"

#include <array>

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

// A fixed-size, CPU-only block store.  Rendering and collision consume a
// rebuilt result later; editing a chunk only changes this authoritative data.
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

    bool dirty() const { return mDirty; }
    void markDirty() { mDirty = true; }
    void clearDirty() { mDirty = false; }
    u64 revision() const { return mRevision; }

private:
    ChunkCoord mCoordinate;
    std::array<BlockId, Volume> mBlocks = {};
    bool mDirty = true;
    u64 mRevision = 0;
};

} // namespace Voxel
} // namespace Radion

#endif // RADION_VOXEL_CHUNK_H
