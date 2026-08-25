#include "VoxelChunk.h"

#include <functional>

namespace Radion
{
namespace Voxel
{

usize ChunkCoordHash::operator()(const ChunkCoord& coord) const
{
    const usize x = std::hash<s32>{}(coord.x);
    const usize y = std::hash<s32>{}(coord.y);
    const usize z = std::hash<s32>{}(coord.z);
    return x ^ (y << 1) ^ (z << 7);
}

VoxelChunk::VoxelChunk(ChunkCoord coordinate)
    : mCoordinate(coordinate)
{
}

bool VoxelChunk::isLocal(VoxelCoord local)
{
    return local.x >= 0 && local.x < Size && local.y >= 0 && local.y < Size && local.z >= 0
        && local.z < Size;
}

usize VoxelChunk::localIndex(VoxelCoord local)
{
    return static_cast<usize>(local.x) + static_cast<usize>(local.y) * Size
        + static_cast<usize>(local.z) * Size * Size;
}

BlockId VoxelChunk::block(VoxelCoord local) const
{
    return isLocal(local) ? mBlocks[localIndex(local)] : AirBlockId;
}

bool VoxelChunk::setBlock(VoxelCoord local, BlockId block)
{
    if (!isLocal(local))
        return false;

    BlockId& current = mBlocks[localIndex(local)];
    if (current == block)
        return false;

    current = block;
    mDirty = true;
    ++mRevision;
    return true;
}

void VoxelChunk::fill(BlockId block)
{
    mBlocks.fill(block);
    mDirty = true;
    ++mRevision;
}

} // namespace Voxel
} // namespace Radion
