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
    if (!isLocal(local))
        return AirBlockId;
    return mBlocks.empty() ? mUniform : mBlocks[localIndex(local)];
}

void VoxelChunk::expand()
{
    mBlocks.assign(Volume, mUniform);
}

bool VoxelChunk::setBlock(VoxelCoord local, BlockId block)
{
    if (!isLocal(local))
        return false;

    if (mBlocks.empty())
    {
        if (block == mUniform)
            return false;
        expand();
    }

    BlockId& current = mBlocks[localIndex(local)];
    if (current == block)
        return false;

    current = block;
    mDirty = true;
    ++mRevision;
    return true;
}

void VoxelChunk::compact()
{
    if (mBlocks.empty())
        return;

    const BlockId first = mBlocks[0];
    for (BlockId block : mBlocks)
    {
        if (block != first)
            return;
    }

    mUniform = first;
    mBlocks.clear();
    mBlocks.shrink_to_fit();
}

void VoxelChunk::reset(ChunkCoord coordinate)
{
    mCoordinate = coordinate;
    mBlocks.clear();
    mUniform = AirBlockId;
    mDirty = true;
    mRevision = 0;
}

void VoxelChunk::fill(BlockId block)
{
    mBlocks.clear();
    mBlocks.shrink_to_fit();
    mUniform = block;
    mDirty = true;
    ++mRevision;
}

} // namespace Voxel
} // namespace Radion
