#include "VoxelWorld.h"

namespace Radion
{
namespace Voxel
{
namespace
{
s32 floorDivide(s32 value, s32 divisor)
{
    const s32 quotient = value / divisor;
    const s32 remainder = value % divisor;
    return remainder < 0 ? quotient - 1 : quotient;
}
} // namespace

ChunkCoord VoxelWorld::chunkFor(VoxelCoord world)
{
    return {floorDivide(world.x, VoxelChunk::Size), floorDivide(world.y, VoxelChunk::Size),
            floorDivide(world.z, VoxelChunk::Size)};
}

VoxelCoord VoxelWorld::localFor(VoxelCoord world)
{
    const ChunkCoord chunk = chunkFor(world);
    return {world.x - chunk.x * VoxelChunk::Size, world.y - chunk.y * VoxelChunk::Size,
            world.z - chunk.z * VoxelChunk::Size};
}

VoxelCoord VoxelWorld::worldFor(ChunkCoord chunk, VoxelCoord local)
{
    return {chunk.x * VoxelChunk::Size + local.x, chunk.y * VoxelChunk::Size + local.y,
            chunk.z * VoxelChunk::Size + local.z};
}

VoxelChunk& VoxelWorld::ensureChunk(ChunkCoord coordinate)
{
    const auto result = mChunks.emplace(coordinate, VoxelChunk(coordinate));
    return result.first->second;
}

VoxelChunk* VoxelWorld::findChunk(ChunkCoord coordinate)
{
    const auto it = mChunks.find(coordinate);
    return it == mChunks.end() ? nullptr : &it->second;
}

const VoxelChunk* VoxelWorld::findChunk(ChunkCoord coordinate) const
{
    const auto it = mChunks.find(coordinate);
    return it == mChunks.end() ? nullptr : &it->second;
}

bool VoxelWorld::removeChunk(ChunkCoord coordinate)
{
    return mChunks.erase(coordinate) != 0;
}

void VoxelWorld::clear()
{
    mChunks.clear();
}

BlockId VoxelWorld::block(VoxelCoord world) const
{
    const VoxelChunk* chunk = findChunk(chunkFor(world));
    return chunk ? chunk->block(localFor(world)) : AirBlockId;
}

bool VoxelWorld::setBlock(VoxelCoord world, BlockId block)
{
    const ChunkCoord chunkCoordinate = chunkFor(world);
    const VoxelCoord local = localFor(world);
    VoxelChunk& chunk = ensureChunk(chunkCoordinate);
    if (!chunk.setBlock(local, block))
        return false;

    markLoadedNeighboursDirty(chunkCoordinate, local);
    return true;
}

void VoxelWorld::markLoadedNeighboursDirty(ChunkCoord chunk, VoxelCoord local)
{
    const auto mark = [this, chunk](s32 x, s32 y, s32 z)
    {
        if (VoxelChunk* neighbour = findChunk({chunk.x + x, chunk.y + y, chunk.z + z}))
            neighbour->markDirty();
    };

    if (local.x == 0)
        mark(-1, 0, 0);
    else if (local.x == VoxelChunk::Size - 1)
        mark(1, 0, 0);
    if (local.y == 0)
        mark(0, -1, 0);
    else if (local.y == VoxelChunk::Size - 1)
        mark(0, 1, 0);
    if (local.z == 0)
        mark(0, 0, -1);
    else if (local.z == VoxelChunk::Size - 1)
        mark(0, 0, 1);
}

} // namespace Voxel
} // namespace Radion
