#include "VoxelNeighbourhood.h"

#include "VoxelWorld.h"

#include <algorithm>
#include <cstring>

namespace Radion
{
namespace Voxel
{

usize VoxelNeighbourhood::paddedIndex(s32 x, s32 y, s32 z)
{
    return static_cast<usize>(x + 1) + static_cast<usize>(y + 1) * Size +
           static_cast<usize>(z + 1) * Size * Size;
}

BlockId VoxelNeighbourhood::block(s32 x, s32 y, s32 z) const
{
    if (x < -1 || x > VoxelChunk::Size || y < -1 || y > VoxelChunk::Size || z < -1 ||
        z > VoxelChunk::Size || !valid())
    {
        return AirBlockId;
    }
    return mBlocks[paddedIndex(x, y, z)];
}

void VoxelNeighbourhood::gather(const VoxelWorld& world, ChunkCoord coordinate)
{
    mCoordinate = coordinate;
    mBlocks.assign(Volume, AirBlockId);

    for (s32 chunkZ = -1; chunkZ <= 1; ++chunkZ)
    {
        for (s32 chunkY = -1; chunkY <= 1; ++chunkY)
        {
            for (s32 chunkX = -1; chunkX <= 1; ++chunkX)
            {
                const VoxelChunk* chunk = world.findChunk({coordinate.x + chunkX,
                                                           coordinate.y + chunkY,
                                                           coordinate.z + chunkZ});
                if (!chunk || chunk->empty())
                    continue;
                const BlockId* source = chunk->blocks();
                const BlockId uniform = chunk->uniformBlock();

                // A neighbour's local coordinate lands on this chunk's local
                // axis shifted by a whole chunk, and only the part that falls
                // inside the padded box is copied.
                const s32 shiftX = chunkX * VoxelChunk::Size;
                const s32 shiftY = chunkY * VoxelChunk::Size;
                const s32 shiftZ = chunkZ * VoxelChunk::Size;
                const s32 firstX = std::max(0, -1 - shiftX);
                const s32 lastX = std::min(VoxelChunk::Size - 1, VoxelChunk::Size - shiftX);
                const s32 firstY = std::max(0, -1 - shiftY);
                const s32 lastY = std::min(VoxelChunk::Size - 1, VoxelChunk::Size - shiftY);
                const s32 firstZ = std::max(0, -1 - shiftZ);
                const s32 lastZ = std::min(VoxelChunk::Size - 1, VoxelChunk::Size - shiftZ);
                if (firstX > lastX || firstY > lastY || firstZ > lastZ)
                    continue;

                const usize count = static_cast<usize>(lastX - firstX + 1);
                for (s32 z = firstZ; z <= lastZ; ++z)
                {
                    for (s32 y = firstY; y <= lastY; ++y)
                    {
                        const usize targetIndex =
                            paddedIndex(firstX + shiftX, y + shiftY, z + shiftZ);
                        if (source)
                        {
                            const usize sourceIndex = VoxelChunk::localIndex({firstX, y, z});
                            std::memcpy(&mBlocks[targetIndex], &source[sourceIndex],
                                        count * sizeof(BlockId));
                        }
                        else
                        {
                            std::fill_n(&mBlocks[targetIndex], count, uniform);
                        }
                    }
                }
            }
        }
    }
}

} // namespace Voxel
} // namespace Radion
