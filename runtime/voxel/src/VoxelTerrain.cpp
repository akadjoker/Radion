#include "VoxelTerrain.h"

#include <algorithm>
#include <cmath>

namespace Radion
{
namespace Voxel
{
namespace
{
constexpr s32 WaterLevel = 13;
constexpr s32 MinHeight = 2;
constexpr s32 MaxHeight = 30;
} // namespace

VoxelTerrain::VoxelTerrain(const BlockRegistry& blocks, u32 seed)
{
    mContinental.initialise(seed);
    mDetail.initialise(seed ^ 0x9E3779B9u);

    mGrass = blocks.findId("grass");
    mDirt = blocks.findId("dirt");
    mStone = blocks.findId("stone");
    mSand = blocks.findId("sand");
    mBedrock = blocks.findId("bedrock");
    mWater = blocks.findId("water");
}

s32 VoxelTerrain::surfaceHeight(s32 x, s32 z) const
{
    // Continental shape: low frequency, large amplitude. Detail: higher
    // frequency, small amplitude. Both map [-1,1] fBm to a height band, then
    // clamp so a single 32-tall chunk layer can hold the whole profile.
    const f32 continental =
        mContinental.compute(static_cast<f32>(x) * 0.016f, static_cast<f32>(z) * 0.016f, 0.0f, 3);
    const f32 detail =
        mDetail.compute(static_cast<f32>(x) * 0.06f, static_cast<f32>(z) * 0.06f, 0.0f, 2);
    const f32 height = 16.0f + continental * 9.0f + detail * 3.0f;
    return static_cast<s32>(
        std::clamp(std::round(height), static_cast<f32>(MinHeight), static_cast<f32>(MaxHeight)));
}

void VoxelTerrain::generate(VoxelWorld& world, ChunkCoord coordinate) const
{
    if (coordinate.y != 0)
        return;

    VoxelChunk& chunk = world.ensureChunk(coordinate);
    const s32 originX = coordinate.x * VoxelChunk::Size;
    const s32 originZ = coordinate.z * VoxelChunk::Size;

    for (s32 z = 0; z < VoxelChunk::Size; ++z)
    {
        for (s32 x = 0; x < VoxelChunk::Size; ++x)
        {
            const s32 worldX = originX + x;
            const s32 worldZ = originZ + z;
            const s32 surface = surfaceHeight(worldX, worldZ);
            const bool beach = surface <= WaterLevel + 1;

            for (s32 y = 0; y < VoxelChunk::Size; ++y)
            {
                BlockId block = AirBlockId;
                if (y == 0)
                    block = mBedrock;
                else if (y < surface - 3)
                    block = mStone;
                else if (y < surface)
                    block = beach ? mSand : mDirt;
                else if (y == surface)
                    block = beach ? mSand : mGrass;
                else if (y <= WaterLevel)
                    block = mWater;

                chunk.setBlock({x, y, z}, block);
            }
        }
    }
}

} // namespace Voxel
} // namespace Radion
