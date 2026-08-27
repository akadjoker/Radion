#ifndef RADION_VOXEL_TERRAIN_H
#define RADION_VOXEL_TERRAIN_H

#include "Noise.h"
#include "VoxelBlock.h"
#include "VoxelWorld.h"

namespace Radion
{
namespace Voxel
{

// Deterministic heightmap terrain over a single vertical layer of chunks.
// Heights are sampled from world (x,z) coordinates, never chunk-local ones,
// so two neighbouring chunks always agree on the column at their shared
// boundary. The same seed reproduces the same world on any machine.
class VoxelTerrain
{
public:
    struct Settings
    {
        s32 minWorldY = 0;
        s32 maxWorldY = VoxelChunk::Size - 1;
        s32 waterLevel = 13;
        s32 minSurfaceHeight = 2;
        s32 maxSurfaceHeight = 30;
        f32 baseSurfaceHeight = 16.0f;
        f32 continentalAmplitude = 9.0f;
        f32 detailAmplitude = 3.0f;
    };

    VoxelTerrain(const BlockRegistry& blocks, u32 seed);
    VoxelTerrain(const BlockRegistry& blocks, u32 seed, const Settings& settings);

    // Fills the part of one chunk within the configured vertical world bounds.
    void generate(VoxelWorld& world, ChunkCoord coordinate) const;

private:
    s32 surfaceHeight(s32 x, s32 z) const;

    Noise::Perlin mContinental;
    Noise::Perlin mDetail;
    Settings mSettings;

    BlockId mGrass = AirBlockId;
    BlockId mDirt = AirBlockId;
    BlockId mStone = AirBlockId;
    BlockId mSand = AirBlockId;
    BlockId mBedrock = AirBlockId;
    BlockId mWater = AirBlockId;
};

} // namespace Voxel
} // namespace Radion

#endif // RADION_VOXEL_TERRAIN_H
