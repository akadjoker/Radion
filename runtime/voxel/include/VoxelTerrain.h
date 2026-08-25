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
    VoxelTerrain(const BlockRegistry& blocks, u32 seed);

    // Fills one chunk. Only the y == 0 layer is generated; higher layers
    // stay air, which keeps this first pass to one vertical chunk.
    void generate(VoxelWorld& world, ChunkCoord coordinate) const;

private:
    s32 surfaceHeight(s32 x, s32 z) const;

    Noise::Perlin mContinental;
    Noise::Perlin mDetail;

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
