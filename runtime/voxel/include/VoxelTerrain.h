#ifndef RADION_VOXEL_TERRAIN_H
#define RADION_VOXEL_TERRAIN_H

#include "Noise.h"
#include "VoxelBlock.h"
#include "VoxelWorld.h"

namespace Radion
{
namespace Voxel
{

enum class BiomeKind : u8
{
    Plains,
    Forest,
    Desert,
    Mountains,
    Snow,
    Count
};

// Deterministic terrain over a vertical band of chunks. Every sample is taken
// from world coordinates, never chunk-local ones, so two neighbouring chunks
// always agree on the column at their shared boundary and the same seed
// reproduces the same world on any machine.
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

        // A table at baseSurfaceHeight, with the noise ignored: somewhere to
        // build on. Caves, ores and trees still obey their own switches.
        bool flat = false;

        // Relief varies how much of the continental amplitude a region gets,
        // which is what separates flat country from mountain range without
        // putting a step at any biome border.
        f32 reliefFrequency = 0.0035f;

        bool biomes = true;
        f32 biomeFrequency = 0.0045f;

        // Two ridged fields intersected: each one alone is a sheet, and only
        // where both ridges meet does a tunnel open.
        bool caves = true;
        f32 caveFrequency = 0.045f;
        f32 caveThreshold = 0.86f;
        // Blocks of ceiling kept under the surface so a tunnel does not open
        // the ground everywhere it passes near it.
        s32 caveCeiling = 2;

        bool trees = true;
        f32 treeDensity = 1.0f;

        bool ores = true;
    };

    VoxelTerrain(const BlockRegistry& blocks, u32 seed);
    VoxelTerrain(const BlockRegistry& blocks, u32 seed, const Settings& settings);

    // Fills the part of one chunk within the configured vertical world bounds.
    void generate(VoxelWorld& world, ChunkCoord coordinate) const;
    // The same, into a chunk the caller owns: a worker thread generates here
    // without touching the world's chunk map.
    void generateChunk(VoxelChunk& chunk) const;
    // False when the chunk sits entirely outside the vertical world bounds and
    // generating it would only produce air.
    bool intersects(ChunkCoord coordinate) const;

    s32 surfaceHeight(s32 x, s32 z) const;
    BiomeKind biomeAt(s32 x, s32 z) const;

    const Settings& settings() const { return mSettings; }

private:
    struct Column
    {
        s32 surface = 0;
        BiomeKind biome = BiomeKind::Plains;
        bool beach = false;
    };

    static u32 hash3(s32 x, s32 y, s32 z, u32 seed);
    static f32 hashUnit(s32 x, s32 y, s32 z, u32 seed);
    static f32 treeDensityFor(BiomeKind biome);

    Column columnAt(s32 x, s32 z) const;
    bool carved(s32 x, s32 y, s32 z) const;
    BlockId oreAt(s32 x, s32 y, s32 z, s32 depth) const;
    BlockId surfaceBlockFor(const Column& column) const;
    BlockId fillerBlockFor(const Column& column) const;
    void placeTrees(VoxelChunk& chunk) const;
    void placeTree(VoxelChunk& chunk, s32 worldX, s32 worldZ, const Column& column) const;

    Noise::Perlin mContinental;
    Noise::Perlin mDetail;
    Noise::Perlin mRelief;
    Noise::Perlin mCaveA;
    Noise::Perlin mCaveB;
    Settings mSettings;
    u32 mSeed = 0;

    BlockId mGrass = AirBlockId;
    BlockId mDirt = AirBlockId;
    BlockId mStone = AirBlockId;
    BlockId mSand = AirBlockId;
    BlockId mGravel = AirBlockId;
    BlockId mBedrock = AirBlockId;
    BlockId mWater = AirBlockId;
    BlockId mSnow = AirBlockId;
    BlockId mLog = AirBlockId;
    BlockId mLeaves = AirBlockId;
    BlockId mCoalOre = AirBlockId;
    BlockId mIronOre = AirBlockId;
    BlockId mGoldOre = AirBlockId;
    BlockId mDiamondOre = AirBlockId;
};

} // namespace Voxel
} // namespace Radion

#endif // RADION_VOXEL_TERRAIN_H
