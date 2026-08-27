#include "VoxelTerrain.h"

#include <algorithm>
#include <cmath>

namespace Radion
{
namespace Voxel
{
namespace
{
constexpr u32 ReliefSeedMix = 0x1B873593u;
constexpr u32 CaveSeedMixA = 0x85EBCA6Bu;
constexpr u32 CaveSeedMixB = 0xC2B2AE35u;
constexpr u32 BiomeSeedMix = 0x27D4EB2Fu;
constexpr u32 TreeSeedMix = 0x165667B1u;
constexpr u32 OreSeedMix = 0x9E3779B1u;

// Resolves through the registry so a project that never registered a block
// simply does without it, instead of writing InvalidBlockId into the world.
BlockId resolveBlock(const BlockRegistry& blocks, const char* name)
{
    const BlockId id = blocks.findId(name);
    return id == InvalidBlockId ? AirBlockId : id;
}
} // namespace

VoxelTerrain::VoxelTerrain(const BlockRegistry& blocks, u32 seed)
    : VoxelTerrain(blocks, seed, Settings())
{
}

VoxelTerrain::VoxelTerrain(const BlockRegistry& blocks, u32 seed, const Settings& settings)
    : mSettings(settings), mSeed(seed)
{
    mContinental.initialise(seed);
    mDetail.initialise(seed ^ 0x9E3779B9u);
    mRelief.initialise(seed ^ ReliefSeedMix);
    mCaveA.initialise(seed ^ CaveSeedMixA);
    mCaveB.initialise(seed ^ CaveSeedMixB);

    mGrass = resolveBlock(blocks, "grass");
    mDirt = resolveBlock(blocks, "dirt");
    mStone = resolveBlock(blocks, "stone");
    mSand = resolveBlock(blocks, "sand");
    mGravel = resolveBlock(blocks, "gravel");
    mBedrock = resolveBlock(blocks, "bedrock");
    mWater = resolveBlock(blocks, "water");
    mSnow = resolveBlock(blocks, "snow");
    mLog = resolveBlock(blocks, "log");
    mLeaves = resolveBlock(blocks, "leaves");
    mCoalOre = resolveBlock(blocks, "coal_ore");
    mIronOre = resolveBlock(blocks, "iron_ore");
    mGoldOre = resolveBlock(blocks, "gold_ore");
    mDiamondOre = resolveBlock(blocks, "diamond_ore");
}

u32 VoxelTerrain::hash3(s32 x, s32 y, s32 z, u32 seed)
{
    u32 value = seed;
    value ^= static_cast<u32>(x) * 0x8DA6B343u;
    value ^= static_cast<u32>(y) * 0xD8163841u;
    value ^= static_cast<u32>(z) * 0xCB1AB31Fu;
    value ^= value >> 15;
    value *= 0x2C1B3C6Du;
    value ^= value >> 12;
    value *= 0x297A2D39u;
    value ^= value >> 15;
    return value;
}

f32 VoxelTerrain::hashUnit(s32 x, s32 y, s32 z, u32 seed)
{
    return static_cast<f32>(hash3(x, y, z, seed) & 0xFFFFFFu) / 16777216.0f;
}

f32 VoxelTerrain::treeDensityFor(BiomeKind biome)
{
    switch (biome)
    {
    case BiomeKind::Forest:
        return 0.045f;
    case BiomeKind::Plains:
        return 0.006f;
    case BiomeKind::Snow:
        return 0.018f;
    case BiomeKind::Mountains:
        return 0.004f;
    case BiomeKind::Desert:
    default:
        return 0.0f;
    }
}

BiomeKind VoxelTerrain::biomeAt(s32 x, s32 z) const
{
    if (!mSettings.biomes)
        return BiomeKind::Plains;

    const Noise::Voronoi::Result cell =
        Noise::Voronoi::compute(static_cast<f32>(x) * mSettings.biomeFrequency,
                                static_cast<f32>(z) * mSettings.biomeFrequency,
                                static_cast<f32>(mSeed ^ BiomeSeedMix));
    const f32 id = cell.cellId - std::floor(cell.cellId);
    if (id < 0.28f)
        return BiomeKind::Plains;
    if (id < 0.56f)
        return BiomeKind::Forest;
    if (id < 0.72f)
        return BiomeKind::Desert;
    if (id < 0.88f)
        return BiomeKind::Mountains;
    return BiomeKind::Snow;
}

s32 VoxelTerrain::surfaceHeight(s32 x, s32 z) const
{
    if (mSettings.flat)
    {
        return static_cast<s32>(std::clamp(std::round(mSettings.baseSurfaceHeight),
                                           static_cast<f32>(mSettings.minSurfaceHeight),
                                           static_cast<f32>(mSettings.maxSurfaceHeight)));
    }

    const f32 continental =
        mContinental.compute(static_cast<f32>(x) * 0.016f, static_cast<f32>(z) * 0.016f, 0.0f, 3);
    const f32 detail =
        mDetail.compute(static_cast<f32>(x) * 0.06f, static_cast<f32>(z) * 0.06f, 0.0f, 2);
    const f32 relief = mRelief.compute(static_cast<f32>(x) * mSettings.reliefFrequency,
                                       static_cast<f32>(z) * mSettings.reliefFrequency, 0.0f);
    const f32 reliefScale = 0.45f + 1.1f * std::clamp(relief * 0.5f + 0.5f, 0.0f, 1.0f);
    const f32 height = mSettings.baseSurfaceHeight +
                       continental * mSettings.continentalAmplitude * reliefScale +
                       detail * mSettings.detailAmplitude;
    return static_cast<s32>(std::clamp(std::round(height),
                                       static_cast<f32>(mSettings.minSurfaceHeight),
                                       static_cast<f32>(mSettings.maxSurfaceHeight)));
}

VoxelTerrain::Column VoxelTerrain::columnAt(s32 x, s32 z) const
{
    Column column;
    column.surface = surfaceHeight(x, z);
    column.biome = biomeAt(x, z);
    column.beach = column.surface <= mSettings.waterLevel + 1;
    return column;
}

BlockId VoxelTerrain::surfaceBlockFor(const Column& column) const
{
    if (column.beach)
        return mSand;
    switch (column.biome)
    {
    case BiomeKind::Desert:
        return mSand;
    case BiomeKind::Snow:
        return mSnow != AirBlockId ? mSnow : mGrass;
    case BiomeKind::Plains:
    case BiomeKind::Forest:
    case BiomeKind::Mountains:
    default:
        return mGrass;
    }
}

BlockId VoxelTerrain::fillerBlockFor(const Column& column) const
{
    if (column.beach)
        return mSand;
    switch (column.biome)
    {
    case BiomeKind::Desert:
        return mSand;
    case BiomeKind::Mountains:
        return mGravel != AirBlockId ? mGravel : mDirt;
    case BiomeKind::Plains:
    case BiomeKind::Forest:
    case BiomeKind::Snow:
    default:
        return mDirt;
    }
}

bool VoxelTerrain::carved(s32 x, s32 y, s32 z) const
{
    const f32 frequency = mSettings.caveFrequency;
    const f32 sampleX = static_cast<f32>(x) * frequency;
    const f32 sampleY = static_cast<f32>(y) * frequency * 1.8f;
    const f32 sampleZ = static_cast<f32>(z) * frequency;
    const f32 ridgeA = 1.0f - std::abs(mCaveA.compute(sampleX, sampleY, sampleZ));
    if (ridgeA <= mSettings.caveThreshold)
        return false;
    const f32 ridgeB = 1.0f - std::abs(mCaveB.compute(sampleX, sampleY, sampleZ));
    return ridgeB > mSettings.caveThreshold;
}

BlockId VoxelTerrain::oreAt(s32 x, s32 y, s32 z, s32 depth) const
{
    // Hashed on halved coordinates, so a hit spreads over a 2x2x2 pocket
    // instead of a single lonely block.
    const f32 value = hashUnit(x >> 1, y >> 1, z >> 1, mSeed ^ OreSeedMix);
    if (mDiamondOre != AirBlockId && value < 0.0016f && y <= mSettings.minWorldY + 16)
        return mDiamondOre;
    if (mGoldOre != AirBlockId && value < 0.0042f && y <= mSettings.minWorldY + 32)
        return mGoldOre;
    if (mIronOre != AirBlockId && value < 0.0125f && depth > 12)
        return mIronOre;
    if (mCoalOre != AirBlockId && value < 0.0260f && depth > 5)
        return mCoalOre;
    return AirBlockId;
}

bool VoxelTerrain::intersects(ChunkCoord coordinate) const
{
    const s32 originY = coordinate.y * VoxelChunk::Size;
    if (originY > mSettings.maxWorldY || originY + VoxelChunk::Size - 1 < mSettings.minWorldY)
        return false;

    // Surface heights are clamped and water never rises past its level, so a
    // chunk that starts above both can only ever be air - except for the trees
    // that stand on the highest ground.
    const s32 highest = std::max(mSettings.maxSurfaceHeight, mSettings.waterLevel) + 12;
    return originY <= highest;
}

void VoxelTerrain::generate(VoxelWorld& world, ChunkCoord coordinate) const
{
    if (!intersects(coordinate))
        return;

    generateChunk(world.ensureChunk(coordinate));
}

void VoxelTerrain::generateChunk(VoxelChunk& chunk) const
{
    const ChunkCoord coordinate = chunk.coordinate();
    const s32 originY = coordinate.y * VoxelChunk::Size;
    const s32 originX = coordinate.x * VoxelChunk::Size;
    const s32 originZ = coordinate.z * VoxelChunk::Size;

    for (s32 z = 0; z < VoxelChunk::Size; ++z)
    {
        for (s32 x = 0; x < VoxelChunk::Size; ++x)
        {
            const s32 worldX = originX + x;
            const s32 worldZ = originZ + z;
            const Column column = columnAt(worldX, worldZ);
            const BlockId surfaceBlock = surfaceBlockFor(column);
            const BlockId fillerBlock = fillerBlockFor(column);

            for (s32 y = 0; y < VoxelChunk::Size; ++y)
            {
                const s32 worldY = originY + y;
                if (worldY < mSettings.minWorldY || worldY > mSettings.maxWorldY)
                    continue;

                BlockId block = AirBlockId;
                if (worldY == mSettings.minWorldY)
                    block = mBedrock;
                else if (worldY < column.surface - 3)
                    block = mStone;
                else if (worldY < column.surface)
                    block = fillerBlock;
                else if (worldY == column.surface)
                    block = surfaceBlock;
                else if (worldY <= mSettings.waterLevel)
                    block = mWater;

                if (block == mStone && mSettings.ores)
                {
                    const BlockId ore = oreAt(worldX, worldY, worldZ, column.surface - worldY);
                    if (ore != AirBlockId)
                        block = ore;
                }

                if (mSettings.caves && block != AirBlockId && block != mBedrock &&
                    block != mWater && worldY <= column.surface - mSettings.caveCeiling &&
                    !(column.beach && worldY > column.surface - 4) &&
                    carved(worldX, worldY, worldZ))
                {
                    block = AirBlockId;
                }

                chunk.setBlock({x, y, z}, block);
            }
        }
    }

    placeTrees(chunk);
    chunk.compact();
}

void VoxelTerrain::placeTrees(VoxelChunk& chunk) const
{
    if (!mSettings.trees || mLog == AirBlockId || mLeaves == AirBlockId)
        return;

    const ChunkCoord coordinate = chunk.coordinate();
    const s32 originX = coordinate.x * VoxelChunk::Size;
    const s32 originZ = coordinate.z * VoxelChunk::Size;
    // Trunks outside the chunk still drop leaves inside it, so the search runs
    // over a margin and every write is clipped. No chunk ever writes into
    // another, which is what keeps trees identical whatever order chunks
    // arrive in.
    constexpr s32 Margin = 3;

    for (s32 z = -Margin; z < VoxelChunk::Size + Margin; ++z)
    {
        for (s32 x = -Margin; x < VoxelChunk::Size + Margin; ++x)
        {
            const s32 worldX = originX + x;
            const s32 worldZ = originZ + z;
            const Column column = columnAt(worldX, worldZ);
            if (column.beach || column.surface <= mSettings.waterLevel)
                continue;

            const f32 density = treeDensityFor(column.biome) * mSettings.treeDensity;
            if (density <= 0.0f)
                continue;
            if (hashUnit(worldX, 0, worldZ, mSeed ^ TreeSeedMix) >= density)
                continue;
            if (surfaceBlockFor(column) == mSand)
                continue;

            placeTree(chunk, worldX, worldZ, column);
        }
    }
}

void VoxelTerrain::placeTree(VoxelChunk& chunk, s32 worldX, s32 worldZ, const Column& column) const
{
    const ChunkCoord coordinate = chunk.coordinate();
    const s32 originX = coordinate.x * VoxelChunk::Size;
    const s32 originY = coordinate.y * VoxelChunk::Size;
    const s32 originZ = coordinate.z * VoxelChunk::Size;
    const u32 shape = hash3(worldX, 1, worldZ, mSeed ^ TreeSeedMix);
    const s32 trunkHeight = 4 + static_cast<s32>(shape % 3u);
    const s32 crownBase = column.surface + trunkHeight - 2;
    const s32 crownTop = column.surface + trunkHeight + 1;

    for (s32 worldY = crownBase; worldY <= crownTop; ++worldY)
    {
        const s32 radius = worldY >= crownTop - 1 ? 1 : 2;
        for (s32 offsetZ = -radius; offsetZ <= radius; ++offsetZ)
        {
            for (s32 offsetX = -radius; offsetX <= radius; ++offsetX)
            {
                if (std::abs(offsetX) == radius && std::abs(offsetZ) == radius && radius > 1)
                    continue;
                const VoxelCoord local = {worldX + offsetX - originX, worldY - originY,
                                          worldZ + offsetZ - originZ};
                if (!VoxelChunk::isLocal(local) || chunk.block(local) != AirBlockId)
                    continue;
                chunk.setBlock(local, mLeaves);
            }
        }
    }

    for (s32 worldY = column.surface + 1; worldY <= column.surface + trunkHeight; ++worldY)
    {
        const VoxelCoord local = {worldX - originX, worldY - originY, worldZ - originZ};
        if (VoxelChunk::isLocal(local))
            chunk.setBlock(local, mLog);
    }
}

} // namespace Voxel
} // namespace Radion
