#include "PCH.h"

#include "VoxelBlock.h"
#include "VoxelMesher.h"
#include "VoxelTerrain.h"
#include "VoxelWorld.h"

#include <cstdio>

using namespace Radion;
using namespace Radion::Voxel;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (condition)
        return;
    std::fprintf(stderr, "VoxelTests:%d: failed: %s\n", line, expression);
    ++gFailures;
}

#define CHECK(expression) check((expression), #expression, __LINE__)

void testRegistry()
{
    BlockRegistry registry;
    CHECK(registry.size() == 1);
    CHECK(registry.findId("air") == AirBlockId);
    CHECK(!registry.air().solid);

    BlockDefinition stone;
    stone.name = "stone";
    const BlockId stoneId = registry.registerBlock(stone);
    CHECK(stoneId == 1);
    CHECK(registry.find(stoneId) != nullptr);
    CHECK(registry.find(stoneId)->name == "stone");
    CHECK(registry.findId("stone") == stoneId);
    CHECK(registry.registerBlock(stone) == InvalidBlockId);
    CHECK(registry.find(99) == nullptr);
}

void testWorldCoordinates()
{
    CHECK((VoxelWorld::chunkFor({0, 0, 0}) == ChunkCoord{0, 0, 0}));
    CHECK((VoxelWorld::chunkFor({31, 31, 31}) == ChunkCoord{0, 0, 0}));
    CHECK((VoxelWorld::chunkFor({32, 32, 32}) == ChunkCoord{1, 1, 1}));
    CHECK((VoxelWorld::chunkFor({-1, -1, -1}) == ChunkCoord{-1, -1, -1}));
    CHECK((VoxelWorld::chunkFor({-32, -32, -32}) == ChunkCoord{-1, -1, -1}));
    CHECK((VoxelWorld::chunkFor({-33, -33, -33}) == ChunkCoord{-2, -2, -2}));
    CHECK((VoxelWorld::localFor({-1, -1, -1}) == VoxelCoord{31, 31, 31}));
    CHECK((VoxelWorld::localFor({-32, -32, -32}) == VoxelCoord{0, 0, 0}));
    CHECK((VoxelWorld::worldFor({-1, 2, 3}, {31, 0, 5}) == VoxelCoord{-1, 64, 101}));
}

void testChunkAndBoundaries()
{
    VoxelWorld world;
    CHECK(world.block({4, 5, 6}) == AirBlockId);
    CHECK(world.setBlock({31, 4, 4}, 7));
    CHECK(world.chunkCount() == 1);
    CHECK(world.block({31, 4, 4}) == 7);
    CHECK(!world.setBlock({31, 4, 4}, 7));

    VoxelChunk& neighbour = world.ensureChunk({1, 0, 0});
    neighbour.clearDirty();
    CHECK(world.setBlock({31, 4, 4}, 8));
    CHECK(neighbour.dirty());

    const VoxelChunk* negativeChunk = world.findChunk({-1, 0, 0});
    CHECK(negativeChunk == nullptr);
    CHECK(world.setBlock({-1, 0, 0}, 3));
    CHECK(world.findChunk({-1, 0, 0}) != nullptr);
    CHECK(world.block({-1, 0, 0}) == 3);
}

void testMeshing()
{
    BlockRegistry registry;
    BlockDefinition stone;
    stone.name = "stone";
    const BlockId stoneId = registry.registerBlock(stone);

    VoxelWorld world;
    world.setBlock({0, 0, 0}, stoneId);
    const VoxelMeshData oneBlock =
        VoxelMesher::buildChunk(world, *world.findChunk({0, 0, 0}), registry);
    CHECK(oneBlock.opaque.positions.size() == 24);
    CHECK(oneBlock.opaque.indices.size() == 36);
    CHECK(oneBlock.opaque.triangleCount() == 12);

    world.setBlock({1, 0, 0}, stoneId);
    const VoxelMeshData adjacent =
        VoxelMesher::buildChunk(world, *world.findChunk({0, 0, 0}), registry);
    CHECK(adjacent.opaque.indices.size() == 36);
    CHECK(adjacent.opaque.triangleCount() == 12);
    bool hasRepeatedUv = false;
    for (const Math::vec2& uv : adjacent.opaque.uvs)
        hasRepeatedUv = hasRepeatedUv || uv.x == 2.0f || uv.y == 2.0f;
    CHECK(hasRepeatedUv);

    world.setBlock({32, 0, 0}, stoneId);
    const VoxelMeshData border =
        VoxelMesher::buildChunk(world, *world.findChunk({0, 0, 0}), registry);
    CHECK(border.opaque.indices.size() == 36);

    VoxelWorld solidWorld;
    VoxelChunk& solidChunk = solidWorld.ensureChunk({0, 0, 0});
    solidChunk.fill(stoneId);
    const VoxelMeshData solidMesh = VoxelMesher::buildChunk(solidWorld, solidChunk, registry);
    CHECK(solidMesh.opaque.positions.size() == 24);
    CHECK(solidMesh.opaque.indices.size() == 36);
    CHECK(solidMesh.opaque.triangleCount() == 12);

    BlockDefinition water;
    water.name = "water";
    water.solid = false;
    water.transparent = true;
    water.blocksLight = false;
    water.renderType = BlockRenderType::Transparent;
    const BlockId waterId = registry.registerBlock(water);
    VoxelWorld transparentWorld;
    transparentWorld.setBlock({0, 0, 0}, waterId);
    transparentWorld.setBlock({1, 0, 0}, waterId);
    const VoxelMeshData waterMesh =
        VoxelMesher::buildChunk(transparentWorld, *transparentWorld.findChunk({0, 0, 0}), registry);
    CHECK(waterMesh.opaque.indices.empty());
    CHECK(waterMesh.transparent.indices.size() == 36);
}

void testAtlasUvs()
{
    BlockRegistry registry;
    BlockDefinition block;
    block.name = "tile";
    for (BlockFaceMaterial& face : block.faces)
    {
        face.atlasX = 1;
        face.atlasY = 1;
    }
    const BlockId id = registry.registerBlock(block);

    VoxelWorld world;
    world.setBlock({0, 0, 0}, id);

    VoxelMesher::Settings settings;
    settings.atlasColumns = 4;
    settings.atlasRows = 2;

    const VoxelMeshData mesh =
        VoxelMesher::buildChunk(world, *world.findChunk({0, 0, 0}), registry, settings);

    // UV0 repeats per voxel; UV1 identifies tile (1,1) in the engine atlas convention.
    CHECK(mesh.opaque.uvs.size() == 24);
    CHECK(mesh.opaque.uvs2.size() == 24);
    for (const Math::vec2& uv : mesh.opaque.uvs2)
    {
        CHECK(uv.x == 0.25f);
        CHECK(uv.y == 0.5f);
    }
}

void testAtlasRotation()
{
    BlockRegistry registry;
    BlockDefinition block;
    block.name = "rotated";
    for (BlockFaceMaterial& face : block.faces)
        face.rotation = BlockFaceRotation::Clockwise90;
    const BlockId id = registry.registerBlock(block);

    VoxelWorld world;
    world.setBlock({0, 0, 0}, id);
    const VoxelMeshData mesh =
        VoxelMesher::buildChunk(world, *world.findChunk({0, 0, 0}), registry);

    bool hasClockwiseCorner = false;
    for (const Math::vec2& uv : mesh.opaque.uvs)
        hasClockwiseCorner = hasClockwiseCorner || (uv.x == 0.0f && uv.y == 1.0f);
    CHECK(hasClockwiseCorner);
}

void testAtlasVerticalFlip()
{
    BlockRegistry registry;
    BlockDefinition block;
    block.name = "flipped";
    for (BlockFaceMaterial& face : block.faces)
        face.flipVertical = true;
    const BlockId id = registry.registerBlock(block);

    VoxelWorld world;
    world.setBlock({0, 0, 0}, id);
    const VoxelMeshData mesh =
        VoxelMesher::buildChunk(world, *world.findChunk({0, 0, 0}), registry);

    bool hasFlippedCorner = false;
    for (const Math::vec2& uv : mesh.opaque.uvs)
        hasFlippedCorner = hasFlippedCorner || (uv.x == 0.0f && uv.y == 1.0f);
    CHECK(hasFlippedCorner);
}

BlockRegistry makeTerrainRegistry()
{
    BlockRegistry registry;
    for (const char* name : {"grass", "dirt", "stone", "sand", "bedrock", "water"})
    {
        BlockDefinition definition;
        definition.name = name;
        registry.registerBlock(definition);
    }
    return registry;
}

void testTerrainGeneration()
{
    const BlockRegistry registry = makeTerrainRegistry();

    VoxelWorld worldA;
    VoxelWorld worldB;
    VoxelTerrain terrainA(registry, 12345);
    VoxelTerrain terrainB(registry, 12345);
    terrainA.generate(worldA, {0, 0, 0});
    terrainB.generate(worldB, {0, 0, 0});

    const VoxelChunk& chunkA = *worldA.findChunk({0, 0, 0});
    const VoxelChunk& chunkB = *worldB.findChunk({0, 0, 0});

    // Same seed reproduces the same chunk, block for block.
    bool identical = true;
    for (s32 z = 0; z < VoxelChunk::Size && identical; ++z)
        for (s32 y = 0; y < VoxelChunk::Size && identical; ++y)
            for (s32 x = 0; x < VoxelChunk::Size && identical; ++x)
                identical = chunkA.block({x, y, z}) == chunkB.block({x, y, z});
    CHECK(identical);

    // A different seed differs somewhere.
    VoxelWorld worldC;
    VoxelTerrain terrainC(registry, 54321);
    terrainC.generate(worldC, {0, 0, 0});
    const VoxelChunk& chunkC = *worldC.findChunk({0, 0, 0});
    bool differs = false;
    for (s32 z = 0; z < VoxelChunk::Size && !differs; ++z)
        for (s32 y = 0; y < VoxelChunk::Size && !differs; ++y)
            for (s32 x = 0; x < VoxelChunk::Size && !differs; ++x)
                differs = chunkA.block({x, y, z}) != chunkC.block({x, y, z});
    CHECK(differs);

    // Bedrock floor and a solid surface column exist regardless of seed.
    CHECK(chunkA.block({0, 0, 0}) == registry.findId("bedrock"));
    const BlockId grass = registry.findId("grass");
    const BlockId sand = registry.findId("sand");
    bool hasSurface = false;
    for (s32 z = 0; z < VoxelChunk::Size && !hasSurface; ++z)
        for (s32 x = 0; x < VoxelChunk::Size && !hasSurface; ++x)
            for (s32 y = 1; y < VoxelChunk::Size; ++y)
                if (chunkA.block({x, y, z}) == grass || chunkA.block({x, y, z}) == sand)
                    hasSurface = true;
    CHECK(hasSurface);
}

void testTallTerrainGeneration()
{
    const BlockRegistry registry = makeTerrainRegistry();
    VoxelTerrain::Settings settings;
    settings.minWorldY = -64;
    settings.maxWorldY = 127;
    settings.minSurfaceHeight = 4;
    settings.maxSurfaceHeight = 96;
    settings.baseSurfaceHeight = 48.0f;
    settings.continentalAmplitude = 28.0f;
    settings.detailAmplitude = 8.0f;

    VoxelWorld world;
    VoxelTerrain terrain(registry, 12345, settings);
    terrain.generate(world, {0, -2, 0});
    terrain.generate(world, {0, 0, 0});
    terrain.generate(world, {0, 3, 0});
    terrain.generate(world, {0, -3, 0});

    CHECK(world.findChunk({0, -2, 0}) != nullptr);
    CHECK(world.findChunk({0, 0, 0}) != nullptr);
    CHECK(world.findChunk({0, 3, 0}) != nullptr);
    CHECK(world.findChunk({0, -3, 0}) == nullptr);
    CHECK(world.block({0, -64, 0}) == registry.findId("bedrock"));
}

void testTallTerrainUsesWorldHeight()
{
    const BlockRegistry registry = makeTerrainRegistry();
    VoxelTerrain::Settings settings;
    settings.minWorldY = -32;
    settings.maxWorldY = 95;
    settings.minSurfaceHeight = 40;
    settings.maxSurfaceHeight = 40;
    settings.baseSurfaceHeight = 40.0f;
    settings.continentalAmplitude = 0.0f;
    settings.detailAmplitude = 0.0f;

    VoxelWorld world;
    VoxelTerrain terrain(registry, 12345, settings);
    terrain.generate(world, {0, 1, 0});
    terrain.generate(world, {0, 2, 0});

    CHECK(world.block({0, 40, 0}) == registry.findId("grass"));
    CHECK(world.block({0, 72, 0}) == AirBlockId);
}
} // namespace

int main()
{
    testRegistry();
    testWorldCoordinates();
    testChunkAndBoundaries();
    testMeshing();
    testAtlasUvs();
    testAtlasRotation();
    testAtlasVerticalFlip();
    testTerrainGeneration();
    testTallTerrainGeneration();
    testTallTerrainUsesWorldHeight();
    return gFailures == 0 ? 0 : 1;
}
