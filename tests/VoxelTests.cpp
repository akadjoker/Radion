#include "PCH.h"

#include "VoxelBlock.h"
#include "VoxelEditHistory.h"
#include "VoxelEditStore.h"
#include "VoxelMesher.h"
#include "VoxelNeighbourhood.h"
#include "VoxelRaycast.h"
#include "VoxelStreamer.h"
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

// The mesher packs a voxel vertex into MeshData::colors and voxel.vert unpacks
// it. These mirror that layout: a test that reads the fields any other way
// would stop noticing the day the two sides disagree.
struct PackedVertex
{
    u32 face = 0;
    u32 occlusion = 0;
    u32 atlasX = 0;
    u32 atlasY = 0;
    f32 u = 0.0f;
    f32 v = 0.0f;
};

PackedVertex unpackVertex(u32 packed)
{
    PackedVertex vertex;
    vertex.face = packed & 0x7u;
    vertex.occlusion = (packed >> 3) & 0x3u;
    vertex.atlasX = (packed >> 5) & 0x1Fu;
    vertex.atlasY = (packed >> 10) & 0x1Fu;
    vertex.u = static_cast<f32>((packed >> 15) & 0x3Fu);
    vertex.v = static_cast<f32>((packed >> 21) & 0x3Fu);
    return vertex;
}

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
    CHECK(oneBlock.mesh.positions.size() == 24);
    CHECK(oneBlock.mesh.indices.size() == 36);
    CHECK(oneBlock.mesh.triangleCount() == 12);

    world.setBlock({1, 0, 0}, stoneId);
    const VoxelMeshData adjacent =
        VoxelMesher::buildChunk(world, *world.findChunk({0, 0, 0}), registry);
    CHECK(adjacent.mesh.indices.size() == 36);
    CHECK(adjacent.mesh.triangleCount() == 12);
    bool hasRepeatedUv = false;
    for (u32 packed : adjacent.mesh.colors)
    {
        const PackedVertex vertex = unpackVertex(packed);
        hasRepeatedUv = hasRepeatedUv || vertex.u == 2.0f || vertex.v == 2.0f;
    }
    CHECK(hasRepeatedUv);
    CHECK(adjacent.mesh.colors.size() == adjacent.mesh.positions.size());

    world.setBlock({32, 0, 0}, stoneId);
    const VoxelMeshData border =
        VoxelMesher::buildChunk(world, *world.findChunk({0, 0, 0}), registry);
    CHECK(border.mesh.indices.size() == 36);

    VoxelWorld solidWorld;
    VoxelChunk& solidChunk = solidWorld.ensureChunk({0, 0, 0});
    solidChunk.fill(stoneId);
    const VoxelMeshData solidMesh = VoxelMesher::buildChunk(solidWorld, solidChunk, registry);
    CHECK(solidMesh.mesh.positions.size() == 24);
    CHECK(solidMesh.mesh.indices.size() == 36);
    CHECK(solidMesh.mesh.triangleCount() == 12);

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
    CHECK(!waterMesh.hasSlot(VoxelMeshData::OpaqueSlot));
    CHECK(waterMesh.hasSlot(VoxelMeshData::TransparentSlot));
    CHECK(waterMesh.mesh.indices.size() == 36);
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

    // The tile travels as a column and a row, not as a UV: voxel.vert turns
    // them into the atlas origin with the material's own tile size, so the
    // atlas may be resized without remeshing the world.
    CHECK(mesh.mesh.colors.size() == 24);
    for (u32 packed : mesh.mesh.colors)
    {
        const PackedVertex vertex = unpackVertex(packed);
        CHECK(vertex.atlasX == 1);
        CHECK(vertex.atlasY == 1);
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
    for (u32 packed : mesh.mesh.colors)
    {
        const PackedVertex vertex = unpackVertex(packed);
        hasClockwiseCorner = hasClockwiseCorner || (vertex.u == 0.0f && vertex.v == 1.0f);
    }
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
    for (u32 packed : mesh.mesh.colors)
    {
        const PackedVertex vertex = unpackVertex(packed);
        hasFlippedCorner = hasFlippedCorner || (vertex.u == 0.0f && vertex.v == 1.0f);
    }
    CHECK(hasFlippedCorner);
}

void testSideFacesStandUpright()
{
    BlockRegistry registry;
    BlockDefinition block;
    block.name = "tile";
    const BlockId id = registry.registerBlock(block);

    VoxelWorld world;
    // A column two blocks tall, so the greedy sweep merges a quad whose two
    // extents differ and a swapped texture basis cannot hide behind a square.
    world.setBlock({0, 0, 0}, id);
    world.setBlock({0, 1, 0}, id);

    const VoxelMeshData mesh =
        VoxelMesher::buildChunk(world, *world.findChunk({0, 0, 0}), registry);

    // Texture v follows world Y on every side face, whichever axis the sweep
    // walked: a grass side or a log must never lie on its side.
    usize sideVertices = 0;
    bool upright = true;
    for (usize index = 0; index < mesh.mesh.positions.size(); ++index)
    {
        const PackedVertex vertex = unpackVertex(mesh.mesh.colors[index]);
        const bool vertical = vertex.face == static_cast<u32>(BlockFace::NegativeY) ||
                              vertex.face == static_cast<u32>(BlockFace::PositiveY);
        if (vertical)
            continue;
        ++sideVertices;
        upright = upright && vertex.v == mesh.mesh.positions[index].y;
    }
    CHECK(sideVertices == 16);
    CHECK(upright);
}

void testRaycast()
{
    BlockRegistry registry;
    BlockDefinition stone;
    stone.name = "stone";
    const BlockId stoneId = registry.registerBlock(stone);
    BlockDefinition water;
    water.name = "water";
    water.solid = false;
    water.transparent = true;
    const BlockId waterId = registry.registerBlock(water);

    VoxelWorld world;
    world.setBlock({3, 0, 0}, stoneId);
    VoxelRaycastHit hit;
    CHECK(raycast(world, registry, {0.5f, 0.5f, 0.5f}, {1.0f, 0.0f, 0.0f}, 10.0f, hit));
    CHECK((hit.block == VoxelCoord{3, 0, 0}));
    CHECK((hit.previousBlock == VoxelCoord{2, 0, 0}));
    CHECK(hit.face == BlockFace::NegativeX);
    CHECK(hit.distance == 2.5f);

    world.setBlock({1, 0, 0}, waterId);
    CHECK(raycast(world, registry, {0.5f, 0.5f, 0.5f}, {1.0f, 0.0f, 0.0f}, 10.0f, hit));
    CHECK((hit.block == VoxelCoord{3, 0, 0}));
    CHECK(hit.face == BlockFace::NegativeX);

    world.setBlock({-3, 0, 0}, stoneId);
    CHECK(raycast(world, registry, {-1.5f, 0.5f, 0.5f}, {-1.0f, 0.0f, 0.0f}, 10.0f, hit));
    CHECK((hit.block == VoxelCoord{-3, 0, 0}));
    CHECK((hit.previousBlock == VoxelCoord{-2, 0, 0}));
    CHECK(!raycast(world, registry, {0.5f, 2.5f, 0.5f}, {0.0f, 1.0f, 0.0f}, 1.0f, hit));
}

void testEditHistory()
{
    VoxelWorld world;
    VoxelEditHistory history;
    CHECK(history.setBlock(world, {2, 3, 4}, 7));
    CHECK(world.block({2, 3, 4}) == 7);
    CHECK(history.canUndo());
    CHECK(history.edits().size() == 1);
    CHECK(history.edits()[0].before == AirBlockId);
    CHECK(history.undo(world));
    CHECK(world.block({2, 3, 4}) == AirBlockId);
    CHECK(history.canRedo());
    CHECK(history.redo(world));
    CHECK(world.block({2, 3, 4}) == 7);

    CHECK(history.setBlock(world, {2, 3, 4}, 9));
    CHECK(!history.canRedo());
    CHECK(history.undo(world));
    CHECK(world.block({2, 3, 4}) == 7);
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

void testNeighbourhoodGather()
{
    const BlockRegistry registry = makeTerrainRegistry();
    const BlockId dirt = registry.findId("dirt");
    const BlockId stone = registry.findId("stone");

    VoxelWorld world;
    world.ensureChunk({0, 0, 0}).fill(dirt);
    world.setBlock({-1, 5, 5}, stone);

    VoxelNeighbourhood neighbourhood;
    neighbourhood.gather(world, {0, 0, 0});

    CHECK((neighbourhood.coordinate() == ChunkCoord{0, 0, 0}));
    CHECK(neighbourhood.block(0, 5, 5) == dirt);
    CHECK(neighbourhood.block(-1, 5, 5) == world.findChunk({-1, 0, 0})->block({31, 5, 5}));
    CHECK(neighbourhood.block(-1, 5, 5) == stone);
    // Chunk {-1,-1,-1} was never loaded, so its corner of the padded box stays air.
    CHECK(neighbourhood.block(-1, -1, -1) == AirBlockId);
}

void runUntilGenerationSettles(VoxelStreamer& streamer)
{
    for (int iteration = 0; iteration < 32 && (iteration == 0 || streamer.pendingGeneration() > 0);
         ++iteration)
    {
        streamer.update();
    }
}

void testStreamerLoadsAroundOrigin()
{
    const BlockRegistry registry = makeTerrainRegistry();

    VoxelStreamer streamer;
    VoxelStreamer::Settings settings;
    settings.viewRadius = 1;
    settings.useJobs = false;
    streamer.configure(settings);
    streamer.setBlocks(registry);
    streamer.setTerrain(12345, VoxelTerrain::Settings());
    streamer.setOrigin({0.0f, 0.0f, 0.0f});

    runUntilGenerationSettles(streamer);

    CHECK(streamer.pendingGeneration() == 0);
    CHECK(streamer.loadedChunks() == 25);
    CHECK(streamer.world().findChunk({3, 0, 0}) == nullptr);
}

void testStreamerUnloadsBehind()
{
    const BlockRegistry registry = makeTerrainRegistry();

    VoxelStreamer streamer;
    VoxelStreamer::Settings settings;
    settings.viewRadius = 1;
    settings.useJobs = false;
    streamer.configure(settings);
    streamer.setBlocks(registry);
    streamer.setTerrain(12345, VoxelTerrain::Settings());
    streamer.setOrigin({0.0f, 0.0f, 0.0f});
    runUntilGenerationSettles(streamer);
    CHECK(streamer.loadedChunks() == 25);

    streamer.setOrigin({6.0f * VoxelChunk::Size, 0.0f, 0.0f});
    streamer.update();

    usize unloadedCount = 0;
    ChunkCoord coordinate;
    while (streamer.popUnloadedChunk(coordinate))
        ++unloadedCount;

    CHECK(unloadedCount == 25);
    CHECK(streamer.world().findChunk({0, 0, 0}) == nullptr);
    CHECK(streamer.world().findChunk({2, 0, 0}) == nullptr);
}

void testStreamerEditsSurviveReload()
{
    const BlockRegistry registry = makeTerrainRegistry();
    const BlockId stone = registry.findId("stone");

    VoxelStreamer streamer;
    VoxelStreamer::Settings settings;
    settings.viewRadius = 1;
    settings.useJobs = false;
    streamer.configure(settings);
    streamer.setBlocks(registry);
    streamer.setTerrain(12345, VoxelTerrain::Settings());
    streamer.setOrigin({0.0f, 0.0f, 0.0f});
    runUntilGenerationSettles(streamer);
    CHECK(streamer.loadedChunks() == 25);

    const VoxelCoord edited = {5, 5, 5};
    const BlockId original = streamer.block(edited);
    const BlockId replacement = original == stone ? AirBlockId : stone;
    CHECK(streamer.setBlock(edited, replacement));
    CHECK(streamer.block(edited) == replacement);

    streamer.setOrigin({6.0f * VoxelChunk::Size, 0.0f, 0.0f});
    streamer.update();
    ChunkCoord coordinate;
    while (streamer.popUnloadedChunk(coordinate))
    {
    }
    CHECK(streamer.world().findChunk({0, 0, 0}) == nullptr);

    streamer.setOrigin({0.0f, 0.0f, 0.0f});
    runUntilGenerationSettles(streamer);

    CHECK(streamer.loadedChunks() == 25);
    CHECK(streamer.block(edited) == replacement);
}

void testEditStoreRoundTrip()
{
    VoxelEditStore store;
    store.record({5, 5, 5}, 7);
    store.record({40, 5, 5}, 9);
    store.record({-1, 5, 5}, 11);
    CHECK(store.recordCount() == 3);

    std::vector<u8> bytes;
    store.write(bytes);

    VoxelEditStore restored;
    CHECK(restored.read(bytes));
    CHECK(restored.recordCount() == store.recordCount());

    VoxelChunk originChunk({0, 0, 0});
    CHECK(restored.apply(originChunk));
    CHECK(originChunk.block({5, 5, 5}) == 7);

    VoxelChunk positiveChunk({1, 0, 0});
    CHECK(restored.apply(positiveChunk));
    CHECK(positiveChunk.block({8, 5, 5}) == 9);

    VoxelChunk negativeChunk({-1, 0, 0});
    CHECK(restored.apply(negativeChunk));
    CHECK(negativeChunk.block({31, 5, 5}) == 11);
}

void testBoundedWorld()
{
    const BlockRegistry registry = makeTerrainRegistry();

    VoxelStreamer streamer;
    VoxelStreamer::Settings settings;
    settings.viewRadius = 4;
    settings.useJobs = false;
    settings.bounded = true;
    settings.boundsMinX = -1;
    settings.boundsMaxX = 1;
    settings.boundsMinZ = -1;
    settings.boundsMaxZ = 1;
    streamer.configure(settings);
    streamer.setBlocks(registry);
    streamer.setTerrain(12345, VoxelTerrain::Settings());

    bool everOutsideBounds = false;
    usize maxLoaded = 0;
    std::vector<ChunkCoord> loaded;
    for (s32 step = 0; step < 20; ++step)
    {
        streamer.setOrigin({static_cast<f32>(step) * VoxelChunk::Size, 0.0f, 0.0f});
        runUntilGenerationSettles(streamer);

        streamer.world().collectCoordinates(loaded);
        maxLoaded = std::max(maxLoaded, loaded.size());
        for (const ChunkCoord& coordinate : loaded)
        {
            if (coordinate.x < settings.boundsMinX || coordinate.x > settings.boundsMaxX ||
                coordinate.z < settings.boundsMinZ || coordinate.z > settings.boundsMaxZ)
            {
                everOutsideBounds = true;
            }
        }
    }

    CHECK(!everOutsideBounds);
    CHECK(maxLoaded == 9);
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
    testSideFacesStandUpright();
    testRaycast();
    testEditHistory();
    testTerrainGeneration();
    testTallTerrainGeneration();
    testTallTerrainUsesWorldHeight();
    testNeighbourhoodGather();
    testStreamerLoadsAroundOrigin();
    testStreamerUnloadsBehind();
    testStreamerEditsSurviveReload();
    testEditStoreRoundTrip();
    testBoundedWorld();
    return gFailures == 0 ? 0 : 1;
}
