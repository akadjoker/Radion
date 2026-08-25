#include "PCH.h"

#include "VoxelBlock.h"
#include "VoxelMesher.h"
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
    const VoxelMeshData oneBlock = VoxelMesher::buildChunk(world, *world.findChunk({0, 0, 0}), registry);
    CHECK(oneBlock.opaque.positions.size() == 24);
    CHECK(oneBlock.opaque.indices.size() == 36);
    CHECK(oneBlock.opaque.triangleCount() == 12);

    world.setBlock({1, 0, 0}, stoneId);
    const VoxelMeshData adjacent = VoxelMesher::buildChunk(world, *world.findChunk({0, 0, 0}), registry);
    CHECK(adjacent.opaque.indices.size() == 60);
    CHECK(adjacent.opaque.triangleCount() == 20);

    world.setBlock({32, 0, 0}, stoneId);
    const VoxelMeshData border = VoxelMesher::buildChunk(world, *world.findChunk({0, 0, 0}), registry);
    CHECK(border.opaque.indices.size() == 60);

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
    CHECK(waterMesh.transparent.indices.size() == 60);
}
} // namespace

int main()
{
    testRegistry();
    testWorldCoordinates();
    testChunkAndBoundaries();
    testMeshing();
    return gFailures == 0 ? 0 : 1;
}
