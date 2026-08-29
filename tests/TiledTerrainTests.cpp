// TiledTerrainTests.cpp - Radion::TiledTerrain patch/UV/wrap math, verified
// without a live GPU device: patch count from tilemap dimensions, the
// rebuild counter after an edit, the tile-atlas UV rectangle, and the
// wrap-around helper a patch overhanging the map edge samples through.

#include "PCH.h"

#include "GameObject.h"
#include "Scene.h"
#include "TiledTerrain.h"

#include <cstdio>

using namespace Radion;

namespace
{

int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (condition)
        return;
    std::fprintf(stderr, "TiledTerrainTests:%d: failed: %s\n", line, expression);
    ++gFailures;
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool near(f32 a, f32 b, f32 epsilon = 1e-5f)
{
    return std::abs(a - b) <= epsilon;
}

bool near(const glm::vec2& a, const glm::vec2& b, f32 epsilon = 1e-5f)
{
    return near(a.x, b.x, epsilon) && near(a.y, b.y, epsilon);
}

void testLoadTilemapProducesExpectedPatchCount()
{
    Scene scene;
    GameObject* object = scene.createGameObject("terrain");
    TiledTerrain* terrain = object->addComponent<TiledTerrain>();
    CHECK(terrain != nullptr);

    terrain->setTilesPerPatch(2);
    const u32 width = 5, height = 4;
    std::vector<u8> map(static_cast<usize>(width) * height, 0);
    terrain->loadTilemap(width, height, map.data());

    // ceil(5/2) x ceil(4/2) = 3 x 2 patches.
    CHECK(terrain->patchCount() == 6);
    CHECK(terrain->mapWidth() == width);
    CHECK(terrain->mapHeight() == height);

    // No live GPU device in this test binary - the rebuild must stay data-
    // only and never abort trying to reach one.
    CHECK(!terrain->mesh().valid());
}

void testSetTileTriggersRebuildAndUpdatesCell()
{
    Scene scene;
    GameObject* object = scene.createGameObject("terrain");
    TiledTerrain* terrain = object->addComponent<TiledTerrain>();

    const u32 width = 3, height = 3;
    std::vector<u8> map(static_cast<usize>(width) * height, 0);
    terrain->loadTilemap(width, height, map.data());

    const u64 revisionAfterLoad = terrain->revision();
    CHECK(terrain->tile(1, 2) == 0);

    terrain->setTile(1, 2, 7);
    CHECK(terrain->revision() == revisionAfterLoad + 1);
    CHECK(terrain->tile(1, 2) == 7);
    // 3x3 tiles, 1 tile per patch (the default) -> 9 patches, unaffected by
    // repainting a single cell.
    CHECK(terrain->patchCount() == 9);
}

void testAtlasUVMatchesExpectedCell()
{
    const int tilesInSide = 4;
    glm::vec2 uvMin, uvMax;

    TiledTerrain::atlasUV(0, tilesInSide, uvMin, uvMax);
    CHECK(near(uvMin, glm::vec2(0.0f, 0.0f)));
    CHECK(near(uvMax, glm::vec2(0.25f, 0.25f)));

    TiledTerrain::atlasUV(static_cast<u8>(tilesInSide * tilesInSide - 1), tilesInSide, uvMin,
                          uvMax);
    CHECK(near(uvMin, glm::vec2(0.75f, 0.75f)));
    CHECK(near(uvMax, glm::vec2(1.0f, 1.0f)));

    TiledTerrain::atlasUV(5, tilesInSide, uvMin, uvMax); // row 1, col 1
    CHECK(near(uvMin, glm::vec2(0.25f, 0.25f)));
    CHECK(near(uvMax, glm::vec2(0.5f, 0.5f)));
}

void testWrappedTileWrapsAroundMapEdge()
{
    const u32 width = 3, height = 2;
    const u8 map[6] = {10, 11, 12, 20, 21, 22};

    CHECK(TiledTerrain::wrappedTile(map, width, height, 0, 0, 99) == 10);
    CHECK(TiledTerrain::wrappedTile(map, width, height, 3, 0, 99) == 10); // wraps x=3 -> 0
    CHECK(TiledTerrain::wrappedTile(map, width, height, -1, 0, 99) == 12); // wraps x=-1 -> 2
    CHECK(TiledTerrain::wrappedTile(map, width, height, 0, 2, 99) == 10); // wraps z=2 -> 0
    CHECK(TiledTerrain::wrappedTile(map, width, height, 0, -1, 99) == 20); // wraps z=-1 -> 1
    CHECK(TiledTerrain::wrappedTile(nullptr, width, height, 0, 0, 99) == 99);
}

} // namespace

int main()
{
    testLoadTilemapProducesExpectedPatchCount();
    testSetTileTriggersRebuildAndUpdatesCell();
    testAtlasUVMatchesExpectedCell();
    testWrappedTileWrapsAroundMapEdge();

    if (gFailures)
        std::fprintf(stderr, "%d tiled terrain test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
