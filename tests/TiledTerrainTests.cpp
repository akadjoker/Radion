// TiledTerrainTests.cpp - Radion::TiledTerrain patch/UV/wrap math, verified
// without a live GPU device: patch count from tilemap dimensions, the
// rebuild counter after an edit, the tile-atlas UV rectangle, and the
// wrap-around helper a patch overhanging the map edge samples through.

#include "PCH.h"

#include "GameObject.h"
#include "Pixmap.h"
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

// Regression: the Inspector's own "Build Tilemap" defaults to an 8x8 map
// (InspectorPanel.cpp, drawTiledTerrainComponent) and the Create-menu popup
// matches it - with tilesPerPatch left at whatever a fresh component
// defaults to (never touched by either flow unless the user opens the
// tooltip and changes it), the two must agree on a single patch, not the
// sixty-four one-tile ones a default of 1 used to produce.
void testDefaultTilesPerPatchMatchesDefaultMapSize()
{
    Scene scene;
    GameObject* object = scene.createGameObject("terrain");
    TiledTerrain* terrain = object->addComponent<TiledTerrain>();
    CHECK(terrain != nullptr);

    const u32 width = 8, height = 8;
    std::vector<u8> map(static_cast<usize>(width) * height, 0);
    terrain->loadTilemap(width, height, map.data());

    CHECK(terrain->patchCount() == 1);
}

void testSetTileTriggersRebuildAndUpdatesCell()
{
    Scene scene;
    GameObject* object = scene.createGameObject("terrain");
    TiledTerrain* terrain = object->addComponent<TiledTerrain>();
    terrain->setTilesPerPatch(1);

    const u32 width = 3, height = 3;
    std::vector<u8> map(static_cast<usize>(width) * height, 0);
    terrain->loadTilemap(width, height, map.data());

    const u64 revisionAfterLoad = terrain->revision();
    CHECK(terrain->tile(1, 2) == 0);

    terrain->setTile(1, 2, 7);
    CHECK(terrain->revision() == revisionAfterLoad + 1);
    CHECK(terrain->tile(1, 2) == 7);
    // 3x3 tiles, 1 tile per patch -> 9 patches, unaffected by repainting a
    // single cell.
    CHECK(terrain->patchCount() == 9);
}

void testAtlasUVMatchesExpectedCell()
{
    // 2% of the cell, inset on every side - see atlasUV()'s own comment.
    const int tilesInSide = 4;
    const f32 step = 0.25f;
    const f32 inset = step * 0.02f;
    glm::vec2 uvMin, uvMax;

    TiledTerrain::atlasUV(0, tilesInSide, uvMin, uvMax);
    CHECK(near(uvMin, glm::vec2(inset, 0.75f + inset)));
    CHECK(near(uvMax, glm::vec2(step - inset, 1.0f - inset)));

    TiledTerrain::atlasUV(static_cast<u8>(tilesInSide * tilesInSide - 1), tilesInSide, uvMin,
                          uvMax);
    CHECK(near(uvMin, glm::vec2(0.75f + inset, inset)));
    CHECK(near(uvMax, glm::vec2(1.0f - inset, step - inset)));

    TiledTerrain::atlasUV(5, tilesInSide, uvMin, uvMax); // row 1, col 1
    CHECK(near(uvMin, glm::vec2(0.25f + inset, 0.5f + inset)));
    CHECK(near(uvMax, glm::vec2(0.5f - inset, 0.75f - inset)));

    // Bits 6-7 encode the rotation, not a different atlas cell.
    TiledTerrain::atlasUV(static_cast<u8>(5 | 0x80), tilesInSide, uvMin, uvMax);
    CHECK(near(uvMin, glm::vec2(0.25f + inset, 0.5f + inset)));
    CHECK(near(uvMax, glm::vec2(0.5f - inset, 0.75f - inset)));
}

void testAtlasUVsMatchEncodedRotations()
{
    const int tilesInSide = 8;
    const u8 atlasTile = 9;
    glm::vec2 uvMin, uvMax;
    TiledTerrain::atlasUV(atlasTile, tilesInSide, uvMin, uvMax);

    glm::vec2 bottomLeft, bottomRight, topLeft, topRight;
    TiledTerrain::atlasUVs(atlasTile, tilesInSide, bottomLeft, bottomRight, topLeft, topRight);
    CHECK(near(bottomLeft, glm::vec2(uvMin.x, uvMax.y)));
    CHECK(near(bottomRight, uvMax));
    CHECK(near(topLeft, uvMin));
    CHECK(near(topRight, glm::vec2(uvMax.x, uvMin.y)));

    TiledTerrain::atlasUVs(static_cast<u8>(atlasTile | 0x40), tilesInSide, bottomLeft,
                           bottomRight, topLeft, topRight);
    CHECK(near(bottomLeft, uvMax));
    CHECK(near(bottomRight, glm::vec2(uvMax.x, uvMin.y)));
    CHECK(near(topLeft, glm::vec2(uvMin.x, uvMax.y)));
    CHECK(near(topRight, uvMin));

    TiledTerrain::atlasUVs(static_cast<u8>(atlasTile | 0x80), tilesInSide, bottomLeft,
                           bottomRight, topLeft, topRight);
    CHECK(near(bottomLeft, glm::vec2(uvMax.x, uvMin.y)));
    CHECK(near(bottomRight, uvMin));
    CHECK(near(topLeft, uvMax));
    CHECK(near(topRight, glm::vec2(uvMin.x, uvMax.y)));

    TiledTerrain::atlasUVs(static_cast<u8>(atlasTile | 0xc0), tilesInSide, bottomLeft,
                           bottomRight, topLeft, topRight);
    CHECK(near(bottomLeft, uvMin));
    CHECK(near(bottomRight, glm::vec2(uvMin.x, uvMax.y)));
    CHECK(near(topLeft, glm::vec2(uvMax.x, uvMin.y)));
    CHECK(near(topRight, uvMax));
}

void testTilesFromImageMatchesApocalyxBottomFirstRows()
{
    Pixmap image(2, 2, 1);
    image.set_pixel(0, 0, 1, 1, 1, 255);
    image.set_pixel(1, 0, 2, 2, 2, 255);
    image.set_pixel(0, 1, 3, 3, 3, 255);
    image.set_pixel(1, 1, 4, 4, 4, 255);

    std::vector<u8> tiles;
    CHECK(TiledTerrain::tilesFromImageColors(image, tiles));
    CHECK(tiles.size() == 4);
    CHECK(tiles[0] == 3);
    CHECK(tiles[1] == 4);
    CHECK(tiles[2] == 1);
    CHECK(tiles[3] == 2);
}

void testEncodedFormatLimitsAtlasToEightCellsPerSide()
{
    Scene scene;
    GameObject* object = scene.createGameObject("terrain");
    TiledTerrain* terrain = object->addComponent<TiledTerrain>();
    CHECK(terrain != nullptr);
    if (!terrain)
        return;

    terrain->setTilesInSide(64);
    CHECK(terrain->tilesInSide() == 8);
    terrain->setTilesInSide(0);
    CHECK(terrain->tilesInSide() == 1);
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

void testAtlasMaterialRoundTripAndSizeWithNoGPU()
{
    Scene scene;
    GameObject* object = scene.createGameObject("terrain");
    TiledTerrain* terrain = object->addComponent<TiledTerrain>();
    CHECK(terrain != nullptr);
    if (!terrain)
        return;

    CHECK(terrain->atlasMaterial().empty());
    terrain->setAtlasMaterial("materials/tiles.material");
    CHECK(terrain->atlasMaterial() == "materials/tiles.material");

    // No live GPU device in this test binary - atlasSize() must fail closed,
    // not crash trying to reach one.
    u32 width = 0, height = 0;
    CHECK(!terrain->atlasSize(width, height));
    CHECK(width == 0);
    CHECK(height == 0);
}

void testFillCellsPaintsOnlyConnectedRegion()
{
    Scene scene;
    GameObject* object = scene.createGameObject("terrain");
    TiledTerrain* terrain = object->addComponent<TiledTerrain>();
    CHECK(terrain != nullptr);
    if (!terrain)
        return;

    const u32 width = 5, height = 5;
    std::vector<u8> map(static_cast<usize>(width) * height, 0);
    terrain->loadTilemap(width, height, map.data());

    // A plus-shaped region of tile 3 in a sea of tile 0, plus one isolated
    // tile-3 cell in the far corner - not 4-connected to the plus, so it
    // must survive a flood fill starting inside the plus untouched.
    terrain->setTile(2, 1, 3);
    terrain->setTile(1, 2, 3);
    terrain->setTile(2, 2, 3);
    terrain->setTile(3, 2, 3);
    terrain->setTile(2, 3, 3);
    terrain->setTile(4, 4, 3);

    TiledTerrain::fillCells(*terrain, 2, 2, 9);

    CHECK(terrain->tile(2, 1) == 9);
    CHECK(terrain->tile(1, 2) == 9);
    CHECK(terrain->tile(2, 2) == 9);
    CHECK(terrain->tile(3, 2) == 9);
    CHECK(terrain->tile(2, 3) == 9);
    CHECK(terrain->tile(4, 4) == 3); // disconnected island, untouched
    CHECK(terrain->tile(0, 0) == 0); // background, untouched
    CHECK(terrain->tile(2, 0) == 0); // touching the plus's boundary, untouched
}

void testPaintRectangleClampsToMapBounds()
{
    Scene scene;
    GameObject* object = scene.createGameObject("terrain");
    TiledTerrain* terrain = object->addComponent<TiledTerrain>();
    CHECK(terrain != nullptr);
    if (!terrain)
        return;

    const u32 width = 4, height = 4;
    std::vector<u8> map(static_cast<usize>(width) * height, 0);
    terrain->loadTilemap(width, height, map.data());

    // Rectangle from (2,2) to (10,10) - the far corner sits well outside the
    // 4x4 map, the clamp must cut it down to (2,2)-(3,3).
    TiledTerrain::paintRectangle(*terrain, 2, 2, 10, 10, 5);

    CHECK(terrain->tile(2, 2) == 5);
    CHECK(terrain->tile(3, 2) == 5);
    CHECK(terrain->tile(2, 3) == 5);
    CHECK(terrain->tile(3, 3) == 5);
    CHECK(terrain->tile(0, 0) == 0);
    CHECK(terrain->tile(1, 1) == 0);
    CHECK(terrain->tile(0, 3) == 0);
    CHECK(terrain->tile(3, 0) == 0);

    // A start corner outside the map (negative) must clamp on that side too.
    TiledTerrain::paintRectangle(*terrain, -5, -5, 1, 1, 7);
    CHECK(terrain->tile(0, 0) == 7);
    CHECK(terrain->tile(1, 1) == 7);
    CHECK(terrain->tile(0, 1) == 7);
    CHECK(terrain->tile(1, 0) == 7);
}

// Regression: fillCells/paintRectangle painted through setTile(), and every
// setTile() rebuilt the entire mesh - so a fill over N cells regenerated the
// whole map N times in one frame (and destroyed/recreated the GPU mesh N
// times with a live device). The revision counter is the observable proxy:
// one batched edit must bump it exactly once, not once per cell.
void testMultiCellEditsRebuildOnce()
{
    Scene scene;
    GameObject* object = scene.createGameObject("terrain");
    TiledTerrain* terrain = object->addComponent<TiledTerrain>();
    CHECK(terrain != nullptr);
    if (!terrain)
        return;

    const u32 width = 8, height = 8;
    std::vector<u8> map(static_cast<usize>(width) * height, 0);
    terrain->loadTilemap(width, height, map.data());

    // A rectangle over 16 cells: one rebuild, not sixteen.
    u64 revision = terrain->revision();
    TiledTerrain::paintRectangle(*terrain, 0, 0, 3, 3, 4);
    CHECK(terrain->revision() == revision + 1);
    CHECK(terrain->tile(0, 0) == 4);
    CHECK(terrain->tile(3, 3) == 4);

    // A flood fill over the remaining 48 zero cells: also one.
    revision = terrain->revision();
    TiledTerrain::fillCells(*terrain, 7, 7, 9);
    CHECK(terrain->revision() == revision + 1);
    CHECK(terrain->tile(7, 7) == 9);
    // The rectangle painted above is a different tile, so the fill must have
    // stopped at it rather than flooding the whole map.
    CHECK(terrain->tile(0, 0) == 4);

    // A fill that paints nothing (target already the wanted tile) returns
    // early and must not bump the revision at all.
    revision = terrain->revision();
    TiledTerrain::fillCells(*terrain, 7, 7, 9);
    CHECK(terrain->revision() == revision);

    // One cell through setTile() still rebuilds immediately - batching must
    // not have made a single edit lazy.
    revision = terrain->revision();
    terrain->setTile(5, 5, 3);
    CHECK(terrain->revision() == revision + 1);
}

// One tile is four vertices, so both editor size fields accepting 4096 per
// side means 67 million vertices - the editor froze building it rather than
// saying anything. loadTilemap() is where every caller (Create popup,
// Inspector, image import, a loaded scene) meets, so the refusal lives there
// and the map is left exactly as it was.
void testLoadTilemapRefusesAnAbsurdSize()
{
    Scene scene;
    GameObject* object = scene.createGameObject("terrain");
    TiledTerrain* terrain = object->addComponent<TiledTerrain>();
    CHECK(terrain != nullptr);
    if (!terrain)
        return;

    const u32 width = 4, height = 4;
    std::vector<u8> good(static_cast<usize>(width) * height, 2);
    terrain->loadTilemap(width, height, good.data());
    CHECK(terrain->mapWidth() == 4);

    // 2048x2048 = 4M tiles, past the limit. No allocation of that map is
    // attempted - the pointer is never read - so a small buffer is enough to
    // prove the guard runs before the copy.
    u8 probe = 0;
    terrain->loadTilemap(2048, 2048, &probe);

    // The previous map survived intact.
    CHECK(terrain->mapWidth() == 4);
    CHECK(terrain->mapHeight() == 4);
    CHECK(terrain->tile(1, 1) == 2);
}

} // namespace

int main()
{
    testLoadTilemapProducesExpectedPatchCount();
    testDefaultTilesPerPatchMatchesDefaultMapSize();
    testSetTileTriggersRebuildAndUpdatesCell();
    testAtlasUVMatchesExpectedCell();
    testAtlasUVsMatchEncodedRotations();
    testTilesFromImageMatchesApocalyxBottomFirstRows();
    testEncodedFormatLimitsAtlasToEightCellsPerSide();
    testWrappedTileWrapsAroundMapEdge();
    testAtlasMaterialRoundTripAndSizeWithNoGPU();
    testFillCellsPaintsOnlyConnectedRegion();
    testPaintRectangleClampsToMapBounds();
    testMultiCellEditsRebuildOnce();
    testLoadTilemapRefusesAnAbsurdSize();

    if (gFailures)
        std::fprintf(stderr, "%d tiled terrain test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
