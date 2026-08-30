#ifndef RADION_TILED_TERRAIN_H
#define RADION_TILED_TERRAIN_H

#include "Component.h"
#include "Math.h"
#include "Mesh.h"

#include <string>
#include <vector>

namespace Radion
{

class MeshRenderer;
class Pixmap;

// Flat tile-atlas terrain: a width x height grid of u8 tile IDs becomes a
// mesh where every tilesPerPatch x tilesPerPatch block of tiles is its own
// submesh, so the scene's per-submesh frustum culling culls invisible
// patches for free. Each tile is one quad, UVs taken from a tile in a
// tilesInSide x tilesInSide texture atlas.
class TiledTerrain final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::TiledTerrain;

    void loadTilemap(u32 width, u32 height, const u8* data);
    void setTile(u32 x, u32 z, u8 id);
    u8 tile(u32 x, u32 z) const;
    u32 mapWidth() const;
    u32 mapHeight() const;

    void setTilesInSide(int tilesInSide);
    int tilesInSide() const;
    void setPatchLength(f32 length);
    f32 patchLength() const;
    void setTilesPerPatch(int tilesPerPatch);
    int tilesPerPatch() const;
    void setDefaultTile(u8 defaultTile);
    u8 defaultTile() const;

    void setAtlasMaterial(const std::string& material);
    const std::string& atlasMaterial() const;

    // The atlas as a plain image file instead of an authored .material. A
    // tile atlas is one albedo texture and nothing else, so requiring a
    // hand-written material for it meant a dropped PNG could not be used at
    // all - and a material name that fails to resolve leaves the terrain with
    // a blank material, drawing untextured. Setting this builds the material
    // in rebuild(): lit, clamped, unfiltered between tiles.
    //
    // Set on its own, or alongside setAtlasMaterial() - the texture wins,
    // because it is the more specific answer to "what does this draw with".
    void setAtlasTexture(const std::string& imageFile);
    const std::string& atlasTexture() const;
    // The atlas albedo, from whichever source is set - the image file first,
    // then the named material. Invalid if neither resolves or there is no
    // live GPU. One place, because the Tile Painter needs the same answer
    // rebuild() draws with, and had its own copy that only knew about
    // materials.
    TextureHandle resolveAtlasTexture() const;
    // Pixel size of resolveAtlasTexture(). False with width/height left
    // untouched when there is no atlas to measure.
    bool atlasSize(u32& width, u32& height) const;

    MeshHandle mesh() const;
    u32 patchCount() const;
    u64 revision() const;

    // Tile coordinates wrapped (not clamped) into the map bounds - what a
    // patch straddling the map edge samples from.
    static u8 wrappedTile(const u8* tileMap, u32 mapWidth, u32 mapHeight,
                          int x, int z, u8 defaultTile);
    // Atlas UV rectangle for one encoded tile byte in a tilesInSide x tilesInSide atlas.
    static void atlasUV(u8 tile, int tilesInSide, glm::vec2& uvMin, glm::vec2& uvMax);
    // UVs in mesh vertex order: bottom-left, bottom-right, top-left, top-right.
    // The tile byte stores the atlas cell in bits 0-5 and its quarter-turn in bits 6-7.
    static void atlasUVs(u8 tile, int tilesInSide, glm::vec2& bottomLeft,
                         glm::vec2& bottomRight, glm::vec2& topLeft, glm::vec2& topRight);

    // Tile-grid paint operations shared by the editor's Tile Painter panel
    // and its unit tests - pure grid math over setTile()/tile(), no ImGui or
    // GPU dependency, so a test binary linking only this component can
    // exercise them directly.
    static void paintCell(TiledTerrain& terrain, int x, int z, u8 tileId);
    static void fillCells(TiledTerrain& terrain, int startX, int startZ, u8 tileId);
    static void paintRectangle(TiledTerrain& terrain, int x0, int z0, int x1, int z1, u8 tileId);

    // Builds a tile-ID grid from a grayscale image: one pixel is one encoded
    // tile byte. Color images are converted to luminance before being read.
    static bool tilesFromImageColors(const Pixmap& image, std::vector<u8>& outTiles);
    // Saves the map in Apocalyx's grayscale image convention. The encoded
    // byte is preserved, including its two rotation bits.
    bool saveTilemapImage(const std::string& path) const;

private:
    friend class GameObject;
    TiledTerrain();

    void onDestroy() override;
    void rebuild();
    // A multi-cell edit rebuilds once at the end instead of once per cell.
    // setTile() rebuilds immediately, which is right for one cell and ruinous
    // for a flood fill: rebuild() regenerates every vertex of the whole map
    // and destroys/recreates the GPU mesh, so filling a 256x256 map ran that
    // 65536 times in a single frame. Between begin and end the rebuild is
    // recorded and deferred; endBatch() runs the one that was owed. Nested
    // begins are counted, so a batched operation calling another one still
    // rebuilds exactly once.
    void beginBatch();
    void endBatch();

    std::vector<u8> mTileMap;
    u32 mMapWidth = 0;
    u32 mMapHeight = 0;
    int mTilesInSide = 8;
    // 1 means one submesh per tile - a bare addComponent<TiledTerrain>() (the
    // Inspector's own "Add Component" fallback, with no size chosen yet)
    // followed by an 8x8 "Build Tilemap" used to hand back 64 one-tile
    // patches instead of the single patch this size deserves. 8 matches the
    // Inspector/Create-menu default map size, so the common "just click
    // through" path produces one sane patch, not sixty-four tiny ones.
    int mTilesPerPatch = 8;
    f32 mPatchLength = 1.0f;
    u8 mDefaultTile = 0;
    std::string mAtlasMaterial;
    std::string mAtlasTexture;

    MeshHandle mMesh;
    MeshRenderer* mRenderer = nullptr;
    u32 mPatchCount = 0;
    u64 mRevision = 0;
    u32 mRebuildSuspended = 0;
    bool mRebuildPending = false;
};

} // namespace Radion

#endif // RADION_TILED_TERRAIN_H
