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

// Each tilesPerPatch x tilesPerPatch block is its own submesh, so per-submesh frustum culling culls patches for free.
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

    // An image file instead of an authored .material; wins over setAtlasMaterial() when both are set. Builds the material in rebuild().
    void setAtlasTexture(const std::string& imageFile);
    const std::string& atlasTexture() const;
    // One place for the answer rebuild() draws with (image file first, then named material).
    TextureHandle resolveAtlasTexture() const;
    // False with width/height untouched when there is no atlas.
    bool atlasSize(u32& width, u32& height) const;

    MeshHandle mesh() const;
    u32 patchCount() const;
    u64 revision() const;

    // Wrapped, not clamped.
    static u8 wrappedTile(const u8* tileMap, u32 mapWidth, u32 mapHeight,
                          int x, int z, u8 defaultTile);
    static void atlasUV(u8 tile, int tilesInSide, Math::vec2& uvMin, Math::vec2& uvMax);
    // UVs in vertex order: bottom-left, bottom-right, top-left, top-right. Tile byte: atlas cell in bits 0-5, quarter-turn in bits 6-7.
    static void atlasUVs(u8 tile, int tilesInSide, Math::vec2& bottomLeft,
                         Math::vec2& bottomRight, Math::vec2& topLeft, Math::vec2& topRight);

    // Pure grid math over setTile()/tile(), no ImGui or GPU dependency.
    static void paintCell(TiledTerrain& terrain, int x, int z, u8 tileId);
    static void fillCells(TiledTerrain& terrain, int startX, int startZ, u8 tileId);
    static void paintRectangle(TiledTerrain& terrain, int x0, int z0, int x1, int z1, u8 tileId);

    // One pixel is one encoded tile byte; color images are converted to luminance.
    static bool tilesFromImageColors(const Pixmap& image, std::vector<u8>& outTiles);
    // The encoded byte is preserved, including its two rotation bits.
    bool saveTilemapImage(const std::string& path) const;

private:
    friend class GameObject;
    TiledTerrain();

    void onDestroy() override;
    void rebuild();
    // Defers rebuild() until endBatch() so a flood fill rebuilds once, not per cell. Nested begins are counted.
    void beginBatch();
    void endBatch();

    std::vector<u8> mTileMap;
    u32 mMapWidth = 0;
    u32 mMapHeight = 0;
    int mTilesInSide = 8;
    // 1 means one submesh per tile; 8 matches the Inspector default map size so a fresh component gets one sane patch.
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
