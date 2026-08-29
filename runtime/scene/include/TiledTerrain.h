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
    // Pixel size of the atlas material's albedo texture. False with width/
    // height left untouched if there is no atlas material, no albedo
    // texture on it, or no live GPU to ask.
    bool atlasSize(u32& width, u32& height) const;

    MeshHandle mesh() const;
    u32 patchCount() const;
    u64 revision() const;

    // Tile coordinates wrapped (not clamped) into the map bounds - what a
    // patch straddling the map edge samples from.
    static u8 wrappedTile(const u8* tileMap, u32 mapWidth, u32 mapHeight,
                          int x, int z, u8 defaultTile);
    // Atlas UV rectangle for one tile ID in a tilesInSide x tilesInSide atlas.
    static void atlasUV(u8 tile, int tilesInSide, glm::vec2& uvMin, glm::vec2& uvMax);

    // Tile-grid paint operations shared by the editor's Tile Painter panel
    // and its unit tests - pure grid math over setTile()/tile(), no ImGui or
    // GPU dependency, so a test binary linking only this component can
    // exercise them directly.
    static void paintCell(TiledTerrain& terrain, int x, int z, u8 tileId);
    static void fillCells(TiledTerrain& terrain, int startX, int startZ, u8 tileId);
    static void paintRectangle(TiledTerrain& terrain, int x0, int z0, int x1, int z1, u8 tileId);

private:
    friend class GameObject;
    TiledTerrain();

    void onDestroy() override;
    void rebuild();

    std::vector<u8> mTileMap;
    u32 mMapWidth = 0;
    u32 mMapHeight = 0;
    int mTilesInSide = 8;
    int mTilesPerPatch = 1;
    f32 mPatchLength = 1.0f;
    u8 mDefaultTile = 0;
    std::string mAtlasMaterial;

    MeshHandle mMesh;
    MeshRenderer* mRenderer = nullptr;
    u32 mPatchCount = 0;
    u64 mRevision = 0;
};

} // namespace Radion

#endif // RADION_TILED_TERRAIN_H
