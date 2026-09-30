#include "PCH.h"

#include "TiledTerrain.h"

#include "AssetManager.h"
#include "GameObject.h"
#include "Log.h"
#include "MaterialManager.h"
#include "MeshRenderer.h"
#include "Pixmap.h"

#include <algorithm>
#include <cmath>

namespace Radion
{

namespace
{
// 1024x1024 tiles is already a four-million-vertex mesh; past this is a mistake.
constexpr usize kMaxTiles = 1024u * 1024u;

// Mip limit for the atlas: past this many halvings a level averages a tile with its neighbours despite atlasUV()'s inset. Tied to tilesInSide, known before load, since a fixed number suits neither coarse nor fine atlases.
u32 atlasMipLimit(int tilesInSide)
{
    return static_cast<u32>(Math::max(1, static_cast<int>(std::log2(
                                         static_cast<f32>(Math::max(tilesInSide, 1))))));
}

bool validCell(const TiledTerrain& terrain, int x, int z)
{
    return x >= 0 && x < static_cast<int>(terrain.mapWidth()) &&
          z >= 0 && z < static_cast<int>(terrain.mapHeight());
}
} // namespace

TiledTerrain::TiledTerrain() : Component(Type)
{
}

void TiledTerrain::onDestroy()
{
    if (mMesh.valid() && GPU::ready())
        Assets().destroyMesh(mMesh);
    mMesh = MeshHandle();
}

void TiledTerrain::loadTilemap(u32 width, u32 height, const u8* data)
{
    if (!data || width == 0 || height == 0)
        return;
    // The one place size must be sane: one tile is four vertices, so the 4096-per-side editor fields would ask rebuild() for 67 million and hang the editor.
    const usize tileCount = static_cast<usize>(width) * height;
    if (tileCount > kMaxTiles)
    {
        Log::error("TiledTerrain: %ux%u is %u tiles, over the %u limit - the mesh is four "
                   "vertices per tile", width, height, static_cast<u32>(tileCount),
                   static_cast<u32>(kMaxTiles));
        return;
    }

    mMapWidth = width;
    mMapHeight = height;
    mTileMap.assign(data, data + tileCount);
    rebuild();
}

void TiledTerrain::setTile(u32 x, u32 z, u8 id)
{
    if (mTileMap.empty() || x >= mMapWidth || z >= mMapHeight)
        return;
    mTileMap[z * mMapWidth + x] = id;
    rebuild();
}

u8 TiledTerrain::tile(u32 x, u32 z) const
{
    return (!mTileMap.empty() && x < mMapWidth && z < mMapHeight) ? mTileMap[z * mMapWidth + x]
                                                                  : mDefaultTile;
}

u32 TiledTerrain::mapWidth() const
{
    return mMapWidth;
}

u32 TiledTerrain::mapHeight() const
{
    return mMapHeight;
}

void TiledTerrain::setTilesInSide(int tilesInSide)
{
    mTilesInSide = std::clamp(tilesInSide, 1, 8);
    if (!mTileMap.empty())
        rebuild();
}

int TiledTerrain::tilesInSide() const
{
    return mTilesInSide;
}

void TiledTerrain::setPatchLength(f32 length)
{
    mPatchLength = length;
    if (!mTileMap.empty())
        rebuild();
}

f32 TiledTerrain::patchLength() const
{
    return mPatchLength;
}

void TiledTerrain::setTilesPerPatch(int tilesPerPatch)
{
    mTilesPerPatch = tilesPerPatch > 0 ? tilesPerPatch : 1;
    if (!mTileMap.empty())
        rebuild();
}

int TiledTerrain::tilesPerPatch() const
{
    return mTilesPerPatch;
}

void TiledTerrain::setDefaultTile(u8 defaultTile)
{
    mDefaultTile = defaultTile;
    if (!mTileMap.empty())
        rebuild();
}

u8 TiledTerrain::defaultTile() const
{
    return mDefaultTile;
}

void TiledTerrain::setAtlasMaterial(const std::string& material)
{
    mAtlasMaterial = material;
    if (!mTileMap.empty())
        rebuild();
}

const std::string& TiledTerrain::atlasMaterial() const
{
    return mAtlasMaterial;
}

void TiledTerrain::setAtlasTexture(const std::string& imageFile)
{
    mAtlasTexture = imageFile;
    if (!mTileMap.empty())
        rebuild();
}

const std::string& TiledTerrain::atlasTexture() const
{
    return mAtlasTexture;
}

TextureHandle TiledTerrain::resolveAtlasTexture() const
{
    if (!GPU::ready())
        return TextureHandle();
    // Same precedence as rebuild(): the image file wins over a named material.
    if (!mAtlasTexture.empty())
    {
        const TextureHandle atlas = Assets().loadTexture(mAtlasTexture, ColorSpace::sRGB, true,
                                                         atlasMipLimit(mTilesInSide));
        if (atlas.valid())
            return atlas;
    }
    if (mAtlasMaterial.empty())
        return TextureHandle();
    std::vector<Material> loaded;
    if (!MaterialManager::getSingleton().load(mAtlasMaterial, loaded) || loaded.empty())
        return TextureHandle();
    return loaded.front().textures[SlotAlbedo].texture;
}

bool TiledTerrain::atlasSize(u32& width, u32& height) const
{
    const TextureHandle albedo = resolveAtlasTexture();
    if (!albedo.valid())
        return false;
    TextureDesc desc;
    if (!GPU::getSingleton().textureInfo(albedo, desc) || desc.width == 0 || desc.height == 0)
        return false;
    width = desc.width;
    height = desc.height;
    return true;
}

MeshHandle TiledTerrain::mesh() const
{
    return mMesh;
}

u32 TiledTerrain::patchCount() const
{
    return mPatchCount;
}

u64 TiledTerrain::revision() const
{
    return mRevision;
}

u8 TiledTerrain::wrappedTile(const u8* tileMap, u32 mapWidth, u32 mapHeight,
                             int x, int z, u8 defaultTile)
{
    if (!tileMap)
        return defaultTile;
    const int w = static_cast<int>(mapWidth);
    const int h = static_cast<int>(mapHeight);
    x = ((x % w) + w) % w;
    z = ((z % h) + h) % h;
    return tileMap[static_cast<usize>(z) * mapWidth + x];
}

void TiledTerrain::atlasUV(u8 tile, int tilesInSide, Math::vec2& uvMin, Math::vec2& uvMax)
{
    const f32 stepUV = 1.0f / static_cast<f32>(tilesInSide);
    const u8 atlasTile = tile & 0x3f;
    const int atlasX = atlasTile % tilesInSide;
    const int atlasZ = tilesInSide - 1 - atlasTile / tilesInSide;
    // Inset a fraction of the cell: filtered sampling blends texels across a cell edge into its neighbour (tile bleeding). One texel would do but the atlas size is unknown here, so a fraction comfortably under one texel.
    const f32 inset = stepUV * 0.02f;
    uvMin = Math::vec2(atlasX * stepUV + inset, atlasZ * stepUV + inset);
    uvMax = Math::vec2((atlasX + 1) * stepUV - inset, (atlasZ + 1) * stepUV - inset);
}

void TiledTerrain::atlasUVs(u8 tile, int tilesInSide, Math::vec2& bottomLeft,
                            Math::vec2& bottomRight, Math::vec2& topLeft, Math::vec2& topRight)
{
    Math::vec2 uvMin, uvMax;
    atlasUV(tile, tilesInSide, uvMin, uvMax);

    switch (tile >> 6)
    {
    case 1:
        bottomLeft = uvMax;
        bottomRight = Math::vec2(uvMax.x, uvMin.y);
        topLeft = Math::vec2(uvMin.x, uvMax.y);
        topRight = uvMin;
        break;
    case 2:
        bottomLeft = Math::vec2(uvMax.x, uvMin.y);
        bottomRight = uvMin;
        topLeft = uvMax;
        topRight = Math::vec2(uvMin.x, uvMax.y);
        break;
    case 3:
        bottomLeft = uvMin;
        bottomRight = Math::vec2(uvMin.x, uvMax.y);
        topLeft = Math::vec2(uvMax.x, uvMin.y);
        topRight = uvMax;
        break;
    default:
        bottomLeft = Math::vec2(uvMin.x, uvMax.y);
        bottomRight = uvMax;
        topLeft = uvMin;
        topRight = Math::vec2(uvMax.x, uvMin.y);
        break;
    }
}

void TiledTerrain::paintCell(TiledTerrain& terrain, int x, int z, u8 tileId)
{
    if (!validCell(terrain, x, z))
        return;
    terrain.setTile(static_cast<u32>(x), static_cast<u32>(z), tileId);
}

void TiledTerrain::fillCells(TiledTerrain& terrain, int startX, int startZ, u8 tileId)
{
    if (!validCell(terrain, startX, startZ))
        return;
    const u8 targetTile = terrain.tile(static_cast<u32>(startX), static_cast<u32>(startZ));
    if (targetTile == tileId)
        return;

    struct Cell
    {
        int x;
        int z;
    };
    std::vector<Cell> pending;
    pending.push_back({startX, startZ});
    terrain.beginBatch();
    while (!pending.empty())
    {
        const Cell cell = pending.back();
        pending.pop_back();
        if (!validCell(terrain, cell.x, cell.z))
            continue;
        if (terrain.tile(static_cast<u32>(cell.x), static_cast<u32>(cell.z)) != targetTile)
            continue;
        paintCell(terrain, cell.x, cell.z, tileId);
        pending.push_back({cell.x - 1, cell.z});
        pending.push_back({cell.x + 1, cell.z});
        pending.push_back({cell.x, cell.z - 1});
        pending.push_back({cell.x, cell.z + 1});
    }
    terrain.endBatch();
}

void TiledTerrain::paintRectangle(TiledTerrain& terrain, int x0, int z0, int x1, int z1, u8 tileId)
{
    const int left = std::max(0, std::min(x0, x1));
    const int right = std::min(static_cast<int>(terrain.mapWidth()) - 1, std::max(x0, x1));
    const int top = std::max(0, std::min(z0, z1));
    const int bottom = std::min(static_cast<int>(terrain.mapHeight()) - 1, std::max(z0, z1));
    terrain.beginBatch();
    for (int z = top; z <= bottom; ++z)
        for (int x = left; x <= right; ++x)
            paintCell(terrain, x, z, tileId);
    terrain.endBatch();
}

bool TiledTerrain::tilesFromImageColors(const Pixmap& image, std::vector<u8>& outTiles)
{
    // Port of the reference's map-from-image loader: the tile ID is the raw grayscale byte, one pixel per tile (gltiledterrain.cpp:37-42).
    // generate_heightmap() is luminance (0.299/0.587/0.114); on grayscale it returns the same byte, so round-trip is exact; a color image is read like a photo.
    if (!image.is_valid() || image.width <= 0 || image.height <= 0)
        return false;

    const u32 width = static_cast<u32>(image.width);
    const u32 height = static_cast<u32>(image.height);
    const usize tileCount = static_cast<usize>(width) * height;
    if (tileCount > kMaxTiles)
    {
        Log::error("TiledTerrain: image is %ux%u (%u tiles), over the %u tile limit - scale it "
                   "down, one pixel is one tile",
                   width, height, static_cast<u32>(tileCount), static_cast<u32>(kMaxTiles));
        return false;
    }

    Pixmap* gray = image.generate_heightmap();
    if (!gray)
        return false;
    std::vector<u8> tiles(tileCount);
    for (u32 z = 0; z < height; ++z)
        for (u32 x = 0; x < width; ++x)
            tiles[static_cast<usize>(height - 1 - z) * width + x] =
                static_cast<u8>(gray->get_pixel_color(x, z).r());
    delete gray;

    outTiles = std::move(tiles);
    return true;
}

bool TiledTerrain::saveTilemapImage(const std::string& path) const
{
    if (mTileMap.empty() || path.empty())
        return false;

    Pixmap image(static_cast<int>(mMapWidth), static_cast<int>(mMapHeight), 1);
    for (u32 z = 0; z < mMapHeight; ++z)
        for (u32 x = 0; x < mMapWidth; ++x)
        {
            const u8 tileId = mTileMap[static_cast<usize>(z) * mMapWidth + x];
            image.set_pixel(x, mMapHeight - 1 - z, tileId, tileId, tileId, 255);
        }
    return image.save(path.c_str());
}

void TiledTerrain::beginBatch()
{
    ++mRebuildSuspended;
}

void TiledTerrain::endBatch()
{
    if (mRebuildSuspended > 0)
        --mRebuildSuspended;
    if (mRebuildSuspended > 0 || !mRebuildPending)
        return;
    mRebuildPending = false;
    rebuild();
}

void TiledTerrain::rebuild()
{
    // Deferred, not dropped: one rebuild at endBatch(). Recorded before the revision bump so a batch counts as one edit.
    if (mRebuildSuspended > 0)
    {
        mRebuildPending = true;
        return;
    }

    ++mRevision;
    mPatchCount = 0;
    if (mTileMap.empty() || !owner())
        return;

    const f32 tileWorld = mPatchLength / static_cast<f32>(mTilesPerPatch);
    const int patchCountX =
        static_cast<int>(std::ceil(static_cast<f32>(mMapWidth) / mTilesPerPatch));
    const int patchCountZ =
        static_cast<int>(std::ceil(static_cast<f32>(mMapHeight) / mTilesPerPatch));

    struct Patch
    {
        u32 first;
        u32 count;
        AABB bounds;
    };
    std::vector<Patch> patches;
    MeshData data;

    for (int pz = 0; pz < patchCountZ; ++pz)
    {
        for (int px = 0; px < patchCountX; ++px)
        {
            const int originX = px * mTilesPerPatch;
            const int originZ = pz * mTilesPerPatch;
            const f32 worldX = originX * tileWorld;
            const f32 worldZ = originZ * tileWorld;
            const u32 firstIndex = static_cast<u32>(data.indices.size());

            AABB bounds;
            bounds.expand(Math::vec3(worldX, -0.01f, worldZ));
            bounds.expand(Math::vec3(worldX + mPatchLength, 0.01f, worldZ + mPatchLength));

            for (int tz = 0; tz < mTilesPerPatch; ++tz)
            {
                for (int tx = 0; tx < mTilesPerPatch; ++tx)
                {
                    const u8 tileId = wrappedTile(mTileMap.data(), mMapWidth, mMapHeight,
                                                  originX + tx, originZ + tz, mDefaultTile);
                    Math::vec2 bottomLeft, bottomRight, topLeft, topRight;
                    atlasUVs(tileId, mTilesInSide, bottomLeft, bottomRight, topLeft, topRight);

                    const f32 x0 = worldX + tx * tileWorld;
                    const f32 x1 = x0 + tileWorld;
                    const f32 z0 = worldZ + tz * tileWorld;
                    const f32 z1 = z0 + tileWorld;
                    const u32 base = static_cast<u32>(data.positions.size());
                    const Math::vec3 normal(0.0f, 1.0f, 0.0f);
                    const Math::vec4 tangent(1.0f, 0.0f, 0.0f, 1.0f);

                    data.positions.push_back(Math::vec3(x0, 0.0f, z0));
                    data.positions.push_back(Math::vec3(x1, 0.0f, z0));
                    data.positions.push_back(Math::vec3(x0, 0.0f, z1));
                    data.positions.push_back(Math::vec3(x1, 0.0f, z1));
                    data.normals.insert(data.normals.end(), 4, normal);
                    data.tangents.insert(data.tangents.end(), 4, tangent);
                    data.uvs.push_back(bottomLeft);
                    data.uvs.push_back(bottomRight);
                    data.uvs.push_back(topLeft);
                    data.uvs.push_back(topRight);

                    data.indices.push_back(base);
                    data.indices.push_back(base + 2);
                    data.indices.push_back(base + 1);
                    data.indices.push_back(base + 1);
                    data.indices.push_back(base + 2);
                    data.indices.push_back(base + 3);
                }
            }

            Patch patch;
            patch.first = firstIndex;
            patch.count = static_cast<u32>(data.indices.size()) - firstIndex;
            patch.bounds = bounds;
            patches.push_back(patch);
        }
    }

    mPatchCount = static_cast<u32>(patches.size());
    if (patches.empty())
        return;

    // Mirrors ManualMesh::beginSubMesh's material resolution: one shared slot for every patch (as the reference's add_surface(..., 0, ...)).
    Material material;
    bool haveMaterial = false;
    if (!mAtlasMaterial.empty())
    {
        std::vector<Material> loaded;
        if (MaterialManager::getSingleton().load(mAtlasMaterial, loaded) && !loaded.empty())
        {
            material = loaded.front();
            haveMaterial = true;
        }
        else
            material.name = mAtlasMaterial;
    }
    // An atlas image is the answer (the only texture a tile terrain has) and covers the case where the material failed to resolve and left it untextured.
    // GPU::ready() gates the load: a failed loadTexture() builds the default checker via GPU::getSingleton(), which aborts with no device. Headless keeps the path and resolves on the next rebuild.
    if (!mAtlasTexture.empty() && GPU::ready())
    {
        const TextureHandle atlas = Assets().loadTexture(mAtlasTexture, ColorSpace::sRGB, true,
                                                         atlasMipLimit(mTilesInSide));
        if (atlas.valid())
        {
            if (!haveMaterial)
            {
                material = Material();
                material.name = mAtlasTexture;
                material.flags |= MaterialLit;
                material.params.baseColor = Math::vec4(1.0f);
                material.params.surface.x = 1.0f; // roughness - a floor, not a mirror
                material.params.surface.y = 0.0f; // metal
            }
            MaterialTexture& albedo = material.textures[SlotAlbedo];
            albedo.texture = atlas;
            albedo.file = mAtlasTexture;
            albedo.source = TextureSource::Static;
            // Trilinear, not Point: Point disables mip filtering and aliases into vertical banding at distance. Edge bleeding is handled by atlasUV()'s inset and the capped mip chain. Clamp keeps edge tiles from wrapping.
            SamplerDesc sampler;
            sampler.filter = Filter::Trilinear;
            sampler.wrapU = Wrap::Clamp;
            sampler.wrapV = Wrap::Clamp;
            sampler.wrapW = Wrap::Clamp;
            albedo.sampler = Assets().getSampler(sampler);
            material.paramsDirty = true;
        }
        else
            Log::error("TiledTerrain: could not load atlas texture '%s'", mAtlasTexture.c_str());
    }
    data.materials.push_back(material);

    data.submeshes.reserve(patches.size());
    for (const Patch& patch : patches)
    {
        SubMesh submesh;
        submesh.indexOffset = patch.first;
        submesh.indexCount = patch.count;
        submesh.materialSlot = 0;
        submesh.bounds = patch.bounds;
        data.submeshes.push_back(submesh);
    }
    Assets().computeBounds(data);

    if (!GPU::ready())
        return;

    if (mMesh.valid())
        Assets().destroyMesh(mMesh);
    mMesh = Assets().createMesh(data);
    if (!mMesh.valid())
        return;
    if (!mRenderer)
    {
        mRenderer = owner()->addComponent<MeshRenderer>(mMesh);
        if (mRenderer)
            mRenderer->setGeneratedBy(this);
    }
    else
        mRenderer->setMesh(mMesh);

    Log::info("TiledTerrain: %ux%u tilemap -> %u patches", mMapWidth, mMapHeight, mPatchCount);
}

} // namespace Radion
