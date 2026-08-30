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
// 1024x1024 tiles - already a four-million-vertex mesh. Past this it is a
// mistake, not a big map.
constexpr usize kMaxTiles = 1024u * 1024u;

// How many mip levels the atlas texture is allowed to build. Past this many
// halvings a level starts averaging a tile with its neighbours in the sheet
// regardless of atlasUV()'s own inset - the level itself no longer has
// enough texels left to keep them apart (loadTexture()'s own doc comment on
// mipLimit makes the same point for a texture atlas in general). Tied to
// tilesInSide, the one atlas dimension already known before the texture is
// even loaded, rather than a fixed number that would be too shallow for a
// coarse atlas and too deep for a fine one.
u32 atlasMipLimit(int tilesInSide)
{
    return static_cast<u32>(glm::max(1, static_cast<int>(std::log2(
                                         static_cast<f32>(glm::max(tilesInSide, 1))))));
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
    // Every caller funnels through here - the Create popup, the Inspector's
    // Build Tilemap, an image import, a loaded scene - so this is the one
    // place the size has to be sane. One tile is four vertices, so the 4096
    // both editor fields accept on each side would ask rebuild() for 67
    // million of them and hang the editor before anything could report why.
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
    // Same precedence rebuild() uses: the image file is the more specific
    // answer, so it wins over a named material.
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

void TiledTerrain::atlasUV(u8 tile, int tilesInSide, glm::vec2& uvMin, glm::vec2& uvMax)
{
    const f32 stepUV = 1.0f / static_cast<f32>(tilesInSide);
    const u8 atlasTile = tile & 0x3f;
    const int atlasX = atlasTile % tilesInSide;
    const int atlasZ = tilesInSide - 1 - atlasTile / tilesInSide;
    // Inset a small fraction of the cell inward on every side. Filtered
    // sampling (mipmapped minification on the terrain mesh, and the Tile
    // Painter's own preview draws through the same texture) blends texels
    // across a cell's edge with its neighbour in the sheet - the "seam" that
    // showed up as one tile bleeding color into the one next to it. The true
    // fix only needs one texel of slack, but the pixel size of the atlas is
    // not known at this call, so the margin is a fraction of the cell
    // instead - comfortably under one texel for any atlas built at a normal
    // resolution, without visibly cropping the art.
    const f32 inset = stepUV * 0.02f;
    uvMin = glm::vec2(atlasX * stepUV + inset, atlasZ * stepUV + inset);
    uvMax = glm::vec2((atlasX + 1) * stepUV - inset, (atlasZ + 1) * stepUV - inset);
}

void TiledTerrain::atlasUVs(u8 tile, int tilesInSide, glm::vec2& bottomLeft,
                            glm::vec2& bottomRight, glm::vec2& topLeft, glm::vec2& topRight)
{
    glm::vec2 uvMin, uvMax;
    atlasUV(tile, tilesInSide, uvMin, uvMax);

    switch (tile >> 6)
    {
    case 1:
        bottomLeft = uvMax;
        bottomRight = glm::vec2(uvMax.x, uvMin.y);
        topLeft = glm::vec2(uvMin.x, uvMax.y);
        topRight = uvMin;
        break;
    case 2:
        bottomLeft = glm::vec2(uvMax.x, uvMin.y);
        bottomRight = uvMin;
        topLeft = uvMax;
        topRight = glm::vec2(uvMin.x, uvMax.y);
        break;
    case 3:
        bottomLeft = uvMin;
        bottomRight = glm::vec2(uvMin.x, uvMax.y);
        topLeft = glm::vec2(uvMax.x, uvMin.y);
        topRight = uvMax;
        break;
    default:
        bottomLeft = glm::vec2(uvMin.x, uvMax.y);
        bottomRight = uvMax;
        topLeft = uvMin;
        topRight = glm::vec2(uvMax.x, uvMin.y);
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
    // Port of the reference's own map-from-image loader: the tile ID is the
    // image's raw grayscale byte, one pixel is one tile, no palette
    // (GLTiledTerrain::GLTiledTerrain, gltiledterrain.cpp:37-42 -
    // "tilesMap[ct] = tileImage.getBuffer()[ct]" over a DRImage the caller
    // already prepared as grayscale). generate_heightmap() is the engine's
    // own grayscale conversion (0.299/0.587/0.114 luminance, Pixmap.cpp:1037-
    // 1038) - on an actual grayscale source, where r=g=b, it returns that
    // same byte back untouched, so an image authored the way the reference
    // expects round-trips exactly; a color image is read the same way a
    // photo would be, rather than refused.
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
    // Deferred, not dropped: the whole point is that the caller gets one
    // rebuild at endBatch() rather than one per edited cell. Recorded before
    // the revision counter too, so a batch reads as the single logical edit
    // it is instead of bumping the revision once per cell.
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
            bounds.expand(glm::vec3(worldX, -0.01f, worldZ));
            bounds.expand(glm::vec3(worldX + mPatchLength, 0.01f, worldZ + mPatchLength));

            for (int tz = 0; tz < mTilesPerPatch; ++tz)
            {
                for (int tx = 0; tx < mTilesPerPatch; ++tx)
                {
                    const u8 tileId = wrappedTile(mTileMap.data(), mMapWidth, mMapHeight,
                                                  originX + tx, originZ + tz, mDefaultTile);
                    glm::vec2 bottomLeft, bottomRight, topLeft, topRight;
                    atlasUVs(tileId, mTilesInSide, bottomLeft, bottomRight, topLeft, topRight);

                    const f32 x0 = worldX + tx * tileWorld;
                    const f32 x1 = x0 + tileWorld;
                    const f32 z0 = worldZ + tz * tileWorld;
                    const f32 z1 = z0 + tileWorld;
                    const u32 base = static_cast<u32>(data.positions.size());
                    const glm::vec3 normal(0.0f, 1.0f, 0.0f);
                    const glm::vec4 tangent(1.0f, 0.0f, 0.0f, 1.0f);

                    data.positions.push_back(glm::vec3(x0, 0.0f, z0));
                    data.positions.push_back(glm::vec3(x1, 0.0f, z0));
                    data.positions.push_back(glm::vec3(x0, 0.0f, z1));
                    data.positions.push_back(glm::vec3(x1, 0.0f, z1));
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

    // Mirrors ManualMesh::beginSubMesh's material resolution, one shared
    // slot for every patch (same single-material-slot design as the
    // reference's add_surface(..., 0, ...) calls).
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
    // An atlas image, when there is one, is the answer - it is the only
    // texture a tile terrain has, so there is nothing an authored material
    // adds that this cannot say. It also covers the case above failing to
    // resolve, which otherwise left a blank material and a terrain drawn
    // untextured with no indication why.
    // GPU::ready() gates the load, not just the upload further down: a failed
    // loadTexture() falls back to the default checker, and building that
    // reaches GPU::getSingleton(), which aborts outright when there is no
    // device. Headless - a test, a scene loaded before the window exists -
    // keeps the path recorded and resolves it on the next rebuild.
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
                material.params.baseColor = glm::vec4(1.0f);
                material.params.surface.x = 1.0f; // roughness - a floor, not a mirror
                material.params.surface.y = 0.0f; // metal
            }
            MaterialTexture& albedo = material.textures[SlotAlbedo];
            albedo.texture = atlas;
            albedo.file = mAtlasTexture;
            albedo.source = TextureSource::Static;
            // Trilinear, not Point: Point disables mip filtering outright,
            // which reads one raw texel per screen pixel - correct up close,
            // but past the distance where several atlas texels map to one
            // screen pixel it aliases into exactly the vertical banding
            // minification artifacts always cause without a mip chain to
            // fall back on. Bleeding across a tile's edge - the reason
            // Point looked tempting - is handled by atlasUV()'s own inset
            // and the capped mip chain below instead, not by disabling
            // filtering. Clamp keeps the edge tiles from wrapping to the
            // far side of the atlas.
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
