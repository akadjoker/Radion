#include "PCH.h"

#include "TiledTerrain.h"

#include "AssetManager.h"
#include "GameObject.h"
#include "Log.h"
#include "MaterialManager.h"
#include "MeshRenderer.h"

#include <cmath>

namespace Radion
{

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
    mMapWidth = width;
    mMapHeight = height;
    mTileMap.assign(data, data + static_cast<usize>(width) * height);
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
    mTilesInSide = tilesInSide > 0 ? tilesInSide : 1;
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
    const int atlasX = tile % tilesInSide;
    const int atlasZ = tile / tilesInSide;
    uvMin = glm::vec2(atlasX * stepUV, atlasZ * stepUV);
    uvMax = uvMin + glm::vec2(stepUV, stepUV);
}

void TiledTerrain::rebuild()
{
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
                    glm::vec2 uvMin, uvMax;
                    atlasUV(tileId, mTilesInSide, uvMin, uvMax);

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
                    data.uvs.push_back(glm::vec2(uvMin.x, uvMin.y));
                    data.uvs.push_back(glm::vec2(uvMax.x, uvMin.y));
                    data.uvs.push_back(glm::vec2(uvMin.x, uvMax.y));
                    data.uvs.push_back(glm::vec2(uvMax.x, uvMax.y));

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
    if (!mAtlasMaterial.empty())
    {
        std::vector<Material> loaded;
        if (MaterialManager::getSingleton().load(mAtlasMaterial, loaded) && !loaded.empty())
            material = loaded.front();
        else
            material.name = mAtlasMaterial;
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
        mRenderer = owner()->addComponent<MeshRenderer>(mMesh);
    else
        mRenderer->setMesh(mMesh);

    Log::info("TiledTerrain: %ux%u tilemap -> %u patches", mMapWidth, mMapHeight, mPatchCount);
}

} // namespace Radion
