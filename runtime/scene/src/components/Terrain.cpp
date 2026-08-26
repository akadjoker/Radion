#include "PCH.h"

#include "Terrain.h"

#include "AssetManager.h"
#include "FileSystem.h"
#include "Forest.h"
#include "GameObject.h"
#include "Grass.h"
#include "MaterialManager.h"
#include "RenderList.h"
#include "Timer.h"

namespace Radion
{

namespace
{
u32 terrainHash(u32 value)
{
    value ^= value >> 16;
    value *= 0x7feb352du;
    value ^= value >> 15;
    value *= 0x846ca68bu;
    value ^= value >> 16;
    return value;
}

f32 terrainRandom(u32 value)
{
    return static_cast<f32>(terrainHash(value) >> 8) / 16777216.0f;
}

struct TerrainLod
{
    u32 indexOffset = 0;
    u32 indexCount = 0;
};

struct TerrainIndices
{
    BufferHandle buffer;
    std::vector<TerrainLod> lods;

    static TerrainIndices& instance()
    {
        static TerrainIndices state = build();
        return state;
    }

private:
    static TerrainIndices build()
    {
        TerrainIndices state;

        constexpr s32 width = static_cast<s32>(Terrain::ChunkWidth);
        // log2(64) + 1 = 7: one LOD per halving of the interior's
        // resolution, down to the coarsest step that still fits inside it.
        const s32 maxLod = static_cast<s32>(std::log2(width - 3)) + 1;

        std::vector<u32> indices;
        state.lods.resize(static_cast<usize>(maxLod));

        for (s32 lod = 0; lod < maxLod; ++lod)
        {
            state.lods[static_cast<usize>(lod)].indexOffset = static_cast<u32>(indices.size());

            if (lod == 0)
            {
                for (s32 x = 0; x < width - 1; ++x)
                {
                    for (s32 z = 0; z < width - 1; ++z)
                    {
                        const s32 lowerLeft = x + z * width;
                        const s32 lowerRight = (x + 1) + z * width;
                        const s32 topLeft = x + (z + 1) * width;
                        const s32 topRight = (x + 1) + (z + 1) * width;

                        indices.push_back(static_cast<u32>(topLeft));
                        indices.push_back(static_cast<u32>(lowerLeft));
                        indices.push_back(static_cast<u32>(lowerRight));
                        indices.push_back(static_cast<u32>(topLeft));
                        indices.push_back(static_cast<u32>(lowerRight));
                        indices.push_back(static_cast<u32>(topRight));
                    }
                }
            }
            else
            {
                const s32 step = 1 << lod;

                for (s32 x = 1; x < width - 2; x += step)
                {
                    for (s32 z = 1; z < width - 2; z += step)
                    {
                        const s32 lowerLeft = x + z * width;
                        const s32 lowerRight = (x + step) + z * width;
                        const s32 topLeft = x + (z + step) * width;
                        const s32 topRight = (x + step) + (z + step) * width;

                        indices.push_back(static_cast<u32>(topLeft));
                        indices.push_back(static_cast<u32>(lowerLeft));
                        indices.push_back(static_cast<u32>(lowerRight));
                        indices.push_back(static_cast<u32>(topLeft));
                        indices.push_back(static_cast<u32>(lowerRight));
                        indices.push_back(static_cast<u32>(topRight));
                    }
                }

                auto fillTriangle = [&](s32 midStep, s32 current, s32 neighbour, s32 connection,
                                        s32 connectionPrev, bool flip)
                {
                    if (flip)
                    {
                        indices.push_back(static_cast<u32>(current));
                        indices.push_back(static_cast<u32>(connection));
                        indices.push_back(static_cast<u32>(neighbour));
                    }
                    else
                    {
                        indices.push_back(static_cast<u32>(current));
                        indices.push_back(static_cast<u32>(neighbour));
                        indices.push_back(static_cast<u32>(connection));
                    }
                    if (midStep == step / 2)
                    {
                        if (flip)
                        {
                            indices.push_back(static_cast<u32>(current));
                            indices.push_back(static_cast<u32>(connectionPrev));
                            indices.push_back(static_cast<u32>(connection));
                        }
                        else
                        {
                            indices.push_back(static_cast<u32>(current));
                            indices.push_back(static_cast<u32>(connection));
                            indices.push_back(static_cast<u32>(connectionPrev));
                        }
                    }
                };

                for (s32 x = 0; x < width - 1; ++x)
                {
                    const s32 z = 0;
                    const s32 current = x + z * width;
                    const s32 neighbour = x + 1 + z * width;
                    const s32 connection =
                        1 + ((x + (step + 1) / 2 - 1) / step) * step + (z + 1) * width;
                    const s32 connectionPrev =
                        1 + (((x - 1) + (step + 1) / 2 - 1) / step) * step + (z + 1) * width;
                    fillTriangle((x - 1) % step, current, neighbour, connection, connectionPrev,
                                false);
                }
                for (s32 x = 0; x < width - 1; ++x)
                {
                    const s32 z = width - 1;
                    const s32 current = x + z * width;
                    const s32 neighbour = x + 1 + z * width;
                    const s32 connection =
                        1 + ((x + (step + 1) / 2 - 1) / step) * step + (z - 1) * width;
                    const s32 connectionPrev =
                        1 + (((x - 1) + (step + 1) / 2 - 1) / step) * step + (z - 1) * width;
                    fillTriangle((x - 1) % step, current, neighbour, connection, connectionPrev,
                                true);
                }
                for (s32 z = 0; z < width - 1; ++z)
                {
                    const s32 x = 0;
                    const s32 current = x + z * width;
                    const s32 neighbour = x + (z + 1) * width;
                    const s32 connection =
                        x + 1 + (((z + (step + 1) / 2 - 1) / step) * step + 1) * width;
                    const s32 connectionPrev =
                        x + 1 + ((((z - 1) + (step + 1) / 2 - 1) / step) * step + 1) * width;
                    fillTriangle((z - 1) % step, current, neighbour, connection, connectionPrev,
                                true);
                }
                for (s32 z = 0; z < width - 1; ++z)
                {
                    const s32 x = width - 1;
                    const s32 current = x + z * width;
                    const s32 neighbour = x + (z + 1) * width;
                    const s32 connection =
                        x - 1 + (((z + (step + 1) / 2 - 1) / step) * step + 1) * width;
                    const s32 connectionPrev =
                        x - 1 + ((((z - 1) + (step + 1) / 2 - 1) / step) * step + 1) * width;
                    fillTriangle((z - 1) % step, current, neighbour, connection, connectionPrev,
                                false);
                }
            }

            state.lods[static_cast<usize>(lod)].indexCount =
                static_cast<u32>(indices.size()) - state.lods[static_cast<usize>(lod)].indexOffset;
        }

        // Front-face convention only, same as Landscape's: swap the last two
        // indices of every triangle once, here, rather than flip GL's winding
        // state around every draw that uses this buffer.
        for (usize i = 0; i + 2 < indices.size(); i += 3)
            std::swap(indices[i + 1], indices[i + 2]);

        GPU& gpu = GPU::getSingleton();
        BufferDesc desc;
        desc.size = indices.size() * sizeof(u32);
        desc.usage = BufferIndex;
        desc.residency = Residency::Static;
        desc.stride = sizeof(u32);
        desc.data = indices.data();
        desc.debugName = "terrain.indices";
        state.buffer = gpu.createBuffer(desc);

        Log::info("Terrain: %d LODs, %zu shared indices (%zu KB)", maxLod, indices.size(),
                  indices.size() * sizeof(u32) / 1024);
        return state;
    }
};

} // namespace

Terrain::Terrain() : Component(Type)
{
    mMaterial.params.baseColor = Math::Vec4(1.0f);
    mMaterial.params.surface.x = 0.88f;
    mMaterial.params.uvTransform = Math::Vec4(1.0f, 1.0f, 0.0f, 0.0f);
    // low height end, snow start, cliff blend start/end
    mMaterial.params.custom0 = Math::Vec4(0.16f, 0.72f, 0.08f, 0.32f);
    // macro tiles/strength, splat strength, rock triplanar world scale
    mMaterial.params.custom1 = Math::Vec4(1.0f, 0.32f, 1.0f, 0.12f);
    mMaterial.cull = CullMode::Back;
    // The real forward pipeline, not the flat unlit path: terrain takes
    // shadows and local lights the same way everything else does.
    mMaterial.flags |= MaterialLit | MaterialTerrain | MaterialReceiveShadow;

    mTreeGeneration.spacing = 9.0f;
    mTreeGeneration.density = 0.22f;
    mTreeGeneration.jitter = 0.9f;
    mTreeGeneration.maximumSlopeDegrees = 26.0f;
    mTreeGeneration.minimumScale = 0.8f;
    mTreeGeneration.maximumScale = 1.3f;
    mTreeGeneration.seed = 7919;
    mTreeGeneration.maximumInstances = 12000;
}

void Terrain::onDestroy()
{
    clear();
}

bool Terrain::load(const Pixmap& heightmap, f32 cellSize, f32 heightScale, u32 maxLod,
                   f32 uvTiles)
{
    Timer generationTimer;
    if (!heightmap.is_valid() || heightmap.width < 2 || heightmap.height < 2 ||
        cellSize <= 0.0f || heightScale < 0.0f ||
        maxLod == 0 || !std::isfinite(uvTiles) || uvTiles <= 0.0f)
    {
        Log::error("Terrain: invalid heightmap or LOD settings");
        return false;
    }

    const u32 width = static_cast<u32>(heightmap.width);
    const u32 height = static_cast<u32>(heightmap.height);
    if ((width - 1) % ChunkSpan != 0 || (height - 1) % ChunkSpan != 0)
    {
        Log::error("Terrain: both heightmap dimensions minus one must be multiples of %u "
                   "(got %ux%u)", ChunkSpan, width, height);
        return false;
    }

    clear();
    mWidth = width;
    mHeight = height;
    mCellSize = cellSize;
    mHeightScale = glm::max(heightScale, 0.0001f);
    mUvTiles = uvTiles;
    mMaterial.params.uvTransform.x = uvTiles;
    mMaterial.params.uvTransform.y = uvTiles;
    mMaterial.paramsDirty = true;
    mChunkCountX = (width - 1) / ChunkSpan;
    mChunkCountZ = (height - 1) / ChunkSpan;

    const TerrainIndices& indices = TerrainIndices::instance();
    mMaxLod = glm::min(maxLod, static_cast<u32>(indices.lods.size()));

    const usize vertexCount = static_cast<usize>(width) * height;
    mHeights.resize(vertexCount);
    const f32 halfX = static_cast<f32>(width - 1) * cellSize * 0.5f;
    const f32 halfZ = static_cast<f32>(height - 1) * cellSize * 0.5f;
    f32 minH = 1e30f;
    f32 maxH = -1e30f;
    for (u32 z = 0; z < height; ++z)
    {
        for (u32 x = 0; x < width; ++x)
        {
            const Color pixel = heightmap.get_pixel_color(x, z);
            const f32 luminance =
                pixel.red() * 0.299f + pixel.green() * 0.587f + pixel.blue() * 0.114f;
            const f32 h = luminance * heightScale;
            mHeights[vertexIndex(x, z)] = h;
            minH = glm::min(minH, h);
            maxH = glm::max(maxH, h);
        }
    }
    mBounds = AABB();
    mBounds.expand(Math::Vec3(-halfX, minH, -halfZ));
    mBounds.expand(Math::Vec3(halfX, maxH, halfZ));

    mChunks.resize(static_cast<usize>(mChunkCountX) * mChunkCountZ);
    for (u32 cz = 0; cz < mChunkCountZ; ++cz)
        for (u32 cx = 0; cx < mChunkCountX; ++cx)
            buildChunk(cx, cz);

    ++mRevision;
    mCellSize = cellSize;
    mHeightScale = glm::max(heightScale, 0.0001f);
    generationTimer.tick();
    Log::info("Terrain: generated %ux%u, %u chunks, %u LODs in %.3f ms", mWidth, mHeight,
              static_cast<u32>(mChunks.size()), mMaxLod,
              generationTimer.getElapsedTime() * 1000.0);
    return true;
}

bool Terrain::loadFile(const char* filename, f32 cellSize, f32 heightScale, u32 maxLod,
                       f32 uvTiles)
{
    if (!filename || !*filename)
        return false;
    Pixmap heightmap;
    if (!heightmap.load(filename))
        return false;
    if (!load(heightmap, cellSize, heightScale, maxLod, uvTiles))
        return false;
    mHeightmapFile = filename;
    return true;
}

bool Terrain::loadRaw(const char* filename, u32 size, f32 cellSize, f32 heightScale, u32 maxLod,
                      f32 uvTiles)
{
    if (!filename || size < 2)
        return false;
    ByteArray bytes = FileSystem::getSingleton().readBinary(filename);
    const usize expected = static_cast<usize>(size) * size;
    if (bytes.size() != expected)
    {
        Log::error("Terrain: RAW '%s' has %u bytes, expected %u", filename,
                   static_cast<u32>(bytes.size()), static_cast<u32>(expected));
        return false;
    }
    Pixmap image(static_cast<int>(size), static_cast<int>(size), 1);
    std::memcpy(image.pixels, bytes.data(), expected);
    return load(image, cellSize, heightScale, maxLod, uvTiles);
}

bool Terrain::saveHeightmap(const char* filename)
{
    if (!filename || !valid() || mWidth == 0 || mHeight == 0)
        return false;
    Pixmap image(static_cast<int>(mWidth), static_cast<int>(mHeight), 1);
    for (usize i = 0; i < mHeights.size(); ++i)
        image.pixels[i] =
            static_cast<u8>(glm::clamp(mHeights[i] / mHeightScale, 0.0f, 1.0f) * 255.0f + 0.5f);

    const std::string path(filename);
    if (path.size() >= 4 && path.substr(path.size() - 4) == ".raw")
    {
        ByteArray bytes;
        bytes.writeBytes(image.pixels, static_cast<usize>(mWidth) * mHeight);
        const bool saved = FileSystem::getSingleton().writeBinary(path, bytes);
        if (saved)
            mHeightmapFile = filename;
        return saved;
    }
    const bool saved = image.save(filename);
    if (saved)
        mHeightmapFile = filename;
    return saved;
}

void Terrain::clear()
{
    discardSurfaceSplat();
    for (Chunk& chunk : mChunks)
        releaseChunk(chunk);
    mChunks.clear();
    mHeights.clear();
    mBounds = AABB();
    mWidth = mHeight = mChunkCountX = mChunkCountZ = mMaxLod = 0;
    mVisibleChunks = 0;
    mHeightmapFile.clear();
    mVegetationMask.reset();
    mVegetationMaskFile.clear();
}

bool Terrain::valid() const
{
    return mWidth > 0 && !mChunks.empty();
}

void Terrain::setReceiveShadows(bool enabled)
{
    if (enabled)
        mMaterial.flags |= MaterialReceiveShadow;
    else
        mMaterial.flags &= ~MaterialReceiveShadow;
    // The flag is in the pipeline key, so the next resolvePipeline() picks the
    // other variant up. Dropping the handle is what makes it re-resolve.
    mMaterial.pipeline = PipelineHandle();
}

bool Terrain::receivesShadows() const
{
    return (mMaterial.flags & MaterialReceiveShadow) != 0;
}

f32 Terrain::cellSize() const { return mCellSize; }
f32 Terrain::heightScale() const { return mHeightScale; }
f32 Terrain::uvTiles() const { return mUvTiles; }
void Terrain::setUvTiles(f32 tiles)
{
    if (!std::isfinite(tiles) || tiles <= 0.0f)
        return;
    mUvTiles = tiles;
    mMaterial.params.uvTransform.x = tiles;
    mMaterial.params.uvTransform.y = tiles;
    mMaterial.paramsDirty = true;
}
u32 Terrain::maxLod() const { return mMaxLod; }
const std::string& Terrain::heightmapFile() const { return mHeightmapFile; }

void Terrain::setSurfaceMode(SurfaceMode mode)
{
    if (mode == mSurfaceMode)
        return;

    // The outgoing mode's live custom0 goes back to its own copy first: a
    // panel editing mMaterial.params directly is the normal case, so the
    // register is the truth here, not what was saved on the last switch.
    if (mSurfaceMode == SurfaceMode::Layers)
        mLayerParams = mMaterial.params.custom0;
    else
        mClassicParams = mMaterial.params.custom0;

    mSurfaceMode = mode;
    if (mode == SurfaceMode::Classic)
    {
        mMaterial.params.custom0 = mClassicParams;
        mMaterial.flags |= MaterialTerrainClassic;
    }
    else
    {
        mMaterial.params.custom0 = mLayerParams;
        mMaterial.flags &= ~MaterialTerrainClassic;
    }
    mMaterial.paramsDirty = true;
    // A different define, so a different program: drop the cached pipeline
    // and let the next draw resolve the variant this mode compiles to.
    mMaterial.pipeline = PipelineHandle();
}

Terrain::SurfaceMode Terrain::surfaceMode() const
{
    return mSurfaceMode;
}

void Terrain::setDetailTiling(f32 tiles)
{
    const f32 clamped = glm::max(tiles, 1.0f);
    mClassicParams.x = clamped;
    if (mSurfaceMode != SurfaceMode::Classic)
        return;
    mMaterial.params.custom0.x = clamped;
    mMaterial.paramsDirty = true;
}

f32 Terrain::detailTiling() const
{
    return mSurfaceMode == SurfaceMode::Classic ? mMaterial.params.custom0.x : mClassicParams.x;
}

void Terrain::setDetailStrength(f32 strength)
{
    const f32 clamped = glm::clamp(strength, 0.0f, 1.0f);
    mClassicParams.y = clamped;
    if (mSurfaceMode != SurfaceMode::Classic)
        return;
    mMaterial.params.custom0.y = clamped;
    mMaterial.paramsDirty = true;
}

f32 Terrain::detailStrength() const
{
    return mSurfaceMode == SurfaceMode::Classic ? mMaterial.params.custom0.y : mClassicParams.y;
}

const Math::Vec4& Terrain::layerThresholds() const
{
    return mSurfaceMode == SurfaceMode::Layers ? mMaterial.params.custom0 : mLayerParams;
}

Material& Terrain::material()
{
    return mMaterial;
}
const Material& Terrain::material() const
{
    return mMaterial;
}

bool Terrain::createVegetationMask(u32 width, u32 height)
{
    if (!valid())
        return false;
    width = width == 0 ? mWidth : width;
    height = height == 0 ? mHeight : height;
    if (width < 2 || height < 2 || width > static_cast<u32>(std::numeric_limits<int>::max()) ||
        height > static_cast<u32>(std::numeric_limits<int>::max()))
        return false;

    mVegetationMask = std::make_unique<Pixmap>(static_cast<int>(width),
                                                static_cast<int>(height), 4);
    if (!mVegetationMask->is_valid())
    {
        mVegetationMask.reset();
        return false;
    }
    mVegetationMask->fill(0, 0, 0, 0);
    mVegetationMaskFile.clear();
    return true;
}

bool Terrain::loadVegetationMask(const char* filename)
{
    if (!filename || !*filename)
        return false;
    Pixmap source;
    if (!source.load(filename) || source.width < 2 || source.height < 2)
        return false;
    std::unique_ptr<Pixmap> rgba = std::make_unique<Pixmap>(source.width, source.height, 4);
    if (!rgba || !rgba->is_valid())
        return false;
    const bool sourceHasAlpha = source.components == 4;
    for (s32 y = 0; y < source.height; ++y)
        for (s32 x = 0; x < source.width; ++x)
        {
            const Color color = source.get_pixel_color(static_cast<u32>(x), static_cast<u32>(y));
            // An ordinary RGB mask remains useful for R/G/B without
            // accidentally turning every pixel into a tree through the
            // implicit opaque alpha that image loaders normally add.
            rgba->set_pixel(static_cast<u32>(x), static_cast<u32>(y), color.r(), color.g(),
                            color.b(), sourceHasAlpha ? color.a() : 0);
        }
    mVegetationMask = std::move(rgba);
    mVegetationMaskFile = filename;
    return true;
}

bool Terrain::saveVegetationMask(const char* filename)
{
    if (!filename || !*filename || !mVegetationMask || !mVegetationMask->is_valid() ||
        !mVegetationMask->save(filename))
        return false;
    mVegetationMaskFile = filename;
    return true;
}

void Terrain::discardVegetationMask()
{
    if (!mVegetationMask)
        return;
    mVegetationMask.reset();
    mVegetationMaskFile.clear();
}

bool Terrain::hasVegetationMask() const
{
    return mVegetationMask && mVegetationMask->is_valid() && mVegetationMask->components == 4;
}

const std::string& Terrain::vegetationMaskFile() const
{
    return mVegetationMaskFile;
}

void Terrain::fillVegetation(VegetationChannel channel, f32 density)
{
    if (!hasVegetationMask())
        return;
    const u32 component = static_cast<u32>(channel);
    const u8 value = static_cast<u8>(glm::clamp(density, 0.0f, 1.0f) * 255.0f + 0.5f);
    const usize count = static_cast<usize>(mVegetationMask->width) * mVegetationMask->height;
    for (usize i = 0; i < count; ++i)
        mVegetationMask->pixels[i * 4 + component] = value;
    mVegetationMaskFile.clear();
}

bool Terrain::paintVegetation(const Math::Vec3& worldCenter, f32 radius,
                              VegetationChannel channel, f32 strength, bool erase)
{
    if (!valid() || !owner() || !hasVegetationMask() || radius <= 0.0f || strength <= 0.0f)
        return false;
    const Math::Vec3 center = Math::Vec3(
        glm::inverse(owner()->globalTransform()) * Math::Vec4(worldCenter, 1.0f));
    const f32 sizeX = static_cast<f32>(mWidth - 1) * mCellSize;
    const f32 sizeZ = static_cast<f32>(mHeight - 1) * mCellSize;
    const f32 halfX = sizeX * 0.5f;
    const f32 halfZ = sizeZ * 0.5f;
    const f32 pixelScaleX = static_cast<f32>(mVegetationMask->width - 1) / sizeX;
    const f32 pixelScaleZ = static_cast<f32>(mVegetationMask->height - 1) / sizeZ;
    const s32 minX = glm::clamp(static_cast<s32>(std::floor((center.x - radius + halfX) * pixelScaleX)),
                                0, mVegetationMask->width - 1);
    const s32 maxX = glm::clamp(static_cast<s32>(std::ceil((center.x + radius + halfX) * pixelScaleX)),
                                0, mVegetationMask->width - 1);
    const s32 minZ = glm::clamp(static_cast<s32>(std::floor((center.z - radius + halfZ) * pixelScaleZ)),
                                0, mVegetationMask->height - 1);
    const s32 maxZ = glm::clamp(static_cast<s32>(std::ceil((center.z + radius + halfZ) * pixelScaleZ)),
                                0, mVegetationMask->height - 1);
    if (minX > maxX || minZ > maxZ)
        return false;

    const u32 component = static_cast<u32>(channel);
    bool changed = false;
    for (s32 z = minZ; z <= maxZ; ++z)
        for (s32 x = minX; x <= maxX; ++x)
        {
            const f32 localX = static_cast<f32>(x) / pixelScaleX - halfX;
            const f32 localZ = static_cast<f32>(z) / pixelScaleZ - halfZ;
            const f32 distance = glm::length(Math::Vec2(localX - center.x, localZ - center.z));
            if (distance > radius)
                continue;
            const f32 falloff = 1.0f - glm::smoothstep(0.0f, radius, distance);
            u8& sample = mVegetationMask->pixels[
                (static_cast<usize>(z) * mVegetationMask->width + x) * 4 + component];
            const f32 oldValue = static_cast<f32>(sample) / 255.0f;
            const f32 delta = glm::clamp(strength * falloff, 0.0f, 1.0f);
            const f32 newValue = erase ? glm::max(0.0f, oldValue - delta)
                                       : glm::min(1.0f, oldValue + delta);
            const u8 encoded = static_cast<u8>(newValue * 255.0f + 0.5f);
            changed |= encoded != sample;
            sample = encoded;
        }
    if (changed)
        mVegetationMaskFile.clear();
    return changed;
}

f32 Terrain::vegetationDensity(f32 localX, f32 localZ, VegetationChannel channel) const
{
    // No mask means the legacy whole-terrain scatter, so existing scenes do
    // not silently lose vegetation when loaded by the newer component.
    if (!hasVegetationMask() || !valid())
        return 1.0f;
    const f32 sizeX = static_cast<f32>(mWidth - 1) * mCellSize;
    const f32 sizeZ = static_cast<f32>(mHeight - 1) * mCellSize;
    const f32 u = localX / sizeX + 0.5f;
    const f32 v = localZ / sizeZ + 0.5f;
    if (u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f)
        return 0.0f;
    const f32 px = u * static_cast<f32>(mVegetationMask->width - 1);
    const f32 pz = v * static_cast<f32>(mVegetationMask->height - 1);
    const u32 x0 = static_cast<u32>(std::floor(px));
    const u32 z0 = static_cast<u32>(std::floor(pz));
    const u32 x1 = glm::min(x0 + 1, static_cast<u32>(mVegetationMask->width - 1));
    const u32 z1 = glm::min(z0 + 1, static_cast<u32>(mVegetationMask->height - 1));
    const u32 component = static_cast<u32>(channel);
    auto sample = [&](u32 x, u32 z) {
        return static_cast<f32>(mVegetationMask->pixels[
                   (static_cast<usize>(z) * mVegetationMask->width + x) * 4 + component]) /
               255.0f;
    };
    const f32 top = glm::mix(sample(x0, z0), sample(x1, z0), px - x0);
    const f32 bottom = glm::mix(sample(x0, z1), sample(x1, z1), px - x0);
    return glm::mix(top, bottom, pz - z0);
}

Math::Vec4 Terrain::automaticSurfaceWeights(f32 localX, f32 localZ) const
{
    const f32 height01 = glm::clamp(heightAt(localX, localZ) / mHeightScale, 0.0f, 1.0f);
    const f32 slope = 1.0f - glm::clamp(normalAt(localX, localZ).y, 0.0f, 1.0f);
    const Math::Vec4& thresholds = layerThresholds();
    const f32 lowEnd = glm::clamp(thresholds.x, 0.001f, 0.999f);
    const f32 highStart = glm::clamp(thresholds.y, 0.001f, 0.999f);
    const f32 slopeStart = glm::clamp(thresholds.z, 0.0f, 0.999f);
    const f32 slopeEnd = glm::max(slopeStart + 0.001f, thresholds.w);
    const f32 rock = glm::smoothstep(slopeStart, slopeEnd, slope);
    const f32 low = (1.0f - glm::smoothstep(0.0f, lowEnd, height01)) * (1.0f - rock);
    const f32 high = glm::smoothstep(highStart, 1.0f, height01) * (1.0f - rock);
    const f32 base = glm::max(0.0f, 1.0f - rock - low - high);
    Math::Vec4 weights(base, rock, low, high);
    return weights / glm::max(glm::dot(weights, Math::Vec4(1.0f)), 0.0001f);
}

void Terrain::uploadSurfaceSplat(bool recreate)
{
    if (!hasSurfaceSplat() || !GPU::ready())
        return;
    GPU& gpu = GPU::getSingleton();
    if (recreate && mSurfaceSplatTexture.valid())
    {
        gpu.destroy(mSurfaceSplatTexture);
        mSurfaceSplatTexture = TextureHandle();
    }
    const bool createTexture = !mSurfaceSplatTexture.valid();
    if (createTexture)
    {
        TextureDesc desc;
        desc.type = TextureType::Tex2D;
        desc.format = Format::RGBA8;
        desc.width = static_cast<u32>(mSurfaceSplat->width);
        desc.height = static_cast<u32>(mSurfaceSplat->height);
        desc.mips = 1;
        desc.usage = TextureSampled;
        desc.data = mSurfaceSplat->pixels;
        desc.debugName = "terrain.surface_splat";
        mSurfaceSplatTexture = gpu.createTexture(desc);
    }
    else
        gpu.updateTexture(mSurfaceSplatTexture, 0, 0, 0, 0,
                          static_cast<u32>(mSurfaceSplat->width),
                          static_cast<u32>(mSurfaceSplat->height), mSurfaceSplat->pixels);

    MaterialTexture& slot = mMaterial.textures[SlotColorMap];
    slot.texture = mSurfaceSplatTexture;
    SamplerDesc sampler;
    sampler.filter = Filter::Linear;
    sampler.wrapU = Wrap::Clamp;
    sampler.wrapV = Wrap::Clamp;
    slot.sampler = Assets().getSampler(sampler);
    slot.source = TextureSource::Static;
    mMaterial.params.custom1.z = 1.0f;
    mMaterial.paramsDirty = true;
    // Adding SlotColorMap changes HAS_COLORMAP and therefore the pipeline;
    // ordinary brush uploads keep using the already-correct variant.
    if (createTexture)
        mMaterial.pipeline = PipelineHandle();
}

bool Terrain::createSurfaceSplat(u32 width, u32 height)
{
    if (!valid())
        return false;
    width = width == 0 ? mWidth : width;
    height = height == 0 ? mHeight : height;
    if (width < 2 || height < 2 || width > static_cast<u32>(std::numeric_limits<int>::max()) ||
        height > static_cast<u32>(std::numeric_limits<int>::max()))
        return false;
    mSurfaceSplat = std::make_unique<Pixmap>(static_cast<int>(width), static_cast<int>(height), 4);
    if (!mSurfaceSplat->is_valid())
    {
        mSurfaceSplat.reset();
        return false;
    }
    const f32 sizeX = static_cast<f32>(mWidth - 1) * mCellSize;
    const f32 sizeZ = static_cast<f32>(mHeight - 1) * mCellSize;
    for (u32 z = 0; z < height; ++z)
        for (u32 x = 0; x < width; ++x)
        {
            const f32 localX = static_cast<f32>(x) / (width - 1) * sizeX - sizeX * 0.5f;
            const f32 localZ = static_cast<f32>(z) / (height - 1) * sizeZ - sizeZ * 0.5f;
            const Math::Vec4 w = automaticSurfaceWeights(localX, localZ) * 255.0f;
            mSurfaceSplat->set_pixel(x, z, static_cast<u8>(w.x + 0.5f),
                                    static_cast<u8>(w.y + 0.5f),
                                    static_cast<u8>(w.z + 0.5f),
                                    static_cast<u8>(w.w + 0.5f));
        }
    mSurfaceSplatFile.clear();
    uploadSurfaceSplat(true);
    return true;
}

bool Terrain::loadSurfaceSplat(const char* filename)
{
    if (!filename || !*filename)
        return false;
    Pixmap source;
    if (!source.load(filename) || source.width < 2 || source.height < 2)
        return false;
    std::unique_ptr<Pixmap> rgba(source.convert_to_rgba());
    if (!rgba || !rgba->is_valid())
        return false;
    for (s32 y = 0; y < rgba->height; ++y)
        for (s32 x = 0; x < rgba->width; ++x)
        {
            Color c = rgba->get_pixel_color(static_cast<u32>(x), static_cast<u32>(y));
            const u32 sum = c.r() + c.g() + c.b() + c.a();
            if (sum == 0)
                rgba->set_pixel(static_cast<u32>(x), static_cast<u32>(y), 255, 0, 0, 0);
        }
    mSurfaceSplat = std::move(rgba);
    mSurfaceSplatFile = filename;
    uploadSurfaceSplat(true);
    return true;
}

bool Terrain::saveSurfaceSplat(const char* filename)
{
    if (!filename || !*filename || !hasSurfaceSplat() || !mSurfaceSplat->save(filename))
        return false;
    mSurfaceSplatFile = filename;
    mMaterial.textures[SlotColorMap].file = filename;
    return true;
}

void Terrain::discardSurfaceSplat()
{
    if (mSurfaceSplatTexture.valid())
        if (GPU* gpu = GPU::tryGet())
            gpu->destroy(mSurfaceSplatTexture);
    mSurfaceSplatTexture = TextureHandle();
    mSurfaceSplat.reset();
    mSurfaceSplatFile.clear();
    MaterialTexture& slot = mMaterial.textures[SlotColorMap];
    slot = MaterialTexture();
    mMaterial.params.custom1.z = 0.0f;
    mMaterial.paramsDirty = true;
    mMaterial.pipeline = PipelineHandle();
}

bool Terrain::hasSurfaceSplat() const
{
    return mSurfaceSplat && mSurfaceSplat->is_valid() && mSurfaceSplat->components == 4;
}

const std::string& Terrain::surfaceSplatFile() const
{
    return mSurfaceSplatFile;
}

bool Terrain::paintSurface(const Math::Vec3& worldCenter, f32 radius, SurfaceLayer layer,
                           f32 strength, bool restoreAutomatic)
{
    if (!valid() || !owner() || !hasSurfaceSplat() || radius <= 0.0f || strength <= 0.0f)
        return false;
    const Math::Vec3 center = Math::Vec3(
        glm::inverse(owner()->globalTransform()) * Math::Vec4(worldCenter, 1.0f));
    const f32 sizeX = static_cast<f32>(mWidth - 1) * mCellSize;
    const f32 sizeZ = static_cast<f32>(mHeight - 1) * mCellSize;
    const f32 halfX = sizeX * 0.5f;
    const f32 halfZ = sizeZ * 0.5f;
    const f32 pixelScaleX = static_cast<f32>(mSurfaceSplat->width - 1) / sizeX;
    const f32 pixelScaleZ = static_cast<f32>(mSurfaceSplat->height - 1) / sizeZ;
    const s32 minX = glm::clamp(static_cast<s32>(std::floor((center.x - radius + halfX) * pixelScaleX)),
                                0, mSurfaceSplat->width - 1);
    const s32 maxX = glm::clamp(static_cast<s32>(std::ceil((center.x + radius + halfX) * pixelScaleX)),
                                0, mSurfaceSplat->width - 1);
    const s32 minZ = glm::clamp(static_cast<s32>(std::floor((center.z - radius + halfZ) * pixelScaleZ)),
                                0, mSurfaceSplat->height - 1);
    const s32 maxZ = glm::clamp(static_cast<s32>(std::ceil((center.z + radius + halfZ) * pixelScaleZ)),
                                0, mSurfaceSplat->height - 1);
    Math::Vec4 target(0.0f);
    target[static_cast<u32>(layer)] = 1.0f;
    bool changed = false;
    for (s32 z = minZ; z <= maxZ; ++z)
        for (s32 x = minX; x <= maxX; ++x)
        {
            const f32 localX = static_cast<f32>(x) / pixelScaleX - halfX;
            const f32 localZ = static_cast<f32>(z) / pixelScaleZ - halfZ;
            const f32 distance = glm::length(Math::Vec2(localX - center.x, localZ - center.z));
            if (distance > radius)
                continue;
            const f32 falloff = 1.0f - glm::smoothstep(0.0f, radius, distance);
            const usize offset = (static_cast<usize>(z) * mSurfaceSplat->width + x) * 4;
            Math::Vec4 oldValue(mSurfaceSplat->pixels[offset + 0], mSurfaceSplat->pixels[offset + 1],
                               mSurfaceSplat->pixels[offset + 2], mSurfaceSplat->pixels[offset + 3]);
            oldValue /= 255.0f;
            oldValue /= glm::max(glm::dot(oldValue, Math::Vec4(1.0f)), 0.0001f);
            const Math::Vec4 desired = restoreAutomatic
                                          ? automaticSurfaceWeights(localX, localZ)
                                          : target;
            Math::Vec4 value = glm::mix(oldValue, desired,
                                       glm::clamp(strength * falloff, 0.0f, 1.0f));
            value /= glm::max(glm::dot(value, Math::Vec4(1.0f)), 0.0001f);
            for (u32 component = 0; component < 4; ++component)
            {
                const u8 encoded = static_cast<u8>(value[component] * 255.0f + 0.5f);
                changed |= encoded != mSurfaceSplat->pixels[offset + component];
                mSurfaceSplat->pixels[offset + component] = encoded;
            }
        }
    if (changed)
    {
        mSurfaceSplatFile.clear();
        mMaterial.textures[SlotColorMap].file.clear();
        uploadSurfaceSplat(false);
    }
    return changed;
}
u32 Terrain::width() const
{
    return mWidth;
}
u32 Terrain::height() const
{
    return mHeight;
}
u32 Terrain::patchCount() const
{
    return static_cast<u32>(mChunks.size());
}
u32 Terrain::visiblePatchCount() const
{
    return mVisibleChunks;
}
u32 Terrain::triangleCount() const
{
    u32 triangles = 0;
    for (const Chunk& chunk : mChunks)
    {
        if (!chunk.visible)
            continue;
        const TerrainIndices& indices = TerrainIndices::instance();
        if (chunk.lastLod < indices.lods.size())
            triangles += indices.lods[chunk.lastLod].indexCount / 3;
    }
    return triangles;
}
u64 Terrain::revision() const
{
    return mRevision;
}

u32 Terrain::vertexIndex(u32 x, u32 z) const
{
    return z * mWidth + x;
}

f32 Terrain::sampleHeight(s32 x, s32 z) const
{
    x = glm::clamp(x, 0, static_cast<s32>(mWidth) - 1);
    z = glm::clamp(z, 0, static_cast<s32>(mHeight) - 1);
    return mHeights[vertexIndex(static_cast<u32>(x), static_cast<u32>(z))];
}

f32 Terrain::heightAt(f32 localX, f32 localZ) const
{
    if (!valid())
        return 0.0f;
    const f32 halfX = static_cast<f32>(mWidth - 1) * mCellSize * 0.5f;
    const f32 halfZ = static_cast<f32>(mHeight - 1) * mCellSize * 0.5f;
    const f32 gx = (localX + halfX) / mCellSize;
    const f32 gz = (localZ + halfZ) / mCellSize;
    if (gx < 0.0f || gz < 0.0f || gx > mWidth - 1 || gz > mHeight - 1)
        return 0.0f;
    const u32 x0 = glm::min(static_cast<u32>(gx), mWidth - 2);
    const u32 z0 = glm::min(static_cast<u32>(gz), mHeight - 2);
    const f32 tx = gx - static_cast<f32>(x0);
    const f32 tz = gz - static_cast<f32>(z0);
    const f32 h00 = mHeights[vertexIndex(x0, z0)];
    const f32 h10 = mHeights[vertexIndex(x0 + 1, z0)];
    const f32 h01 = mHeights[vertexIndex(x0, z0 + 1)];
    const f32 h11 = mHeights[vertexIndex(x0 + 1, z0 + 1)];

    // The shared index buffer splits a cell along top-left -> bottom-right.
    // Query the same two planes the renderer draws rather than a bilinear
    // surface which can visibly float above or below a road/brush cursor.
    if (tx + tz <= 1.0f)
        return h00 + (h10 - h00) * tx + (h01 - h00) * tz;
    return h11 + (h01 - h11) * (1.0f - tx) + (h10 - h11) * (1.0f - tz);
}

Math::Vec3 Terrain::normalAt(f32 localX, f32 localZ) const
{
    if (!valid())
        return Math::Vec3(0.0f, 1.0f, 0.0f);
    const f32 step = mCellSize;
    const f32 left = heightAt(localX - step, localZ);
    const f32 right = heightAt(localX + step, localZ);
    const f32 top = heightAt(localX, localZ - step);
    const f32 bottom = heightAt(localX, localZ + step);
    return glm::normalize(Math::Vec3(left - right, 2.0f * step, top - bottom));
}

bool Terrain::raycast(const Ray& worldRay, Math::Vec3& worldHit) const
{
    if (!valid() || !owner())
        return false;
    const Math::Mat4 inverse = glm::inverse(owner()->globalTransform());
    Ray ray;
    ray.origin = Math::Vec3(inverse * Math::Vec4(worldRay.origin, 1.0f));
    ray.direction = glm::normalize(Math::Vec3(inverse * Math::Vec4(worldRay.direction, 0.0f)));
    f32 enter = 0.0f;
    f32 leave = std::numeric_limits<f32>::max();
    for (u32 axis = 0; axis < 3; ++axis)
    {
        if (glm::abs(ray.direction[axis]) < 1e-8f)
        {
            if (ray.origin[axis] < mBounds.min[axis] || ray.origin[axis] > mBounds.max[axis])
                return false;
            continue;
        }
        f32 a = (mBounds.min[axis] - ray.origin[axis]) / ray.direction[axis];
        f32 b = (mBounds.max[axis] - ray.origin[axis]) / ray.direction[axis];
        if (a > b)
            std::swap(a, b);
        enter = glm::max(enter, a);
        leave = glm::min(leave, b);
        if (enter > leave)
            return false;
    }

    const f32 halfX = static_cast<f32>(mWidth - 1) * mCellSize * 0.5f;
    const f32 halfZ = static_cast<f32>(mHeight - 1) * mCellSize * 0.5f;
    const f32 epsilon = glm::max(1e-5f, mCellSize * 1e-5f);
    const Math::Vec3 start = ray.at(glm::min(leave, enter + epsilon));
    s32 cellX = glm::clamp(static_cast<s32>(std::floor((start.x + halfX) / mCellSize)), 0,
                           static_cast<s32>(mWidth) - 2);
    s32 cellZ = glm::clamp(static_cast<s32>(std::floor((start.z + halfZ) / mCellSize)), 0,
                           static_cast<s32>(mHeight) - 2);

    const s32 stepX = ray.direction.x > 1e-8f ? 1 : (ray.direction.x < -1e-8f ? -1 : 0);
    const s32 stepZ = ray.direction.z > 1e-8f ? 1 : (ray.direction.z < -1e-8f ? -1 : 0);
    const f32 infinity = std::numeric_limits<f32>::infinity();
    f32 nextX = infinity;
    f32 nextZ = infinity;
    f32 deltaX = infinity;
    f32 deltaZ = infinity;
    if (stepX != 0)
    {
        const f32 boundary = -halfX + static_cast<f32>(cellX + (stepX > 0 ? 1 : 0)) * mCellSize;
        nextX = (boundary - ray.origin.x) / ray.direction.x;
        deltaX = mCellSize / glm::abs(ray.direction.x);
    }
    if (stepZ != 0)
    {
        const f32 boundary = -halfZ + static_cast<f32>(cellZ + (stepZ > 0 ? 1 : 0)) * mCellSize;
        nextZ = (boundary - ray.origin.z) / ray.direction.z;
        deltaZ = mCellSize / glm::abs(ray.direction.z);
    }

    while (cellX >= 0 && cellX < static_cast<s32>(mWidth) - 1 &&
           cellZ >= 0 && cellZ < static_cast<s32>(mHeight) - 1)
    {
        const f32 cellLeave = glm::min(leave, glm::min(nextX, nextZ));
        const f32 x = static_cast<f32>(cellX) * mCellSize - halfX;
        const f32 z = static_cast<f32>(cellZ) * mCellSize - halfZ;
        const Math::Vec3 p00(x, sampleHeight(cellX, cellZ), z);
        const Math::Vec3 p10(x + mCellSize, sampleHeight(cellX + 1, cellZ), z);
        const Math::Vec3 p01(x, sampleHeight(cellX, cellZ + 1), z + mCellSize);
        const Math::Vec3 p11(x + mCellSize, sampleHeight(cellX + 1, cellZ + 1), z + mCellSize);

        f32 best = std::numeric_limits<f32>::max();
        f32 hit = 0.0f;
        if (ray.intersects(p01, p10, p00, hit) && hit >= enter - epsilon &&
            hit <= cellLeave + epsilon)
            best = hit;
        if (ray.intersects(p01, p11, p10, hit) && hit >= enter - epsilon &&
            hit <= cellLeave + epsilon)
            best = glm::min(best, hit);
        if (best != std::numeric_limits<f32>::max())
        {
            worldHit = Math::Vec3(owner()->globalTransform() * Math::Vec4(ray.at(best), 1.0f));
            return true;
        }

        if (cellLeave >= leave)
            break;
        if (nextX < nextZ)
        {
            cellX += stepX;
            nextX += deltaX;
        }
        else if (nextZ < nextX)
        {
            cellZ += stepZ;
            nextZ += deltaZ;
        }
        else
        {
            cellX += stepX;
            cellZ += stepZ;
            nextX += deltaX;
            nextZ += deltaZ;
        }
    }
    return false;
}

bool Terrain::raise(const Math::Vec3& center, f32 radius, f32 amount)
{
    return edit(center, radius, amount, false);
}
bool Terrain::lower(const Math::Vec3& center, f32 radius, f32 amount)
{
    return edit(center, radius, -amount, false);
}
bool Terrain::smooth(const Math::Vec3& center, f32 radius, f32 strength)
{
    return edit(center, radius, glm::clamp(strength, 0.0f, 1.0f), true);
}

bool Terrain::flatten(const Math::Vec3& worldCenter, f32 radius, f32 targetHeight, f32 strength)
{
    if (!valid() || !owner() || radius <= 0.0f || strength <= 0.0f)
        return false;
    const Math::Vec3 center =
        Math::Vec3(glm::inverse(owner()->globalTransform()) * Math::Vec4(worldCenter, 1.0f));
    const f32 halfX = static_cast<f32>(mWidth - 1) * mCellSize * 0.5f;
    const f32 halfZ = static_cast<f32>(mHeight - 1) * mCellSize * 0.5f;
    const s32 minX =
        glm::clamp(static_cast<s32>(std::floor((center.x - radius + halfX) / mCellSize)), 0,
                   static_cast<s32>(mWidth) - 1);
    const s32 maxX = glm::clamp(static_cast<s32>(std::ceil((center.x + radius + halfX) / mCellSize)),
                                0, static_cast<s32>(mWidth) - 1);
    const s32 minZ =
        glm::clamp(static_cast<s32>(std::floor((center.z - radius + halfZ) / mCellSize)), 0,
                   static_cast<s32>(mHeight) - 1);
    const s32 maxZ = glm::clamp(static_cast<s32>(std::ceil((center.z + radius + halfZ) / mCellSize)),
                                0, static_cast<s32>(mHeight) - 1);
    bool changed = false;
    for (s32 z = minZ; z <= maxZ; ++z)
        for (s32 x = minX; x <= maxX; ++x)
        {
            const Math::Vec2 delta(static_cast<f32>(x) * mCellSize - halfX - center.x,
                                  static_cast<f32>(z) * mCellSize - halfZ - center.z);
            const f32 distance = glm::length(delta);
            if (distance > radius)
                continue;
            const f32 falloff = 1.0f - glm::smoothstep(0.0f, radius, distance);
            const u32 index = vertexIndex(static_cast<u32>(x), static_cast<u32>(z));
            mHeights[index] =
                glm::mix(mHeights[index], targetHeight, glm::clamp(strength * falloff, 0.0f, 1.0f));
            changed = true;
        }
    if (!changed)
        return false;
    rebuildChunksTouching(static_cast<u32>(minX), static_cast<u32>(minZ), static_cast<u32>(maxX),
                          static_cast<u32>(maxZ));
    ++mRevision;
    return true;
}

u32 Terrain::generateGrass(Grass& grass, bool clearExisting) const
{
    if (!valid() || !owner() || !grass.owner() || grass.regionCount() == 0)
        return 0;
    const VegetationSettings& settings = mGrassGeneration;
    if (settings.spacing <= 0.0f || settings.density <= 0.0f ||
        settings.maximumInstances == 0)
        return 0;

    if (clearExisting)
        grass.clear();
    grass.setSeed(settings.seed);

    const f32 sizeX = static_cast<f32>(mWidth - 1) * mCellSize;
    const f32 sizeZ = static_cast<f32>(mHeight - 1) * mCellSize;
    f32 spacing = glm::max(settings.spacing, 0.05f);
    u64 columns = static_cast<u64>(std::ceil(sizeX / spacing));
    u64 rows = static_cast<u64>(std::ceil(sizeZ / spacing));
    const u64 candidateBudget =
        std::max<u64>(1024, static_cast<u64>(settings.maximumInstances) * 4);
    if (columns * rows > candidateBudget)
    {
        spacing *= std::sqrt(static_cast<f64>(columns * rows) /
                             static_cast<f64>(candidateBudget));
        columns = static_cast<u64>(std::ceil(sizeX / spacing));
        rows = static_cast<u64>(std::ceil(sizeZ / spacing));
    }

    const Math::Mat4 terrainToWorld = owner()->globalTransform();
    const Math::Mat4 worldToGrass = glm::inverse(grass.owner()->globalTransform());
    const Math::Mat3 normalToWorld = glm::transpose(glm::inverse(Math::Mat3(terrainToWorld)));
    const Math::Mat3 normalToGrass = glm::transpose(Math::Mat3(grass.owner()->globalTransform()));
    const f32 cosSlope = glm::cos(glm::radians(glm::clamp(settings.maximumSlopeDegrees,
                                                          0.0f, 89.9f)));
    const f32 jitter = glm::clamp(settings.jitter, 0.0f, 1.0f);
    const f32 density = glm::clamp(settings.density, 0.0f, 1.0f);
    const f32 scaleMin = glm::max(0.001f, glm::min(settings.minimumScale, settings.maximumScale));
    const f32 scaleMax = glm::max(scaleMin, glm::max(settings.minimumScale, settings.maximumScale));
    const f32 halfX = sizeX * 0.5f;
    const f32 halfZ = sizeZ * 0.5f;

    u32 planted = 0;
    for (u64 z = 0; z < rows && planted < settings.maximumInstances; ++z)
    {
        for (u64 x = 0; x < columns && planted < settings.maximumInstances; ++x)
        {
            const u32 key = terrainHash(settings.seed ^ static_cast<u32>(x) * 0x9e3779b9u ^
                                        static_cast<u32>(z) * 0x85ebca6bu);
            const f32 ox = (terrainRandom(key ^ 0x68bc21ebu) - 0.5f) * jitter;
            const f32 oz = (terrainRandom(key ^ 0x02e5be93u) - 0.5f) * jitter;
            const f32 localX = glm::clamp((static_cast<f32>(x) + 0.5f + ox) * spacing,
                                          0.0f, sizeX) - halfX;
            const f32 localZ = glm::clamp((static_cast<f32>(z) + 0.5f + oz) * spacing,
                                          0.0f, sizeZ) - halfZ;
            if (terrainRandom(key) >
                density * vegetationDensity(localX, localZ, VegetationChannel::Grass))
                continue;
            const f32 localY = heightAt(localX, localZ);
            if (localY < settings.minimumHeight || localY > settings.maximumHeight)
                continue;
            const Math::Vec3 localNormal = normalAt(localX, localZ);
            if (localNormal.y < cosSlope)
                continue;

            const Math::Vec3 worldPosition = Math::Vec3(
                terrainToWorld * Math::Vec4(localX, localY, localZ, 1.0f));
            const Math::Vec3 grassPosition = Math::Vec3(worldToGrass * Math::Vec4(worldPosition, 1.0f));
            const Math::Vec3 worldNormal = glm::normalize(normalToWorld * localNormal);
            const Math::Vec3 grassNormal = glm::normalize(normalToGrass * worldNormal);
            const f32 scale = glm::mix(scaleMin, scaleMax, terrainRandom(key ^ 0xa511e9b3u));
            if (grass.plant(grassPosition, grassNormal, scale))
                ++planted;
        }
    }
    return planted;
}

u32 Terrain::generateTrees(Forest& forest, bool clearExisting) const
{
    if (!valid() || !owner() || !forest.owner() || forest.speciesCount() == 0)
        return 0;
    const VegetationSettings& settings = mTreeGeneration;
    if (settings.spacing <= 0.0f || settings.density <= 0.0f ||
        settings.maximumInstances == 0)
        return 0;

    if (clearExisting)
        forest.clear();

    const f32 sizeX = static_cast<f32>(mWidth - 1) * mCellSize;
    const f32 sizeZ = static_cast<f32>(mHeight - 1) * mCellSize;
    f32 spacing = glm::max(settings.spacing, 0.1f);
    u64 columns = static_cast<u64>(std::ceil(sizeX / spacing));
    u64 rows = static_cast<u64>(std::ceil(sizeZ / spacing));
    const u64 candidateBudget =
        std::max<u64>(256, static_cast<u64>(settings.maximumInstances) * 4);
    if (columns * rows > candidateBudget)
    {
        spacing *= std::sqrt(static_cast<f64>(columns * rows) /
                             static_cast<f64>(candidateBudget));
        columns = static_cast<u64>(std::ceil(sizeX / spacing));
        rows = static_cast<u64>(std::ceil(sizeZ / spacing));
    }

    f32 totalWeight = 0.0f;
    for (u32 i = 0; i < forest.speciesCount(); ++i)
        totalWeight += glm::max(0.0f, forest.speciesWeight(i));

    const Math::Mat4 terrainToWorld = owner()->globalTransform();
    const Math::Mat4 worldToForest = glm::inverse(forest.owner()->globalTransform());
    const f32 cosSlope = glm::cos(glm::radians(glm::clamp(settings.maximumSlopeDegrees,
                                                          0.0f, 89.9f)));
    const f32 jitter = glm::clamp(settings.jitter, 0.0f, 1.0f);
    const f32 density = glm::clamp(settings.density, 0.0f, 1.0f);
    const f32 scaleMin = glm::max(0.001f, glm::min(settings.minimumScale, settings.maximumScale));
    const f32 scaleMax = glm::max(scaleMin, glm::max(settings.minimumScale, settings.maximumScale));
    const f32 halfX = sizeX * 0.5f;
    const f32 halfZ = sizeZ * 0.5f;

    u32 planted = 0;
    for (u64 z = 0; z < rows && planted < settings.maximumInstances; ++z)
    {
        for (u64 x = 0; x < columns && planted < settings.maximumInstances; ++x)
        {
            const u32 key = terrainHash(settings.seed ^ static_cast<u32>(x) * 0x27d4eb2du ^
                                        static_cast<u32>(z) * 0x165667b1u);
            const f32 ox = (terrainRandom(key ^ 0xb5297a4du) - 0.5f) * jitter;
            const f32 oz = (terrainRandom(key ^ 0x1b56c4e9u) - 0.5f) * jitter;
            const f32 localX = glm::clamp((static_cast<f32>(x) + 0.5f + ox) * spacing,
                                          0.0f, sizeX) - halfX;
            const f32 localZ = glm::clamp((static_cast<f32>(z) + 0.5f + oz) * spacing,
                                          0.0f, sizeZ) - halfZ;
            if (terrainRandom(key) >
                density * vegetationDensity(localX, localZ, VegetationChannel::Trees))
                continue;
            const f32 localY = heightAt(localX, localZ);
            if (localY < settings.minimumHeight || localY > settings.maximumHeight ||
                normalAt(localX, localZ).y < cosSlope)
                continue;

            u32 species = key % forest.speciesCount();
            if (totalWeight > 0.0f)
            {
                f32 pick = terrainRandom(key ^ 0xc2b2ae35u) * totalWeight;
                for (u32 i = 0; i < forest.speciesCount(); ++i)
                {
                    pick -= glm::max(0.0f, forest.speciesWeight(i));
                    if (pick <= 0.0f)
                    {
                        species = i;
                        break;
                    }
                }
            }
            const Math::Vec3 worldPosition = Math::Vec3(
                terrainToWorld * Math::Vec4(localX, localY, localZ, 1.0f));
            const Math::Vec3 forestPosition = Math::Vec3(worldToForest * Math::Vec4(worldPosition, 1.0f));
            const f32 scale = glm::mix(scaleMin, scaleMax, terrainRandom(key ^ 0x9e3779b9u));
            const f32 yaw = terrainRandom(key ^ 0x85ebca6bu) * 360.0f;
            if (forest.plant(forestPosition, species, scale, yaw))
                ++planted;
        }
    }
    return planted;
}

bool Terrain::edit(const Math::Vec3& worldCenter, f32 radius, f32 amount, bool smoothing)
{
    if (!valid() || !owner() || radius <= 0.0f || amount == 0.0f)
        return false;
    const Math::Vec3 center =
        Math::Vec3(glm::inverse(owner()->globalTransform()) * Math::Vec4(worldCenter, 1.0f));
    const f32 halfX = static_cast<f32>(mWidth - 1) * mCellSize * 0.5f;
    const f32 halfZ = static_cast<f32>(mHeight - 1) * mCellSize * 0.5f;
    const s32 minX =
        glm::clamp(static_cast<s32>(std::floor((center.x - radius + halfX) / mCellSize)), 0,
                   static_cast<s32>(mWidth) - 1);
    const s32 maxX = glm::clamp(static_cast<s32>(std::ceil((center.x + radius + halfX) / mCellSize)),
                                0, static_cast<s32>(mWidth) - 1);
    const s32 minZ =
        glm::clamp(static_cast<s32>(std::floor((center.z - radius + halfZ) / mCellSize)), 0,
                   static_cast<s32>(mHeight) - 1);
    const s32 maxZ = glm::clamp(static_cast<s32>(std::ceil((center.z + radius + halfZ) / mCellSize)),
                                0, static_cast<s32>(mHeight) - 1);
    std::vector<f32> source;
    if (smoothing)
        source = mHeights;
    bool changed = false;
    for (s32 z = minZ; z <= maxZ; ++z)
    {
        for (s32 x = minX; x <= maxX; ++x)
        {
            const Math::Vec2 delta(static_cast<f32>(x) * mCellSize - halfX - center.x,
                                  static_cast<f32>(z) * mCellSize - halfZ - center.z);
            const f32 distance = glm::length(delta);
            if (distance > radius)
                continue;
            const f32 falloff = 1.0f - glm::smoothstep(0.0f, radius, distance);
            const u32 index = vertexIndex(static_cast<u32>(x), static_cast<u32>(z));
            if (smoothing)
            {
                f32 average = 0.0f;
                u32 samples = 0;
                for (s32 dz = -1; dz <= 1; ++dz)
                    for (s32 dx = -1; dx <= 1; ++dx)
                    {
                        const s32 sx = glm::clamp(x + dx, 0, static_cast<s32>(mWidth) - 1);
                        const s32 sz = glm::clamp(z + dz, 0, static_cast<s32>(mHeight) - 1);
                        average += source[vertexIndex(static_cast<u32>(sx), static_cast<u32>(sz))];
                        ++samples;
                    }
                mHeights[index] = glm::mix(source[index], average / samples, amount * falloff);
            }
            else
                mHeights[index] += amount * falloff;
            changed = true;
        }
    }
    if (!changed)
        return false;
    rebuildChunksTouching(static_cast<u32>(minX), static_cast<u32>(minZ), static_cast<u32>(maxX),
                          static_cast<u32>(maxZ));
    ++mRevision;
    return true;
}

void Terrain::rebuildChunksTouching(u32 minX, u32 minZ, u32 maxX, u32 maxZ)
{
    if (mChunkCountX == 0)
        return;

    // A chunk's own border normals read one vertex PAST its own range (see
    // buildChunk()), so an edit at a chunk's edge also has to rebuild the
    // chunk on the other side of that edge - the "-1"/"+1" here, not just
    // the chunk the edited vertices themselves fall in.
    auto chunkIndexFor = [](s32 globalIndex, u32 chunkCount) {
        const s32 c = static_cast<s32>(
            std::floor(static_cast<f32>(globalIndex) / static_cast<f32>(ChunkSpan)));
        return static_cast<u32>(glm::clamp(c, 0, static_cast<s32>(chunkCount) - 1));
    };
    const u32 chunkMinX = chunkIndexFor(static_cast<s32>(minX) - 1, mChunkCountX);
    const u32 chunkMaxX = chunkIndexFor(static_cast<s32>(maxX) + 1, mChunkCountX);
    const u32 chunkMinZ = chunkIndexFor(static_cast<s32>(minZ) - 1, mChunkCountZ);
    const u32 chunkMaxZ = chunkIndexFor(static_cast<s32>(maxZ) + 1, mChunkCountZ);

    for (u32 cz = chunkMinZ; cz <= chunkMaxZ; ++cz)
        for (u32 cx = chunkMinX; cx <= chunkMaxX; ++cx)
            buildChunk(cx, cz);

    // Loose on purpose: only grows, matching what raycast()'s early-out
    // needs (a bound that never excludes real geometry, not a tight one).
    for (u32 z = minZ; z <= maxZ; ++z)
        for (u32 x = minX; x <= maxX; ++x)
        {
            const f32 halfX = static_cast<f32>(mWidth - 1) * mCellSize * 0.5f;
            const f32 halfZ = static_cast<f32>(mHeight - 1) * mCellSize * 0.5f;
            mBounds.expand(Math::Vec3(static_cast<f32>(x) * mCellSize - halfX,
                                     mHeights[vertexIndex(x, z)],
                                     static_cast<f32>(z) * mCellSize - halfZ));
        }
}

bool Terrain::buildChunk(u32 chunkX, u32 chunkZ)
{
    const TerrainIndices& indices = TerrainIndices::instance();
    if (!indices.buffer.valid())
        return false;

    Chunk& chunk = mChunks[chunkZ * mChunkCountX + chunkX];

    const f32 halfX = static_cast<f32>(mWidth - 1) * mCellSize * 0.5f;
    const f32 halfZ = static_cast<f32>(mHeight - 1) * mCellSize * 0.5f;
    constexpr f32 chunkHalfWidth = static_cast<f32>(ChunkWidth - 1) * 0.5f;
    const u32 originX = chunkX * ChunkSpan;
    const u32 originZ = chunkZ * ChunkSpan;
    chunk.position = Math::Vec3((static_cast<f32>(originX) + chunkHalfWidth) * mCellSize - halfX,
                               0.0f,
                               (static_cast<f32>(originZ) + chunkHalfWidth) * mCellSize - halfZ);

    // Height grid WITH PADDING: 68x68 instead of 67x67, one step further than
    // the chunk's own vertices in every direction. A padded sample at an
    // internal chunk boundary reads the true neighbour's own height (this is
    // one master array, not a per-chunk copy), so two neighbouring chunks
    // always compute the identical normal at the edge they share - crack-free
    // shading, not just crack-free geometry.
    constexpr u32 paddedWidth = ChunkWidth + 1;
    std::vector<f32> padded(static_cast<usize>(paddedWidth) * paddedWidth);
    for (u32 lz = 0; lz < paddedWidth; ++lz)
        for (u32 lx = 0; lx < paddedWidth; ++lx)
            padded[lx + lz * paddedWidth] = sampleHeight(static_cast<s32>(originX + lx),
                                                          static_cast<s32>(originZ + lz));

    std::vector<Math::Vec3> positions(Terrain::VertexCount);
    std::vector<MeshAttribs> attribs(Terrain::VertexCount);

    const f32 inverseX = 1.0f / static_cast<f32>(mWidth - 1);
    const f32 inverseZ = 1.0f / static_cast<f32>(mHeight - 1);
    f32 minY = 1e30f;
    f32 maxY = -1e30f;
    bool slopeCastShadow = false;
    for (u32 lz = 0; lz < ChunkWidth; ++lz)
    {
        for (u32 lx = 0; lx < ChunkWidth; ++lx)
        {
            const u32 index = lx + lz * ChunkWidth;
            const f32 x = (static_cast<f32>(lx) - chunkHalfWidth) * mCellSize;
            const f32 z = (static_cast<f32>(lz) - chunkHalfWidth) * mCellSize;
            const f32 height = padded[lx + lz * paddedWidth];

            const Math::Vec3 c0(x, height, z);
            const Math::Vec3 c1(x + mCellSize, padded[(lx + 1) + lz * paddedWidth], z);
            const Math::Vec3 c2(x, padded[lx + (lz + 1) * paddedWidth], z + mCellSize);
            const Math::Vec3 t = c1 - c2;
            const Math::Vec3 b = c0 - c1;
            const Math::Vec3 n = glm::normalize(glm::cross(t, b));

            const f32 slopeAmount = 1.0f - glm::clamp(n.y, 0.0f, 1.0f);
            if (slopeAmount > 0.1f)
                slopeCastShadow = true;

            positions[index] = Math::Vec3(x, height, z);

            const u32 gx = originX + lx;
            const u32 gz = originZ + lz;
            attribs[index].normal = n;
            attribs[index].tangent = Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
            const Math::Vec2 terrainUV(static_cast<f32>(gx) * inverseX,
                                      static_cast<f32>(gz) * inverseZ);
            attribs[index].uv = terrainUV;
            attribs[index].uv2 = terrainUV;

            const u8 encodedHeight = static_cast<u8>(
                glm::clamp(height / mHeightScale, 0.0f, 1.0f) * 255.0f + 0.5f);
            const u8 encodedSlope = static_cast<u8>(
                glm::clamp(slopeAmount, 0.0f, 1.0f) * 255.0f + 0.5f);
            u32 hash = (gx * 0x8da6b343u) ^ (gz * 0xd8163841u) ^ 0xcb1ab31fu;
            hash ^= hash >> 13;
            hash *= 0x85ebca6bu;
            const u8 variation = static_cast<u8>(hash >> 24);
            attribs[index].color = static_cast<u32>(encodedHeight) |
                                   (static_cast<u32>(encodedSlope) << 8) |
                                   (static_cast<u32>(variation) << 16) | 0xFF000000u;

            minY = glm::min(minY, height);
            maxY = glm::max(maxY, height);
        }
    }

    chunk.castShadow = slopeCastShadow;
    chunk.minimumY = minY;
    chunk.maximumY = maxY;
    chunk.sphereCentre = chunk.position + Math::Vec3(0.0f, (minY + maxY) * 0.5f, 0.0f);
    const f32 chunkSpan = static_cast<f32>(ChunkWidth - 1) * mCellSize;
    const f32 halfSpan = chunkSpan * 0.5f;
    const f32 halfY = (maxY - minY) * 0.5f;
    chunk.sphereRadius = std::sqrt(2.0f * halfSpan * halfSpan + halfY * halfY);

    GPU& gpu = GPU::getSingleton();
    AABB chunkBounds;
    for (const Math::Vec3& p : positions)
        chunkBounds.expand(p);

    // Sculpting never changes a chunk's vertex count or layout. Reuse its
    // buffers instead of destroying/adopting a Mesh for every brush sample.
    if (Mesh* existing = Assets().getMesh(chunk.mesh))
    {
        gpu.updateBuffer(existing->positionBuffer, 0, positions.size() * sizeof(Math::Vec3),
                         positions.data());
        gpu.updateBuffer(existing->attribBuffer, 0, attribs.size() * sizeof(MeshAttribs),
                         attribs.data());
        existing->bounds = chunkBounds;
        if (!existing->submeshes.empty())
            existing->submeshes[0].bounds = chunkBounds;
        return true;
    }

    BufferDesc positionDesc;
    positionDesc.size = positions.size() * sizeof(Math::Vec3);
    positionDesc.usage = BufferVertex;
    // Sculpting updates this buffer in-place in buildChunk(). Static buffers
    // reject updateBuffer() on the backends, leaving the rendered ground at
    // the old height while Road already conforms to the new CPU heightmap.
    positionDesc.residency = Residency::Dynamic;
    positionDesc.stride = sizeof(Math::Vec3);
    positionDesc.data = positions.data();
    positionDesc.debugName = "terrain.chunk.position";
    const BufferHandle positionBuffer = gpu.createBuffer(positionDesc);

    BufferDesc attribDesc;
    attribDesc.size = attribs.size() * sizeof(MeshAttribs);
    attribDesc.usage = BufferVertex;
    // Normals, height/slope colour weights and bounds change with the same
    // brush edit, so this stream must be writable as well.
    attribDesc.residency = Residency::Dynamic;
    attribDesc.stride = sizeof(MeshAttribs);
    attribDesc.data = attribs.data();
    attribDesc.debugName = "terrain.chunk.attrib";
    const BufferHandle attribBuffer = gpu.createBuffer(attribDesc);

    if (!positionBuffer.valid() || !attribBuffer.valid())
    {
        gpu.destroy(positionBuffer);
        gpu.destroy(attribBuffer);
        return false;
    }

    Mesh mesh;
    mesh.positionBuffer = positionBuffer;
    mesh.attribBuffer = attribBuffer;
    mesh.indexBuffer = indices.buffer;
    mesh.indexType = IndexType::U32;
    mesh.ownsIndexBuffer = false; // the shared buffer outlives every chunk
    mesh.vertexCount = Terrain::VertexCount;
    mesh.indexCount = indices.lods[0].indexCount;

    mesh.depthLayout.streamCount = 1;
    mesh.depthLayout.streams[StreamPosition].stride = sizeof(Math::Vec3);
    mesh.depthLayout.attribCount = 1;
    mesh.depthLayout.attribs[0] = {0, StreamPosition, 0, AttribFormat::Float3};

    mesh.colorLayout = mesh.depthLayout;
    mesh.colorLayout.streamCount = 2;
    mesh.colorLayout.streams[StreamAttribs].stride = sizeof(MeshAttribs);
    mesh.colorLayout.attribCount = 6;
    mesh.colorLayout.attribs[1] = {1, StreamAttribs, offsetof(MeshAttribs, normal),
                                   AttribFormat::Float3};
    mesh.colorLayout.attribs[2] = {2, StreamAttribs, offsetof(MeshAttribs, tangent),
                                   AttribFormat::Float4};
    mesh.colorLayout.attribs[3] = {3, StreamAttribs, offsetof(MeshAttribs, uv),
                                   AttribFormat::Float2};
    mesh.colorLayout.attribs[4] = {4, StreamAttribs, offsetof(MeshAttribs, color),
                                   AttribFormat::UByte4N};
    mesh.colorLayout.attribs[5] = {7, StreamAttribs, offsetof(MeshAttribs, uv2),
                                   AttribFormat::Float2};

    SubMesh submesh;
    submesh.indexOffset = indices.lods[0].indexOffset;
    submesh.indexCount = indices.lods[0].indexCount;
    submesh.materialSlot = 0;
    submesh.bounds = chunkBounds;
    mesh.bounds = submesh.bounds;
    mesh.submeshes = {submesh};

    chunk.mesh = Assets().adoptMesh(mesh);
    return chunk.mesh.valid();
}

void Terrain::releaseChunk(Chunk& chunk)
{
    if (chunk.mesh.valid())
        Assets().destroyMesh(chunk.mesh);
    chunk.mesh = MeshHandle();
}

u32 Terrain::pickLod(const Chunk& chunk, const Math::Vec3& cameraPosition,
                     const Math::Mat4& transform) const
{
    const TerrainIndices& indices = TerrainIndices::instance();
    const s32 maxLod = glm::min(static_cast<s32>(indices.lods.size()) - 1,
                                static_cast<s32>(mMaxLod) - 1);
    const f32 scale = glm::max(glm::length(Math::Vec3(transform[0])),
                              glm::max(glm::length(Math::Vec3(transform[1])),
                                       glm::length(Math::Vec3(transform[2]))));
    const f32 chunkSpan = static_cast<f32>(ChunkWidth - 1) * mCellSize * scale;
    const Math::Vec3 centre = Math::Vec3(transform * Math::Vec4(chunk.sphereCentre, 1.0f));
    const f32 radius = chunk.sphereRadius * scale;

    const f32 distance =
        glm::max(0.0f, glm::distance(cameraPosition, centre) - radius);
    const f32 ratio = distance / chunkSpan;
    const s32 target = glm::clamp(
        static_cast<s32>(std::floor(std::log2(glm::max(1.0f, ratio)))), 0,
        glm::max(0, maxLod));

    // Fifteen percent hysteresis around each power-of-two boundary. Without
    // it, a camera hovering at exactly one boundary swaps thousands of
    // triangles back and forth on tiny movements and the ground shimmers.
    s32 lod = glm::clamp(static_cast<s32>(chunk.lastLod), 0, glm::max(0, maxLod));
    if (target > lod)
    {
        const f32 coarsenAt = std::exp2(static_cast<f32>(lod + 1)) * 1.15f;
        if (ratio >= coarsenAt)
            lod = target;
    }
    else if (target < lod)
    {
        const f32 refineAt = std::exp2(static_cast<f32>(lod)) * 0.85f;
        if (ratio <= refineAt)
            lod = target;
    }
    return static_cast<u32>(lod);
}

void Terrain::prepare(const Frustum& frustum, const Math::Vec3& cameraPosition)
{
    if (!valid() || !owner())
        return;

    const Math::Mat4 transform = owner()->globalTransform();
    const f32 scale = glm::max(glm::length(Math::Vec3(transform[0])),
                              glm::max(glm::length(Math::Vec3(transform[1])),
                                       glm::length(Math::Vec3(transform[2]))));
    mVisibleChunks = 0;
    for (Chunk& chunk : mChunks)
    {
        const Math::Vec3 centre = Math::Vec3(transform * Math::Vec4(chunk.sphereCentre, 1.0f));
        chunk.visible = frustum.intersects(Sphere{centre, chunk.sphereRadius * scale});
        if (chunk.visible)
        {
            ++mVisibleChunks;
            chunk.lastLod = pickLod(chunk, cameraPosition, transform);
        }
    }
}

void Terrain::submitCamera(RenderList& list, const Math::Mat4& transform)
{
    const TerrainIndices& indices = TerrainIndices::instance();
    if (!indices.buffer.valid() || mChunks.empty())
        return;

    AssetManager& assets = Assets();
    MaterialManager& materials = MaterialManager::getSingleton();

    for (Chunk& chunk : mChunks)
    {
        if (!chunk.visible)
            continue;
        Mesh* mesh = assets.getMesh(chunk.mesh);
        if (!mesh || mesh->submeshes.empty())
            continue;
        materials.resolvePipeline(mMaterial, mesh->colorLayout);
        materials.sync(mMaterial);

        const TerrainLod& range = indices.lods[chunk.lastLod];
        mesh->submeshes[0].indexOffset = range.indexOffset;
        mesh->submeshes[0].indexCount = range.indexCount;

        const Math::Mat4 model = glm::translate(transform, chunk.position);
        list.submit(chunk.mesh, *mesh, model, &mMaterial, 1);
    }
}

void Terrain::submitShadow(RenderList& list, const Math::Mat4& transform)
{
    const TerrainIndices& indices = TerrainIndices::instance();
    if (!indices.buffer.valid() || mChunks.empty())
        return;

    AssetManager& assets = Assets();
    const s32 maxLod = static_cast<s32>(indices.lods.size()) - 1;

    for (Chunk& chunk : mChunks)
    {
        // Only chunks with slope enter the shadow view: flat ground cannot
        // shadow itself, and most of a terrain is flat ground.
        if (!chunk.castShadow)
            continue;
        Mesh* mesh = assets.getMesh(chunk.mesh);
        if (!mesh || mesh->submeshes.empty())
            continue;

        // One LOD coarser than the scene view uses: the shadow map has no
        // resolution to show the difference, and this halves the triangles.
        const u32 lod = static_cast<u32>(
            glm::clamp(static_cast<s32>(chunk.lastLod) + 1, 0, maxLod));
        const TerrainLod& range = indices.lods[lod];
        mesh->submeshes[0].indexOffset = range.indexOffset;
        mesh->submeshes[0].indexCount = range.indexCount;

        const Math::Mat4 model = glm::translate(transform, chunk.position);
        list.submit(chunk.mesh, *mesh, model, &mMaterial, 1);
    }
}

} // namespace Radion
