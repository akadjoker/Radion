#include "PCH.h"

#include "VoxelWorldComponent.h"

#include "AssetManager.h"
#include "ByteArray.h"
#include "FileSystem.h"
#include "GameObject.h"
#include "Log.h"
#include "MeshRenderer.h"
#include "Profiler.h"
#include "Scene.h"

#include <algorithm>

namespace Radion
{

namespace
{
enum class VoxelPass : u8
{
    Opaque,
    Cutout,
    Transparent
};

constexpr u16 AtlasColumns = 16;
constexpr u16 AtlasRows = 16;

Material makeVoxelMaterial(const std::string& atlasFile, VoxelPass pass)
{
    Material material;
    material.flags |= MaterialLit | MaterialVoxelAtlas;
    material.textures[SlotAlbedo].file = atlasFile;
    material.textures[SlotAlbedo].source = TextureSource::Static;
    // No mips and point sampling: any filtering across a tile edge pulls in
    // the neighbouring block's texture, and a half-texel inset cannot fix a
    // mip level whose texel spans several tiles.
    material.textures[SlotAlbedo].texture =
        Assets().loadTexture(atlasFile, ColorSpace::sRGB, false, 1);
    SamplerDesc sampler;
    sampler.filter = Filter::Point;
    material.textures[SlotAlbedo].sampler = Assets().getSampler(sampler);
    // The tile the mesher wrote into uv2 covers this much of the atlas, and
    // lit.frag's VOXEL_ATLAS path maps each face's tiled uv inside it. Left at
    // zero the whole face samples the tile's first texel and every block comes
    // out flat.
    material.params.custom0 =
        glm::vec4(1.0f / static_cast<f32>(AtlasColumns), 1.0f / static_cast<f32>(AtlasRows),
                  0.0f, 0.0f);
    material.blend = pass == VoxelPass::Transparent ? BlendMode::Alpha : BlendMode::Opaque;
    if (pass == VoxelPass::Transparent)
    {
        material.flags |= MaterialNoDepthWrite;
        // Water sits at alpha 129 of 255 in the atlas, a texel above the
        // default cutoff. Blended geometry has no business being alpha tested
        // at all, and leaving it at 0.5 puts the whole ocean one authored
        // texel away from disappearing.
        material.params.surface.z = 0.0f;
    }
    // Leaves keep depth and sort with the opaque geometry; only the texels the
    // atlas leaves empty are dropped.
    if (pass == VoxelPass::Cutout)
        material.flags |= MaterialAlphaTest;
    return material;
}

// Tiles are read off assets/textures/terrain.png, a 16x16 atlas of 16px tiles.
struct BlockTemplate
{
    const char* name;
    u16 sideX;
    u16 sideY;
    u16 topX;
    u16 topY;
    u16 bottomX;
    u16 bottomY;
    Voxel::BlockRenderType render;
    bool solid;
    bool blocksLight;
};

const BlockTemplate DefaultBlocks[] = {
    {"grass", 3, 0, 0, 0, 2, 0, Voxel::BlockRenderType::Opaque, true, true},
    {"dirt", 2, 0, 2, 0, 2, 0, Voxel::BlockRenderType::Opaque, true, true},
    {"stone", 1, 0, 1, 0, 1, 0, Voxel::BlockRenderType::Opaque, true, true},
    {"sand", 2, 1, 2, 1, 2, 1, Voxel::BlockRenderType::Opaque, true, true},
    {"gravel", 3, 1, 3, 1, 3, 1, Voxel::BlockRenderType::Opaque, true, true},
    {"bedrock", 1, 1, 1, 1, 1, 1, Voxel::BlockRenderType::Opaque, true, true},
    {"snow", 2, 4, 2, 4, 2, 4, Voxel::BlockRenderType::Opaque, true, true},
    {"log", 4, 1, 5, 1, 5, 1, Voxel::BlockRenderType::Opaque, true, true},
    {"coal_ore", 2, 2, 2, 2, 2, 2, Voxel::BlockRenderType::Opaque, true, true},
    {"iron_ore", 1, 2, 1, 2, 1, 2, Voxel::BlockRenderType::Opaque, true, true},
    {"gold_ore", 0, 2, 0, 2, 0, 2, Voxel::BlockRenderType::Opaque, true, true},
    {"diamond_ore", 2, 3, 2, 3, 2, 3, Voxel::BlockRenderType::Opaque, true, true},
    {"leaves", 4, 3, 4, 3, 4, 3, Voxel::BlockRenderType::Cutout, true, false},
    {"water", 13, 12, 13, 12, 13, 12, Voxel::BlockRenderType::Transparent, false, false},
};

Voxel::BlockRegistry makeDefaultRegistry()
{
    Voxel::BlockRegistry registry;
    for (const BlockTemplate& source : DefaultBlocks)
    {
        Voxel::BlockDefinition definition;
        definition.name = source.name;
        definition.solid = source.solid;
        definition.blocksLight = source.blocksLight;
        definition.transparent = source.render != Voxel::BlockRenderType::Opaque;
        definition.renderType = source.render;
        for (Voxel::BlockFaceMaterial& face : definition.faces)
        {
            face.atlasX = source.sideX;
            face.atlasY = source.sideY;
        }
        Voxel::BlockFaceMaterial& top =
            definition.faces[static_cast<usize>(Voxel::BlockFace::PositiveY)];
        top.atlasX = source.topX;
        top.atlasY = source.topY;
        Voxel::BlockFaceMaterial& bottom =
            definition.faces[static_cast<usize>(Voxel::BlockFace::NegativeY)];
        bottom.atlasX = source.bottomX;
        bottom.atlasY = source.bottomY;
        registry.registerBlock(std::move(definition));
    }
    return registry;
}
} // namespace

VoxelWorldComponent::VoxelWorldComponent() : Component(Type, ComponentEventUpdate)
{
    mStreamer.setBlocks(makeDefaultRegistry());
    mMesher.atlasColumns = AtlasColumns;
    mMesher.atlasRows = AtlasRows;
    mStreamer.setMesherSettings(mMesher);

    mTerrain.minWorldY = 0;
    mTerrain.maxWorldY = 127;
    mTerrain.waterLevel = 40;
    mTerrain.minSurfaceHeight = 8;
    mTerrain.maxSurfaceHeight = 110;
    mTerrain.baseSurfaceHeight = 48.0f;
    mTerrain.continentalAmplitude = 26.0f;
    mTerrain.detailAmplitude = 7.0f;

    applyStreamingSettings();
    applyTerrainSettings();
}

void VoxelWorldComponent::markTerrainDirty()
{
    mTerrainDirty = true;
    mTerrainDirtyUpdate = mUpdateCounter;
}

void VoxelWorldComponent::applyTerrainSettings()
{
    mTerrainDirty = false;
    mStreamer.setTerrain(mSeed, mTerrain);
}

void VoxelWorldComponent::applyStreamingSettings()
{
    Voxel::VoxelStreamer::Settings settings;
    settings.viewRadius = mChunkRadius;
    settings.bounded = mBounded;
    settings.boundsMinX = mBoundsMinX;
    settings.boundsMaxX = mBoundsMaxX;
    settings.boundsMinZ = mBoundsMinZ;
    settings.boundsMaxZ = mBoundsMaxZ;
    settings.maxUploadsPerFrame = mMaxUploadsPerFrame;
    settings.maxGenerationJobs = mMaxGenerationJobs;
    settings.maxMeshJobs = mMaxMeshJobs;
    mStreamer.configure(settings);
}

void VoxelWorldComponent::setSeed(u32 seed)
{
    if (mSeed == seed)
        return;
    mSeed = seed;
    markTerrainDirty();
}

void VoxelWorldComponent::setChunkRadius(s32 radius)
{
    const s32 clamped = std::max(0, radius);
    if (mChunkRadius == clamped)
        return;
    mChunkRadius = clamped;
    applyStreamingSettings();
}

void VoxelWorldComponent::setTerrainSettings(const Voxel::VoxelTerrain::Settings& settings)
{
    mTerrain = settings;
    mTerrain.maxWorldY = std::max(mTerrain.minWorldY, mTerrain.maxWorldY);
    mTerrain.maxSurfaceHeight = std::max(mTerrain.minSurfaceHeight, mTerrain.maxSurfaceHeight);
    markTerrainDirty();
}

void VoxelWorldComponent::setMinWorldY(s32 value)
{
    const s32 clamped = std::min(value, mTerrain.maxWorldY);
    if (mTerrain.minWorldY == clamped)
        return;
    mTerrain.minWorldY = clamped;
    markTerrainDirty();
}

void VoxelWorldComponent::setMaxWorldY(s32 value)
{
    const s32 clamped = std::max(value, mTerrain.minWorldY);
    if (mTerrain.maxWorldY == clamped)
        return;
    mTerrain.maxWorldY = clamped;
    markTerrainDirty();
}

void VoxelWorldComponent::setWorldHeightRange(s32 minValue, s32 maxValue)
{
    const s32 low = std::min(minValue, maxValue);
    const s32 high = std::max(minValue, maxValue);
    if (mTerrain.minWorldY == low && mTerrain.maxWorldY == high)
        return;
    mTerrain.minWorldY = low;
    mTerrain.maxWorldY = high;
    markTerrainDirty();
}

void VoxelWorldComponent::setWaterLevel(s32 value)
{
    if (mTerrain.waterLevel == value)
        return;
    mTerrain.waterLevel = value;
    markTerrainDirty();
}

void VoxelWorldComponent::setBaseSurfaceHeight(f32 value)
{
    if (mTerrain.baseSurfaceHeight == value)
        return;
    mTerrain.baseSurfaceHeight = value;
    markTerrainDirty();
}

void VoxelWorldComponent::setContinentalAmplitude(f32 value)
{
    if (mTerrain.continentalAmplitude == value)
        return;
    mTerrain.continentalAmplitude = value;
    markTerrainDirty();
}

void VoxelWorldComponent::setDetailAmplitude(f32 value)
{
    if (mTerrain.detailAmplitude == value)
        return;
    mTerrain.detailAmplitude = value;
    markTerrainDirty();
}

void VoxelWorldComponent::setReliefFrequency(f32 value)
{
    if (mTerrain.reliefFrequency == value)
        return;
    mTerrain.reliefFrequency = value;
    markTerrainDirty();
}

void VoxelWorldComponent::setBiomes(bool enabled)
{
    if (mTerrain.biomes == enabled)
        return;
    mTerrain.biomes = enabled;
    markTerrainDirty();
}

void VoxelWorldComponent::setBiomeFrequency(f32 value)
{
    if (mTerrain.biomeFrequency == value)
        return;
    mTerrain.biomeFrequency = value;
    markTerrainDirty();
}

void VoxelWorldComponent::setCaves(bool enabled)
{
    if (mTerrain.caves == enabled)
        return;
    mTerrain.caves = enabled;
    markTerrainDirty();
}

void VoxelWorldComponent::setCaveFrequency(f32 value)
{
    if (mTerrain.caveFrequency == value)
        return;
    mTerrain.caveFrequency = value;
    markTerrainDirty();
}

void VoxelWorldComponent::setCaveThreshold(f32 value)
{
    if (mTerrain.caveThreshold == value)
        return;
    mTerrain.caveThreshold = value;
    markTerrainDirty();
}

void VoxelWorldComponent::setCaveCeiling(s32 value)
{
    if (mTerrain.caveCeiling == value)
        return;
    mTerrain.caveCeiling = value;
    markTerrainDirty();
}

void VoxelWorldComponent::setTrees(bool enabled)
{
    if (mTerrain.trees == enabled)
        return;
    mTerrain.trees = enabled;
    markTerrainDirty();
}

void VoxelWorldComponent::setTreeDensity(f32 value)
{
    if (mTerrain.treeDensity == value)
        return;
    mTerrain.treeDensity = value;
    markTerrainDirty();
}

void VoxelWorldComponent::setFlat(bool enabled)
{
    if (mTerrain.flat == enabled)
        return;
    mTerrain.flat = enabled;
    markTerrainDirty();
}

usize VoxelWorldComponent::blockCount() const
{
    return mStreamer.blocks().size();
}

const Voxel::BlockDefinition* VoxelWorldComponent::blockDefinition(Voxel::BlockId id) const
{
    return mStreamer.blocks().find(id);
}

Voxel::BlockId VoxelWorldComponent::addBlock(const Voxel::BlockDefinition& definition)
{
    const Voxel::BlockId id = mStreamer.addBlock(definition);
    if (id != Voxel::InvalidBlockId)
        markTerrainDirty();
    return id;
}

bool VoxelWorldComponent::setBlockDefinition(Voxel::BlockId id,
                                             const Voxel::BlockDefinition& definition)
{
    if (!mStreamer.replaceBlock(id, definition))
        return false;
    markTerrainDirty();
    return true;
}

void VoxelWorldComponent::resetBlocksToDefault()
{
    mStreamer.setBlocks(makeDefaultRegistry());
    markTerrainDirty();
}

void VoxelWorldComponent::setAmbientOcclusion(bool enabled)
{
    if (mMesher.ambientOcclusion == enabled)
        return;
    mMesher.ambientOcclusion = enabled;
    mStreamer.setMesherSettings(mMesher);
    // Meshes already built keep the shading they were built with, so the
    // world has to come back through the mesher for this to show.
    markTerrainDirty();
}

void VoxelWorldComponent::setOres(bool enabled)
{
    if (mTerrain.ores == enabled)
        return;
    mTerrain.ores = enabled;
    markTerrainDirty();
}

void VoxelWorldComponent::setMinSurfaceHeight(s32 value)
{
    const s32 clamped = std::min(value, mTerrain.maxSurfaceHeight);
    if (mTerrain.minSurfaceHeight == clamped)
        return;
    mTerrain.minSurfaceHeight = clamped;
    markTerrainDirty();
}

void VoxelWorldComponent::setMaxSurfaceHeight(s32 value)
{
    const s32 clamped = std::max(value, mTerrain.minSurfaceHeight);
    if (mTerrain.maxSurfaceHeight == clamped)
        return;
    mTerrain.maxSurfaceHeight = clamped;
    markTerrainDirty();
}

void VoxelWorldComponent::setSurfaceHeightRange(s32 minValue, s32 maxValue)
{
    const s32 low = std::min(minValue, maxValue);
    const s32 high = std::max(minValue, maxValue);
    if (mTerrain.minSurfaceHeight == low && mTerrain.maxSurfaceHeight == high)
        return;
    mTerrain.minSurfaceHeight = low;
    mTerrain.maxSurfaceHeight = high;
    markTerrainDirty();
}

void VoxelWorldComponent::setBounded(bool bounded)
{
    if (mBounded == bounded)
        return;
    mBounded = bounded;
    applyStreamingSettings();
}

void VoxelWorldComponent::setBoundsMinX(s32 value)
{
    const s32 clamped = std::min(value, mBoundsMaxX);
    if (mBoundsMinX == clamped)
        return;
    mBoundsMinX = clamped;
    applyStreamingSettings();
}

void VoxelWorldComponent::setBoundsMaxX(s32 value)
{
    const s32 clamped = std::max(value, mBoundsMinX);
    if (mBoundsMaxX == clamped)
        return;
    mBoundsMaxX = clamped;
    applyStreamingSettings();
}

void VoxelWorldComponent::setBoundsMinZ(s32 value)
{
    const s32 clamped = std::min(value, mBoundsMaxZ);
    if (mBoundsMinZ == clamped)
        return;
    mBoundsMinZ = clamped;
    applyStreamingSettings();
}

void VoxelWorldComponent::setBoundsMaxZ(s32 value)
{
    const s32 clamped = std::max(value, mBoundsMinZ);
    if (mBoundsMaxZ == clamped)
        return;
    mBoundsMaxZ = clamped;
    applyStreamingSettings();
}

void VoxelWorldComponent::setMaxUploadsPerFrame(u32 value)
{
    const u32 clamped = std::max(1u, value);
    if (mMaxUploadsPerFrame == clamped)
        return;
    mMaxUploadsPerFrame = clamped;
    applyStreamingSettings();
}

void VoxelWorldComponent::setMaxGenerationJobs(u32 value)
{
    const u32 clamped = std::max(1u, value);
    if (mMaxGenerationJobs == clamped)
        return;
    mMaxGenerationJobs = clamped;
    applyStreamingSettings();
}

void VoxelWorldComponent::setMaxMeshJobs(u32 value)
{
    const u32 clamped = std::max(1u, value);
    if (mMaxMeshJobs == clamped)
        return;
    mMaxMeshJobs = clamped;
    applyStreamingSettings();
}

void VoxelWorldComponent::setAtlasFile(const std::string& file)
{
    if (mAtlasFile == file)
        return;
    mAtlasFile = file;
    mMaterialsReady = false;
    clearChunkRenders();
    regenerate();
}

void VoxelWorldComponent::regenerate()
{
    applyStreamingSettings();
    markTerrainDirty();
}

const GameObject* VoxelWorldComponent::resolveOrigin() const
{
    const GameObject* origin = owner();
    if (mOriginObjectId != 0 && owner() && owner()->scene())
    {
        if (const GameObject* found = owner()->scene()->findGameObject(mOriginObjectId))
            origin = found;
    }
    return origin;
}

void VoxelWorldComponent::onStart()
{
    if (const GameObject* origin = resolveOrigin())
        mStreamer.setOrigin(origin->globalPosition());
}

void VoxelWorldComponent::onUpdate(f32)
{
    // One row for the whole of it: streaming, unloading and the GPU uploads.
    // The breakdown only earns a slot back when this one starts showing.
    RADION_PROFILE_SCOPE("Voxel");

    ++mUpdateCounter;
    if (mTerrainDirty && mUpdateCounter > mTerrainDirtyUpdate + 1)
        applyTerrainSettings();

    if (const GameObject* origin = resolveOrigin())
        mStreamer.setOrigin(origin->globalPosition());

    mStreamer.update();

    // Budgeted like the uploads: dropping the view radius from twenty-seven to
    // six otherwise destroys thousands of objects and meshes inside one frame.
    Voxel::ChunkCoord unloaded;
    u32 destroyed = 0;
    while (destroyed < mMaxUnloadsPerFrame && mStreamer.popUnloadedChunk(unloaded))
    {
        ++destroyed;
        const auto it = mChunkRenders.find(unloaded);
        if (it == mChunkRenders.end())
            continue;
        destroyChunkRender(it->second);
        mChunkRenders.erase(it);
    }

    Voxel::VoxelStreamer::ChunkMesh mesh;
    while (mStreamer.popChunkMesh(mesh))
        applyChunkMesh(mesh.coordinate, mesh.mesh);
}

void VoxelWorldComponent::onDestroy()
{
    mStreamer.waitForJobs();
    clearChunkRenders();
}

void VoxelWorldComponent::ensureMaterials()
{
    if (mMaterialsReady)
        return;
    mOpaqueMaterial = makeVoxelMaterial(mAtlasFile, VoxelPass::Opaque);
    mCutoutMaterial = makeVoxelMaterial(mAtlasFile, VoxelPass::Cutout);
    mTransparentMaterial = makeVoxelMaterial(mAtlasFile, VoxelPass::Transparent);
    mMaterialsReady = true;
}

void VoxelWorldComponent::clearChunkRenders()
{
    for (auto& entry : mChunkRenders)
        destroyChunkRender(entry.second);
    mChunkRenders.clear();
}

void VoxelWorldComponent::destroyChunkRender(ChunkRender& render)
{
    if (render.mesh.valid())
        Assets().destroyMesh(render.mesh);
    render.mesh = MeshHandle();

    if (render.object && owner() && owner()->scene())
        owner()->scene()->destroy(render.object);
    render.object = nullptr;
}

void VoxelWorldComponent::applyChunkMesh(Voxel::ChunkCoord coordinate,
                                         const Voxel::VoxelMeshData& mesh)
{
    const auto existing = mChunkRenders.find(coordinate);
    if (mesh.empty())
    {
        if (existing != mChunkRenders.end())
        {
            destroyChunkRender(existing->second);
            mChunkRenders.erase(existing);
        }
        return;
    }

    ensureMaterials();
    ChunkRender& render = existing != mChunkRenders.end() ? existing->second
                                                          : mChunkRenders[coordinate];

    const Material materials[] = {mOpaqueMaterial, mCutoutMaterial, mTransparentMaterial};
    if (render.mesh.valid())
        Assets().replaceVoxelMesh(render.mesh, mesh.mesh, materials, 3);
    else
        render.mesh = Assets().createVoxelMesh(mesh.mesh, materials, 3);
    if (!render.mesh.valid())
        return;

    if (!render.object && owner() && owner()->scene())
    {
        render.object = owner()->scene()->createGameObject("Voxel.Chunk");
        if (render.object)
        {
            render.object->addComponent<MeshRenderer>(render.mesh);
            render.object->setPosition(
                glm::vec3(static_cast<f32>(coordinate.x * Voxel::VoxelChunk::Size),
                          static_cast<f32>(coordinate.y * Voxel::VoxelChunk::Size),
                          static_cast<f32>(coordinate.z * Voxel::VoxelChunk::Size)));
        }
    }
    if (render.object)
    {
        if (MeshRenderer* renderer = render.object->getComponent<MeshRenderer>())
            renderer->setMesh(render.mesh);
    }
}

bool VoxelWorldComponent::raycast(const glm::vec3& origin, const glm::vec3& direction,
                                  f32 maxDistance, Voxel::VoxelRaycastHit& hit) const
{
    return Voxel::raycast(mStreamer.world(), mStreamer.blocks(), origin, direction, maxDistance,
                          hit);
}

bool VoxelWorldComponent::placeBlock(const Voxel::VoxelRaycastHit& hit, Voxel::BlockId block)
{
    return mStreamer.setBlock(hit.previousBlock, block);
}

bool VoxelWorldComponent::removeBlock(const Voxel::VoxelRaycastHit& hit)
{
    return mStreamer.setBlock(hit.block, Voxel::AirBlockId);
}

Voxel::BlockId VoxelWorldComponent::blockIdByName(const std::string& name) const
{
    return mStreamer.blocks().findId(name);
}

bool VoxelWorldComponent::saveEdits(const char* filename)
{
    if (!filename || !*filename)
        return false;

    std::vector<u8> bytes;
    mStreamer.edits().write(bytes);
    ByteArray buffer;
    buffer.reserve(bytes.size());
    if (!bytes.empty() && !buffer.writeBytes(bytes.data(), bytes.size()))
        return false;
    if (!FileSystem::getSingleton().writeBinary(filename, buffer))
        return false;

    mEditsFile = filename;
    return true;
}

bool VoxelWorldComponent::loadEdits(const char* filename)
{
    if (!filename || !*filename)
        return false;

    const ByteArray buffer = FileSystem::getSingleton().readBinary(filename);
    if (buffer.size() == 0)
        return false;

    std::vector<u8> bytes(buffer.data(), buffer.data() + buffer.size());
    if (!mStreamer.edits().read(bytes))
    {
        Log::error("VoxelWorld: '%s' is not a voxel edit file", filename);
        return false;
    }

    mEditsFile = filename;
    // Chunks already in memory were built before these edits existed, so the
    // world has to stream back through the generator to pick them up.
    markTerrainDirty();
    return true;
}

AABB VoxelWorldComponent::blockBounds(Voxel::VoxelCoord block)
{
    AABB bounds;
    bounds.min = glm::vec3(static_cast<f32>(block.x), static_cast<f32>(block.y),
                           static_cast<f32>(block.z));
    bounds.max = bounds.min + glm::vec3(1.0f);
    return bounds;
}

} // namespace Radion
