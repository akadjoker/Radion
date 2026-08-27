#include "PCH.h"

#include "VoxelWorldComponent.h"

#include "AssetManager.h"
#include "GameObject.h"
#include "Material.h"
#include "MeshRenderer.h"
#include "Scene.h"

#include <algorithm>
#include <cmath>

namespace Radion
{

namespace
{
Material makeVoxelMaterial(const std::string& atlasFile, bool transparent)
{
    Material material;
    material.flags |= MaterialLit | MaterialVoxelAtlas;
    material.textures[SlotAlbedo].file = atlasFile;
    material.textures[SlotAlbedo].source = TextureSource::Static;
    material.textures[SlotAlbedo].texture =
        Assets().loadTexture(atlasFile, ColorSpace::sRGB, true, 5);
    material.blend = transparent ? BlendMode::Alpha : BlendMode::Opaque;
    if (transparent)
        material.flags |= MaterialNoDepthWrite;
    return material;
}
}

VoxelWorldComponent::VoxelWorldComponent() : Component(Type, ComponentEventUpdate)
{
    const char* names[] = {"grass", "dirt", "stone", "sand", "bedrock", "water"};
    const u16 atlasX[] = {0, 1, 2, 3, 4, 15};
    const u16 atlasY[] = {0, 0, 0, 0, 0, 14};
    for (usize index = 0; index < 6; ++index)
    {
        Voxel::BlockDefinition definition;
        definition.name = names[index];
        for (Voxel::BlockFaceMaterial& face : definition.faces)
        {
            face.atlasX = atlasX[index];
            face.atlasY = atlasY[index];
        }
        if (index == 5)
        {
            definition.solid = false;
            definition.transparent = true;
            definition.blocksLight = false;
            definition.renderType = Voxel::BlockRenderType::Transparent;
        }
        mBlocks.registerBlock(std::move(definition));
    }
}

void VoxelWorldComponent::setSeed(u32 seed)
{
    if (mSeed != seed)
    {
        mSeed = seed;
        regenerate();
    }
}

void VoxelWorldComponent::setChunkRadius(s32 radius)
{
    const s32 clamped = std::max(0, radius);
    if (mChunkRadius != clamped)
    {
        mChunkRadius = clamped;
        regenerate();
    }
}

void VoxelWorldComponent::setMinWorldY(s32 value)
{
    if (mMinWorldY != value)
    {
        mMinWorldY = std::min(value, mMaxWorldY);
        regenerate();
    }
}

void VoxelWorldComponent::setMaxWorldY(s32 value)
{
    if (mMaxWorldY != value)
    {
        mMaxWorldY = std::max(value, mMinWorldY);
        regenerate();
    }
}

void VoxelWorldComponent::setWaterLevel(s32 value)
{
    if (mWaterLevel != value)
    {
        mWaterLevel = value;
        regenerate();
    }
}

void VoxelWorldComponent::onStart()
{
    enqueueGeneration();
}

void VoxelWorldComponent::onUpdate(f32)
{
    finishGeneration();
    const GameObject* origin = owner();
    if (mOriginObjectId != 0 && owner() && owner()->scene())
        origin = owner()->scene()->findGameObject(mOriginObjectId);
    if (origin)
    {
        const glm::vec3 position = origin->globalPosition();
        const Voxel::ChunkCoord current = Voxel::VoxelWorld::chunkFor(
            {static_cast<s32>(std::floor(position.x)), static_cast<s32>(std::floor(position.y)),
             static_cast<s32>(std::floor(position.z))});
        if (mHasOriginChunk && !(current == mLastOriginChunk))
            regenerate();
        mLastOriginChunk = current;
        mHasOriginChunk = true;
    }
    usize rebuilt = 0;
    constexpr usize kMaxChunkRebuildsPerFrame = 1;
    mWorld.forEachChunk([this, &rebuilt](const Voxel::ChunkCoord& coordinate,
                                         Voxel::VoxelChunk& chunk)
                         {
                             if (rebuilt < kMaxChunkRebuildsPerFrame && chunk.dirty())
                             {
                                 rebuildChunk(coordinate, chunk);
                                 ++rebuilt;
                             }
                         });
}

void VoxelWorldComponent::onDestroy()
{
    if (mGenerationPending)
        Jobs().wait(mGenerationJob);
    clearChunkRenders();
}

void VoxelWorldComponent::generateWorldJob(void* userData)
{
    VoxelWorldComponent& component = *static_cast<VoxelWorldComponent*>(userData);
    const GenerationRequest& request = component.mGenerationRequest;
    Voxel::VoxelTerrain::Settings settings;
    settings.minWorldY = request.minWorldY;
    settings.maxWorldY = request.maxWorldY;
    settings.waterLevel = request.waterLevel;
    Voxel::VoxelTerrain terrain(component.mBlocks, request.seed, settings);
    const s32 minChunkY = Voxel::VoxelWorld::chunkFor({0, request.minWorldY, 0}).y;
    const s32 maxChunkY = Voxel::VoxelWorld::chunkFor({0, request.maxWorldY, 0}).y;
    for (s32 y = minChunkY; y <= maxChunkY; ++y)
        for (s32 z = request.center.z - request.chunkRadius;
             z <= request.center.z + request.chunkRadius; ++z)
            for (s32 x = request.center.x - request.chunkRadius;
                 x <= request.center.x + request.chunkRadius; ++x)
                terrain.generate(component.mWorldNext, {x, y, z});
}

void VoxelWorldComponent::enqueueGeneration()
{
    if (mGenerationPending || !owner())
        return;
    const GameObject* origin = owner();
    if (mOriginObjectId != 0 && owner()->scene())
        origin = owner()->scene()->findGameObject(mOriginObjectId);
    if (!origin)
        return;
    const glm::vec3 position = origin->globalPosition();
    mGenerationRequest = {mSeed,
                          mChunkRadius,
                          mMinWorldY,
                          mMaxWorldY,
                          mWaterLevel,
                          Voxel::VoxelWorld::chunkFor(
                              {static_cast<s32>(std::floor(position.x)),
                               static_cast<s32>(std::floor(position.y)),
                               static_cast<s32>(std::floor(position.z))})};
    mWorldNext.clear();
    Jobs().enqueue(mGenerationJob, &VoxelWorldComponent::generateWorldJob, this);
    mGenerationPending = true;
}

void VoxelWorldComponent::finishGeneration()
{
    if (!mGenerationPending || !Jobs().finished(mGenerationJob))
        return;
    std::swap(mWorld, mWorldNext);
    mGenerationPending = false;
    clearChunkRenders();
}

void VoxelWorldComponent::clearChunkRenders()
{
    for (auto& entry : mChunkRenders)
    {
        ChunkRender& render = entry.second;
        if (render.opaqueMesh.valid()) Assets().destroyMesh(render.opaqueMesh);
        if (render.cutoutMesh.valid()) Assets().destroyMesh(render.cutoutMesh);
        if (render.transparentMesh.valid()) Assets().destroyMesh(render.transparentMesh);
        if (owner() && owner()->scene())
        {
            if (render.opaque) owner()->scene()->destroy(render.opaque);
            if (render.cutout) owner()->scene()->destroy(render.cutout);
            if (render.transparent) owner()->scene()->destroy(render.transparent);
        }
    }
    mChunkRenders.clear();
}

void VoxelWorldComponent::rebuildChunk(const Voxel::ChunkCoord& coordinate,
                                       Voxel::VoxelChunk& chunk)
{
    Voxel::VoxelMesher::Settings settings;
    settings.atlasColumns = 16;
    settings.atlasRows = 16;
    settings.atlasTilePixels = 16;
    const Voxel::VoxelMeshData data =
        Voxel::VoxelMesher::buildChunk(mWorld, chunk, mBlocks, settings);
    ChunkRender& render = mChunkRenders[coordinate];
    const Material materials[] = {makeVoxelMaterial(mAtlasFile, false),
                                   makeVoxelMaterial(mAtlasFile, false),
                                   makeVoxelMaterial(mAtlasFile, true)};
    const MeshData* meshes[] = {&data.opaque, &data.cutout, &data.transparent};
    MeshHandle* handles[] = {&render.opaqueMesh, &render.cutoutMesh, &render.transparentMesh};
    GameObject** objects[] = {&render.opaque, &render.cutout, &render.transparent};
    const char* names[] = {"Voxel.Opaque", "Voxel.Cutout", "Voxel.Water"};
    for (usize index = 0; index < 3; ++index)
    {
        MeshData mesh = *meshes[index];
        mesh.materials.push_back(materials[index]);
        if (mesh.indices.empty())
        {
            if (handles[index]->valid())
                Assets().destroyMesh(*handles[index]);
            *handles[index] = MeshHandle();
            continue;
        }
        if (handles[index]->valid())
            Assets().replaceMesh(*handles[index], mesh);
        else
            *handles[index] = Assets().createDynamicMesh(mesh);
        if (!*objects[index] && owner() && owner()->scene())
        {
            *objects[index] = owner()->scene()->createGameObject(names[index]);
            (*objects[index])->addComponent<MeshRenderer>(*handles[index]);
        }
        if (*objects[index])
        {
            (*objects[index])->setPosition(glm::vec3(
                static_cast<f32>(coordinate.x * Voxel::VoxelChunk::Size),
                static_cast<f32>(coordinate.y * Voxel::VoxelChunk::Size),
                static_cast<f32>(coordinate.z * Voxel::VoxelChunk::Size)));
            (*objects[index])->getComponent<MeshRenderer>()->setMesh(*handles[index]);
        }
    }
    chunk.clearDirty();
}

void VoxelWorldComponent::regenerate()
{
    enqueueGeneration();
}

} // namespace Radion