#include "AssetManager.h"
#include "AssetPaths.h"
#include "Camera.h"
#include "CameraControllers.h"
#include "Engine.h"
#include "FileSystem.h"
#include "GameObject.h"
#include "Light.h"
#include "Material.h"
#include "Mesh.h"
#include "MeshRenderer.h"
#include "Scene.h"
#include "Thread.h"
#include "VoxelBlock.h"
#include "VoxelMesher.h"
#include "VoxelTerrain.h"
#include "VoxelWorld.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <vector>

using namespace Radion;
using namespace Radion::Voxel;

namespace
{
constexpr int kChunkRadius = 2; // chunks around the centre, per axis
constexpr u32 kSeed = 1337;
constexpr s32 kWorldMinY = -64;
constexpr s32 kWorldMaxY = 127;
constexpr s32 kMinChunkY = kWorldMinY / VoxelChunk::Size;
constexpr s32 kMaxChunkY = kWorldMaxY / VoxelChunk::Size;
constexpr u16 kAtlasTilePixels = 16;
constexpr u16 kAtlasColumns = 16;
constexpr u16 kAtlasRows = 16;
constexpr u32 kAtlasMipCount = 5;
constexpr const char* kTerrainAtlasFile = "terrain.png";

// The world grows around this point. Streaming around a GameObject later is
// the same loop with this read from playerObject->position() every frame.
const glm::vec3 kWorldCenter(0.0f, 0.0f, 0.0f);

struct AtlasTile
{
    int x;
    int y;
};

struct ChunkMeshBuild
{
    const VoxelWorld* world = nullptr;
    const VoxelChunk* chunk = nullptr;
    const BlockRegistry* blocks = nullptr;
    VoxelMesher::Settings settings;
    VoxelMeshData mesh;
};

void buildChunkMesh(void* userData)
{
    ChunkMeshBuild& build = *static_cast<ChunkMeshBuild*>(userData);
    build.mesh = VoxelMesher::buildChunk(*build.world, *build.chunk, *build.blocks, build.settings);
}

void addSearchPathIfPresent(FileSystem& files, const std::filesystem::path& path)
{
    std::error_code error;
    if (std::filesystem::is_directory(path, error))
        files.addSearchPath(path.string());
}

BlockId registerBlock(BlockRegistry& registry, const char* name, bool solid,
                      BlockRenderType renderType, AtlasTile tile, AtlasTile top = {-1, -1},
                      bool flipVertical = false)
{
    BlockDefinition definition;
    definition.name = name;
    definition.solid = solid;
    definition.transparent = renderType != BlockRenderType::Opaque;
    definition.blocksLight = solid;
    definition.renderType = renderType;
    for (BlockFaceMaterial& face : definition.faces)
    {
        face.atlasX = static_cast<u16>(tile.x);
        face.atlasY = static_cast<u16>(tile.y);
        face.flipVertical = flipVertical;
    }
    if (top.x >= 0)
    {
        definition.faces[static_cast<usize>(BlockFace::PositiveY)].atlasX = static_cast<u16>(top.x);
        definition.faces[static_cast<usize>(BlockFace::PositiveY)].atlasY = static_cast<u16>(top.y);
    }
    return registry.registerBlock(std::move(definition));
}

Material makeMaterial(TextureHandle atlas, bool transparent)
{
    Material material;
    material.flags |= MaterialVoxelAtlas;
    const f32 atlasWidth = static_cast<f32>(kAtlasColumns * kAtlasTilePixels);
    const f32 atlasHeight = static_cast<f32>(kAtlasRows * kAtlasTilePixels);
    material.params.custom0 = {1.0f / kAtlasColumns, 1.0f / kAtlasRows, 0.5f / atlasWidth,
                               0.5f / atlasHeight};
    material.blend = transparent ? BlendMode::Alpha : BlendMode::Opaque;
    if (transparent)
        material.flags |= MaterialNoDepthWrite;

    SamplerDesc sampler;
    sampler.filter = Filter::Anisotropic;
    sampler.anisotropy = 8.0f;
    sampler.wrapU = Wrap::Clamp;
    sampler.wrapV = Wrap::Clamp;

    material.textures[SlotAlbedo].texture = atlas;
    material.textures[SlotAlbedo].sampler = Assets().getSampler(sampler);
    return material;
}

MeshHandle uploadMesh(const MeshData& source, const Material& material)
{
    if (source.positions.empty() || source.indices.empty())
        return MeshHandle();

    MeshData data = source;
    data.materials.push_back(material);
    return Assets().createMesh(data);
}

void addChunkMesh(Scene& scene, const char* name, ChunkCoord chunk, const MeshData& source,
                  const Material& material)
{
    if (source.positions.empty() || source.indices.empty())
        return;

    const MeshHandle mesh = uploadMesh(source, material);
    if (!mesh.valid())
        return;

    GameObject* object = scene.createGameObject(name);
    object->addComponent<MeshRenderer>(mesh);
    object->setPosition(glm::vec3(static_cast<f32>(chunk.x * VoxelChunk::Size),
                                  static_cast<f32>(chunk.y * VoxelChunk::Size),
                                  static_cast<f32>(chunk.z * VoxelChunk::Size)));
}
} // namespace

int main(int, char**)
{
    FileSystem& files = FileSystem::getSingleton();
    const std::filesystem::path builtInAssets = resolveAssetDirectory(RADION_ASSET_DIR);
    addSearchPathIfPresent(files, builtInAssets);
    addSearchPathIfPresent(files, builtInAssets / "shaders");
    addSearchPathIfPresent(files, builtInAssets / "textures");

    Engine engine;
    EngineConfig config;
    config.title = "Radion Voxel Demo";
    config.width = 1280;
    config.height = 720;
    if (!engine.initialize(config))
        return 1;
    engine.setBuiltinPanelsVisible(false);
    engine.setImGuiVisible(false);

    Scene* scene = engine.createScene();
    if (!scene || !engine.setActiveScene(scene))
    {
        engine.shutdown();
        return 1;
    }
    scene->setRunningInEditor(false);

    // Camera: free fly starting above and in front of the generated terrain.
    GameObject* cameraObject = scene->createGameObject("Camera");
    Camera* camera = cameraObject->addComponent<Camera>();
    camera->setPerspective(60.0f, 16.0f / 9.0f, 0.1f, 1000.0f);
    cameraObject->setPosition(kWorldCenter + glm::vec3(48.0f, 112.0f, -96.0f));
    cameraObject->lookAt(kWorldCenter + glm::vec3(0.0f, 42.0f, 0.0f));
    FreeFly* fly = cameraObject->addComponent<FreeFly>();
    fly->setMoveSpeed(28.0f);
    fly->setSprintMultiplier(2.5f);
    scene->setActiveCamera(camera);

    GameObject* sunObject = scene->createGameObject("Sun");
    DirectionalLight* sun = sunObject->addComponent<DirectionalLight>();
    sun->setColor(glm::vec3(1.0f, 0.96f, 0.88f));
    sun->setIntensity(1.2f);
    sunObject->setPosition(kWorldCenter + glm::vec3(-60.0f, 80.0f, -40.0f));
    sunObject->lookAt(kWorldCenter);

    BlockRegistry blocks;
    // terrain.png is a general-purpose tile sheet rather than a voxel atlas.
    // Keep the temporary terrain coherent while a dedicated voxel atlas is authored.
    constexpr AtlasTile terrainTile{0, 0};
    registerBlock(blocks, "grass", true, BlockRenderType::Opaque, terrainTile);
    registerBlock(blocks, "dirt", true, BlockRenderType::Opaque, terrainTile);
    registerBlock(blocks, "stone", true, BlockRenderType::Opaque, terrainTile);
    registerBlock(blocks, "sand", true, BlockRenderType::Opaque, terrainTile);
    registerBlock(blocks, "bedrock", true, BlockRenderType::Opaque, terrainTile);
    registerBlock(blocks, "water", false, BlockRenderType::Transparent, terrainTile);

    const TextureHandle atlas =
        Assets().loadTexture(kTerrainAtlasFile, ColorSpace::sRGB, true, kAtlasMipCount);
    const Material terrainMaterial = makeMaterial(atlas, false);
    const Material waterMaterial = makeMaterial(atlas, true);

    VoxelWorld world;
    VoxelTerrain::Settings terrainSettings;
    terrainSettings.minWorldY = kWorldMinY;
    terrainSettings.maxWorldY = kWorldMaxY;
    terrainSettings.waterLevel = 20;
    terrainSettings.minSurfaceHeight = 4;
    terrainSettings.maxSurfaceHeight = 96;
    terrainSettings.baseSurfaceHeight = 48.0f;
    terrainSettings.continentalAmplitude = 28.0f;
    terrainSettings.detailAmplitude = 8.0f;
    VoxelTerrain terrain(blocks, kSeed, terrainSettings);

    const VoxelCoord centerVoxel{static_cast<s32>(std::floor(kWorldCenter.x)),
                                 static_cast<s32>(std::floor(kWorldCenter.y)),
                                 static_cast<s32>(std::floor(kWorldCenter.z))};
    const ChunkCoord centerChunk = VoxelWorld::chunkFor(centerVoxel);

    for (s32 y = kMinChunkY; y <= kMaxChunkY; ++y)
        for (s32 z = centerChunk.z - kChunkRadius; z <= centerChunk.z + kChunkRadius; ++z)
            for (s32 x = centerChunk.x - kChunkRadius; x <= centerChunk.x + kChunkRadius; ++x)
                terrain.generate(world, {x, y, z});

    VoxelMesher::Settings settings;
    settings.atlasColumns = kAtlasColumns;
    settings.atlasRows = kAtlasRows;
    settings.atlasTilePixels = kAtlasTilePixels;

    std::vector<ChunkMeshBuild> meshBuilds;
    meshBuilds.reserve(static_cast<usize>(kMaxChunkY - kMinChunkY + 1) *
                       static_cast<usize>((kChunkRadius * 2 + 1) * (kChunkRadius * 2 + 1)));
    for (s32 y = kMinChunkY; y <= kMaxChunkY; ++y)
        for (s32 z = centerChunk.z - kChunkRadius; z <= centerChunk.z + kChunkRadius; ++z)
            for (s32 x = centerChunk.x - kChunkRadius; x <= centerChunk.x + kChunkRadius; ++x)
            {
                const VoxelChunk* voxelChunk = world.findChunk({x, y, z});
                if (voxelChunk)
                    meshBuilds.push_back({&world, voxelChunk, &blocks, settings, {}});
            }

    JobGroup meshJobs;
    for (ChunkMeshBuild& build : meshBuilds)
        Jobs().enqueue(meshJobs, buildChunkMesh, &build);
    Jobs().wait(meshJobs);

    for (const ChunkMeshBuild& build : meshBuilds)
        {
        const ChunkCoord chunk = build.chunk->coordinate();
        addChunkMesh(*scene, "Chunk", chunk, build.mesh.opaque, terrainMaterial);
        addChunkMesh(*scene, "Chunk.Water", chunk, build.mesh.transparent, waterMaterial);
        addChunkMesh(*scene, "Chunk.Cutout", chunk, build.mesh.cutout, terrainMaterial);
    }

    while (engine.update())
    {
        const f32 deltaTime = std::min(engine.getWindow().getDeltaTime(), 0.1f);
        scene->update(deltaTime);
        engine.render(*scene);
        engine.flip();
    }

    engine.shutdown();
    return 0;
}
