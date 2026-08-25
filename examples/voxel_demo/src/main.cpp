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
#include "VoxelBlock.h"
#include "VoxelMesher.h"
#include "VoxelTerrain.h"
#include "VoxelWorld.h"

#include <algorithm>
#include <filesystem>

using namespace Radion;
using namespace Radion::Voxel;

namespace
{
constexpr int kChunkRadius = 2; // 5x5 chunks around the origin
constexpr u32 kSeed = 1337;

void addSearchPathIfPresent(FileSystem& files, const std::filesystem::path& path)
{
    std::error_code error;
    if (std::filesystem::is_directory(path, error))
        files.addSearchPath(path.string());
}

BlockId registerBlock(BlockRegistry& registry, const char* name, const glm::vec4& color,
                      bool solid = true, BlockRenderType renderType = BlockRenderType::Opaque,
                      const glm::vec4& topColor = glm::vec4(0.0f))
{
    BlockDefinition definition;
    definition.name = name;
    definition.solid = solid;
    definition.transparent = renderType != BlockRenderType::Opaque;
    definition.blocksLight = solid;
    definition.renderType = renderType;
    for (BlockFaceMaterial& face : definition.faces)
        face.color = color;
    if (topColor.a > 0.0f)
        definition.faces[static_cast<usize>(BlockFace::PositiveY)].color = topColor;
    return registry.registerBlock(std::move(definition));
}

MeshHandle uploadMesh(const MeshData& source)
{
    if (source.positions.empty() || source.indices.empty())
        return MeshHandle();

    MeshData data = source;
    if (data.materials.empty())
        data.materials.push_back(Material());
    return Assets().createMesh(data);
}

void addChunkMesh(Scene& scene, const char* name, ChunkCoord chunk, const MeshData& source)
{
    if (source.positions.empty() || source.indices.empty())
        return;

    const MeshHandle mesh = uploadMesh(source);
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
    cameraObject->setPosition(glm::vec3(48.0f, 58.0f, -96.0f));
    cameraObject->lookAt(glm::vec3(0.0f, 14.0f, 0.0f));
    FreeFly* fly = cameraObject->addComponent<FreeFly>();
    fly->setMoveSpeed(28.0f);
    fly->setSprintMultiplier(2.5f);
    scene->setActiveCamera(camera);

    GameObject* sunObject = scene->createGameObject("Sun");
    DirectionalLight* sun = sunObject->addComponent<DirectionalLight>();
    sun->setColor(glm::vec3(1.0f, 0.96f, 0.88f));
    sun->setIntensity(1.2f);
    sunObject->setPosition(glm::vec3(-60.0f, 80.0f, -40.0f));
    sunObject->lookAt(glm::vec3(0.0f, 0.0f, 0.0f));

    BlockRegistry blocks;
    registerBlock(blocks, "grass", glm::vec4(0.35f, 0.60f, 0.24f, 1.0f), true,
                  BlockRenderType::Opaque, glm::vec4(0.36f, 0.62f, 0.25f, 1.0f));
    registerBlock(blocks, "dirt", glm::vec4(0.51f, 0.36f, 0.24f, 1.0f));
    registerBlock(blocks, "stone", glm::vec4(0.48f, 0.48f, 0.50f, 1.0f));
    registerBlock(blocks, "sand", glm::vec4(0.83f, 0.76f, 0.54f, 1.0f));
    registerBlock(blocks, "bedrock", glm::vec4(0.22f, 0.22f, 0.24f, 1.0f));
    registerBlock(blocks, "water", glm::vec4(0.25f, 0.45f, 0.72f, 1.0f), false,
                  BlockRenderType::Transparent);

    VoxelWorld world;
    VoxelTerrain terrain(blocks, kSeed);
    for (s32 z = -kChunkRadius; z <= kChunkRadius; ++z)
        for (s32 x = -kChunkRadius; x <= kChunkRadius; ++x)
            terrain.generate(world, {x, 0, z});

    for (s32 z = -kChunkRadius; z <= kChunkRadius; ++z)
        for (s32 x = -kChunkRadius; x <= kChunkRadius; ++x)
        {
            const ChunkCoord chunk{x, 0, z};
            const VoxelChunk* voxelChunk = world.findChunk(chunk);
            if (!voxelChunk)
                continue;
            const VoxelMeshData mesh = VoxelMesher::buildChunk(world, *voxelChunk, blocks);
            addChunkMesh(*scene, "Chunk", chunk, mesh.opaque);
            addChunkMesh(*scene, "Chunk.Water", chunk, mesh.transparent);
            addChunkMesh(*scene, "Chunk.Cutout", chunk, mesh.cutout);
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
