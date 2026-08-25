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
#include <cmath>
#include <filesystem>
#include <vector>

using namespace Radion;
using namespace Radion::Voxel;

namespace
{
constexpr int kChunkRadius = 2; // chunks around the centre, per axis
constexpr u32 kSeed = 1337;
constexpr int kAtlasTile = 16; // texels per atlas tile
constexpr u16 kAtlasColumns = 4;
constexpr u16 kAtlasRows = 2;

// The world grows around this point. Streaming around a GameObject later is
// the same loop with this read from playerObject->position() every frame.
const glm::vec3 kWorldCenter(0.0f, 0.0f, 0.0f);

struct AtlasTile
{
    int x;
    int y;
};

void addSearchPathIfPresent(FileSystem& files, const std::filesystem::path& path)
{
    std::error_code error;
    if (std::filesystem::is_directory(path, error))
        files.addSearchPath(path.string());
}

BlockId registerBlock(BlockRegistry& registry, const char* name, bool solid,
                      BlockRenderType renderType, AtlasTile tile, AtlasTile top = {-1, -1})
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
    }
    if (top.x >= 0)
    {
        definition.faces[static_cast<usize>(BlockFace::PositiveY)].atlasX = static_cast<u16>(top.x);
        definition.faces[static_cast<usize>(BlockFace::PositiveY)].atlasY = static_cast<u16>(top.y);
    }
    return registry.registerBlock(std::move(definition));
}

TextureHandle makeAtlasTexture()
{
    const u32 width = static_cast<u32>(kAtlasTile) * kAtlasColumns;
    const u32 height = static_cast<u32>(kAtlasTile) * kAtlasRows;
    std::vector<u32> pixels(static_cast<usize>(width) * height);

    struct TileColor
    {
        u8 r, g, b, a;
    };
    const TileColor tiles[kAtlasRows][kAtlasColumns] = {
        {{106, 170, 64, 255}, {94, 144, 58, 255}, {134, 96, 67, 255}, {125, 125, 125, 255}},
        {{216, 200, 146, 255}, {60, 60, 64, 255}, {58, 114, 184, 160}, {255, 0, 255, 255}},
    };

    for (int cy = 0; cy < kAtlasRows; ++cy)
    {
        for (int cx = 0; cx < kAtlasColumns; ++cx)
        {
            const TileColor& tile = tiles[cy][cx];
            for (int ty = 0; ty < kAtlasTile; ++ty)
            {
                for (int tx = 0; tx < kAtlasTile; ++tx)
                {
                    const bool border =
                        tx == 0 || ty == 0 || tx == kAtlasTile - 1 || ty == kAtlasTile - 1;
                    const u8 r = border ? static_cast<u8>(tile.r * 3 / 4) : tile.r;
                    const u8 g = border ? static_cast<u8>(tile.g * 3 / 4) : tile.g;
                    const u8 b = border ? static_cast<u8>(tile.b * 3 / 4) : tile.b;
                    const int px = cx * kAtlasTile + tx;
                    const int py = cy * kAtlasTile + ty;
                    pixels[static_cast<usize>(py) * width + static_cast<usize>(px)] =
                        static_cast<u32>(r) | (static_cast<u32>(g) << 8) |
                        (static_cast<u32>(b) << 16) | (static_cast<u32>(tile.a) << 24);
                }
            }
        }
    }

    TextureDesc desc;
    desc.type = TextureType::Tex2D;
    desc.format = Format::RGBA8_sRGB;
    desc.width = width;
    desc.height = height;
    desc.mips = 1;
    desc.usage = TextureSampled;
    desc.data = pixels.data();
    desc.debugName = "voxel_atlas";
    return Assets().createTexture("voxel_atlas", desc);
}

Material makeMaterial(TextureHandle atlas, bool transparent)
{
    Material material;
    material.blend = transparent ? BlendMode::Alpha : BlendMode::Opaque;
    if (transparent)
        material.flags |= MaterialNoDepthWrite;

    SamplerDesc sampler;
    sampler.filter = Filter::Point; // crisp tiles, no bleeding at tile borders
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
    cameraObject->setPosition(kWorldCenter + glm::vec3(48.0f, 58.0f, -96.0f));
    cameraObject->lookAt(kWorldCenter + glm::vec3(0.0f, 14.0f, 0.0f));
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
    // Atlas tiles, column/row in the 4x2 grid: grass top, grass side, dirt,
    // stone, sand, bedrock, water, (unused).
    registerBlock(blocks, "grass", true, BlockRenderType::Opaque, {1, 0}, {0, 0});
    registerBlock(blocks, "dirt", true, BlockRenderType::Opaque, {2, 0});
    registerBlock(blocks, "stone", true, BlockRenderType::Opaque, {3, 0});
    registerBlock(blocks, "sand", true, BlockRenderType::Opaque, {0, 1});
    registerBlock(blocks, "bedrock", true, BlockRenderType::Opaque, {1, 1});
    registerBlock(blocks, "water", false, BlockRenderType::Transparent, {2, 1});

    const TextureHandle atlas = makeAtlasTexture();
    const Material terrainMaterial = makeMaterial(atlas, false);
    const Material waterMaterial = makeMaterial(atlas, true);

    VoxelWorld world;
    VoxelTerrain terrain(blocks, kSeed);

    const VoxelCoord centerVoxel{static_cast<s32>(std::floor(kWorldCenter.x)),
                                 static_cast<s32>(std::floor(kWorldCenter.y)),
                                 static_cast<s32>(std::floor(kWorldCenter.z))};
    const ChunkCoord centerChunk = VoxelWorld::chunkFor(centerVoxel);

    for (s32 z = centerChunk.z - kChunkRadius; z <= centerChunk.z + kChunkRadius; ++z)
        for (s32 x = centerChunk.x - kChunkRadius; x <= centerChunk.x + kChunkRadius; ++x)
            terrain.generate(world, {x, 0, z});

    VoxelMesher::Settings settings;
    settings.atlasColumns = kAtlasColumns;
    settings.atlasRows = kAtlasRows;

    for (s32 z = centerChunk.z - kChunkRadius; z <= centerChunk.z + kChunkRadius; ++z)
        for (s32 x = centerChunk.x - kChunkRadius; x <= centerChunk.x + kChunkRadius; ++x)
        {
            const ChunkCoord chunk{x, 0, z};
            const VoxelChunk* voxelChunk = world.findChunk(chunk);
            if (!voxelChunk)
                continue;
            const VoxelMeshData mesh =
                VoxelMesher::buildChunk(world, *voxelChunk, blocks, settings);
            addChunkMesh(*scene, "Chunk", chunk, mesh.opaque, terrainMaterial);
            addChunkMesh(*scene, "Chunk.Water", chunk, mesh.transparent, waterMaterial);
            addChunkMesh(*scene, "Chunk.Cutout", chunk, mesh.cutout, terrainMaterial);
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
