#ifndef RADION_VOXEL_WORLD_COMPONENT_H
#define RADION_VOXEL_WORLD_COMPONENT_H

#include "Component.h"
#include "Thread.h"
#include "VoxelTerrain.h"
#include "VoxelMesher.h"
#include "VoxelWorld.h"

#include <unordered_map>
#include <string>

namespace Radion
{

class VoxelWorldComponent final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::VoxelWorld;

    u32 seed() const { return mSeed; }
    void setSeed(u32 seed);
    s32 chunkRadius() const { return mChunkRadius; }
    void setChunkRadius(s32 radius);
    s32 minWorldY() const { return mMinWorldY; }
    void setMinWorldY(s32 value);
    s32 maxWorldY() const { return mMaxWorldY; }
    void setMaxWorldY(s32 value);
    s32 waterLevel() const { return mWaterLevel; }
    void setWaterLevel(s32 value);
    const std::string& atlasFile() const { return mAtlasFile; }
    void setAtlasFile(const std::string& file) { mAtlasFile = file; }
    u64 originObjectId() const { return mOriginObjectId; }
    void setOriginObjectId(u64 id) { mOriginObjectId = id; }

    Voxel::VoxelWorld& world() { return mWorld; }
    const Voxel::VoxelWorld& world() const { return mWorld; }
    const Voxel::BlockRegistry& blocks() const { return mBlocks; }
    void regenerate();

private:
    friend class GameObject;
    friend class Scene;

    VoxelWorldComponent();
    void onStart() override;
    void onUpdate(f32 deltaTime) override;
    void onDestroy() override;

    struct ChunkRender
    {
        GameObject* opaque = nullptr;
        GameObject* cutout = nullptr;
        GameObject* transparent = nullptr;
        MeshHandle opaqueMesh;
        MeshHandle cutoutMesh;
        MeshHandle transparentMesh;
    };

    void clearChunkRenders();
    void rebuildChunk(const Voxel::ChunkCoord& coordinate, Voxel::VoxelChunk& chunk);
    static void generateWorldJob(void* userData);
    void enqueueGeneration();
    void finishGeneration();

    struct GenerationRequest
    {
        u32 seed = 0;
        s32 chunkRadius = 0;
        s32 minWorldY = 0;
        s32 maxWorldY = 0;
        s32 waterLevel = 0;
        Voxel::ChunkCoord center;
    };

    Voxel::BlockRegistry mBlocks;
    Voxel::VoxelWorld mWorld;
    u32 mSeed = 1337;
    s32 mChunkRadius = 2;
    s32 mMinWorldY = -64;
    s32 mMaxWorldY = 127;
    s32 mWaterLevel = 20;
    std::string mAtlasFile = "terrain.png";
    u64 mOriginObjectId = 0;
    Voxel::ChunkCoord mLastOriginChunk;
    bool mHasOriginChunk = false;
    std::unordered_map<Voxel::ChunkCoord, ChunkRender, Voxel::ChunkCoordHash> mChunkRenders;
    Voxel::VoxelWorld mWorldNext;
    GenerationRequest mGenerationRequest;
    JobGroup mGenerationJob;
    bool mGenerationPending = false;
};

} // namespace Radion

#endif // RADION_VOXEL_WORLD_COMPONENT_H