#ifndef RADION_VOXEL_WORLD_COMPONENT_H
#define RADION_VOXEL_WORLD_COMPONENT_H

#include "Component.h"
#include "Material.h"
#include "VoxelCollision.h"
#include "VoxelRaycast.h"
#include "VoxelStreamer.h"

#include <string>
#include <unordered_map>

namespace Radion
{

// Drives a VoxelStreamer from a GameObject's position and turns the chunk
// meshes it hands back into renderables. The world itself, its generation and
// its edits live in the voxel module; this component only bridges to Scene.
class VoxelWorldComponent final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::VoxelWorld;

    u32 seed() const { return mSeed; }
    void setSeed(u32 seed);
    s32 chunkRadius() const { return mChunkRadius; }
    void setChunkRadius(s32 radius);

    // Terrain shape. Every setter here only marks the generator for a rebuild;
    // the world is dropped and streamed back once per frame, so dragging a
    // slider across a range costs one rebuild instead of one per frame.
    const Voxel::VoxelTerrain::Settings& terrainSettings() const { return mTerrain; }
    void setTerrainSettings(const Voxel::VoxelTerrain::Settings& settings);

    s32 minWorldY() const { return mTerrain.minWorldY; }
    void setMinWorldY(s32 value);
    s32 maxWorldY() const { return mTerrain.maxWorldY; }
    void setMaxWorldY(s32 value);
    // Both at once, clamped against each other rather than against whatever
    // the component happens to hold: reading a saved world back one field at a
    // time is otherwise clamped by the value not yet read.
    void setWorldHeightRange(s32 minValue, s32 maxValue);
    s32 waterLevel() const { return mTerrain.waterLevel; }
    void setWaterLevel(s32 value);

    f32 baseSurfaceHeight() const { return mTerrain.baseSurfaceHeight; }
    void setBaseSurfaceHeight(f32 value);
    f32 continentalAmplitude() const { return mTerrain.continentalAmplitude; }
    void setContinentalAmplitude(f32 value);
    f32 detailAmplitude() const { return mTerrain.detailAmplitude; }
    void setDetailAmplitude(f32 value);
    s32 minSurfaceHeight() const { return mTerrain.minSurfaceHeight; }
    void setMinSurfaceHeight(s32 value);
    s32 maxSurfaceHeight() const { return mTerrain.maxSurfaceHeight; }
    void setMaxSurfaceHeight(s32 value);
    void setSurfaceHeightRange(s32 minValue, s32 maxValue);
    f32 reliefFrequency() const { return mTerrain.reliefFrequency; }
    void setReliefFrequency(f32 value);

    bool biomes() const { return mTerrain.biomes; }
    void setBiomes(bool enabled);
    f32 biomeFrequency() const { return mTerrain.biomeFrequency; }
    void setBiomeFrequency(f32 value);

    bool caves() const { return mTerrain.caves; }
    void setCaves(bool enabled);
    f32 caveFrequency() const { return mTerrain.caveFrequency; }
    void setCaveFrequency(f32 value);
    f32 caveThreshold() const { return mTerrain.caveThreshold; }
    void setCaveThreshold(f32 value);
    s32 caveCeiling() const { return mTerrain.caveCeiling; }
    void setCaveCeiling(s32 value);

    bool trees() const { return mTerrain.trees; }
    void setTrees(bool enabled);
    f32 treeDensity() const { return mTerrain.treeDensity; }
    void setTreeDensity(f32 value);

    bool ores() const { return mTerrain.ores; }
    void setOres(bool enabled);

    bool ambientOcclusion() const { return mMesher.ambientOcclusion; }
    void setAmbientOcclusion(bool enabled);

    bool flat() const { return mTerrain.flat; }
    void setFlat(bool enabled);

    // The block palette, by atlas tile. A project defines what a world is made
    // of here rather than in code: the generator only ever asks the registry
    // for "grass", "stone" and the rest by name, so renaming or retiling a
    // block needs no rebuild.
    usize blockCount() const;
    const Voxel::BlockDefinition* blockDefinition(Voxel::BlockId id) const;
    Voxel::BlockId addBlock(const Voxel::BlockDefinition& definition);
    // Rewrites one definition in place, keeping its id and so every block
    // already placed in the world. Remeshes what is loaded.
    bool setBlockDefinition(Voxel::BlockId id, const Voxel::BlockDefinition& definition);
    void resetBlocksToDefault();

    bool bounded() const { return mBounded; }
    void setBounded(bool bounded);
    s32 boundsMinX() const { return mBoundsMinX; }
    void setBoundsMinX(s32 value);
    s32 boundsMaxX() const { return mBoundsMaxX; }
    void setBoundsMaxX(s32 value);
    s32 boundsMinZ() const { return mBoundsMinZ; }
    void setBoundsMinZ(s32 value);
    s32 boundsMaxZ() const { return mBoundsMaxZ; }
    void setBoundsMaxZ(s32 value);

    u32 maxUploadsPerFrame() const { return mMaxUploadsPerFrame; }
    void setMaxUploadsPerFrame(u32 value);
    u32 maxGenerationJobs() const { return mMaxGenerationJobs; }
    void setMaxGenerationJobs(u32 value);
    u32 maxMeshJobs() const { return mMaxMeshJobs; }
    void setMaxMeshJobs(u32 value);
    u32 maxUnloadsPerFrame() const { return mMaxUnloadsPerFrame; }
    void setMaxUnloadsPerFrame(u32 value) { mMaxUnloadsPerFrame = value > 0 ? value : 1; }

    const std::string& atlasFile() const { return mAtlasFile; }
    void setAtlasFile(const std::string& file);

    // A saved world is its seed plus the blocks somebody changed. The terrain
    // comes back from the settings the scene already carries; these are the
    // edits, written beside the scene the way Terrain writes its heightmap.
    const std::string& editsFile() const { return mEditsFile; }
    void setEditsFile(const std::string& file) { mEditsFile = file; }
    bool saveEdits(const char* filename);
    bool loadEdits(const char* filename);
    usize editedBlocks() const { return mStreamer.edits().recordCount(); }
    u64 originObjectId() const { return mOriginObjectId; }
    void setOriginObjectId(u64 id) { mOriginObjectId = id; }

    Voxel::VoxelWorld& world() { return mStreamer.world(); }
    const Voxel::VoxelWorld& world() const { return mStreamer.world(); }
    const Voxel::BlockRegistry& blocks() const { return mStreamer.blocks(); }
    Voxel::VoxelStreamer& streamer() { return mStreamer; }
    void regenerate();

    // Editing, in world block coordinates. A ray that hits nothing loaded
    // leaves `hit` untouched and returns false.
    bool raycast(const glm::vec3& origin, const glm::vec3& direction, f32 maxDistance,
                 Voxel::VoxelRaycastHit& hit) const;
    // Adds a block in the empty cell against the face that was hit: aim at the
    // top of one and it stacks, aim at a side and it butts against it.
    bool placeBlock(const Voxel::VoxelRaycastHit& hit, Voxel::BlockId block);
    // Rewrites the block that was hit, adding nothing. What a builder wants
    // for changing the material of a wall already standing.
    bool replaceBlock(const Voxel::VoxelRaycastHit& hit, Voxel::BlockId block);
    bool removeBlock(const Voxel::VoxelRaycastHit& hit);
    Voxel::BlockId blockIdByName(const std::string& name) const;
    // World-space bounds of one block, for a selection outline or a hit test.
    static AABB blockBounds(Voxel::VoxelCoord block);

    // Moves an axis-aligned body through the blocks, resolving one axis at a
    // time. Nothing is meshed for this: the grid is the collision geometry,
    // so breaking a block changes what the body can walk through in the same
    // frame.
    Voxel::VoxelMoveResult moveBox(const glm::vec3& position, const glm::vec3& halfExtents,
                                   const glm::vec3& displacement) const;
    bool boxOverlaps(const glm::vec3& position, const glm::vec3& halfExtents) const;

    usize loadedChunks() const { return mStreamer.loadedChunks(); }
    usize pendingGeneration() const { return mStreamer.pendingGeneration(); }
    usize pendingMeshing() const { return mStreamer.pendingMeshing(); }
    usize queuedMeshes() const { return mStreamer.queuedMeshes(); }
    usize renderedChunks() const { return mChunkRenders.size(); }
    usize worldMemoryBytes() const { return mStreamer.world().memoryBytes(); }

private:
    friend class GameObject;
    friend class Scene;

    VoxelWorldComponent();
    void onStart() override;
    void onUpdate(f32 deltaTime) override;
    void onDestroy() override;

    struct ChunkRender
    {
        GameObject* object = nullptr;
        MeshHandle mesh;
    };

    void markTerrainDirty();
    void applyTerrainSettings();
    void applyStreamingSettings();
    void ensureMaterials();
    void clearChunkRenders();
    void destroyChunkRender(ChunkRender& render);
    void applyChunkMesh(Voxel::ChunkCoord coordinate, const Voxel::VoxelMeshData& mesh);
    const GameObject* resolveOrigin() const;

    Voxel::VoxelStreamer mStreamer;
    Voxel::VoxelTerrain::Settings mTerrain;
    Voxel::VoxelMesher::Settings mMesher;
    bool mTerrainDirty = false;
    // Rebuilding drops every loaded chunk, so it waits for the settings to
    // stop moving: dragging a slider is one rebuild when the drag ends, not
    // one per frame it passes through.
    u64 mTerrainDirtyUpdate = 0;
    u64 mUpdateCounter = 0;
    u32 mSeed = 1337;
    s32 mChunkRadius = 6;
    bool mBounded = false;
    s32 mBoundsMinX = -8;
    s32 mBoundsMaxX = 8;
    s32 mBoundsMinZ = -8;
    s32 mBoundsMaxZ = 8;
    u32 mMaxUploadsPerFrame = 4;
    u32 mMaxUnloadsPerFrame = 32;
    u32 mMaxGenerationJobs = 8;
    u32 mMaxMeshJobs = 8;
    std::string mAtlasFile = "terrain.png";
    std::string mEditsFile;
    u64 mOriginObjectId = 0;

    Material mOpaqueMaterial;
    Material mCutoutMaterial;
    Material mTransparentMaterial;
    bool mMaterialsReady = false;
    std::unordered_map<Voxel::ChunkCoord, ChunkRender, Voxel::ChunkCoordHash> mChunkRenders;
};

} // namespace Radion

#endif // RADION_VOXEL_WORLD_COMPONENT_H
