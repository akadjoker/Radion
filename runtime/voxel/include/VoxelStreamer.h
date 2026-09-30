#ifndef RADION_VOXEL_STREAMER_H
#define RADION_VOXEL_STREAMER_H

#include "Thread.h"
#include "VoxelEditStore.h"
#include "VoxelMesher.h"
#include "VoxelNeighbourhood.h"
#include "VoxelTerrain.h"
#include "VoxelWorld.h"

#include "Math.h"

#include <unordered_set>
#include <vector>

namespace Radion
{
namespace Voxel
{

// Keeps chunks around an origin loaded and meshed, one at a time.
// Threading contract:
//  - the chunk map is touched by the main thread only;
//  - a generation job fills a chunk it owns, and the main thread moves it in;
//  - a mesh job reads a gathered VoxelNeighbourhood copy, never the map.
// `bounded` clamps the wanted set to a fixed box; everything else is identical.
class VoxelStreamer
{
public:
    struct Settings
    {
        bool bounded = false;
        // Chunks kept meshed; generation runs one ring wider so no chunk meshes against a missing neighbour (wall of seam faces).
        s32 viewRadius = 6;
        s32 boundsMinX = -8;
        s32 boundsMaxX = 8;
        s32 boundsMinZ = -8;
        s32 boundsMaxZ = 8;
        u32 maxGenerationJobs = 8;
        u32 maxMeshJobs = 8;
        // Meshes handed to the renderer per update: the frame budget against GPU upload stalls.
        u32 maxUploadsPerFrame = 4;
        // False runs jobs inline on the calling thread, for deterministic tests.
        bool useJobs = true;
    };

    struct ChunkMesh
    {
        ChunkCoord coordinate;
        VoxelMeshData mesh;
    };

    VoxelStreamer();
    ~VoxelStreamer();

    VoxelStreamer(const VoxelStreamer&) = delete;
    VoxelStreamer& operator=(const VoxelStreamer&) = delete;

    void configure(const Settings& settings);
    const Settings& settings() const { return mSettings; }

    void setBlocks(const BlockRegistry& blocks);
    const BlockRegistry& blocks() const { return mBlocks; }
    // Editing the palette waits for the jobs first: a mesh job reads the
    // registry on a worker thread.
    BlockId addBlock(const BlockDefinition& definition);
    bool replaceBlock(BlockId id, const BlockDefinition& definition);
    // Rebuilds the generator and drops every loaded chunk; edits are kept and
    // replayed as chunks come back.
    void setTerrain(u32 seed, const VoxelTerrain::Settings& terrain);
    const VoxelTerrain::Settings& terrainSettings() const { return mTerrainSettings; }
    u32 seed() const { return mSeed; }
    void setMesherSettings(const VoxelMesher::Settings& settings) { mMesherSettings = settings; }

    void setOrigin(const Math::vec3& position);
    // Collects finished jobs, unloads what left the radius and dispatches new
    // work. Main thread only.
    void update();

    // Meshes ready for upload, at most maxUploadsPerFrame per update.
    bool popChunkMesh(ChunkMesh& mesh);
    // Chunks that left the loaded set: their renderables must go with them.
    bool popUnloadedChunk(ChunkCoord& coordinate);

    // Records into the edit store and marks the affected chunks for remeshing.
    bool setBlock(VoxelCoord position, BlockId block);
    BlockId block(VoxelCoord position) const;

    VoxelWorld& world() { return mWorld; }
    const VoxelWorld& world() const { return mWorld; }
    VoxelEditStore& edits() { return mEdits; }
    const VoxelEditStore& edits() const { return mEdits; }

    // Drops every chunk and every pending job; settings and edits survive.
    void reset();
    void waitForJobs();

    usize loadedChunks() const { return mWorld.chunkCount(); }
    usize pendingGeneration() const { return mGenerating.size(); }
    usize pendingMeshing() const { return mMeshing.size(); }
    usize queuedMeshes() const { return mReadyMeshes.size() - mReadyCursor; }

private:
    struct GenerationTask
    {
        VoxelStreamer* streamer = nullptr;
        VoxelChunk* chunk = nullptr;
    };

    struct MeshTask
    {
        VoxelStreamer* streamer = nullptr;
        VoxelNeighbourhood neighbourhood;
        VoxelMeshData mesh;
    };

    struct Candidate
    {
        s32 distance = 0;
        ChunkCoord coordinate;
    };

    static void generationJob(void* userData);
    static void meshJob(void* userData);
    static bool nearestFirst(const Candidate& a, const Candidate& b);
    static bool furthestFirst(const Candidate& a, const Candidate& b);

    void collectFinished();
    void unloadDistant();
    void dispatchGeneration();
    void dispatchMeshing();
    bool insideBounds(ChunkCoord coordinate) const;
    bool wanted(ChunkCoord coordinate) const;
    bool renderable(ChunkCoord coordinate) const;
    bool neighboursLoaded(ChunkCoord coordinate) const;
    void markNeighboursDirty(ChunkCoord coordinate);
    void markDirty(ChunkCoord coordinate);
    void rebuildCandidates();

    // Tasks and chunks are fixed-size with capped in-flight count, so they are recycled: a moving camera otherwise churns ~1 MB per frame.
    GenerationTask* acquireGenerationTask();
    MeshTask* acquireMeshTask();
    VoxelChunk* acquireChunk(ChunkCoord coordinate);
    void recycle(GenerationTask* task);
    void recycle(MeshTask* task);
    void recycle(VoxelChunk* chunk);
    void releasePools();

    Settings mSettings;
    BlockRegistry mBlocks;
    VoxelTerrain* mTerrain = nullptr;
    VoxelTerrain::Settings mTerrainSettings;
    VoxelMesher::Settings mMesherSettings;
    u32 mSeed = 0;
    s32 mMinChunkY = 0;
    s32 mMaxChunkY = 0;

    VoxelWorld mWorld;
    VoxelEditStore mEdits;
    ChunkCoord mOrigin;
    bool mHasOrigin = false;

    std::unordered_set<ChunkCoord, ChunkCoordHash> mGenerating;
    std::unordered_set<ChunkCoord, ChunkCoordHash> mMeshing;
    // Chunks waiting to be meshed and the wanted set waiting to be generated: rebuilt when the origin moves, consumed by budget,
    // since rescanning the world or wanted volume every frame stalls the main thread at large radii.
    std::unordered_set<ChunkCoord, ChunkCoordHash> mDirty;
    bool mNeedsRescan = true;
    std::vector<ChunkMesh> mReadyMeshes;
    std::vector<ChunkCoord> mUnloaded;
    std::vector<Candidate> mCandidates;
    std::vector<Candidate> mMeshCandidates;
    std::vector<ChunkCoord> mLoaded;
    // Cursors, not erase-at-front: both queues are drained in order and only
    // cleared once empty.
    usize mReadyCursor = 0;
    usize mUnloadedCursor = 0;
    u32 mUploadsThisUpdate = 0;

    // Members, not locals: collectFinished() runs every frame and would allocate two vectors for usually nothing.
    std::vector<GenerationTask*> mFinishedGeneration;
    std::vector<MeshTask*> mFinishedMeshing;
    std::vector<GenerationTask*> mCollectedGeneration;
    std::vector<MeshTask*> mCollectedMeshing;
    std::vector<GenerationTask*> mGenerationPool;
    std::vector<MeshTask*> mMeshPool;
    std::vector<VoxelChunk*> mChunkPool;
    JobGroup mJobs;
    mutable Mutex mMutex;
};

} // namespace Voxel
} // namespace Radion

#endif // RADION_VOXEL_STREAMER_H
