#include "VoxelStreamer.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace Radion
{
namespace Voxel
{
namespace
{
const VoxelCoord NeighbourOffsets[] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0},
                                       {0, 1, 0},  {0, 0, -1}, {0, 0, 1}};

s32 chebyshevDistance(ChunkCoord a, ChunkCoord b)
{
    return std::max(std::abs(a.x - b.x), std::abs(a.z - b.z));
}

s32 squaredDistance(ChunkCoord a, ChunkCoord b)
{
    const s32 x = a.x - b.x;
    const s32 z = a.z - b.z;
    return x * x + z * z;
}

} // namespace

VoxelStreamer::VoxelStreamer() = default;

VoxelStreamer::~VoxelStreamer()
{
    waitForJobs();
    for (GenerationTask* task : mFinishedGeneration)
    {
        delete task->chunk;
        delete task;
    }
    for (MeshTask* task : mFinishedMeshing)
        delete task;
    releasePools();
    delete mTerrain;
}

VoxelStreamer::GenerationTask* VoxelStreamer::acquireGenerationTask()
{
    if (mGenerationPool.empty())
        return new GenerationTask();
    GenerationTask* task = mGenerationPool.back();
    mGenerationPool.pop_back();
    return task;
}

VoxelStreamer::MeshTask* VoxelStreamer::acquireMeshTask()
{
    if (mMeshPool.empty())
        return new MeshTask();
    MeshTask* task = mMeshPool.back();
    mMeshPool.pop_back();
    return task;
}

VoxelChunk* VoxelStreamer::acquireChunk(ChunkCoord coordinate)
{
    if (mChunkPool.empty())
        return new VoxelChunk(coordinate);
    VoxelChunk* chunk = mChunkPool.back();
    mChunkPool.pop_back();
    chunk->reset(coordinate);
    return chunk;
}

void VoxelStreamer::recycle(GenerationTask* task)
{
    task->chunk = nullptr;
    mGenerationPool.push_back(task);
}

void VoxelStreamer::recycle(MeshTask* task)
{
    task->mesh.clear();
    mMeshPool.push_back(task);
}

void VoxelStreamer::recycle(VoxelChunk* chunk)
{
    mChunkPool.push_back(chunk);
}

void VoxelStreamer::releasePools()
{
    for (GenerationTask* task : mGenerationPool)
        delete task;
    for (MeshTask* task : mMeshPool)
        delete task;
    for (VoxelChunk* chunk : mChunkPool)
        delete chunk;
    mGenerationPool.clear();
    mMeshPool.clear();
    mChunkPool.clear();
}

bool VoxelStreamer::nearestFirst(const Candidate& a, const Candidate& b)
{
    return a.distance < b.distance;
}

bool VoxelStreamer::furthestFirst(const Candidate& a, const Candidate& b)
{
    return a.distance > b.distance;
}

void VoxelStreamer::configure(const Settings& settings)
{
    mSettings = settings;
    mSettings.viewRadius = std::max(0, mSettings.viewRadius);
    mSettings.maxUploadsPerFrame = std::max(1u, mSettings.maxUploadsPerFrame);
    mSettings.maxGenerationJobs = std::max(1u, mSettings.maxGenerationJobs);
    mSettings.maxMeshJobs = std::max(1u, mSettings.maxMeshJobs);
    // The radius or the bounds may have moved, and both decide what is wanted.
    mNeedsRescan = true;
}

void VoxelStreamer::setBlocks(const BlockRegistry& blocks)
{
    waitForJobs();
    mBlocks = blocks;
    if (mTerrain)
        setTerrain(mSeed, mTerrainSettings);
}

BlockId VoxelStreamer::addBlock(const BlockDefinition& definition)
{
    waitForJobs();
    return mBlocks.registerBlock(definition);
}

bool VoxelStreamer::replaceBlock(BlockId id, const BlockDefinition& definition)
{
    waitForJobs();
    return mBlocks.replaceBlock(id, definition);
}

void VoxelStreamer::setTerrain(u32 seed, const VoxelTerrain::Settings& terrain)
{
    waitForJobs();
    delete mTerrain;
    mSeed = seed;
    mTerrainSettings = terrain;
    mTerrain = new VoxelTerrain(mBlocks, seed, terrain);
    mMinChunkY = VoxelWorld::chunkFor({0, terrain.minWorldY, 0}).y;
    mMaxChunkY = VoxelWorld::chunkFor({0, terrain.maxWorldY, 0}).y;
    reset();
}

void VoxelStreamer::setOrigin(const glm::vec3& position)
{
    const ChunkCoord coordinate = VoxelWorld::chunkFor({static_cast<s32>(std::floor(position.x)),
                                                        static_cast<s32>(std::floor(position.y)),
                                                        static_cast<s32>(std::floor(position.z))});
    if (mHasOrigin && coordinate == mOrigin)
        return;

    mOrigin = coordinate;
    mHasOrigin = true;
    mNeedsRescan = true;
}

void VoxelStreamer::update()
{
    mUploadsThisUpdate = 0;
    if (!mTerrain || !mHasOrigin)
        return;

    collectFinished();
    if (mNeedsRescan)
    {
        unloadDistant();
        rebuildCandidates();
    }
    dispatchGeneration();
    dispatchMeshing();
}

void VoxelStreamer::collectFinished()
{
    mCollectedGeneration.clear();
    mCollectedMeshing.clear();
    {
        ScopedLock lock(mMutex);
        mCollectedGeneration.swap(mFinishedGeneration);
        mCollectedMeshing.swap(mFinishedMeshing);
    }

    for (GenerationTask* task : mCollectedGeneration)
    {
        VoxelChunk* chunk = task->chunk;
        const ChunkCoord coordinate = chunk->coordinate();
        mGenerating.erase(coordinate);
        if (wanted(coordinate))
        {
            mEdits.apply(*chunk);
            mWorld.adopt(chunk);
            markDirty(coordinate);
            markNeighboursDirty(coordinate);
        }
        else
        {
            recycle(chunk);
        }
        recycle(task);
    }

    for (MeshTask* task : mCollectedMeshing)
    {
        const ChunkCoord coordinate = task->neighbourhood.coordinate();
        mMeshing.erase(coordinate);
        if (mWorld.findChunk(coordinate))
            mReadyMeshes.push_back({coordinate, std::move(task->mesh)});
        recycle(task);
    }
}

void VoxelStreamer::unloadDistant()
{
    // A chunk only leaves the radius when the origin moves, so this walks the
    // world on that frame and on no other.
    mWorld.collectCoordinates(mLoaded);
    for (const ChunkCoord& coordinate : mLoaded)
    {
        if (wanted(coordinate))
            continue;
        if (VoxelChunk* chunk = mWorld.detachChunk(coordinate))
            recycle(chunk);
        mDirty.erase(coordinate);
        mUnloaded.push_back(coordinate);
    }
}

void VoxelStreamer::rebuildCandidates()
{
    mNeedsRescan = false;
    mCandidates.clear();
    const s32 radius = mSettings.viewRadius + 1;
    for (s32 y = mMinChunkY; y <= mMaxChunkY; ++y)
    {
        for (s32 z = mOrigin.z - radius; z <= mOrigin.z + radius; ++z)
        {
            for (s32 x = mOrigin.x - radius; x <= mOrigin.x + radius; ++x)
            {
                const ChunkCoord coordinate = {x, y, z};
                if (!insideBounds(coordinate) || !mTerrain->intersects(coordinate))
                    continue;
                if (mWorld.findChunk(coordinate) ||
                    mGenerating.find(coordinate) != mGenerating.end())
                {
                    continue;
                }
                mCandidates.push_back({squaredDistance(coordinate, mOrigin), coordinate});
            }
        }
    }

    // Furthest first, so dispatch takes from the back and never has to shift
    // the front of the queue.
    std::sort(mCandidates.begin(), mCandidates.end(), &VoxelStreamer::furthestFirst);
}

void VoxelStreamer::dispatchGeneration()
{
    if (mGenerating.size() >= mSettings.maxGenerationJobs)
        return;

    while (mGenerating.size() < mSettings.maxGenerationJobs && !mCandidates.empty())
    {
        const ChunkCoord coordinate = mCandidates.back().coordinate;
        mCandidates.pop_back();
        // The queue was built before some of these arrived, so what is
        // already here or already in flight is simply dropped.
        if (mWorld.findChunk(coordinate) || mGenerating.find(coordinate) != mGenerating.end())
            continue;
        if (!wanted(coordinate))
            continue;

        GenerationTask* task = acquireGenerationTask();
        task->streamer = this;
        task->chunk = acquireChunk(coordinate);
        mGenerating.insert(coordinate);
        if (mSettings.useJobs)
            Jobs().enqueue(mJobs, &VoxelStreamer::generationJob, task);
        else
            generationJob(task);
    }
}

void VoxelStreamer::dispatchMeshing()
{
    if (mMeshing.size() >= mSettings.maxMeshJobs)
        return;

    // Only chunks that were touched are looked at. Walking the loaded world
    // instead costs the main thread a full scan every frame, which is what a
    // large view radius turns into a stall.
    mMeshCandidates.clear();
    for (auto it = mDirty.begin(); it != mDirty.end();)
    {
        const ChunkCoord coordinate = *it;
        VoxelChunk* chunk = mWorld.findChunk(coordinate);
        if (!chunk || !renderable(coordinate))
        {
            it = mDirty.erase(it);
            continue;
        }
        if (!chunk->dirty() || mMeshing.find(coordinate) != mMeshing.end())
        {
            it = mDirty.erase(it);
            continue;
        }
        // Waiting on a neighbour that has not arrived: stays in the set and
        // is looked at again when it does.
        if (!neighboursLoaded(coordinate))
        {
            ++it;
            continue;
        }
        // Open sky: no faces to build, but the renderer still has to be told
        // the chunk is empty so an old mesh of it goes away.
        if (chunk->empty())
        {
            chunk->clearDirty();
            mReadyMeshes.push_back({coordinate, VoxelMeshData()});
            it = mDirty.erase(it);
            continue;
        }
        mMeshCandidates.push_back({squaredDistance(coordinate, mOrigin), coordinate});
        it = mDirty.erase(it);
    }

    std::sort(mMeshCandidates.begin(), mMeshCandidates.end(), &VoxelStreamer::nearestFirst);
    for (usize index = 0; index < mMeshCandidates.size(); ++index)
    {
        const ChunkCoord coordinate = mMeshCandidates[index].coordinate;
        VoxelChunk* chunk = mWorld.findChunk(coordinate);
        if (!chunk)
            continue;
        if (mMeshing.size() >= mSettings.maxMeshJobs)
        {
            // Over budget: back into the set, so the next frame picks it up
            // without another scan of the world.
            mDirty.insert(coordinate);
            continue;
        }
        MeshTask* task = acquireMeshTask();
        task->streamer = this;
        task->neighbourhood.gather(mWorld, coordinate);
        chunk->clearDirty();
        mMeshing.insert(coordinate);
        if (mSettings.useJobs)
            Jobs().enqueue(mJobs, &VoxelStreamer::meshJob, task);
        else
            meshJob(task);
    }
}

void VoxelStreamer::generationJob(void* userData)
{
    GenerationTask* task = static_cast<GenerationTask*>(userData);
    VoxelStreamer& streamer = *task->streamer;
    streamer.mTerrain->generateChunk(*task->chunk);
    ScopedLock lock(streamer.mMutex);
    streamer.mFinishedGeneration.push_back(task);
}

void VoxelStreamer::meshJob(void* userData)
{
    MeshTask* task = static_cast<MeshTask*>(userData);
    VoxelStreamer& streamer = *task->streamer;
    task->mesh =
        VoxelMesher::buildChunk(task->neighbourhood, streamer.mBlocks, streamer.mMesherSettings);
    ScopedLock lock(streamer.mMutex);
    streamer.mFinishedMeshing.push_back(task);
}

bool VoxelStreamer::insideBounds(ChunkCoord coordinate) const
{
    if (!mSettings.bounded)
        return true;
    return coordinate.x >= mSettings.boundsMinX && coordinate.x <= mSettings.boundsMaxX &&
           coordinate.z >= mSettings.boundsMinZ && coordinate.z <= mSettings.boundsMaxZ;
}

bool VoxelStreamer::wanted(ChunkCoord coordinate) const
{
    if (!insideBounds(coordinate) || coordinate.y < mMinChunkY || coordinate.y > mMaxChunkY)
        return false;
    return chebyshevDistance(coordinate, mOrigin) <= mSettings.viewRadius + 2;
}

bool VoxelStreamer::renderable(ChunkCoord coordinate) const
{
    if (!insideBounds(coordinate) || coordinate.y < mMinChunkY || coordinate.y > mMaxChunkY)
        return false;
    return chebyshevDistance(coordinate, mOrigin) <= mSettings.viewRadius;
}

bool VoxelStreamer::neighboursLoaded(ChunkCoord coordinate) const
{
    for (const VoxelCoord& offset : NeighbourOffsets)
    {
        const ChunkCoord neighbour = {coordinate.x + offset.x, coordinate.y + offset.y,
                                      coordinate.z + offset.z};
        if (mWorld.findChunk(neighbour))
            continue;
        // Never generated and never will be: outside the world's vertical band
        // or outside a bounded world, which is air for good.
        if (neighbour.y < mMinChunkY || neighbour.y > mMaxChunkY || !insideBounds(neighbour))
            continue;
        if (!mTerrain->intersects(neighbour))
            continue;
        return false;
    }
    return true;
}

void VoxelStreamer::markDirty(ChunkCoord coordinate)
{
    mDirty.insert(coordinate);
}

void VoxelStreamer::markNeighboursDirty(ChunkCoord coordinate)
{
    for (const VoxelCoord& offset : NeighbourOffsets)
    {
        const ChunkCoord neighbourCoordinate = {coordinate.x + offset.x, coordinate.y + offset.y,
                                                coordinate.z + offset.z};
        if (VoxelChunk* neighbour = mWorld.findChunk(neighbourCoordinate))
        {
            neighbour->markDirty();
            markDirty(neighbourCoordinate);
        }
    }
}

bool VoxelStreamer::popChunkMesh(ChunkMesh& mesh)
{
    // Drained through a cursor: erasing at the front would move every mesh
    // still queued, and each one carries its whole vertex and index arrays.
    while (mUploadsThisUpdate < mSettings.maxUploadsPerFrame && mReadyCursor < mReadyMeshes.size())
    {
        ChunkMesh& ready = mReadyMeshes[mReadyCursor++];
        // Built for a chunk that has since been unloaded: nothing to upload.
        if (!mWorld.findChunk(ready.coordinate))
            continue;
        mesh = std::move(ready);
        ++mUploadsThisUpdate;
        if (mReadyCursor == mReadyMeshes.size())
        {
            mReadyMeshes.clear();
            mReadyCursor = 0;
        }
        return true;
    }

    if (mReadyCursor == mReadyMeshes.size())
    {
        mReadyMeshes.clear();
        mReadyCursor = 0;
    }
    return false;
}

bool VoxelStreamer::popUnloadedChunk(ChunkCoord& coordinate)
{
    if (mUnloadedCursor >= mUnloaded.size())
    {
        mUnloaded.clear();
        mUnloadedCursor = 0;
        return false;
    }

    coordinate = mUnloaded[mUnloadedCursor++];
    return true;
}

bool VoxelStreamer::setBlock(VoxelCoord position, BlockId block)
{
    if (!mWorld.findChunk(VoxelWorld::chunkFor(position)))
        return false;
    if (!mWorld.setBlock(position, block))
        return false;

    mEdits.record(position, block);
    // The world already flagged the chunk and any neighbour across the
    // boundary; the streamer needs their coordinates in its own queue.
    const ChunkCoord coordinate = VoxelWorld::chunkFor(position);
    markDirty(coordinate);
    markNeighboursDirty(coordinate);
    return true;
}

BlockId VoxelStreamer::block(VoxelCoord position) const
{
    return mWorld.block(position);
}

void VoxelStreamer::reset()
{
    waitForJobs();
    {
        ScopedLock lock(mMutex);
        for (GenerationTask* task : mFinishedGeneration)
        {
            recycle(task->chunk);
            recycle(task);
        }
        for (MeshTask* task : mFinishedMeshing)
            recycle(task);
        mFinishedGeneration.clear();
        mFinishedMeshing.clear();
    }
    mGenerating.clear();
    mMeshing.clear();
    mDirty.clear();
    mCandidates.clear();
    mMeshCandidates.clear();
    mReadyMeshes.clear();
    mReadyCursor = 0;
    mNeedsRescan = true;

    mWorld.collectCoordinates(mLoaded);
    for (const ChunkCoord& coordinate : mLoaded)
    {
        if (VoxelChunk* chunk = mWorld.detachChunk(coordinate))
            recycle(chunk);
        mUnloaded.push_back(coordinate);
    }
    mWorld.clear();
}

void VoxelStreamer::waitForJobs()
{
    // Unconditional: a group with nothing outstanding returns at once, and
    // asking mSettings first would skip the wait for work enqueued before
    // somebody turned jobs off - with the terrain generator deleted from
    // under a worker still reading it.
    Jobs().wait(mJobs);
}

} // namespace Voxel
} // namespace Radion
