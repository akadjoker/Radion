#ifndef RADION_SCENE_BVH_H
#define RADION_SCENE_BVH_H

#include "BoundsTree.h"
#include "GPU.h"
#include "Math.h"
#include "Types.h"

#include <vector>

namespace Radion
{

class Frustum;
class MeshRenderer;

// Static submeshes only; dynamic objects stay on the linear scan (dozens/hundreds, not worth a tree rebuild on every move).
class SceneBVH
{
public:
    // Releases every entry's occlusion query when the Scene goes away.
    ~SceneBVH()
    {
        clear();
    }

    struct Hit
    {
        MeshRenderer* renderer = nullptr;
        u32 submeshIndex = 0;
        // Stable once build() returns (query() never reorders).
        u32 entryIndex = 0;
    };

    struct Stats
    {
        u32 nodeCount = 0;
        u32 entryCount = 0;
        u32 nodesVisited = 0;
        u32 entriesAccepted = 0;
    };

    void build(const std::vector<MeshRenderer*>& renderers);
    void clear();

    // cullSphere prunes whole subtrees during descent (a shadow cascade's wedge is wider than the light's range).
    // Scene::buildRenderList() still checks active()/isVisibleInHierarchy().
    void query(const Frustum& frustum, std::vector<Hit>& out, const Sphere* cullSphere = nullptr,
               const std::vector<Plane>* casterPlanes = nullptr);

    usize entryCount() const
    {
        return mEntries.size();
    }
    const Stats& stats() const
    {
        return mStats;
    }

    // World-space, computed once in build(); valid because static objects never move.
    static const AABB& emptyBoundsFallback()
    {
        static const AABB empty;
        return empty;
    }
    const AABB& entryBounds(u32 entryIndex) const
    {
        return entryIndex < mEntries.size() ? mEntries[entryIndex].bounds : emptyBoundsFallback();
    }

    // One persistent query per entry, created in build(); Scene::updateOcclusionQueries() owns the GPU calls.
    QueryHandle queryHandle(u32 entryIndex) const
    {
        return entryIndex < mEntries.size() ? mEntries[entryIndex].query : QueryHandle();
    }
    // Occluded only when the last 32 measurements all said so (from the reference's occlusionHistory);
    // popping in and out is worse than drawing something hidden.
    bool lastVisible(u32 entryIndex) const
    {
        return entryIndex >= mEntries.size() || mEntries[entryIndex].occlusionHistory != 0;
    }
    void setLastVisible(u32 entryIndex, bool visible, u32 currentFrame)
    {
        if (entryIndex >= mEntries.size())
            return;
        Entry& entry = mEntries[entryIndex];
        entry.occlusionHistory = (entry.occlusionHistory << 1) | (visible ? 1u : 0u);
        entry.verdictFrame = currentFrame;
    }
    u32 visibleFrameCount(u32 entryIndex) const
    {
        if (entryIndex >= mEntries.size())
            return 32;
        u32 bits = mEntries[entryIndex].occlusionHistory;
        u32 count = 0;
        while (bits)
        {
            count += bits & 1u;
            bits >>= 1;
        }
        return count;
    }
    // Frames since the last real measurement, saturating. A stale verdict may hide an entry long after the occluder moved; never used to draw less.
    u32 verdictAge(u32 entryIndex, u32 currentFrame) const
    {
        if (entryIndex >= mEntries.size() || mEntries[entryIndex].verdictFrame == 0)
            return ~0u;
        return currentFrame - mEntries[entryIndex].verdictFrame;
    }

    // Only a pending query is worth polling: glGetQueryObject on a never-begun query is a GL error every frame.
    // Relaunching a pending one would keep the answer "not yet".
    bool queryPending(u32 entryIndex) const
    {
        return entryIndex < mEntries.size() && mEntries[entryIndex].queryPending;
    }
    // Saturating; a never-launched entry reads as long ago.
    u32 framesSinceQuery(u32 entryIndex, u32 currentFrame) const
    {
        if (entryIndex >= mEntries.size() || mEntries[entryIndex].queryFrame == 0)
            return ~0u;
        return currentFrame - mEntries[entryIndex].queryFrame;
    }
    void markQueryLaunched(u32 entryIndex, u32 currentFrame)
    {
        if (entryIndex >= mEntries.size())
            return;
        mEntries[entryIndex].queryPending = true;
        mEntries[entryIndex].queryFrame = currentFrame;
    }
    void markQueryResolved(u32 entryIndex)
    {
        if (entryIndex < mEntries.size())
            mEntries[entryIndex].queryPending = false;
    }

    // lastVisible() is stale for an entry that just re-entered view; more than one frame's gap since last seen means treat it as fresh.
    bool justEnteredView(u32 entryIndex, u32 currentFrame) const
    {
        if (entryIndex >= mEntries.size())
            return false;
        // 0 means never seen, distinct from seen 1 frame ago.
        const u32 last = mEntries[entryIndex].lastSeenFrame;
        return last == 0 || currentFrame - last > 1;
    }
    void markSeenThisFrame(u32 entryIndex, u32 currentFrame)
    {
        if (entryIndex < mEntries.size())
            mEntries[entryIndex].lastSeenFrame = currentFrame;
    }

    // Green for a leaf, yellow for an internal node.
    void debugDraw() const;

private:
    struct Entry
    {
        MeshRenderer* renderer = nullptr;
        u32 submeshIndex = 0;
        AABB bounds; // world space, computed once at build()
        QueryHandle query;
        // All ones = nothing measured yet, reads as visible.
        u32 occlusionHistory = ~0u;
        // 0 (never seen) reads as long ago.
        u32 lastSeenFrame = 0;
        // 0 means never launched.
        u32 queryFrame = 0;
        // 0 means never measured: infinitely old, never trusted.
        u32 verdictFrame = 0;
        bool queryPending = false;
    };

    std::vector<Entry> mEntries;
    // The spatial work lives in BoundsTree; this class keeps the scene-specific parts (renderer, submesh, occlusion verdict).
    BoundsTree mTree;
    // Kept so a query allocates nothing.
    mutable std::vector<u32> mCandidates;
    // Parallel to mEntries.
    std::vector<AABB> mBounds;
    Stats mStats;
};

} // namespace Radion

#endif // RADION_SCENE_BVH_H
