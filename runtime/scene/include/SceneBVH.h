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

// Spatial index over the submeshes of every MeshRenderer whose owner
// GameObject::isStatic() - built once, queried every frame instead of the
// linear scan buildRenderList() used to do. Dynamic objects stay on that
// linear scan: they are counted in dozens/hundreds, not thousands, and a
// tree that has to rebuild whenever one of them moves is not worth it.
class SceneBVH
{
public:
    // Releases every entry's occlusion query - clear() does the same thing
    // build() starts with, this just makes sure it also runs when a Scene
    // (and its SceneBVH member) goes away, not only on the next rebuild.
    ~SceneBVH()
    {
        clear();
    }

    struct Hit
    {
        MeshRenderer* renderer = nullptr;
        u32 submeshIndex = 0;
        // Index into mEntries - stable once build() returns (query() never
        // reorders), so a caller can round-trip through lastVisible()/
        // queryHandle() below for the occlusion query pass.
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

    // Appends one Hit per submesh neither the frustum nor `cullSphere` (when
    // given) rejects. cullSphere prunes whole subtrees during the descent -
    // a shadow cascade's frustum is a wide wedge but nothing outside the
    // light's own range can ever be lit, so this is what actually keeps a
    // large scene from walking most of the tree per cascade.
    // Scene::buildRenderList() still checks active()/isVisibleInHierarchy()
    // on the owning renderer, and RenderList::submitSubmesh() still does its
    // own AABB test on top of this.
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

    // World-space, computed once in build() from the object's transform at
    // the time - valid for as long as that promise holds (static means it
    // never moves again), so a caller with an entryIndex from a Hit never
    // needs to redo transformAABB(submesh.bounds, renderer->owner()->
    // globalTransform()) itself; it is the same matrix multiply landing on
    // the same numbers.
    static const AABB& emptyBoundsFallback()
    {
        static const AABB empty;
        return empty;
    }
    const AABB& entryBounds(u32 entryIndex) const
    {
        return entryIndex < mEntries.size() ? mEntries[entryIndex].bounds : emptyBoundsFallback();
    }

    // One persistent hardware occlusion query per entry, created in build()
    // and kept for as long as the entry lives - never recreated per frame.
    // Whoever launches/reads them (Scene::updateOcclusionQueries()) owns the
    // GPU calls; this class only stores the handle and the last verdict.
    QueryHandle queryHandle(u32 entryIndex) const
    {
        return entryIndex < mEntries.size() ? mEntries[entryIndex].query : QueryHandle();
    }
    // Occluded only when the last 32 measurements ALL said so. One bit per
    // verdict, shifted in - so a single query that came back wrong, or one
    // frame where the entry happened to be behind something passing in front
    // of it, cannot hide it. Taken from the reference (wiScene.h's
    // occlusionHistory), which is the same trick and for the same reason:
    // popping in and out is far worse to look at than drawing something that
    // turned out to be hidden.
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
    // How many of the last 32 verdicts said visible - for a debug view, and
    // for telling "just went out of sight" from "has been hidden for a while".
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
    // Frames since the last real measurement landed, saturating. Skipping a
    // draw on a verdict is only safe while the verdict still describes the
    // scene: nothing is re-tested every single frame (see the query latency
    // in Scene::updateOcclusionQueries()), so a caller that trusts one
    // indefinitely holds an entry invisible long after whatever was in front
    // of it moved away. Never used to justify drawing less - only more.
    u32 verdictAge(u32 entryIndex, u32 currentFrame) const
    {
        if (entryIndex >= mEntries.size() || mEntries[entryIndex].verdictFrame == 0)
            return ~0u;
        return currentFrame - mEntries[entryIndex].verdictFrame;
    }

    // A query is pending from the begin/end that launched it until its
    // result is actually read back. Only a pending one is ever worth
    // polling: glGetQueryObject{iv,uiv} on a query name that has never had
    // a matching glBeginQuery/glEndQuery is invalid per the GL spec, not a
    // harmless no-op - it is a GL error every frame, and with the debug
    // context this build turns on, a synchronous driver callback on every
    // one. Relaunching a query that is still pending is the other half:
    // the frame it was launched in is the only thing that says whether its
    // result can be had without blocking, and overwriting it every frame
    // means that answer is always "not yet".
    bool queryPending(u32 entryIndex) const
    {
        return entryIndex < mEntries.size() && mEntries[entryIndex].queryPending;
    }
    // Frames since the launch, saturating - a never-launched entry reads as
    // "long ago", which no caller can mistake for a result being ready
    // because queryPending() is false for it anyway.
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

    // lastVisible() only means anything for an entry the camera's frustum
    // has actually been near recently - one that just re-entered view after
    // a while off-screen (a fast turn, a door opening the view into a new
    // room) is still carrying whatever verdict a query launched frames ago
    // happened to land on, which has nothing to do with what's in front of
    // it now. `currentFrame` is Scene's own per-buildRenderList() counter;
    // more than one frame's gap since this entry was last in the query
    // means treat it as freshly entered, not "still occluded from before".
    bool justEnteredView(u32 entryIndex, u32 currentFrame) const
    {
        if (entryIndex >= mEntries.size())
            return false;
        // 0 means never seen at all - not the same as "seen 1 frame ago",
        // and relying on currentFrame happening to be small enough for the
        // subtraction to fall out right on its own is not the same as this
        // actually saying what it means.
        const u32 last = mEntries[entryIndex].lastSeenFrame;
        return last == 0 || currentFrame - last > 1;
    }
    void markSeenThisFrame(u32 entryIndex, u32 currentFrame)
    {
        if (entryIndex < mEntries.size())
            mEntries[entryIndex].lastSeenFrame = currentFrame;
    }

    // Wireframe box per node, green for a leaf and yellow for an internal
    // one - a debug panel toggle, not called on its own.
    void debugDraw() const;

private:
    struct Entry
    {
        MeshRenderer* renderer = nullptr;
        u32 submeshIndex = 0;
        AABB bounds; // world space, computed once at build()
        QueryHandle query;
        // One bit per measurement, 32 frames of history. All ones means
        // nothing has been measured yet, which reads as visible - an entry
        // must survive real measurements before anything starts skipping it.
        u32 occlusionHistory = ~0u;
        // See justEnteredView()/markSeenThisFrame() - 0 (never seen) reads
        // as "long ago" against any real frame counter, which is correct:
        // an entry that has never been in the camera's frustum has no stale
        // verdict to distrust in the first place.
        u32 lastSeenFrame = 0;
        // See queryPending()/framesSinceQuery() - 0 means never launched.
        u32 queryFrame = 0;
        // See verdictAge() - 0 means never measured, which reads as infinitely
        // old and so is never trusted to skip a draw.
        u32 verdictFrame = 0;
        bool queryPending = false;
    };

    std::vector<Entry> mEntries;
    // The spatial work, which knows nothing about renderers or occlusion
    // queries. This class used to carry its own median-split BVH; it is the
    // same algorithm, moved somewhere the physics and the tests can reach it
    // too. What stays here is everything that is about the SCENE - which
    // renderer, which submesh, and the occlusion verdict.
    BoundsTree mTree;
    // Item indices the last query came back with, kept so a query allocates
    // nothing.
    mutable std::vector<u32> mCandidates;
    // Parallel to mEntries, rebuilt at build() and handed to the tree.
    std::vector<AABB> mBounds;
    Stats mStats;
};

} // namespace Radion

#endif // RADION_SCENE_BVH_H
