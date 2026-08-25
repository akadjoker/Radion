#ifndef RADION_PHYSICS_COLLISION_BROADPHASE_H
#define RADION_PHYSICS_COLLISION_BROADPHASE_H

#include "Math.h"
#include "Types.h"
#include "collision/CollisionFilter.h"

#include <vector>

namespace Radion::Physics
{

struct BroadphaseProxy
{
    AABB bounds;
    u32 id = 0;
    CollisionFilter filter;
    // Static against static is never worth a narrowphase call: neither can
    // move, so the answer cannot change.
    bool movable = true;
};

struct BroadphasePair
{
    u32 a = 0;
    u32 b = 0;
};

// Sweep and prune on one axis. Proxies are sorted by their lower bound and
// each is only tested against those that start before its own upper bound -
// which turns the all-pairs quadratic into something close to linear once the
// bodies are spread out, and degrades to the same quadratic when they are all
// stacked in one place, which is the honest worst case.
//
// This exists so the collision pipeline can be built and tested on its own.
// In the engine the Scene already keeps an octree and a BVH over exactly
// these bodies, and it is the Scene that should be answering this question -
// a second spatial structure covering the same objects is the thing the
// design set out to avoid.
class Broadphase
{
public:
    void clear();
    void add(const BroadphaseProxy& proxy);
    void reserve(usize count);

    // Overwrites `out` with every pair whose bounds overlap and whose layers
    // accept each other. Pairs come back with a < b.
    void findPairs(std::vector<BroadphasePair>& out);

    usize proxyCount() const
    {
        return mProxies.size();
    }

    // Which axis the sweep runs along. Picking the one the bodies are most
    // spread over is what keeps the scan short; recomputed by findPairs().
    u32 sweepAxis() const
    {
        return mSweepAxis;
    }

    static bool overlaps(const AABB& a, const AABB& b);

private:
    std::vector<BroadphaseProxy> mProxies;
    std::vector<u32> mOrder;
    u32 mSweepAxis = 0;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_COLLISION_BROADPHASE_H
