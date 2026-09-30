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
    // Static against static never needs a narrowphase call.
    bool movable = true;
};

struct BroadphasePair
{
    u32 a = 0;
    u32 b = 0;
};

// Sweep and prune on one axis: proxies sorted by lower bound; degrades to quadratic when all are stacked in one place.
// Standalone for the collision pipeline; in the engine the Scene's octree/BVH should answer this.
class Broadphase
{
public:
    void clear();
    void add(const BroadphaseProxy& proxy);
    void reserve(usize count);

    // Overwrites `out` with overlapping pairs whose layers accept each other; a < b.
    void findPairs(std::vector<BroadphasePair>& out);

    usize proxyCount() const
    {
        return mProxies.size();
    }

    // Sweep axis: the one the bodies spread over most; recomputed by findPairs().
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
