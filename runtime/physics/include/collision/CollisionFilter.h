#ifndef RADION_PHYSICS_COLLISION_FILTER_H
#define RADION_PHYSICS_COLLISION_FILTER_H

#include "Types.h"

namespace Radion::Physics
{

class RigidBody;

// Category bits and acceptance bits are deliberately separate. A pair is
// accepted only when both objects opt in, matching Jolt/Bullet filtering.
struct CollisionFilter
{
    u32 group = 1;
    u32 mask = 0xFFFFFFFFu;
};

inline bool shouldCollide(const CollisionFilter& a, const CollisionFilter& b)
{
    return (a.mask & b.group) != 0 && (b.mask & a.group) != 0;
}

struct QueryFilter
{
    CollisionFilter collision{0xFFFFFFFFu, 0xFFFFFFFFu};
    const RigidBody* ignoredBody = nullptr;

    bool accepts(const RigidBody* body, const CollisionFilter& bodyFilter) const
    {
        return body != ignoredBody && shouldCollide(collision, bodyFilter);
    }
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_COLLISION_FILTER_H
