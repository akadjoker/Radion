#ifndef RADION_COLLISION_WORLD_H
#define RADION_COLLISION_WORLD_H

#include "Math.h"
#include "Types.h"

#include "Math.h"
#include <vector>

namespace Radion
{

class Scene;
class Collider;

enum class CollisionResponse : u8
{
    None,
    Stop,
    Slide,
    SlideXZ
};

// enable(A, B) also answers for (B, A); a pair never enabled never collides.
class CollisionWorld
{
public:
    struct MoveResult
    {
        Math::vec3 position{0.0f};
        u32 hitCount = 0;
        bool collided = false;
        Math::vec3 lastNormal{0.0f, 1.0f, 0.0f};
    };

    // Epsilon: how far a resting contact is pushed off the surface it just
    // hit, so the next substep's sweep does not immediately re-report the
    // same contact at t = 0. zeroEpsilon: how small a remaining slide/crease
    // direction has to be before it counts as "not actually moving" and the
    // loop stops instead of dividing by it.
    struct MoveConfig
    {
        f32 epsilon = 0.001f;
        f32 zeroEpsilon = Epsilon;
    };

    CollisionWorld();

    void initialize(Scene& scene);

    void enable(u32 typeA, u32 typeB, CollisionResponse response = CollisionResponse::None);
    void disable(u32 typeA, u32 typeB);
    bool enabled(u32 typeA, u32 typeB) const;
    CollisionResponse response(u32 typeA, u32 typeB) const;

    void step();

    MoveConfig& moveConfig();
    const MoveConfig& moveConfig() const;

    // Resolves a moving sphere from `from` towards `to` against every
    // collider whose type pairs with `movingType`, sliding/stopping per the
    // pair's own response. The mover itself carries no Collider - a purely
    // geometric query, same as the character controller's ellipsoid sweep.
    MoveResult moveSphere(const Math::vec3& from, const Math::vec3& to, f32 radius,
                          u32 movingType, u32 maxHits = 10) const;

private:
    struct Pair
    {
        u32 typeA = 0;
        u32 typeB = 0;
        CollisionResponse response = CollisionResponse::None;
    };

    s32 findPair(u32 typeA, u32 typeB) const;

    static void collideMeshPair(Collider& mesh, Collider& other);
    static void collidePair(Collider& a, Collider& b);
    static void notifyCollision(Collider& self, Collider& other);

    Scene* mScene = nullptr;
    std::vector<Pair> mPairs;
    std::vector<Collider*> mStepColliders;
    std::vector<AABB> mStepBounds;
    MoveConfig mMoveConfig;
};

} // namespace Radion

#endif // RADION_COLLISION_WORLD_H
