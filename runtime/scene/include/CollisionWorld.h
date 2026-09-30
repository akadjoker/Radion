#ifndef RADION_COLLISION_WORLD_H
#define RADION_COLLISION_WORLD_H

#include "Math.h"
#include "Types.h"

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

    // epsilon: push-off from a resting contact so the next sweep does not re-report it at t = 0.
    // zeroEpsilon: below this a slide direction counts as not moving.
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
