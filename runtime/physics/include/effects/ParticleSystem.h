#ifndef RADION_PHYSICS_EFFECTS_PARTICLESYSTEM_H
#define RADION_PHYSICS_EFFECTS_PARTICLESYSTEM_H

// Cheap point/sphere projectiles for VFX (debris, sparks): no inertia tensor, joints or sleeping; integrated in bulk and swept with Scene::raycast().
// A hit callback reports where and with what normal/velocity; no opinion on visuals.

#include "Types.h"
#include "collision/CollisionFilter.h"

#include "Math.h"
#include <vector>

namespace Radion
{
class Scene;
}

namespace Radion::Physics
{

enum class ParticleResponse : u8
{
    Kill,   // removed immediately after the callback runs (sparks, blood mist)
    Bounce, // reflects with its own restitution (debris, gravel)
    Stick   // zero velocity, stays at the hit point until life runs out (blood drops, mud)
};

struct ParticleHit
{
    u32 particleId = 0xFFFFFFFFu;
    u32 userTag = 0;
    Math::vec3 point{0.0f};
    Math::vec3 normal{0.0f, 1.0f, 0.0f};
    Math::vec3 incomingVelocity{0.0f};
};

struct ParticleSpawn
{
    Math::vec3 position{0.0f};
    Math::vec3 velocity{0.0f};
    f32 radius = 0.02f;
    f32 life = 1.0f;
    f32 restitution = 0.3f;
    f32 drag = 0.1f; // fraction of velocity lost per second, independent of gravity
    u32 userTag = 0;
    ParticleResponse response = ParticleResponse::Kill;
};

// Pooled; dead particles are swap-removed at the end of step(), so an id handed to the hit callback is only valid during that step.
class ParticleSystem
{
public:
    using HitCallback = void (*)(const ParticleHit& hit, void* userData);

    explicit ParticleSystem(Radion::Scene& scene);

    void setGravity(const Math::vec3& gravity)
    {
        mGravity = gravity;
    }
    const Math::vec3& gravity() const
    {
        return mGravity;
    }

    void setHitCallback(HitCallback callback, void* userData)
    {
        mHitCallback = callback;
        mHitUserData = userData;
    }

    void setQueryFilter(const QueryFilter& filter)
    {
        mFilter = filter;
    }

    u32 emit(const ParticleSpawn& spawn);

    // Explosion burst: `count` particles from `center`, speed/radius uniform in the ranges, direction uniform over the sphere.
    void explode(const Math::vec3& center, u32 count, f32 speedMin, f32 speedMax, f32 radiusMin,
                f32 radiusMax, f32 life, u32 userTag = 0, ParticleResponse response = ParticleResponse::Bounce);

    // Integrates every live particle, sweeps each against the world and fires the hit callback on impact; dead particles are removed at the end.
    void step(f32 dt);

    void clear()
    {
        mParticles.clear();
    }

    usize count() const
    {
        return mParticles.size();
    }

private:
    struct Particle
    {
        Math::vec3 position{0.0f};
        Math::vec3 velocity{0.0f};
        f32 radius = 0.02f;
        f32 life = 1.0f;
        f32 restitution = 0.3f;
        f32 drag = 0.1f;
        u32 userTag = 0;
        u32 id = 0;
        ParticleResponse response = ParticleResponse::Kill;
        bool settled = false; // Stick response after it has come to rest: skip integration
    };

    Radion::Scene& mScene;
    std::vector<Particle> mParticles;
    Math::vec3 mGravity{0.0f, -9.81f, 0.0f};
    QueryFilter mFilter;
    HitCallback mHitCallback = nullptr;
    void* mHitUserData = nullptr;
    u32 mNextId = 1;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_EFFECTS_PARTICLESYSTEM_H
