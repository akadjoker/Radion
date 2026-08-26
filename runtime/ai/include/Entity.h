#ifndef RADION_AI_ENTITY_H
#define RADION_AI_ENTITY_H

#include "Types.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

namespace Radion::AI
{

class Behavior;
class Entity;
class Group;
class World;

using EntityType = Radion::u32;

struct EntityDist
{
    float distance = 0.0f;
    Entity* entity = nullptr;
};

class Entity
{
public:
    struct Settings
    {
        EntityType type = 0;
        float senseRange = 4.0f;        // radius in which others are "sensed"
        float maxVelocityChange = 1.0f; // max acceleration per iterate()
        float maxSpeed = 5.0f;
        float desiredSpeed = 2.0f;
        float radius = 0.5f;      // bounding-sphere radius
        float moveXScalar = 1.0f; // per-axis velocity scalars (0 = locked)
        float moveYScalar = 0.0f;
        float moveZScalar = 1.0f;
    };

    Entity(World& world, const Settings& settings);
    virtual ~Entity() = default;

    virtual void iterate(float timeDelta);

    void setFriendMask(EntityType mask)
    {
        mFriendMask = mask;
        mEnemyMask = ~mask;
    }
    EntityType friendMask() const
    {
        return mFriendMask;
    }
    EntityType enemyMask() const
    {
        return mEnemyMask;
    }
    EntityType type() const
    {
        return mEntityType;
    }

    // Sense data refreshed each iterate(); sorted ascending by distance.
    // Both lists only ever hold living entities - a dead one stops being
    // sensed by anyone the same frame its health reaches zero.
    const std::vector<EntityDist>& visibleGroupMembers() const
    {
        return mVisibleGroupMembers;
    }
    const std::vector<EntityDist>& visibleEnemies() const
    {
        return mVisibleEnemies;
    }

    // Combat: plain hit points, no regen. iterate() freezes a dead entity in
    // place (skips sensing, behaviors and movement) rather than removing it
    // from its Group, so a caller can still find it, play a death pose on
    // it, or hide its GameObject - Group ownership/cleanup is unaffected.
    float health() const
    {
        return mHealth;
    }
    void setHealth(float health)
    {
        mHealth = glm::max(health, 0.0f);
    }
    bool alive() const
    {
        return mHealth > 0.0f;
    }
    void applyDamage(float amount)
    {
        mHealth = glm::max(mHealth - amount, 0.0f);
    }

    // CombatBehavior's own per-entity cooldown between shots - lives here
    // rather than inside the (shared, stateless-by-convention) Behavior
    // instance, the same way mDesiredMoveVector is per-entity steering state
    // rather than something the steering Behaviors hold themselves.
    float attackCooldown() const
    {
        return mAttackCooldown;
    }
    void setAttackCooldown(float seconds)
    {
        mAttackCooldown = glm::max(seconds, 0.0f);
    }
    void tickAttackCooldown(float timeDelta)
    {
        mAttackCooldown = glm::max(mAttackCooldown - timeDelta, 0.0f);
    }

    // One shot's worth of event state, valid for the frame CombatBehavior
    // actually fires and cleared at the top of the next iterate() - a
    // caller polls this once per frame (after World::iterate()) to trigger
    // a muzzle flash / tracer / fire animation without CombatBehavior
    // needing to know anything about rendering.
    bool firedThisFrame() const
    {
        return mFiredThisFrame;
    }
    Entity* lastFireTarget() const
    {
        return mLastFireTarget;
    }
    void markFired(Entity* target)
    {
        mFiredThisFrame = true;
        mLastFireTarget = target;
    }

    // Behaviors are non-owning; do not delete them here.
    void addBehavior(Behavior& behavior);
    void removeBehavior(Behavior& behavior);

    void setCurrentGroup(Group* group)
    {
        mCurrentGroup = group;
    }
    Group* currentGroup() const
    {
        return mCurrentGroup;
    }

    const Math::Vec3& position() const
    {
        return mPosition;
    }
    void setPosition(const Math::Vec3& position)
    {
        mPosition = position;
    }
    const Math::Vec3& velocity() const
    {
        return mVelocity;
    }
    void setVelocity(const Math::Vec3& velocity)
    {
        mVelocity = velocity;
    }
    const Math::Quaternion& orientation() const
    {
        return mOrientation;
    }
    void setOrientation(const Math::Quaternion& orientation)
    {
        mOrientation = orientation;
    }
    const Math::Vec3& desiredMove() const
    {
        return mDesiredMoveVector;
    }
    void setDesiredMove(const Math::Vec3& move)
    {
        mDesiredMoveVector = move;
    }

    float maxSpeed() const
    {
        return mMaxSpeed;
    }
    // Read every iterate() (Behavior.cpp's own speed-matching and flee
    // terms), never cached - safe to change on a live entity, not just at
    // construction through Settings.
    void setMaxSpeed(float speed)
    {
        mMaxSpeed = glm::max(speed, 0.0f);
    }
    float desiredSpeed() const
    {
        return mDesiredSpeed;
    }
    void setDesiredSpeed(float speed)
    {
        mDesiredSpeed = glm::max(speed, 0.0f);
    }
    float senseRange() const
    {
        return mSenseRange;
    }

    // ---- vehicle interface --------------------------------------------------
    // Local frame convention (consistent with FormationBehavior): right = +X,
    // up = +Y, forward (look) = +Z, all read from the orientation quaternion.
    // This is the reference demos' convention (D3DX-era: forward = +Z), kept
    // as-is because the AI's own math is internally self-consistent and
    // tested against it.
    //
    // It does NOT match runtime/scene: GameObject::forward() is
    // rotation * (0,0,-1) - forward = -Z (right() and up() do agree: +X, +Y).
    // Copying mOrientation straight onto a GameObject's rotation therefore
    // faces it 180 degrees off. examples/ai_squad works around this by never
    // doing that copy - it drives visual facing from velocity instead
    // (rotationBetween(soldierNose, vel) in main.cpp). Any future code that
    // wires an Entity's orientation() directly to a GameObject must apply a
    // 180 degree turn around up() first.

    Math::Vec3 forward() const
    {
        return glm::mat3_cast(mOrientation)[2];
    }
    Math::Vec3 side() const
    {
        return glm::mat3_cast(mOrientation)[0];
    }
    Math::Vec3 up() const
    {
        return glm::mat3_cast(mOrientation)[1];
    }

    // Velocity is a free vector; speed() is its magnitude and setSpeed()
    // rescales it.
    float speed() const
    {
        return glm::length(mVelocity);
    }
    void setSpeed(float newSpeed);

    float radius() const
    {
        return mRadius;
    }
    void setRadius(float radius)
    {
        mRadius = radius;
    }

    // maxForce() is the steering-force limit; it maps to the acceleration
    // limit maxVelocityChange that iterate() clamps against.
    float maxForce() const
    {
        return mMaxVelocityChange;
    }
    void setMaxForce(float force)
    {
        mMaxVelocityChange = force;
    }

    // Predicted position in `predictionTime` seconds (straight-line
    // extrapolation).
    Math::Vec3 predictFuturePosition(float predictionTime) const
    {
        return mPosition + (mVelocity * predictionTime);
    }

    // Transform helpers in the vehicle's local frame.
    Math::Vec3 localizeDirection(const Math::Vec3& globalDirection) const;
    Math::Vec3 localizePosition(const Math::Vec3& globalPosition) const;
    Math::Vec3 globalizePosition(const Math::Vec3& localPosition) const;
    Math::Vec3 globalizeDirection(const Math::Vec3& localDirection) const;

    // Rotate the orientation so forward() points along the current velocity,
    // keeping up as close as possible.
    void alignWithVelocity();

    World& world() const
    {
        return mWorld;
    }

protected:
    void updateGroupVisibility();
    void updateEnemyVisibility();
    bool visibilityTest(const Entity& other, float& dist) const;

    Group* mCurrentGroup = nullptr;
    std::vector<Behavior*> mBehaviors; // non-owning
    World& mWorld;
    EntityType mFriendMask = 0;
    EntityType mEnemyMask = ~EntityType(0);
    EntityType mEntityType = 0;

    Math::Vec3 mPosition = Math::Vec3(0.0f);
    Math::Vec3 mVelocity = Math::Vec3(0.0f);
    Math::Quaternion mOrientation = Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    Math::Vec3 mDesiredMoveVector = Math::Vec3(0.0f);

    float mSenseRange = 4.0f;
    float mMaxVelocityChange = 1.0f;
    float mMaxSpeed = 5.0f;
    float mDesiredSpeed = 2.0f;
    float mRadius = 0.5f;
    float mMoveXScalar = 1.0f;
    float mMoveYScalar = 0.0f;
    float mMoveZScalar = 1.0f;

    std::vector<EntityDist> mVisibleGroupMembers; // sorted by distance
    std::vector<EntityDist> mVisibleEnemies;      // sorted by distance

    float mHealth = 100.0f;
    float mAttackCooldown = 0.0f;
    bool mFiredThisFrame = false;
    Entity* mLastFireTarget = nullptr;
};

} // namespace Radion::AI

#endif // RADION_AI_ENTITY_H
