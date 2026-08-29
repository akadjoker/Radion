#ifndef RADION_AI_BEHAVIOR_H
#define RADION_AI_BEHAVIOR_H

#include "BehaviorFactory.h"
#include "Types.h"

#include <glm/glm.hpp>

namespace Radion
{
class Agent;
}

namespace Radion::AI
{

// Describes one tunable field of a Behavior, for a generic editor slider and
// a generic serializer field - the inspector and SceneSerializer read this
// instead of a hand-written case per behavior.
struct BehaviorParam
{
    enum class Kind : u8
    {
        Float,
        Int,
        Bool,
        Vec3
    };

    const char* name;
    Kind kind;
    f32 minValue;
    f32 maxValue;
    const char* tooltip;
};

class Behavior
{
public:
    Behavior() = default;
    virtual ~Behavior() = default;

    virtual void iterate(float timeDelta, Radion::Agent& entity) = 0;

    float gain() const
    {
        return mGain;
    }
    void setGain(float gain)
    {
        mGain = gain;
    }

    virtual const char* name() const
    {
        return "Base Behavior";
    }

    // Identifies the concrete class for BehaviorFactory - every registered
    // subclass overrides this; SteerBehavior (not registered) is the one
    // exception, returning BehaviorType::Count.
    virtual BehaviorType type() const = 0;

    // Generic parameter access for the editor and the serializer, backed by
    // each subclass's own static BehaviorParam table and a switch on index -
    // no reflection, no std::function. Defaults below say "no parameters";
    // a subclass overrides only the accessor families its own params use.
    virtual u32 paramCount() const;
    virtual const BehaviorParam& paramInfo(u32 index) const;
    virtual f32 paramFloat(u32 index) const;
    virtual void setParamFloat(u32 index, f32 value);
    virtual glm::vec3 paramVec3(u32 index) const;
    virtual void setParamVec3(u32 index, const glm::vec3& value);
    virtual bool paramBool(u32 index) const;
    virtual void setParamBool(u32 index, bool value);

    // The agent that owns and will delete this behavior, or null while it is
    // still loose. Agent::addBehavior() rejects one that already has an
    // owner - the same guard Group::add() kept against an entity landing in
    // two groups, and it matters more here: a behavior handed to two agents
    // is deleted twice.
    Radion::Agent* owner() const
    {
        return mOwner;
    }

private:
    friend class Radion::Agent;

    float mGain = 1.0f;
    Radion::Agent* mOwner = nullptr;
};

// Push away from the closest visible group member when inside the separation
// distance; pull toward it when it drifts beyond it.
class SeparationBehavior final : public Behavior
{
public:
    SeparationBehavior(float separationDistance = 4.0f, float minPercent = 0.2f,
                       float maxPercent = 1.0f);
    void iterate(float timeDelta, Radion::Agent& entity) override;
    const char* name() const override
    {
        return "Separation Behavior";
    }
    BehaviorType type() const override
    {
        return BehaviorType::Separation;
    }
    u32 paramCount() const override;
    const BehaviorParam& paramInfo(u32 index) const override;
    f32 paramFloat(u32 index) const override;
    void setParamFloat(u32 index, f32 value) override;

    float separationDistance() const
    {
        return mSeparationDistance;
    }
    void setSeparationDistance(float dist)
    {
        mSeparationDistance = dist;
    }
    float minSeparationPercentage() const
    {
        return mMinSeparationPercentage;
    }
    void setMinSeparationPercentage(float pct)
    {
        mMinSeparationPercentage = pct;
    }
    float maxSeparationPercentage() const
    {
        return mMaxSeparationPercentage;
    }
    void setMaxSeparationPercentage(float pct)
    {
        mMaxSeparationPercentage = pct;
    }

private:
    float mSeparationDistance;
    float mMinSeparationPercentage;
    float mMaxSeparationPercentage;
};

// Match the heading of the closest visible group member.
class AlignmentBehavior final : public Behavior
{
public:
    explicit AlignmentBehavior(float turnRate = 1.0f);
    void iterate(float timeDelta, Radion::Agent& entity) override;
    const char* name() const override
    {
        return "Alignment Behavior";
    }
    BehaviorType type() const override
    {
        return BehaviorType::Alignment;
    }
    u32 paramCount() const override;
    const BehaviorParam& paramInfo(u32 index) const override;
    f32 paramFloat(u32 index) const override;
    void setParamFloat(u32 index, f32 value) override;

    float turnRate() const
    {
        return mTurnRate;
    }
    void setTurnRate(float rate)
    {
        mTurnRate = rate;
    }

private:
    float mTurnRate;
};

// Steer toward the centre of mass of the visible group members.
class CohesionBehavior final : public Behavior
{
public:
    explicit CohesionBehavior(float turnRate = 1.0f);
    void iterate(float timeDelta, Radion::Agent& entity) override;
    const char* name() const override
    {
        return "Cohesion Behavior";
    }
    BehaviorType type() const override
    {
        return BehaviorType::Cohesion;
    }
    u32 paramCount() const override;
    const BehaviorParam& paramInfo(u32 index) const override;
    f32 paramFloat(u32 index) const override;
    void setParamFloat(u32 index, f32 value) override;

    float turnRate() const
    {
        return mTurnRate;
    }
    void setTurnRate(float rate)
    {
        mTurnRate = rate;
    }

private:
    float mTurnRate;
};

// Flee from the closest visible enemy inside the avoidance distance.
class AvoidanceBehavior final : public Behavior
{
public:
    AvoidanceBehavior(float avoidanceDistance = 4.0f, float avoidanceSpeed = 4.0f);
    void iterate(float timeDelta, Radion::Agent& entity) override;
    const char* name() const override
    {
        return "Avoidance Behavior";
    }
    BehaviorType type() const override
    {
        return BehaviorType::Avoidance;
    }
    u32 paramCount() const override;
    const BehaviorParam& paramInfo(u32 index) const override;
    f32 paramFloat(u32 index) const override;
    void setParamFloat(u32 index, f32 value) override;

    float avoidanceDistance() const
    {
        return mAvoidanceDistance;
    }
    void setAvoidanceDistance(float dist)
    {
        mAvoidanceDistance = dist;
    }
    float avoidanceSpeed() const
    {
        return mAvoidanceSpeed;
    }
    void setAvoidanceSpeed(float speed)
    {
        mAvoidanceSpeed = speed;
    }

private:
    float mAvoidanceDistance;
    float mAvoidanceSpeed;
};

// Wander: nudge the desired move toward/away from the desired speed and throw
// in occasional random per-axis movement.
class CruisingBehavior final : public Behavior
{
public:
    CruisingBehavior(float randMoveXChance = 0.1f, float randMoveYChance = 0.0f,
                     float randMoveZChance = 0.1f, float minRandomMove = 0.5f,
                     float maxRateChange = 0.3f, float minRateChange = 0.05f);
    void iterate(float timeDelta, Radion::Agent& entity) override;
    const char* name() const override
    {
        return "Cruising Behavior";
    }
    BehaviorType type() const override
    {
        return BehaviorType::Cruising;
    }
    u32 paramCount() const override;
    const BehaviorParam& paramInfo(u32 index) const override;
    f32 paramFloat(u32 index) const override;
    void setParamFloat(u32 index, f32 value) override;

    float randMoveXChance() const
    {
        return mRandMoveXChance;
    }
    void setRandMoveXChance(float chance)
    {
        mRandMoveXChance = chance;
    }
    float randMoveYChance() const
    {
        return mRandMoveYChance;
    }
    void setRandMoveYChance(float chance)
    {
        mRandMoveYChance = chance;
    }
    float randMoveZChance() const
    {
        return mRandMoveZChance;
    }
    void setRandMoveZChance(float chance)
    {
        mRandMoveZChance = chance;
    }
    float minRandomMove() const
    {
        return mMinRandomMove;
    }
    void setMinRandomMove(float move)
    {
        mMinRandomMove = move;
    }
    float maxRateChange() const
    {
        return mMaxRateChange;
    }
    void setMaxRateChange(float rateChange)
    {
        mMaxRateChange = rateChange;
    }
    float minRateChange() const
    {
        return mMinRateChange;
    }
    void setMinRateChange(float rateChange)
    {
        mMinRateChange = rateChange;
    }

private:
    float mRandMoveXChance;
    float mRandMoveYChance;
    float mRandMoveZChance;
    float mMinRandomMove;
    float mMaxRateChange;
    float mMinRateChange;
};

// Push back toward the sphere centre once the entity leaves the sphere.
class StayWithinSphereBehavior final : public Behavior
{
public:
    StayWithinSphereBehavior(const glm::vec3& center = glm::vec3(0.0f), float radius = 20.0f);
    void iterate(float timeDelta, Radion::Agent& entity) override;
    const char* name() const override
    {
        return "Stay Within Sphere Behavior";
    }
    BehaviorType type() const override
    {
        return BehaviorType::StayWithinSphere;
    }
    u32 paramCount() const override;
    const BehaviorParam& paramInfo(u32 index) const override;
    f32 paramFloat(u32 index) const override;
    void setParamFloat(u32 index, f32 value) override;
    glm::vec3 paramVec3(u32 index) const override;
    void setParamVec3(u32 index, const glm::vec3& value) override;

    const glm::vec3& sphereCenter() const
    {
        return mCenter;
    }
    void setSphereCenter(const glm::vec3& center)
    {
        mCenter = center;
    }
    float sphereRadius() const
    {
        return mRadius;
    }
    void setSphereRadius(float radius)
    {
        mRadius = radius;
    }

private:
    glm::vec3 mCenter;
    float mRadius;
};

// Deals damage to the nearest visible enemy once it is within fire range
// and the entity's own attack cooldown (Agent::attackCooldown(), ticked
// here every frame regardless of range) has elapsed. Purely reactive - it
// does not move the entity or pick a target beyond "nearest visible enemy";
// closing the distance is PathfindBehavior/FormationBehavior's job, same
// division as every other Behavior in this file.
class CombatBehavior final : public Behavior
{
public:
    CombatBehavior(float fireRange = 10.0f, float damagePerHit = 10.0f, float fireInterval = 1.0f);
    void iterate(float timeDelta, Radion::Agent& entity) override;
    const char* name() const override
    {
        return "Combat Behavior";
    }
    BehaviorType type() const override
    {
        return BehaviorType::Combat;
    }
    u32 paramCount() const override;
    const BehaviorParam& paramInfo(u32 index) const override;
    f32 paramFloat(u32 index) const override;
    void setParamFloat(u32 index, f32 value) override;

    float fireRange() const
    {
        return mFireRange;
    }
    void setFireRange(float range)
    {
        mFireRange = range;
    }
    float damagePerHit() const
    {
        return mDamagePerHit;
    }
    void setDamagePerHit(float damage)
    {
        mDamagePerHit = damage;
    }
    float fireInterval() const
    {
        return mFireInterval;
    }
    void setFireInterval(float interval)
    {
        mFireInterval = interval;
    }

private:
    float mFireRange;
    float mDamagePerHit;
    float mFireInterval;
};

} // namespace Radion::AI

#endif // RADION_AI_BEHAVIOR_H
