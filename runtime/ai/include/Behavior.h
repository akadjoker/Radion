#ifndef RADION_AI_BEHAVIOR_H
#define RADION_AI_BEHAVIOR_H

#include "BehaviorFactory.h"
#include "Types.h"

#include "Math.h"

namespace Radion
{
class Agent;
}

namespace Radion::AI
{

// One tunable field, read by the inspector and SceneSerializer.
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

    // Every registered subclass overrides this; SteerBehavior (unregistered) returns Count.
    virtual BehaviorType type() const = 0;

    // Generic parameter access for the editor and serializer; defaults mean "no parameters".
    virtual u32 paramCount() const;
    virtual const BehaviorParam& paramInfo(u32 index) const;
    virtual f32 paramFloat(u32 index) const;
    virtual void setParamFloat(u32 index, f32 value);
    virtual Math::vec3 paramVec3(u32 index) const;
    virtual void setParamVec3(u32 index, const Math::vec3& value);
    virtual bool paramBool(u32 index) const;
    virtual void setParamBool(u32 index, bool value);

    // Owning agent, or null while loose. addBehavior() rejects one already owned: it would be deleted twice.
    Radion::Agent* owner() const
    {
        return mOwner;
    }

private:
    friend class Radion::Agent;

    float mGain = 1.0f;
    Radion::Agent* mOwner = nullptr;
};

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

// Wander: nudge the desired move toward/away from the desired speed plus random per-axis movement.
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

class StayWithinSphereBehavior final : public Behavior
{
public:
    StayWithinSphereBehavior(const Math::vec3& center = Math::vec3(0.0f), float radius = 20.0f);
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
    Math::vec3 paramVec3(u32 index) const override;
    void setParamVec3(u32 index, const Math::vec3& value) override;

    const Math::vec3& sphereCenter() const
    {
        return mCenter;
    }
    void setSphereCenter(const Math::vec3& center)
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
    Math::vec3 mCenter;
    float mRadius;
};

// Purely reactive; closing the distance is PathfindBehavior/FormationBehavior's job.
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
