#ifndef RADION_AI_BEHAVIOR_H
#define RADION_AI_BEHAVIOR_H

#include <glm/glm.hpp>

namespace Radion::AI
{

class Entity;

class Behavior
{
public:
    Behavior() = default;
    virtual ~Behavior() = default;

    virtual void iterate(float timeDelta, Entity& entity) = 0;

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

private:
    float mGain = 1.0f;
};

// Push away from the closest visible group member when inside the separation
// distance; pull toward it when it drifts beyond it.
class SeparationBehavior final : public Behavior
{
public:
    SeparationBehavior(float separationDistance, float minPercent, float maxPercent);
    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Separation Behavior";
    }

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
    explicit AlignmentBehavior(float turnRate);
    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Alignment Behavior";
    }

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
    explicit CohesionBehavior(float turnRate);
    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Cohesion Behavior";
    }

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
    AvoidanceBehavior(float avoidanceDistance, float avoidanceSpeed);
    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Avoidance Behavior";
    }

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
    CruisingBehavior(float randMoveXChance, float randMoveYChance, float randMoveZChance,
                     float minRandomMove, float maxRateChange, float minRateChange);
    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Cruising Behavior";
    }

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
    StayWithinSphereBehavior(const glm::vec3& center, float radius);
    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Stay Within Sphere Behavior";
    }

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
// and the entity's own attack cooldown (Entity::attackCooldown(), ticked
// here every frame regardless of range) has elapsed. Purely reactive - it
// does not move the entity or pick a target beyond "nearest visible enemy";
// closing the distance is PathfindBehavior/FormationBehavior's job, same
// division as every other Behavior in this file.
class CombatBehavior final : public Behavior
{
public:
    CombatBehavior(float fireRange, float damagePerHit, float fireInterval);
    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Combat Behavior";
    }

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
