#ifndef RADION_AI_STEERING_H
#define RADION_AI_STEERING_H

#include "Behavior.h"
#include "Obstacle.h"

#include <functional>
#include <vector>

namespace Radion
{
class Agent;
struct EntityDist;
} // namespace Radion

namespace Radion::AI
{

class SteerLibrary
{
public:
    SteerLibrary() = default;
    explicit SteerLibrary(const Radion::Agent& vehicle) : mVehicle(&vehicle)
    {
    }

    void setVehicle(const Radion::Agent& vehicle)
    {
        mVehicle = &vehicle;
    }
    const Radion::Agent& vehicle() const
    {
        return *mVehicle;
    }

    Math::vec3 seek(const Math::vec3& target) const;

    Math::vec3 flee(const Math::vec3& target) const;

    Math::vec3 wander(float dt);

    Math::vec3 pursuit(const Radion::Agent& quarry) const;
    Math::vec3 pursuit(const Radion::Agent& quarry, float maxPredictionTime) const;

    Math::vec3 evasion(const Radion::Agent& menace, float maxPredictionTime) const;

    Math::vec3 separation(float maxDistance, float cosMaxAngle,
                         const std::vector<Radion::EntityDist>& flock) const;
    Math::vec3 alignment(float maxDistance, float cosMaxAngle,
                        const std::vector<Radion::EntityDist>& flock) const;
    Math::vec3 cohesion(float maxDistance, float cosMaxAngle,
                       const std::vector<Radion::EntityDist>& flock) const;

    Math::vec3 avoidObstacles(float minTimeToCollision, const ObstacleGroup& obstacles) const;

    Math::vec3 avoidNeighbors(float minTimeToCollision,
                             const std::vector<Radion::EntityDist>& others);

    Math::vec3 targetSpeed(float targetSpeed) const;

    float predictNearestApproachTime(const Radion::Agent& other) const;

    // Fills the annotation fields, so not const.
    float computeNearestApproachPositions(const Radion::Agent& other, float time);

    bool inBoidNeighborhood(const Radion::Agent& other, float minDistance, float maxDistance,
                            float cosMaxAngle) const;

    bool isAhead(const Math::vec3& target, float cosThreshold = 0.707f) const;
    bool isAside(const Math::vec3& target, float cosThreshold = 0.707f) const;
    bool isBehind(const Math::vec3& target, float cosThreshold = -0.707f) const;

    float wanderSide = 0.0f;
    float wanderUp = 0.0f;

    Math::vec3 hisPositionAtNearestApproach = Math::vec3(0.0f);
    Math::vec3 ourPositionAtNearestApproach = Math::vec3(0.0f);

private:
    Math::vec3 avoidCloseNeighbors(float minSeparationDistance,
                                  const std::vector<Radion::EntityDist>& others) const;

    const Radion::Agent* mVehicle = nullptr;
};

class SeekBehavior final : public Behavior
{
public:
    explicit SeekBehavior(const Math::vec3& target = Math::vec3(0.0f)) : mTarget(target)
    {
    }
    void setTarget(const Math::vec3& target)
    {
        mTarget = target;
    }
    const Math::vec3& target() const
    {
        return mTarget;
    }
    const char* name() const override
    {
        return "Seek Behavior";
    }
    BehaviorType type() const override
    {
        return BehaviorType::Seek;
    }
    u32 paramCount() const override;
    const BehaviorParam& paramInfo(u32 index) const override;
    Math::vec3 paramVec3(u32 index) const override;
    void setParamVec3(u32 index, const Math::vec3& value) override;

    void iterate(float timeDelta, Radion::Agent& entity) override;

private:
    Math::vec3 mTarget;
    SteerLibrary mSteer;
};

class FleeBehavior final : public Behavior
{
public:
    explicit FleeBehavior(const Math::vec3& threat = Math::vec3(0.0f)) : mThreat(threat)
    {
    }
    void setThreat(const Math::vec3& threat)
    {
        mThreat = threat;
    }
    const Math::vec3& threat() const
    {
        return mThreat;
    }
    const char* name() const override
    {
        return "Flee Behavior";
    }
    BehaviorType type() const override
    {
        return BehaviorType::Flee;
    }
    u32 paramCount() const override;
    const BehaviorParam& paramInfo(u32 index) const override;
    Math::vec3 paramVec3(u32 index) const override;
    void setParamVec3(u32 index, const Math::vec3& value) override;

    void iterate(float timeDelta, Radion::Agent& entity) override;

private:
    Math::vec3 mThreat;
    SteerLibrary mSteer;
};

class WanderBehavior final : public Behavior
{
public:
    const char* name() const override
    {
        return "Wander Behavior";
    }
    BehaviorType type() const override
    {
        return BehaviorType::Wander;
    }
    void iterate(float timeDelta, Radion::Agent& entity) override;

private:
    SteerLibrary mSteer; // holds the wander state
};

class ObstacleAvoidanceBehavior final : public Behavior
{
public:
    explicit ObstacleAvoidanceBehavior(float minTimeToCollision = 2.0f);
    void setObstacles(const ObstacleGroup& obstacles)
    {
        mObstacles = &obstacles;
    }
    void setMinTimeToCollision(float time)
    {
        mMinTimeToCollision = time;
    }
    float minTimeToCollision() const
    {
        return mMinTimeToCollision;
    }
    const char* name() const override
    {
        return "Obstacle Avoidance Behavior";
    }
    BehaviorType type() const override
    {
        return BehaviorType::ObstacleAvoidance;
    }
    u32 paramCount() const override;
    const BehaviorParam& paramInfo(u32 index) const override;
    f32 paramFloat(u32 index) const override;
    void setParamFloat(u32 index, f32 value) override;

    void iterate(float timeDelta, Radion::Agent& entity) override;

private:
    float mMinTimeToCollision;
    const ObstacleGroup* mObstacles = nullptr; // non-owning; null reads the Scene's group
    SteerLibrary mSteer;
};

// Not registered in BehaviorFactory: a std::function has no by-name meaning, and calling through it
// per agent per frame is the lambda cost the rest of this file avoids.
class SteerBehavior final : public Behavior
{
public:
    using SteerFunc = std::function<Math::vec3(SteerLibrary&, float)>;
    explicit SteerBehavior(SteerFunc func) : mFunc(std::move(func))
    {
    }
    const char* name() const override
    {
        return "Steer Behavior";
    }
    BehaviorType type() const override
    {
        return BehaviorType::Count;
    }

    void iterate(float timeDelta, Radion::Agent& entity) override;

private:
    SteerLibrary mSteer;
    SteerFunc mFunc;
};

} // namespace Radion::AI

#endif // RADION_AI_STEERING_H
