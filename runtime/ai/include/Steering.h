#ifndef RADION_AI_STEERING_H
#define RADION_AI_STEERING_H

// Steering.h - steering behaviors (seek/flee/wander/pursuit/evasion/...) for
// agents.
//
// A SteerLibrary computes steering forces for a bound Agent; the Behavior
// subclasses at the bottom feed each force into the agent's desired-move
// accumulator (scaled by gain).

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

    // --- behaviors ----------------------------------------------------------

    // Steer toward a target (desired velocity - current velocity).
    Math::vec3 seek(const Math::vec3& target) const;

    // Steer away from a threat.
    Math::vec3 flee(const Math::vec3& target) const;

    // Random lateral wander (stateful; uses dt for the random-walk step).
    Math::vec3 wander(float dt);

    // Pursuit of another agent, with an optional ceiling on prediction time.
    Math::vec3 pursuit(const Radion::Agent& quarry) const;
    Math::vec3 pursuit(const Radion::Agent& quarry, float maxPredictionTime) const;

    // Evasion of a menace, with a ceiling on prediction time.
    Math::vec3 evasion(const Radion::Agent& menace, float maxPredictionTime) const;

    // Flocking on the vehicle's sense lists (Agent::visibleGroupMembers).
    Math::vec3 separation(float maxDistance, float cosMaxAngle,
                         const std::vector<Radion::EntityDist>& flock) const;
    Math::vec3 alignment(float maxDistance, float cosMaxAngle,
                        const std::vector<Radion::EntityDist>& flock) const;
    Math::vec3 cohesion(float maxDistance, float cosMaxAngle,
                       const std::vector<Radion::EntityDist>& flock) const;

    // Obstacle avoidance over an ObstacleGroup.
    Math::vec3 avoidObstacles(float minTimeToCollision, const ObstacleGroup& obstacles) const;

    // Unaligned collision avoidance against a set of nearby agents.
    Math::vec3 avoidNeighbors(float minTimeToCollision,
                             const std::vector<Radion::EntityDist>& others);

    // Try to maintain a given speed, clipped to maxForce, along forward.
    Math::vec3 targetSpeed(float targetSpeed) const;

    // --- helpers ------------------------------------------------------------

    // Time until the nearest approach of this vehicle and another.
    float predictNearestApproachTime(const Radion::Agent& other) const;

    // Positions of both vehicles at nearest approach; returns the distance
    // between them and fills the annotation fields (mutates state, so not const).
    float computeNearestApproachPositions(const Radion::Agent& other, float time);

    // Is another agent within this boid's neighborhood (min/max sphere plus
    // forward-angle cone)?
    bool inBoidNeighborhood(const Radion::Agent& other, float minDistance, float maxDistance,
                            float cosMaxAngle) const;

    bool isAhead(const Math::vec3& target, float cosThreshold = 0.707f) const;
    bool isAside(const Math::vec3& target, float cosThreshold = 0.707f) const;
    bool isBehind(const Math::vec3& target, float cosThreshold = -0.707f) const;

    // Wander state.
    float wanderSide = 0.0f;
    float wanderUp = 0.0f;

    // Nearest-approach positions, filled by computeNearestApproachPositions.
    Math::vec3 hisPositionAtNearestApproach = Math::vec3(0.0f);
    Math::vec3 ourPositionAtNearestApproach = Math::vec3(0.0f);

private:
    Math::vec3 avoidCloseNeighbors(float minSeparationDistance,
                                  const std::vector<Radion::EntityDist>& others) const;

    const Radion::Agent* mVehicle = nullptr;
};

// --- convenience behaviors (feed a SteerLibrary force into desiredMove) -----

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
    // Explicit override of the default source (Agent::scene()->obstacleGroup()) -
    // for a caller that wants this instance to avoid a group of its own.
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

// Generic behavior that runs a user-supplied steering function each iterate.
// Not registered in BehaviorFactory (see BehaviorFactory.h): a std::function
// supplied from C++ has no by-name meaning for an editor combo or a save
// file, and calling through it every agent every frame is exactly the
// per-frame lambda cost the rest of this file avoids.
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
