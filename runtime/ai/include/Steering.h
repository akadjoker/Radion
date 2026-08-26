#ifndef RADION_AI_STEERING_H
#define RADION_AI_STEERING_H

// Steering.h - steering behaviors (seek/flee/wander/pursuit/evasion/...) for
// entities.
//
// A SteerLibrary computes steering forces for a bound Entity; the Behavior
// subclasses at the bottom feed each force into the entity's desired-move
// accumulator (scaled by gain).

#include "Behavior.h"
#include "Entity.h"
#include "Obstacle.h"

#include <functional>

namespace Radion::AI
{

class SteerLibrary
{
public:
    SteerLibrary() = default;
    explicit SteerLibrary(const Entity& vehicle) : mVehicle(&vehicle)
    {
    }

    void setVehicle(const Entity& vehicle)
    {
        mVehicle = &vehicle;
    }
    const Entity& vehicle() const
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

    // Pursuit of another entity, with an optional ceiling on prediction time.
    Math::vec3 pursuit(const Entity& quarry) const;
    Math::vec3 pursuit(const Entity& quarry, float maxPredictionTime) const;

    // Evasion of a menace, with a ceiling on prediction time.
    Math::vec3 evasion(const Entity& menace, float maxPredictionTime) const;

    // Flocking on the vehicle's sense lists (Entity::visibleGroupMembers).
    Math::vec3 separation(float maxDistance, float cosMaxAngle,
                         const std::vector<EntityDist>& flock) const;
    Math::vec3 alignment(float maxDistance, float cosMaxAngle,
                        const std::vector<EntityDist>& flock) const;
    Math::vec3 cohesion(float maxDistance, float cosMaxAngle,
                       const std::vector<EntityDist>& flock) const;

    // Obstacle avoidance over an ObstacleGroup.
    Math::vec3 avoidObstacles(float minTimeToCollision, const ObstacleGroup& obstacles) const;

    // Unaligned collision avoidance against a set of nearby entities.
    Math::vec3 avoidNeighbors(float minTimeToCollision, const std::vector<EntityDist>& others);

    // Try to maintain a given speed, clipped to maxForce, along forward.
    Math::vec3 targetSpeed(float targetSpeed) const;

    // --- helpers ------------------------------------------------------------

    // Time until the nearest approach of this vehicle and another.
    float predictNearestApproachTime(const Entity& other) const;

    // Positions of both vehicles at nearest approach; returns the distance
    // between them and fills the annotation fields (mutates state, so not const).
    float computeNearestApproachPositions(const Entity& other, float time);

    // Is another entity within this boid's neighborhood (min/max sphere plus
    // forward-angle cone)?
    bool inBoidNeighborhood(const Entity& other, float minDistance, float maxDistance,
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
                                  const std::vector<EntityDist>& others) const;

    const Entity* mVehicle = nullptr;
};

// --- convenience behaviors (feed a SteerLibrary force into desiredMove) -----

class SeekBehavior final : public Behavior
{
public:
    explicit SeekBehavior(const Math::vec3& target) : mTarget(target)
    {
    }
    void setTarget(const Math::vec3& target)
    {
        mTarget = target;
    }
    const char* name() const override
    {
        return "Seek Behavior";
    }

    void iterate(float timeDelta, Entity& entity) override;

private:
    Math::vec3 mTarget;
    SteerLibrary mSteer;
};

class FleeBehavior final : public Behavior
{
public:
    explicit FleeBehavior(const Math::vec3& threat) : mThreat(threat)
    {
    }
    void setThreat(const Math::vec3& threat)
    {
        mThreat = threat;
    }
    const char* name() const override
    {
        return "Flee Behavior";
    }

    void iterate(float timeDelta, Entity& entity) override;

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
    void iterate(float timeDelta, Entity& entity) override;

private:
    SteerLibrary mSteer; // holds the wander state
};

class ObstacleAvoidanceBehavior final : public Behavior
{
public:
    ObstacleAvoidanceBehavior(float minTimeToCollision, const ObstacleGroup& obstacles);
    void setObstacles(const ObstacleGroup& obstacles)
    {
        mObstacles = &obstacles;
    }
    void setMinTimeToCollision(float time)
    {
        mMinTimeToCollision = time;
    }
    const char* name() const override
    {
        return "Obstacle Avoidance Behavior";
    }

    void iterate(float timeDelta, Entity& entity) override;

private:
    float mMinTimeToCollision;
    const ObstacleGroup* mObstacles; // non-owning
    SteerLibrary mSteer;
};

// Generic behavior that runs a user-supplied steering function each iterate.
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

    void iterate(float timeDelta, Entity& entity) override;

private:
    SteerLibrary mSteer;
    SteerFunc mFunc;
};

} // namespace Radion::AI

#endif // RADION_AI_STEERING_H
