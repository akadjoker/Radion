#include "PCH.h"

#include "Steering.h"

#include "AIInternal.h"
#include "Agent.h"
#include "Scene.h"

#include <cfloat>

namespace Radion::AI
{

using detail::clip;
using detail::intervalComparison;
using detail::perpendicularComponent;
using detail::safeNormalize;
using detail::scalarRandomWalk;
using Radion::Agent;
using Radion::EntityDist;
using Radion::Scene;

namespace
{
const BehaviorParam kSeekParams[] = {
    {"Target", BehaviorParam::Kind::Vec3, 0.0f, 0.0f,
     "World-space point this agent steers toward."},
};
const BehaviorParam kFleeParams[] = {
    {"Threat", BehaviorParam::Kind::Vec3, 0.0f, 0.0f,
     "World-space point this agent steers away from."},
};
const BehaviorParam kObstacleAvoidanceParams[] = {
    {"Min Time To Collision", BehaviorParam::Kind::Float, 0.0f, 10.0f,
     "How far ahead, in seconds of travel at the agent's current speed, an obstacle is worth "
     "steering around."},
};
} // namespace

u32 SeekBehavior::paramCount() const
{
    return static_cast<u32>(sizeof(kSeekParams) / sizeof(kSeekParams[0]));
}

const BehaviorParam& SeekBehavior::paramInfo(u32 index) const
{
    return kSeekParams[index];
}

Math::vec3 SeekBehavior::paramVec3(u32 index) const
{
    (void)index;
    return mTarget;
}

void SeekBehavior::setParamVec3(u32 index, const Math::vec3& value)
{
    (void)index;
    mTarget = value;
}

u32 FleeBehavior::paramCount() const
{
    return static_cast<u32>(sizeof(kFleeParams) / sizeof(kFleeParams[0]));
}

const BehaviorParam& FleeBehavior::paramInfo(u32 index) const
{
    return kFleeParams[index];
}

Math::vec3 FleeBehavior::paramVec3(u32 index) const
{
    (void)index;
    return mThreat;
}

void FleeBehavior::setParamVec3(u32 index, const Math::vec3& value)
{
    (void)index;
    mThreat = value;
}

u32 ObstacleAvoidanceBehavior::paramCount() const
{
    return static_cast<u32>(sizeof(kObstacleAvoidanceParams) / sizeof(kObstacleAvoidanceParams[0]));
}

const BehaviorParam& ObstacleAvoidanceBehavior::paramInfo(u32 index) const
{
    return kObstacleAvoidanceParams[index];
}

f32 ObstacleAvoidanceBehavior::paramFloat(u32 index) const
{
    (void)index;
    return mMinTimeToCollision;
}

void ObstacleAvoidanceBehavior::setParamFloat(u32 index, f32 value)
{
    (void)index;
    mMinTimeToCollision = value;
}

Math::vec3 SteerLibrary::seek(const Math::vec3& target) const
{
    const Math::vec3 desiredVelocity = target - vehicle().position();
    return desiredVelocity - vehicle().velocity();
}

Math::vec3 SteerLibrary::flee(const Math::vec3& target) const
{
    const Math::vec3 desiredVelocity = vehicle().position() - target;
    return desiredVelocity - vehicle().velocity();
}

Math::vec3 SteerLibrary::wander(float dt)
{
    const float speed = 12.0f * dt;
    wanderSide = scalarRandomWalk(wanderSide, speed, -1.0f, +1.0f);
    wanderUp = scalarRandomWalk(wanderUp, speed, -1.0f, +1.0f);

    return (vehicle().side() * wanderSide) + (vehicle().up() * wanderUp);
}

Math::vec3 SteerLibrary::pursuit(const Agent& quarry) const
{
    return pursuit(quarry, FLT_MAX);
}

Math::vec3 SteerLibrary::pursuit(const Agent& quarry, float maxPredictionTime) const
{
    const Math::vec3 offset = quarry.position() - vehicle().position();
    const float distance = Math::length(offset);
    const Math::vec3 unitOffset = distance > 0.0f ? offset / distance : vehicle().forward();

    const float parallelness = Math::dot(vehicle().forward(), quarry.forward());

    const float forwardness = Math::dot(vehicle().forward(), unitOffset);

    // 0 if not moving yet, so a parked quarry does not produce NaN.
    const float directTravelTime = vehicle().speed() > 0.0f ? distance / vehicle().speed() : 0.0f;
    const int f = intervalComparison(forwardness, -0.707f, 0.707f);
    const int p = intervalComparison(parallelness, -0.707f, 0.707f);

    float timeFactor = 0.0f;
    switch (f)
    {
    case +1: // ahead
        switch (p)
        {
        case +1:
            timeFactor = 4.0f;
            break; // ahead, parallel
        case 0:
            timeFactor = 1.8f;
            break; // ahead, perpendicular
        case -1:
            timeFactor = 0.85f;
            break; // ahead, anti-parallel
        }
        break;
    case 0: // aside
        switch (p)
        {
        case +1:
            timeFactor = 1.0f;
            break; // aside, parallel
        case 0:
            timeFactor = 0.8f;
            break; // aside, perpendicular
        case -1:
            timeFactor = 4.0f;
            break; // aside, anti-parallel
        }
        break;
    case -1: // behind
        switch (p)
        {
        case +1:
            timeFactor = 0.5f;
            break; // behind, parallel
        case 0:
            timeFactor = 2.0f;
            break; // behind, perpendicular
        case -1:
            timeFactor = 2.0f;
            break; // behind, anti-parallel
        }
        break;
    }

    const float et = directTravelTime * timeFactor;
    const float etl = (et > maxPredictionTime) ? maxPredictionTime : et;

    const Math::vec3 target = quarry.predictFuturePosition(etl);

    return seek(target);
}

Math::vec3 SteerLibrary::evasion(const Agent& menace, float maxPredictionTime) const
{
    const Math::vec3 offset = menace.position() - vehicle().position();
    const float distance = Math::length(offset);

    // Capped at maxPredictionTime (a stationary menace would divide by zero).
    const float roughTime = menace.speed() > 0.0f ? distance / menace.speed() : maxPredictionTime;
    const float predictionTime = (roughTime > maxPredictionTime) ? maxPredictionTime : roughTime;

    const Math::vec3 target = menace.predictFuturePosition(predictionTime);

    return flee(target);
}

bool SteerLibrary::inBoidNeighborhood(const Agent& other, float minDistance, float maxDistance,
                                      float cosMaxAngle) const
{
    if (&other == &vehicle())
        return false;

    const Math::vec3 offset = other.position() - vehicle().position();
    const float distanceSquared = Math::dot(offset, offset);

    if (distanceSquared < (minDistance * minDistance))
        return true;

    if (distanceSquared > (maxDistance * maxDistance))
        return false;

    const Math::vec3 unitOffset = offset / std::sqrt(distanceSquared);
    const float forwardness = Math::dot(vehicle().forward(), unitOffset);
    return forwardness > cosMaxAngle;
}

Math::vec3 SteerLibrary::separation(float maxDistance, float cosMaxAngle,
                                   const std::vector<EntityDist>& flock) const
{
    Math::vec3 steering(0.0f);

    for (const EntityDist& member : flock)
    {
        const Agent& other = *member.entity;
        if (inBoidNeighborhood(other, vehicle().radius() * 3.0f, maxDistance, cosMaxAngle))
        {
            // Opposite of the offset direction, divided once by distance to normalize and again for 1/d falloff.
            const Math::vec3 offset = other.position() - vehicle().position();
            const float distanceSquared = Math::dot(offset, offset);
            if (distanceSquared > 0.0f)
                steering += (offset / -distanceSquared);
        }
    }

    return safeNormalize(steering);
}

Math::vec3 SteerLibrary::alignment(float maxDistance, float cosMaxAngle,
                                  const std::vector<EntityDist>& flock) const
{
    Math::vec3 steering(0.0f);
    int neighbors = 0;

    for (const EntityDist& member : flock)
    {
        const Agent& other = *member.entity;
        if (inBoidNeighborhood(other, vehicle().radius() * 3.0f, maxDistance, cosMaxAngle))
        {
            steering += other.forward();
            ++neighbors;
        }
    }

    if (neighbors > 0)
        steering = safeNormalize((steering / static_cast<float>(neighbors)) - vehicle().forward());

    return steering;
}

Math::vec3 SteerLibrary::cohesion(float maxDistance, float cosMaxAngle,
                                 const std::vector<EntityDist>& flock) const
{
    Math::vec3 steering(0.0f);
    int neighbors = 0;

    for (const EntityDist& member : flock)
    {
        const Agent& other = *member.entity;
        if (inBoidNeighborhood(other, vehicle().radius() * 3.0f, maxDistance, cosMaxAngle))
        {
            steering += other.position();
            ++neighbors;
        }
    }

    if (neighbors > 0)
        steering = safeNormalize((steering / static_cast<float>(neighbors)) - vehicle().position());

    return steering;
}

Math::vec3 SteerLibrary::avoidObstacles(float minTimeToCollision,
                                       const ObstacleGroup& obstacles) const
{
    return Obstacle::steerToAvoidObstacles(vehicle(), minTimeToCollision, obstacles);
}

Math::vec3 SteerLibrary::avoidCloseNeighbors(float minSeparationDistance,
                                            const std::vector<EntityDist>& others) const
{
    for (const EntityDist& entry : others)
    {
        const Agent& other = *entry.entity;
        if (&other == &vehicle())
            continue;

        const float sumOfRadii = vehicle().radius() + other.radius();
        const float minCenterToCenter = minSeparationDistance + sumOfRadii;
        const Math::vec3 offset = other.position() - vehicle().position();
        const float currentDistance = Math::length(offset);

        if (currentDistance < minCenterToCenter)
        {
            // Remove the along-heading component; a head-on overlap projects to zero, so use the vehicle's side as a deterministic escape.
            const Math::vec3 lateral = perpendicularComponent(-offset, vehicle().forward());
            if (Math::dot(lateral, lateral) > 1e-8f)
                return safeNormalize(lateral);
            return vehicle().side();
        }
    }

    return Math::vec3(0.0f);
}

float SteerLibrary::predictNearestApproachTime(const Agent& other) const
{
    const Math::vec3 relVelocity = other.velocity() - vehicle().velocity();
    const float relSpeed = Math::length(relVelocity);

    // Parallel paths stay at the same distance: return 0 (now).
    if (relSpeed == 0.0f)
        return 0.0f;

    const Math::vec3 relTangent = relVelocity / relSpeed;
    const Math::vec3 relPosition = vehicle().position() - other.position();
    const float projection = Math::dot(relTangent, relPosition);

    return projection / relSpeed;
}

float SteerLibrary::computeNearestApproachPositions(const Agent& other, float time)
{
    const Math::vec3 myTravel = vehicle().velocity() * time;
    const Math::vec3 otherTravel = other.velocity() * time;

    const Math::vec3 myFinal = vehicle().position() + myTravel;
    const Math::vec3 otherFinal = other.position() + otherTravel;

    ourPositionAtNearestApproach = myFinal;
    hisPositionAtNearestApproach = otherFinal;

    return Math::length(myFinal - otherFinal);
}

Math::vec3 SteerLibrary::avoidNeighbors(float minTimeToCollision,
                                       const std::vector<EntityDist>& others)
{
    const Math::vec3 separation = avoidCloseNeighbors(0.0f, others);
    if (Math::length(separation) > 0.0f)
        return separation;

    float steer = 0.0f;
    const Agent* threat = nullptr;

    // Threshold: do not look further than this many seconds ahead.
    float minTime = minTimeToCollision;

    Math::vec3 threatPositionAtNearestApproach(0.0f);
    Math::vec3 ourPositionAtNearestApproachTmp(0.0f);

    for (const EntityDist& entry : others)
    {
        const Agent& other = *entry.entity;
        if (&other == &vehicle())
            continue;

        const float collisionDangerThreshold = vehicle().radius() * 2.0f;

        const float time = predictNearestApproachTime(other);

        if ((time >= 0.0f) && (time < minTime))
        {
            if (computeNearestApproachPositions(other, time) < collisionDangerThreshold)
            {
                minTime = time;
                threat = &other;
                threatPositionAtNearestApproach = hisPositionAtNearestApproach;
                ourPositionAtNearestApproachTmp = ourPositionAtNearestApproach;
            }
        }
    }

    if (threat != nullptr)
    {
        const float parallelness = Math::dot(vehicle().forward(), threat->forward());
        const float angle = 0.707f;

        if (parallelness < -angle)
        {
            const Math::vec3 offset = threatPositionAtNearestApproach - vehicle().position();
            const float sideDot = Math::dot(offset, vehicle().side());
            steer = (sideDot > 0.0f) ? -1.0f : 1.0f;
        }
        else if (parallelness > angle)
        {
            const Math::vec3 offset = threat->position() - vehicle().position();
            const float sideDot = Math::dot(offset, vehicle().side());
            steer = (sideDot > 0.0f) ? -1.0f : 1.0f;
        }
        else
        {
            // Perpendicular paths: steer behind the threat (only the slower of the two does this).
            if (threat->speed() <= vehicle().speed())
            {
                const float sideDot = Math::dot(vehicle().side(), threat->velocity());
                steer = (sideDot > 0.0f) ? -1.0f : 1.0f;
            }
        }
    }

    return vehicle().side() * steer;
}

Math::vec3 SteerLibrary::targetSpeed(float targetSpeed) const
{
    const float mf = vehicle().maxForce();
    const float speedError = targetSpeed - vehicle().speed();
    return vehicle().forward() * clip(speedError, -mf, +mf);
}

bool SteerLibrary::isAhead(const Math::vec3& target, float cosThreshold) const
{
    const Math::vec3 offset = target - vehicle().position();
    if (Math::dot(offset, offset) <= 1e-8f)
        return false;
    const Math::vec3 targetDirection = safeNormalize(offset);
    return Math::dot(vehicle().forward(), targetDirection) > cosThreshold;
}

bool SteerLibrary::isAside(const Math::vec3& target, float cosThreshold) const
{
    const Math::vec3 offset = target - vehicle().position();
    if (Math::dot(offset, offset) <= 1e-8f)
        return false;
    const Math::vec3 targetDirection = safeNormalize(offset);
    const float dp = Math::dot(vehicle().forward(), targetDirection);
    return (dp < cosThreshold) && (dp > -cosThreshold);
}

bool SteerLibrary::isBehind(const Math::vec3& target, float cosThreshold) const
{
    const Math::vec3 offset = target - vehicle().position();
    if (Math::dot(offset, offset) <= 1e-8f)
        return false;
    const Math::vec3 targetDirection = safeNormalize(offset);
    return Math::dot(vehicle().forward(), targetDirection) < cosThreshold;
}

void SeekBehavior::iterate(float timeDelta, Agent& entity)
{
    (void)timeDelta;
    mSteer.setVehicle(entity);
    entity.setDesiredMove(entity.desiredMove() + (mSteer.seek(mTarget) * gain()));
}

void FleeBehavior::iterate(float timeDelta, Agent& entity)
{
    (void)timeDelta;
    mSteer.setVehicle(entity);
    entity.setDesiredMove(entity.desiredMove() + (mSteer.flee(mThreat) * gain()));
}

void WanderBehavior::iterate(float timeDelta, Agent& entity)
{
    mSteer.setVehicle(entity);
    entity.setDesiredMove(entity.desiredMove() + (mSteer.wander(timeDelta) * gain()));
}

ObstacleAvoidanceBehavior::ObstacleAvoidanceBehavior(float minTimeToCollision)
    : mMinTimeToCollision(minTimeToCollision)
{
}

void ObstacleAvoidanceBehavior::iterate(float timeDelta, Agent& entity)
{
    (void)timeDelta;

    // An explicit setObstacles() group wins; otherwise use the Scene's group.
    const ObstacleGroup* obstacles = mObstacles;
    if (!obstacles)
    {
        Scene* scene = entity.scene();
        if (!scene)
            return;
        obstacles = &scene->obstacleGroup();
    }
    if (obstacles->empty())
        return;

    mSteer.setVehicle(entity);
    entity.setDesiredMove(entity.desiredMove() +
                          (mSteer.avoidObstacles(mMinTimeToCollision, *obstacles) * gain()));
}

void SteerBehavior::iterate(float timeDelta, Agent& entity)
{
    mSteer.setVehicle(entity);
    if (mFunc)
        entity.setDesiredMove(entity.desiredMove() + (mFunc(mSteer, timeDelta) * gain()));
}

} // namespace Radion::AI
