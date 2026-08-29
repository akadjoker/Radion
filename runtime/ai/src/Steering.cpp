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

glm::vec3 SeekBehavior::paramVec3(u32 index) const
{
    (void)index;
    return mTarget;
}

void SeekBehavior::setParamVec3(u32 index, const glm::vec3& value)
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

glm::vec3 FleeBehavior::paramVec3(u32 index) const
{
    (void)index;
    return mThreat;
}

void FleeBehavior::setParamVec3(u32 index, const glm::vec3& value)
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

// --- seek / flee ------------------------------------------------------------

glm::vec3 SteerLibrary::seek(const glm::vec3& target) const
{
    const glm::vec3 desiredVelocity = target - vehicle().position();
    return desiredVelocity - vehicle().velocity();
}

glm::vec3 SteerLibrary::flee(const glm::vec3& target) const
{
    const glm::vec3 desiredVelocity = vehicle().position() - target;
    return desiredVelocity - vehicle().velocity();
}

// --- wander -----------------------------------------------------------------

glm::vec3 SteerLibrary::wander(float dt)
{
    // Random walk WanderSide and WanderUp between -1 and +1.
    const float speed = 12.0f * dt;
    wanderSide = scalarRandomWalk(wanderSide, speed, -1.0f, +1.0f);
    wanderUp = scalarRandomWalk(wanderUp, speed, -1.0f, +1.0f);

    // Return a pure lateral steering vector: (+/-Side) + (+/-Up).
    return (vehicle().side() * wanderSide) + (vehicle().up() * wanderUp);
}

// --- pursuit / evasion ------------------------------------------------------

glm::vec3 SteerLibrary::pursuit(const Agent& quarry) const
{
    return pursuit(quarry, FLT_MAX);
}

glm::vec3 SteerLibrary::pursuit(const Agent& quarry, float maxPredictionTime) const
{
    // Offset from this to the quarry, its distance, and a unit vector toward it.
    const glm::vec3 offset = quarry.position() - vehicle().position();
    const float distance = glm::length(offset);
    const glm::vec3 unitOffset = distance > 0.0f ? offset / distance : vehicle().forward();

    // How parallel are the paths of "this" and the quarry?
    const float parallelness = glm::dot(vehicle().forward(), quarry.forward());

    // How "forward" is the direction to the quarry?
    const float forwardness = glm::dot(vehicle().forward(), unitOffset);

    // Estimated time to intercept at current speed (0 if not moving yet, so a
    // parked quarry doesn't produce NaN).
    const float directTravelTime = vehicle().speed() > 0.0f ? distance / vehicle().speed() : 0.0f;
    const int f = intervalComparison(forwardness, -0.707f, 0.707f);
    const int p = intervalComparison(parallelness, -0.707f, 0.707f);

    // Time factor for the nine cases of quarry position/heading.
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

    // Estimated time until intercept of the quarry.
    const float et = directTravelTime * timeFactor;
    const float etl = (et > maxPredictionTime) ? maxPredictionTime : et;

    // Estimated position of the quarry at intercept.
    const glm::vec3 target = quarry.predictFuturePosition(etl);

    return seek(target);
}

glm::vec3 SteerLibrary::evasion(const Agent& menace, float maxPredictionTime) const
{
    // Offset from this to the menace, its distance, unit vector toward menace.
    const glm::vec3 offset = menace.position() - vehicle().position();
    const float distance = glm::length(offset);

    // Predicted intercept time, capped at maxPredictionTime (a stationary
    // menace is predicted at the cap rather than dividing by zero).
    const float roughTime = menace.speed() > 0.0f ? distance / menace.speed() : maxPredictionTime;
    const float predictionTime = (roughTime > maxPredictionTime) ? maxPredictionTime : roughTime;

    const glm::vec3 target = menace.predictFuturePosition(predictionTime);

    return flee(target);
}

// --- flocking --------------------------------------------------------------

bool SteerLibrary::inBoidNeighborhood(const Agent& other, float minDistance, float maxDistance,
                                      float cosMaxAngle) const
{
    if (&other == &vehicle())
        return false;

    const glm::vec3 offset = other.position() - vehicle().position();
    const float distanceSquared = glm::dot(offset, offset);

    // Definitely in the neighborhood if inside the minDistance sphere.
    if (distanceSquared < (minDistance * minDistance))
        return true;

    // Definitely not in the neighborhood if outside the maxDistance sphere.
    if (distanceSquared > (maxDistance * maxDistance))
        return false;

    // Otherwise, test the angular offset from the forward axis.
    const glm::vec3 unitOffset = offset / std::sqrt(distanceSquared);
    const float forwardness = glm::dot(vehicle().forward(), unitOffset);
    return forwardness > cosMaxAngle;
}

glm::vec3 SteerLibrary::separation(float maxDistance, float cosMaxAngle,
                                   const std::vector<EntityDist>& flock) const
{
    // Steering accumulator and neighbor count, both initially zero.
    glm::vec3 steering(0.0f);

    for (const EntityDist& member : flock)
    {
        const Agent& other = *member.entity;
        if (inBoidNeighborhood(other, vehicle().radius() * 3.0f, maxDistance, cosMaxAngle))
        {
            // Add the steering contribution: opposite of the offset direction,
            // divided once by distance to normalize, again for a 1/d falloff.
            const glm::vec3 offset = other.position() - vehicle().position();
            const float distanceSquared = glm::dot(offset, offset);
            if (distanceSquared > 0.0f)
                steering += (offset / -distanceSquared);
        }
    }

    // Normalize to a pure direction.
    return safeNormalize(steering);
}

glm::vec3 SteerLibrary::alignment(float maxDistance, float cosMaxAngle,
                                  const std::vector<EntityDist>& flock) const
{
    glm::vec3 steering(0.0f);
    int neighbors = 0;

    for (const EntityDist& member : flock)
    {
        const Agent& other = *member.entity;
        if (inBoidNeighborhood(other, vehicle().radius() * 3.0f, maxDistance, cosMaxAngle))
        {
            // Accumulate the sum of the neighbor headings.
            steering += other.forward();
            ++neighbors;
        }
    }

    // Divide by neighbors, subtract the current heading to get the error-
    // correcting direction, then normalize to a pure direction.
    if (neighbors > 0)
        steering = safeNormalize((steering / static_cast<float>(neighbors)) - vehicle().forward());

    return steering;
}

glm::vec3 SteerLibrary::cohesion(float maxDistance, float cosMaxAngle,
                                 const std::vector<EntityDist>& flock) const
{
    glm::vec3 steering(0.0f);
    int neighbors = 0;

    for (const EntityDist& member : flock)
    {
        const Agent& other = *member.entity;
        if (inBoidNeighborhood(other, vehicle().radius() * 3.0f, maxDistance, cosMaxAngle))
        {
            // Accumulate the sum of the neighbor positions.
            steering += other.position();
            ++neighbors;
        }
    }

    // Divide by neighbors, subtract the current position to get the error-
    // correcting direction, then normalize to a pure direction.
    if (neighbors > 0)
        steering = safeNormalize((steering / static_cast<float>(neighbors)) - vehicle().position());

    return steering;
}

// --- obstacle avoidance -----------------------------------------------------

glm::vec3 SteerLibrary::avoidObstacles(float minTimeToCollision,
                                       const ObstacleGroup& obstacles) const
{
    return Obstacle::steerToAvoidObstacles(vehicle(), minTimeToCollision, obstacles);
}

// --- neighbor avoidance -----------------------------------------------------

glm::vec3 SteerLibrary::avoidCloseNeighbors(float minSeparationDistance,
                                            const std::vector<EntityDist>& others) const
{
    // Hard steer away from any other agent within a critical distance.
    for (const EntityDist& entry : others)
    {
        const Agent& other = *entry.entity;
        if (&other == &vehicle())
            continue;

        const float sumOfRadii = vehicle().radius() + other.radius();
        const float minCenterToCenter = minSeparationDistance + sumOfRadii;
        const glm::vec3 offset = other.position() - vehicle().position();
        const float currentDistance = glm::length(offset);

        if (currentDistance < minCenterToCenter)
        {
            // Remove the component along the heading, then normalize the
            // lateral escape direction.  If the overlap is head-on this
            // projection is zero; use the vehicle's side as a deterministic
            // escape direction instead of returning no avoidance force.
            const glm::vec3 lateral = perpendicularComponent(-offset, vehicle().forward());
            if (glm::dot(lateral, lateral) > 1e-8f)
                return safeNormalize(lateral);
            return vehicle().side();
        }
    }

    return glm::vec3(0.0f);
}

float SteerLibrary::predictNearestApproachTime(const Agent& other) const
{
    // Imagine we are at the origin with no velocity; compute the relative
    // velocity of the other agent.
    const glm::vec3 relVelocity = other.velocity() - vehicle().velocity();
    const float relSpeed = glm::length(relVelocity);

    // For parallel paths the vehicles are always at the same distance, so
    // return 0 (aka "now") - "there is no time like the present".
    if (relSpeed == 0.0f)
        return 0.0f;

    // In this relative space the other vehicle's path is a line defined by
    // its relative position and velocity. The distance from the origin (us)
    // to that line is the nearest approach.
    const glm::vec3 relTangent = relVelocity / relSpeed;
    const glm::vec3 relPosition = vehicle().position() - other.position();
    const float projection = glm::dot(relTangent, relPosition);

    return projection / relSpeed;
}

float SteerLibrary::computeNearestApproachPositions(const Agent& other, float time)
{
    const glm::vec3 myTravel = vehicle().velocity() * time;
    const glm::vec3 otherTravel = other.velocity() * time;

    const glm::vec3 myFinal = vehicle().position() + myTravel;
    const glm::vec3 otherFinal = other.position() + otherTravel;

    // For annotation.
    ourPositionAtNearestApproach = myFinal;
    hisPositionAtNearestApproach = otherFinal;

    return glm::length(myFinal - otherFinal);
}

glm::vec3 SteerLibrary::avoidNeighbors(float minTimeToCollision,
                                       const std::vector<EntityDist>& others)
{
    // First priority is to prevent immediate interpenetration.
    const glm::vec3 separation = avoidCloseNeighbors(0.0f, others);
    if (glm::length(separation) > 0.0f)
        return separation;

    // Otherwise, go on to consider potential future collisions.
    float steer = 0.0f;
    const Agent* threat = nullptr;

    // Time (in seconds) until the most immediate collision threat found so
    // far; initial value is a threshold: don't look more than this many
    // seconds into the future.
    float minTime = minTimeToCollision;

    glm::vec3 threatPositionAtNearestApproach(0.0f);
    glm::vec3 ourPositionAtNearestApproachTmp(0.0f);

    for (const EntityDist& entry : others)
    {
        const Agent& other = *entry.entity;
        if (&other == &vehicle())
            continue;

        // Avoid when future positions are this close (or less).
        const float collisionDangerThreshold = vehicle().radius() * 2.0f;

        // Predicted time until the nearest approach of us and this one.
        const float time = predictNearestApproachTime(other);

        // If the time is in the future, sooner than any other threatened
        // collision...
        if ((time >= 0.0f) && (time < minTime))
        {
            // If the two will be close enough to collide, make a note of it.
            if (computeNearestApproachPositions(other, time) < collisionDangerThreshold)
            {
                minTime = time;
                threat = &other;
                threatPositionAtNearestApproach = hisPositionAtNearestApproach;
                ourPositionAtNearestApproachTmp = ourPositionAtNearestApproach;
            }
        }
    }

    // If a potential collision was found, compute steering to avoid it.
    if (threat != nullptr)
    {
        // Parallel: +1, perpendicular: 0, anti-parallel: -1.
        const float parallelness = glm::dot(vehicle().forward(), threat->forward());
        const float angle = 0.707f;

        if (parallelness < -angle)
        {
            // Anti-parallel "head on" paths: steer away from the future
            // threat position.
            const glm::vec3 offset = threatPositionAtNearestApproach - vehicle().position();
            const float sideDot = glm::dot(offset, vehicle().side());
            steer = (sideDot > 0.0f) ? -1.0f : 1.0f;
        }
        else if (parallelness > angle)
        {
            // Parallel paths: steer away from the threat.
            const glm::vec3 offset = threat->position() - vehicle().position();
            const float sideDot = glm::dot(offset, vehicle().side());
            steer = (sideDot > 0.0f) ? -1.0f : 1.0f;
        }
        else
        {
            // Perpendicular paths: steer behind the threat (only the slower of
            // the two does this).
            if (threat->speed() <= vehicle().speed())
            {
                const float sideDot = glm::dot(vehicle().side(), threat->velocity());
                steer = (sideDot > 0.0f) ? -1.0f : 1.0f;
            }
        }
    }

    return vehicle().side() * steer;
}

// --- target speed -----------------------------------------------------------

glm::vec3 SteerLibrary::targetSpeed(float targetSpeed) const
{
    const float mf = vehicle().maxForce();
    const float speedError = targetSpeed - vehicle().speed();
    return vehicle().forward() * clip(speedError, -mf, +mf);
}

// --- isAhead / isAside / isBehind -------------------------------------------

bool SteerLibrary::isAhead(const glm::vec3& target, float cosThreshold) const
{
    const glm::vec3 offset = target - vehicle().position();
    if (glm::dot(offset, offset) <= 1e-8f)
        return false;
    const glm::vec3 targetDirection = safeNormalize(offset);
    return glm::dot(vehicle().forward(), targetDirection) > cosThreshold;
}

bool SteerLibrary::isAside(const glm::vec3& target, float cosThreshold) const
{
    const glm::vec3 offset = target - vehicle().position();
    if (glm::dot(offset, offset) <= 1e-8f)
        return false;
    const glm::vec3 targetDirection = safeNormalize(offset);
    const float dp = glm::dot(vehicle().forward(), targetDirection);
    return (dp < cosThreshold) && (dp > -cosThreshold);
}

bool SteerLibrary::isBehind(const glm::vec3& target, float cosThreshold) const
{
    const glm::vec3 offset = target - vehicle().position();
    if (glm::dot(offset, offset) <= 1e-8f)
        return false;
    const glm::vec3 targetDirection = safeNormalize(offset);
    return glm::dot(vehicle().forward(), targetDirection) < cosThreshold;
}

// --- convenience behaviors --------------------------------------------------

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

// DESVIO 2: WanderBehavior's mSteer (and the wanderSide/wanderUp state it
// carries) used to be shared by every agent the same behavior instance was
// added to (Entity::addBehavior() was non-owning, Steering.h:157 before this
// port) - every one of them wandered in lockstep, the same random-walk
// sequence. Behaviors are now owned one-per-agent (Agent::addBehavior()), so
// mSteer is this agent's alone and each wanders independently. No math here
// changed to make that true - it already would have been true had the
// instance not been shared.
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

    // An explicit setObstacles() group wins; otherwise fall back to whatever
    // the Scene collects from every Radion::Obstacle component - the caller
    // no longer has to assemble a group by hand for the common case.
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
