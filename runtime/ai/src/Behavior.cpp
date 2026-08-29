// Behavior.cpp - flocking behavior implementations.

#include "PCH.h"

#include "Behavior.h"

#include "AIInternal.h"
#include "Agent.h"

#include <cstdlib>

namespace Radion::AI
{

using detail::safeNormalize;
using Radion::Agent;
using Radion::EntityDist;

// --- Behavior (base defaults) ------------------------------------------------
//
// A behavior with no parameters (WanderBehavior, SteerBehavior) inherits
// these as-is; one with parameters overrides only the accessor family its
// own BehaviorParam::Kind values use.

u32 Behavior::paramCount() const
{
    return 0;
}

const BehaviorParam& Behavior::paramInfo(u32 index) const
{
    (void)index;
    static const BehaviorParam kNone{"", BehaviorParam::Kind::Float, 0.0f, 0.0f, ""};
    return kNone;
}

f32 Behavior::paramFloat(u32 index) const
{
    (void)index;
    return 0.0f;
}

void Behavior::setParamFloat(u32 index, f32 value)
{
    (void)index;
    (void)value;
}

glm::vec3 Behavior::paramVec3(u32 index) const
{
    (void)index;
    return glm::vec3(0.0f);
}

void Behavior::setParamVec3(u32 index, const glm::vec3& value)
{
    (void)index;
    (void)value;
}

bool Behavior::paramBool(u32 index) const
{
    (void)index;
    return false;
}

void Behavior::setParamBool(u32 index, bool value)
{
    (void)index;
    (void)value;
}

// --- Separation -------------------------------------------------------------

namespace
{
const BehaviorParam kSeparationParams[] = {
    {"Separation Distance", BehaviorParam::Kind::Float, 0.0f, 20.0f,
     "How close a group member has to be before this agent starts pushing away from it."},
    {"Min Separation %", BehaviorParam::Kind::Float, 0.0f, 1.0f,
     "Weakest push applied right at the separation distance edge, as a fraction of full strength."},
    {"Max Separation %", BehaviorParam::Kind::Float, 0.0f, 1.0f,
     "Strongest push applied once a neighbour is fully overlapping this agent."},
};
} // namespace

SeparationBehavior::SeparationBehavior(float separationDistance, float minPercent, float maxPercent)
    : mSeparationDistance(separationDistance), mMinSeparationPercentage(minPercent),
      mMaxSeparationPercentage(maxPercent)
{
}

u32 SeparationBehavior::paramCount() const
{
    return static_cast<u32>(sizeof(kSeparationParams) / sizeof(kSeparationParams[0]));
}

const BehaviorParam& SeparationBehavior::paramInfo(u32 index) const
{
    return kSeparationParams[index];
}

f32 SeparationBehavior::paramFloat(u32 index) const
{
    switch (index)
    {
    case 0: return mSeparationDistance;
    case 1: return mMinSeparationPercentage;
    case 2: return mMaxSeparationPercentage;
    default: return 0.0f;
    }
}

void SeparationBehavior::setParamFloat(u32 index, f32 value)
{
    switch (index)
    {
    case 0: mSeparationDistance = value; break;
    case 1: mMinSeparationPercentage = value; break;
    case 2: mMaxSeparationPercentage = value; break;
    default: break;
    }
}

void SeparationBehavior::iterate(float timeDelta, Agent& entity)
{
    (void)timeDelta;

    const std::vector<EntityDist>& groupMembers = entity.visibleGroupMembers();
    if (groupMembers.empty())
        return;

    // Repel from EVERY neighbour inside the separation distance (summed, with
    // a falloff), not just the closest one. Reacting to a single neighbour
    // lets a school collapse into a dense ball at the centre of mass where
    // separation and cohesion fight every frame - the "crazy fish at the
    // centre" look.
    glm::vec3 separationPush(0.0f);
    for (const EntityDist& member : groupMembers)
    {
        const float d = member.distance;
        if (d >= mSeparationDistance)
            continue;

        // 0.0 at the separation edge, growing to (1 - minPct) as the
        // neighbour fully overlaps us.
        const float pct =
            std::clamp(d / mSeparationDistance, mMinSeparationPercentage, mMaxSeparationPercentage);

        glm::vec3 away = member.entity->position() - entity.position();
        if (glm::dot(away, away) < 1e-8f)
            away = glm::vec3(1.0f, 0.0f, 0.0f); // coincident - pick a stable direction
        else
            away = safeNormalize(away);

        separationPush -= away * (1.0f - pct);
    }

    if (glm::dot(separationPush, separationPush) < 1e-8f)
        return;

    glm::vec3 currentDesiredMove = entity.desiredMove();
    currentDesiredMove += safeNormalize(separationPush) * gain();
    entity.setDesiredMove(currentDesiredMove);
}

// --- Alignment --------------------------------------------------------------

namespace
{
const BehaviorParam kAlignmentParams[] = {
    {"Turn Rate", BehaviorParam::Kind::Float, 0.0f, 2.0f,
     "How strongly this agent turns to match its nearest group member's heading each update."},
};
} // namespace

AlignmentBehavior::AlignmentBehavior(float turnRate) : mTurnRate(turnRate)
{
}

u32 AlignmentBehavior::paramCount() const
{
    return static_cast<u32>(sizeof(kAlignmentParams) / sizeof(kAlignmentParams[0]));
}

const BehaviorParam& AlignmentBehavior::paramInfo(u32 index) const
{
    return kAlignmentParams[index];
}

f32 AlignmentBehavior::paramFloat(u32 index) const
{
    (void)index;
    return mTurnRate;
}

void AlignmentBehavior::setParamFloat(u32 index, f32 value)
{
    (void)index;
    mTurnRate = value;
}

void AlignmentBehavior::iterate(float timeDelta, Agent& entity)
{
    (void)timeDelta;

    const std::vector<EntityDist>& groupMembers = entity.visibleGroupMembers();
    if (groupMembers.empty())
        return;

    const Agent& nearestGroupMember = *groupMembers.front().entity;

    // Match the heading of our closest group member.
    glm::vec3 desiredMoveAdj = safeNormalize(nearestGroupMember.velocity()) * mTurnRate;
    glm::vec3 currentDesiredMove = entity.desiredMove();
    currentDesiredMove += desiredMoveAdj * gain();
    entity.setDesiredMove(currentDesiredMove);
}

// --- Cohesion ---------------------------------------------------------------

namespace
{
const BehaviorParam kCohesionParams[] = {
    {"Turn Rate", BehaviorParam::Kind::Float, 0.0f, 2.0f,
     "How strongly this agent steers toward the group's centre of mass each update."},
};
} // namespace

CohesionBehavior::CohesionBehavior(float turnRate) : mTurnRate(turnRate)
{
}

u32 CohesionBehavior::paramCount() const
{
    return static_cast<u32>(sizeof(kCohesionParams) / sizeof(kCohesionParams[0]));
}

const BehaviorParam& CohesionBehavior::paramInfo(u32 index) const
{
    return kCohesionParams[index];
}

f32 CohesionBehavior::paramFloat(u32 index) const
{
    (void)index;
    return mTurnRate;
}

void CohesionBehavior::setParamFloat(u32 index, f32 value)
{
    (void)index;
    mTurnRate = value;
}

void CohesionBehavior::iterate(float timeDelta, Agent& entity)
{
    (void)timeDelta;

    const std::vector<EntityDist>& groupMembers = entity.visibleGroupMembers();
    if (groupMembers.empty())
        return;

    // Compute the centre of mass of the group.
    glm::vec3 groupCenterOfMass(0.0f);
    for (const EntityDist& member : groupMembers)
        groupCenterOfMass += member.entity->position();
    groupCenterOfMass /= static_cast<float>(groupMembers.size());

    // Dead zone: when we are essentially ON the centre of mass the direction
    // is pure floating-point noise and the force fights the separation every
    // frame (the "crazy at the centre" look). Skip it until we drift away.
    const glm::vec3 toCenterOfMass = groupCenterOfMass - entity.position();
    if (glm::dot(toCenterOfMass, toCenterOfMass) < 0.0625f) // < 0.25 units
        return;

    // Move toward the centre of the group.
    glm::vec3 desiredMoveAdj = safeNormalize(toCenterOfMass) * mTurnRate;
    glm::vec3 currentDesiredMove = entity.desiredMove();
    currentDesiredMove += desiredMoveAdj * gain();
    entity.setDesiredMove(currentDesiredMove);
}

// --- Avoidance --------------------------------------------------------------

namespace
{
const BehaviorParam kAvoidanceParams[] = {
    {"Avoidance Distance", BehaviorParam::Kind::Float, 0.0f, 20.0f,
     "Distance to the nearest visible enemy inside which this agent turns and runs."},
    {"Avoidance Speed", BehaviorParam::Kind::Float, 0.0f, 10.0f,
     "Strength of the desired-move push applied away from that enemy."},
};
} // namespace

AvoidanceBehavior::AvoidanceBehavior(float avoidanceDistance, float avoidanceSpeed)
    : mAvoidanceDistance(avoidanceDistance), mAvoidanceSpeed(avoidanceSpeed)
{
}

u32 AvoidanceBehavior::paramCount() const
{
    return static_cast<u32>(sizeof(kAvoidanceParams) / sizeof(kAvoidanceParams[0]));
}

const BehaviorParam& AvoidanceBehavior::paramInfo(u32 index) const
{
    return kAvoidanceParams[index];
}

f32 AvoidanceBehavior::paramFloat(u32 index) const
{
    switch (index)
    {
    case 0: return mAvoidanceDistance;
    case 1: return mAvoidanceSpeed;
    default: return 0.0f;
    }
}

void AvoidanceBehavior::setParamFloat(u32 index, f32 value)
{
    switch (index)
    {
    case 0: mAvoidanceDistance = value; break;
    case 1: mAvoidanceSpeed = value; break;
    default: break;
    }
}

void AvoidanceBehavior::iterate(float timeDelta, Agent& entity)
{
    (void)timeDelta;

    const std::vector<EntityDist>& enemies = entity.visibleEnemies();
    if (enemies.empty())
        return;

    const Agent& nearestEnemy = *enemies.front().entity;
    float nearestEnemyDist = enemies.front().distance;

    // Head away from the enemy.
    if (nearestEnemyDist < mAvoidanceDistance)
    {
        glm::vec3 desiredMoveAdj =
            safeNormalize(entity.position() - nearestEnemy.position()) * mAvoidanceSpeed;
        glm::vec3 currentDesiredMove = entity.desiredMove();
        currentDesiredMove += desiredMoveAdj * gain();
        entity.setDesiredMove(currentDesiredMove);
    }
}

// --- Cruising ---------------------------------------------------------------

namespace
{
const BehaviorParam kCruisingParams[] = {
    {"Rand Move X Chance", BehaviorParam::Kind::Float, 0.0f, 1.0f,
     "Per-roll probability of a random nudge along the local X axis."},
    {"Rand Move Y Chance", BehaviorParam::Kind::Float, 0.0f, 1.0f,
     "Per-roll probability of a random nudge along the local Y axis."},
    {"Rand Move Z Chance", BehaviorParam::Kind::Float, 0.0f, 1.0f,
     "Per-roll probability of a random nudge along the local Z axis; the three chances are "
     "tested as cumulative bands, so they need not add up to 1."},
    {"Min Random Move", BehaviorParam::Kind::Float, -2.0f, 2.0f,
     "Base magnitude (and sign) of the random per-axis nudge before it is renormalized."},
    {"Max Rate Change", BehaviorParam::Kind::Float, 0.0f, 1.0f,
     "Upper clamp on how much of the gap to the desired speed is corrected in one update."},
    {"Min Rate Change", BehaviorParam::Kind::Float, 0.0f, 1.0f,
     "Lower clamp on that same correction, and the strength of the random-move contribution."},
};
} // namespace

CruisingBehavior::CruisingBehavior(float randMoveXChance, float randMoveYChance,
                                   float randMoveZChance, float minRandomMove, float maxRateChange,
                                   float minRateChange)
    : mRandMoveXChance(randMoveXChance), mRandMoveYChance(randMoveYChance),
      mRandMoveZChance(randMoveZChance), mMinRandomMove(minRandomMove),
      mMaxRateChange(maxRateChange), mMinRateChange(minRateChange)
{
}

u32 CruisingBehavior::paramCount() const
{
    return static_cast<u32>(sizeof(kCruisingParams) / sizeof(kCruisingParams[0]));
}

const BehaviorParam& CruisingBehavior::paramInfo(u32 index) const
{
    return kCruisingParams[index];
}

f32 CruisingBehavior::paramFloat(u32 index) const
{
    switch (index)
    {
    case 0: return mRandMoveXChance;
    case 1: return mRandMoveYChance;
    case 2: return mRandMoveZChance;
    case 3: return mMinRandomMove;
    case 4: return mMaxRateChange;
    case 5: return mMinRateChange;
    default: return 0.0f;
    }
}

void CruisingBehavior::setParamFloat(u32 index, f32 value)
{
    switch (index)
    {
    case 0: mRandMoveXChance = value; break;
    case 1: mRandMoveYChance = value; break;
    case 2: mRandMoveZChance = value; break;
    case 3: mMinRandomMove = value; break;
    case 4: mMaxRateChange = value; break;
    case 5: mMinRateChange = value; break;
    default: break;
    }
}

void CruisingBehavior::iterate(float timeDelta, Agent& entity)
{
    (void)timeDelta;

    // How fast we are going vs how fast we'd like to be going.
    float currentSpeed = glm::length(entity.velocity());
    float percentDesiredSpeed =
        std::fabs((currentSpeed - entity.desiredSpeed()) / entity.maxSpeed());
    float signum = (currentSpeed - entity.desiredSpeed()) > 0.0f ? -1.0f : 1.0f;

    // Clamp rate changes.
    percentDesiredSpeed = std::clamp(percentDesiredSpeed, mMinRateChange, mMaxRateChange);

    // Add some random movement. The chances are per-axis probabilities, so
    // the roll is tested against cumulative bands - comparing each against
    // the raw roll would make an axis unreachable whenever its chance is
    // smaller than the previous one (Y never fired for X=0.45, Y=0.2).
    glm::vec3 desiredMoveAdj(0.0f);
    float randmove = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
    if (randmove < mRandMoveXChance)
        desiredMoveAdj.x += mMinRandomMove * signum;
    else if (randmove < mRandMoveXChance + mRandMoveYChance)
        desiredMoveAdj.y += mMinRandomMove * signum;
    else if (randmove < mRandMoveXChance + mRandMoveYChance + mRandMoveZChance)
        desiredMoveAdj.z += mMinRandomMove * signum;

    // Scaled by how far off the desired speed the agent actually is - which
    // is what percentDesiredSpeed was computed and clamped for, and was then
    // dropped in favour of a constant mMinRateChange. With the constant, the
    // nudge never varied, the "toward/away from the desired speed" half of
    // this behavior never happened, and mMaxRateChange had no effect at all
    // (it only ever bounded a value nothing read). The clamp's lower bound
    // is mMinRateChange, so an agent already at its desired speed gets what
    // it got before.
    glm::vec3 currentDesiredMove = entity.desiredMove();
    desiredMoveAdj = safeNormalize(desiredMoveAdj) * (percentDesiredSpeed * signum);
    currentDesiredMove += desiredMoveAdj * gain();
    entity.setDesiredMove(currentDesiredMove);
}

// --- Stay Within Sphere -----------------------------------------------------

namespace
{
const BehaviorParam kStayWithinSphereParams[] = {
    {"Sphere Center", BehaviorParam::Kind::Vec3, 0.0f, 0.0f,
     "World-space centre of the sphere this agent is kept inside of."},
    {"Sphere Radius", BehaviorParam::Kind::Float, 0.0f, 100.0f,
     "Distance from the centre at which the agent is steered back in."},
};
} // namespace

StayWithinSphereBehavior::StayWithinSphereBehavior(const glm::vec3& center, float radius)
    : mCenter(center), mRadius(radius)
{
}

u32 StayWithinSphereBehavior::paramCount() const
{
    return static_cast<u32>(sizeof(kStayWithinSphereParams) / sizeof(kStayWithinSphereParams[0]));
}

const BehaviorParam& StayWithinSphereBehavior::paramInfo(u32 index) const
{
    return kStayWithinSphereParams[index];
}

f32 StayWithinSphereBehavior::paramFloat(u32 index) const
{
    (void)index;
    return mRadius;
}

void StayWithinSphereBehavior::setParamFloat(u32 index, f32 value)
{
    (void)index;
    mRadius = value;
}

glm::vec3 StayWithinSphereBehavior::paramVec3(u32 index) const
{
    (void)index;
    return mCenter;
}

void StayWithinSphereBehavior::setParamVec3(u32 index, const glm::vec3& value)
{
    (void)index;
    mCenter = value;
}

void StayWithinSphereBehavior::iterate(float timeDelta, Agent& entity)
{
    (void)timeDelta;

    glm::vec3 toCenter = mCenter - entity.position();
    float dist = glm::length(toCenter);
    if (dist > mRadius)
    {
        glm::vec3 desiredMoveAdj = safeNormalize(toCenter) * entity.maxSpeed();
        glm::vec3 currentDesiredMove = entity.desiredMove();
        currentDesiredMove += desiredMoveAdj * gain();
        entity.setDesiredMove(currentDesiredMove);
    }
}

// --- Combat -------------------------------------------------------------

namespace
{
const BehaviorParam kCombatParams[] = {
    {"Fire Range", BehaviorParam::Kind::Float, 0.0f, 50.0f,
     "Distance to the nearest visible enemy within which this agent opens fire."},
    {"Damage Per Hit", BehaviorParam::Kind::Float, 0.0f, 100.0f,
     "Health removed from the target on each shot."},
    {"Fire Interval", BehaviorParam::Kind::Float, 0.0f, 10.0f,
     "Seconds of cooldown enforced after a shot before the next one can fire."},
};
} // namespace

CombatBehavior::CombatBehavior(float fireRange, float damagePerHit, float fireInterval)
    : mFireRange(fireRange), mDamagePerHit(damagePerHit), mFireInterval(fireInterval)
{
}

u32 CombatBehavior::paramCount() const
{
    return static_cast<u32>(sizeof(kCombatParams) / sizeof(kCombatParams[0]));
}

const BehaviorParam& CombatBehavior::paramInfo(u32 index) const
{
    return kCombatParams[index];
}

f32 CombatBehavior::paramFloat(u32 index) const
{
    switch (index)
    {
    case 0: return mFireRange;
    case 1: return mDamagePerHit;
    case 2: return mFireInterval;
    default: return 0.0f;
    }
}

void CombatBehavior::setParamFloat(u32 index, f32 value)
{
    switch (index)
    {
    case 0: mFireRange = value; break;
    case 1: mDamagePerHit = value; break;
    case 2: mFireInterval = value; break;
    default: break;
    }
}

void CombatBehavior::iterate(float timeDelta, Agent& entity)
{
    entity.tickAttackCooldown(timeDelta);

    const std::vector<EntityDist>& enemies = entity.visibleEnemies();
    if (enemies.empty() || entity.attackCooldown() > 0.0f)
        return;

    // Sorted ascending by distance - front() is the nearest.
    const EntityDist& nearest = enemies.front();
    if (nearest.distance > mFireRange)
        return;

    nearest.entity->applyDamage(mDamagePerHit);
    entity.setAttackCooldown(mFireInterval);
    entity.markFired(nearest.entity);
}

} // namespace Radion::AI
