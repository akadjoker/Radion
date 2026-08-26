// Behavior.cpp - flocking behavior implementations.

#include "PCH.h"

#include "Behavior.h"

#include "AIInternal.h"
#include "Entity.h"

#include <cstdlib>

namespace Radion::AI
{

using detail::safeNormalize;

// --- Separation -------------------------------------------------------------

SeparationBehavior::SeparationBehavior(float separationDistance, float minPercent, float maxPercent)
    : mSeparationDistance(separationDistance), mMinSeparationPercentage(minPercent),
      mMaxSeparationPercentage(maxPercent)
{
}

void SeparationBehavior::iterate(float timeDelta, Entity& entity)
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
    Math::Vec3 separationPush(0.0f);
    for (const EntityDist& member : groupMembers)
    {
        const float d = member.distance;
        if (d >= mSeparationDistance)
            continue;

        // 0.0 at the separation edge, growing to (1 - minPct) as the
        // neighbour fully overlaps us.
        const float pct =
            std::clamp(d / mSeparationDistance, mMinSeparationPercentage, mMaxSeparationPercentage);

        Math::Vec3 away = member.entity->position() - entity.position();
        if (glm::dot(away, away) < 1e-8f)
            away = Math::Vec3(1.0f, 0.0f, 0.0f); // coincident - pick a stable direction
        else
            away = safeNormalize(away);

        separationPush -= away * (1.0f - pct);
    }

    if (glm::dot(separationPush, separationPush) < 1e-8f)
        return;

    Math::Vec3 currentDesiredMove = entity.desiredMove();
    currentDesiredMove += safeNormalize(separationPush) * gain();
    entity.setDesiredMove(currentDesiredMove);
}

// --- Alignment --------------------------------------------------------------

AlignmentBehavior::AlignmentBehavior(float turnRate) : mTurnRate(turnRate)
{
}

void AlignmentBehavior::iterate(float timeDelta, Entity& entity)
{
    (void)timeDelta;

    const std::vector<EntityDist>& groupMembers = entity.visibleGroupMembers();
    if (groupMembers.empty())
        return;

    const Entity& nearestGroupMember = *groupMembers.front().entity;

    // Match the heading of our closest group member.
    Math::Vec3 desiredMoveAdj = safeNormalize(nearestGroupMember.velocity()) * mTurnRate;
    Math::Vec3 currentDesiredMove = entity.desiredMove();
    currentDesiredMove += desiredMoveAdj * gain();
    entity.setDesiredMove(currentDesiredMove);
}

// --- Cohesion ---------------------------------------------------------------

CohesionBehavior::CohesionBehavior(float turnRate) : mTurnRate(turnRate)
{
}

void CohesionBehavior::iterate(float timeDelta, Entity& entity)
{
    (void)timeDelta;

    const std::vector<EntityDist>& groupMembers = entity.visibleGroupMembers();
    if (groupMembers.empty())
        return;

    // Compute the centre of mass of the group.
    Math::Vec3 groupCenterOfMass(0.0f);
    for (const EntityDist& member : groupMembers)
        groupCenterOfMass += member.entity->position();
    groupCenterOfMass /= static_cast<float>(groupMembers.size());

    // Dead zone: when we are essentially ON the centre of mass the direction
    // is pure floating-point noise and the force fights the separation every
    // frame (the "crazy at the centre" look). Skip it until we drift away.
    const Math::Vec3 toCenterOfMass = groupCenterOfMass - entity.position();
    if (glm::dot(toCenterOfMass, toCenterOfMass) < 0.0625f) // < 0.25 units
        return;

    // Move toward the centre of the group.
    Math::Vec3 desiredMoveAdj = safeNormalize(toCenterOfMass) * mTurnRate;
    Math::Vec3 currentDesiredMove = entity.desiredMove();
    currentDesiredMove += desiredMoveAdj * gain();
    entity.setDesiredMove(currentDesiredMove);
}

// --- Avoidance --------------------------------------------------------------

AvoidanceBehavior::AvoidanceBehavior(float avoidanceDistance, float avoidanceSpeed)
    : mAvoidanceDistance(avoidanceDistance), mAvoidanceSpeed(avoidanceSpeed)
{
}

void AvoidanceBehavior::iterate(float timeDelta, Entity& entity)
{
    (void)timeDelta;

    const std::vector<EntityDist>& enemies = entity.visibleEnemies();
    if (enemies.empty())
        return;

    const Entity& nearestEnemy = *enemies.front().entity;
    float nearestEnemyDist = enemies.front().distance;

    // Head away from the enemy.
    if (nearestEnemyDist < mAvoidanceDistance)
    {
        Math::Vec3 desiredMoveAdj =
            safeNormalize(entity.position() - nearestEnemy.position()) * mAvoidanceSpeed;
        Math::Vec3 currentDesiredMove = entity.desiredMove();
        currentDesiredMove += desiredMoveAdj * gain();
        entity.setDesiredMove(currentDesiredMove);
    }
}

// --- Cruising ---------------------------------------------------------------

CruisingBehavior::CruisingBehavior(float randMoveXChance, float randMoveYChance,
                                   float randMoveZChance, float minRandomMove, float maxRateChange,
                                   float minRateChange)
    : mRandMoveXChance(randMoveXChance), mRandMoveYChance(randMoveYChance),
      mRandMoveZChance(randMoveZChance), mMinRandomMove(minRandomMove),
      mMaxRateChange(maxRateChange), mMinRateChange(minRateChange)
{
}

void CruisingBehavior::iterate(float timeDelta, Entity& entity)
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
    Math::Vec3 desiredMoveAdj(0.0f);
    float randmove = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
    if (randmove < mRandMoveXChance)
        desiredMoveAdj.x += mMinRandomMove * signum;
    else if (randmove < mRandMoveXChance + mRandMoveYChance)
        desiredMoveAdj.y += mMinRandomMove * signum;
    else if (randmove < mRandMoveXChance + mRandMoveYChance + mRandMoveZChance)
        desiredMoveAdj.z += mMinRandomMove * signum;

    Math::Vec3 currentDesiredMove = entity.desiredMove();
    desiredMoveAdj = safeNormalize(desiredMoveAdj) * (mMinRateChange * signum);
    currentDesiredMove += desiredMoveAdj * gain();
    entity.setDesiredMove(currentDesiredMove);
}

// --- Stay Within Sphere -----------------------------------------------------

StayWithinSphereBehavior::StayWithinSphereBehavior(const Math::Vec3& center, float radius)
    : mCenter(center), mRadius(radius)
{
}

void StayWithinSphereBehavior::iterate(float timeDelta, Entity& entity)
{
    (void)timeDelta;

    Math::Vec3 toCenter = mCenter - entity.position();
    float dist = glm::length(toCenter);
    if (dist > mRadius)
    {
        Math::Vec3 desiredMoveAdj = safeNormalize(toCenter) * entity.maxSpeed();
        Math::Vec3 currentDesiredMove = entity.desiredMove();
        currentDesiredMove += desiredMoveAdj * gain();
        entity.setDesiredMove(currentDesiredMove);
    }
}

// --- Combat -------------------------------------------------------------

CombatBehavior::CombatBehavior(float fireRange, float damagePerHit, float fireInterval)
    : mFireRange(fireRange), mDamagePerHit(damagePerHit), mFireInterval(fireInterval)
{
}

void CombatBehavior::iterate(float timeDelta, Entity& entity)
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
