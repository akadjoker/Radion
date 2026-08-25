// Entity.cpp - agent update, sensing and steering.
//
// updateEnemyVisibility() skips non-enemy entities (continue) so a group
// mixing friendlies and enemies still exposes the enemies instead of stopping
// the scan at the first friendly.

#include "PCH.h"

#include "Entity.h"

#include "Behavior.h"
#include "Group.h"
#include "World.h"

namespace Radion::AI
{

namespace
{
void insertSortedDist(std::vector<EntityDist>& list, Entity* entity, float distance)
{
    EntityDist entry{distance, entity};
    auto pos = std::upper_bound(list.begin(), list.end(), entry,
                                [](const EntityDist& a, const EntityDist& b)
                                {
                                    return a.distance < b.distance;
                                });
    list.insert(pos, entry);
}
} // namespace

Entity::Entity(World& world, const Settings& settings)
    : mWorld(world), mFriendMask(settings.type), mEnemyMask(~settings.type),
      mEntityType(settings.type), mSenseRange(settings.senseRange),
      mMaxVelocityChange(settings.maxVelocityChange), mMaxSpeed(settings.maxSpeed),
      mDesiredSpeed(settings.desiredSpeed), mRadius(settings.radius),
      mMoveXScalar(settings.moveXScalar), mMoveYScalar(settings.moveYScalar),
      mMoveZScalar(settings.moveZScalar)
{
}

void Entity::setSpeed(float newSpeed)
{
    float spd = glm::length(mVelocity);
    if (spd > 0.0f)
        mVelocity *= newSpeed / spd;
    else
        mVelocity = forward() * newSpeed;
}

void Entity::alignWithVelocity()
{
    float spd = glm::length(mVelocity);
    // A direction from a near-zero velocity is numerical noise. Updating the
    // orientation from it makes formation goals rotate while the squad is at
    // rest, which in turn makes the debug path visibly oscillate.
    if (spd <= 0.1f)
        return;

    // Standard right-handed orthonormal regeneration:
    // forward = velocity, side = normalize(cross(up, forward)), up = cross(forward, side).
    glm::vec3 newForward = mVelocity / spd;
    glm::vec3 oldUp = up();
    glm::vec3 sideReference = oldUp;
    // A forward vector parallel to up has no valid cross product.  Keep the
    // previous side (projected onto the plane perpendicular to forward) so a
    // vertical/near-vertical velocity cannot poison the quaternion with NaNs.
    glm::vec3 newSide = glm::cross(sideReference, newForward);
    if (glm::dot(newSide, newSide) <= 1e-8f)
        newSide = side();
    newSide -= newForward * glm::dot(newSide, newForward);
    if (glm::dot(newSide, newSide) <= 1e-8f)
        newSide = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), newForward);
    newSide = glm::normalize(newSide);
    glm::vec3 newUp = glm::cross(newForward, newSide);

    mOrientation = glm::quat_cast(glm::mat3(newSide, newUp, newForward));
}

glm::vec3 Entity::localizeDirection(const glm::vec3& globalDirection) const
{
    const glm::mat3 basis(side(), up(), forward());
    return glm::transpose(basis) * globalDirection;
}

glm::vec3 Entity::localizePosition(const glm::vec3& globalPosition) const
{
    const glm::mat3 basis(side(), up(), forward());
    return glm::transpose(basis) * (globalPosition - mPosition);
}

glm::vec3 Entity::globalizePosition(const glm::vec3& localPosition) const
{
    const glm::mat3 basis(side(), up(), forward());
    return mPosition + (basis * localPosition);
}

glm::vec3 Entity::globalizeDirection(const glm::vec3& localDirection) const
{
    const glm::mat3 basis(side(), up(), forward());
    return basis * localDirection;
}

void Entity::iterate(float timeDelta)
{
    // A dead entity is frozen exactly where it fell - no sensing, no
    // behaviors, no movement - rather than removed from its Group. Removal
    // would invalidate Group::entities() iterators mid-World::iterate() (a
    // dying entity is discovered from inside another entity's own
    // CombatBehavior pass) and would also drop it from the scene the caller
    // still needs to look up to hide/pose its GameObject.
    if (!alive())
    {
        mVelocity = glm::vec3(0.0f);
        mDesiredMoveVector = glm::vec3(0.0f);
        return;
    }

    mPosition += mVelocity * timeDelta;

    // The caller polls firedThisFrame() right after World::iterate() - clear
    // it here, once, so a shot fired this pass by CombatBehavior below stays
    // visible for exactly one frame.
    mFiredThisFrame = false;
    mLastFireTarget = nullptr;

    // Behaviors contribute to a per-frame steering accumulator.  Keeping the
    // previous value makes acceleration compound forever and is especially
    // visible as oscillating turns in formations.
    mDesiredMoveVector = glm::vec3(0.0f);

    // Refresh sense data.
    mVisibleGroupMembers.clear();
    mVisibleEnemies.clear();
    updateGroupVisibility();
    updateEnemyVisibility();

    // Let behaviors accumulate their desired-move contributions.
    for (Behavior* behavior : mBehaviors)
        behavior->iterate(timeDelta, *this);

    // Clamp the desired move to the maximum velocity change (acceleration).
    float velChange = glm::length(mDesiredMoveVector);
    if (velChange > mMaxVelocityChange && velChange > 0.0f)
        mDesiredMoveVector = glm::normalize(mDesiredMoveVector) * mMaxVelocityChange;

    // Apply the change.
    mVelocity += mDesiredMoveVector;

    // Per-axis scaling (restrict movement; > 1.0f destabilises the system).
    mVelocity.x *= mMoveXScalar;
    mVelocity.y *= mMoveYScalar;
    mVelocity.z *= mMoveZScalar;

    // Clamp the actual velocity to max speed.
    float speed = glm::length(mVelocity);
    if (speed > mMaxSpeed && speed > 0.0f)
        mVelocity = glm::normalize(mVelocity) * mMaxSpeed;

    // Snap tiny residual velocities to rest. Without this dead zone an agent
    // that has reached a formation slot keeps moving by sub-pixel amounts;
    // those changes are especially visible when a debug path is aligned with
    // the camera.
    if (glm::length(mVelocity) < 0.1f)
        mVelocity = glm::vec3(0.0f);
}

void Entity::updateGroupVisibility()
{
    if (!mCurrentGroup)
        return;

    for (Entity* other : mCurrentGroup->entities())
    {
        if (other == this || !other->alive())
            continue;

        float dist = 0.0f;
        if (visibilityTest(*other, dist))
            insertSortedDist(mVisibleGroupMembers, other, dist);
    }
}

void Entity::updateEnemyVisibility()
{
    for (Group* group : mWorld.groups())
    {
        for (Entity* other : group->entities())
        {
            if (!other->alive())
                continue;
            if ((other->type() & enemyMask()) == 0)
                continue; // friendly

            float dist = 0.0f;
            if (visibilityTest(*other, dist))
                insertSortedDist(mVisibleEnemies, other, dist);
        }
    }
}

bool Entity::visibilityTest(const Entity& other, float& dist) const
{
    glm::vec3 distVec = other.position() - position();
    dist = glm::length(distVec);
    return dist < mSenseRange;
}

void Entity::addBehavior(Behavior& behavior)
{
    mBehaviors.push_back(&behavior);
}

void Entity::removeBehavior(Behavior& behavior)
{
    auto it = std::find(mBehaviors.begin(), mBehaviors.end(), &behavior);
    if (it != mBehaviors.end())
        mBehaviors.erase(it);
}

} // namespace Radion::AI
