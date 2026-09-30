#include "PCH.h"

#include "Agent.h"

#include "Behavior.h"
#include "GameObject.h"
#include "Log.h"
#include "PointOfInterest.h"
#include "Scene.h"
#include "StateMachine.h"
#include "Waypoint.h"
#include "WaypointNetwork.h"

namespace Radion
{

namespace
{
constexpr f32 kPoseSyncEpsilon = 1e-5f;
constexpr f32 kEntityRadius = 0.75f; // reference ENTITY_RADIUS (SquadEntity.cpp)

void insertSortedDist(std::vector<EntityDist>& list, Agent* entity, f32 distance)
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

Agent::Agent() : Component(Type)
{
}

Agent::~Agent()
{
    // A loose agent outliving its Scene registration would leave the Scene holding a freed pointer (same bug RigidBody had).
    if (mScene)
        mScene->removeAgent(*this);
    // Both ends of the squad link, or a destroyed member/leader leaves a freed pointer for the next order.
    if (mSquadLeader)
        mSquadLeader->removeSquadMember(this);
    clearSquadMembers();
    clearBehaviors();
    delete mStateMachine;
}

void Agent::applySettings(const Settings& settings)
{
    mAgentType = settings.type;
    mFriendMask = settings.type;
    mEnemyMask = ~settings.type;
    mSenseRange = settings.senseRange;
    mMaxVelocityChange = settings.maxVelocityChange;
    mMaxSpeed = settings.maxSpeed;
    mDesiredSpeed = settings.desiredSpeed;
    mRadius = settings.radius;
    mMoveXScalar = settings.moveXScalar;
    mMoveYScalar = settings.moveYScalar;
    mMoveZScalar = settings.moveZScalar;
}

void Agent::setSpeed(f32 newSpeed)
{
    f32 spd = Math::length(mVelocity);
    if (spd > 0.0f)
        mVelocity *= newSpeed / spd;
    else
        mVelocity = forward() * newSpeed;
}

void Agent::alignWithVelocity()
{
    f32 spd = Math::length(mVelocity);
    // Direction from near-zero velocity is noise; it would rotate formation goals at rest and make the debug path oscillate.
    if (spd <= 0.1f)
        return;

    // Right-handed orthonormal regeneration: forward = velocity, side = normalize(cross(up, forward)), up = cross(forward, side).
    Math::vec3 newForward = mVelocity / spd;
    Math::vec3 oldUp = up();
    Math::vec3 sideReference = oldUp;
    // A forward parallel to up has no valid cross product: keep the previous side (projected perpendicular to forward) so a vertical velocity cannot produce NaN quaternions.
    Math::vec3 newSide = Math::cross(sideReference, newForward);
    if (Math::dot(newSide, newSide) <= 1e-8f)
        newSide = side();
    newSide -= newForward * Math::dot(newSide, newForward);
    if (Math::dot(newSide, newSide) <= 1e-8f)
        newSide = Math::cross(Math::vec3(0.0f, 1.0f, 0.0f), newForward);
    newSide = Math::normalize(newSide);
    Math::vec3 newUp = Math::cross(newForward, newSide);

    mOrientation = Math::quat_cast(Math::mat3(newSide, newUp, newForward));
}

Math::vec3 Agent::localizeDirection(const Math::vec3& globalDirection) const
{
    const Math::mat3 basis(side(), up(), forward());
    return Math::transpose(basis) * globalDirection;
}

Math::vec3 Agent::localizePosition(const Math::vec3& globalPosition) const
{
    const Math::mat3 basis(side(), up(), forward());
    return Math::transpose(basis) * (globalPosition - mPosition);
}

Math::vec3 Agent::globalizePosition(const Math::vec3& localPosition) const
{
    const Math::mat3 basis(side(), up(), forward());
    return mPosition + (basis * localPosition);
}

Math::vec3 Agent::globalizeDirection(const Math::vec3& localDirection) const
{
    const Math::mat3 basis(side(), up(), forward());
    return basis * localDirection;
}

void Agent::update(f32 deltaTime)
{
    // The state machine steps first, even before the alive() check (as SquadEntity did).
    if (mStateMachine)
        mStateMachine->iterate();

    // A dead agent freezes where it fell rather than being removed: removal would invalidate Scene::agents() iterators mid-update and drop it from the scene the caller needs to look up.
    if (!alive())
    {
        mVelocity = Math::vec3(0.0f);
        mDesiredMoveVector = Math::vec3(0.0f);
        return;
    }

    mPosition += mVelocity * deltaTime;

    // Cleared once here so a shot fired this pass stays visible for exactly one frame.
    mFiredThisFrame = false;
    mLastFireTarget = nullptr;

    // Behaviors accumulate into a per-frame vector; keeping the old one makes acceleration compound and turns oscillate.
    mDesiredMoveVector = Math::vec3(0.0f);

    mVisibleGroupMembers.clear();
    mVisibleEnemies.clear();
    updateVisibility();

    for (AI::Behavior* behavior : mBehaviors)
        behavior->iterate(deltaTime, *this);

    f32 velChange = Math::length(mDesiredMoveVector);
    if (velChange > mMaxVelocityChange && velChange > 0.0f)
        mDesiredMoveVector = Math::normalize(mDesiredMoveVector) * mMaxVelocityChange;

    mVelocity += mDesiredMoveVector;

    // > 1.0f destabilises the system.
    mVelocity.x *= mMoveXScalar;
    mVelocity.y *= mMoveYScalar;
    mVelocity.z *= mMoveZScalar;

    f32 spd = Math::length(mVelocity);
    if (spd > mMaxSpeed && spd > 0.0f)
        mVelocity = Math::normalize(mVelocity) * mMaxSpeed;

    // Dead zone: an agent at its formation slot otherwise keeps moving by sub-pixel amounts, visible when a debug path aligns with the camera.
    if (Math::length(mVelocity) < 0.1f)
        mVelocity = Math::vec3(0.0f);

    // Forward must track velocity: every "ahead" test (avoidance, isAhead/isAside/isBehind, pursuit) uses the local frame and would otherwise use the spawn facing forever.
    alignWithVelocity();
}

void Agent::updateVisibility()
{
    // One flat scan preserving both tests: an agent sensed as a group member can also land in mVisibleEnemies if its type matches enemyMask().
    // The enemy scan skips `this`: with a friendMask not covering its own type an agent once sensed itself as the nearest enemy and shot itself.
    if (!mScene)
        return;

    for (Agent* other : mScene->agents())
    {
        // Null while an agent is destroyed mid-update (see Scene::agents()).
        if (!other || other == this || !other->alive())
            continue;

        f32 dist = 0.0f;
        if (!visibilityTest(*other, dist))
            continue;

        if (mGroupId != 0 && other->groupId() == mGroupId)
            insertSortedDist(mVisibleGroupMembers, other, dist);
        if ((other->type() & enemyMask()) != 0)
            insertSortedDist(mVisibleEnemies, other, dist);
    }
}

bool Agent::visibilityTest(const Agent& other, f32& dist) const
{
    Math::vec3 distVec = other.position() - position();
    dist = Math::length(distVec);
    return dist < mSenseRange;
}

AI::Behavior* Agent::addBehavior(AI::BehaviorType type)
{
    AI::Behavior* behavior = AI::BehaviorFactory::create(type);
    if (!behavior)
        return nullptr;
    if (!adoptBehavior(behavior))
        return nullptr;
    return behavior;
}

bool Agent::adoptBehavior(AI::Behavior* behavior)
{
    if (!behavior)
        return false;

    // Behaviors are owned by the agent, so one reaching two agents (or the same one twice) is a double free; refused, never deleted, since the other owner still frees it.
    if (behavior->owner())
    {
        Log::warning("Agent: a behavior already owned by an agent cannot be added to a second "
                     "one; ignored");
        return false;
    }
    behavior->mOwner = this;
    mBehaviors.push_back(behavior);
    return true;
}

bool Agent::removeBehavior(AI::Behavior& behavior)
{
    auto it = std::find(mBehaviors.begin(), mBehaviors.end(), &behavior);
    if (it == mBehaviors.end())
        return false;
    delete *it;
    mBehaviors.erase(it);
    return true;
}

bool Agent::removeBehavior(AI::BehaviorType type)
{
    for (usize i = 0; i < mBehaviors.size(); ++i)
    {
        if (mBehaviors[i]->type() != type)
            continue;
        delete mBehaviors[i];
        mBehaviors.erase(mBehaviors.begin() + static_cast<std::ptrdiff_t>(i));
        return true;
    }
    return false;
}

AI::Behavior* Agent::behavior(AI::BehaviorType type) const
{
    for (AI::Behavior* behavior : mBehaviors)
        if (behavior->type() == type)
            return behavior;
    return nullptr;
}

void Agent::clearBehaviors()
{
    for (AI::Behavior* behavior : mBehaviors)
        delete behavior;
    mBehaviors.clear();
}

usize Agent::behaviorCount() const
{
    return mBehaviors.size();
}

AI::Behavior* Agent::behaviorAt(usize index) const
{
    return mBehaviors[index];
}

void Agent::setStateMachine(AI::StateMachine* machine)
{
    if (mStateMachine == machine)
        return;
    delete mStateMachine;
    mStateMachine = machine;
}

bool Agent::waypointReached()
{
    if (mNextWaypoint != 0 && mWaypointNetwork)
    {
        AI::Waypoint* wp = mWaypointNetwork->findWaypoint(mNextWaypoint);
        if (wp)
        {
            Math::vec3 vec = wp->position() - position();
            vec.y = 0.0f; // XZ only, matching the reference demo
            f32 distToWP = Math::length(vec);
            return (distToWP - kEntityRadius) < wp->radius();
        }
    }
    return false;
}

void Agent::onWaypointReached()
{
    // The reference read the NPC weapon status from the waypoint's editor blind data (IWF export); not carried over, so the hook is intentionally empty.
}

bool Agent::goalReached()
{
    Math::vec3 vec = mGoalPosition - position();
    vec.y = 0.0f; // XZ only
    return Math::length(vec) < mGoalRadius;
}

void Agent::onGoalReached()
{
}

void Agent::onWaitingForCommand()
{
}

void Agent::setCommand(AI::SquadCommand command)
{
    // The leader path is picked by squadId() == 0. mLastCommand/mCommandAcknowledged are only read via hasCommandChanged(), which only the leader's state machine calls, so updating them on non-leaders is harmless.
    if (mSquadId == 0)
        mLastCommand = mCommand; // the command we are leaving

    mCommand = command;
    if (mCommand == AI::SquadCommand::StandGround)
    {
        setGoal(position());
        setNextWaypoint(0);
        mPath.clear();
    }

    if (mSquadId != 0)
        return;

    if (command == AI::SquadCommand::AttackTarget)
        sendSquadToTarget();
    else
        for (Agent* member : mSquadMembers)
            member->setCommand(command);
}

void Agent::sendSquadToTarget()
{
    if (mSquadMembers.empty())
        return;
    AI::PointOfInterest* poi = mSelectedPointOfInterest;
    if (!poi)
        return;

    AI::Path emptyPath;
    for (Agent* member : mSquadMembers)
    {
        member->setNextWaypoint(0);
        member->setPath(emptyPath);
        member->setGoal(poi->position());
    }
}

void Agent::sendSquadToRandomPOI()
{
    if (!mPointsOfInterest || mSquadMembers.empty())
        return;

    AI::PointOfInterest* closest = mPointsOfInterest->findNearest(mSquadMembers[0]->goal());
    AI::PointOfInterest* poi = mPointsOfInterest->selectRandom(closest ? closest->id() : 0);
    if (!poi)
        return;
    mSelectedPointOfInterest = poi;

    AI::Path emptyPath;
    for (Agent* member : mSquadMembers)
    {
        member->setNextWaypoint(0);
        member->setPath(emptyPath);
        member->setGoal(poi->position());
    }
}

void Agent::sendSquadToRandomWaypoint()
{
    // The null check must come before the dereference: a leader with members but no network would crash.
    if (mSquadMembers.empty() || !mWaypointNetwork)
        return;

    const AI::WaypointID wpID = AI::selectRandomWaypoint(*mWaypointNetwork);
    AI::Waypoint* wp = mWaypointNetwork->findWaypoint(wpID);
    if (!wp)
        return;
    mSelectedWaypoint = wp;

    AI::Path emptyPath;
    for (Agent* member : mSquadMembers)
    {
        member->setNextWaypoint(0);
        member->setPath(emptyPath);
        member->setGoal(wp->position());
    }
}

void Agent::commandSquadToRallyOnLeader()
{
    if (mSquadMembers.empty())
        return;

    AI::Path emptyPath;
    for (Agent* member : mSquadMembers)
    {
        member->setNextWaypoint(0);
        member->setPath(emptyPath);
        member->setGoal(position());
    }
}

void Agent::addSquadMember(Agent* member)
{
    if (!member || member == this || member->mSquadLeader == this)
        return;
    if (member->mSquadLeader)
        member->mSquadLeader->removeSquadMember(member);
    member->mSquadLeader = this;
    mSquadMembers.push_back(member);

    // Lowest free slot, not mSquadMembers.size() (which would hand out an id a survivor still holds after a member left). Leader is 0, first member 1, -1 = never assigned; a preassigned slot is kept.
    if (member->mSquadId < 0)
    {
        int slot = 1;
        for (;;)
        {
            bool taken = false;
            for (Agent* other : mSquadMembers)
            {
                if (other != member && other->mSquadId == slot)
                {
                    taken = true;
                    break;
                }
            }
            if (!taken)
                break;
            ++slot;
        }
        member->mSquadId = slot;
    }
}

void Agent::removeSquadMember(Agent* member)
{
    auto it = std::find(mSquadMembers.begin(), mSquadMembers.end(), member);
    if (it == mSquadMembers.end())
        return;
    (*it)->mSquadLeader = nullptr;
    // Give up the slot, or it travels with the member into a later squad and collides with its holder.
    (*it)->mSquadId = -1;
    mSquadMembers.erase(it);
}

void Agent::clearSquadMembers()
{
    for (Agent* member : mSquadMembers)
    {
        member->mSquadLeader = nullptr;
        member->mSquadId = -1;
    }
    mSquadMembers.clear();
}

// The AI frame has forward = +Z, GameObject::forward() is -Z, so both sync directions apply a 180 degree turn around up() (its own inverse).
// Consequence: the owner's right() ends up the negation of Agent::side(); only a reflection could avoid that.

bool Agent::simulating() const
{
    const GameObject* object = owner();
    return active() && (!object || (object->isActiveInHierarchy() && !object->disposed()));
}

void Agent::pushOwnerPose()
{
    GameObject* object = owner();
    if (!object)
        return;
    // mSynced* is the last-seen owner pose and is refreshed even when sync is disabled, else ownerMoved() answers true every frame.
    mSyncedPosition = object->globalPosition();
    mSyncedRotation = object->globalRotation();
    if (mSyncPosition)
        mPosition = mSyncedPosition;
    if (mSyncRotation)
        mOrientation =
            mSyncedRotation * Math::angleAxis(Math::pi<f32>(), Math::vec3(0.0f, 1.0f, 0.0f));
}

bool Agent::ownerMoved() const
{
    const GameObject* object = owner();
    if (!object)
        return false;
    const Math::vec3 delta = object->globalPosition() - mSyncedPosition;
    if (Math::dot(delta, delta) > kPoseSyncEpsilon * kPoseSyncEpsilon)
        return true;
    const f32 alignment = Math::abs(Math::dot(object->globalRotation(), mSyncedRotation));
    return alignment < 1.0f - kPoseSyncEpsilon;
}

void Agent::pullAgentPose()
{
    GameObject* object = owner();
    if (!object)
        return;
    if (mSyncPosition)
        object->setGlobalPosition(mPosition);
    if (mSyncRotation)
        object->setGlobalRotation(mOrientation *
                                  Math::angleAxis(Math::pi<f32>(), Math::vec3(0.0f, 1.0f, 0.0f)));
    mSyncedPosition = object->globalPosition();
    mSyncedRotation = object->globalRotation();
}

} // namespace Radion
