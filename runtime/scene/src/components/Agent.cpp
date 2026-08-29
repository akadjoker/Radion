// Agent.cpp - port of AI::Entity::iterate()/sensing (Entity.cpp) and
// AI::SquadEntity/AI::SquadLeaderEntity (SquadEntity.cpp), folded into one
// Component the Scene drives directly instead of a World/Group tree.

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
    // A loose agent outliving nothing but itself (a test local never
    // attached to a GameObject, or one detached mid-scene) would otherwise
    // leave the Scene holding a pointer to freed memory - the same bug
    // radion-fisica-destrutor-desregista-scene already had for RigidBody.
    if (mScene)
        mScene->removeAgent(*this);
    // Both ends of the squad link, for the same reason: a destroyed member
    // still listed by its leader, or a leader still pointed at by its
    // members, is a pointer into freed memory that the next order follows.
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
    f32 spd = glm::length(mVelocity);
    if (spd > 0.0f)
        mVelocity *= newSpeed / spd;
    else
        mVelocity = forward() * newSpeed;
}

void Agent::alignWithVelocity()
{
    f32 spd = glm::length(mVelocity);
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

glm::vec3 Agent::localizeDirection(const glm::vec3& globalDirection) const
{
    const glm::mat3 basis(side(), up(), forward());
    return glm::transpose(basis) * globalDirection;
}

glm::vec3 Agent::localizePosition(const glm::vec3& globalPosition) const
{
    const glm::mat3 basis(side(), up(), forward());
    return glm::transpose(basis) * (globalPosition - mPosition);
}

glm::vec3 Agent::globalizePosition(const glm::vec3& localPosition) const
{
    const glm::mat3 basis(side(), up(), forward());
    return mPosition + (basis * localPosition);
}

glm::vec3 Agent::globalizeDirection(const glm::vec3& localDirection) const
{
    const glm::mat3 basis(side(), up(), forward());
    return basis * localDirection;
}

void Agent::update(f32 deltaTime)
{
    // AI::SquadEntity::iterate() (removed) stepped its state machine before
    // the steering/physics pass; Agent::update() is now the single entry
    // point for every role, so the state machine (if any) still goes first,
    // even ahead of the alive() check below (SquadEntity.cpp:33-39 ran it
    // unconditionally too).
    if (mStateMachine)
        mStateMachine->iterate();

    // A dead agent is frozen exactly where it fell - no sensing, no
    // behaviors, no movement - rather than removed from the Scene's list.
    // Removal would invalidate Scene::agents() iterators mid-update (a dying
    // agent is discovered from inside another agent's own CombatBehavior
    // pass) and would also drop it from the scene the caller still needs to
    // look up to hide/pose its GameObject.
    if (!alive())
    {
        mVelocity = glm::vec3(0.0f);
        mDesiredMoveVector = glm::vec3(0.0f);
        return;
    }

    mPosition += mVelocity * deltaTime;

    // The caller polls firedThisFrame() right after Scene::update() - clear
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
    updateVisibility();

    // Let behaviors accumulate their desired-move contributions.
    for (AI::Behavior* behavior : mBehaviors)
        behavior->iterate(deltaTime, *this);

    // Clamp the desired move to the maximum velocity change (acceleration).
    f32 velChange = glm::length(mDesiredMoveVector);
    if (velChange > mMaxVelocityChange && velChange > 0.0f)
        mDesiredMoveVector = glm::normalize(mDesiredMoveVector) * mMaxVelocityChange;

    // Apply the change.
    mVelocity += mDesiredMoveVector;

    // Per-axis scaling (restrict movement; > 1.0f destabilises the system).
    mVelocity.x *= mMoveXScalar;
    mVelocity.y *= mMoveYScalar;
    mVelocity.z *= mMoveZScalar;

    // Clamp the actual velocity to max speed.
    f32 spd = glm::length(mVelocity);
    if (spd > mMaxSpeed && spd > 0.0f)
        mVelocity = glm::normalize(mVelocity) * mMaxSpeed;

    // Snap tiny residual velocities to rest. Without this dead zone an agent
    // that has reached a formation slot keeps moving by sub-pixel amounts;
    // those changes are especially visible when a debug path is aligned with
    // the camera.
    if (glm::length(mVelocity) < 0.1f)
        mVelocity = glm::vec3(0.0f);
}

void Agent::updateVisibility()
{
    // AI::Entity::updateGroupVisibility()/updateEnemyVisibility() (removed)
    // ran as two separate scans: one over the current Group's members, one
    // over every Group in the World. Without Group/World, the Scene keeps a
    // flat agent list, so this is one scan preserving both independent
    // tests - an agent already sensed as a group member can also land in
    // mVisibleEnemies if its type matches enemyMask(), as before
    // (updateEnemyVisibility() looped every group in the world, its own
    // included).
    //
    // One deliberate difference: the enemy scan now skips `this`. It did not
    // before, so an agent whose friendMask() was set to something that does
    // not cover its own type sensed ITSELF as the nearest enemy, at distance
    // zero, and CombatBehavior shot it. Unreachable with the default masks
    // (enemyMask is ~type), which is why it survived.
    if (!mScene)
        return;

    for (Agent* other : mScene->agents())
    {
        // Null while an agent is being destroyed mid-update - see
        // Scene::agents().
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
    glm::vec3 distVec = other.position() - position();
    dist = glm::length(distVec);
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

    // Behaviors used to be non-owning and were routinely shared by a whole
    // flock; now that the agent deletes them, the same instance reaching two
    // agents (or the same one twice) is a double free. Rejected the way
    // Group::add() rejected an entity that already had a group.
    //
    // Refused, never deleted: this one already has an owner, so the memory
    // is not ours to free - the agent that does own it still will. Only the
    // caller holding an unowned behavior has something that could be
    // stranded, and addBehavior<T>() closes that by never handing one out.
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

// ---- squad-member state (AI::SquadEntity) ----------------------------------

bool Agent::waypointReached()
{
    if (mNextWaypoint != 0 && mWaypointNetwork)
    {
        AI::Waypoint* wp = mWaypointNetwork->findWaypoint(mNextWaypoint);
        if (wp)
        {
            glm::vec3 vec = wp->position() - position();
            vec.y = 0.0f; // XZ only, matching the reference demo
            f32 distToWP = glm::length(vec);
            return (distToWP - kEntityRadius) < wp->radius();
        }
    }
    return false;
}

void Agent::onWaypointReached()
{
    // The reference read the NPC weapon status out of the waypoint's editor
    // blind data here (IWF export); that data is not carried over, so the
    // hook is intentionally empty for engines to fill in.
}

bool Agent::goalReached()
{
    glm::vec3 vec = mGoalPosition - position();
    vec.y = 0.0f; // XZ only
    return glm::length(vec) < mGoalRadius;
}

void Agent::onGoalReached()
{
}

void Agent::onWaitingForCommand()
{
}

void Agent::setCommand(AI::SquadCommand command)
{
    // AI::SquadLeaderEntity::setCommand() overrode AI::SquadEntity::setCommand()
    // to redistribute the command to every member; there is only one class
    // now, so the leader path is picked by squadId() == 0 instead of a
    // vtable. mLastCommand/mCommandAcknowledged are only ever read through
    // hasCommandChanged(), which only the leader's state machine calls
    // (SquadAI.cpp), so leaving them updated on non-leaders is harmless.
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

// ---- leader-only state (AI::SquadLeaderEntity) -----------------------------

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
    // The null check used to sit on the line below, one dereference too
    // late: a leader with members but no network crashed here instead of
    // returning.
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
}

void Agent::removeSquadMember(Agent* member)
{
    auto it = std::find(mSquadMembers.begin(), mSquadMembers.end(), member);
    if (it == mSquadMembers.end())
        return;
    (*it)->mSquadLeader = nullptr;
    mSquadMembers.erase(it);
}

void Agent::clearSquadMembers()
{
    for (Agent* member : mSquadMembers)
        member->mSquadLeader = nullptr;
    mSquadMembers.clear();
}

// ---- pose sync with the owning GameObject ----------------------------------
//
// The AI's local frame uses forward = +Z (Agent::forward()); GameObject::
// forward() is rotation * (0,0,-1), forward = -Z. Copying the orientation
// straight across would face the owner 180 degrees off, so both directions
// of the sync apply a 180 degree turn around up() - its own inverse, so the
// same expression undoes itself on the way back. Accepted consequence: the
// owner's right() ends up the negation of Agent::side() - a rotation cannot
// align forward without also flipping right; only a reflection could, and
// this is not one.

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
    // mSynced* is the record of where the owner was last seen, so it is
    // refreshed whether or not the agent takes the pose - left behind under
    // a disabled sync flag, ownerMoved() would answer true every frame for
    // the rest of the object's life and call this on every one of them.
    mSyncedPosition = object->globalPosition();
    mSyncedRotation = object->globalRotation();
    if (mSyncPosition)
        mPosition = mSyncedPosition;
    if (mSyncRotation)
        mOrientation =
            mSyncedRotation * glm::angleAxis(glm::pi<f32>(), glm::vec3(0.0f, 1.0f, 0.0f));
}

bool Agent::ownerMoved() const
{
    const GameObject* object = owner();
    if (!object)
        return false;
    const glm::vec3 delta = object->globalPosition() - mSyncedPosition;
    if (glm::dot(delta, delta) > kPoseSyncEpsilon * kPoseSyncEpsilon)
        return true;
    const f32 alignment = glm::abs(glm::dot(object->globalRotation(), mSyncedRotation));
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
                                  glm::angleAxis(glm::pi<f32>(), glm::vec3(0.0f, 1.0f, 0.0f)));
    mSyncedPosition = object->globalPosition();
    mSyncedRotation = object->globalRotation();
}

} // namespace Radion
