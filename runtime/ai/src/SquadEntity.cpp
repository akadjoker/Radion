// SquadEntity.cpp - squad entity and leader/member implementations.

#include "PCH.h"

#include "SquadEntity.h"

#include "PointOfInterest.h"
#include "StateMachine.h"
#include "WaypointNetwork.h"

#include <cstdlib>

namespace Radion::AI
{

WaypointID selectRandomWaypoint(const WaypointNetwork& network)
{
    const auto& waypoints = network.waypoints();
    if (waypoints.empty())
        return 0;

    std::vector<WaypointID> ids;
    ids.reserve(waypoints.size());
    for (const auto& kv : waypoints)
        ids.push_back(kv.first);
    return ids[std::rand() % static_cast<int>(ids.size())];
}

SquadEntity::SquadEntity(World& world, const Settings& settings) : Entity(world, settings)
{
}

void SquadEntity::iterate(float timeDelta)
{
    // Step the state machine first, then the steering/physics.
    if (mStateMachine)
        mStateMachine->iterate();
    Entity::iterate(timeDelta);
}

namespace
{
constexpr float kEntityRadius = 0.75f; // reference ENTITY_RADIUS
} // namespace

bool SquadEntity::waypointReached()
{
    if (mNextWaypoint != 0 && mWaypointNetwork)
    {
        Waypoint* wp = mWaypointNetwork->findWaypoint(mNextWaypoint);
        if (wp)
        {
            Math::Vec3 vec = wp->position() - position();
            vec.y = 0.0f; // XZ only, matching the reference demo
            float distToWP = glm::length(vec);
            return (distToWP - kEntityRadius) < wp->radius();
        }
    }
    return false;
}

void SquadEntity::onWaypointReached()
{
    // The reference read the NPC weapon status out of the waypoint's editor
    // blind data here (IWF export); that data is not carried over, so the
    // hook is intentionally empty for engines to fill in.
}

bool SquadEntity::goalReached()
{
    Math::Vec3 vec = mGoalPosition - position();
    vec.y = 0.0f; // XZ only
    return glm::length(vec) < mGoalRadius;
}

void SquadEntity::onGoalReached()
{
}

void SquadEntity::onWaitingForCommand()
{
}

void SquadEntity::setCommand(SquadCommand command)
{
    mCommand = command;
    if (mCommand == SquadCommand::StandGround)
    {
        setGoal(position());
        setNextWaypoint(0);
        mPath.clear();
    }
}

SquadLeaderEntity::SquadLeaderEntity(World& world, const Settings& settings)
    : SquadEntity(world, settings)
{
}

void SquadLeaderEntity::setCommand(SquadCommand command)
{
    mLastCommand = this->command(); // the command we are leaving
    SquadEntity::setCommand(command);

    if (command == SquadCommand::AttackTarget)
        sendSquadToTarget();
    else
    {
        for (SquadEntity* member : mSquadMembers)
            member->setCommand(command);
    }
}

void SquadLeaderEntity::sendSquadToTarget()
{
    if (mSquadMembers.empty())
        return;
    PointOfInterest* poi = mSelectedPointOfInterest;
    if (!poi)
        return;

    Path emptyPath;
    for (SquadEntity* member : mSquadMembers)
    {
        member->setNextWaypoint(0);
        member->setPath(emptyPath);
        member->setGoal(poi->position());
    }
}

void SquadLeaderEntity::sendSquadToRandomPOI()
{
    if (!mPointsOfInterest || mSquadMembers.empty())
        return;

    PointOfInterest* closest = mPointsOfInterest->findNearest(mSquadMembers[0]->goal());
    PointOfInterest* poi = mPointsOfInterest->selectRandom(closest ? closest->id() : 0);
    if (!poi)
        return;
    mSelectedPointOfInterest = poi;

    Path emptyPath;
    for (SquadEntity* member : mSquadMembers)
    {
        member->setNextWaypoint(0);
        member->setPath(emptyPath);
        member->setGoal(poi->position());
    }
}

void SquadLeaderEntity::sendSquadToRandomWaypoint()
{
    if (mSquadMembers.empty())
        return;

    WaypointID wpID = selectRandomWaypoint(*mWaypointNetwork);
    Waypoint* wp = mWaypointNetwork ? mWaypointNetwork->findWaypoint(wpID) : nullptr;
    if (!wp)
        return;
    mSelectedWaypoint = wp;

    Path emptyPath;
    for (SquadEntity* member : mSquadMembers)
    {
        member->setNextWaypoint(0);
        member->setPath(emptyPath);
        member->setGoal(wp->position());
    }
}

void SquadLeaderEntity::commandSquadToRallyOnLeader()
{
    if (mSquadMembers.empty())
        return;

    Path emptyPath;
    for (SquadEntity* member : mSquadMembers)
    {
        member->setNextWaypoint(0);
        member->setPath(emptyPath);
        member->setGoal(position());
    }
}

void SquadLeaderEntity::removeSquadMember(SquadEntity* member)
{
    auto it = std::find(mSquadMembers.begin(), mSquadMembers.end(), member);
    if (it != mSquadMembers.end())
        mSquadMembers.erase(it);
}

} // namespace Radion::AI
