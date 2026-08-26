#ifndef RADION_AI_SQUADENTITY_H
#define RADION_AI_SQUADENTITY_H

// SquadEntity.h - an Entity that carries pathfinding/squad state.
//
// A SquadEntity holds a waypoint network, a goal position, a Path, a state
// machine and a current command. It does NOT own its state machine; the
// caller must delete the machine (or keep it alive) and must not let it
// outlive the entity while captured by its callbacks.

#include "Entity.h"
#include "FormationBehavior.h"
#include "Waypoint.h"
#include "WaypointNetwork.h"

#include <vector>

namespace Radion::AI
{

class PointOfInterest;
class PointsOfInterest;
class StateMachine;
class Waypoint;
class WaypointNetwork;

enum class SquadCommand
{
    PatrolWaypointNetwork,
    PatrolPointsOfInterest,
    RallyToLeaderPosition,
    StandGround,
    AttackTarget,
    FlankTarget,
    SuppressTarget,
    Regroup,
    Retreat,
    DefendPosition,
    MoveToPoint,
    SearchArea,
    EscortTarget,
    Count
};

// Random waypoint id from the network.
WaypointID selectRandomWaypoint(const WaypointNetwork& network);

class SquadEntity : public Entity
{
public:
    SquadEntity(World& world, const Settings& settings);

    void iterate(float timeDelta) override;

    void setPath(const Path& path)
    {
        mPath = path;
    }
    Path& path()
    {
        return mPath;
    }
    const Path& path() const
    {
        return mPath;
    }

    void setGoal(const Math::Vec3& goal)
    {
        mGoalPosition = goal;
    }
    const Math::Vec3& goal() const
    {
        return mGoalPosition;
    }

    void setNextWaypoint(WaypointID wp)
    {
        mNextWaypoint = wp;
    }
    WaypointID nextWaypoint() const
    {
        return mNextWaypoint;
    }
    void setCurrentWaypoint(WaypointID wp)
    {
        mCurrentWaypoint = wp;
    }
    WaypointID currentWaypoint() const
    {
        return mCurrentWaypoint;
    }

    void setWaypointNetwork(WaypointNetwork* network)
    {
        mWaypointNetwork = network;
    }
    WaypointNetwork* waypointNetwork() const
    {
        return mWaypointNetwork;
    }

    // State machine is NOT owned by the entity.
    void setStateMachine(StateMachine* machine)
    {
        mStateMachine = machine;
    }
    StateMachine* stateMachine() const
    {
        return mStateMachine;
    }

    bool hasValidWaypoint() const
    {
        return mNextWaypoint != 0;
    }
    bool hasValidPath() const
    {
        return !mPath.empty();
    }

    bool waypointReached();
    bool goalReached();
    void onWaypointReached();
    void onGoalReached();
    void onWaitingForCommand();

    void resetTimeSinceWaypointReached()
    {
        mTimeSinceNextWaypointReached = 0.0f;
    }
    void incrementTimeSinceWaypointReached(float deltaTime)
    {
        mTimeSinceNextWaypointReached += deltaTime;
    }
    float timeSinceWaypointReached() const
    {
        return mTimeSinceNextWaypointReached;
    }
    void resetTimeSinceGoalReached()
    {
        mTimeSinceGoalReached = 0.0f;
    }
    void incrementTimeSinceGoalReached(float deltaTime)
    {
        mTimeSinceGoalReached += deltaTime;
    }
    float timeSinceGoalReached() const
    {
        return mTimeSinceGoalReached;
    }
    void resetTimeSinceLOSTest()
    {
        mTimeSinceLOSTested = 0.0f;
    }
    void incrementTimeSinceLOSTest(float deltaTime)
    {
        mTimeSinceLOSTested += deltaTime;
    }
    float timeSinceLOSTest() const
    {
        return mTimeSinceLOSTested;
    }

    void setLOSStatus(bool status)
    {
        mLOSStatus = status;
    }
    bool losStatus() const
    {
        return mLOSStatus;
    }

    void setSquadId(int id)
    {
        mSquadId = id;
    }
    int squadId() const
    {
        return mSquadId;
    }

    void setGoalRadius(float radius)
    {
        mGoalRadius = radius;
    }
    float goalRadius() const
    {
        return mGoalRadius;
    }

    void setSquadFormation(int formation)
    {
        mSquadFormation = formation;
    }
    int squadFormation() const
    {
        return mSquadFormation;
    }

    virtual void setCommand(SquadCommand command);
    SquadCommand command() const
    {
        return mCommand;
    }

protected:
    WaypointNetwork* mWaypointNetwork = nullptr;
    WaypointID mNextWaypoint = 0;
    WaypointID mCurrentWaypoint = 0;
    Math::Vec3 mGoalPosition = Math::Vec3(0.0f);
    Path mPath;
    StateMachine* mStateMachine = nullptr; // non-owning
    SquadCommand mCommand = SquadCommand::PatrolPointsOfInterest;
    bool mLOSStatus = false;
    float mTimeSinceNextWaypointReached = 0.0f;
    float mTimeSinceGoalReached = 0.0f;
    float mTimeSinceLOSTested = 100.0f; // first LOS test fires immediately
    float mGoalRadius = 25.0f;
    int mSquadId = -1;
    int mSquadFormation = static_cast<int>(SquadFormation::Abreast);
};

class SquadLeaderEntity final : public SquadEntity
{
public:
    SquadLeaderEntity(World& world, const Settings& settings);

    void setCommand(SquadCommand command) override;

    bool hasCommandChanged() const
    {
        return !mCommandAcknowledged || command() != mLastCommand;
    }

    void acknowledgeCommand()
    {
        mLastCommand = command();
        mCommandAcknowledged = true;
    }

    void sendSquadToRandomPOI();
    void sendSquadToRandomWaypoint();
    void commandSquadToRallyOnLeader();
    void sendSquadToTarget();

    void setSelectedPointOfInterest(PointOfInterest* poi)
    {
        mSelectedPointOfInterest = poi;
    }
    PointOfInterest* selectedPointOfInterest() const
    {
        return mSelectedPointOfInterest;
    }
    void setSelectedWaypoint(Waypoint* wp)
    {
        mSelectedWaypoint = wp;
    }
    Waypoint* selectedWaypoint() const
    {
        return mSelectedWaypoint;
    }
    void setPointsOfInterest(PointsOfInterest* pois)
    {
        mPointsOfInterest = pois;
    }
    PointsOfInterest* pointsOfInterest() const
    {
        return mPointsOfInterest;
    }

    // Squad members are NOT owned by the leader.
    void addSquadMember(SquadEntity* member)
    {
        mSquadMembers.push_back(member);
    }
    void removeSquadMember(SquadEntity* member);
    void clearSquadMembers()
    {
        mSquadMembers.clear();
    }
    std::vector<SquadEntity*>& squadMembers()
    {
        return mSquadMembers;
    }
    const std::vector<SquadEntity*>& squadMembers() const
    {
        return mSquadMembers;
    }

private:
    std::vector<SquadEntity*> mSquadMembers; // non-owning
    PointsOfInterest* mPointsOfInterest = nullptr;
    PointOfInterest* mSelectedPointOfInterest = nullptr;
    Waypoint* mSelectedWaypoint = nullptr;
    SquadCommand mLastCommand = SquadCommand::PatrolPointsOfInterest;
    bool mCommandAcknowledged = false;
};

class SquadMemberEntity final : public SquadEntity
{
public:
    using SquadEntity::SquadEntity;
};

} // namespace Radion::AI

#endif // RADION_AI_SQUADENTITY_H
