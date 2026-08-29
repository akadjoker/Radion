#ifndef RADION_AI_SQUADENTITY_H
#define RADION_AI_SQUADENTITY_H

// SquadEntity.h - squad command enum and the random-waypoint helper.
//
// The entity classes that used to live here (SquadEntity, SquadLeaderEntity,
// SquadMemberEntity) are gone: their state is now Radion::Agent's own
// (runtime/scene/include/Agent.h) - one Component instead of an inheritance
// chain, with squadId() == 0 marking the leader.

#include "Waypoint.h"

namespace Radion::AI
{

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

} // namespace Radion::AI

#endif // RADION_AI_SQUADENTITY_H
