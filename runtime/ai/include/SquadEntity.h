#ifndef RADION_AI_SQUADENTITY_H
#define RADION_AI_SQUADENTITY_H

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

WaypointID selectRandomWaypoint(const WaypointNetwork& network);

} // namespace Radion::AI

#endif // RADION_AI_SQUADENTITY_H
