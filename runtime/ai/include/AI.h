#ifndef RADION_AI_AI_H
#define RADION_AI_AI_H

// AI.h - umbrella header for the Radion AI library.
//
// Contents:
//   - State machine:  State.h, Action.h, Transition.h, StateMachine.h
//   - Waypoints:      Waypoint.h, WaypointNetwork.h (A* over the graph),
//                     PointOfInterest.h
//   - Pathfinding:    GridMap.h, GridPathfinder.h (A* over a cost grid)
//   - Flocking:       Behavior.h, Entity.h, Group.h, World.h
//   - Steering:       Obstacle.h (obstacle avoidance), Steering.h
//                     (seek/flee/wander/pursuit/evasion/...)
//   - Squad glue:     PathfindBehavior.h, FormationBehavior.h, SquadEntity.h,
//                     SquadAI.h
//

#include "Action.h"
#include "Behavior.h"
#include "Entity.h"
#include "FormationBehavior.h"
#include "GridMap.h"
#include "GridPathfinder.h"
#include "Group.h"
#include "Obstacle.h"
#include "NavMeshBehavior.h"
#include "PathfindBehavior.h"
#include "PointOfInterest.h"
#include "SquadAI.h"
#include "SquadEntity.h"
#include "State.h"
#include "StateMachine.h"
#include "Steering.h"
#include "Transition.h"
#include "Waypoint.h"
#include "WaypointNetwork.h"
#include "World.h"

#endif // RADION_AI_AI_H
