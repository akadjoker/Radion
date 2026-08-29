#ifndef RADION_AI_AI_H
#define RADION_AI_AI_H

// AI.h - umbrella header for the Radion AI library.
//
// Contents:
//   - Agent:          Agent.h (runtime/scene) - the flocking/steering/squad
//                      Component the Scene drives; AI::World/Group/Entity/
//                      SquadEntity are gone, folded into it and Scene.
//   - State machine:  State.h, Action.h, Transition.h, StateMachine.h
//   - Waypoints:      Waypoint.h, WaypointNetwork.h (A* over the graph),
//                     PointOfInterest.h
//   - Pathfinding:    GridMap.h, GridPathfinder.h (A* over a cost grid)
//   - Flocking:       Behavior.h, Steering.h (seek/flee/wander/pursuit/
//                     evasion/...), Obstacle.h (obstacle avoidance)
//   - Squad glue:     PathfindBehavior.h, FormationBehavior.h,
//                     NavMeshBehavior.h, SquadEntity.h (SquadCommand enum),
//                     SquadAI.h
//

#include "Action.h"
#include "Agent.h"
#include "Behavior.h"
#include "FormationBehavior.h"
#include "GridMap.h"
#include "GridPathfinder.h"
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

#endif // RADION_AI_AI_H
