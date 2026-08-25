#ifndef RADION_AI_SQUADAI_H
#define RADION_AI_SQUADAI_H

// SquadAI.h - builds the squad leader/member state machines.
//
// The machines are built from their definitions using CallbackAction /
// CallbackTransition lambdas that capture the owning entity. Ownership of the
// returned StateMachine is transferred to the caller (assign it with
// SquadEntity::setStateMachine and delete it before the entity dies - the
// callbacks capture the entity by reference).

#include "SquadEntity.h"
#include "StateMachine.h"

namespace Radion::AI
{

// Leader machine: AwaitingSquadTaskCompletion <-> CommandSquadToPOI <-> StandingGround.
StateMachine* buildLeaderStateMachine(SquadLeaderEntity& leader);

// Member machine: WaitingForCommand -> MovingToGoal -> WaypointReached.
StateMachine* buildMemberStateMachine(SquadEntity& member);

} // namespace Radion::AI

#endif // RADION_AI_SQUADAI_H
