#ifndef RADION_AI_SQUADAI_H
#define RADION_AI_SQUADAI_H

// SquadAI.h - builds the squad leader/member state machines.
//
// The machines are built from their definitions using CallbackAction /
// CallbackTransition lambdas that capture the owning agent. Ownership of the
// returned StateMachine is transferred to the caller (assign it with
// Agent::setStateMachine() and delete it before the agent dies - the
// callbacks capture the agent by reference).

#include "StateMachine.h"

namespace Radion
{
class Agent;
}

namespace Radion::AI
{

// Leader machine: AwaitingSquadTaskCompletion <-> CommandSquadToPOI <-> StandingGround.
// `leader` is expected to have squadId() == 0 (the leader-only methods this
// calls - sendSquadToRandomPOI() etc. - are Agent's own until Squad exists).
StateMachine* buildLeaderStateMachine(Radion::Agent& leader);

// Member machine: WaitingForCommand -> MovingToGoal -> WaypointReached.
StateMachine* buildMemberStateMachine(Radion::Agent& member);

} // namespace Radion::AI

#endif // RADION_AI_SQUADAI_H
