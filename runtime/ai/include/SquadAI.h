#ifndef RADION_AI_SQUADAI_H
#define RADION_AI_SQUADAI_H

// Ownership of the returned StateMachine passes to the caller; delete it before the agent dies
// (the callbacks capture the agent by reference).

#include "StateMachine.h"

namespace Radion
{
class Agent;
}

namespace Radion::AI
{

// `leader` is expected to have squadId() == 0.
StateMachine* buildLeaderStateMachine(Radion::Agent& leader);

StateMachine* buildMemberStateMachine(Radion::Agent& member);

} // namespace Radion::AI

#endif // RADION_AI_SQUADAI_H
