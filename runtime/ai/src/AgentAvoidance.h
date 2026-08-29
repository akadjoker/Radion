#ifndef RADION_AI_AGENT_AVOIDANCE_H
#define RADION_AI_AGENT_AVOIDANCE_H

// AgentAvoidance.h - the agent-to-agent repulsion the routing behaviors share.
// Internal to the AI implementation, not a public header.

namespace Radion
{
class Agent;
}

namespace Radion::AI::detail
{

// Pushes `agent` away from every other agent inside `avoidDistance`, then
// bends its desired move toward that escape by `turnRate` (0 leaves the move
// alone, 1 replaces it). Repulsion is summed over all neighbours and weighted
// (1 - d/r), so it grows as they close instead of switching on at the edge;
// two agents standing in exactly the same place take deterministically
// opposite directions rather than both picking the same one and staying
// stuck.
//
// One copy, because PathfindBehavior and NavMeshBehavior had this same fifty
// lines each - identical but for brace style - and a fix to one would have
// silently missed the other.
void applyAgentAvoidance(Radion::Agent& agent, float avoidDistance, float turnRate);

} // namespace Radion::AI::detail

#endif // RADION_AI_AGENT_AVOIDANCE_H
