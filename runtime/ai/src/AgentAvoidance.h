#ifndef RADION_AI_AGENT_AVOIDANCE_H
#define RADION_AI_AGENT_AVOIDANCE_H

namespace Radion
{
class Agent;
}

namespace Radion::AI::detail
{

// Repulsion is summed over neighbours, weighted (1 - d/r); turnRate 0 leaves the move alone, 1 replaces it.
// Coincident agents take deterministic opposite directions so they do not stay stuck.
void applyAgentAvoidance(Radion::Agent& agent, float avoidDistance, float turnRate);

} // namespace Radion::AI::detail

#endif // RADION_AI_AGENT_AVOIDANCE_H
