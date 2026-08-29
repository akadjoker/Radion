#include "PCH.h"

#include "AgentAvoidance.h"

#include "Agent.h"
#include "Scene.h"

namespace Radion::AI::detail
{

void applyAgentAvoidance(Radion::Agent& agent, float avoidDistance, float turnRate)
{
    if (avoidDistance <= 0.0f)
        return;

    Radion::Scene* scene = agent.scene();
    if (!scene)
        return;

    glm::vec3 repulsion(0.0f);
    const glm::vec3 position = agent.position();

    for (Radion::Agent* other : scene->agents())
    {
        if (other == &agent)
            continue;

        glm::vec3 away = position - other->position();
        away.y = 0.0f;
        const float distance = glm::length(away);
        if (distance >= avoidDistance)
            continue;

        if (distance > 1e-5f)
            repulsion += (away / distance) * (1.0f - distance / avoidDistance);
        else
            // Coincident agents need opposite deterministic directions, or
            // both pick the same escape and stay stuck together.
            repulsion += (&agent < other) ? agent.side() : -agent.side();
    }

    const float repulsionLength = glm::length(repulsion);
    if (repulsionLength <= 1e-5f)
        return;

    glm::vec3 desired = agent.desiredMove();
    const float desiredLength = glm::length(desired);
    const glm::vec3 escape = repulsion / repulsionLength;
    const float weight = glm::clamp(turnRate, 0.0f, 1.0f);

    if (desiredLength > 1e-5f)
    {
        glm::vec3 direction = desired / desiredLength;
        direction = glm::normalize(direction * (1.0f - weight) + escape * weight);
        desired = direction * desiredLength;
    }
    else
        desired = escape * turnRate;

    agent.setDesiredMove(desired);
}

} // namespace Radion::AI::detail
