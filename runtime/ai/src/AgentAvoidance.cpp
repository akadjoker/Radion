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

    Math::vec3 repulsion(0.0f);
    const Math::vec3 position = agent.position();

    for (Radion::Agent* other : scene->agents())
    {
        // Null while an agent is being destroyed mid-update (see Scene::agents()).
        if (!other || other == &agent)
            continue;

        Math::vec3 away = position - other->position();
        away.y = 0.0f;
        const float distance = Math::length(away);
        if (distance >= avoidDistance)
            continue;

        if (distance > 1e-5f)
            repulsion += (away / distance) * (1.0f - distance / avoidDistance);
        else
            // Coincident agents need opposite deterministic directions, or both pick the same escape.
            repulsion += (&agent < other) ? agent.side() : -agent.side();
    }

    const float repulsionLength = Math::length(repulsion);
    if (repulsionLength <= 1e-5f)
        return;

    Math::vec3 desired = agent.desiredMove();
    const float desiredLength = Math::length(desired);
    const Math::vec3 escape = repulsion / repulsionLength;
    const float weight = Math::clamp(turnRate, 0.0f, 1.0f);

    if (desiredLength > 1e-5f)
    {
        Math::vec3 direction = desired / desiredLength;
        direction = Math::normalize(direction * (1.0f - weight) + escape * weight);
        desired = direction * desiredLength;
    }
    else
        desired = escape * turnRate;

    agent.setDesiredMove(desired);
}

} // namespace Radion::AI::detail
