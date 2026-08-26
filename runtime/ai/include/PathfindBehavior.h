#ifndef RADION_AI_PATHFINDBEHAVIOR_H
#define RADION_AI_PATHFINDBEHAVIOR_H

// PathfindBehavior.h - steering behavior that follows a waypoint path.
//
// Drives a SquadEntity along its Path, popping waypoints as they are reached,
// periodically testing line of sight to the goal (short-circuiting the path
// when it becomes visible), and nudging the agent with a perpendicular
// agitation vector if it stalls. Requires the owning entity to be a
// SquadEntity (dynamic_cast).

#include "Behavior.h"
#include "WaypointNetwork.h"

#include <glm/glm.hpp>

namespace Radion::AI
{

class WaypointNetwork;

class PathfindBehavior final : public Behavior
{
public:
    struct Settings
    {
        float turnRate = 0.2f;                 // desired-move strength toward the next node
        float goalRadius = 50.0f;              // distance at which the goal counts as reached
        float avoidDistance = 0.0f;            // <= 0 disables avoidance
        float maxTimeBeforeAgitation = 25.0f;  // seconds stuck before the agitation nudge
        float maxTimeBeforeLineOfSight = 0.5f; // seconds between goal line-of-sight tests
        glm::vec3 upVector = glm::vec3(0.0f, 1.0f, 0.0f);
        WaypointNetwork* waypointNetwork = nullptr; // non-owning
        // Line-of-sight functor used for the goal short-circuit. Supply one to
        // feed real LOS results, or nullptr for "always visible".
        const WaypointVisibility* visibility = nullptr;
    };

    explicit PathfindBehavior(const Settings& settings);

    void iterate(float timeDelta, Entity& entity) override;
    void applyAvoidance(Entity& entity);
    const char* name() const override
    {
        return "Pathfind Behavior";
    }

    Settings& settings()
    {
        return mSettings;
    }
    const Settings& settings() const
    {
        return mSettings;
    }

private:
    Settings mSettings;
};

} // namespace Radion::AI

#endif // RADION_AI_PATHFINDBEHAVIOR_H
