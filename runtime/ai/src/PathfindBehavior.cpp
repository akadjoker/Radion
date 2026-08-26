// PathfindBehavior.cpp - waypoint-path steering behavior.
//
// applyAvoidance() rotates the desired move away from nearby entities around
// the up vector on the XZ plane. It stays off by default (avoidDistance <= 0).

#include "PCH.h"

#include "PathfindBehavior.h"

#include "Entity.h"
#include "Group.h"
#include "SquadEntity.h"
#include "World.h"

namespace Radion::AI
{

namespace
{
// normalize * turnRate without producing NaNs on a zero vector.
Math::Vec3 normalizedScaled(const Math::Vec3& v, float scale)
{
    float len = glm::length(v);
    if (len <= 0.0f)
        return Math::Vec3(0.0f);
    return v * (scale / len);
}
} // namespace

PathfindBehavior::PathfindBehavior(const Settings& settings) : mSettings(settings)
{
}

void PathfindBehavior::iterate(float timeDelta, Entity& entity)
{
    // Pathfinding only works on squad entities (reference dynamic_cast).
    SquadEntity& squadmate = dynamic_cast<SquadEntity&>(entity);
    WaypointNetwork* network = mSettings.waypointNetwork;
    if (!network)
        return;

    Path& path = squadmate.path();
    WaypointID wpID = squadmate.nextWaypoint();
    Math::Vec3 entityPos = entity.position();
    Math::Vec3 desiredMoveAdj(0.0f);

    WaypointVisibility defaultVisibility;
    const WaypointVisibility& visibility =
        mSettings.visibility ? *mSettings.visibility : defaultVisibility;

    // If we can see the destination, short-circuit the pathfinding and go
    // straight there.
    squadmate.incrementTimeSinceLOSTest(timeDelta);
    if (squadmate.timeSinceLOSTest() > mSettings.maxTimeBeforeLineOfSight)
    {
        squadmate.setLOSStatus(visibility.isVisible(squadmate.position(), squadmate.goal()));
        if (squadmate.losStatus())
        {
            path.clear();
            squadmate.setNextWaypoint(0);
        }
        squadmate.resetTimeSinceLOSTest();
    }

    if (wpID == 0 && !path.empty())
    {
        // Pop the next waypoint.
        wpID = path.front();
        path.pop_front();
        squadmate.setNextWaypoint(wpID);
        squadmate.resetTimeSinceWaypointReached();
    }
    else if (wpID == 0 || path.empty())
    {
        desiredMoveAdj = squadmate.goal() - entityPos;
        desiredMoveAdj.y = 0.0f;
        if (glm::length(desiredMoveAdj) < squadmate.goalRadius())
        {
            // We made it - stand around.
            entity.setDesiredMove(-entity.velocity());
            applyAvoidance(entity);
            squadmate.resetTimeSinceWaypointReached();
            return;
        }
        else if (!squadmate.losStatus())
        {
            // No waypoint, no path, no line of sight and not at the goal:
            // generate a fresh path to get there.
            network->findPath(entityPos, squadmate.goal(), visibility, path);
            squadmate.setPath(path);
            wpID = squadmate.nextWaypoint();
        }
    }

    Waypoint* wp = network->findWaypoint(wpID);
    if (wp)
    {
        Math::Vec3 wppos = wp->position();
        desiredMoveAdj = wppos - entityPos;
        desiredMoveAdj.y = 0.0f;
        if (glm::length(desiredMoveAdj) < wp->radius())
        {
            // Close enough - advance to the next waypoint.
            squadmate.setCurrentWaypoint(wp->id());
            if (!path.empty())
            {
                wpID = path.front();
                path.pop_front();
                squadmate.setNextWaypoint(wpID);
                wp = network->findWaypoint(wpID);
                if (!wp) // the reference asserted here; guard instead
                    return;
                wppos = wp->position();
                desiredMoveAdj = wppos - entityPos;
                squadmate.resetTimeSinceWaypointReached();
            }
            else
            {
                // No more waypoints - walk toward the goal.
                desiredMoveAdj = squadmate.goal() - entityPos;
                squadmate.setNextWaypoint(0);
                if (glm::length(desiredMoveAdj) < mSettings.goalRadius)
                {
                    entity.setDesiredMove(-entity.velocity());
                    applyAvoidance(entity);
                    squadmate.resetTimeSinceWaypointReached();
                    return;
                }
            }
        }
    }

    // Move in the direction of the next path node or the goal position.
    squadmate.incrementTimeSinceWaypointReached(timeDelta);
    Math::Vec3 currentDesiredMove = entity.desiredMove();
    currentDesiredMove += normalizedScaled(desiredMoveAdj, mSettings.turnRate) * gain();

    // Do we need to agitate a bit to get back on track?
    if (squadmate.timeSinceWaypointReached() > mSettings.maxTimeBeforeAgitation)
    {
        // Nudge the desired move with a vector perpendicular to its direction.
        currentDesiredMove = glm::cross(currentDesiredMove, mSettings.upVector);
        squadmate.resetTimeSinceWaypointReached();
        squadmate.resetTimeSinceGoalReached();
    }

    entity.setDesiredMove(currentDesiredMove);

    applyAvoidance(entity);
}

void PathfindBehavior::applyAvoidance(Entity& entity)
{
    if (mSettings.avoidDistance <= 0.0f)
        return;

    Math::Vec3 repulsion(0.0f);
    const Math::Vec3 position = entity.position();
    const float avoidRadius = mSettings.avoidDistance;

    for (Group* group : entity.world().groups())
    {
        for (Entity* other : group->entities())
        {
            if (other == &entity)
                continue;

            Math::Vec3 away = position - other->position();
            away.y = 0.0f;
            float distance = glm::length(away);
            if (distance >= avoidRadius)
                continue;

            if (distance > 1e-5f)
            {
                const float strength = 1.0f - distance / avoidRadius;
                repulsion += (away / distance) * strength;
            }
            else
            {
                // Coincident agents need opposite deterministic directions.
                repulsion += (&entity < other) ? entity.side() : -entity.side();
            }
        }
    }

    const float repulsionLength = glm::length(repulsion);
    if (repulsionLength <= 1e-5f)
        return;

    Math::Vec3 desired = entity.desiredMove();
    const float desiredLength = glm::length(desired);
    const Math::Vec3 escape = repulsion / repulsionLength;
    const float weight = std::clamp(mSettings.turnRate, 0.0f, 1.0f);

    if (desiredLength > 1e-5f)
    {
        Math::Vec3 direction = desired / desiredLength;
        direction = glm::normalize(direction * (1.0f - weight) + escape * weight);
        desired = direction * desiredLength;
    }
    else
    {
        desired = escape * mSettings.turnRate;
    }
    entity.setDesiredMove(desired);
}

} // namespace Radion::AI
