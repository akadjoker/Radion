// PathfindBehavior.cpp - waypoint-path steering behavior.
//
// applyAvoidance() rotates the desired move away from nearby agents around
// the up vector on the XZ plane. It stays off by default (avoidDistance <= 0).

#include "PCH.h"

#include "PathfindBehavior.h"

#include "Agent.h"
#include "AgentAvoidance.h"
#include "Scene.h"

namespace Radion::AI
{

using Radion::Agent;
using Radion::Scene;

namespace
{
// normalize * turnRate without producing NaNs on a zero vector.
glm::vec3 normalizedScaled(const glm::vec3& v, float scale)
{
    float len = glm::length(v);
    if (len <= 0.0f)
        return glm::vec3(0.0f);
    return v * (scale / len);
}

const BehaviorParam kPathfindParams[] = {
    {"Turn Rate", BehaviorParam::Kind::Float, 0.0f, 1.0f,
     "How strongly the agent steers toward the next path node or the goal each update."},
    {"Goal Radius", BehaviorParam::Kind::Float, 0.0f, 100.0f,
     "Distance to the goal at which it counts as reached and the agent brakes."},
    {"Avoid Distance", BehaviorParam::Kind::Float, 0.0f, 20.0f,
     "Radius other agents are repelled from; 0 disables agent-to-agent avoidance."},
    {"Max Time Before Agitation", BehaviorParam::Kind::Float, 0.0f, 120.0f,
     "Seconds stuck on the same waypoint before the perpendicular agitation nudge kicks in."},
    {"Max Time Before LOS", BehaviorParam::Kind::Float, 0.0f, 5.0f,
     "Seconds between line-of-sight tests toward the goal, which short-circuit the path when "
     "it becomes visible."},
    {"Up Vector", BehaviorParam::Kind::Vec3, 0.0f, 0.0f,
     "Axis the agitation nudge rotates the desired move around."},
};
} // namespace

PathfindBehavior::PathfindBehavior() : mSettings()
{
}

PathfindBehavior::PathfindBehavior(const Settings& settings) : mSettings(settings)
{
}

u32 PathfindBehavior::paramCount() const
{
    return static_cast<u32>(sizeof(kPathfindParams) / sizeof(kPathfindParams[0]));
}

const BehaviorParam& PathfindBehavior::paramInfo(u32 index) const
{
    return kPathfindParams[index];
}

f32 PathfindBehavior::paramFloat(u32 index) const
{
    switch (index)
    {
    case 0: return mSettings.turnRate;
    case 1: return mSettings.goalRadius;
    case 2: return mSettings.avoidDistance;
    case 3: return mSettings.maxTimeBeforeAgitation;
    case 4: return mSettings.maxTimeBeforeLineOfSight;
    default: return 0.0f;
    }
}

void PathfindBehavior::setParamFloat(u32 index, f32 value)
{
    switch (index)
    {
    case 0: mSettings.turnRate = value; break;
    case 1: mSettings.goalRadius = value; break;
    case 2: mSettings.avoidDistance = value; break;
    case 3: mSettings.maxTimeBeforeAgitation = value; break;
    case 4: mSettings.maxTimeBeforeLineOfSight = value; break;
    default: break;
    }
}

glm::vec3 PathfindBehavior::paramVec3(u32 index) const
{
    if (index == 5)
        return mSettings.upVector;
    return glm::vec3(0.0f);
}

void PathfindBehavior::setParamVec3(u32 index, const glm::vec3& value)
{
    if (index == 5)
        mSettings.upVector = value;
}

void PathfindBehavior::iterate(float timeDelta, Agent& entity)
{
    Agent& squadmate = entity;
    WaypointNetwork* network = mSettings.waypointNetwork;
    if (!network)
        return;

    Path& path = squadmate.path();
    WaypointID wpID = squadmate.nextWaypoint();
    glm::vec3 entityPos = entity.position();
    glm::vec3 desiredMoveAdj(0.0f);

    WaypointVisibility defaultVisibility;
    const WaypointVisibility& visibility =
        mSettings.visibility ? *mSettings.visibility : defaultVisibility;

    // If we can see the destination, short-circuit the pathfinding and go
    // straight there.
    mSinceRepath += timeDelta;
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
        else if (!squadmate.losStatus() && mSinceRepath >= mSettings.repathInterval)
        {
            // No waypoint, no path, no line of sight and not at the goal:
            // generate a fresh path to get there. Rate limited, because a
            // search that finds nothing leaves the path empty and lands right
            // back here on the next frame - unbounded, that is a full A* per
            // agent per frame for as long as the goal stays unreachable.
            mSinceRepath = 0.0f;
            network->findPath(entityPos, squadmate.goal(), visibility, path);
            squadmate.setPath(path);
            wpID = squadmate.nextWaypoint();
        }
    }

    Waypoint* wp = network->findWaypoint(wpID);
    if (wp)
    {
        glm::vec3 wppos = wp->position();
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
    glm::vec3 currentDesiredMove = entity.desiredMove();
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

void PathfindBehavior::applyAvoidance(Agent& entity)
{
    detail::applyAgentAvoidance(entity, mSettings.avoidDistance, mSettings.turnRate);
}

} // namespace Radion::AI
