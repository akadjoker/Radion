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
Math::vec3 normalizedScaled(const Math::vec3& v, float scale)
{
    float len = Math::length(v);
    if (len <= 0.0f)
        return Math::vec3(0.0f);
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

Math::vec3 PathfindBehavior::paramVec3(u32 index) const
{
    if (index == 5)
        return mSettings.upVector;
    return Math::vec3(0.0f);
}

void PathfindBehavior::setParamVec3(u32 index, const Math::vec3& value)
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
    Math::vec3 entityPos = entity.position();
    Math::vec3 desiredMoveAdj(0.0f);

    WaypointVisibility defaultVisibility;
    const WaypointVisibility& visibility =
        mSettings.visibility ? *mSettings.visibility : defaultVisibility;

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
        wpID = path.front();
        path.pop_front();
        squadmate.setNextWaypoint(wpID);
        squadmate.resetTimeSinceWaypointReached();
    }
    else if (wpID == 0 || path.empty())
    {
        desiredMoveAdj = squadmate.goal() - entityPos;
        desiredMoveAdj.y = 0.0f;
        if (Math::length(desiredMoveAdj) < squadmate.goalRadius())
        {
            entity.setDesiredMove(-entity.velocity());
            applyAvoidance(entity);
            squadmate.resetTimeSinceWaypointReached();
            return;
        }
        else if (!squadmate.losStatus() && mSinceRepath >= mSettings.repathInterval)
        {
            // Rate limited: a failed search leaves the path empty and would rerun A* every frame per agent.
            mSinceRepath = 0.0f;
            network->findPath(entityPos, squadmate.goal(), visibility, path);
            squadmate.setPath(path);
            wpID = squadmate.nextWaypoint();
        }
    }

    Waypoint* wp = network->findWaypoint(wpID);
    if (wp)
    {
        Math::vec3 wppos = wp->position();
        desiredMoveAdj = wppos - entityPos;
        desiredMoveAdj.y = 0.0f;
        if (Math::length(desiredMoveAdj) < wp->radius())
        {
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
                desiredMoveAdj = squadmate.goal() - entityPos;
                squadmate.setNextWaypoint(0);
                if (Math::length(desiredMoveAdj) < mSettings.goalRadius)
                {
                    entity.setDesiredMove(-entity.velocity());
                    applyAvoidance(entity);
                    squadmate.resetTimeSinceWaypointReached();
                    return;
                }
            }
        }
    }

    squadmate.incrementTimeSinceWaypointReached(timeDelta);
    Math::vec3 currentDesiredMove = entity.desiredMove();
    currentDesiredMove += normalizedScaled(desiredMoveAdj, mSettings.turnRate) * gain();

    if (squadmate.timeSinceWaypointReached() > mSettings.maxTimeBeforeAgitation)
    {
        currentDesiredMove = Math::cross(currentDesiredMove, mSettings.upVector);
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
