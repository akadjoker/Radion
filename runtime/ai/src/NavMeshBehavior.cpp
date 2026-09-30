#include "PCH.h"

#include "NavMeshBehavior.h"

#include "AIInternal.h"
#include "AgentAvoidance.h"
#include "Agent.h"
#include "NavMesh.h"
#include "Scene.h"

namespace Radion::AI
{

using detail::safeNormalize;
using Radion::Agent;
using Radion::Scene;

namespace
{
const BehaviorParam kNavMeshParams[] = {
    {"Turn Rate", BehaviorParam::Kind::Float, 0.0f, 2.0f,
     "How strongly the agent steers toward the next corner of its route each update."},
    {"Goal Radius", BehaviorParam::Kind::Float, 0.0f, 10.0f,
     "Distance to the goal at which it counts as reached and the agent brakes."},
    {"Corner Radius", BehaviorParam::Kind::Float, 0.0f, 5.0f,
     "How close a route corner has to be before it is popped for the next one."},
    {"Avoid Distance", BehaviorParam::Kind::Float, 0.0f, 10.0f,
     "Radius other agents are repelled from; 0 disables agent-to-agent avoidance."},
    {"Repath Interval", BehaviorParam::Kind::Float, 0.0f, 5.0f,
     "Minimum seconds between route searches - the goal also has to have moved for a new "
     "one to run."},
    {"Goal Move Threshold", BehaviorParam::Kind::Float, 0.0f, 10.0f,
     "How far the goal has to travel since the last search before a new route is worth "
     "finding."},
    {"Search Extents", BehaviorParam::Kind::Vec3, 0.0f, 20.0f,
     "Half-extents of the box searched around a point when snapping it onto the navmesh."},
};
} // namespace

NavMeshBehavior::NavMeshBehavior(const NavMesh* navMesh) : mNavMesh(navMesh), mSettings()
{
}

NavMeshBehavior::NavMeshBehavior(const NavMesh* navMesh, const Settings& settings)
    : mNavMesh(navMesh), mSettings(settings)
{
}

u32 NavMeshBehavior::paramCount() const
{
    return static_cast<u32>(sizeof(kNavMeshParams) / sizeof(kNavMeshParams[0]));
}

const BehaviorParam& NavMeshBehavior::paramInfo(u32 index) const
{
    return kNavMeshParams[index];
}

f32 NavMeshBehavior::paramFloat(u32 index) const
{
    switch (index)
    {
    case 0: return mSettings.turnRate;
    case 1: return mSettings.goalRadius;
    case 2: return mSettings.cornerRadius;
    case 3: return mSettings.avoidDistance;
    case 4: return mSettings.repathInterval;
    case 5: return mSettings.goalMoveThreshold;
    default: return 0.0f;
    }
}

void NavMeshBehavior::setParamFloat(u32 index, f32 value)
{
    switch (index)
    {
    case 0: mSettings.turnRate = value; break;
    case 1: mSettings.goalRadius = value; break;
    case 2: mSettings.cornerRadius = value; break;
    case 3: mSettings.avoidDistance = value; break;
    case 4: mSettings.repathInterval = value; break;
    case 5: mSettings.goalMoveThreshold = value; break;
    default: break;
    }
}

Math::vec3 NavMeshBehavior::paramVec3(u32 index) const
{
    if (index == 6)
        return mSettings.searchExtents;
    return Math::vec3(0.0f);
}

void NavMeshBehavior::setParamVec3(u32 index, const Math::vec3& value)
{
    if (index == 6)
        mSettings.searchExtents = value;
}

void NavMeshBehavior::iterate(float timeDelta, Agent& entity)
{
    if (!mNavMesh || !mNavMesh->valid())
        return;

    Route& route = mRoute;
    route.sinceRepath += timeDelta;

    // Agent::update() integrates position before behaviors run, so it may have crossed a wall; slide it back onto the surface.
    constrainToSurface(entity, route);

    const Math::vec3 position = entity.position();
    const Math::vec3 goal = entity.goal();

    Math::vec3 flatToGoal = goal - position;
    flatToGoal.y = 0.0f;
    if (Math::length(flatToGoal) < mSettings.goalRadius)
    {
        // Arrived: brake, but still resolve avoidance so a crowd on the goal spreads out.
        entity.setDesiredMove(-entity.velocity());
        applyAvoidance(entity);
        return;
    }

    // Two gates: the interval limits rate, goal movement decides whether there is anything new; a stationary goal costs one search.
    const bool goalMoved =
        !route.hasRoute ||
        Math::length(goal - route.goalWhenFound) > mSettings.goalMoveThreshold;
    const bool outOfCorners = route.next >= route.corners.size();
    // Out of corners does not bypass the interval: a failing findPath() leaves the list empty and would run A* every frame per agent.
    if (route.sinceRepath >= mSettings.repathInterval && (goalMoved || outOfCorners))
    {
        route.sinceRepath = 0.0f;
        std::vector<Math::vec3> fresh;
        if (mNavMesh->findPath(position, goal, fresh, mSettings.searchExtents) && fresh.size() > 1)
        {
            route.corners = std::move(fresh);
            route.next = 1; // [0] is where the agent already stands
            route.goalWhenFound = goal;
            route.hasRoute = true;
        }
        else
        {
            // Off the mesh or unreachable: head straight at the goal instead of freezing.
            route.corners.clear();
            route.next = 0;
            route.hasRoute = false;
        }
    }

    Math::vec3 towards = flatToGoal;
    if (route.next < route.corners.size())
    {
        Math::vec3 toCorner = route.corners[route.next] - position;
        toCorner.y = 0.0f;
        if (Math::length(toCorner) < mSettings.cornerRadius)
        {
            ++route.next;
            if (route.next < route.corners.size())
            {
                toCorner = route.corners[route.next] - position;
                toCorner.y = 0.0f;
            }
        }
        if (route.next < route.corners.size())
            towards = toCorner;
    }

    Math::vec3 desired = entity.desiredMove();
    desired += safeNormalize(towards) * mSettings.turnRate * gain();
    entity.setDesiredMove(desired);

    applyAvoidance(entity);
}

void NavMeshBehavior::constrainToSurface(Agent& entity, Route& route)
{
    const Math::vec3 wanted = entity.position();

    // First pass has no known-good position: snap onto the surface instead of sliding.
    if (!route.onSurface)
    {
        Math::vec3 snapped;
        if (!mNavMesh->nearestPoint(wanted, snapped, mSettings.searchExtents))
            return;
        route.surfacePosition = snapped;
        route.onSurface = true;
        entity.setPosition(Math::vec3(snapped.x, wanted.y, snapped.z));
        return;
    }

    Math::vec3 slid;
    if (!mNavMesh->moveAlongSurface(route.surfacePosition, wanted, slid, mSettings.searchExtents))
    {
        // Last good position left the mesh (rebuilt or teleported); re-acquire next pass.
        route.onSurface = false;
        return;
    }

    route.surfacePosition = slid;

    // Height stays the caller's business; overwriting y would fight its ground offset.
    entity.setPosition(Math::vec3(slid.x, wanted.y, slid.z));
}

void NavMeshBehavior::applyAvoidance(Agent& entity)
{
    detail::applyAgentAvoidance(entity, mSettings.avoidDistance, mSettings.turnRate);
}

} // namespace Radion::AI
