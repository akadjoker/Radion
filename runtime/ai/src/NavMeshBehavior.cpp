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

glm::vec3 NavMeshBehavior::paramVec3(u32 index) const
{
    if (index == 6)
        return mSettings.searchExtents;
    return glm::vec3(0.0f);
}

void NavMeshBehavior::setParamVec3(u32 index, const glm::vec3& value)
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

    // Agent::update() advances the position by the velocity before any
    // behavior runs, so what it holds now is a freely integrated guess that
    // may already have crossed a wall. Slide that guess back onto the
    // walkable surface: this is what makes leaving the floor impossible
    // rather than merely unlikely, since a route only ever suggests a
    // direction and cutting a corner or being shoved by avoidance leaves it
    // on its own.
    constrainToSurface(entity, route);

    const glm::vec3 position = entity.position();
    const glm::vec3 goal = entity.goal();

    glm::vec3 flatToGoal = goal - position;
    flatToGoal.y = 0.0f;
    if (glm::length(flatToGoal) < mSettings.goalRadius)
    {
        // Arrived: brake rather than drift past, then still resolve
        // avoidance so a crowd standing on the goal spreads out.
        entity.setDesiredMove(-entity.velocity());
        applyAvoidance(entity);
        return;
    }

    // Two gates, not one: the interval keeps the rate down, and the goal
    // having actually moved is what decides there is anything new to find.
    // A stationary goal costs one search and then nothing - re-running A*
    // every interval to rediscover the same corners is pure waste with a
    // crowd on screen.
    const bool goalMoved =
        !route.hasRoute ||
        glm::length(goal - route.goalWhenFound) > mSettings.goalMoveThreshold;
    const bool outOfCorners = route.next >= route.corners.size();
    // Running out of corners is a reason to search again, but not a reason to
    // skip the interval: when findPath() fails - goal off the mesh, nothing
    // reachable - the corner list stays empty, so outOfCorners stays true and
    // an uncapped retry ran a full A* every frame, for every agent, exactly
    // when the level is hardest to search.
    if (route.sinceRepath >= mSettings.repathInterval && (goalMoved || outOfCorners))
    {
        route.sinceRepath = 0.0f;
        std::vector<glm::vec3> fresh;
        if (mNavMesh->findPath(position, goal, fresh, mSettings.searchExtents) && fresh.size() > 1)
        {
            route.corners = std::move(fresh);
            route.next = 1; // [0] is where the agent already stands
            route.goalWhenFound = goal;
            route.hasRoute = true;
        }
        else
        {
            // Off the mesh, or nothing reachable. Fall back to heading
            // straight at the goal rather than freezing - a zombie that
            // stops dead reads as broken, one that walks into a wall reads
            // as a zombie.
            route.corners.clear();
            route.next = 0;
            route.hasRoute = false;
        }
    }

    glm::vec3 towards = flatToGoal;
    if (route.next < route.corners.size())
    {
        glm::vec3 toCorner = route.corners[route.next] - position;
        toCorner.y = 0.0f;
        if (glm::length(toCorner) < mSettings.cornerRadius)
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

    glm::vec3 desired = entity.desiredMove();
    desired += safeNormalize(towards) * mSettings.turnRate * gain();
    entity.setDesiredMove(desired);

    applyAvoidance(entity);
}

void NavMeshBehavior::constrainToSurface(Agent& entity, Route& route)
{
    const glm::vec3 wanted = entity.position();

    // First pass for this agent: it has no known-good position behind it, so
    // snap onto the surface instead of sliding across it.
    if (!route.onSurface)
    {
        glm::vec3 snapped;
        if (!mNavMesh->nearestPoint(wanted, snapped, mSettings.searchExtents))
            return;
        route.surfacePosition = snapped;
        route.onSurface = true;
        entity.setPosition(glm::vec3(snapped.x, wanted.y, snapped.z));
        return;
    }

    glm::vec3 slid;
    if (!mNavMesh->moveAlongSurface(route.surfacePosition, wanted, slid, mSettings.searchExtents))
    {
        // The last good position stopped being on the mesh - the surface was
        // rebuilt, or the agent was teleported. Re-acquire next pass.
        route.onSurface = false;
        return;
    }

    route.surfacePosition = slid;

    // Height stays the caller's business: the demo places its characters on
    // its own ground offset, and overwriting y here would fight it.
    entity.setPosition(glm::vec3(slid.x, wanted.y, slid.z));
}

void NavMeshBehavior::applyAvoidance(Agent& entity)
{
    detail::applyAgentAvoidance(entity, mSettings.avoidDistance, mSettings.turnRate);
}

} // namespace Radion::AI
