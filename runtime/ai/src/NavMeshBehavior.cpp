#include "PCH.h"

#include "NavMeshBehavior.h"

#include "AIInternal.h"
#include "Group.h"
#include "NavMesh.h"
#include "SquadEntity.h"
#include "World.h"

namespace Radion::AI
{

using detail::safeNormalize;

NavMeshBehavior::NavMeshBehavior(const NavMesh& navMesh, const Settings& settings)
    : mNavMesh(navMesh), mSettings(settings)
{
}

void NavMeshBehavior::iterate(float timeDelta, Entity& entity)
{
    // The goal lives on SquadEntity, same as PathfindBehavior requires.
    SquadEntity* squadmate = dynamic_cast<SquadEntity*>(&entity);
    if (!squadmate || !mNavMesh.valid())
        return;

    Route& route = mRoutes[&entity];
    route.sinceRepath += timeDelta;

    // Entity::iterate() advances the position by the velocity before any
    // behavior runs, so what it holds now is a freely integrated guess that
    // may already have crossed a wall. Slide that guess back onto the
    // walkable surface: this is what makes leaving the floor impossible
    // rather than merely unlikely, since a route only ever suggests a
    // direction and cutting a corner or being shoved by avoidance leaves it
    // on its own.
    constrainToSurface(entity, route);

    const Math::Vec3 position = entity.position();
    const Math::Vec3 goal = squadmate->goal();

    Math::Vec3 flatToGoal = goal - position;
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
    if ((route.sinceRepath >= mSettings.repathInterval && goalMoved) || outOfCorners)
    {
        route.sinceRepath = 0.0f;
        std::vector<Math::Vec3> fresh;
        if (mNavMesh.findPath(position, goal, fresh, mSettings.searchExtents) && fresh.size() > 1)
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

    Math::Vec3 towards = flatToGoal;
    if (route.next < route.corners.size())
    {
        Math::Vec3 toCorner = route.corners[route.next] - position;
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

    Math::Vec3 desired = entity.desiredMove();
    desired += safeNormalize(towards) * mSettings.turnRate * gain();
    entity.setDesiredMove(desired);

    applyAvoidance(entity);
}

void NavMeshBehavior::constrainToSurface(Entity& entity, Route& route)
{
    const Math::Vec3 wanted = entity.position();

    // First pass for this agent: it has no known-good position behind it, so
    // snap onto the surface instead of sliding across it.
    if (!route.onSurface)
    {
        Math::Vec3 snapped;
        if (!mNavMesh.nearestPoint(wanted, snapped, mSettings.searchExtents))
            return;
        route.surfacePosition = snapped;
        route.onSurface = true;
        entity.setPosition(Math::Vec3(snapped.x, wanted.y, snapped.z));
        return;
    }

    Math::Vec3 slid;
    if (!mNavMesh.moveAlongSurface(route.surfacePosition, wanted, slid, mSettings.searchExtents))
    {
        // The last good position stopped being on the mesh - the surface was
        // rebuilt, or the agent was teleported. Re-acquire next pass.
        route.onSurface = false;
        return;
    }

    route.surfacePosition = slid;

    // Height stays the caller's business: the demo places its characters on
    // its own ground offset, and overwriting y here would fight it.
    entity.setPosition(Math::Vec3(slid.x, wanted.y, slid.z));
}

void NavMeshBehavior::applyAvoidance(Entity& entity)
{
    if (mSettings.avoidDistance <= 0.0f)
        return;

    // Same repulsion PathfindBehavior::applyAvoidance() applies: summed over
    // every neighbour inside the radius and weighted (1 - d/r), so it grows
    // as they close rather than switching on at the edge.
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
            const float distance = glm::length(away);
            if (distance >= avoidRadius)
                continue;

            if (distance > 1e-5f)
                repulsion += (away / distance) * (1.0f - distance / avoidRadius);
            else
                // Coincident agents need opposite deterministic directions,
                // or both pick the same escape and stay stuck together.
                repulsion += (&entity < other) ? entity.side() : -entity.side();
        }
    }

    const float repulsionLength = glm::length(repulsion);
    if (repulsionLength <= 1e-5f)
        return;

    Math::Vec3 desired = entity.desiredMove();
    const float desiredLength = glm::length(desired);
    const Math::Vec3 escape = repulsion / repulsionLength;
    const float weight = glm::clamp(mSettings.turnRate, 0.0f, 1.0f);

    if (desiredLength > 1e-5f)
    {
        Math::Vec3 direction = desired / desiredLength;
        direction = glm::normalize(direction * (1.0f - weight) + escape * weight);
        desired = direction * desiredLength;
    }
    else
        desired = escape * mSettings.turnRate;

    entity.setDesiredMove(desired);
}

} // namespace Radion::AI
