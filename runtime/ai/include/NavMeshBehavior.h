#ifndef RADION_AI_NAVMESHBEHAVIOR_H
#define RADION_AI_NAVMESHBEHAVIOR_H

// NavMeshBehavior.h - steering behavior that walks a SquadEntity across a
// NavMesh.
//
// The counterpart to PathfindBehavior, which routes over a hand-authored
// WaypointNetwork: this one routes over the walkable surface generated from
// the level's own geometry, so a route can never leave the floor. Same
// division of labour as its sibling - it only steers, and the avoidance that
// keeps several agents out of each other is applied here too.

#include "Behavior.h"

#include <glm/glm.hpp>
#include <vector>

namespace Radion::AI
{

class NavMesh;

class NavMeshBehavior final : public Behavior
{
public:
    struct Settings
    {
        float turnRate = 0.35f;    // desired-move strength toward the next corner
        float goalRadius = 1.0f;   // distance at which the goal counts as reached
        float cornerRadius = 0.6f; // how near a corner has to be to pop it
        float avoidDistance = 0.0f; // <= 0 disables agent avoidance
        // Seconds between route queries. A moving goal needs re-pathing, but
        // not every frame: findPath() is a real A* over the surface.
        float repathInterval = 0.35f;
        // And not even every interval: a route is only recomputed once the
        // goal has actually travelled this far from where it was when the
        // current one was found (or the agent ran out of corners). A player
        // standing still costs no searches at all after the first.
        float goalMoveThreshold = 1.0f;
        // How far off the mesh a point may sit and still snap onto it - the
        // goal is usually a player standing on the floor, not a point already
        // known to be on the navmesh.
        Math::Vec3 searchExtents = Math::Vec3(2.0f, 6.0f, 2.0f);
    };

    NavMeshBehavior(const NavMesh& navMesh, const Settings& settings);

    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "NavMesh Behavior";
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
    // Per-entity route state. The behavior itself is shared by every agent
    // using it, so nothing about one agent's path can live in its fields.
    struct Route
    {
        std::vector<Math::Vec3> corners;
        usize next = 0;
        float sinceRepath = 0.0f;
        Math::Vec3 goalWhenFound = Math::Vec3(0.0f);
        bool hasRoute = false;
        // Last position known to be on the walkable surface, which every
        // following move is slid from.
        Math::Vec3 surfacePosition = Math::Vec3(0.0f);
        bool onSurface = false;
    };

    void constrainToSurface(Entity& entity, Route& route);
    void applyAvoidance(Entity& entity);

    const NavMesh& mNavMesh;
    Settings mSettings;
    std::unordered_map<const Entity*, Route> mRoutes;
};

} // namespace Radion::AI

#endif // RADION_AI_NAVMESHBEHAVIOR_H
