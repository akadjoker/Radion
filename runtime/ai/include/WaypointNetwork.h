#ifndef RADION_AI_WAYPOINTNETWORK_H
#define RADION_AI_WAYPOINTNETWORK_H

// WaypointNetwork.h - a collection of waypoints connected by edges, with A*
// pathfinding over the graph.
//
// The network OWNS the waypoints added via addWaypoint() and deletes them on
// destruction. A path is a deque of WaypointIDs (O(1) pop_front).

#include "Waypoint.h"

#include <deque>
#include <glm/glm.hpp>
#include <unordered_map>

namespace Radion::AI
{

using Path = std::deque<WaypointID>;

// Determines whether a waypoint is reachable/visible from a position. The
// base implementation accepts everything. Derive and override isVisible() to
// feed line-of-sight results from your own world.
class WaypointVisibility
{
public:
    virtual ~WaypointVisibility() = default;
    virtual bool isVisible(const Math::Vec3& origin, const Math::Vec3& destination) const
    {
        (void)origin;
        (void)destination;
        return true;
    }
};

class WaypointNetwork
{
public:
    using WaypointMap = std::unordered_map<WaypointID, Waypoint*>;

    WaypointNetwork() = default;
    ~WaypointNetwork();

    // Ownership: the network takes ownership of waypoint and deletes it on
    // remove/clear/destruction.
    bool addWaypoint(Waypoint* waypoint);
    bool removeWaypoint(WaypointID waypointID);
    void clearWaypoints();
    Waypoint* findWaypoint(WaypointID waypointID) const;

    // A* over the waypoint graph.
    // Returns true and fills outPath (start..goal) if a route exists.
    bool findPath(WaypointID fromWaypoint, WaypointID toWaypoint, Path& outPath) const;

    // A* between arbitrary positions: snaps both endpoints to the closest
    // visible waypoint, then searches. If both snap to the same waypoint the
    // path is left empty and true is returned (walk straight to the goal).
    bool findPath(const Math::Vec3& origin, const Math::Vec3& destination,
                  const WaypointVisibility& visibility, Path& outPath) const;

    // False, with minimum/maximum left untouched, when the network is empty -
    // (+FLT_MAX, -FLT_MAX) used to be returned as if it were a valid box.
    bool extents(Math::Vec3& minimum, Math::Vec3& maximum) const;

    const WaypointMap& waypoints() const
    {
        return mWaypoints;
    }
    WaypointMap& waypoints()
    {
        return mWaypoints;
    }

private:
    bool findClosestValidWaypoint(const Math::Vec3& origin, const WaypointVisibility& visibility,
                                  WaypointID& outWaypointID) const;
    float goalEstimate(WaypointID fromWaypoint, WaypointID toWaypoint) const;

    WaypointMap mWaypoints;
};

} // namespace Radion::AI

#endif // RADION_AI_WAYPOINTNETWORK_H
