#ifndef RADION_AI_WAYPOINTNETWORK_H
#define RADION_AI_WAYPOINTNETWORK_H

// Owns its waypoints (deleted on destruction). A path is a deque of WaypointIDs.

#include "Waypoint.h"

#include <deque>
#include "Math.h"
#include <unordered_map>

namespace Radion::AI
{

using Path = std::deque<WaypointID>;

// Base implementation accepts everything; override isVisible() for real line-of-sight.
class WaypointVisibility
{
public:
    virtual ~WaypointVisibility() = default;
    virtual bool isVisible(const Math::vec3& origin, const Math::vec3& destination) const
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

    // Takes ownership of the waypoint.
    bool addWaypoint(Waypoint* waypoint);
    bool removeWaypoint(WaypointID waypointID);
    void clearWaypoints();
    Waypoint* findWaypoint(WaypointID waypointID) const;

    bool findPath(WaypointID fromWaypoint, WaypointID toWaypoint, Path& outPath) const;

    // Snaps both endpoints to the closest visible waypoint; if both snap to the same one the path
    // is left empty and true is returned.
    bool findPath(const Math::vec3& origin, const Math::vec3& destination,
                  const WaypointVisibility& visibility, Path& outPath) const;

    // False, minimum/maximum untouched, when the network is empty.
    bool extents(Math::vec3& minimum, Math::vec3& maximum) const;

    const WaypointMap& waypoints() const
    {
        return mWaypoints;
    }
    WaypointMap& waypoints()
    {
        return mWaypoints;
    }

private:
    bool findClosestValidWaypoint(const Math::vec3& origin, const WaypointVisibility& visibility,
                                  WaypointID& outWaypointID) const;
    float goalEstimate(WaypointID fromWaypoint, WaypointID toWaypoint) const;

    WaypointMap mWaypoints;
};

} // namespace Radion::AI

#endif // RADION_AI_WAYPOINTNETWORK_H
