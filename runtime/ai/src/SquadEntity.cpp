#include "PCH.h"

#include "SquadEntity.h"

#include "WaypointNetwork.h"

#include <cstdlib>

namespace Radion::AI
{

WaypointID selectRandomWaypoint(const WaypointNetwork& network)
{
    const auto& waypoints = network.waypoints();
    if (waypoints.empty())
        return 0;

    std::vector<WaypointID> ids;
    ids.reserve(waypoints.size());
    for (const auto& kv : waypoints)
        ids.push_back(kv.first);
    return ids[std::rand() % static_cast<int>(ids.size())];
}

} // namespace Radion::AI
