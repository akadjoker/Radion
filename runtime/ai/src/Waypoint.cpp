// Waypoint.cpp - implementation of Waypoint.

#include "PCH.h"

#include "Waypoint.h"

#include "WaypointNetwork.h"

#include <cfloat>

namespace Radion::AI
{

namespace
{
WaypointID nextWaypointId()
{
    static WaypointID counter = 1; // 0 is reserved as the null/invalid id
    return counter++;
}
} // namespace

Waypoint::Waypoint(const glm::vec3& position, const glm::quat& orientation, float radius)
    : mId(nextWaypointId()), mPosition(position), mOrientation(orientation), mRadius(radius)
{
}

bool Waypoint::addEdge(const NetworkEdge& edge)
{
    for (const NetworkEdge& e : mOutgoingEdges)
    {
        if (e.destination == edge.destination && e.costModifier == edge.costModifier &&
            e.open == edge.open)
            return false;
    }
    mOutgoingEdges.push_back(edge);
    return true;
}

bool Waypoint::removeEdge(const NetworkEdge& edge)
{
    for (auto it = mOutgoingEdges.begin(); it != mOutgoingEdges.end(); ++it)
    {
        const NetworkEdge& e = *it;
        if (e.destination == edge.destination && e.costModifier == edge.costModifier &&
            e.open == edge.open)
        {
            mOutgoingEdges.erase(it);
            return true;
        }
    }
    return false;
}

void Waypoint::clearEdges()
{
    mOutgoingEdges.clear();
}

float Waypoint::costForEdge(const NetworkEdge& edge, const WaypointNetwork& network) const
{
    const Waypoint* dest = network.findWaypoint(edge.destination);
    // WaypointNetwork::findPath() already skips edges whose destination does
    // not resolve, so this path never feeds the search. It is still reachable
    // by any other caller, so a missing destination must read as "impassable",
    // not "free": 0.0f here used to look like a zero-cost edge.
    if (!dest)
        return FLT_MAX;
    return glm::length(mPosition - dest->position()) * edge.costModifier;
}

} // namespace Radion::AI
