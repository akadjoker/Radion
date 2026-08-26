// WaypointNetwork.cpp - implementation of A* over a waypoint graph.
//
// The A* nodes are pooled per search and deleted before returning (no leak
// across searches). The algorithm: open list sorted by f-cost, node map keyed
// by waypoint id, g = parent.g + edge cost, h = Euclidean distance to goal.

#include "PCH.h"

#include "WaypointNetwork.h"

#include <cfloat>

namespace Radion::AI
{

WaypointNetwork::~WaypointNetwork()
{
    clearWaypoints();
}

bool WaypointNetwork::addWaypoint(Waypoint* waypoint)
{
    WaypointID id = waypoint->id();
    if (mWaypoints.find(id) == mWaypoints.end())
    {
        mWaypoints[id] = waypoint;
        return true;
    }
    return false;
}

bool WaypointNetwork::removeWaypoint(WaypointID waypointID)
{
    auto it = mWaypoints.find(waypointID);
    if (it == mWaypoints.end())
        return false;

    delete it->second;
    mWaypoints.erase(it);
    return true;
}

void WaypointNetwork::clearWaypoints()
{
    for (auto& kv : mWaypoints)
        delete kv.second;
    mWaypoints.clear();
}

Waypoint* WaypointNetwork::findWaypoint(WaypointID waypointID) const
{
    auto it = mWaypoints.find(waypointID);
    return it == mWaypoints.end() ? nullptr : it->second;
}

bool WaypointNetwork::findPath(WaypointID fromWaypoint, WaypointID toWaypoint, Path& outPath) const
{
    if (fromWaypoint == toWaypoint)
    {
        outPath.clear();
        outPath.push_back(fromWaypoint);
        return true;
    }
    if (!findWaypoint(fromWaypoint) || !findWaypoint(toWaypoint))
        return false;

    // Per-search node pool; freed on every return path below.
    struct AStarNode
    {
        WaypointID waypoint = 0;
        AStarNode* parent = nullptr;
        float f = 0.0f, g = 0.0f, h = 0.0f;
        bool inOpen = false;
        bool inClosed = false;
    };

    std::vector<AStarNode*> pool;
    std::unordered_map<WaypointID, AStarNode*> nodeMap;
    nodeMap.reserve(mWaypoints.size());

    auto allocNode = [&](WaypointID id)
    {
        AStarNode* node = new AStarNode();
        node->waypoint = id;
        pool.push_back(node);
        return node;
    };

    // Open list kept sorted descending by f so pop_back() yields the lowest f.
    auto insertSorted = [](std::vector<AStarNode*>& list, AStarNode* node)
    {
        auto pos = std::lower_bound(list.begin(), list.end(), node,
                                    [](const AStarNode* a, const AStarNode* b)
                                    {
                                        return a->f > b->f;
                                    });
        list.insert(pos, node);
    };

    // Seed the search with the starting waypoint.
    AStarNode* start = allocNode(fromWaypoint);
    start->g = 0.0f;
    start->h = goalEstimate(fromWaypoint, toWaypoint);
    start->f = start->g + start->h;
    start->inOpen = true;
    nodeMap[fromWaypoint] = start;

    std::vector<AStarNode*> open;
    open.push_back(start);

    while (!open.empty())
    {
        AStarNode* n = open.back();
        open.pop_back();
        n->inOpen = false;

        if (n->waypoint == toWaypoint)
        {
            outPath.clear();
            for (AStarNode* node = n; node != nullptr; node = node->parent)
                outPath.push_front(node->waypoint);

            for (AStarNode* p : pool)
                delete p;
            return true;
        }

        const Waypoint* nwp = findWaypoint(n->waypoint);
        for (const NetworkEdge& edge : nwp->edges())
        {
            const Waypoint* dest = findWaypoint(edge.destination);
            if (!dest || !edge.open)
                continue;

            float newg = n->g + nwp->costForEdge(edge, *this);

            auto nodeIt = nodeMap.find(edge.destination);
            bool wasInMap = nodeIt != nodeMap.end();
            AStarNode* node = wasInMap ? nodeIt->second : allocNode(edge.destination);
            if (!wasInMap)
                nodeMap[edge.destination] = node;

            if (wasInMap && (node->inOpen || node->inClosed) && node->g <= newg)
            {
                // Already in a queue with a cheaper or equal path to it.
                continue;
            }

            node->parent = n;
            node->g = newg;
            node->h = goalEstimate(node->waypoint, toWaypoint);
            node->f = node->g + node->h;

            node->inClosed = false;

            if (!node->inOpen)
            {
                node->inOpen = true;
                insertSorted(open, node);
            }
            else
            {
                // Cost changed - reposition within the sorted open list.
                auto pos = std::find(open.begin(), open.end(), node);
                if (pos != open.end())
                    open.erase(pos);
                insertSorted(open, node);
            }
        }

        n->inClosed = true;
    }

    for (AStarNode* p : pool)
        delete p;
    return false;
}

bool WaypointNetwork::findPath(const glm::vec3& origin, const glm::vec3& destination,
                               const WaypointVisibility& visibility, Path& outPath) const
{
    WaypointID closestToOrigin;
    WaypointID closestToDestination;

    if (!findClosestValidWaypoint(origin, visibility, closestToOrigin))
        return false;
    if (!findClosestValidWaypoint(destination, visibility, closestToDestination))
        return false;

    outPath.clear();

    // Same waypoint for both ends: walk straight to the destination point.
    if (closestToOrigin == closestToDestination)
        return true;

    return findPath(closestToOrigin, closestToDestination, outPath);
}

bool WaypointNetwork::extents(glm::vec3& minimum, glm::vec3& maximum) const
{
    if (mWaypoints.empty())
        return false;

    glm::vec3 min(FLT_MAX);
    glm::vec3 max(-FLT_MAX);
    for (const auto& kv : mWaypoints)
    {
        const Waypoint* wp = kv.second;
        const glm::vec3 radius(wp->radius());
        min = glm::min(min, wp->position() - radius);
        max = glm::max(max, wp->position() + radius);
    }
    minimum = min;
    maximum = max;
    return true;
}

bool WaypointNetwork::findClosestValidWaypoint(const glm::vec3& origin,
                                               const WaypointVisibility& visibility,
                                               WaypointID& outWaypointID) const
{
    float closestDistanceSq = FLT_MAX;
    bool foundClosest = false;

    for (const auto& kv : mWaypoints)
    {
        const Waypoint* wp = kv.second;
        if (wp && visibility.isVisible(origin, wp->position()))
        {
            glm::vec3 delta = origin - wp->position();
            float distSq = glm::dot(delta, delta);
            if (distSq < closestDistanceSq)
            {
                closestDistanceSq = distSq;
                outWaypointID = wp->id();
                foundClosest = true;
            }
        }
    }
    return foundClosest;
}

float WaypointNetwork::goalEstimate(WaypointID fromWaypoint, WaypointID toWaypoint) const
{
    const Waypoint* fromwp = findWaypoint(fromWaypoint);
    const Waypoint* towp = findWaypoint(toWaypoint);
    if (!fromwp || !towp)
        return 0.0f;
    return glm::length(fromwp->position() - towp->position());
}

} // namespace Radion::AI
