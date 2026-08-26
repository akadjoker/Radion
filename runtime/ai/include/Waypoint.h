#ifndef RADION_AI_WAYPOINT_H
#define RADION_AI_WAYPOINT_H

// Waypoint.h - a node in a waypoint network, plus its outgoing edges.
//
// A WaypointID is a Radion::u32 auto-incremented from 1 (0 = invalid).
// Waypoints are owned by the WaypointNetwork they are added to.

#include "Types.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

namespace Radion::AI
{

using WaypointID = Radion::u32;

// An edge from one waypoint to another. costModifier scales the distance-based
// edge cost; open == false makes the edge impassable to the A* search.
struct NetworkEdge
{
    WaypointID destination = 0;
    float costModifier = 1.0f;
    bool open = true;
};

class WaypointNetwork;

class Waypoint
{
public:
    explicit Waypoint(const glm::vec3& position = glm::vec3(0.0f),
                      const glm::quat& orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                      float radius = 1.0f);

    WaypointID id() const
    {
        return mId;
    }

    const glm::vec3& position() const
    {
        return mPosition;
    }
    void setPosition(const glm::vec3& position)
    {
        mPosition = position;
    }
    const glm::quat& orientation() const
    {
        return mOrientation;
    }
    void setOrientation(const glm::quat& orientation)
    {
        mOrientation = orientation;
    }
    float radius() const
    {
        return mRadius;
    }
    void setRadius(float radius)
    {
        mRadius = radius;
    }

    // Edge management. addEdge() returns false if an equal edge exists.
    bool addEdge(const NetworkEdge& edge);
    bool removeEdge(const NetworkEdge& edge);
    void clearEdges();

    // Cost of travelling along this edge: distance to the destination
    // waypoint scaled by the edge's cost modifier.
    float costForEdge(const NetworkEdge& edge, const WaypointNetwork& network) const;

    std::vector<NetworkEdge>& edges()
    {
        return mOutgoingEdges;
    }
    const std::vector<NetworkEdge>& edges() const
    {
        return mOutgoingEdges;
    }

private:
    WaypointID mId;
    std::vector<NetworkEdge> mOutgoingEdges;
    glm::vec3 mPosition;
    glm::quat mOrientation;
    float mRadius;
};

} // namespace Radion::AI

#endif // RADION_AI_WAYPOINT_H
