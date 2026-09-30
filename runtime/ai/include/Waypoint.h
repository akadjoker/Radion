#ifndef RADION_AI_WAYPOINT_H
#define RADION_AI_WAYPOINT_H

// WaypointID auto-increments from 1 (0 = invalid); waypoints are owned by their WaypointNetwork.

#include "Types.h"

#include "Math.h"
#include <vector>

namespace Radion::AI
{

using WaypointID = Radion::u32;

// costModifier scales the distance-based edge cost; open == false makes the edge impassable.
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
    explicit Waypoint(const Math::vec3& position = Math::vec3(0.0f),
                      const Math::quat& orientation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f),
                      float radius = 1.0f);

    WaypointID id() const
    {
        return mId;
    }

    const Math::vec3& position() const
    {
        return mPosition;
    }
    void setPosition(const Math::vec3& position)
    {
        mPosition = position;
    }
    const Math::quat& orientation() const
    {
        return mOrientation;
    }
    void setOrientation(const Math::quat& orientation)
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

    bool addEdge(const NetworkEdge& edge);
    bool removeEdge(const NetworkEdge& edge);
    void clearEdges();

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
    Math::vec3 mPosition;
    Math::quat mOrientation;
    float mRadius;
};

} // namespace Radion::AI

#endif // RADION_AI_WAYPOINT_H
