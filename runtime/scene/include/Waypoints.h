#ifndef RADION_WAYPOINTS_H
#define RADION_WAYPOINTS_H

#include "Component.h"

#include "Math.h"
#include <vector>

namespace Radion
{

// Position is local to the owner, so moving the object carries the graph. Links are indices into the node list, stored once per pair on the lower index's node; edges are two-way.
struct WaypointNode
{
    Math::vec3 position = Math::vec3(0.0f);
    f32 radius = 1.5f;
    std::vector<u32> links;
};

// The authored source; AI::WaypointNetwork is the runtime structure A* searches.
class Waypoints final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Waypoints;

    // Returns the new node's index.
    u32 addPoint(const Math::vec3& localPosition, f32 radius = 1.5f);
    // Also drops links to it and renumbers those above (indices are identity).
    void removePoint(u32 index);
    void clear();

    usize pointCount() const;
    const WaypointNode& point(u32 index) const;
    WaypointNode& point(u32 index);

    void setPointPosition(u32 index, const Math::vec3& localPosition);
    void setPointRadius(u32 index, f32 radius);

    // World space, for building an AI::WaypointNetwork.
    Math::vec3 worldPosition(u32 index) const;

    // Two-way. No-op for an out-of-range index, a self-link, or an existing pair.
    bool link(u32 a, u32 b);
    bool unlink(u32 a, u32 b);
    bool linked(u32 a, u32 b) const;
    void clearLinks();
    // Replaces the current links; hand-editing after it is expected.
    void autoLink(f32 radius);

private:
    friend class GameObject;

    Waypoints();

    std::vector<WaypointNode> mPoints;
};

} // namespace Radion

#endif // RADION_WAYPOINTS_H
