#ifndef RADION_WAYPOINTS_H
#define RADION_WAYPOINTS_H

#include "Component.h"

#include "Math.h"
#include <vector>

namespace Radion
{

// One node of a navigation graph authored in the editor. The position is
// LOCAL to the owning GameObject, so moving or rotating that object carries
// the whole graph with it - a castle's waypoints stay attached to the
// castle. Links are indices into the same component's node list, stored
// once per pair on the lower index's node; edges are always two-way, which
// is what a hand-authored walkable route means.
struct WaypointNode
{
    Math::vec3 position = Math::vec3(0.0f);
    f32 radius = 1.5f;
    std::vector<u32> links;
};

// A whole waypoint graph on one GameObject, instead of one object per point.
// The editor adds/drags points on it directly; a demo reads it back and
// feeds AI::WaypointNetwork, which is the runtime structure A* actually
// searches - this component is the authored source, not the search itself.
class Waypoints final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Waypoints;

    // Returns the new node's index. Position is local to the owner.
    u32 addPoint(const Math::vec3& localPosition, f32 radius = 1.5f);
    // Also drops every link pointing at it and renumbers the ones above -
    // indices are the identity here, so a removal has to fix them up.
    void removePoint(u32 index);
    void clear();

    usize pointCount() const;
    const WaypointNode& point(u32 index) const;
    WaypointNode& point(u32 index);

    void setPointPosition(u32 index, const Math::vec3& localPosition);
    void setPointRadius(u32 index, f32 radius);

    // Node position through the owner's world transform - what a demo
    // building an AI::WaypointNetwork wants, since A* runs in world space.
    Math::vec3 worldPosition(u32 index) const;

    // Two-way. link() is a no-op for an index out of range, a self-link, or
    // a pair already linked.
    bool link(u32 a, u32 b);
    bool unlink(u32 a, u32 b);
    bool linked(u32 a, u32 b) const;
    void clearLinks();
    // Links every pair closer than `radius`, replacing the current links.
    // The editor's one-click way to turn a scattering of dropped points into
    // a connected graph; hand-editing after it is expected.
    void autoLink(f32 radius);

private:
    friend class GameObject;

    Waypoints();

    std::vector<WaypointNode> mPoints;
};

} // namespace Radion

#endif // RADION_WAYPOINTS_H
