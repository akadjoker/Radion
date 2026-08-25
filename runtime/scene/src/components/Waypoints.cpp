#include "PCH.h"

#include "Waypoints.h"

#include "GameObject.h"

namespace Radion
{

Waypoints::Waypoints() : Component(Type)
{
}

u32 Waypoints::addPoint(const glm::vec3& localPosition, f32 radius)
{
    WaypointNode node;
    node.position = localPosition;
    node.radius = radius;
    mPoints.push_back(node);
    return static_cast<u32>(mPoints.size() - 1);
}

void Waypoints::removePoint(u32 index)
{
    if (index >= mPoints.size())
        return;

    mPoints.erase(mPoints.begin() + index);
    // Indices ARE the node identity, so every link above the hole shifts
    // down by one and every link at the hole disappears.
    for (WaypointNode& node : mPoints)
    {
        for (usize i = node.links.size(); i-- > 0;)
        {
            u32& link = node.links[i];
            if (link == index)
                node.links.erase(node.links.begin() + static_cast<std::ptrdiff_t>(i));
            else if (link > index)
                --link;
        }
    }
}

void Waypoints::clear()
{
    mPoints.clear();
}

usize Waypoints::pointCount() const
{
    return mPoints.size();
}

const WaypointNode& Waypoints::point(u32 index) const
{
    return mPoints[index];
}

WaypointNode& Waypoints::point(u32 index)
{
    return mPoints[index];
}

void Waypoints::setPointPosition(u32 index, const glm::vec3& localPosition)
{
    if (index < mPoints.size())
        mPoints[index].position = localPosition;
}

void Waypoints::setPointRadius(u32 index, f32 radius)
{
    if (index < mPoints.size())
        mPoints[index].radius = glm::max(radius, 0.0f);
}

glm::vec3 Waypoints::worldPosition(u32 index) const
{
    if (index >= mPoints.size())
        return glm::vec3(0.0f);
    const GameObject* object = owner();
    if (!object)
        return mPoints[index].position;
    return glm::vec3(object->globalTransform() * glm::vec4(mPoints[index].position, 1.0f));
}

bool Waypoints::linked(u32 a, u32 b) const
{
    if (a >= mPoints.size() || b >= mPoints.size())
        return false;
    const u32 low = glm::min(a, b);
    const u32 high = glm::max(a, b);
    const std::vector<u32>& links = mPoints[low].links;
    return std::find(links.begin(), links.end(), high) != links.end();
}

bool Waypoints::link(u32 a, u32 b)
{
    if (a == b || a >= mPoints.size() || b >= mPoints.size() || linked(a, b))
        return false;
    // Stored once, on the lower index - a two-way edge kept in one place
    // cannot fall out of sync with itself.
    mPoints[glm::min(a, b)].links.push_back(glm::max(a, b));
    return true;
}

bool Waypoints::unlink(u32 a, u32 b)
{
    if (a == b || a >= mPoints.size() || b >= mPoints.size())
        return false;
    std::vector<u32>& links = mPoints[glm::min(a, b)].links;
    const auto found = std::find(links.begin(), links.end(), glm::max(a, b));
    if (found == links.end())
        return false;
    links.erase(found);
    return true;
}

void Waypoints::clearLinks()
{
    for (WaypointNode& node : mPoints)
        node.links.clear();
}

void Waypoints::autoLink(f32 radius)
{
    clearLinks();
    const f32 radiusSquared = radius * radius;
    for (u32 i = 0; i < mPoints.size(); ++i)
        for (u32 j = i + 1; j < static_cast<u32>(mPoints.size()); ++j)
        {
            const glm::vec3 delta = mPoints[j].position - mPoints[i].position;
            if (glm::dot(delta, delta) <= radiusSquared)
                mPoints[i].links.push_back(j);
        }
}

} // namespace Radion
