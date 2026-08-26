#include "PCH.h"

#include "TriangleBVH.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Radion
{
namespace
{
constexpr u32 kLeafSize = 4;

bool rayBox(const AABB& box, const Math::Vec3& origin, const Math::Vec3& direction, f32 maxDistance)
{
    f32 nearT = 0.0f, farT = maxDistance;
    for (u32 axis = 0; axis < 3; ++axis)
    {
        if (std::abs(direction[axis]) < 1e-8f)
        {
            if (origin[axis] < box.min[axis] || origin[axis] > box.max[axis]) return false;
            continue;
        }
        const f32 inverse = 1.0f / direction[axis];
        f32 a = (box.min[axis] - origin[axis]) * inverse;
        f32 b = (box.max[axis] - origin[axis]) * inverse;
        if (a > b) std::swap(a, b);
        nearT = std::max(nearT, a); farT = std::min(farT, b);
        if (nearT > farT) return false;
    }
    return true;
}

bool triangleBVHRayIntersect(const Math::Vec3& origin, const Math::Vec3& direction,
                             const TriangleBVH::Triangle& triangle, f32 maxDistance, f32& distance,
                             Math::Vec3& normal)
{
    const Math::Vec3 edge1 = triangle.b - triangle.a;
    const Math::Vec3 edge2 = triangle.c - triangle.a;
    const Math::Vec3 p = glm::cross(direction, edge2);
    const f32 determinant = glm::dot(edge1, p);
    if (std::abs(determinant) < 1e-8f) return false;
    const f32 inverse = 1.0f / determinant;
    const Math::Vec3 toOrigin = origin - triangle.a;
    const f32 u = inverse * glm::dot(toOrigin, p);
    if (u < 0.0f || u > 1.0f) return false;
    const Math::Vec3 q = glm::cross(toOrigin, edge1);
    const f32 v = inverse * glm::dot(direction, q);
    if (v < 0.0f || u + v > 1.0f) return false;
    const f32 t = inverse * glm::dot(edge2, q);
    if (t < 1e-5f || t > maxDistance) return false;
    distance = t;
    normal = glm::normalize(glm::cross(edge1, edge2));
    return true;
}
}

void TriangleBVH::clear() { m_triangles.clear(); m_nodes.clear(); }

bool TriangleBVH::build(const MeshData& mesh, const Math::Mat4& transform)
{
    clear();
    if (mesh.positions.empty() || mesh.indices.empty() || mesh.indices.size() % 3 != 0) return false;
    std::vector<Triangle> triangles;
    triangles.reserve(mesh.indices.size() / 3);
    for (usize i = 0; i < mesh.indices.size(); i += 3)
    {
        const u32 ia = mesh.indices[i], ib = mesh.indices[i + 1], ic = mesh.indices[i + 2];
        if (ia >= mesh.positions.size() || ib >= mesh.positions.size() || ic >= mesh.positions.size()) { clear(); return false; }
        Triangle triangle;
        triangle.a = Math::Vec3(transform * Math::Vec4(mesh.positions[ia], 1.0f));
        triangle.b = Math::Vec3(transform * Math::Vec4(mesh.positions[ib], 1.0f));
        triangle.c = Math::Vec3(transform * Math::Vec4(mesh.positions[ic], 1.0f));
        if (!std::isfinite(triangle.a.x) || !std::isfinite(triangle.b.x) || !std::isfinite(triangle.c.x)) { clear(); return false; }
        triangle.bounds.expand(triangle.a); triangle.bounds.expand(triangle.b); triangle.bounds.expand(triangle.c);
        triangle.centroid = (triangle.a + triangle.b + triangle.c) / 3.0f;
        if (glm::length(glm::cross(triangle.b - triangle.a, triangle.c - triangle.a)) > 1e-8f) triangles.push_back(triangle);
    }
    if (triangles.empty()) return false;
    m_triangles = std::move(triangles);
    m_nodes.reserve(m_triangles.size() * 2);
    buildNode(0, static_cast<u32>(m_triangles.size()));
    return true;
}

u32 TriangleBVH::buildNode(u32 first, u32 count)
{
    Node node;
    for (u32 i = first; i < first + count; ++i) node.bounds.merge(m_triangles[i].bounds);
    const u32 nodeIndex = static_cast<u32>(m_nodes.size()); m_nodes.push_back(node);
    if (count <= kLeafSize) { m_nodes[nodeIndex].first = first; m_nodes[nodeIndex].count = count; return nodeIndex; }
    const Math::Vec3 extent = node.bounds.extents();
    u32 axis = extent.y > extent.x ? 1 : 0; if (extent.z > extent[axis]) axis = 2;
    const u32 mid = first + count / 2;
    std::nth_element(m_triangles.begin() + first, m_triangles.begin() + mid, m_triangles.begin() + first + count,
                     [axis](const Triangle& a, const Triangle& b) { return a.centroid[axis] < b.centroid[axis]; });
    const u32 left = buildNode(first, count / 2), right = buildNode(mid, count - count / 2);
    m_nodes[nodeIndex].left = left; m_nodes[nodeIndex].right = right;
    return nodeIndex;
}

bool TriangleBVH::intersectNode(u32 nodeIndex, const Math::Vec3& origin, const Math::Vec3& direction,
                                f32& closest, Hit& hit) const
{
    const Node& node = m_nodes[nodeIndex];
    if (!rayBox(node.bounds, origin, direction, closest)) return false;
    bool found = false;
    if (node.leaf())
        for (u32 i = node.first; i < node.first + node.count; ++i)
        {
            f32 distance; Math::Vec3 normal;
            if (triangleBVHRayIntersect(origin, direction, m_triangles[i], closest, distance, normal))
            { closest = distance; hit = {distance, i, origin + direction * distance, normal}; found = true; }
        }
    else found = intersectNode(node.left, origin, direction, closest, hit) || intersectNode(node.right, origin, direction, closest, hit);
    return found;
}

bool TriangleBVH::intersect(const Math::Vec3& origin, const Math::Vec3& direction, f32 maxDistance, Hit* hit) const
{
    if (m_nodes.empty() || maxDistance <= 0.0f || glm::length(direction) < 1e-8f) return false;
    const Math::Vec3 normalized = glm::normalize(direction);
    f32 closest = maxDistance; Hit result;
    if (!intersectNode(0, origin, normalized, closest, result)) return false;
    if (hit) *hit = result;
    return true;
}
} // namespace Radion
