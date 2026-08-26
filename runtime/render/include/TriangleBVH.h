#ifndef RADION_TRIANGLE_BVH_H
#define RADION_TRIANGLE_BVH_H

#include "Math.h"
#include "Mesh.h"

#include <vector>

namespace Radion
{

class TriangleBVH
{
public:
    struct Hit
    {
        f32 distance = 0.0f;
        u32 triangle = 0;
        Math::Vec3 position{0.0f};
        Math::Vec3 normal{0.0f};
    };

    void clear();
    bool build(const MeshData& mesh, const Math::Mat4& transform = Math::Mat4(1.0f));
    bool intersect(const Math::Vec3& origin, const Math::Vec3& direction,
                   f32 maxDistance, Hit* hit = nullptr) const;
    usize triangleCount() const { return m_triangles.size(); }
    usize nodeCount() const { return m_nodes.size(); }

public:
    struct Triangle
    {
        Math::Vec3 a, b, c;
        AABB bounds;
        Math::Vec3 centroid;
    };
private:
    struct Node
    {
        AABB bounds;
        u32 first = 0;
        u32 count = 0;
        u32 left = 0;
        u32 right = 0;
        bool leaf() const { return count != 0; }
    };

    u32 buildNode(u32 first, u32 count);
    bool intersectNode(u32 node, const Math::Vec3& origin, const Math::Vec3& direction,
                       f32& closest, Hit& hit) const;

    std::vector<Triangle> m_triangles;
    std::vector<Node> m_nodes;
};

} // namespace Radion

#endif // RADION_TRIANGLE_BVH_H
