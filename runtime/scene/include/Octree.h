#ifndef RADION_OCTREE_H
#define RADION_OCTREE_H

#include "Math.h"
#include "Mesh.h"
#include "Types.h"

#include <vector>

namespace Radion
{

class DebugDraw3D;

// Triangles are baked to world space at build() and stored at the deepest node that fully contains them;
// straddlers live one level up, so none is duplicated.
class TriangleOctree
{
public:
    static constexpr u32 kNoNode = ~0u;
    static constexpr u32 kDefaultMaxDepth = 8;
    static constexpr u32 kDefaultMaxTriangles = 8;

    struct Triangle
    {
        Math::vec3 v0, v1, v2;
        Math::vec3 normal;
        Math::vec3 centroid;
        AABB bounds;
        u32 source = 0;
        u32 index = 0;
    };

    struct RayHit
    {
        f32 t = 0.0f;
        Math::vec3 point;
        Math::vec3 normal;
        u32 source = 0;
        u32 triangle = 0;
    };

    // t is a fraction of the query velocity in [0,1], or a negative penetration depth when already overlapping.
    struct SweepHit
    {
        f32 t = 0.0f;
        Math::vec3 normal;
        Math::vec3 point;
        u32 source = 0;
        u32 triangle = 0;
        bool collided = false;
    };

    struct Stats
    {
        u32 nodeCount = 0;
        u32 leafCount = 0;
        u32 triangleCount = 0;
        u32 maxDepth = 0;
        u32 trianglesVisited = 0;
    };

    TriangleOctree() = default;
    ~TriangleOctree() = default;

    TriangleOctree(const TriangleOctree&) = delete;
    TriangleOctree& operator=(const TriangleOctree&) = delete;

    void clear();

    // transform bakes a level mesh into world space. Call once per mesh before build().
    void addCollisionMesh(const CollisionMesh& mesh, const Math::mat4& transform);

    void build(u32 maxDepth = kDefaultMaxDepth, u32 maxTriangles = kDefaultMaxTriangles);

    bool raycast(const Ray& ray, RayHit& out) const;

    // Unit radii is a plain swept sphere.
    bool sweepEllipsoid(const Math::vec3& center, const Math::vec3& radii, const Math::vec3& velocity,
                        SweepHit& out) const;

    bool sweepSphere(const Math::vec3& center, f32 radius, const Math::vec3& velocity,
                     SweepHit& out) const;

    void collect(const AABB& region, std::vector<u32>& out) const;

    const AABB& bounds() const
    {
        return mBounds;
    }
    const Stats& stats() const
    {
        return mStats;
    }
    usize triangleCount() const
    {
        return mTriangles.size();
    }
    bool empty() const
    {
        return mTriangles.empty();
    }

    void drawDebug(DebugDraw3D& debug, u32 maxDepth = kDefaultMaxDepth,
                   bool leavesOnly = false) const;

private:
    struct Node
    {
        AABB bounds;
        u32 children[8] = {kNoNode, kNoNode, kNoNode, kNoNode, kNoNode, kNoNode, kNoNode, kNoNode};
        std::vector<u32> triangles;
        u8 depth = 0;
    };

    u32 createNode(const AABB& bounds, u8 depth);
    void subdivide(u32 nodeIndex, u8 depth, u32 maxDepth, u32 maxTriangles);
    void insertTriangle(u32 triangleIndex, u32 nodeIndex, u8 depth, u32 maxDepth, u32 maxTriangles);
    bool nodeSplittable(const Node& node) const;

    void collectInternal(u32 nodeIndex, const AABB& region, std::vector<u32>& out) const;
    void raycastNode(u32 nodeIndex, const Ray& ray, RayHit& best, bool& hit) const;
    void sweepNode(u32 nodeIndex, const AABB& sweptBounds, const Math::vec3& center,
                   const Math::vec3& radii, const Math::vec3& invRadii, const Math::vec3& velocity,
                   f32& bestT, SweepHit& out, bool& hit, u32& visited) const;

    std::vector<Triangle> mTriangles;
    std::vector<Node> mNodes;
    AABB mBounds;
    mutable Stats mStats;
    u32 mMaxDepth = kDefaultMaxDepth;
    u32 mSourceCount = 0;
};

} // namespace Radion

#endif // RADION_OCTREE_H
