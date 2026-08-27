#ifndef RADION_OCTREE_H
#define RADION_OCTREE_H

#include "Math.h"
#include "Mesh.h"
#include "Types.h"

#include "Math.h"
#include "Math.h"
#include <vector>

namespace Radion
{

class DebugDraw3D;

// A spatial octree over world-space triangles, built once from the static
// level meshes (one CollisionMesh per mesh, transformed to world space) and
// queried every frame by the CharacterController and by picking. This is the
// acceleration structure the study code ("docs/study/Lab Project 13.1 -
// Collision Detection") lacks: CCollision there sweeps every triangle in the
// scene every iteration of CollideEllipsoid. Here the sweep and the raycast
// only touch the triangles in the nodes that overlap the query volume.
//
// Design notes:
//  * Triangles are baked to world space at build() time, so a query never
//    re-reads (or re-transforms) the caller's mesh. Each triangle remembers
//    the source and index it came from so a hit can be reported back.
//  * A triangle is stored at the deepest node that fully contains it. If it
//    straddles a split plane it lives one level up, where every query that
//    overlaps it is guaranteed to visit. No triangle is ever duplicated, so
//    the tree stays memory-tight.
//  * Nodes split only when they overflow and there is depth left to spend;
//    leaves are just nodes whose children were never created.
class TriangleOctree
{
public:
    static constexpr u32 kNoNode = ~0u;
    static constexpr u32 kDefaultMaxDepth = 8;
    static constexpr u32 kDefaultMaxTriangles = 8;

    // One triangle in the tree, with its world-space vertices baked in.
    struct Triangle
    {
        Math::vec3 v0, v1, v2;
        Math::vec3 normal; // face normal, normalized
        Math::vec3 centroid;
        AABB bounds;
        u32 source = 0; // index of the CollisionMesh that contributed it
        u32 index = 0;  // triangle index within that source
    };

    struct RayHit
    {
        f32 t = 0.0f;
        Math::vec3 point;
        Math::vec3 normal;
        u32 source = 0;
        u32 triangle = 0;
    };

    // The swept-primitive hit the CharacterController's slide loop feeds on.
    // t is a dimensionless fraction of the query velocity in [0,1] (or a
    // negative penetration depth when the volume already overlaps), so it is
    // the same number whether the caller thinks in ellipsoid or world space.
    struct SweepHit
    {
        f32 t = 0.0f;
        Math::vec3 normal; // world-space contact normal
        Math::vec3 point;  // world-space point of contact
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
        // How many candidate triangles the last query actually tested - a
        // live measure of how much the tree is pruning.
        u32 trianglesVisited = 0;
    };

    TriangleOctree() = default;
    ~TriangleOctree() = default;

    TriangleOctree(const TriangleOctree&) = delete;
    TriangleOctree& operator=(const TriangleOctree&) = delete;

    void clear();

    // Copies the referenced vertices of one collision mesh into the tree,
    // transformed by `transform` (so a level mesh living in its own local
    // space lines up with a world-space character). Call once per level mesh
    // before build().
    void addCollisionMesh(const CollisionMesh& mesh, const Math::mat4& transform);

    // Subdivides until no node holds more than `maxTriangles` (subject to
    // `maxDepth`). Cheap enough to re-run at load only.
    void build(u32 maxDepth = kDefaultMaxDepth, u32 maxTriangles = kDefaultMaxTriangles);

    // Nearest triangle along the ray, or false. `t` is the ray distance.
    bool raycast(const Ray& ray, RayHit& out) const;

    // Swept ellipsoid from `center` with half-extents `radii`, moved by
    // `velocity` - the CollideAndSlide primitive. Unit radii everywhere is a
    // plain swept sphere. Returns the earliest contact, if any.
    bool sweepEllipsoid(const Math::vec3& center, const Math::vec3& radii, const Math::vec3& velocity,
                        SweepHit& out) const;

    // Convenience wrapper: a swept sphere of `radius`.
    bool sweepSphere(const Math::vec3& center, f32 radius, const Math::vec3& velocity,
                     SweepHit& out) const;

    // Gathers every triangle whose bounds overlap `region`, for a query that
    // is neither a ray nor a sweep.
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

    // Debug: box() for every node, so a demo can draw the tree it queries.
    void drawDebug(DebugDraw3D& debug, u32 maxDepth = kDefaultMaxDepth,
                   bool leavesOnly = false) const;

private:
    struct Node
    {
        AABB bounds;
        u32 children[8] = {kNoNode, kNoNode, kNoNode, kNoNode, kNoNode, kNoNode, kNoNode, kNoNode};
        std::vector<u32> triangles; // indices into mTriangles stored here
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
    u32 mSourceCount = 0; // how many CollisionMeshes fed the tree
};

} // namespace Radion

#endif // RADION_OCTREE_H
