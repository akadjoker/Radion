#ifndef RADION_PHYSICS_COLLISION_SHAPE_H
#define RADION_PHYSICS_COLLISION_SHAPE_H

#include "BoundsTree.h"
#include "Color.h"
#include "ConvexHullComputer.h"
#include "Math.h"
#include "Types.h"

#include <vector>

namespace Radion::Geometry
{
struct Shard;
}

namespace Radion::Physics
{

enum class ShapeType : u8
{
    Sphere,
    Box,
    Capsule,
    Plane,
    Triangle,
    Trimesh,
    ConvexHull,
    Count
};

// Pure geometry in body space; no transform or body pointer, so one shape serves any bodies from any threads.
class CollisionShape
{
public:
    virtual ~CollisionShape() = default;

    virtual ShapeType type() const = 0;

    virtual Math::mat3 inertia(f32 mass) const = 0;

    virtual AABB bounds(const Math::mat4& transform) const = 0;

    // Support point along `direction` (world space, need not be normalized): the primitive SAT projection uses.
    virtual Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const = 0;

    void project(const Math::mat4& transform, const Math::vec3& axis, f32& minimum,
                 f32& maximum) const;

    // Emitted through batched DebugDraw3D: many colliders cost one draw call.
    virtual void debugDraw(const Math::mat4& transform, Color color) const = 0;
};

class SphereShape final : public CollisionShape
{
public:
    explicit SphereShape(f32 radius);

    ShapeType type() const override
    {
        return ShapeType::Sphere;
    }
    Math::mat3 inertia(f32 mass) const override;
    AABB bounds(const Math::mat4& transform) const override;
    Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const override;
    void debugDraw(const Math::mat4& transform, Color color) const override;

    f32 radius() const
    {
        return mRadius;
    }

private:
    f32 mRadius;
};

class BoxShape final : public CollisionShape
{
public:
    explicit BoxShape(const Math::vec3& halfExtents);

    ShapeType type() const override
    {
        return ShapeType::Box;
    }
    Math::mat3 inertia(f32 mass) const override;
    AABB bounds(const Math::mat4& transform) const override;
    Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const override;
    void debugDraw(const Math::mat4& transform, Color color) const override;

    const Math::vec3& halfExtents() const
    {
        return mHalfExtents;
    }
    // World-space corners, in the order the face table indexes them.
    void corners(const Math::mat4& transform, Math::vec3 out[8]) const;
    static const u8* faceCorners(u32 index);
    static Math::vec3 faceNormal(const Math::mat4& transform, u32 index);

private:
    Math::vec3 mHalfExtents;
};

// Segment along local Y with a radius: the sphere case at the closest point on the segment.
class CapsuleShape final : public CollisionShape
{
public:
    // `halfHeight` is half the SEGMENT, not the capsule: total height is 2*(halfHeight + radius).
    CapsuleShape(f32 radius, f32 halfHeight);

    ShapeType type() const override
    {
        return ShapeType::Capsule;
    }
    Math::mat3 inertia(f32 mass) const override;
    AABB bounds(const Math::mat4& transform) const override;
    Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const override;
    void debugDraw(const Math::mat4& transform, Color color) const override;

    f32 radius() const
    {
        return mRadius;
    }
    f32 halfHeight() const
    {
        return mHalfHeight;
    }
    void segment(const Math::mat4& transform, Math::vec3& lower, Math::vec3& upper) const;

private:
    f32 mRadius;
    f32 mHalfHeight;
};

class PlaneShape final : public CollisionShape
{
public:
    PlaneShape(const Math::vec3& normal, f32 constant = 0.0f);

    ShapeType type() const override
    {
        return ShapeType::Plane;
    }
    Math::mat3 inertia(f32 mass) const override;
    AABB bounds(const Math::mat4& transform) const override;
    Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const override;
    void debugDraw(const Math::mat4& transform, Color color) const override;

    const Math::vec3& normal() const
    {
        return mNormal;
    }
    f32 constant() const
    {
        return mConstant;
    }

private:
    Math::vec3 mNormal;
    f32 mConstant;
};

// Edge i runs from vertex i to vertex i+1.
enum class TriangleFeature : u8
{
    Face,
    Edge0,
    Edge1,
    Edge2,
    Vertex0,
    Vertex1,
    Vertex2
};

// One triangle of a TrimeshShape in mesh space; built transiently in the trimesh narrowphase, never owned by a body.
class TriangleShape final : public CollisionShape
{
public:
    TriangleShape(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c, u8 sharedEdges = 0);

    ShapeType type() const override
    {
        return ShapeType::Triangle;
    }
    Math::mat3 inertia(f32 mass) const override;
    AABB bounds(const Math::mat4& transform) const override;
    Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const override;
    void debugDraw(const Math::mat4& transform, Color color) const override;

    const Math::vec3& vertex(u32 index) const
    {
        return mVertices[index];
    }
    // Unnormalized when degenerate; callers check the length.
    Math::vec3 rawNormal() const;

    // A shared edge is interior, not a rim: a contact there must push out along the face or a character catches on the seam.
    bool edgeIsShared(u32 edge) const
    {
        return (mSharedEdges & (1u << edge)) != 0;
    }
    bool featureIsInternal(TriangleFeature feature) const;

private:
    Math::vec3 mVertices[3];
    u8 mSharedEdges = 0;
};

// Static concave mesh: no single inertia tensor, so a body with it must be static or kinematic (enforced by Scene::addBody()).
class TrimeshShape final : public CollisionShape
{
public:
    // Copies both arrays: the shape outlives the MeshData and the tree indexes into this copy.
    TrimeshShape(const Math::vec3* vertices, u32 vertexCount, const u32* indices, u32 indexCount);

    ShapeType type() const override
    {
        return ShapeType::Trimesh;
    }
    Math::mat3 inertia(f32 mass) const override;
    AABB bounds(const Math::mat4& transform) const override;
    Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const override;
    void debugDraw(const Math::mat4& transform, Color color) const override;

    // Normals come from the WINDING (what collision uses), not the mesh's renderer normals, which can disagree.
    void debugDrawFaceNormals(const Math::mat4& transform, f32 length, Color color,
                              Color flippedColor) const;

    // Shared edges should be traversed in opposite directions; otherwise a triangle faces backwards and a one-sided sweep passes through.
    void windingErrors(std::vector<u32>& out) const;

    u32 triangleCount() const
    {
        return static_cast<u32>(mIndices.size() / 3);
    }
    TriangleShape triangle(u32 index) const;
    void query(const AABB& localBox, std::vector<u32>& out) const;

    // Non-solver queries (picking, line of sight, prop placement), all in this mesh's space.
    struct RayHit
    {
        u32 triangle = 0;
        f32 distance = 0.0f;
        Math::vec3 point{0.0f};
        Math::vec3 normal{0.0f, 1.0f, 0.0f};
    };

    struct SweepHit
    {
        u32 triangle = 0;
        // Fraction of `velocity` travelled before contact, in [0, 1]; negative means already overlapping, and the value is the depth.
        f32 t = 0.0f;
        Math::vec3 normal{0.0f, 1.0f, 0.0f};
    };

    // Sweeps an ellipsoid of `radii` from `centre` along `velocity` and reports the first hit. A sweep, not a push-out:
    // characters need distance-to-hit and what they landed on.
    bool sweepEllipsoid(const Math::vec3& localCentre, const Math::vec3& radii,
                        const Math::vec3& velocity, SweepHit& hit) const;
    bool sweepSphere(const Math::vec3& localCentre, f32 radius, const Math::vec3& velocity,
                     SweepHit& hit) const;

    // Third-person camera collision: sweeps a sphere of `radius` from `from` (anchor) to `to`, returning where the camera can sit,
    // pulled back to the first contact. `margin` keeps it clear of the surface (z-fighting).
    Math::vec3 slideCamera(const Math::vec3& from, const Math::vec3& to, f32 radius,
                          f32 margin = 0.02f) const;

    bool raycast(const Ray& localRay, f32 maxDistance, RayHit& hit) const;
    // Triangles truly within `radius` of `centre` (the tree only narrows by box).
    void overlapSphere(const Math::vec3& localCentre, f32 radius, std::vector<u32>& out) const;

private:
    void buildAdjacency();

    std::vector<Math::vec3> mVertices;
    std::vector<u32> mIndices;
    // Three bits per triangle: edge i is shared with another triangle.
    std::vector<u8> mSharedEdges;
    BoundsTree mTree;
    AABB mBounds;
};

// Arbitrary convex polyhedron from a ConvexHullComputer half-edge mesh, copied into flat arrays (the shape outlives the source).
// For Voronoi shatter debris.
class ConvexHullShape final : public CollisionShape
{
public:
    using Edge = Radion::Geometry::ConvexHullComputer::Edge;

    // `faces` holds one edge index per face (ConvexHullComputer convention); walk it with getNextEdgeOfFace().
    ConvexHullShape(const Math::vec3* vertices, u32 vertexCount, const Edge* edges, u32 edgeCount,
                    const int* faces, u32 faceCount);
    // Copies a shard's hull; its vertices are already relative to its centroid, the local space a CollisionShape expects.
    explicit ConvexHullShape(const Radion::Geometry::Shard& shard);

    ShapeType type() const override
    {
        return ShapeType::ConvexHull;
    }
    Math::mat3 inertia(f32 mass) const override;
    AABB bounds(const Math::mat4& transform) const override;
    Math::vec3 support(const Math::mat4& transform, const Math::vec3& direction) const override;
    void debugDraw(const Math::mat4& transform, Color color) const override;

    const std::vector<Math::vec3>& vertices() const
    {
        return mVertices;
    }
    const std::vector<Edge>& edges() const
    {
        return mEdges;
    }
    const std::vector<int>& faces() const
    {
        return mFaces;
    }
    u32 faceCount() const
    {
        return static_cast<u32>(mFaces.size());
    }

private:
    std::vector<Math::vec3> mVertices;
    std::vector<Edge> mEdges;
    std::vector<int> mFaces;
};

Math::vec3 closestPointOnSegment(const Math::vec3& a, const Math::vec3& b, const Math::vec3& point);

// Closest point on triangle by Voronoi regions; the region is also the `feature`, telling an edge contact from a face one.
Math::vec3 closestPointOnTriangle(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                                 const Math::vec3& point, TriangleFeature* feature = nullptr);

// Closest points between two segments; handles the parallel/degenerate cases a naive solve divides by zero on.
void closestPointsBetweenSegments(const Math::vec3& p1, const Math::vec3& q1, const Math::vec3& p2,
                                  const Math::vec3& q2, Math::vec3& c1, Math::vec3& c2);

} // namespace Radion::Physics

#endif // RADION_PHYSICS_COLLISION_SHAPE_H
