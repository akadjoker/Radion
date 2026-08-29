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

// Pure geometry in the body's own space. A shape holds no transform and no
// back-pointer to a body - the caller passes the transform in - so one shape
// serves any number of bodies from any number of threads.
class CollisionShape
{
public:
    virtual ~CollisionShape() = default;

    virtual ShapeType type() const = 0;

    // Moment of inertia about the centre of mass, body space.
    virtual glm::mat3 inertia(f32 mass) const = 0;

    // World AABB under `transform` - what the broadphase sorts and prunes on.
    virtual AABB bounds(const glm::mat4& transform) const = 0;

    // Furthest point of the shape along `direction` (world space, direction
    // need not be normalized). The one primitive SAT projection is built on.
    virtual glm::vec3 support(const glm::mat4& transform, const glm::vec3& direction) const = 0;

    // Extent of the shape's shadow on `axis`, world space.
    void project(const glm::mat4& transform, const glm::vec3& axis, f32& minimum,
                 f32& maximum) const;

    // Emitted through DebugDraw3D, which is already a batch - a hundred
    // colliders cost one draw call, not a hundred.
    virtual void debugDraw(const glm::mat4& transform, Color color) const = 0;
};

class SphereShape final : public CollisionShape
{
public:
    explicit SphereShape(f32 radius);

    ShapeType type() const override
    {
        return ShapeType::Sphere;
    }
    glm::mat3 inertia(f32 mass) const override;
    AABB bounds(const glm::mat4& transform) const override;
    glm::vec3 support(const glm::mat4& transform, const glm::vec3& direction) const override;
    void debugDraw(const glm::mat4& transform, Color color) const override;

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
    explicit BoxShape(const glm::vec3& halfExtents);

    ShapeType type() const override
    {
        return ShapeType::Box;
    }
    glm::mat3 inertia(f32 mass) const override;
    AABB bounds(const glm::mat4& transform) const override;
    glm::vec3 support(const glm::mat4& transform, const glm::vec3& direction) const override;
    void debugDraw(const glm::mat4& transform, Color color) const override;

    const glm::vec3& halfExtents() const
    {
        return mHalfExtents;
    }
    // The eight corners in world space, in the order the face table below
    // indexes them.
    void corners(const glm::mat4& transform, glm::vec3 out[8]) const;
    // Outward normal and the four corner indices of face `index`, 0..5.
    static const u8* faceCorners(u32 index);
    static glm::vec3 faceNormal(const glm::mat4& transform, u32 index);

private:
    glm::vec3 mHalfExtents;
};

// A segment along the body's local Y with a radius around it. Everything a
// capsule does is the sphere case applied to the closest point on that
// segment, which is why it is the cheapest shape to be right about and the
// one a character controller wants: no corners to catch on a step.
class CapsuleShape final : public CollisionShape
{
public:
    // `halfHeight` is half the length of the SEGMENT, not of the whole
    // capsule - the total height is 2*(halfHeight + radius). Measuring the
    // segment is what keeps the two ends spherical whatever the radius.
    CapsuleShape(f32 radius, f32 halfHeight);

    ShapeType type() const override
    {
        return ShapeType::Capsule;
    }
    glm::mat3 inertia(f32 mass) const override;
    AABB bounds(const glm::mat4& transform) const override;
    glm::vec3 support(const glm::mat4& transform, const glm::vec3& direction) const override;
    void debugDraw(const glm::mat4& transform, Color color) const override;

    f32 radius() const
    {
        return mRadius;
    }
    f32 halfHeight() const
    {
        return mHalfHeight;
    }
    // The two ends of the inner segment, world space.
    void segment(const glm::mat4& transform, glm::vec3& lower, glm::vec3& upper) const;

private:
    f32 mRadius;
    f32 mHalfHeight;
};

class PlaneShape final : public CollisionShape
{
public:
    PlaneShape(const glm::vec3& normal, f32 constant = 0.0f);

    ShapeType type() const override
    {
        return ShapeType::Plane;
    }
    glm::mat3 inertia(f32 mass) const override;
    AABB bounds(const glm::mat4& transform) const override;
    glm::vec3 support(const glm::mat4& transform, const glm::vec3& direction) const override;
    void debugDraw(const glm::mat4& transform, Color color) const override;

    const glm::vec3& normal() const
    {
        return mNormal;
    }
    f32 constant() const
    {
        return mConstant;
    }

private:
    glm::vec3 mNormal;
    f32 mConstant;
};

// Which part of a triangle a closest-point query landed on. Edge i runs from
// vertex i to vertex i+1.
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

// One triangle of a TrimeshShape, in that mesh's own space. Built and thrown
// away inside the trimesh narrowphase, never owned by a body: it has no
// volume, so no mass and no inertia to speak of.
class TriangleShape final : public CollisionShape
{
public:
    TriangleShape(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, u8 sharedEdges = 0);

    ShapeType type() const override
    {
        return ShapeType::Triangle;
    }
    glm::mat3 inertia(f32 mass) const override;
    AABB bounds(const glm::mat4& transform) const override;
    glm::vec3 support(const glm::mat4& transform, const glm::vec3& direction) const override;
    void debugDraw(const glm::mat4& transform, Color color) const override;

    const glm::vec3& vertex(u32 index) const
    {
        return mVertices[index];
    }
    // Unnormalized when the triangle is degenerate - callers check the length
    // rather than trusting a normalize() of nothing.
    glm::vec3 rawNormal() const;

    // An edge another triangle also uses is inside the surface, not on its
    // rim. A contact there must be pushed out along the face, never along the
    // edge: the edge direction is what catches a character walking over a
    // seam and stops him dead on flat ground.
    bool edgeIsShared(u32 edge) const
    {
        return (mSharedEdges & (1u << edge)) != 0;
    }
    // True when the normal for `feature` has to be taken from the face rather
    // than from the closest-point direction.
    bool featureIsInternal(TriangleFeature feature) const;

private:
    glm::vec3 mVertices[3];
    u8 mSharedEdges = 0;
};

// A static concave mesh. Dynamics has no meaning for one - a concave body has
// no single inertia tensor - so a body carrying this must be static or
// kinematic, which Scene::addBody() enforces.
class TrimeshShape final : public CollisionShape
{
public:
    // Copies both arrays: a shape outlives whatever MeshData it was built
    // from, and the tree indexes into this copy.
    TrimeshShape(const glm::vec3* vertices, u32 vertexCount, const u32* indices, u32 indexCount);

    ShapeType type() const override
    {
        return ShapeType::Trimesh;
    }
    glm::mat3 inertia(f32 mass) const override;
    AABB bounds(const glm::mat4& transform) const override;
    glm::vec3 support(const glm::mat4& transform, const glm::vec3& direction) const override;
    void debugDraw(const glm::mat4& transform, Color color) const override;

    // One line per triangle, from its centre along the normal the WINDING
    // gives - which is the normal collision actually uses. The normals stored
    // on the mesh are the renderer's and can disagree: a model can light
    // perfectly and still be walked through.
    void debugDrawFaceNormals(const glm::mat4& transform, f32 length, Color color,
                              Color flippedColor) const;

    // Triangles wound against their neighbours. Two triangles sharing an edge
    // should traverse it in opposite directions; when they do not, one of
    // them faces backwards and a one-sided sweep passes straight through it.
    void windingErrors(std::vector<u32>& out) const;

    u32 triangleCount() const
    {
        return static_cast<u32>(mIndices.size() / 3);
    }
    TriangleShape triangle(u32 index) const;
    // Triangles whose bounds meet `box`, expressed in this mesh's own space.
    void query(const AABB& localBox, std::vector<u32>& out) const;

    // Geometry queries for everything that is not the solver: picking with
    // the mouse, a line of sight, deciding where a prop can stand. All in
    // this mesh's own space - the caller brings the ray or the sphere in.
    struct RayHit
    {
        u32 triangle = 0;
        f32 distance = 0.0f;
        glm::vec3 point{0.0f};
        // Face normal, unit length, as wound in the mesh.
        glm::vec3 normal{0.0f, 1.0f, 0.0f};
    };

    struct SweepHit
    {
        u32 triangle = 0;
        // Fraction of `velocity` travelled before contact, in [0, 1]. Negative
        // means the shape already overlapped, and the value is the depth.
        f32 t = 0.0f;
        glm::vec3 normal{0.0f, 1.0f, 0.0f};
    };

    // Moves an ellipsoid of `radii` from `centre` along `velocity` and reports
    // the first thing it meets.
    //
    // A sweep, not a push-out: a character needs "how far can I go before I
    // hit" and "what did I land on", and no amount of pushing out gives
    // either.
    bool sweepEllipsoid(const glm::vec3& localCentre, const glm::vec3& radii,
                        const glm::vec3& velocity, SweepHit& hit) const;
    bool sweepSphere(const glm::vec3& localCentre, f32 radius, const glm::vec3& velocity,
                     SweepHit& hit) const;

    // Third-person camera collision: sweeps a sphere of `radius` from `from`
    // (the camera anchor, usually the player's eye) to `to` (the desired
    // camera position) and returns where the camera can actually sit. A wall
    // on the way pulls the camera back to the first contact, so it hugs the
    // surface instead of clipping through it - and because `from` is the
    // player, the camera can never lose him, it only gets closer. `margin`
    // keeps the camera sphere clear of the surface so it does not z-fight.
    // All in this mesh's own space, like the other queries.
    glm::vec3 slideCamera(const glm::vec3& from, const glm::vec3& to, f32 radius,
                          f32 margin = 0.02f) const;

    // Nearest triangle the ray meets within `maxDistance`, or false.
    bool raycast(const Ray& localRay, f32 maxDistance, RayHit& hit) const;
    // Triangles actually within `radius` of `centre`, not merely whose boxes
    // are - the tree narrows it down, this confirms.
    void overlapSphere(const glm::vec3& localCentre, f32 radius, std::vector<u32>& out) const;

private:
    void buildAdjacency();

    std::vector<glm::vec3> mVertices;
    std::vector<u32> mIndices;
    // Three bits per triangle: edge i is shared with another triangle.
    std::vector<u8> mSharedEdges;
    BoundsTree mTree;
    AABB mBounds;
};

// An arbitrary convex polyhedron: the vertices/edges/faces of a
// ConvexHullComputer half-edge mesh, kept as three flat arrays rather than a
// pointer to one - a shape outlives whatever Shard or ConvexHullComputer it
// was built from. Meant for Voronoi shatter debris: bodies with a handful of
// faces and tens of vertices, not for anything a broadphase would call large.
class ConvexHullShape final : public CollisionShape
{
public:
    using Edge = Radion::Geometry::ConvexHullComputer::Edge;

    // `faces` holds one edge index per face, matching ConvexHullComputer's
    // own convention - walking that edge with getNextEdgeOfFace() visits the
    // whole face in order.
    ConvexHullShape(const glm::vec3* vertices, u32 vertexCount, const Edge* edges, u32 edgeCount,
                    const int* faces, u32 faceCount);
    // Copies a shatter shard's own hull straight in - the shard's vertices are
    // already relative to its own centroid, which is exactly the local space
    // a CollisionShape is expected to be defined in.
    explicit ConvexHullShape(const Radion::Geometry::Shard& shard);

    ShapeType type() const override
    {
        return ShapeType::ConvexHull;
    }
    glm::mat3 inertia(f32 mass) const override;
    AABB bounds(const glm::mat4& transform) const override;
    glm::vec3 support(const glm::mat4& transform, const glm::vec3& direction) const override;
    void debugDraw(const glm::mat4& transform, Color color) const override;

    const std::vector<glm::vec3>& vertices() const
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
    std::vector<glm::vec3> mVertices;
    std::vector<Edge> mEdges;
    std::vector<int> mFaces;
};

// Closest point on the segment [a,b] to `point`.
glm::vec3 closestPointOnSegment(const glm::vec3& a, const glm::vec3& b, const glm::vec3& point);

// Closest point on triangle [a,b,c] to `point`, by Voronoi regions rather
// than by projecting and clamping.
// The region it lands in is also which feature it lands on, so `feature` costs
// nothing to report and is what tells an edge contact from a face one.
glm::vec3 closestPointOnTriangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                                 const glm::vec3& point, TriangleFeature* feature = nullptr);

// Closest pair of points between two segments. Handles the parallel and
// degenerate cases, which is the whole difficulty - two capsules lying side
// by side are exactly the case a naive solve divides by zero on.
void closestPointsBetweenSegments(const glm::vec3& p1, const glm::vec3& q1, const glm::vec3& p2,
                                  const glm::vec3& q2, glm::vec3& c1, glm::vec3& c2);

} // namespace Radion::Physics

#endif // RADION_PHYSICS_COLLISION_SHAPE_H
