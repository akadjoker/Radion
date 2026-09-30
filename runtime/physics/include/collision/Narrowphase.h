#ifndef RADION_PHYSICS_COLLISION_NARROWPHASE_H
#define RADION_PHYSICS_COLLISION_NARROWPHASE_H

#include "collision/CollisionShape.h"

namespace Radion::Physics
{

// One contact point. Impulses persist between steps for warm starting (lets a stack settle instead of sink).
struct ContactPoint
{
    Math::vec3 position{0.0f}; // world, midway between the two surfaces
    f32 penetration = 0.0f;   // positive when overlapping
    f32 normalImpulse = 0.0f;
    f32 tangentImpulse[2] = {0.0f, 0.0f};
    // Target separation speed from the pre-impulse approach speed; recomputed each step by ContactSolver::solve(), zero unless bouncing.
    f32 velocityBias = 0.0f;
};

// At most four points (a box face against a box face); more adds nothing.
struct ContactManifold
{
    static constexpr u32 MaxPoints = 4;

    // Points from A towards B: pushing B along it separates them.
    Math::vec3 normal{0.0f, 1.0f, 0.0f};
    Math::vec3 tangent[2] = {Math::vec3(1.0f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f)};
    ContactPoint points[MaxPoints];
    u32 count = 0;

    void buildTangents();
};

struct ShapeRayHit
{
    f32 distance = 0.0f;
    Math::vec3 point{0.0f};
    Math::vec3 normal{0.0f, 1.0f, 0.0f};
};

// SAT and contact generation, one static entry point per shape pair; nothing here touches a RigidBody.
class Narrowphase
{
public:
    // `margin` reports near-but-not-touching shapes with NEGATIVE penetration = gap; 0 is overlap-only.
    // The position pass pushes a body just clear, and without a margin the contact vanishes until gravity returns (33 steps for a
    // 2.5 m box drop), losing accumulated impulses and enter/stay/exit state. The solver ignores negative penetration and applies
    // no impulse while separating, so speculative contacts are free.
    static bool collide(const CollisionShape& a, const Math::mat4& transformA,
                        const CollisionShape& b, const Math::mat4& transformB,
                        ContactManifold& out, f32 margin = 0.0f);

    static bool sphereSphere(const SphereShape& a, const Math::mat4& transformA,
                             const SphereShape& b, const Math::mat4& transformB,
                             ContactManifold& out, f32 margin = 0.0f);
    static bool sphereBox(const SphereShape& a, const Math::mat4& transformA, const BoxShape& b,
                          const Math::mat4& transformB, ContactManifold& out, f32 margin = 0.0f);
    static bool boxBox(const BoxShape& a, const Math::mat4& transformA, const BoxShape& b,
                       const Math::mat4& transformB, ContactManifold& out, f32 margin = 0.0f);

    // Capsule cases reduce to the sphere case at the closest segment point(s): exact, not approximations.
    static bool capsuleSphere(const CapsuleShape& a, const Math::mat4& transformA,
                              const SphereShape& b, const Math::mat4& transformB,
                              ContactManifold& out, f32 margin = 0.0f);
    static bool capsuleCapsule(const CapsuleShape& a, const Math::mat4& transformA,
                               const CapsuleShape& b, const Math::mat4& transformB,
                               ContactManifold& out, f32 margin = 0.0f);
    // SAT over box faces, capsule axis and cross products; a parallel capsule is clipped to the box face, giving TWO points so it does not pivot.
    static bool capsuleBox(const CapsuleShape& a, const Math::mat4& transformA, const BoxShape& b,
                           const Math::mat4& transformB, ContactManifold& out, f32 margin = 0.0f);
    static bool convexPlane(const CollisionShape& a, const Math::mat4& transformA,
                            const PlaneShape& b, const Math::mat4& transformB,
                            ContactManifold& out, f32 margin = 0.0f);

    // Same face+edge SAT as boxBox(), using the hull's own face normals and edge directions.
    static bool convexHullSphere(const ConvexHullShape& a, const Math::mat4& transformA,
                                 const SphereShape& b, const Math::mat4& transformB,
                                 ContactManifold& out, f32 margin = 0.0f);
    static bool convexHullBox(const ConvexHullShape& a, const Math::mat4& transformA,
                              const BoxShape& b, const Math::mat4& transformB, ContactManifold& out,
                              f32 margin = 0.0f);
    static bool convexHullCapsule(const ConvexHullShape& a, const Math::mat4& transformA,
                                  const CapsuleShape& b, const Math::mat4& transformB,
                                  ContactManifold& out, f32 margin = 0.0f);
    static bool convexHullConvexHull(const ConvexHullShape& a, const Math::mat4& transformA,
                                     const ConvexHullShape& b, const Math::mat4& transformB,
                                     ContactManifold& out, f32 margin = 0.0f);

    // Triangle cases, normal from the convex towards the triangle. A contact on a shared edge takes the edge direction: without neighbour
    // normals an outer edge cannot be told from an interior seam, so sliding can still catch (adjacency would fix it).
    static bool sphereTriangle(const SphereShape& a, const Math::mat4& transformA,
                               const TriangleShape& b, const Math::mat4& transformB,
                               ContactManifold& out, f32 margin = 0.0f);
    static bool boxTriangle(const BoxShape& a, const Math::mat4& transformA,
                            const TriangleShape& b, const Math::mat4& transformB,
                            ContactManifold& out, f32 margin = 0.0f);
    static bool capsuleTriangle(const CapsuleShape& a, const Math::mat4& transformA,
                                const TriangleShape& b, const Math::mat4& transformB,
                                ContactManifold& out, f32 margin = 0.0f);

    // One manifold PER TOUCHING TRIANGLE appended to `out`: a manifold has one normal, and a box wedged in a corner needs one per face.
    static bool convexTrimesh(const CollisionShape& convex, const Math::mat4& convexTransform,
                              const TrimeshShape& mesh, const Math::mat4& meshTransform,
                              std::vector<ContactManifold>& out, f32 margin = 0.0f);

    static bool raycast(const CollisionShape& shape, const Math::mat4& transform, const Ray& ray,
                        f32 maxDistance, ShapeRayHit& hit);

    static bool overlapSphere(const CollisionShape& shape, const Math::mat4& transform,
                              const Math::vec3& centre, f32 radius);
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_COLLISION_NARROWPHASE_H
