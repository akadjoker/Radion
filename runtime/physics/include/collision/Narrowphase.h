#ifndef RADION_PHYSICS_COLLISION_NARROWPHASE_H
#define RADION_PHYSICS_COLLISION_NARROWPHASE_H

#include "collision/CollisionShape.h"

namespace Radion::Physics
{

// One point where two shapes touch. The impulses persist between steps so the
// solver can start from what it needed last time (warm starting) instead of
// finding the same answer again from zero every frame - it is the difference
// between a stack that settles and one that sinks.
struct ContactPoint
{
    glm::vec3 position{0.0f}; // world, midway between the two surfaces
    f32 penetration = 0.0f;   // positive when overlapping
    f32 normalImpulse = 0.0f;
    f32 tangentImpulse[2] = {0.0f, 0.0f};
    // Separation speed the solver aims for, from the approach speed measured
    // before any impulse is applied. Not persistent state like the impulses -
    // ContactSolver::solve() recomputes it every step, and it is zero for
    // everything that is not bouncing.
    f32 velocityBias = 0.0f;
};

// The at-most-four points of one contact patch, plus the shared normal.
// Four is what a box face against a box face needs; more adds nothing a
// solver can use and costs iterations.
struct ContactManifold
{
    static constexpr u32 MaxPoints = 4;

    // Points from A towards B: pushing B along it separates them.
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    // Perpendicular to the normal, for the two friction directions.
    glm::vec3 tangent[2] = {glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)};
    ContactPoint points[MaxPoints];
    u32 count = 0;

    void buildTangents();
};

struct ShapeRayHit
{
    f32 distance = 0.0f;
    glm::vec3 point{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
};

// Separating Axis Test and contact generation, one static entry point per
// shape pair. Nothing here touches a RigidBody: collision is a question about
// two transformed shapes, and keeping it that way is what lets it be tested
// without building a world.
class Narrowphase
{
public:
    // `margin` reports shapes that are close but not yet touching, with a
    // NEGATIVE penetration saying how far apart they are. Zero is plain
    // overlap-only detection.
    //
    // A margin matters because the position pass pushes a body just clear of
    // the surface it landed on. With overlap-only detection the contact then
    // vanishes for as long as it takes gravity to pull it back - measured at
    // 33 steps for a box dropped 2.5 m - taking the pair's accumulated
    // impulses and its enter/stay/exit state with it. The solver ignores a
    // negative penetration for position and produces no impulse while the
    // bodies are separating, so a speculative contact costs nothing and
    // keeps the pair alive across the gap.
    static bool collide(const CollisionShape& a, const glm::mat4& transformA,
                        const CollisionShape& b, const glm::mat4& transformB,
                        ContactManifold& out, f32 margin = 0.0f);

    static bool sphereSphere(const SphereShape& a, const glm::mat4& transformA,
                             const SphereShape& b, const glm::mat4& transformB,
                             ContactManifold& out, f32 margin = 0.0f);
    static bool sphereBox(const SphereShape& a, const glm::mat4& transformA, const BoxShape& b,
                          const glm::mat4& transformB, ContactManifold& out, f32 margin = 0.0f);
    static bool boxBox(const BoxShape& a, const glm::mat4& transformA, const BoxShape& b,
                       const glm::mat4& transformB, ContactManifold& out, f32 margin = 0.0f);

    // A capsule against a sphere is the sphere case moved to the closest
    // point on its segment, and against another capsule it is the same with
    // both segments - so these are exact, not approximations.
    static bool capsuleSphere(const CapsuleShape& a, const glm::mat4& transformA,
                              const SphereShape& b, const glm::mat4& transformB,
                              ContactManifold& out, f32 margin = 0.0f);
    static bool capsuleCapsule(const CapsuleShape& a, const glm::mat4& transformA,
                               const CapsuleShape& b, const glm::mat4& transformB,
                               ContactManifold& out, f32 margin = 0.0f);
    // Separation by SAT over the box's faces, the capsule's axis and their
    // cross products; contact by clipping the capsule's segment to the box
    // face when the two are parallel, so a capsule lying on a surface gets
    // TWO points and stays put instead of pivoting on one.
    static bool capsuleBox(const CapsuleShape& a, const glm::mat4& transformA, const BoxShape& b,
                           const glm::mat4& transformB, ContactManifold& out, f32 margin = 0.0f);
    static bool convexPlane(const CollisionShape& a, const glm::mat4& transformA,
                            const PlaneShape& b, const glm::mat4& transformB,
                            ContactManifold& out, f32 margin = 0.0f);

    // The same face+edge SAT boxBox() uses, generalized from six fixed faces
    // to however many a ConvexHullShape has: its own face normals stand in
    // for the box's three, and its own edge directions stand in for the
    // box's three when a cross-product axis is needed.
    static bool convexHullSphere(const ConvexHullShape& a, const glm::mat4& transformA,
                                 const SphereShape& b, const glm::mat4& transformB,
                                 ContactManifold& out, f32 margin = 0.0f);
    static bool convexHullBox(const ConvexHullShape& a, const glm::mat4& transformA,
                              const BoxShape& b, const glm::mat4& transformB, ContactManifold& out,
                              f32 margin = 0.0f);
    static bool convexHullCapsule(const ConvexHullShape& a, const glm::mat4& transformA,
                                  const CapsuleShape& b, const glm::mat4& transformB,
                                  ContactManifold& out, f32 margin = 0.0f);
    static bool convexHullConvexHull(const ConvexHullShape& a, const glm::mat4& transformA,
                                     const ConvexHullShape& b, const glm::mat4& transformB,
                                     ContactManifold& out, f32 margin = 0.0f);

    // Triangle cases, normal pointing from the convex towards the triangle.
    // A contact landing on a shared edge takes that edge's direction, not the
    // face's: without the neighbouring triangles' normals there is no way to
    // tell an outer edge from an interior seam, so a body sliding across a
    // mesh can still catch on one. Adjacency would fix it and is not here.
    static bool sphereTriangle(const SphereShape& a, const glm::mat4& transformA,
                               const TriangleShape& b, const glm::mat4& transformB,
                               ContactManifold& out, f32 margin = 0.0f);
    static bool boxTriangle(const BoxShape& a, const glm::mat4& transformA,
                            const TriangleShape& b, const glm::mat4& transformB,
                            ContactManifold& out, f32 margin = 0.0f);
    static bool capsuleTriangle(const CapsuleShape& a, const glm::mat4& transformA,
                                const TriangleShape& b, const glm::mat4& transformB,
                                ContactManifold& out, f32 margin = 0.0f);

    // One manifold PER TOUCHING TRIANGLE, appended to `out`. Not one merged
    // manifold: a manifold carries a single normal, and a box wedged into a
    // corner needs one normal per face it rests on or the solver pushes it
    // out along an average that belongs to neither.
    static bool convexTrimesh(const CollisionShape& convex, const glm::mat4& convexTransform,
                              const TrimeshShape& mesh, const glm::mat4& meshTransform,
                              std::vector<ContactManifold>& out, f32 margin = 0.0f);

    static bool raycast(const CollisionShape& shape, const glm::mat4& transform, const Ray& ray,
                        f32 maxDistance, ShapeRayHit& hit);

    static bool overlapSphere(const CollisionShape& shape, const glm::mat4& transform,
                              const glm::vec3& centre, f32 radius);
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_COLLISION_NARROWPHASE_H
