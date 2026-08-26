#include "PCH.h"

#include "character/CharacterBody.h"
#include "character/CharacterRigidBody.h"
#include "collision/Broadphase.h"
#include "collision/Narrowphase.h"
#include "dynamics/ContactSolver.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/PointJoint.h"
#include "dynamics/RigidBody.h"

#include "VoronoiShatter.h"

#include <cstdio>

using namespace Radion;
using namespace Radion::Physics;
using namespace Radion::Geometry;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "CollisionTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool near(f32 a, f32 b, f32 epsilon = 1e-4f)
{
    return std::abs(a - b) <= epsilon;
}

bool near(const Math::Vec3& a, const Math::Vec3& b, f32 epsilon = 1e-4f)
{
    return glm::length(a - b) <= epsilon;
}

Math::Mat4 at(const Math::Vec3& position, const Math::Quaternion& rotation = Math::Quaternion(1, 0, 0, 0))
{
    Math::Mat4 transform = glm::mat4_cast(rotation);
    transform[3] = Math::Vec4(position, 1.0f);
    return transform;
}

AABB boxAt(const Math::Vec3& center, f32 half)
{
    AABB bounds;
    bounds.min = center - Math::Vec3(half);
    bounds.max = center + Math::Vec3(half);
    return bounds;
}

// A cube's convex hull, built through the real ConvexHullComputer the way
// VoronoiShatter itself builds one, so the half-edge structure the tests
// exercise is the same kind ConvexHullShape gets handed from an actual
// shatter.
Shard buildCubeShard(f32 halfExtent)
{
    std::vector<Math::Vec3> corners;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2)
                corners.push_back(Math::Vec3(sx, sy, sz) * halfExtent);

    ConvexHullComputer computer;
    computer.compute(&corners[0].x, sizeof(Math::Vec3), static_cast<int>(corners.size()), 0.0f,
                     0.0f);

    Shard shard;
    shard.vertices = computer.vertices;
    shard.edges = computer.edges;
    shard.faces = computer.faces;
    return shard;
}

// ----------------------------------------------------------------- support

void testSupportAndBounds()
{
    const BoxShape box(Math::Vec3(1.0f, 2.0f, 3.0f));
    const Math::Mat4 identity = at(Math::Vec3(0.0f));

    // The support point along an axis is the corner in that direction.
    CHECK(near(box.support(identity, Math::Vec3(1, 1, 1)), Math::Vec3(1, 2, 3)));
    CHECK(near(box.support(identity, Math::Vec3(-1, 1, -1)), Math::Vec3(-1, 2, -3)));

    f32 minimum = 0.0f;
    f32 maximum = 0.0f;
    box.project(identity, Math::Vec3(0, 1, 0), minimum, maximum);
    CHECK(near(minimum, -2.0f));
    CHECK(near(maximum, 2.0f));

    // Rotated a quarter turn about z, x and y swap.
    const Math::Mat4 turned =
        at(Math::Vec3(0.0f), glm::angleAxis(glm::half_pi<f32>(), Math::Vec3(0, 0, 1)));
    box.project(turned, Math::Vec3(0, 1, 0), minimum, maximum);
    CHECK(near(maximum, 1.0f, 1e-3f));

    // A rotated box's AABB has to grow: a 45 degree turn puts a corner
    // furthest out, at half*sqrt(2) on each axis.
    const BoxShape cube(Math::Vec3(1.0f));
    const Math::Mat4 diagonal =
        at(Math::Vec3(0.0f), glm::angleAxis(glm::quarter_pi<f32>(), Math::Vec3(0, 0, 1)));
    const AABB bounds = cube.bounds(diagonal);
    CHECK(near(bounds.max.x, std::sqrt(2.0f), 1e-3f));
    CHECK(near(bounds.max.z, 1.0f, 1e-3f));

    const SphereShape sphere(2.5f);
    CHECK(near(sphere.support(at(Math::Vec3(1, 0, 0)), Math::Vec3(0, 3, 0)), Math::Vec3(1, 2.5f, 0)));
    // A sphere's AABB does not care how it is turned.
    CHECK(near(sphere.bounds(diagonal).max, Math::Vec3(2.5f)));
}

// --------------------------------------------------------------- broadphase

void testBroadphasePairs()
{
    Broadphase broadphase;
    BroadphaseProxy proxy;

    proxy.id = 1;
    proxy.bounds = boxAt(Math::Vec3(0.0f), 1.0f);
    broadphase.add(proxy);
    proxy.id = 2;
    proxy.bounds = boxAt(Math::Vec3(1.5f, 0.0f, 0.0f), 1.0f); // overlaps 1
    broadphase.add(proxy);
    proxy.id = 3;
    proxy.bounds = boxAt(Math::Vec3(50.0f, 0.0f, 0.0f), 1.0f); // far away
    broadphase.add(proxy);

    std::vector<BroadphasePair> pairs;
    broadphase.findPairs(pairs);
    CHECK(pairs.size() == 1);
    if (pairs.size() == 1)
    {
        CHECK(pairs[0].a == 1);
        CHECK(pairs[0].b == 2);
    }

    // The sweep must have picked x, which is the axis they are spread over.
    CHECK(broadphase.sweepAxis() == 0);
}

void testBroadphaseSkipsStaticPairs()
{
    Broadphase broadphase;
    BroadphaseProxy proxy;
    proxy.movable = false;

    proxy.id = 1;
    proxy.bounds = boxAt(Math::Vec3(0.0f), 1.0f);
    broadphase.add(proxy);
    proxy.id = 2;
    proxy.bounds = boxAt(Math::Vec3(0.5f, 0.0f, 0.0f), 1.0f);
    broadphase.add(proxy);

    std::vector<BroadphasePair> pairs;
    broadphase.findPairs(pairs);
    // Two overlapping statics are still two things that cannot move.
    CHECK(pairs.empty());

    // One of them moving brings the pair back.
    Broadphase mixed;
    proxy.id = 1;
    proxy.movable = false;
    proxy.bounds = boxAt(Math::Vec3(0.0f), 1.0f);
    mixed.add(proxy);
    proxy.id = 2;
    proxy.movable = true;
    proxy.bounds = boxAt(Math::Vec3(0.5f, 0.0f, 0.0f), 1.0f);
    mixed.add(proxy);
    mixed.findPairs(pairs);
    CHECK(pairs.size() == 1);
}

void testBroadphaseLayers()
{
    Broadphase broadphase;
    BroadphaseProxy proxy;
    proxy.bounds = boxAt(Math::Vec3(0.0f), 1.0f);

    proxy.id = 1;
    proxy.filter.group = 1;
    proxy.filter.mask = 2; // only sees group 2
    broadphase.add(proxy);
    proxy.id = 2;
    proxy.filter.group = 4; // not 2
    proxy.filter.mask = 0xFFFFFFFFu;
    broadphase.add(proxy);

    std::vector<BroadphasePair> pairs;
    broadphase.findPairs(pairs);
    // One side refusing is enough - both halves have to agree.
    CHECK(pairs.empty());
}

void testBroadphaseFindsEveryOverlapInAStack()
{
    // A column along y, each touching the next. The sweep has to pick y and
    // still find all nine neighbouring pairs - an axis chosen wrongly, or a
    // break taken too early, silently drops contacts and the stack falls
    // through itself.
    Broadphase broadphase;
    BroadphaseProxy proxy;
    for (u32 i = 0; i < 10; ++i)
    {
        proxy.id = i;
        proxy.bounds = boxAt(Math::Vec3(0.0f, static_cast<f32>(i) * 0.9f, 0.0f), 0.5f);
        broadphase.add(proxy);
    }
    std::vector<BroadphasePair> pairs;
    broadphase.findPairs(pairs);
    CHECK(broadphase.sweepAxis() == 1);
    CHECK(pairs.size() == 9);
    for (const BroadphasePair& pair : pairs)
        CHECK(pair.b == pair.a + 1);
}


// --------------------------------------------------------------- narrowphase

void testSphereSphere()
{
    const SphereShape a(1.0f);
    const SphereShape b(1.0f);
    ContactManifold manifold;

    CHECK(!Narrowphase::collide(a, at(Math::Vec3(0.0f)), b, at(Math::Vec3(3.0f, 0, 0)), manifold));

    CHECK(Narrowphase::collide(a, at(Math::Vec3(0.0f)), b, at(Math::Vec3(1.5f, 0, 0)), manifold));
    CHECK(manifold.count == 1);
    // Normal points from A to B.
    CHECK(near(manifold.normal, Math::Vec3(1, 0, 0)));
    CHECK(near(manifold.points[0].penetration, 0.5f));
    // Contact sits between the two surfaces: A's surface is at x=1, B's at
    // x=0.5, so the midpoint is 0.75.
    CHECK(near(manifold.points[0].position.x, 0.75f));

    // Exactly touching is not a collision.
    CHECK(!Narrowphase::collide(a, at(Math::Vec3(0.0f)), b, at(Math::Vec3(2.0f, 0, 0)), manifold));

    // Concentric: must not divide by zero, and must still report something
    // to push apart along.
    CHECK(Narrowphase::collide(a, at(Math::Vec3(0.0f)), b, at(Math::Vec3(0.0f)), manifold));
    CHECK(near(glm::length(manifold.normal), 1.0f));
    CHECK(std::isfinite(manifold.points[0].penetration));
}

void testSphereBox()
{
    const SphereShape sphere(1.0f);
    const BoxShape box(Math::Vec3(1.0f));
    ContactManifold manifold;

    // Sphere above the box's +y face, overlapping by 0.25.
    CHECK(Narrowphase::collide(sphere, at(Math::Vec3(0.0f, 1.75f, 0.0f)), box, at(Math::Vec3(0.0f)),
                               manifold));
    CHECK(manifold.count == 1);
    // From sphere (A) to box (B) is downwards.
    CHECK(near(manifold.normal, Math::Vec3(0, -1, 0)));
    CHECK(near(manifold.points[0].penetration, 0.25f));
    CHECK(near(manifold.points[0].position, Math::Vec3(0, 1, 0)));

    // Clear of a corner diagonally is a miss even though the AABBs overlap -
    // this is exactly what the broadphase cannot decide on its own.
    CHECK(!Narrowphase::collide(sphere, at(Math::Vec3(1.8f, 1.8f, 1.8f)), box, at(Math::Vec3(0.0f)),
                                manifold));

    // Centre inside the box: it has to come out through the nearest face,
    // and no division by a zero distance.
    CHECK(Narrowphase::collide(sphere, at(Math::Vec3(0.0f, 0.8f, 0.0f)), box, at(Math::Vec3(0.0f)),
                               manifold));
    CHECK(near(manifold.normal, Math::Vec3(0, -1, 0)));
    CHECK(manifold.points[0].penetration > 1.0f);

    // The box-first ordering has to give the mirrored normal, not a
    // different answer.
    ContactManifold flipped;
    CHECK(Narrowphase::collide(box, at(Math::Vec3(0.0f)), sphere, at(Math::Vec3(0.0f, 1.75f, 0.0f)),
                               flipped));
    CHECK(near(flipped.normal, Math::Vec3(0, 1, 0)));
    CHECK(near(flipped.points[0].penetration, 0.25f));
}

void testBoxBoxFaceContact()
{
    const BoxShape a(Math::Vec3(1.0f));
    const BoxShape b(Math::Vec3(1.0f));
    ContactManifold manifold;

    CHECK(!Narrowphase::collide(a, at(Math::Vec3(0.0f)), b, at(Math::Vec3(2.5f, 0, 0)), manifold));

    // Stacked with 0.2 of overlap: a face against a face is four points, not
    // one - a box resting on one contact point tips over.
    CHECK(
        Narrowphase::collide(a, at(Math::Vec3(0.0f)), b, at(Math::Vec3(0.0f, 1.8f, 0.0f)), manifold));
    CHECK(manifold.count == 4);
    CHECK(near(manifold.normal, Math::Vec3(0, 1, 0)));
    for (u32 i = 0; i < manifold.count; ++i)
    {
        CHECK(near(manifold.points[i].penetration, 0.2f, 1e-3f));
        CHECK(near(manifold.points[i].position.y, 1.0f, 1e-3f));
    }

    // The four must actually span the face rather than bunch in a corner.
    f32 spread = 0.0f;
    for (u32 i = 0; i < manifold.count; ++i)
        for (u32 j = i + 1; j < manifold.count; ++j)
            spread = glm::max(
                spread, glm::length(manifold.points[i].position - manifold.points[j].position));
    CHECK(spread > 1.5f);

    // Tangents have to be a proper frame around the normal, or friction
    // pushes in a direction that is partly the normal.
    CHECK(near(glm::dot(manifold.tangent[0], manifold.normal), 0.0f));
    CHECK(near(glm::dot(manifold.tangent[1], manifold.normal), 0.0f));
    CHECK(near(glm::dot(manifold.tangent[0], manifold.tangent[1]), 0.0f));
    CHECK(near(glm::length(manifold.tangent[0]), 1.0f));
}

void testBoxBoxEdgeContact()
{
    // Two cubes each turned 45 degrees about different axes, meeting edge to
    // edge. The separating direction is a cross product of their edges and is
    // on none of the six face normals - a SAT that tests only faces (which is
    // what the Lumos reference does, fetching the edges and never using them)
    // reports a face normal here and slides the boxes sideways instead of
    // apart.
    const BoxShape a(Math::Vec3(0.5f));
    const BoxShape b(Math::Vec3(0.5f));
    const Math::Mat4 transformA =
        at(Math::Vec3(0.0f), glm::angleAxis(glm::quarter_pi<f32>(), Math::Vec3(0, 0, 1)));
    const Math::Mat4 transformB = at(Math::Vec3(0.0f, 1.30f, 0.0f),
                                    glm::angleAxis(glm::quarter_pi<f32>(), Math::Vec3(1, 0, 0)));

    ContactManifold manifold;
    CHECK(Narrowphase::collide(a, transformA, b, transformB, manifold));
    CHECK(manifold.count >= 1);
    // They are stacked along y, so whatever axis wins, separating them has to
    // have a real upward component.
    CHECK(std::abs(manifold.normal.y) > 0.5f);
    CHECK(manifold.points[0].penetration > 0.0f);
    CHECK(near(glm::length(manifold.normal), 1.0f, 1e-3f));

    // Pulled apart along that axis, they must separate.
    const Math::Mat4 clear =
        at(Math::Vec3(0.0f, 2.5f, 0.0f), glm::angleAxis(glm::quarter_pi<f32>(), Math::Vec3(1, 0, 0)));
    CHECK(!Narrowphase::collide(a, transformA, b, clear, manifold));
}

void testBoxBoxNormalAlwaysSeparates()
{
    // Whatever the relative pose, moving B along the normal by the reported
    // penetration has to end the overlap. This is the property the solver
    // depends on, and a wrong-sign normal passes every other check.
    const BoxShape a(Math::Vec3(0.5f));
    const BoxShape b(Math::Vec3(0.7f, 0.4f, 0.6f));
    u32 tested = 0;
    for (u32 i = 0; i < 24; ++i)
    {
        const f32 angle = static_cast<f32>(i) * 0.26f;
        const Math::Vec3 axis =
            glm::normalize(Math::Vec3(std::sin(angle * 1.3f), 1.0f, std::cos(angle * 0.7f)));
        const Math::Mat4 transformA = at(Math::Vec3(0.0f), glm::angleAxis(angle, axis));
        const Math::Vec3 offset(std::cos(angle) * 0.6f, std::sin(angle * 2.0f) * 0.5f, 0.35f);
        const Math::Mat4 transformB = at(offset, glm::angleAxis(angle * 0.5f, Math::Vec3(1, 0, 0)));

        ContactManifold manifold;
        if (!Narrowphase::collide(a, transformA, b, transformB, manifold))
            continue;
        ++tested;
        // The distance that separates them is the DEEPEST point's, not the
        // first one's - though after reducePoints those are the same thing,
        // which this also checks.
        f32 deepest = 0.0f;
        for (u32 p = 0; p < manifold.count; ++p)
            deepest = glm::max(deepest, manifold.points[p].penetration);
        CHECK(near(deepest, manifold.points[0].penetration, 1e-4f));

        const Math::Mat4 pushed = at(offset + manifold.normal * (deepest + 0.02f),
                                    glm::angleAxis(angle * 0.5f, Math::Vec3(1, 0, 0)));
        ContactManifold after;
        if (Narrowphase::collide(a, transformA, b, pushed, after))
        {
            std::fprintf(stderr,
                         "  case %u: normal (%.3f %.3f %.3f) depth %.4f still overlaps by %.4f "
                         "along (%.3f %.3f %.3f)\n",
                         i, manifold.normal.x, manifold.normal.y, manifold.normal.z,
                         manifold.points[0].penetration, after.points[0].penetration,
                         after.normal.x, after.normal.y, after.normal.z);
            ++gFailures;
        }
    }
    // The sweep has to have actually produced overlaps, or this proved
    // nothing at all.
    CHECK(tested > 8);
}

// ------------------------------------------------------------------ capsule

void testSegmentHelpers()
{
    const Math::Vec3 a(0.0f, 0.0f, 0.0f);
    const Math::Vec3 b(0.0f, 4.0f, 0.0f);
    // Alongside the middle, past each end, and exactly on an end.
    CHECK(near(closestPointOnSegment(a, b, Math::Vec3(3.0f, 2.0f, 0.0f)), Math::Vec3(0, 2, 0)));
    CHECK(near(closestPointOnSegment(a, b, Math::Vec3(0.0f, 9.0f, 0.0f)), b));
    CHECK(near(closestPointOnSegment(a, b, Math::Vec3(0.0f, -9.0f, 0.0f)), a));
    // Degenerate segment must not divide by zero.
    CHECK(near(closestPointOnSegment(a, a, Math::Vec3(5.0f, 5.0f, 5.0f)), a));

    // Crossing segments: the closest pair is where they cross in xz.
    Math::Vec3 c1, c2;
    closestPointsBetweenSegments(Math::Vec3(-1, 0, 0), Math::Vec3(1, 0, 0), Math::Vec3(0, 1, -1),
                                 Math::Vec3(0, 1, 1), c1, c2);
    CHECK(near(c1, Math::Vec3(0, 0, 0)));
    CHECK(near(c2, Math::Vec3(0, 1, 0)));

    // Parallel segments have no single answer - the routine must pick one and
    // not divide by a zero determinant, which is what a naive solve does.
    closestPointsBetweenSegments(Math::Vec3(0, 0, 0), Math::Vec3(2, 0, 0), Math::Vec3(0, 1, 0),
                                 Math::Vec3(2, 1, 0), c1, c2);
    CHECK(near(glm::length(c2 - c1), 1.0f));
    CHECK(std::isfinite(c1.x));

    // Apart along their own direction: the answer is the two facing ends.
    closestPointsBetweenSegments(Math::Vec3(0, 0, 0), Math::Vec3(1, 0, 0), Math::Vec3(5, 0, 0),
                                 Math::Vec3(6, 0, 0), c1, c2);
    CHECK(near(c1, Math::Vec3(1, 0, 0)));
    CHECK(near(c2, Math::Vec3(5, 0, 0)));
}

void testCapsuleShape()
{
    const CapsuleShape capsule(0.5f, 1.0f); // segment 2 long, total height 3
    const Math::Mat4 identity = at(Math::Vec3(0.0f));

    Math::Vec3 lower, upper;
    capsule.segment(identity, lower, upper);
    CHECK(near(lower, Math::Vec3(0, -1, 0)));
    CHECK(near(upper, Math::Vec3(0, 1, 0)));

    // Support straight up is the top cap; sideways is the radius out from
    // whichever end, and both ends are equally far.
    CHECK(near(capsule.support(identity, Math::Vec3(0, 1, 0)), Math::Vec3(0, 1.5f, 0)));
    CHECK(near(capsule.support(identity, Math::Vec3(1, 0, 0)).x, 0.5f));

    const AABB bounds = capsule.bounds(identity);
    CHECK(near(bounds.max, Math::Vec3(0.5f, 1.5f, 0.5f)));

    // Laid on its side, the tall axis becomes x.
    const Math::Mat4 lying =
        at(Math::Vec3(0.0f), glm::angleAxis(glm::half_pi<f32>(), Math::Vec3(0, 0, 1)));
    const AABB sideways = capsule.bounds(lying);
    CHECK(near(sideways.max.x, 1.5f, 1e-3f));
    CHECK(near(sideways.max.y, 0.5f, 1e-3f));
}

void testCapsuleSphereAndCapsule()
{
    const CapsuleShape capsule(0.5f, 1.0f);
    const SphereShape sphere(0.5f);
    ContactManifold manifold;

    // Beside the middle of the segment: this is the sphere case, and the
    // capsule's length must not change the answer.
    CHECK(Narrowphase::collide(capsule, at(Math::Vec3(0.0f)), sphere, at(Math::Vec3(0.8f, 0, 0)),
                               manifold));
    CHECK(near(manifold.normal, Math::Vec3(1, 0, 0)));
    CHECK(near(manifold.points[0].penetration, 0.2f));

    // Off the end, the cap is a sphere at the segment's tip.
    CHECK(Narrowphase::collide(capsule, at(Math::Vec3(0.0f)), sphere, at(Math::Vec3(0, 1.8f, 0)),
                               manifold));
    CHECK(near(manifold.normal, Math::Vec3(0, 1, 0)));
    CHECK(near(manifold.points[0].penetration, 0.2f));

    // Level with the middle but beyond the radius: no contact, however long
    // the capsule is.
    CHECK(!Narrowphase::collide(capsule, at(Math::Vec3(0.0f)), sphere, at(Math::Vec3(1.2f, 0, 0)),
                                manifold));

    // Two parallel capsules side by side - the case with no single closest
    // pair, and the one a naive segment solve divides by zero on.
    const CapsuleShape other(0.5f, 1.0f);
    CHECK(Narrowphase::collide(capsule, at(Math::Vec3(0.0f)), other, at(Math::Vec3(0.8f, 0, 0)),
                               manifold));
    CHECK(near(std::abs(manifold.normal.x), 1.0f, 1e-3f));
    CHECK(near(manifold.points[0].penetration, 0.2f));
    CHECK(std::isfinite(manifold.points[0].position.x));

    // Crossed at right angles, one above the other.
    const Math::Mat4 crossed =
        at(Math::Vec3(0.0f, 0.8f, 0.0f), glm::angleAxis(glm::half_pi<f32>(), Math::Vec3(0, 0, 1)));
    CHECK(Narrowphase::collide(capsule, at(Math::Vec3(0.0f)), other, crossed, manifold));
    CHECK(manifold.points[0].penetration > 0.0f);

    // Sphere first has to give the mirrored normal, not a different answer.
    ContactManifold flipped;
    CHECK(Narrowphase::collide(sphere, at(Math::Vec3(0.8f, 0, 0)), capsule, at(Math::Vec3(0.0f)),
                               flipped));
    CHECK(near(flipped.normal, Math::Vec3(-1, 0, 0)));
    CHECK(near(flipped.points[0].penetration, 0.2f));
}

void testCapsuleBox()
{
    const CapsuleShape capsule(0.5f, 1.0f);
    const BoxShape box(Math::Vec3(4.0f, 0.5f, 4.0f));
    ContactManifold manifold;

    // Standing upright on the box: one point, on the cap.
    CHECK(Narrowphase::collide(capsule, at(Math::Vec3(0.0f, 1.9f, 0.0f)), box, at(Math::Vec3(0.0f)),
                               manifold));
    CHECK(manifold.count == 1);
    CHECK(near(manifold.normal, Math::Vec3(0, -1, 0), 1e-3f));
    CHECK(near(manifold.points[0].penetration, 0.1f, 1e-3f));

    // Lying flat on it: this MUST give two points. With one, the capsule can
    // pivot about it and rolls off a surface it should rest on.
    const Math::Mat4 lying =
        at(Math::Vec3(0.0f, 0.95f, 0.0f), glm::angleAxis(glm::half_pi<f32>(), Math::Vec3(0, 0, 1)));
    CHECK(Narrowphase::collide(capsule, lying, box, at(Math::Vec3(0.0f)), manifold));
    CHECK(manifold.count == 2);
    CHECK(near(std::abs(manifold.normal.y), 1.0f, 1e-3f));
    // The two have to be at the segment's ends, a segment length apart.
    if (manifold.count == 2)
        CHECK(near(glm::length(manifold.points[0].position - manifold.points[1].position), 2.0f,
                   1e-2f));

    // Clear above it is no contact.
    CHECK(!Narrowphase::collide(capsule, at(Math::Vec3(0.0f, 3.0f, 0.0f)), box, at(Math::Vec3(0.0f)),
                                manifold));

    // Box first, mirrored normal.
    ContactManifold flipped;
    CHECK(Narrowphase::collide(box, at(Math::Vec3(0.0f)), capsule, at(Math::Vec3(0.0f, 1.9f, 0.0f)),
                               flipped));
    CHECK(near(flipped.normal, Math::Vec3(0, 1, 0), 1e-3f));
}

void testCapsuleRestsOnGround()
{
    // A capsule lying on the ground has to stay lying: two contact points
    // hold it, one lets it rotate away.
    BoxShape groundShape(Math::Vec3(20.0f, 0.5f, 20.0f));
    CapsuleShape capsuleShape(0.5f, 1.0f);

    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));

    RigidBody capsule;
    capsule.setMass(2.0f);
    capsule.setInertiaTensor(capsuleShape.inertia(2.0f));
    capsule.setPosition(Math::Vec3(0.0f, 2.0f, 0.0f));
    capsule.setOrientation(glm::angleAxis(glm::half_pi<f32>(), Math::Vec3(0, 0, 1)));
    capsule.setDamping(0.999f, 0.999f);

    PhysicsWorld world;
    world.setGravity(Math::Vec3(0.0f, -9.81f, 0.0f));
    BodyEntry entry;
    entry.friction = 0.8f;
    entry.shape = &groundShape;
    entry.body = &ground;
    world.addBody(entry);
    entry.shape = &capsuleShape;
    entry.body = &capsule;
    world.addBody(entry);

    for (u32 i = 0; i < 600; ++i)
        world.step(1.0f / 120.0f);

    // Resting on its side, the centre sits one radius above the surface.
    if (std::abs(capsule.position().y - 0.5f) >= 0.06f)
        std::fprintf(stderr, "    capsule rest: y %.4f axis %.3f %.3f %.3f |v| %.4f\n",
                     capsule.position().y, capsule.directionToWorld(Math::Vec3(0, 1, 0)).x,
                     capsule.directionToWorld(Math::Vec3(0, 1, 0)).y,
                     capsule.directionToWorld(Math::Vec3(0, 1, 0)).z,
                     glm::length(capsule.velocity()));
    CHECK(std::abs(capsule.position().y - 0.5f) < 0.06f);
    // And it is still on its side: its local Y, which was turned onto world
    // X, must not have tipped back up.
    const Math::Vec3 axis = capsule.directionToWorld(Math::Vec3(0.0f, 1.0f, 0.0f));
    CHECK(std::abs(axis.y) < 0.25f);
    CHECK(glm::length(capsule.velocity()) < 0.3f);
}

void testSphereRollsFromFriction()
{
    // A sphere thrown along the ground has to start spinning: friction acts
    // at the contact point, which is a radius below the centre, so it is a
    // torque. Without it the ball slides like a hockey puck forever and the
    // whole point of an inertia tensor is lost.
    BoxShape groundShape(Math::Vec3(50.0f, 0.5f, 50.0f));
    SphereShape sphereShape(0.5f);

    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));

    RigidBody ball;
    ball.setMass(1.0f);
    ball.setInertiaTensor(sphereShape.inertia(1.0f));
    ball.setPosition(Math::Vec3(0.0f, 0.5f, 0.0f));
    ball.setVelocity(Math::Vec3(6.0f, 0.0f, 0.0f));
    ball.setDamping(1.0f, 1.0f);
    ball.setCanSleep(false);

    PhysicsWorld world;
    world.setGravity(Math::Vec3(0.0f, -9.81f, 0.0f));
    BodyEntry entry;
    entry.friction = 0.6f;
    entry.shape = &groundShape;
    entry.body = &ground;
    world.addBody(entry);
    entry.shape = &sphereShape;
    entry.body = &ball;
    world.addBody(entry);

    for (u32 i = 0; i < 240; ++i)
        world.step(1.0f / 120.0f);

    // Moving along +x on a floor below it spins the ball about -z.
    CHECK(ball.angularVelocity().z < -1.0f);
    // And it must roll the right way round: at rolling without slipping the
    // contact point is stationary, which means v = -w x r, giving
    // w_z = -v_x / radius. Getting the sign backwards makes a ball that
    // spins against its own travel.
    const f32 rolling = -ball.velocity().x / sphereShape.radius();
    CHECK(ball.angularVelocity().z < 0.0f && rolling < 0.0f);
    CHECK(std::abs(ball.angularVelocity().z - rolling) < std::abs(rolling) * 0.5f);
    // Friction takes speed away, never adds it or reverses it.
    CHECK(ball.velocity().x > 0.0f);
    CHECK(ball.velocity().x < 6.0f);
}

// ------------------------------------------------------------------- solver

RigidBody makeDynamic(const CollisionShape& shape, f32 mass, const Math::Vec3& position)
{
    RigidBody body;
    body.setMass(mass);
    body.setInertiaTensor(shape.inertia(mass));
    body.setPosition(position);
    body.setDamping(1.0f, 1.0f);
    body.setCanSleep(false);
    return body;
}

RigidBody makeStatic(const Math::Vec3& position)
{
    RigidBody body;
    body.setBodyType(BodyType::Static);
    body.setPosition(position);
    return body;
}

void testSolverStopsAFall()
{
    const BoxShape shape(Math::Vec3(0.5f));
    RigidBody ground = makeStatic(Math::Vec3(0.0f, -0.5f, 0.0f));
    RigidBody box = makeDynamic(shape, 1.0f, Math::Vec3(0.0f, 0.45f, 0.0f));
    box.setVelocity(Math::Vec3(0.0f, -5.0f, 0.0f));

    Contact contact;
    contact.a = &ground;
    contact.b = &box;
    contact.friction = 0.5f;
    contact.restitution = 0.0f;
    contact.manifold.normal = Math::Vec3(0, 1, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = Math::Vec3(0.0f, -0.05f, 0.0f);
    contact.manifold.points[0].penetration = 0.05f;

    ContactSolver solver;
    solver.solve(&contact, 1, 1.0f / 60.0f);

    // The downward velocity has to be gone, and the box must not have been
    // thrown upwards instead.
    CHECK(box.velocity().y > -0.01f);
    CHECK(box.velocity().y < 0.5f);
    // A static body takes nothing from the contact.
    CHECK(near(ground.velocity(), Math::Vec3(0.0f)));
    CHECK(near(ground.position(), Math::Vec3(0.0f, -0.5f, 0.0f)));
    // Position correction pushed the overlap out, up to the slop.
    CHECK(box.position().y > 0.45f);
}

void testSolverRestitution()
{
    const SphereShape shape(0.5f);
    RigidBody ground = makeStatic(Math::Vec3(0.0f, -0.5f, 0.0f));
    RigidBody ball = makeDynamic(shape, 1.0f, Math::Vec3(0.0f, 0.5f, 0.0f));
    ball.setVelocity(Math::Vec3(0.0f, -10.0f, 0.0f));

    Contact contact;
    contact.a = &ground;
    contact.b = &ball;
    contact.friction = 0.0f;
    contact.restitution = 0.5f;
    contact.manifold.normal = Math::Vec3(0, 1, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = Math::Vec3(0.0f, 0.0f, 0.0f);
    contact.manifold.points[0].penetration = 0.01f;

    ContactSolver solver;
    solver.solve(&contact, 1, 1.0f / 60.0f);

    // Half the approach speed comes back, not all of it and not none.
    CHECK(ball.velocity().y > 3.0f);
    CHECK(ball.velocity().y < 7.0f);
}

void testSolverMomentumBetweenDynamics()
{
    // Head-on, equal masses, no restitution: they must end at the same speed,
    // and the total momentum must be what it was.
    const SphereShape shape(0.5f);
    RigidBody left = makeDynamic(shape, 2.0f, Math::Vec3(-0.49f, 0.0f, 0.0f));
    RigidBody right = makeDynamic(shape, 2.0f, Math::Vec3(0.49f, 0.0f, 0.0f));
    left.setVelocity(Math::Vec3(4.0f, 0.0f, 0.0f));
    right.setVelocity(Math::Vec3(-4.0f, 0.0f, 0.0f));

    const Math::Vec3 before = left.velocity() * 2.0f + right.velocity() * 2.0f;

    Contact contact;
    contact.a = &left;
    contact.b = &right;
    contact.friction = 0.0f;
    contact.restitution = 0.0f;
    contact.manifold.normal = Math::Vec3(1, 0, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = Math::Vec3(0.0f);
    contact.manifold.points[0].penetration = 0.02f;

    ContactSolver solver;
    solver.solve(&contact, 1, 1.0f / 60.0f);

    const Math::Vec3 after = left.velocity() * 2.0f + right.velocity() * 2.0f;
    CHECK(near(before, after, 1e-2f));
    CHECK(near(left.velocity().x, right.velocity().x, 1e-2f));
}

void testSolverFrictionStopsSliding()
{
    const BoxShape shape(Math::Vec3(0.5f));
    RigidBody ground = makeStatic(Math::Vec3(0.0f, -0.5f, 0.0f));
    RigidBody box = makeDynamic(shape, 1.0f, Math::Vec3(0.0f, 0.5f, 0.0f));
    box.setVelocity(Math::Vec3(3.0f, -1.0f, 0.0f));

    Contact contact;
    contact.a = &ground;
    contact.b = &box;
    contact.friction = 1.0f;
    contact.restitution = 0.0f;
    contact.manifold.normal = Math::Vec3(0, 1, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = Math::Vec3(0.0f);
    contact.manifold.points[0].penetration = 0.01f;

    ContactSolver solver;
    const f32 before = box.velocity().x;
    for (u32 i = 0; i < 20; ++i)
    {
        contact.manifold.points[0].penetration = 0.01f;
        solver.solve(&contact, 1, 1.0f / 60.0f);
    }
    // Friction has to take the sideways speed away, and never reverse it -
    // a friction impulse that overshoots drives the box backwards.
    CHECK(box.velocity().x < before);
    CHECK(box.velocity().x >= -0.05f);

    // Zero friction leaves it sliding.
    RigidBody slippery = makeDynamic(shape, 1.0f, Math::Vec3(0.0f, 0.5f, 0.0f));
    slippery.setVelocity(Math::Vec3(3.0f, -1.0f, 0.0f));
    Contact frictionless = contact;
    frictionless.b = &slippery;
    frictionless.friction = 0.0f;
    frictionless.manifold.points[0].normalImpulse = 0.0f;
    frictionless.manifold.points[0].tangentImpulse[0] = 0.0f;
    frictionless.manifold.points[0].tangentImpulse[1] = 0.0f;
    solver.solve(&frictionless, 1, 1.0f / 60.0f);
    CHECK(near(slippery.velocity().x, 3.0f, 1e-3f));
}

void testStaticPairDoesNothing()
{
    RigidBody groundA = makeStatic(Math::Vec3(0.0f));
    RigidBody groundB = makeStatic(Math::Vec3(0.1f, 0.0f, 0.0f));

    Contact contact;
    contact.a = &groundA;
    contact.b = &groundB;
    contact.manifold.normal = Math::Vec3(1, 0, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = Math::Vec3(0.05f, 0.0f, 0.0f);
    contact.manifold.points[0].penetration = 0.5f;

    ContactSolver solver;
    // Two infinite masses: dividing by their total would be a division by
    // zero, and moving either would be wrong.
    solver.solve(&contact, 1, 1.0f / 60.0f);
    CHECK(near(groundA.position(), Math::Vec3(0.0f)));
    CHECK(near(groundB.position(), Math::Vec3(0.1f, 0.0f, 0.0f)));
    CHECK(std::isfinite(groundA.position().x));
}

void testWarmStartingCarriesImpulse()
{
    const BoxShape shape(Math::Vec3(0.5f));
    RigidBody ground = makeStatic(Math::Vec3(0.0f, -0.5f, 0.0f));
    RigidBody box = makeDynamic(shape, 1.0f, Math::Vec3(0.0f, 0.5f, 0.0f));

    Contact contact;
    contact.a = &ground;
    contact.b = &box;
    contact.friction = 0.5f;
    contact.manifold.normal = Math::Vec3(0, 1, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = Math::Vec3(0.0f);
    contact.manifold.points[0].penetration = 0.005f;

    ContactSolver solver;
    box.setVelocity(Math::Vec3(0.0f, -2.0f, 0.0f));
    solver.solve(&contact, 1, 1.0f / 60.0f);
    // The impulse it needed is kept on the point, which is what the next
    // step starts from instead of finding it again from zero.
    CHECK(contact.manifold.points[0].normalImpulse > 0.0f);
}

void testRigidBodyForcesAndImpulses()
{
    const BoxShape shape(Math::Vec3(0.5f));
    RigidBody body = makeDynamic(shape, 2.0f, Math::Vec3(0.0f));

    // A force over a step accelerates by F/m, so dv = F/m * dt.
    body.addForce(Math::Vec3(10.0f, 0.0f, 0.0f));
    body.integrate(1.0f / 60.0f);
    CHECK(near(body.velocity().x, 10.0f / 2.0f / 60.0f, 1e-4f));

    // An impulse changes velocity by I/m immediately.
    body.applyLinearImpulse(Math::Vec3(4.0f, 0.0f, 0.0f));
    CHECK(near(body.velocity().x, 10.0f / 2.0f / 60.0f + 4.0f / 2.0f, 1e-4f));

    // setAcceleration is gravity: it accelerates without dividing by mass.
    body.setAcceleration(Math::Vec3(0.0f, -9.81f, 0.0f));
    const f32 beforeY = body.velocity().y;
    body.integrate(1.0f / 60.0f);
    CHECK(near(body.velocity().y, beforeY - 9.81f / 60.0f, 1e-4f));

    // addForceAtBodyPoint turns part of the force into torque.
    body.clearAccumulators();
    body.addForceAtBodyPoint(Math::Vec3(0.0f, 0.0f, 8.0f), Math::Vec3(0.5f, 0.0f, 0.0f));
    body.integrate(1.0f / 60.0f);
    CHECK(body.angularVelocity().y != 0.0f);
}

void testRigidBodyOffCenterImpulseSpins()
{
    const BoxShape shape(Math::Vec3(0.5f));
    RigidBody body = makeDynamic(shape, 1.0f, Math::Vec3(0.0f));

    // An impulse straight up at a point off to the +x side: it translates the
    // body and spins it about z (r x I points +z for r on +x and I up).
    body.applyImpulseAtPoint(Math::Vec3(0.0f, 5.0f, 0.0f), Math::Vec3(0.5f, 0.0f, 0.0f));

    CHECK(near(body.velocity(), Math::Vec3(0.0f, 5.0f, 0.0f), 1e-4f));
    CHECK(body.angularVelocity().z > 0.0f);
    // A point on the surface moves with v + w x r, so it is not just the
    // centre's velocity.
    const Math::Vec3 surfaceVel = body.velocityAtPoint(Math::Vec3(0.5f, 0.0f, 0.0f));
    CHECK(std::isfinite(surfaceVel.x));
}

void testRigidBodySleepsAndImpulseWakes()
{
    const BoxShape shape(Math::Vec3(0.5f));
    RigidBody body = makeDynamic(shape, 1.0f, Math::Vec3(0.0f));
    body.setCanSleep(true);
    body.setVelocity(Math::Vec3(0.001f, 0.0f, 0.0f));

    // Below the sleep epsilon and left alone, it has to fall asleep.
    bool slept = false;
    for (u32 i = 0; i < 60 && !slept; ++i)
    {
        body.integrate(1.0f / 60.0f);
        if (!body.awake())
            slept = true;
    }
    CHECK(slept);

    // Anything arriving wakes it up again.
    body.applyLinearImpulse(Math::Vec3(1.0f, 0.0f, 0.0f));
    CHECK(body.awake());
}

void testRigidBodyStaticAndKinematic()
{
    const BoxShape shape(Math::Vec3(0.5f));

    // Static: integrate is a no-op and forces do nothing.
    RigidBody statik = makeStatic(Math::Vec3(1.0f, 2.0f, 3.0f));
    statik.setVelocity(Math::Vec3(5.0f, 0.0f, 0.0f));
    statik.addForce(Math::Vec3(100.0f, 0.0f, 0.0f));
    statik.integrate(1.0f / 60.0f);
    CHECK(!statik.isDynamic());
    CHECK(near(statik.position(), Math::Vec3(1.0f, 2.0f, 3.0f)));

    // Kinematic: moves by its velocity alone, forces ignored.
    RigidBody kinematic;
    kinematic.setBodyType(BodyType::Kinematic);
    kinematic.setPosition(Math::Vec3(0.0f));
    kinematic.setVelocity(Math::Vec3(3.0f, 0.0f, 0.0f));
    kinematic.addForce(Math::Vec3(1000.0f, 0.0f, 0.0f));
    kinematic.integrate(1.0f / 60.0f);
    CHECK(near(kinematic.position().x, 3.0f / 60.0f, 1e-4f));
    CHECK(near(kinematic.velocity(), Math::Vec3(3.0f, 0.0f, 0.0f), 1e-4f));
}

void testSolverOffCenterContactSpinsABox()
{
    // A box falling onto a contact point off to one side: the impulse that
    // stops it goes through a point that is not under the centre of mass, so
    // it has to spin the box rather than only stopping it.
    const BoxShape shape(Math::Vec3(0.5f));
    RigidBody ground = makeStatic(Math::Vec3(0.0f, -0.5f, 0.0f));
    RigidBody box = makeDynamic(shape, 1.0f, Math::Vec3(0.0f, 0.45f, 0.0f));
    box.setVelocity(Math::Vec3(0.0f, -5.0f, 0.0f));
    box.setAngularVelocity(Math::Vec3(0.0f));

    const Math::Vec3 contactPoint(0.4f, -0.05f, 0.0f); // off-centre
    Contact contact;
    contact.a = &ground;
    contact.b = &box;
    contact.friction = 0.0f;
    contact.restitution = 0.0f;
    contact.manifold.normal = Math::Vec3(0, 1, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = contactPoint;
    contact.manifold.points[0].penetration = 0.05f;

    ContactSolver solver;
    solver.solve(&contact, 1, 1.0f / 60.0f);

    // What the solver cancels is the velocity AT the contact point, not the
    // centre of mass: the box keeps falling a little at its centre while it
    // spins. The point's downward speed has to be cut well below the -5 it
    // came in with, and the off-centre impulse has spun it.
    CHECK(box.velocityAtPoint(contactPoint).y > -1.0f);
    CHECK(std::abs(box.angularVelocity().z) > 1e-3f);
}

void testSolverSurvivesDeepPenetration()
{
    // A pathological overlap - a whole box's width into the ground - must be
    // corrected without exploding: the velocity stays bounded and the body
    // comes out clear instead of being flung.
    const BoxShape shape(Math::Vec3(0.5f));
    RigidBody ground = makeStatic(Math::Vec3(0.0f, -0.5f, 0.0f));
    RigidBody box = makeDynamic(shape, 1.0f, Math::Vec3(0.0f, -0.2f, 0.0f)); // deep inside
    box.setVelocity(Math::Vec3(0.0f, -10.0f, 0.0f));

    Contact contact;
    contact.a = &ground;
    contact.b = &box;
    contact.friction = 0.5f;
    contact.restitution = 0.0f;
    contact.manifold.normal = Math::Vec3(0, 1, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = Math::Vec3(0.0f, -0.05f, 0.0f);
    contact.manifold.points[0].penetration = 0.7f;

    ContactSolver solver;
    const f32 startY = box.position().y;
    for (u32 i = 0; i < 5; ++i)
        solver.solve(&contact, 1, 1.0f / 60.0f);

    // Corrected out of the ground, without a crazy velocity or a NaN.
    CHECK(box.position().y > startY);
    CHECK(std::abs(box.velocity().y) < 20.0f);
    CHECK(std::isfinite(box.position().y));
}

// ------------------------------------------------------- end to end, no demo

void testBoxSettlesOnGround()
{
    // The whole pipeline against gravity: narrowphase every step, solve, and
    // integrate. The box has to come to rest on the ground and stay there,
    // neither sinking through nor drifting sideways.
    const BoxShape groundShape(Math::Vec3(10.0f, 0.5f, 10.0f));
    const BoxShape boxShape(Math::Vec3(0.5f));

    RigidBody ground = makeStatic(Math::Vec3(0.0f, -0.5f, 0.0f));
    RigidBody box = makeDynamic(boxShape, 1.0f, Math::Vec3(0.0f, 3.0f, 0.0f));
    box.setAcceleration(Math::Vec3(0.0f, -9.81f, 0.0f));
    box.setDamping(0.999f, 0.999f);

    ContactSolver solver;
    constexpr f32 step = 1.0f / 120.0f;
    for (u32 i = 0; i < 600; ++i)
    {
        box.integrate(step);

        ContactManifold manifold;
        if (Narrowphase::collide(groundShape, ground.transform(), boxShape, box.transform(),
                                 manifold))
        {
            Contact contact;
            contact.a = &ground;
            contact.b = &box;
            contact.manifold = manifold;
            contact.friction = 0.6f;
            contact.restitution = 0.0f;
            solver.solve(&contact, 1, step);
        }
    }

    // Resting on the ground: its centre sits one half-extent above the
    // ground's top face at y = 0, give or take the solver's slop.
    CHECK(box.position().y > 0.45f);
    CHECK(box.position().y < 0.55f);
    CHECK(std::abs(box.velocity().y) < 0.5f);
    // Nothing pushed it sideways, so it must not have wandered.
    CHECK(std::abs(box.position().x) < 0.05f);
    CHECK(std::abs(box.position().z) < 0.05f);
    CHECK(std::isfinite(box.position().y));
}

// --------------------------------------------------------------- the world

void testWorldStackStandsUp()
{
    // Five boxes stacked on the ground, dropped from a small gap so they
    // settle rather than start interpenetrating. This is the case the whole
    // pipeline exists for and the one that exposes everything: a broadphase
    // that misses a pair, a manifold with too few points, or a solver without
    // warm starting all end with the tower sunk into itself or on the floor.
    BoxShape groundShape(Math::Vec3(20.0f, 0.5f, 20.0f));
    BoxShape boxShape(Math::Vec3(0.5f));

    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));

    constexpr u32 kCount = 5;
    RigidBody boxes[kCount];
    for (u32 i = 0; i < kCount; ++i)
    {
        boxes[i].setMass(1.0f);
        boxes[i].setInertiaTensor(boxShape.inertia(1.0f));
        boxes[i].setPosition(Math::Vec3(0.0f, 0.5f + static_cast<f32>(i) * 1.02f, 0.0f));
        boxes[i].setDamping(0.999f, 0.999f);
    }

    PhysicsWorld world;
    world.setGravity(Math::Vec3(0.0f, -9.81f, 0.0f));
    world.setFixedStep(1.0f / 120.0f);

    BodyEntry entry;
    entry.shape = &groundShape;
    entry.body = &ground;
    entry.friction = 0.8f;
    world.addBody(entry);
    entry.shape = &boxShape;
    for (u32 i = 0; i < kCount; ++i)
    {
        entry.body = &boxes[i];
        world.addBody(entry);
    }

    for (u32 i = 0; i < 900; ++i)
        world.step(1.0f / 120.0f);

    // Every box has to end up at its own level, within a fraction of its own
    // size. Sagging shows up here as a box sitting well below where it should.
    const int before = gFailures;
    for (u32 i = 0; i < kCount; ++i)
    {
        const f32 expected = 0.5f + static_cast<f32>(i) * 1.0f;
        CHECK(std::abs(boxes[i].position().y - expected) < 0.12f);
        // And it must not have wandered sideways: nothing pushed it.
        CHECK(std::abs(boxes[i].position().x) < 0.15f);
        CHECK(std::abs(boxes[i].position().z) < 0.15f);
        CHECK(std::isfinite(boxes[i].position().y));
    }

    // Settled means slow. A tower that is still moving after seven seconds is
    // a tower that never converged.
    for (u32 i = 0; i < kCount; ++i)
        CHECK(glm::length(boxes[i].velocity()) < 0.35f);

    if (gFailures != before)
        for (u32 i = 0; i < kCount; ++i)
            std::fprintf(stderr, "    box %u: y %.4f (want %.2f) x %.4f z %.4f |v| %.4f\n", i,
                         boxes[i].position().y, 0.5 + double(i), boxes[i].position().x,
                         boxes[i].position().z, glm::length(boxes[i].velocity()));
}

void testStackSleepsTogether()
{
    // Sleep decided per body freezes a stack half-settled: the boxes lower
    // down are still sinking their last few millimetres, the ones on top have
    // stopped moving and fall asleep - and a sleeping body does not
    // integrate, so it stays exactly where it was while the tower shrinks
    // underneath it. The result is boxes hanging in the air until something
    // else hits them. Bodies joined by contacts have to sleep as one.
    BoxShape groundShape(Math::Vec3(20.0f, 0.5f, 20.0f));
    BoxShape boxShape(Math::Vec3(0.5f));

    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));

    constexpr u32 kCount = 6;
    RigidBody boxes[kCount];

    PhysicsWorld world;
    world.setGravity(Math::Vec3(0.0f, -9.81f, 0.0f));
    world.setFixedStep(1.0f / 120.0f);

    BodyEntry entry;
    entry.friction = 0.7f;
    entry.shape = &groundShape;
    entry.body = &ground;
    world.addBody(entry);
    entry.shape = &boxShape;
    for (u32 i = 0; i < kCount; ++i)
    {
        boxes[i].setMass(1.0f);
        boxes[i].setInertiaTensor(boxShape.inertia(1.0f));
        boxes[i].setPosition(Math::Vec3(0.0f, 0.5f + static_cast<f32>(i) * 1.005f, 0.0f));
        boxes[i].setDamping(0.999f, 0.999f);
        entry.body = &boxes[i];
        world.addBody(entry);
    }

    const int before = gFailures;
    u32 worstSplitStep = 0;
    u32 worstAsleep = 0;
    for (u32 step = 0; step < 1200; ++step)
    {
        world.step(1.0f / 120.0f);

        // Checked EVERY step, not only at the end: a stack that freezes
        // half-asleep may still look right once everything has stopped, and
        // the state that hangs a box is a transient one.
        u32 asleep = 0;
        for (u32 i = 0; i < kCount; ++i)
            if (!boxes[i].awake())
                ++asleep;
        if (asleep != 0 && asleep != kCount && worstSplitStep == 0)
        {
            worstSplitStep = step;
            worstAsleep = asleep;
        }
    }
    if (worstSplitStep != 0)
        std::fprintf(stderr, "    stack was %u/%u asleep at step %u\n", worstAsleep, kCount,
                     worstSplitStep);
    CHECK(worstSplitStep == 0);

    // Every box must be resting ON the one below, not floating above it. A
    // box is 1 tall, so the centres are 1 apart plus whatever overlap the
    // solver leaves - never a gap.
    for (u32 i = 1; i < kCount; ++i)
    {
        const f32 spacing = boxes[i].position().y - boxes[i - 1].position().y;
        CHECK(spacing <= 1.0f + world.contactMargin());
        CHECK(spacing > 0.9f);
    }

    if (gFailures != before)
        for (u32 i = 0; i < kCount; ++i)
            std::fprintf(stderr, "    box %u: y %.4f %s\n", i, boxes[i].position().y,
                         boxes[i].awake() ? "awake" : "asleep");
}

void testWorldEventsEnterStayExit()
{
    struct Recorder
    {
        u32 enters = 0;
        u32 stays = 0;
        u32 exits = 0;
    };
    Recorder recorder;

    BoxShape shape(Math::Vec3(0.5f));
    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));

    RigidBody box;
    box.setMass(1.0f);
    box.setInertiaTensor(shape.inertia(1.0f));
    box.setPosition(Math::Vec3(0.0f, 3.0f, 0.0f));
    box.setCanSleep(false);

    PhysicsWorld world;
    world.setGravity(Math::Vec3(0.0f, -9.81f, 0.0f));
    BodyEntry entry;
    entry.shape = &shape;
    entry.body = &ground;
    world.addBody(entry);
    entry.body = &box;
    world.addBody(entry);

    world.setEventCallback(
        [](const ContactEventInfo& info, void* user)
        {
            Recorder& target = *static_cast<Recorder*>(user);
            if (info.event == ContactEvent::Enter)
                ++target.enters;
            else if (info.event == ContactEvent::Stay)
                ++target.stays;
            else
                ++target.exits;
        },
        &recorder);

    u32 longestGap = 0;
    u32 gap = 0;
    for (u32 i = 0; i < 300; ++i)
    {
        const u32 before = recorder.enters + recorder.stays;
        world.step(1.0f / 120.0f);
        if (recorder.enters + recorder.stays == before && box.position().y < 2.0f)
            longestGap = glm::max(longestGap, ++gap);
        else
            gap = 0;
    }

    // Landing is exactly one Enter followed by many Stays - an Enter every
    // step would mean the cache is not remembering the pair, which is the
    // same bookkeeping warm starting depends on.
    if (recorder.enters != 1 || recorder.exits != 0)
        std::fprintf(stderr,
                     "  landing: %u enters, %u stays, %u exits, resting y %.4f, longest gap %u\n",
                     recorder.enters, recorder.stays, recorder.exits, box.position().y, longestGap);
    CHECK(recorder.enters == 1);
    CHECK(recorder.stays > 100);
    CHECK(recorder.exits == 0);

    // Lifted away, the pair has to report Exit once and then stop. It takes
    // as many steps as the persistence window, which is deliberate: a single
    // missed step is what happens on landing and must not read as separation.
    box.setBodyType(BodyType::Kinematic);
    box.setPosition(Math::Vec3(0.0f, 20.0f, 0.0f));
    for (u32 i = 0; i < world.contactPersistence(); ++i)
        world.step(1.0f / 120.0f);
    CHECK(recorder.exits == 1);
    const u32 after = recorder.exits;
    for (u32 i = 0; i < 5; ++i)
        world.step(1.0f / 120.0f);
    CHECK(recorder.exits == after);
}

void testWorldFixedStepIsFrameRateIndependent()
{
    // The same second of simulation, delivered as one long frame or as many
    // short ones, has to land in the same place - that is the whole point of
    // a fixed step.
    BoxShape shape(Math::Vec3(0.5f));

    auto run = [&shape](f32 frame, u32 frames)
    {
        RigidBody box;
        box.setMass(1.0f);
        box.setInertiaTensor(shape.inertia(1.0f));
        box.setPosition(Math::Vec3(0.0f, 10.0f, 0.0f));
        box.setCanSleep(false);
        PhysicsWorld world;
        world.setGravity(Math::Vec3(0.0f, -9.81f, 0.0f));
        world.setFixedStep(1.0f / 120.0f);
        BodyEntry entry;
        entry.shape = &shape;
        entry.body = &box;
        world.addBody(entry);
        for (u32 i = 0; i < frames; ++i)
            world.update(frame);
        return box.position().y;
    };

    const f32 fine = run(1.0f / 120.0f, 60);
    const f32 coarse = run(1.0f / 30.0f, 15);
    CHECK(near(fine, coarse, 1e-3f));
}

// A unit quad on the XZ plane at y = 0, spanning [-5, 5], as two triangles.
void makeGroundMesh(std::vector<Math::Vec3>& vertices, std::vector<u32>& indices)
{
    vertices = {Math::Vec3(-5.0f, 0.0f, -5.0f), Math::Vec3(5.0f, 0.0f, -5.0f),
                Math::Vec3(5.0f, 0.0f, 5.0f), Math::Vec3(-5.0f, 0.0f, 5.0f)};
    indices = {0, 2, 1, 0, 3, 2};
}

void testClosestPointOnTriangle()
{
    const Math::Vec3 a(0.0f, 0.0f, 0.0f);
    const Math::Vec3 b(1.0f, 0.0f, 0.0f);
    const Math::Vec3 c(0.0f, 0.0f, 1.0f);

    CHECK(near(closestPointOnTriangle(a, b, c, Math::Vec3(0.25f, 3.0f, 0.25f)),
               Math::Vec3(0.25f, 0.0f, 0.25f)));
    CHECK(near(closestPointOnTriangle(a, b, c, Math::Vec3(-2.0f, 0.0f, -2.0f)), a));
    CHECK(near(closestPointOnTriangle(a, b, c, Math::Vec3(5.0f, 0.0f, 0.0f)), b));
    CHECK(near(closestPointOnTriangle(a, b, c, Math::Vec3(0.5f, 1.0f, -1.0f)),
               Math::Vec3(0.5f, 0.0f, 0.0f)));
}

void testTrimeshTreeFindsOnlyNearbyTriangles()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));
    CHECK(mesh.triangleCount() == 2);

    std::vector<u32> hits;
    AABB far;
    far.min = Math::Vec3(100.0f);
    far.max = Math::Vec3(101.0f);
    mesh.query(far, hits);
    CHECK(hits.empty());

    AABB over;
    over.min = Math::Vec3(-1.0f, -1.0f, -1.0f);
    over.max = Math::Vec3(1.0f, 1.0f, 1.0f);
    mesh.query(over, hits);
    CHECK(!hits.empty());
}

void testSphereOnTriangle()
{
    const SphereShape sphere(1.0f);
    const TriangleShape triangle(Math::Vec3(-5.0f, 0.0f, -5.0f), Math::Vec3(5.0f, 0.0f, 5.0f),
                                 Math::Vec3(5.0f, 0.0f, -5.0f));

    ContactManifold manifold;
    CHECK(!Narrowphase::sphereTriangle(sphere, at(Math::Vec3(0.0f, 3.0f, 0.0f)), triangle,
                                       Math::Mat4(1.0f), manifold));

    CHECK(Narrowphase::sphereTriangle(sphere, at(Math::Vec3(1.0f, 0.75f, -1.0f)), triangle,
                                      Math::Mat4(1.0f), manifold));
    CHECK(manifold.count == 1);
    CHECK(near(manifold.points[0].penetration, 0.25f));
    // Sphere above, triangle below: A to B points down.
    CHECK(near(manifold.normal, Math::Vec3(0.0f, -1.0f, 0.0f)));
}

void testBoxOnTriangleGetsAPatch()
{
    const BoxShape box(Math::Vec3(1.0f));
    const TriangleShape triangle(Math::Vec3(-5.0f, 0.0f, -5.0f), Math::Vec3(5.0f, 0.0f, 5.0f),
                                 Math::Vec3(5.0f, 0.0f, -5.0f));

    ContactManifold manifold;
    CHECK(Narrowphase::boxTriangle(box, at(Math::Vec3(1.0f, 0.9f, -1.0f)), triangle, Math::Mat4(1.0f),
                                   manifold));
    CHECK(near(manifold.normal, Math::Vec3(0.0f, -1.0f, 0.0f)));
    // A face resting on a face has to give more than one point, or the box
    // pivots on the single one instead of settling flat.
    CHECK(manifold.count > 1);
    CHECK(near(manifold.points[0].penetration, 0.1f, 1e-3f));

    CHECK(!Narrowphase::boxTriangle(box, at(Math::Vec3(1.0f, 3.0f, -1.0f)), triangle,
                                    Math::Mat4(1.0f), manifold));
}

void testCapsuleOnTriangle()
{
    const CapsuleShape capsule(0.5f, 1.0f);
    const TriangleShape triangle(Math::Vec3(-5.0f, 0.0f, -5.0f), Math::Vec3(5.0f, 0.0f, 5.0f),
                                 Math::Vec3(5.0f, 0.0f, -5.0f));

    ContactManifold manifold;
    CHECK(Narrowphase::capsuleTriangle(capsule, at(Math::Vec3(1.0f, 1.25f, -1.0f)), triangle,
                                       Math::Mat4(1.0f), manifold));
    CHECK(near(manifold.points[0].penetration, 0.25f));
    CHECK(near(manifold.normal, Math::Vec3(0.0f, -1.0f, 0.0f)));
}

void testConvexTrimeshSpansBothTriangles()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));
    const BoxShape box(Math::Vec3(1.0f));

    std::vector<ContactManifold> manifolds;
    CHECK(Narrowphase::convexTrimesh(box, at(Math::Vec3(0.0f, 0.9f, 0.0f)), mesh, Math::Mat4(1.0f),
                                     manifolds));
    // Straddling the quad's diagonal touches both triangles, and each one
    // brings its own manifold rather than being merged into a single normal.
    CHECK(manifolds.size() == 2);
    for (const ContactManifold& manifold : manifolds)
        CHECK(near(manifold.normal, Math::Vec3(0.0f, -1.0f, 0.0f)));

    manifolds.clear();
    CHECK(!Narrowphase::convexTrimesh(box, at(Math::Vec3(0.0f, 5.0f, 0.0f)), mesh, Math::Mat4(1.0f),
                                      manifolds));
    CHECK(manifolds.empty());
}

void testTrimeshUnderRotation()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));
    const SphereShape sphere(1.0f);

    // The mesh rolled 90 degrees about Z is a wall in the YZ plane; a sphere
    // beside it must be found through the same inverse-transform path.
    const Math::Mat4 meshTransform =
        at(Math::Vec3(0.0f), glm::angleAxis(glm::radians(90.0f), Math::Vec3(0.0f, 0.0f, 1.0f)));
    std::vector<ContactManifold> manifolds;
    CHECK(Narrowphase::convexTrimesh(sphere, at(Math::Vec3(0.75f, 0.0f, 0.0f)), mesh, meshTransform,
                                     manifolds));
    CHECK(!manifolds.empty());
    CHECK(near(std::abs(manifolds[0].normal.x), 1.0f, 1e-3f));
}

// A floor at y = 0 plus a wall standing at x = 2, both as quads.
void makeRoomMesh(std::vector<Math::Vec3>& vertices, std::vector<u32>& indices)
{
    vertices = {// floor
                Math::Vec3(-10.0f, 0.0f, -10.0f), Math::Vec3(10.0f, 0.0f, -10.0f),
                Math::Vec3(10.0f, 0.0f, 10.0f), Math::Vec3(-10.0f, 0.0f, 10.0f),
                // wall facing -X
                Math::Vec3(2.0f, 0.0f, -10.0f), Math::Vec3(2.0f, 0.0f, 10.0f),
                Math::Vec3(2.0f, 6.0f, 10.0f), Math::Vec3(2.0f, 6.0f, -10.0f)};
    // Floor up, wall towards -X where the character comes from. A sweep is
    // one-sided, unlike a push-out, so a back-facing wall is simply not there.
    indices = {0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7};
}

// A floor at y = 0, a ceiling at y = 3 and a wall at x = 2 facing -X - enough
// for the Godot-style isOnWall()/isOnCeiling() state to have something to hit.
void makeRoomWithCeilingMesh(std::vector<Math::Vec3>& vertices, std::vector<u32>& indices)
{
    vertices = {// floor (+Y)
                Math::Vec3(-10.0f, 0.0f, -10.0f), Math::Vec3(10.0f, 0.0f, -10.0f),
                Math::Vec3(10.0f, 0.0f, 10.0f), Math::Vec3(-10.0f, 0.0f, 10.0f),
                // wall facing -X
                Math::Vec3(2.0f, 0.0f, -10.0f), Math::Vec3(2.0f, 0.0f, 10.0f),
                Math::Vec3(2.0f, 3.0f, 10.0f), Math::Vec3(2.0f, 3.0f, -10.0f),
                // ceiling (-Y)
                Math::Vec3(-10.0f, 3.0f, -10.0f), Math::Vec3(10.0f, 3.0f, -10.0f),
                Math::Vec3(10.0f, 3.0f, 10.0f), Math::Vec3(-10.0f, 3.0f, 10.0f)};
    indices = {0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 8, 9, 10, 8, 10, 11};
}

void testCharacterReportsWallAndCeiling()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeRoomWithCeilingMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::Vec3(-3.0f, 2.0f, 0.0f));
    for (u32 i = 0; i < 240; ++i)
        character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));
    CHECK(character.isOnFloor());

    // Walk into the wall at x = 2: the state has to report a wall, with the
    // wall's own normal, and the character must stop before the wall face.
    character.setMoveInput(Math::Vec3(4.0f, 0.0f, 0.0f));
    bool sawWall = false;
    for (u32 i = 0; i < 120 && !sawWall; ++i)
    {
        character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));
        if (character.isOnWall())
        {
            sawWall = true;
            CHECK(character.wallNormal().x < -0.5f);
        }
    }
    CHECK(sawWall);
    CHECK(character.position().x < 2.0f);

    // Jump into the ceiling at y = 3: isOnCeiling() with a downward normal.
    character.setMoveInput(Math::Vec3(0.0f));
    character.jump(8.0f);
    bool sawCeiling = false;
    for (u32 i = 0; i < 120 && !sawCeiling; ++i)
    {
        character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));
        if (character.isOnCeiling())
        {
            sawCeiling = true;
            CHECK(character.ceilingNormal().y < -0.5f);
        }
    }
    CHECK(sawCeiling);
}

void testCharacterMoveAndSlideGodotStyle()
{
    // Godot's pattern: the caller owns the velocity (gravity included) and
    // calls moveAndSlide() each frame. Landing has to zero the up component
    // so gravity does not keep accumulating into the floor.
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::Vec3(-4.0f, 4.0f, 0.0f));

    Math::Vec3 velocity(2.0f, -3.0f, 0.0f);
    for (u32 i = 0; i < 240; ++i)
    {
        velocity.y -= 20.0f * (1.0f / 60.0f);
        velocity.y = glm::max(velocity.y, -50.0f);
        character.setVelocity(velocity);
        character.moveAndSlide(1.0f / 60.0f, mesh, Math::Mat4(1.0f));
        velocity = character.velocity();
    }
    CHECK(character.isOnFloor());
    CHECK(near(character.verticalSpeed(), 0.0f, 1e-3f));
    // The horizontal component survives the slide (it is not a wall).
    CHECK(character.position().x > -3.0f);
}

void testCharacterSetVerticalSpeedJumpsAnytime()
{
    // setVerticalSpeed() is Godot's no-grounded-check jump: an airborne body
    // still rises, unlike jump() which only fires from the ground.
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::Vec3(-4.0f, 3.0f, 0.0f));
    // Never let it land: it spawns above the floor and rises immediately.
    character.setVerticalSpeed(6.0f);
    const f32 startY = character.position().y;
    bool rose = false;
    for (u32 i = 0; i < 30; ++i)
    {
        character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));
        if (character.position().y > startY + 0.2f)
            rose = true;
    }
    CHECK(rose);
}

void testCharacterApplyFloorSnapIsPublic()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    // A little above the floor, not yet grounded - a straight probe down
    // settles it (Godot's apply_floor_snap()).
    character.setPosition(Math::Vec3(-2.0f, 1.1f, 0.0f));
    CHECK(character.applyFloorSnap(mesh, Math::Mat4(1.0f)));
    CHECK(character.isOnFloor());
    // Rests one vertical radius plus the skin above the floor.
    CHECK(near(character.position().y, 1.02f, 0.02f));
}

void testSlideCameraPullsBackFromAWall()
{
    // Anchor at x = 0, desired camera position at x = 8, a wall at x = 2
    // between them. The camera sphere has to stop short of the wall, hugging
    // it, instead of reaching the desired position through the wall.
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    const f32 radius = 0.3f;
    const Math::Vec3 cam =
        mesh.slideCamera(Math::Vec3(0.0f, 2.0f, 0.0f), Math::Vec3(8.0f, 2.0f, 0.0f), radius);

    // Pulled back before the wall face at x = 2, clear of the anchor, and the
    // camera sphere itself does not cut the wall.
    CHECK(cam.x < 2.0f);
    CHECK(cam.x > 1.0f);
    CHECK(cam.x + radius < 2.0f + 1e-3f);
}

void testSlideCameraReturnsTheDesiredPositionWhenClear()
{
    // No wall in the way: the camera reaches the desired position exactly.
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    const Math::Vec3 desired(5.0f, 2.0f, 0.0f);
    const Math::Vec3 cam = mesh.slideCamera(Math::Vec3(0.0f, 2.0f, 0.0f), desired, 0.3f);
    CHECK(near(cam, desired, 1e-3f));
}

void testCharacterLandsAndStandsOnTheFloor()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::Vec3(-2.0f, 4.0f, 0.0f));

    for (u32 i = 0; i < 120; ++i)
        character.move(Math::Vec3(0.0f, -0.05f, 0.0f), mesh, Math::Mat4(1.0f));

    CHECK(character.grounded());
    // Centre sits a full half-capsule above the floor, plus the skin.
    const f32 expected = 0.4f + 0.6f;
    CHECK(character.position().y > expected - 0.1f);
    CHECK(character.position().y < expected + 0.15f);
    CHECK(near(character.groundNormal().y, 1.0f, 1e-2f));
}

void testCharacterDoesNotWalkThroughAWall()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::Vec3(-2.0f, 1.0f, 0.0f));

    // Walked hard into the wall at x = 2 for two seconds.
    for (u32 i = 0; i < 120; ++i)
    {
        character.move(Math::Vec3(0.12f, 0.0f, 0.0f), mesh, Math::Mat4(1.0f));
        character.move(Math::Vec3(0.0f, -0.05f, 0.0f), mesh, Math::Mat4(1.0f));
    }
    CHECK(character.position().x < 2.0f);
    CHECK(character.position().x > 1.0f);
}

void testCharacterDoesNotTeleportToAFarWall()
{
    // The wall at x = 2 is eight units away. A swept sphere's plane
    // intersection sits far beyond the end of the path (t = distance /
    // step = 80), and the sweep used to report it as the closest hit because
    // the running best started at infinity instead of at 1.0 - so a single
    // 0.1 step moved the character all the way to the wall in one frame.
    // That is the castle's "touch a wall and it flies off". One step has to
    // move him 0.1, not to the wall.
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::Vec3(-6.0f, 1.0f, 0.0f));

    const Math::Vec3 before = character.position();
    character.move(Math::Vec3(0.1f, 0.0f, 0.0f), mesh, Math::Mat4(1.0f));
    // Moved one 0.1 step, not teleported to the wall at x = 2.
    CHECK(glm::length(character.position() - before) < 0.2f);
    CHECK(character.position().x < -5.5f);
}

void testCharacterCrossesSeamsWithoutStopping()
{
    // A floor cut into a grid, so walking it crosses many shared edges - the
    // case that used to catch a body and stop it dead on flat ground.
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    constexpr int kCells = 12;
    constexpr f32 kStep = 1.0f;
    for (int r = 0; r <= kCells; ++r)
        for (int c = 0; c <= kCells; ++c)
            vertices.push_back(Math::Vec3(static_cast<f32>(c) * kStep - 6.0f, 0.0f,
                                         static_cast<f32>(r) * kStep - 6.0f));
    for (int r = 0; r < kCells; ++r)
        for (int c = 0; c < kCells; ++c)
        {
            const u32 i0 = static_cast<u32>(r * (kCells + 1) + c);
            const u32 i1 = i0 + 1;
            const u32 i2 = i0 + static_cast<u32>(kCells + 1);
            const u32 i3 = i2 + 1;
            indices.insert(indices.end(), {i0, i2, i1, i1, i2, i3});
        }

    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::Vec3(-5.0f, 1.05f, 0.3f));

    const f32 startX = character.position().x;
    for (u32 i = 0; i < 200; ++i)
    {
        character.move(Math::Vec3(0.05f, 0.0f, 0.0f), mesh, Math::Mat4(1.0f));
        character.move(Math::Vec3(0.0f, -0.02f, 0.0f), mesh, Math::Mat4(1.0f));
    }

    // Ten units asked for; anything much short of it means a seam stopped him.
    const f32 travelled = character.position().x - startX;
    CHECK(travelled > 9.0f);
    CHECK(character.grounded());
}

// Floor at y = 0 up to x = 0, then a ledge `height` tall from x = 0 onwards.
void makeLedgeMesh(f32 height, std::vector<Math::Vec3>& vertices, std::vector<u32>& indices)
{
    vertices = {Math::Vec3(-10.0f, 0.0f, -10.0f), Math::Vec3(0.0f, 0.0f, -10.0f),
                Math::Vec3(0.0f, 0.0f, 10.0f),    Math::Vec3(-10.0f, 0.0f, 10.0f),
                Math::Vec3(0.0f, height, -10.0f), Math::Vec3(0.0f, height, 10.0f),
                Math::Vec3(10.0f, height, 10.0f), Math::Vec3(10.0f, height, -10.0f),
                Math::Vec3(0.0f, 0.0f, -10.0f),   Math::Vec3(0.0f, 0.0f, 10.0f)};
    // Wound so every face looks where it should: floors up, riser towards
    // the character who walks into it. A back-facing wall is a different
    // test, not this one.
    indices = {// lower floor, +Y
               0, 2, 1, 0, 3, 2,
               // upper floor, +Y
               4, 5, 6, 4, 6, 7,
               // the riser between them, -X
               8, 9, 4, 9, 5, 4};
}

f32 walkAtLedge(f32 ledgeHeight, f32 stepOffset)
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeLedgeMesh(ledgeHeight, vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setStepOffset(stepOffset);
    character.setPosition(Math::Vec3(-3.0f, 1.05f, 0.0f));
    character.setMoveInput(Math::Vec3(3.0f, 0.0f, 0.0f));
    for (u32 i = 0; i < 180; ++i)
    {
        const auto r = character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));
        if (std::getenv("RADION_TRACE") && i % 15 == 0)
            std::fprintf(
                stderr, "  ledge %.2f step %.2f  f=%3u pos=(%.3f,%.3f) g=%d blocked=%d hit=%d\n",
                static_cast<f64>(ledgeHeight), static_cast<f64>(stepOffset), i,
                static_cast<f64>(character.position().x), static_cast<f64>(character.position().y),
                character.grounded(), r.blocked, r.collided);
    }
    return character.position().x;
}

void testCharacterClimbsAStepAndIsStoppedByAWall()
{
    if (std::getenv("RADION_TRACE"))
        for (f32 h = 0.1f; h < 1.4f; h += 0.1f)
            std::fprintf(stderr, "  height %.2f: no-step %.2f  step0.35 %.2f\n",
                         static_cast<f64>(h), static_cast<f64>(walkAtLedge(h, 0.0f)),
                         static_cast<f64>(walkAtLedge(h, 0.35f)));

    // An ellipsoid rides over a step on its own curved base - that is how
    // collide-and-slide gets stairs with no step-up pass at all, and why
    // nobody using this algorithm reports catching on them.
    //
    // Measured on this character, whose vertical radius is 1.0: up to 0.7 is
    // climbed, 0.8 and over is a wall. The ceiling sits just under the
    // vertical radius for a reason - the ellipsoid's widest cross-section is
    // at its centre, so a ledge at or below that can be rolled onto and one
    // above it meets the upper half and pushes back.
    //
    // This number moved every time something was fixed, which is the useful
    // part: 0.6 while the skin was too small to stop it re-embedding, 1.0
    // once the push-out was being read back as velocity and flinging it up,
    // 0.9 after ground snapping, 0.8 once the push-out stopped counting as
    // motion, 0.7 once the slide went back to projecting its destination with
    // the world normal like the scene CharacterController. That last change
    // is the one that removed the wall launch: the ellipsoid-space projection
    // had bought the extra 0.1 of climbing by leaving a residual that was
    // tangent to the ellipsoid but not to the real surface, and converting it
    // back to world amplified its vertical component - pressing against a
    // slanted wall (or brushing a seam) shot the character up. Anything above
    // 0.7 here means that crept back.
    CHECK(walkAtLedge(0.3f, 0.0f) > 0.5f);
    CHECK(walkAtLedge(0.7f, 0.0f) > 0.5f);
    CHECK(walkAtLedge(0.8f, 0.0f) < 0.0f);
    CHECK(walkAtLedge(1.5f, 0.35f) < 0.0f);
}

void testCharacterDoesNotHopAtAPlatformSeam()
{
    // Two separate platforms at the SAME height, meeting at x = 0 without
    // sharing vertices - which is how level geometry is really built, one
    // piece butted against the next. The seam is an edge, and an edge contact
    // reports a steep normal even on dead level ground.
    std::vector<Math::Vec3> vertices = {Math::Vec3(-8.0f, 1.0f, -8.0f), Math::Vec3(0.0f, 1.0f, -8.0f),
                                       Math::Vec3(0.0f, 1.0f, 8.0f),   Math::Vec3(-8.0f, 1.0f, 8.0f),
                                       Math::Vec3(0.0f, 1.0f, -8.0f),  Math::Vec3(8.0f, 1.0f, -8.0f),
                                       Math::Vec3(8.0f, 1.0f, 8.0f),   Math::Vec3(0.0f, 1.0f, 8.0f)};
    std::vector<u32> indices = {0, 2, 1, 0, 3, 2, 4, 6, 5, 4, 7, 6};

    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setStepOffset(0.35f);
    character.setPosition(Math::Vec3(-4.0f, 3.0f, 0.0f));
    for (u32 i = 0; i < 180; ++i)
        character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));
    CHECK(character.grounded());

    const f32 settled = character.position().y;
    character.setMoveInput(Math::Vec3(3.0f, 0.0f, 0.0f));
    f32 highest = settled;
    for (u32 i = 0; i < 180; ++i)
    {
        character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));
        highest = glm::max(highest, character.position().y);
    }

    // Crossed the seam and kept walking, without ever being lifted. A step
    // offset firing at the junction shows up here as a 0.35 hop.
    CHECK(character.position().x > 1.0f);
    CHECK(highest < settled + 0.05f);
    CHECK(character.grounded());
}

void testCharacterIsNeverLaunchedByAContact()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    // Started deliberately INSIDE the wall at x = 2, which is the worst case
    // the push-out has to handle - and the one that used to convert the
    // ejection into velocity and fire him into the sky.
    character.setPosition(Math::Vec3(2.0f, 1.0f, 0.0f));

    f32 highest = character.position().y;
    for (u32 i = 0; i < 180; ++i)
    {
        const Math::Vec3 before = character.position();
        character.setMoveInput(Math::Vec3(4.0f, 0.0f, 0.0f));
        character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));
        highest = glm::max(highest, character.position().y);
        // Nothing here can send him upward at all, so any climb is the
        // push-out being read back as motion.
        CHECK(character.verticalSpeed() < 1.0f);
        // And no single frame may move him further than one push-out plus the
        // step he asked for. Iterating the push-out is how someone wedged in
        // a corner gets shoved once per iteration and flies across the level.
        CHECK(glm::length(character.position() - before) < 1.5f);
    }
    CHECK(highest < 2.0f);
    CHECK(character.position().x < 2.0f);
}

void testGroundedNeverFlickersWhileStandingStill()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::Vec3(-2.0f, 4.0f, 0.0f));
    for (u32 i = 0; i < 180; ++i)
        character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));
    CHECK(character.grounded());

    // Settled and untouched. He rests one skin width clear of the floor, but
    // one frame of falling covers a quarter of that, so a bare downward sweep
    // finds nothing and `grounded` drops - then speed accumulates for a few
    // frames until it reaches, and the flag ticks yes/no forever. Every frame
    // has to report standing, and he must not creep downwards.
    const f32 settled = character.position().y;
    u32 airborneFrames = 0;
    for (u32 i = 0; i < 240; ++i)
    {
        character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));
        if (!character.grounded())
            ++airborneFrames;
    }
    CHECK(airborneFrames == 0);
    CHECK(near(character.position().y, settled, 1e-3f));

    // The same while walking, which is when it was actually noticed.
    character.setMoveInput(Math::Vec3(2.0f, 0.0f, 0.0f));
    airborneFrames = 0;
    for (u32 i = 0; i < 180; ++i)
    {
        character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));
        if (!character.grounded())
            ++airborneFrames;
    }
    CHECK(airborneFrames == 0);
}

void testCharacterFallsAndLandsUnderGravity()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::Vec3(-2.0f, 6.0f, 0.0f));
    CHECK(!character.grounded());

    for (u32 i = 0; i < 240; ++i)
        character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));

    CHECK(character.grounded());
    // The fall has to stop, or standing still keeps accumulating speed and
    // the character shoots off the first ramp he meets.
    CHECK(near(character.verticalSpeed(), 0.0f, 1e-3f));
    CHECK(near(character.slopeAngle(), 0.0f, 2.0f));
}

void testCharacterJumpsOnlyFromTheGround()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::Vec3(-2.0f, 6.0f, 0.0f));
    for (u32 i = 0; i < 240; ++i)
        character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));

    const f32 standing = character.position().y;
    character.jump(8.0f);
    f32 peak = standing;
    for (u32 i = 0; i < 40; ++i)
    {
        // Asked for again every frame while in the air. Ignored, or the
        // character climbs the sky - and the peak below would run away.
        character.jump(8.0f);
        character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));
        peak = glm::max(peak, character.position().y);
    }
    CHECK(peak > standing + 1.0f);
    CHECK(peak < standing + 3.0f);

    // Left alone, he has to come back down to where he started.
    for (u32 i = 0; i < 120; ++i)
        character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));
    CHECK(character.grounded());
    CHECK(near(character.position().y, standing, 0.05f));
}

void testTeleportClearsTheFall()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::Vec3(-2.0f, 20.0f, 0.0f));
    for (u32 i = 0; i < 60; ++i)
        character.update(1.0f / 60.0f, mesh, Math::Mat4(1.0f));
    CHECK(character.verticalSpeed() < -1.0f);

    character.teleport(Math::Vec3(-4.0f, 3.0f, 0.0f));
    CHECK(near(character.verticalSpeed(), 0.0f));
    CHECK(near(character.position(), Math::Vec3(-4.0f, 3.0f, 0.0f)));
}

void testTrimeshRaycastFindsTheNearestTriangle()
{
    // Two floors: one at y = 0 and one at y = 2, so a ray fired down from
    // above crosses both and has to report the upper one.
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const usize base = vertices.size();
    for (usize i = 0; i < base; ++i)
        vertices.push_back(vertices[i] + Math::Vec3(0.0f, 2.0f, 0.0f));
    const usize indexBase = indices.size();
    for (usize i = 0; i < indexBase; ++i)
        indices.push_back(indices[i] + static_cast<u32>(base));

    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Ray ray;
    ray.origin = Math::Vec3(1.0f, 10.0f, 1.0f);
    ray.direction = Math::Vec3(0.0f, -1.0f, 0.0f);

    TrimeshShape::RayHit hit;
    CHECK(mesh.raycast(ray, 100.0f, hit));
    CHECK(near(hit.point.y, 2.0f));
    CHECK(near(hit.distance, 8.0f));
    CHECK(near(std::abs(hit.normal.y), 1.0f));

    // Short enough to fall between the two floors.
    CHECK(!mesh.raycast(ray, 4.0f, hit));

    // Fired upwards from below everything.
    ray.origin = Math::Vec3(1.0f, -10.0f, 1.0f);
    ray.direction = Math::Vec3(0.0f, 1.0f, 0.0f);
    CHECK(mesh.raycast(ray, 100.0f, hit));
    CHECK(near(hit.point.y, 0.0f));
}

void testTrimeshRaycastFromInsideTheBounds()
{
    // Three floors, one deep below: the root box then extends far under the
    // ray, so the exit distance from a box containing the origin is much
    // larger than the ray's own budget. The tree prune must not confuse the
    // two - a short suspension-style ray standing between floors has to hit
    // the one right beneath it (the bug this guards against culled the root
    // and reported open air).
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const usize base = vertices.size();
    const usize indexBase = indices.size();
    for (usize i = 0; i < base; ++i)
        vertices.push_back(vertices[i] + Math::Vec3(0.0f, 2.0f, 0.0f));
    for (usize i = 0; i < base; ++i)
        vertices.push_back(vertices[i] + Math::Vec3(0.0f, -10.0f, 0.0f));
    for (usize i = 0; i < indexBase; ++i)
        indices.push_back(indices[i] + static_cast<u32>(base));
    for (usize i = 0; i < indexBase; ++i)
        indices.push_back(indices[i] + static_cast<u32>(base * 2));

    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Ray ray;
    ray.origin = Math::Vec3(1.0f, 1.0f, 1.0f);
    ray.direction = Math::Vec3(0.0f, -1.0f, 0.0f);

    TrimeshShape::RayHit hit;
    CHECK(mesh.raycast(ray, 1.5f, hit));
    CHECK(near(hit.point.y, 0.0f));
    CHECK(near(hit.distance, 1.0f));
}

void testTrimeshOverlapSphereRejectsNearMisses()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    std::vector<u32> hits;
    mesh.overlapSphere(Math::Vec3(0.0f, 0.5f, 0.0f), 1.0f, hits);
    CHECK(!hits.empty());

    // Well above the plane: the tree may still offer candidates, and the
    // exact test is what has to throw them out.
    mesh.overlapSphere(Math::Vec3(0.0f, 5.0f, 0.0f), 1.0f, hits);
    CHECK(hits.empty());

    // Beyond the rim on X, level with it.
    mesh.overlapSphere(Math::Vec3(7.0f, 0.0f, 0.0f), 1.0f, hits);
    CHECK(hits.empty());
}

void testSharedEdgesAreDetected()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    // The quad's diagonal is used by both triangles; its other four edges are
    // the rim and belong to one each.
    u32 shared = 0;
    for (u32 i = 0; i < mesh.triangleCount(); ++i)
    {
        const TriangleShape triangle = mesh.triangle(i);
        for (u32 edge = 0; edge < 3; ++edge)
            if (triangle.edgeIsShared(edge))
                ++shared;
    }
    CHECK(shared == 2);
}

void testNoNormalCatchesOnASeam()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));
    const CapsuleShape capsule(0.4f, 0.8f);

    // Walk the capsule straight across the diagonal seam. Every contact along
    // the way has to push straight up: a normal tilted towards the seam is a
    // wall in the middle of flat ground, and the character stops on it.
    for (int step = -8; step <= 8; ++step)
    {
        const f32 x = static_cast<f32>(step) * 0.25f;
        const Math::Vec3 position(x, 1.15f, -x);
        std::vector<ContactManifold> manifolds;
        if (!Narrowphase::convexTrimesh(capsule, at(position), mesh, Math::Mat4(1.0f), manifolds))
            continue;
        for (const ContactManifold& manifold : manifolds)
        {
            CHECK(near(std::abs(manifold.normal.y), 1.0f, 1e-3f));
            CHECK(near(manifold.normal.x, 0.0f, 1e-3f));
            CHECK(near(manifold.normal.z, 0.0f, 1e-3f));
        }
    }
}

void testRimEdgeStillPushesOutwards()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));
    const SphereShape sphere(1.0f);

    // Just past the outer rim at x = 5 and level with it: a real edge, which
    // must not be flattened to the face normal or nothing ever falls off.
    std::vector<ContactManifold> manifolds;
    CHECK(Narrowphase::convexTrimesh(sphere, at(Math::Vec3(5.6f, 0.0f, 0.0f)), mesh, Math::Mat4(1.0f),
                                     manifolds));
    CHECK(!manifolds.empty());
    bool sideways = false;
    for (const ContactManifold& manifold : manifolds)
        if (std::abs(manifold.normal.x) > 0.5f)
            sideways = true;
    CHECK(sideways);
}

void testTrimeshWindingErrorsDetectsFlippedTriangles()
{
    // Two triangles sharing the diagonal, wound the same way around the quad
    // -> no errors. Flip one triangle and the shared edge is traversed the
    // same way by both, which windingErrors has to flag as a backface.
    std::vector<Math::Vec3> vertices = {Math::Vec3(0, 0, 0), Math::Vec3(2, 0, 0), Math::Vec3(2, 0, 2),
                                       Math::Vec3(0, 0, 2)};
    const std::vector<u32> goodIndices = {0, 2, 1, 0, 3, 2};
    const TrimeshShape good(vertices.data(), 4, goodIndices.data(), 6);
    std::vector<u32> errors;
    good.windingErrors(errors);
    CHECK(errors.empty());

    const std::vector<u32> flippedIndices = {0, 2, 1, 0, 2, 3};
    const TrimeshShape bad(vertices.data(), 4, flippedIndices.data(), 6);
    bad.windingErrors(errors);
    CHECK(!errors.empty());
    CHECK(errors.size() == 2);
}

void testSweepSphereRejectsHitsBeyondThePath()
{
    // A wall four units away, swept towards by only 0.5: the plane hit sits
    // at t >> 1 and has to be rejected - the sweep must not report a contact
    // it cannot reach in this move. A full-length sweep does reach it and
    // has to report the wall's normal at the right fraction.
    std::vector<Math::Vec3> vertices = {
        Math::Vec3(-5, 0, -5), Math::Vec3(5, 0, -5), Math::Vec3(5, 0, 5), Math::Vec3(-5, 0, 5),
        // wall at z = 4, facing -Z
        Math::Vec3(-5, 0, 4), Math::Vec3(5, 0, 4), Math::Vec3(5, 6, 4), Math::Vec3(-5, 6, 4)};
    const std::vector<u32> indices = {0, 2, 1, 0, 3, 2, 4, 6, 5, 4, 7, 6};
    const TrimeshShape mesh(vertices.data(), 8, indices.data(), 12);

    TrimeshShape::SweepHit hit;
    // Short sweep cannot reach the wall (the sphere surface is 3.0 short of
    // it, the path is 0.5) -> no hit, the character walks freely.
    CHECK(!mesh.sweepSphere(Math::Vec3(0.0f, 1.0f, 0.0f), 0.5f, Math::Vec3(0.0f, 0.0f, 0.5f), hit));

    // Full sweep: the sphere surface touches the wall at z = 3.5 of a 4.0
    // path, with the wall's outward normal.
    CHECK(mesh.sweepSphere(Math::Vec3(0.0f, 1.0f, 0.0f), 0.5f, Math::Vec3(0.0f, 0.0f, 4.0f), hit));
    CHECK(near(hit.t, 3.5f / 4.0f, 1e-3f));
    CHECK(near(hit.normal, Math::Vec3(0.0f, 0.0f, -1.0f), 1e-3f));
}

void testNarrowphaseMarginReportsSpeculativeContacts()
{
    // Two spheres 0.1 apart: no contact at margin 0, and a speculative contact
    // with a negative penetration (how far apart they are) within a margin.
    const SphereShape a(0.5f);
    const SphereShape b(0.5f);
    const Math::Mat4 ta = at(Math::Vec3(0.0f));
    const Math::Mat4 tb = at(Math::Vec3(1.1f, 0.0f, 0.0f)); // surfaces 0.1 apart

    ContactManifold manifold;
    CHECK(!Narrowphase::collide(a, ta, b, tb, manifold));

    CHECK(Narrowphase::collide(a, ta, b, tb, manifold, 0.2f));
    CHECK(manifold.count == 1);
    CHECK(manifold.points[0].penetration < 0.0f);
    CHECK(near(manifold.points[0].penetration, -0.1f, 1e-3f));
    CHECK(near(manifold.normal, Math::Vec3(1.0f, 0.0f, 0.0f), 1e-3f));

    // The dispatch, a sphere against a box, honours the margin too.
    const BoxShape box(Math::Vec3(0.5f));
    const Math::Mat4 tbox = at(Math::Vec3(1.1f, 0.0f, 0.0f)); // 0.1 from the sphere
    ContactManifold boxManifold;
    CHECK(Narrowphase::collide(a, ta, box, tbox, boxManifold, 0.2f));
    CHECK(boxManifold.points[0].penetration < 0.0f);
}

void testShapeInertiaTensors()
{
    // Solid sphere: (2/5) m r^2 on every axis, nothing off-diagonal.
    const SphereShape sphere(1.0f);
    const Math::Mat3 sphereI = sphere.inertia(5.0f);
    const f32 expected = 0.4f * 5.0f * 1.0f;
    CHECK(near(sphereI[0][0], expected));
    CHECK(near(sphereI[1][1], expected));
    CHECK(near(sphereI[2][2], expected));
    CHECK(near(sphereI[0][1], 0.0f, 1e-6f));

    // Box: m/12 * (dy^2 + dz^2) about x, using the FULL edge lengths.
    const BoxShape box(Math::Vec3(1.0f, 2.0f, 3.0f));
    const f32 m = 6.0f;
    const Math::Mat3 boxI = box.inertia(m);
    CHECK(near(boxI[0][0], m / 12.0f * (4.0f * 4.0f + 6.0f * 6.0f), 1e-3f));
    CHECK(near(boxI[1][1], m / 12.0f * (2.0f * 2.0f + 6.0f * 6.0f), 1e-3f));
    CHECK(near(boxI[2][2], m / 12.0f * (2.0f * 2.0f + 4.0f * 4.0f), 1e-3f));

    // Capsule: radial (x and z) moment greater than the axial (y) one, and
    // no coupling terms.
    const CapsuleShape capsule(0.5f, 1.0f);
    const Math::Mat3 capsuleI = capsule.inertia(2.0f);
    CHECK(near(capsuleI[0][1], 0.0f, 1e-6f));
    CHECK(capsuleI[0][0] > capsuleI[1][1]);
    CHECK(capsuleI[1][1] > 0.0f);
}

void testBoxFaceHelpers()
{
    const BoxShape box(Math::Vec3(1.0f, 2.0f, 3.0f));
    const Math::Mat4 identity = at(Math::Vec3(0.0f));

    // Faces in order -x,+x,-y,+y,-z,+z with the matching outward normals.
    const Math::Vec3 expected[6] = {Math::Vec3(-1, 0, 0), Math::Vec3(1, 0, 0),  Math::Vec3(0, -1, 0),
                                   Math::Vec3(0, 1, 0),  Math::Vec3(0, 0, -1), Math::Vec3(0, 0, 1)};
    for (u32 face = 0; face < 6; ++face)
    {
        CHECK(near(BoxShape::faceNormal(identity, face), expected[face], 1e-5f));
        const Math::Vec3 normal = expected[face];
        const f32 halfExtent =
            1.0f * (normal.x != 0.0f) + 2.0f * (normal.y != 0.0f) + 3.0f * (normal.z != 0.0f);
        const u8* corners = BoxShape::faceCorners(face);
        Math::Vec3 world[8];
        box.corners(identity, world);
        for (u32 c = 0; c < 4; ++c)
            // Every corner of the face sits on the face's plane.
            CHECK(near(glm::dot(world[corners[c]], normal), halfExtent, 1e-5f));
    }

    // Turned a quarter turn about z, the +x face normal becomes +y.
    const Math::Mat4 turned =
        at(Math::Vec3(0.0f), glm::angleAxis(glm::half_pi<f32>(), Math::Vec3(0, 0, 1)));
    CHECK(near(BoxShape::faceNormal(turned, 1), Math::Vec3(0.0f, 1.0f, 0.0f), 1e-3f));
}

void testTriangleFeatureIsInternal()
{
    // Two edges shared (bits 0 and 1): the face is always internal, shared
    // edges report the face normal, and a vertex is internal only when both
    // of its edges are shared.
    const TriangleShape triangle(Math::Vec3(0, 0, 0), Math::Vec3(1, 0, 0), Math::Vec3(0, 1, 0),
                                 /*sharedEdges=*/0b011);
    CHECK(triangle.edgeIsShared(0));
    CHECK(triangle.edgeIsShared(1));
    CHECK(!triangle.edgeIsShared(2));

    CHECK(triangle.featureIsInternal(TriangleFeature::Face));
    CHECK(triangle.featureIsInternal(TriangleFeature::Edge0));
    CHECK(triangle.featureIsInternal(TriangleFeature::Edge1));
    CHECK(!triangle.featureIsInternal(TriangleFeature::Edge2));

    // Vertex1 meets edges 0 and 1 (both shared) -> internal. Vertex2 meets
    // edge 1 (shared) and edge 2 (open rim) -> a real corner.
    CHECK(triangle.featureIsInternal(TriangleFeature::Vertex1));
    CHECK(!triangle.featureIsInternal(TriangleFeature::Vertex2));

    CHECK(near(triangle.rawNormal(), Math::Vec3(0.0f, 0.0f, 1.0f)));
}

void testConvexTrimeshBoxRestsOnFloorAndInCorner()
{
    // A floor of two triangles at y = 0 plus a wall at x = 2 facing -X. A box
    // resting on the floor must report up normals; a box wedged into the
    // corner must report ONE manifold per touching triangle with distinct
    // normals - never one averaged normal that belongs to neither.
    std::vector<Math::Vec3> vertices = {
        Math::Vec3(-5, 0, -5), Math::Vec3(5, 0, -5), Math::Vec3(5, 0, 5), Math::Vec3(-5, 0, 5),
        // wall facing -X at x = 2
        Math::Vec3(2, 0, -5), Math::Vec3(2, 0, 5), Math::Vec3(2, 6, 5), Math::Vec3(2, 6, -5)};
    const std::vector<u32> indices = {0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7};
    const TrimeshShape mesh(vertices.data(), 8, indices.data(), 12);

    const BoxShape box(Math::Vec3(0.5f));
    std::vector<ContactManifold> manifolds;

    // Resting on the floor (0.01 of the box below y = 0): every manifold is
    // a floor contact, normal pointing from the box DOWN to the floor.
    manifolds.clear();
    CHECK(Narrowphase::convexTrimesh(box, at(Math::Vec3(0.0f, 0.49f, 0.0f)), mesh,
                                     at(Math::Vec3(0.0f)), manifolds));
    CHECK(!manifolds.empty());
    for (const ContactManifold& manifold : manifolds)
        CHECK(near(manifold.normal, Math::Vec3(0.0f, -1.0f, 0.0f), 1e-3f));

    // Wedged into the corner: at least one floor manifold and one wall
    // manifold, each with its own normal (down for the floor, +x for the
    // wall the box presses into).
    manifolds.clear();
    CHECK(Narrowphase::convexTrimesh(box, at(Math::Vec3(1.51f, 0.49f, 0.0f)), mesh,
                                     at(Math::Vec3(0.0f)), manifolds));
    CHECK(manifolds.size() >= 2);
    bool up = false;
    bool wall = false;
    for (const ContactManifold& manifold : manifolds)
    {
        if (near(manifold.normal, Math::Vec3(0.0f, -1.0f, 0.0f), 1e-3f))
            up = true;
        if (near(manifold.normal, Math::Vec3(1.0f, 0.0f, 0.0f), 1e-3f))
            wall = true;
    }
    CHECK(up);
    CHECK(wall);
}

void testCharacterRigidBodyOnGround()
{
    BoxShape floor(Math::Vec3(5.0f, 0.5f, 5.0f));
    RigidBody floorBody;
    floorBody.setBodyType(BodyType::Static);
    floorBody.setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));

    PhysicsWorld world;
    BodyEntry floorEntry;
    floorEntry.body = &floorBody;
    floorEntry.shape = &floor;
    world.addBody(floorEntry);

    CharacterRigidBody character;
    character.setShape(0.4f, 1.2f);
    character.addToWorld(world, Math::Vec3(0.0f, 1.0f, 0.0f));
    character.postSimulation(0.05f);

    CHECK(character.groundState() == CharacterRigidBody::GroundState::OnGround);
    CHECK(character.isSupported());
    CHECK(near(character.groundNormal(), Math::Vec3(0.0f, 1.0f, 0.0f), 1e-2f));

    character.removeFromWorld();
}

void testCharacterRigidBodyOnSteepGround()
{
    const f32 angleDegrees = 70.0f;
    const Math::Quaternion rotation =
        glm::angleAxis(glm::radians(angleDegrees), Math::Vec3(0.0f, 0.0f, 1.0f));
    const Math::Vec3 normal = glm::normalize(rotation * Math::Vec3(0.0f, 1.0f, 0.0f));

    BoxShape ramp(Math::Vec3(5.0f, 0.5f, 5.0f));
    RigidBody rampBody;
    rampBody.setBodyType(BodyType::Static);
    rampBody.setPosition(Math::Vec3(0.0f));
    rampBody.setOrientation(rotation);

    PhysicsWorld world;
    BodyEntry rampEntry;
    rampEntry.body = &rampBody;
    rampEntry.shape = &ramp;
    world.addBody(rampEntry);

    const Math::Vec3 topFaceCentre = rotation * Math::Vec3(0.0f, 0.5f, 0.0f);
    CharacterRigidBody character;
    character.setShape(0.4f, 0.0f);
    character.addToWorld(world, topFaceCentre + normal * 0.38f);
    character.postSimulation(0.05f);

    CHECK(character.groundState() == CharacterRigidBody::GroundState::OnSteepGround);
    CHECK(character.isSupported());
    CHECK(near(character.groundNormal(), normal, 1e-2f));

    character.removeFromWorld();
}

void testCharacterRigidBodyInAir()
{
    PhysicsWorld world;
    CharacterRigidBody character;
    character.setShape(0.4f, 1.2f);
    character.addToWorld(world, Math::Vec3(0.0f, 100.0f, 0.0f));
    character.postSimulation(0.05f);

    CHECK(character.groundState() == CharacterRigidBody::GroundState::InAir);
    CHECK(!character.isSupported());

    character.removeFromWorld();
}

void testCharacterRigidBodyPushesALightDynamicBox()
{
    PhysicsWorld world;
    world.setGravity(Math::Vec3(0.0f));

    BoxShape boxShape(Math::Vec3(0.3f));
    RigidBody boxBody;
    boxBody.setMass(0.2f);
    boxBody.setInertiaTensor(boxShape.inertia(0.2f));
    boxBody.setPosition(Math::Vec3(0.68f, 0.0f, 0.0f));

    BodyEntry boxEntry;
    boxEntry.body = &boxBody;
    boxEntry.shape = &boxShape;
    boxEntry.friction = 0.0f;
    world.addBody(boxEntry);

    CharacterRigidBody character;
    character.setShape(0.4f, 1.2f);
    character.setFriction(0.0f);
    character.addToWorld(world, Math::Vec3(0.0f, 0.0f, 0.0f));
    character.setLinearVelocity(Math::Vec3(5.0f, 0.0f, 0.0f));

    world.step(1.0f / 120.0f);
    character.postSimulation(0.05f);

    CHECK(boxBody.velocity().x > 0.01f);

    character.removeFromWorld();
}

void testCharacterRigidBodyMovesAfterFallingAsleep()
{
    BoxShape floorShape(Math::Vec3(10.0f, 0.5f, 10.0f));
    RigidBody floor;
    floor.setBodyType(BodyType::Static);
    floor.setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));

    PhysicsWorld world;
    BodyEntry floorEntry;
    floorEntry.body = &floor;
    floorEntry.shape = &floorShape;
    world.addBody(floorEntry);

    CharacterRigidBody character;
    character.setShape(0.4f, 1.2f);
    character.addToWorld(world, Math::Vec3(0.0f, 1.2f, 0.0f));

    for (u32 i = 0; i < 600; ++i)
        world.step(1.0f / 120.0f);

    const Math::Vec3 rested = character.position();

    for (u32 i = 0; i < 120; ++i)
    {
        character.setLinearVelocity(Math::Vec3(3.0f, character.linearVelocity().y, 0.0f));
        world.step(1.0f / 120.0f);
    }

    CHECK(character.position().x - rested.x > 1.0f);

    character.removeFromWorld();
}

void testDynamicBoxDroppedFromHeightRestsOnTrimesh()
{
    std::vector<Math::Vec3> vertices;
    std::vector<u32> indices;
    const f32 extent = 12.0f;
    const u32 segments = 6;
    for (u32 z = 0; z <= segments; ++z)
        for (u32 x = 0; x <= segments; ++x)
            vertices.push_back(Math::Vec3(-extent + 2.0f * extent * x / segments, 0.0f,
                                         -extent + 2.0f * extent * z / segments));
    for (u32 z = 0; z < segments; ++z)
        for (u32 x = 0; x < segments; ++x)
        {
            const u32 a = z * (segments + 1) + x;
            const u32 b = a + 1;
            const u32 c = a + segments + 1;
            const u32 d = c + 1;
            indices.insert(indices.end(), {a, c, b, b, c, d});
        }
    TrimeshShape floorMesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                           static_cast<u32>(indices.size()));

    RigidBody floor;
    floor.setBodyType(BodyType::Static);
    floor.setPosition(Math::Vec3(0.0f));

    PhysicsWorld world;
    world.setGravity(Math::Vec3(0.0f, -20.0f, 0.0f));

    BoxShape boxShape(Math::Vec3(0.4f));
    RigidBody box;
    box.setMass(4.0f);
    box.setInertiaTensor(boxShape.inertia(4.0f));
    box.setPosition(Math::Vec3(1.7f, 8.0f, 1.3f));
    box.setDamping(0.999f, 0.999f);

    BodyEntry boxEntry;
    boxEntry.body = &box;
    boxEntry.shape = &boxShape;
    boxEntry.friction = 0.6f;
    boxEntry.restitution = 0.05f;
    world.addBody(boxEntry);

    BodyEntry floorEntry;
    floorEntry.body = &floor;
    floorEntry.shape = &floorMesh;
    floorEntry.friction = 0.8f;
    world.addBody(floorEntry);

    for (u32 i = 0; i < 600; ++i)
        world.step(1.0f / 120.0f);

    CHECK(std::isfinite(box.position().y));
    CHECK(box.position().y > 0.3f);
    CHECK(box.position().y < 1.0f);
}

void testWorldRaycastFindsTheNearestBody()
{
    SphereShape sphere(1.0f);
    RigidBody nearBody;
    nearBody.setBodyType(BodyType::Static);
    nearBody.setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    RigidBody farBody;
    farBody.setBodyType(BodyType::Static);
    farBody.setPosition(Math::Vec3(5.0f, 0.0f, 0.0f));

    PhysicsWorld world;
    BodyEntry entry;
    entry.shape = &sphere;
    entry.body = &nearBody;
    const u32 nearId = world.addBody(entry);
    entry.body = &farBody;
    world.addBody(entry);

    Ray ray;
    ray.origin = Math::Vec3(-5.0f, 0.0f, 0.0f);
    ray.direction = Math::Vec3(1.0f, 0.0f, 0.0f);

    WorldRayHit hit;
    CHECK(world.raycast(ray, 100.0f, QueryFilter(), hit));
    CHECK(hit.body == nearId);
    CHECK(near(hit.distance, 4.0f));
    CHECK(near(hit.point, Math::Vec3(-1.0f, 0.0f, 0.0f)));
    CHECK(near(hit.normal, Math::Vec3(-1.0f, 0.0f, 0.0f)));
}

void testWorldRaycastMaskExcludesLayer()
{
    SphereShape sphere(1.0f);
    RigidBody body;
    body.setBodyType(BodyType::Static);
    body.setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));

    PhysicsWorld world;
    BodyEntry entry;
    entry.shape = &sphere;
    entry.body = &body;
    entry.filter.group = 2;
    world.addBody(entry);

    Ray ray;
    ray.origin = Math::Vec3(-5.0f, 0.0f, 0.0f);
    ray.direction = Math::Vec3(1.0f, 0.0f, 0.0f);

    WorldRayHit hit;
    CHECK(world.raycast(ray, 100.0f, QueryFilter(), hit));
    QueryFilter onlyGroup1;
    onlyGroup1.collision.mask = 1u;
    CHECK(!world.raycast(ray, 100.0f, onlyGroup1, hit));
}

void testWorldOverlapSphereRespectsMask()
{
    SphereShape shape(1.0f);
    RigidBody body;
    body.setBodyType(BodyType::Static);
    body.setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));

    PhysicsWorld world;
    BodyEntry entry;
    entry.shape = &shape;
    entry.body = &body;
    entry.filter.group = 4;
    const u32 id = world.addBody(entry);

    std::vector<u32> hits;
    world.overlapSphere(Math::Vec3(0.5f, 0.0f, 0.0f), 1.0f, QueryFilter(), hits);
    CHECK(hits.size() == 1);
    CHECK(hits[0] == id);

    QueryFilter onlyGroup1;
    onlyGroup1.collision.mask = 1u;
    world.overlapSphere(Math::Vec3(0.5f, 0.0f, 0.0f), 1.0f, onlyGroup1, hits);
    CHECK(hits.empty());

    world.overlapSphere(Math::Vec3(50.0f, 0.0f, 0.0f), 1.0f, QueryFilter(), hits);
    CHECK(hits.empty());
}

void testWorldStepSkipsIncompatibleMasks()
{
    BoxShape groundShape(Math::Vec3(5.0f, 0.5f, 5.0f));
    BoxShape dropperShape(Math::Vec3(0.5f));

    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));

    RigidBody dropper;
    dropper.setMass(1.0f);
    dropper.setInertiaTensor(dropperShape.inertia(1.0f));
    dropper.setPosition(Math::Vec3(0.0f, 1.5f, 0.0f));

    PhysicsWorld world;
    world.setGravity(Math::Vec3(0.0f, -9.81f, 0.0f));

    BodyEntry entry;
    entry.shape = &groundShape;
    entry.body = &ground;
    entry.filter = {1, 1};
    world.addBody(entry);

    entry.shape = &dropperShape;
    entry.body = &dropper;
    entry.filter = {2, 2};
    world.addBody(entry);

    for (u32 i = 0; i < 200; ++i)
        world.step(1.0f / 120.0f);

    CHECK(dropper.position().y < -1.0f);
}

void testBodyHandleRejectsReusedSlot()
{
    SphereShape shape(1.0f);
    RigidBody first;
    first.setBodyType(BodyType::Static);
    RigidBody second;
    second.setBodyType(BodyType::Static);

    PhysicsWorld world;
    BodyEntry entry;
    entry.body = &first;
    entry.shape = &shape;
    const u32 firstId = world.addBody(entry);
    const BodyHandle stale = world.bodyHandle(firstId);
    CHECK(world.body(stale) != nullptr);

    world.removeBody(firstId);
    CHECK(world.body(stale) == nullptr);

    entry.body = &second;
    const u32 secondId = world.addBody(entry);
    CHECK(secondId == firstId);
    CHECK(world.body(stale) == nullptr);
    CHECK(world.body(world.bodyHandle(secondId)) != nullptr);
}

void testWorldRemovalKeepsDenseStorageAndStableIds()
{
    SphereShape shape(1.0f);
    RigidBody bodies[4];
    PhysicsWorld world;
    u32 ids[4]{};
    BodyHandle handles[3];

    BodyEntry entry;
    entry.shape = &shape;
    for (u32 i = 0; i < 3; ++i)
    {
        bodies[i].setBodyType(BodyType::Static);
        bodies[i].setPosition(Math::Vec3(static_cast<f32>(i) * 4.0f, 0.0f, 0.0f));
        entry.body = &bodies[i];
        ids[i] = world.addBody(entry);
        handles[i] = world.bodyHandle(ids[i]);
    }

    world.removeBody(ids[1]);
    CHECK(world.bodyCount() == 2);
    CHECK(world.body(ids[1]) == nullptr);
    CHECK(world.body(ids[0])->body == &bodies[0]);
    CHECK(world.body(ids[2])->body == &bodies[2]);
    CHECK(world.body(handles[2])->body == &bodies[2]);

    bodies[3].setBodyType(BodyType::Static);
    entry.body = &bodies[3];
    ids[3] = world.addBody(entry);
    CHECK(ids[3] == ids[1]);
    CHECK(world.bodyCount() == 3);
    CHECK(world.body(handles[1]) == nullptr);
    CHECK(world.body(ids[2])->body == &bodies[2]);
}

void testWorldAllowsMutationFromCollisionCallback()
{
    struct Mutation
    {
        PhysicsWorld* world = nullptr;
        BodyEntry addition;
        BodyHandle removedHandle;
        u32 removed = 0xFFFFFFFFu;
        u32 added = 0xFFFFFFFFu;
        bool done = false;
    };

    BoxShape shape(Math::Vec3(0.5f));
    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));
    RigidBody box;
    box.setMass(1.0f);
    box.setInertiaTensor(shape.inertia(1.0f));
    box.setPosition(Math::Vec3(0.0f, 0.45f, 0.0f));
    RigidBody spare;
    spare.setBodyType(BodyType::Static);
    spare.setPosition(Math::Vec3(20.0f, 0.0f, 0.0f));

    PhysicsWorld world;
    BodyEntry entry;
    entry.shape = &shape;
    entry.body = &ground;
    const u32 groundId = world.addBody(entry);
    entry.body = &box;
    const u32 boxId = world.addBody(entry);

    Mutation mutation;
    mutation.world = &world;
    mutation.addition.shape = &shape;
    mutation.addition.body = &spare;
    world.setEventCallback(
        [](const ContactEventInfo& info, void* user)
        {
            Mutation& mutation = *static_cast<Mutation*>(user);
            if (mutation.done || info.event != ContactEvent::Enter)
                return;
            mutation.done = true;
            mutation.removed = info.bodyB;
            mutation.removedHandle = mutation.world->bodyHandle(mutation.removed);
            mutation.world->removeBody(mutation.removed);
            mutation.added = mutation.world->addBody(mutation.addition);
            // Events are dispatched after the solver releases its temporary
            // references, so mutation is already safe and visible here.
            CHECK(mutation.world->body(mutation.removedHandle) == nullptr);
            CHECK(mutation.added != mutation.removed);
            CHECK(mutation.world->body(mutation.added)->body == mutation.addition.body);
        },
        &mutation);

    world.step(1.0f / 120.0f);
    CHECK(mutation.done);
    CHECK(mutation.removed == boxId);
    CHECK(world.bodyCount() == 2);
    CHECK(world.body(groundId) != nullptr);
    CHECK(world.body(mutation.removedHandle) == nullptr);
    CHECK(world.body(mutation.added)->body == &spare);
}

void testWorldAreaForces()
{
    SphereShape shape(0.25f);
    RigidBody nearBody;
    RigidBody farBody;
    RigidBody outsideBody;
    RigidBody staticBody;
    RigidBody* bodies[] = {&nearBody, &farBody, &outsideBody, &staticBody};
    const f32 positions[] = {1.0f, 3.0f, 6.0f, 1.0f};

    PhysicsWorld world;
    world.setGravity(Math::Vec3(0.0f));
    BodyEntry entry;
    entry.shape = &shape;
    for (u32 i = 0; i < 4; ++i)
    {
        if (i == 3)
            bodies[i]->setBodyType(BodyType::Static);
        else
        {
            bodies[i]->setMass(1.0f);
            bodies[i]->setInertiaTensor(shape.inertia(1.0f));
            bodies[i]->setCanSleep(false);
        }
        bodies[i]->setPosition(Math::Vec3(positions[i], 0.0f, 0.0f));
        entry.body = bodies[i];
        world.addBody(entry);
    }

    CHECK(world.applyRadialImpulse(Math::Vec3(0.0f), 5.0f, 10.0f) == 2);
    CHECK(nearBody.velocity().x > farBody.velocity().x);
    CHECK(farBody.velocity().x > 0.0f);
    CHECK(near(outsideBody.velocity(), Math::Vec3(0.0f)));
    CHECK(near(staticBody.velocity(), Math::Vec3(0.0f)));

    nearBody.setVelocity(Math::Vec3(0.0f));
    farBody.setVelocity(Math::Vec3(0.0f));
    CHECK(world.addRadialForce(Math::Vec3(0.0f), 5.0f, -10.0f) == 2);
    world.step(0.1f);
    CHECK(nearBody.velocity().x < farBody.velocity().x);
    CHECK(farBody.velocity().x < 0.0f);

    nearBody.setVelocity(Math::Vec3(0.0f));
    farBody.setVelocity(Math::Vec3(0.0f));
    CHECK(world.addDirectionalForce(Math::Vec3(0.0f), 2.0f, Math::Vec3(0.0f, 0.0f, 8.0f)) == 1);
    world.step(0.1f);
    CHECK(nearBody.velocity().z > 0.0f);
    CHECK(near(farBody.velocity().z, 0.0f));
}

void testPlaneShape()
{
    const PlaneShape plane(Math::Vec3(0.0f, 2.0f, 0.0f), 2.0f);
    const SphereShape sphere(1.0f);
    const Math::Mat4 planeTransform = at(Math::Vec3(0.0f, 1.0f, 0.0f));
    ContactManifold manifold;

    CHECK(Narrowphase::collide(sphere, at(Math::Vec3(0.0f, 3.5f, 0.0f)), plane,
                               planeTransform, manifold));
    CHECK(near(manifold.normal, Math::Vec3(0.0f, -1.0f, 0.0f)));
    CHECK(near(manifold.points[0].penetration, 0.5f));
    CHECK(near(manifold.points[0].position, Math::Vec3(0.0f, 3.0f, 0.0f)));

    CHECK(Narrowphase::collide(plane, planeTransform, sphere,
                               at(Math::Vec3(0.0f, 3.5f, 0.0f)), manifold));
    CHECK(near(manifold.normal, Math::Vec3(0.0f, 1.0f, 0.0f)));

    CHECK(!Narrowphase::collide(sphere, at(Math::Vec3(0.0f, 4.1f, 0.0f)), plane,
                                planeTransform, manifold));
    CHECK(Narrowphase::collide(sphere, at(Math::Vec3(0.0f, 4.1f, 0.0f)), plane,
                               planeTransform, manifold, 0.2f));
    CHECK(near(manifold.points[0].penetration, -0.1f));

    Ray ray;
    ray.origin = Math::Vec3(0.0f, 5.0f, 0.0f);
    ray.direction = Math::Vec3(0.0f, -1.0f, 0.0f);
    ShapeRayHit hit;
    CHECK(Narrowphase::raycast(plane, planeTransform, ray, 10.0f, hit));
    CHECK(near(hit.distance, 2.0f));
    CHECK(near(hit.point, Math::Vec3(0.0f, 3.0f, 0.0f)));
    CHECK(!Narrowphase::overlapSphere(plane, planeTransform, Math::Vec3(0.0f, 4.2f, 0.0f),
                                     1.0f));
    CHECK(Narrowphase::overlapSphere(plane, planeTransform, Math::Vec3(0.0f, 3.5f, 0.0f),
                                    1.0f));
}

void testPointJoint()
{
    SphereShape shape(0.1f);
    RigidBody fixed;
    fixed.setBodyType(BodyType::Static);
    RigidBody moving;
    moving.setMass(1.0f);
    moving.setInertiaTensor(shape.inertia(1.0f));
    moving.setPosition(Math::Vec3(2.0f, 0.0f, 0.0f));
    moving.setCanSleep(false);

    PhysicsWorld world;
    world.setGravity(Math::Vec3(0.0f));
    BodyEntry entry;
    entry.shape = &shape;
    entry.filter.mask = 0;
    entry.body = &fixed;
    world.addBody(entry);
    entry.body = &moving;
    const u32 movingId = world.addBody(entry);

    PointJoint joint(fixed, Math::Vec3(0.0f), moving, Math::Vec3(0.0f));
    world.addJoint(&joint);
    CHECK(world.jointCount() == 1);
    for (u32 i = 0; i < 120; ++i)
        world.step(1.0f / 120.0f);
    CHECK(glm::length(joint.worldAnchorB() - joint.worldAnchorA()) < 0.01f);

    world.removeBody(movingId);
    CHECK(world.jointCount() == 0);
}

void testPointJointCarMoves()
{
    PlaneShape groundShape(Math::Vec3(0.0f, 1.0f, 0.0f));
    BoxShape chassisShape(Math::Vec3(0.8f, 0.25f, 1.4f));
    SphereShape wheelShape(0.4f);
    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    RigidBody chassis;
    chassis.setMass(8.0f);
    chassis.setInertiaTensor(chassisShape.inertia(8.0f));
    chassis.setPosition(Math::Vec3(0.0f, 0.85f, 0.0f));
    chassis.setCanSleep(false);
    RigidBody wheels[4];
    const Math::Vec3 offsets[4] = {
        Math::Vec3(-0.9f, -0.45f, -0.9f), Math::Vec3(0.9f, -0.45f, -0.9f),
        Math::Vec3(-0.9f, -0.45f, 0.9f), Math::Vec3(0.9f, -0.45f, 0.9f)};

    PhysicsWorld world;
    BodyEntry groundEntry;
    groundEntry.body = &ground;
    groundEntry.shape = &groundShape;
    groundEntry.friction = 1.0f;
    world.addBody(groundEntry);
    CollisionFilter carFilter;
    carFilter.group = 2;
    carFilter.mask = 1;
    BodyEntry chassisEntry;
    chassisEntry.body = &chassis;
    chassisEntry.shape = &chassisShape;
    chassisEntry.filter = carFilter;
    chassisEntry.friction = 1.0f;
    world.addBody(chassisEntry);

    std::vector<PointJoint> joints;
    joints.reserve(4);
    for (u32 i = 0; i < 4; ++i)
    {
        wheels[i].setMass(1.0f);
        wheels[i].setInertiaTensor(wheelShape.inertia(1.0f));
        wheels[i].setPosition(chassis.position() + offsets[i]);
        wheels[i].setCanSleep(false);
        BodyEntry wheelEntry;
        wheelEntry.body = &wheels[i];
        wheelEntry.shape = &wheelShape;
        wheelEntry.filter = carFilter;
        wheelEntry.friction = 1.0f;
        world.addBody(wheelEntry);
        joints.emplace_back(chassis, wheels[i], wheels[i].position());
        joints.back().setMotor(Math::Vec3(1.0f, 0.0f, 0.0f), 14.0f, 25.0f);
        world.addJoint(&joints.back());
    }

    ContactSolverSettings settings;
    settings.velocityIterations = 16;
    settings.positionIterations = 6;
    world.setSolverSettings(settings);
    f32 maximumAnchorError = 0.0f;
    for (u32 step = 0; step < 1200; ++step)
    {
        if (step == 600)
            for (u32 i = 0; i < 4; ++i)
            {
                const f32 side = offsets[i].x < 0.0f ? -1.0f : 1.0f;
                joints[i].setMotor(Math::Vec3(1.0f, 0.0f, 0.0f), 10.0f + 5.0f * side,
                                   25.0f);
            }
        world.step(1.0f / 120.0f);
        for (const PointJoint& joint : joints)
            maximumAnchorError =
                glm::max(maximumAnchorError,
                         glm::length(joint.worldAnchorB() - joint.worldAnchorA()));
    }

    CHECK(std::abs(chassis.position().z) > 0.5f);
    CHECK(maximumAnchorError < 0.1f);
    for (const PointJoint& joint : joints)
        CHECK(glm::length(joint.worldAnchorB() - joint.worldAnchorA()) < 0.1f);
}

// ------------------------------------------------------------ convex hull

void testConvexHullShapeMatchesBox()
{
    const Shard shard = buildCubeShard(1.0f);
    const ConvexHullShape hull(shard);
    const BoxShape box(Math::Vec3(1.0f));
    const Math::Mat4 identity = at(Math::Vec3(0.0f));

    CHECK(near(hull.support(identity, Math::Vec3(1, 1, 1)), box.support(identity, Math::Vec3(1, 1, 1))));
    CHECK(near(hull.support(identity, Math::Vec3(-1, 1, -1)),
              box.support(identity, Math::Vec3(-1, 1, -1))));

    const AABB hullBounds = hull.bounds(identity);
    const AABB boxBounds = box.bounds(identity);
    CHECK(near(hullBounds.min, boxBounds.min));
    CHECK(near(hullBounds.max, boxBounds.max));

    const Math::Mat3 hullI = hull.inertia(6.0f);
    const Math::Mat3 boxI = box.inertia(6.0f);
    CHECK(near(hullI[0][0], boxI[0][0], 1e-2f));
    CHECK(near(hullI[1][1], boxI[1][1], 1e-2f));
    CHECK(near(hullI[2][2], boxI[2][2], 1e-2f));
    CHECK(near(hullI[0][1], 0.0f, 1e-2f));
    CHECK(near(hullI[0][2], 0.0f, 1e-2f));
    CHECK(near(hullI[1][2], 0.0f, 1e-2f));
}

void testConvexHullInertiaMatchesBoxClosedForm()
{
    // The one shape with an independent formula to check the tetrahedron
    // decomposition against: a non-cubic box, so a bug that only shows up
    // once the three axes differ cannot hide behind a cube's symmetry.
    const Shard shard = buildCubeShard(1.0f);
    Shard scaled = shard;
    for (Math::Vec3& vertex : scaled.vertices)
        vertex *= Math::Vec3(1.0f, 2.0f, 3.0f);
    const ConvexHullShape hull(scaled);
    const BoxShape box(Math::Vec3(1.0f, 2.0f, 3.0f));

    const f32 mass = 6.0f;
    const Math::Mat3 hullI = hull.inertia(mass);
    const Math::Mat3 boxI = box.inertia(mass);
    CHECK(near(hullI[0][0], boxI[0][0], 1e-2f));
    CHECK(near(hullI[1][1], boxI[1][1], 1e-2f));
    CHECK(near(hullI[2][2], boxI[2][2], 1e-2f));
}

void testConvexHullBoxMatchesBoxBoxInvariants()
{
    const Shard shard = buildCubeShard(1.0f);
    const ConvexHullShape hull(shard);
    const BoxShape ground(Math::Vec3(3.0f, 0.5f, 3.0f));

    // Ground top face is at y=0; hull half-extent 1 centred at y=0.8 puts
    // its bottom face at y=-0.2, a 0.2 overlap - the same setup
    // testBoxBoxFaceContact() uses for two unit boxes.
    const Math::Mat4 hullTransform = at(Math::Vec3(0.0f, 0.8f, 0.0f));
    const Math::Mat4 groundTransform = at(Math::Vec3(0.0f, -0.5f, 0.0f));

    ContactManifold manifold;
    // Hull is A, above; ground is B, below - A to B points down.
    CHECK(Narrowphase::collide(hull, hullTransform, ground, groundTransform, manifold));
    CHECK(manifold.count == 4);
    CHECK(near(manifold.normal, Math::Vec3(0, -1, 0), 1e-3f));
    for (u32 i = 0; i < manifold.count; ++i)
        CHECK(near(manifold.points[i].penetration, 0.2f, 1e-2f));

    f32 spread = 0.0f;
    for (u32 i = 0; i < manifold.count; ++i)
        for (u32 j = i + 1; j < manifold.count; ++j)
            spread = glm::max(
                spread, glm::length(manifold.points[i].position - manifold.points[j].position));
    CHECK(spread > 1.5f);

    // Pulled clear, no contact.
    const Math::Mat4 clear = at(Math::Vec3(0.0f, 5.0f, 0.0f));
    CHECK(!Narrowphase::collide(hull, clear, ground, groundTransform, manifold));

    // The box-first ordering has to give the mirrored normal.
    ContactManifold flipped;
    CHECK(Narrowphase::collide(ground, groundTransform, hull, hullTransform, flipped));
    CHECK(near(flipped.normal, Math::Vec3(0, 1, 0), 1e-3f));
}

void testConvexHullSphereBasicContact()
{
    const Shard shard = buildCubeShard(1.0f);
    const ConvexHullShape hull(shard);
    const SphereShape sphere(0.5f);

    ContactManifold manifold;
    // Sphere resting on the hull's +y face, overlapping by 0.2.
    CHECK(Narrowphase::collide(hull, at(Math::Vec3(0.0f)), sphere, at(Math::Vec3(0.0f, 1.3f, 0.0f)),
                               manifold));
    CHECK(manifold.count == 1);
    CHECK(near(manifold.normal, Math::Vec3(0, 1, 0), 1e-2f));
    CHECK(near(manifold.points[0].penetration, 0.2f, 1e-2f));

    CHECK(!Narrowphase::collide(hull, at(Math::Vec3(0.0f)), sphere, at(Math::Vec3(0.0f, 3.0f, 0.0f)),
                                manifold));
}

void testConvexHullCapsuleBasicContact()
{
    const Shard shard = buildCubeShard(1.0f);
    const ConvexHullShape hull(shard);
    const CapsuleShape capsule(0.3f, 0.6f);

    ContactManifold manifold;
    // Capsule standing on the hull's +y face, overlapping by 0.15.
    CHECK(Narrowphase::collide(hull, at(Math::Vec3(0.0f)), capsule,
                               at(Math::Vec3(0.0f, 1.85f, 0.0f)), manifold));
    CHECK(manifold.count >= 1);
    CHECK(std::abs(manifold.normal.y) > 0.9f);
    CHECK(manifold.points[0].penetration > 0.0f);

    CHECK(!Narrowphase::collide(hull, at(Math::Vec3(0.0f)), capsule,
                                at(Math::Vec3(0.0f, 4.0f, 0.0f)), manifold));
}

void testConvexHullConvexHullOverlapAndSeparation()
{
    // Two cells of the same box, split down the middle - they sit face to
    // face at their natural centroids, so nudging one towards the other
    // along the line between the centroids is what forces a real overlap.
    std::vector<Math::Vec3> boxCorners;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2)
                boxCorners.push_back(Math::Vec3(sx, sy, sz));
    std::vector<Math::Vec3> voronoiPoints = {Math::Vec3(-0.5f, 0, 0), Math::Vec3(0.5f, 0, 0)};
    std::vector<Shard> shards;
    VoronoiShatter::shatter(boxCorners, voronoiPoints, shards);
    CHECK(shards.size() == 2);
    if (shards.size() != 2)
        return;

    const ConvexHullShape hullA(shards[0]);
    const ConvexHullShape hullB(shards[1]);
    const Math::Vec3 direction = glm::normalize(shards[0].centroid - shards[1].centroid);

    ContactManifold manifold;
    // Pulled well apart: no collision.
    const Math::Vec3 farTransformB = shards[1].centroid - direction * 2.0f;
    CHECK(!Narrowphase::collide(hullA, at(shards[0].centroid), hullB, at(farTransformB), manifold));

    // Pushed together past their natural, touching layout: a real overlap.
    const Math::Vec3 nearTransformB = shards[1].centroid + direction * 0.3f;
    CHECK(Narrowphase::collide(hullA, at(shards[0].centroid), hullB, at(nearTransformB), manifold));
    CHECK(manifold.count >= 1);
    CHECK(near(glm::length(manifold.normal), 1.0f, 1e-3f));
    CHECK(manifold.points[0].penetration > 0.0f);
    CHECK(std::isfinite(manifold.points[0].penetration));
}

void testConvexHullDegenerateShardIsFinite()
{
    // A very thin sliver, the shape a "glass pane" shatter produces - checks
    // support/bounds/inertia never divide by the near-zero volume into a NaN.
    Shard shard = buildCubeShard(1.0f);
    for (Math::Vec3& vertex : shard.vertices)
        vertex.z *= 0.0005f;
    const ConvexHullShape hull(shard);
    const Math::Mat4 identity = at(Math::Vec3(0.0f));

    const Math::Vec3 support = hull.support(identity, Math::Vec3(0.3f, 1.0f, 0.2f));
    CHECK(std::isfinite(support.x) && std::isfinite(support.y) && std::isfinite(support.z));

    const AABB bounds = hull.bounds(identity);
    CHECK(std::isfinite(bounds.min.x) && std::isfinite(bounds.max.x));

    const Math::Mat3 inertia = hull.inertia(1.0f);
    for (u32 col = 0; col < 3; ++col)
        for (u32 row = 0; row < 3; ++row)
            CHECK(std::isfinite(inertia[col][row]));
}

void testConvexHullConvexHullCoincidentFacesDoNotCrash()
{
    // Same hull, same transform - zero gap, exactly face to face on every
    // side at once. Must report a collision and terminate, not spin.
    const Shard shard = buildCubeShard(1.0f);
    const ConvexHullShape hullA(shard);
    const ConvexHullShape hullB(shard);
    const Math::Mat4 transform = at(Math::Vec3(0.0f));

    ContactManifold manifold;
    CHECK(Narrowphase::collide(hullA, transform, hullB, transform, manifold));
    CHECK(std::isfinite(manifold.points[0].penetration));
    CHECK(std::isfinite(manifold.normal.x));
}

} // namespace

int main()
{
    testPointJointCarMoves();
    testPointJoint();
    testPlaneShape();
    testCharacterLandsAndStandsOnTheFloor();
    testCharacterDoesNotWalkThroughAWall();
    testCharacterDoesNotTeleportToAFarWall();
    testCharacterReportsWallAndCeiling();
    testCharacterMoveAndSlideGodotStyle();
    testCharacterSetVerticalSpeedJumpsAnytime();
    testCharacterApplyFloorSnapIsPublic();
    testSlideCameraPullsBackFromAWall();
    testSlideCameraReturnsTheDesiredPositionWhenClear();
    testCharacterCrossesSeamsWithoutStopping();
    testCharacterClimbsAStepAndIsStoppedByAWall();
    testCharacterDoesNotHopAtAPlatformSeam();
    testCharacterIsNeverLaunchedByAContact();
    testGroundedNeverFlickersWhileStandingStill();
    testCharacterFallsAndLandsUnderGravity();
    testCharacterJumpsOnlyFromTheGround();
    testTeleportClearsTheFall();
    testTrimeshRaycastFindsTheNearestTriangle();
    testTrimeshRaycastFromInsideTheBounds();
    testTrimeshOverlapSphereRejectsNearMisses();
    testSharedEdgesAreDetected();
    testNoNormalCatchesOnASeam();
    testRimEdgeStillPushesOutwards();
    testClosestPointOnTriangle();
    testTrimeshTreeFindsOnlyNearbyTriangles();
    testSphereOnTriangle();
    testBoxOnTriangleGetsAPatch();
    testCapsuleOnTriangle();
    testConvexTrimeshSpansBothTriangles();
    testTrimeshUnderRotation();
    testTrimeshWindingErrorsDetectsFlippedTriangles();
    testSweepSphereRejectsHitsBeyondThePath();
    testNarrowphaseMarginReportsSpeculativeContacts();
    testShapeInertiaTensors();
    testBoxFaceHelpers();
    testTriangleFeatureIsInternal();
    testConvexTrimeshBoxRestsOnFloorAndInCorner();
    testCharacterRigidBodyOnGround();
    testCharacterRigidBodyOnSteepGround();
    testCharacterRigidBodyInAir();
    testCharacterRigidBodyPushesALightDynamicBox();
    testCharacterRigidBodyMovesAfterFallingAsleep();
    testDynamicBoxDroppedFromHeightRestsOnTrimesh();
    testSupportAndBounds();
    testBroadphasePairs();
    testBroadphaseSkipsStaticPairs();
    testBroadphaseLayers();
    testBroadphaseFindsEveryOverlapInAStack();
    testSphereSphere();
    testSphereBox();
    testBoxBoxFaceContact();
    testBoxBoxEdgeContact();
    testBoxBoxNormalAlwaysSeparates();
    testSegmentHelpers();
    testCapsuleShape();
    testCapsuleSphereAndCapsule();
    testCapsuleBox();
    testCapsuleRestsOnGround();
    testSphereRollsFromFriction();
    testSolverStopsAFall();
    testSolverRestitution();
    testSolverMomentumBetweenDynamics();
    testSolverFrictionStopsSliding();
    testStaticPairDoesNothing();
    testWarmStartingCarriesImpulse();
    testRigidBodyForcesAndImpulses();
    testRigidBodyOffCenterImpulseSpins();
    testRigidBodySleepsAndImpulseWakes();
    testRigidBodyStaticAndKinematic();
    testSolverOffCenterContactSpinsABox();
    testSolverSurvivesDeepPenetration();
    testBoxSettlesOnGround();
    testWorldStackStandsUp();
    testStackSleepsTogether();
    testWorldEventsEnterStayExit();
    testWorldFixedStepIsFrameRateIndependent();
    testWorldRaycastFindsTheNearestBody();
    testWorldRaycastMaskExcludesLayer();
    testWorldOverlapSphereRespectsMask();
    testWorldStepSkipsIncompatibleMasks();
    testBodyHandleRejectsReusedSlot();
    testWorldRemovalKeepsDenseStorageAndStableIds();
    testWorldAllowsMutationFromCollisionCallback();
    testWorldAreaForces();
    testConvexHullShapeMatchesBox();
    testConvexHullInertiaMatchesBoxClosedForm();
    testConvexHullBoxMatchesBoxBoxInvariants();
    testConvexHullSphereBasicContact();
    testConvexHullCapsuleBasicContact();
    testConvexHullConvexHullOverlapAndSeparation();
    testConvexHullDegenerateShardIsFinite();
    testConvexHullConvexHullCoincidentFacesDoNotCrash();
    if (gFailures)
        std::fprintf(stderr, "%d collision test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
