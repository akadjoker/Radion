#include "PCH.h"

#include "Scene.h"
#include "character/CharacterBody.h"
#include "character/CharacterRigidBody.h"
#include "collision/Broadphase.h"
#include "collision/Narrowphase.h"
#include "dynamics/ContactSolver.h"
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

bool near(const Math::vec3& a, const Math::vec3& b, f32 epsilon = 1e-4f)
{
    return Math::length(a - b) <= epsilon;
}

Math::mat4 at(const Math::vec3& position, const Math::quat& rotation = Math::quat(1, 0, 0, 0))
{
    Math::mat4 transform = Math::mat4_cast(rotation);
    transform[3] = Math::vec4(position, 1.0f);
    return transform;
}

AABB boxAt(const Math::vec3& center, f32 half)
{
    AABB bounds;
    bounds.min = center - Math::vec3(half);
    bounds.max = center + Math::vec3(half);
    return bounds;
}

// Cube hull built through the real ConvexHullComputer, as VoronoiShatter does.
Shard buildCubeShard(f32 halfExtent)
{
    std::vector<Math::vec3> corners;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2)
                corners.push_back(Math::vec3(sx, sy, sz) * halfExtent);

    ConvexHullComputer computer;
    computer.compute(&corners[0].x, sizeof(Math::vec3), static_cast<int>(corners.size()), 0.0f,
                     0.0f);

    Shard shard;
    shard.vertices = computer.vertices;
    shard.edges = computer.edges;
    shard.faces = computer.faces;
    return shard;
}

void testSupportAndBounds()
{
    const BoxShape box(Math::vec3(1.0f, 2.0f, 3.0f));
    const Math::mat4 identity = at(Math::vec3(0.0f));

    CHECK(near(box.support(identity, Math::vec3(1, 1, 1)), Math::vec3(1, 2, 3)));
    CHECK(near(box.support(identity, Math::vec3(-1, 1, -1)), Math::vec3(-1, 2, -3)));

    f32 minimum = 0.0f;
    f32 maximum = 0.0f;
    box.project(identity, Math::vec3(0, 1, 0), minimum, maximum);
    CHECK(near(minimum, -2.0f));
    CHECK(near(maximum, 2.0f));

    const Math::mat4 turned =
        at(Math::vec3(0.0f), Math::angleAxis(Math::half_pi<f32>(), Math::vec3(0, 0, 1)));
    box.project(turned, Math::vec3(0, 1, 0), minimum, maximum);
    CHECK(near(maximum, 1.0f, 1e-3f));

    // A 45 degree turn puts a corner furthest out, at half*sqrt(2) per axis.
    const BoxShape cube(Math::vec3(1.0f));
    const Math::mat4 diagonal =
        at(Math::vec3(0.0f), Math::angleAxis(Math::quarter_pi<f32>(), Math::vec3(0, 0, 1)));
    const AABB bounds = cube.bounds(diagonal);
    CHECK(near(bounds.max.x, std::sqrt(2.0f), 1e-3f));
    CHECK(near(bounds.max.z, 1.0f, 1e-3f));

    const SphereShape sphere(2.5f);
    CHECK(near(sphere.support(at(Math::vec3(1, 0, 0)), Math::vec3(0, 3, 0)), Math::vec3(1, 2.5f, 0)));
    CHECK(near(sphere.bounds(diagonal).max, Math::vec3(2.5f)));
}

void testBroadphasePairs()
{
    Broadphase broadphase;
    BroadphaseProxy proxy;

    proxy.id = 1;
    proxy.bounds = boxAt(Math::vec3(0.0f), 1.0f);
    broadphase.add(proxy);
    proxy.id = 2;
    proxy.bounds = boxAt(Math::vec3(1.5f, 0.0f, 0.0f), 1.0f);
    broadphase.add(proxy);
    proxy.id = 3;
    proxy.bounds = boxAt(Math::vec3(50.0f, 0.0f, 0.0f), 1.0f);
    broadphase.add(proxy);

    std::vector<BroadphasePair> pairs;
    broadphase.findPairs(pairs);
    CHECK(pairs.size() == 1);
    if (pairs.size() == 1)
    {
        CHECK(pairs[0].a == 1);
        CHECK(pairs[0].b == 2);
    }

    CHECK(broadphase.sweepAxis() == 0);
}

void testBroadphaseSkipsStaticPairs()
{
    Broadphase broadphase;
    BroadphaseProxy proxy;
    proxy.movable = false;

    proxy.id = 1;
    proxy.bounds = boxAt(Math::vec3(0.0f), 1.0f);
    broadphase.add(proxy);
    proxy.id = 2;
    proxy.bounds = boxAt(Math::vec3(0.5f, 0.0f, 0.0f), 1.0f);
    broadphase.add(proxy);

    std::vector<BroadphasePair> pairs;
    broadphase.findPairs(pairs);
    CHECK(pairs.empty());

    Broadphase mixed;
    proxy.id = 1;
    proxy.movable = false;
    proxy.bounds = boxAt(Math::vec3(0.0f), 1.0f);
    mixed.add(proxy);
    proxy.id = 2;
    proxy.movable = true;
    proxy.bounds = boxAt(Math::vec3(0.5f, 0.0f, 0.0f), 1.0f);
    mixed.add(proxy);
    mixed.findPairs(pairs);
    CHECK(pairs.size() == 1);
}

void testBroadphaseLayers()
{
    Broadphase broadphase;
    BroadphaseProxy proxy;
    proxy.bounds = boxAt(Math::vec3(0.0f), 1.0f);

    proxy.id = 1;
    proxy.filter.group = 1;
    proxy.filter.mask = 2;
    broadphase.add(proxy);
    proxy.id = 2;
    proxy.filter.group = 4;
    proxy.filter.mask = 0xFFFFFFFFu;
    broadphase.add(proxy);

    std::vector<BroadphasePair> pairs;
    broadphase.findPairs(pairs);
    CHECK(pairs.empty());
}

void testBroadphaseFindsEveryOverlapInAStack()
{
    // The sweep must pick y and find all nine pairs; a wrong axis or an early break silently drops contacts.
    Broadphase broadphase;
    BroadphaseProxy proxy;
    for (u32 i = 0; i < 10; ++i)
    {
        proxy.id = i;
        proxy.bounds = boxAt(Math::vec3(0.0f, static_cast<f32>(i) * 0.9f, 0.0f), 0.5f);
        broadphase.add(proxy);
    }
    std::vector<BroadphasePair> pairs;
    broadphase.findPairs(pairs);
    CHECK(broadphase.sweepAxis() == 1);
    CHECK(pairs.size() == 9);
    for (const BroadphasePair& pair : pairs)
        CHECK(pair.b == pair.a + 1);
}

void testSphereSphere()
{
    const SphereShape a(1.0f);
    const SphereShape b(1.0f);
    ContactManifold manifold;

    CHECK(!Narrowphase::collide(a, at(Math::vec3(0.0f)), b, at(Math::vec3(3.0f, 0, 0)), manifold));

    CHECK(Narrowphase::collide(a, at(Math::vec3(0.0f)), b, at(Math::vec3(1.5f, 0, 0)), manifold));
    CHECK(manifold.count == 1);
    CHECK(near(manifold.normal, Math::vec3(1, 0, 0)));
    CHECK(near(manifold.points[0].penetration, 0.5f));
    // A's surface is at x=1, B's at x=0.5, so the midpoint is 0.75.
    CHECK(near(manifold.points[0].position.x, 0.75f));

    CHECK(!Narrowphase::collide(a, at(Math::vec3(0.0f)), b, at(Math::vec3(2.0f, 0, 0)), manifold));

    // Concentric: no division by zero, and still report a push-apart direction.
    CHECK(Narrowphase::collide(a, at(Math::vec3(0.0f)), b, at(Math::vec3(0.0f)), manifold));
    CHECK(near(Math::length(manifold.normal), 1.0f));
    CHECK(std::isfinite(manifold.points[0].penetration));
}

void testSphereBox()
{
    const SphereShape sphere(1.0f);
    const BoxShape box(Math::vec3(1.0f));
    ContactManifold manifold;

    CHECK(Narrowphase::collide(sphere, at(Math::vec3(0.0f, 1.75f, 0.0f)), box, at(Math::vec3(0.0f)),
                               manifold));
    CHECK(manifold.count == 1);
    CHECK(near(manifold.normal, Math::vec3(0, -1, 0)));
    CHECK(near(manifold.points[0].penetration, 0.25f));
    CHECK(near(manifold.points[0].position, Math::vec3(0, 1, 0)));

    CHECK(!Narrowphase::collide(sphere, at(Math::vec3(1.8f, 1.8f, 1.8f)), box, at(Math::vec3(0.0f)),
                                manifold));

    CHECK(Narrowphase::collide(sphere, at(Math::vec3(0.0f, 0.8f, 0.0f)), box, at(Math::vec3(0.0f)),
                               manifold));
    CHECK(near(manifold.normal, Math::vec3(0, -1, 0)));
    CHECK(manifold.points[0].penetration > 1.0f);

    ContactManifold flipped;
    CHECK(Narrowphase::collide(box, at(Math::vec3(0.0f)), sphere, at(Math::vec3(0.0f, 1.75f, 0.0f)),
                               flipped));
    CHECK(near(flipped.normal, Math::vec3(0, 1, 0)));
    CHECK(near(flipped.points[0].penetration, 0.25f));
}

void testBoxBoxFaceContact()
{
    const BoxShape a(Math::vec3(1.0f));
    const BoxShape b(Math::vec3(1.0f));
    ContactManifold manifold;

    CHECK(!Narrowphase::collide(a, at(Math::vec3(0.0f)), b, at(Math::vec3(2.5f, 0, 0)), manifold));

    // A face against a face is four points; one point tips a box over.
    CHECK(
        Narrowphase::collide(a, at(Math::vec3(0.0f)), b, at(Math::vec3(0.0f, 1.8f, 0.0f)), manifold));
    CHECK(manifold.count == 4);
    CHECK(near(manifold.normal, Math::vec3(0, 1, 0)));
    for (u32 i = 0; i < manifold.count; ++i)
    {
        CHECK(near(manifold.points[i].penetration, 0.2f, 1e-3f));
        CHECK(near(manifold.points[i].position.y, 1.0f, 1e-3f));
    }

    f32 spread = 0.0f;
    for (u32 i = 0; i < manifold.count; ++i)
        for (u32 j = i + 1; j < manifold.count; ++j)
            spread = Math::max(
                spread, Math::length(manifold.points[i].position - manifold.points[j].position));
    CHECK(spread > 1.5f);

    // Tangents must be a proper frame around the normal, or friction partly pushes along it.
    CHECK(near(Math::dot(manifold.tangent[0], manifold.normal), 0.0f));
    CHECK(near(Math::dot(manifold.tangent[1], manifold.normal), 0.0f));
    CHECK(near(Math::dot(manifold.tangent[0], manifold.tangent[1]), 0.0f));
    CHECK(near(Math::length(manifold.tangent[0]), 1.0f));
}

void testBoxBoxEdgeContact()
{
    // Edge-edge contact: the separating direction is a cross product of edges, on none of the six face normals; a face-only SAT slides the boxes sideways.
    const BoxShape a(Math::vec3(0.5f));
    const BoxShape b(Math::vec3(0.5f));
    const Math::mat4 transformA =
        at(Math::vec3(0.0f), Math::angleAxis(Math::quarter_pi<f32>(), Math::vec3(0, 0, 1)));
    const Math::mat4 transformB = at(Math::vec3(0.0f, 1.30f, 0.0f),
                                    Math::angleAxis(Math::quarter_pi<f32>(), Math::vec3(1, 0, 0)));

    ContactManifold manifold;
    CHECK(Narrowphase::collide(a, transformA, b, transformB, manifold));
    CHECK(manifold.count >= 1);
    CHECK(std::abs(manifold.normal.y) > 0.5f);
    CHECK(manifold.points[0].penetration > 0.0f);
    CHECK(near(Math::length(manifold.normal), 1.0f, 1e-3f));

    const Math::mat4 clear =
        at(Math::vec3(0.0f, 2.5f, 0.0f), Math::angleAxis(Math::quarter_pi<f32>(), Math::vec3(1, 0, 0)));
    CHECK(!Narrowphase::collide(a, transformA, b, clear, manifold));
}

void testBoxBoxNormalAlwaysSeparates()
{
    // Moving B along the normal by the penetration must end the overlap; a wrong-sign normal passes every other check.
    const BoxShape a(Math::vec3(0.5f));
    const BoxShape b(Math::vec3(0.7f, 0.4f, 0.6f));
    u32 tested = 0;
    for (u32 i = 0; i < 24; ++i)
    {
        const f32 angle = static_cast<f32>(i) * 0.26f;
        const Math::vec3 axis =
            Math::normalize(Math::vec3(std::sin(angle * 1.3f), 1.0f, std::cos(angle * 0.7f)));
        const Math::mat4 transformA = at(Math::vec3(0.0f), Math::angleAxis(angle, axis));
        const Math::vec3 offset(std::cos(angle) * 0.6f, std::sin(angle * 2.0f) * 0.5f, 0.35f);
        const Math::mat4 transformB = at(offset, Math::angleAxis(angle * 0.5f, Math::vec3(1, 0, 0)));

        ContactManifold manifold;
        if (!Narrowphase::collide(a, transformA, b, transformB, manifold))
            continue;
        ++tested;
        // The separating distance is the DEEPEST point's (the same as the first after reducePoints).
        f32 deepest = 0.0f;
        for (u32 p = 0; p < manifold.count; ++p)
            deepest = Math::max(deepest, manifold.points[p].penetration);
        CHECK(near(deepest, manifold.points[0].penetration, 1e-4f));

        const Math::mat4 pushed = at(offset + manifold.normal * (deepest + 0.02f),
                                    Math::angleAxis(angle * 0.5f, Math::vec3(1, 0, 0)));
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
    // The sweep must actually produce overlaps.
    CHECK(tested > 8);
}

void testSegmentHelpers()
{
    const Math::vec3 a(0.0f, 0.0f, 0.0f);
    const Math::vec3 b(0.0f, 4.0f, 0.0f);
    CHECK(near(closestPointOnSegment(a, b, Math::vec3(3.0f, 2.0f, 0.0f)), Math::vec3(0, 2, 0)));
    CHECK(near(closestPointOnSegment(a, b, Math::vec3(0.0f, 9.0f, 0.0f)), b));
    CHECK(near(closestPointOnSegment(a, b, Math::vec3(0.0f, -9.0f, 0.0f)), a));
    CHECK(near(closestPointOnSegment(a, a, Math::vec3(5.0f, 5.0f, 5.0f)), a));

    Math::vec3 c1, c2;
    closestPointsBetweenSegments(Math::vec3(-1, 0, 0), Math::vec3(1, 0, 0), Math::vec3(0, 1, -1),
                                 Math::vec3(0, 1, 1), c1, c2);
    CHECK(near(c1, Math::vec3(0, 0, 0)));
    CHECK(near(c2, Math::vec3(0, 1, 0)));

    // Parallel segments: pick one answer, with no division by a zero determinant.
    closestPointsBetweenSegments(Math::vec3(0, 0, 0), Math::vec3(2, 0, 0), Math::vec3(0, 1, 0),
                                 Math::vec3(2, 1, 0), c1, c2);
    CHECK(near(Math::length(c2 - c1), 1.0f));
    CHECK(std::isfinite(c1.x));

    closestPointsBetweenSegments(Math::vec3(0, 0, 0), Math::vec3(1, 0, 0), Math::vec3(5, 0, 0),
                                 Math::vec3(6, 0, 0), c1, c2);
    CHECK(near(c1, Math::vec3(1, 0, 0)));
    CHECK(near(c2, Math::vec3(5, 0, 0)));
}

void testCapsuleShape()
{
    const CapsuleShape capsule(0.5f, 1.0f); // segment 2 long, total height 3
    const Math::mat4 identity = at(Math::vec3(0.0f));

    Math::vec3 lower, upper;
    capsule.segment(identity, lower, upper);
    CHECK(near(lower, Math::vec3(0, -1, 0)));
    CHECK(near(upper, Math::vec3(0, 1, 0)));

    CHECK(near(capsule.support(identity, Math::vec3(0, 1, 0)), Math::vec3(0, 1.5f, 0)));
    CHECK(near(capsule.support(identity, Math::vec3(1, 0, 0)).x, 0.5f));

    const AABB bounds = capsule.bounds(identity);
    CHECK(near(bounds.max, Math::vec3(0.5f, 1.5f, 0.5f)));

    const Math::mat4 lying =
        at(Math::vec3(0.0f), Math::angleAxis(Math::half_pi<f32>(), Math::vec3(0, 0, 1)));
    const AABB sideways = capsule.bounds(lying);
    CHECK(near(sideways.max.x, 1.5f, 1e-3f));
    CHECK(near(sideways.max.y, 0.5f, 1e-3f));
}

void testCapsuleSphereAndCapsule()
{
    const CapsuleShape capsule(0.5f, 1.0f);
    const SphereShape sphere(0.5f);
    ContactManifold manifold;

    CHECK(Narrowphase::collide(capsule, at(Math::vec3(0.0f)), sphere, at(Math::vec3(0.8f, 0, 0)),
                               manifold));
    CHECK(near(manifold.normal, Math::vec3(1, 0, 0)));
    CHECK(near(manifold.points[0].penetration, 0.2f));

    CHECK(Narrowphase::collide(capsule, at(Math::vec3(0.0f)), sphere, at(Math::vec3(0, 1.8f, 0)),
                               manifold));
    CHECK(near(manifold.normal, Math::vec3(0, 1, 0)));
    CHECK(near(manifold.points[0].penetration, 0.2f));

    CHECK(!Narrowphase::collide(capsule, at(Math::vec3(0.0f)), sphere, at(Math::vec3(1.2f, 0, 0)),
                                manifold));

    const CapsuleShape other(0.5f, 1.0f);
    CHECK(Narrowphase::collide(capsule, at(Math::vec3(0.0f)), other, at(Math::vec3(0.8f, 0, 0)),
                               manifold));
    CHECK(near(std::abs(manifold.normal.x), 1.0f, 1e-3f));
    CHECK(near(manifold.points[0].penetration, 0.2f));
    CHECK(std::isfinite(manifold.points[0].position.x));

    const Math::mat4 crossed =
        at(Math::vec3(0.0f, 0.8f, 0.0f), Math::angleAxis(Math::half_pi<f32>(), Math::vec3(0, 0, 1)));
    CHECK(Narrowphase::collide(capsule, at(Math::vec3(0.0f)), other, crossed, manifold));
    CHECK(manifold.points[0].penetration > 0.0f);

    ContactManifold flipped;
    CHECK(Narrowphase::collide(sphere, at(Math::vec3(0.8f, 0, 0)), capsule, at(Math::vec3(0.0f)),
                               flipped));
    CHECK(near(flipped.normal, Math::vec3(-1, 0, 0)));
    CHECK(near(flipped.points[0].penetration, 0.2f));
}

void testCapsuleBox()
{
    const CapsuleShape capsule(0.5f, 1.0f);
    const BoxShape box(Math::vec3(4.0f, 0.5f, 4.0f));
    ContactManifold manifold;

    CHECK(Narrowphase::collide(capsule, at(Math::vec3(0.0f, 1.9f, 0.0f)), box, at(Math::vec3(0.0f)),
                               manifold));
    CHECK(manifold.count == 1);
    CHECK(near(manifold.normal, Math::vec3(0, -1, 0), 1e-3f));
    CHECK(near(manifold.points[0].penetration, 0.1f, 1e-3f));

    // Lying flat MUST give two points; with one the capsule pivots and rolls off.
    const Math::mat4 lying =
        at(Math::vec3(0.0f, 0.95f, 0.0f), Math::angleAxis(Math::half_pi<f32>(), Math::vec3(0, 0, 1)));
    CHECK(Narrowphase::collide(capsule, lying, box, at(Math::vec3(0.0f)), manifold));
    CHECK(manifold.count == 2);
    CHECK(near(std::abs(manifold.normal.y), 1.0f, 1e-3f));
    if (manifold.count == 2)
        CHECK(near(Math::length(manifold.points[0].position - manifold.points[1].position), 2.0f,
                   1e-2f));

    CHECK(!Narrowphase::collide(capsule, at(Math::vec3(0.0f, 3.0f, 0.0f)), box, at(Math::vec3(0.0f)),
                                manifold));

    ContactManifold flipped;
    CHECK(Narrowphase::collide(box, at(Math::vec3(0.0f)), capsule, at(Math::vec3(0.0f, 1.9f, 0.0f)),
                               flipped));
    CHECK(near(flipped.normal, Math::vec3(0, 1, 0), 1e-3f));
}

void testCapsuleRestsOnGround()
{
    BoxShape groundShape(Math::vec3(20.0f, 0.5f, 20.0f));
    CapsuleShape capsuleShape(0.5f, 1.0f);

    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::vec3(0.0f, -0.5f, 0.0f));

    RigidBody capsule;
    capsule.setMass(2.0f);
    capsule.setInertiaTensor(capsuleShape.inertia(2.0f));
    capsule.setPosition(Math::vec3(0.0f, 2.0f, 0.0f));
    capsule.setOrientation(Math::angleAxis(Math::half_pi<f32>(), Math::vec3(0, 0, 1)));
    capsule.setDamping(0.999f, 0.999f);

    ground.setShape(&groundShape);
    ground.setFriction(0.8f);
    capsule.setShape(&capsuleShape);
    capsule.setFriction(0.8f);

    Radion::Scene world;
    world.setGravity(Math::vec3(0.0f, -9.81f, 0.0f));
    world.addBody(ground);
    world.addBody(capsule);

    for (u32 i = 0; i < 600; ++i)
        world.stepPhysics(1.0f / 120.0f);

    if (std::abs(capsule.position().y - 0.5f) >= 0.06f)
        std::fprintf(stderr, "    capsule rest: y %.4f axis %.3f %.3f %.3f |v| %.4f\n",
                     capsule.position().y, capsule.directionToWorld(Math::vec3(0, 1, 0)).x,
                     capsule.directionToWorld(Math::vec3(0, 1, 0)).y,
                     capsule.directionToWorld(Math::vec3(0, 1, 0)).z,
                     Math::length(capsule.velocity()));
    CHECK(std::abs(capsule.position().y - 0.5f) < 0.06f);
    const Math::vec3 axis = capsule.directionToWorld(Math::vec3(0.0f, 1.0f, 0.0f));
    CHECK(std::abs(axis.y) < 0.25f);
    CHECK(Math::length(capsule.velocity()) < 0.3f);
}

void testSphereRollsFromFriction()
{
    // Friction acts a radius below the centre, so it torques; without it the ball slides forever.
    BoxShape groundShape(Math::vec3(50.0f, 0.5f, 50.0f));
    SphereShape sphereShape(0.5f);

    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::vec3(0.0f, -0.5f, 0.0f));

    RigidBody ball;
    ball.setMass(1.0f);
    ball.setInertiaTensor(sphereShape.inertia(1.0f));
    ball.setPosition(Math::vec3(0.0f, 0.5f, 0.0f));
    ball.setVelocity(Math::vec3(6.0f, 0.0f, 0.0f));
    ball.setDamping(1.0f, 1.0f);
    ball.setCanSleep(false);

    ground.setShape(&groundShape);
    ground.setFriction(0.6f);
    ball.setShape(&sphereShape);
    ball.setFriction(0.6f);

    Radion::Scene world;
    world.setGravity(Math::vec3(0.0f, -9.81f, 0.0f));
    world.addBody(ground);
    world.addBody(ball);

    for (u32 i = 0; i < 240; ++i)
        world.stepPhysics(1.0f / 120.0f);

    CHECK(ball.angularVelocity().z < -1.0f);
    // Rolling without slipping: v = -w x r, so w_z = -v_x / radius; the wrong sign spins against travel.
    const f32 rolling = -ball.velocity().x / sphereShape.radius();
    CHECK(ball.angularVelocity().z < 0.0f && rolling < 0.0f);
    CHECK(std::abs(ball.angularVelocity().z - rolling) < std::abs(rolling) * 0.5f);
    CHECK(ball.velocity().x > 0.0f);
    CHECK(ball.velocity().x < 6.0f);
}

RigidBody makeDynamic(const CollisionShape& shape, f32 mass, const Math::vec3& position)
{
    RigidBody body;
    body.setMass(mass);
    body.setInertiaTensor(shape.inertia(mass));
    body.setPosition(position);
    body.setDamping(1.0f, 1.0f);
    body.setCanSleep(false);
    return body;
}

RigidBody makeStatic(const Math::vec3& position)
{
    RigidBody body;
    body.setBodyType(BodyType::Static);
    body.setPosition(position);
    return body;
}

void testSolverStopsAFall()
{
    const BoxShape shape(Math::vec3(0.5f));
    RigidBody ground = makeStatic(Math::vec3(0.0f, -0.5f, 0.0f));
    RigidBody box = makeDynamic(shape, 1.0f, Math::vec3(0.0f, 0.45f, 0.0f));
    box.setVelocity(Math::vec3(0.0f, -5.0f, 0.0f));

    Contact contact;
    contact.a = &ground;
    contact.b = &box;
    contact.friction = 0.5f;
    contact.restitution = 0.0f;
    contact.manifold.normal = Math::vec3(0, 1, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = Math::vec3(0.0f, -0.05f, 0.0f);
    contact.manifold.points[0].penetration = 0.05f;

    ContactSolver solver;
    solver.solve(&contact, 1, 1.0f / 60.0f);

    CHECK(box.velocity().y > -0.01f);
    CHECK(box.velocity().y < 0.5f);
    CHECK(near(ground.velocity(), Math::vec3(0.0f)));
    CHECK(near(ground.position(), Math::vec3(0.0f, -0.5f, 0.0f)));
    CHECK(box.position().y > 0.45f);
}

void testSolverRestitution()
{
    const SphereShape shape(0.5f);
    RigidBody ground = makeStatic(Math::vec3(0.0f, -0.5f, 0.0f));
    RigidBody ball = makeDynamic(shape, 1.0f, Math::vec3(0.0f, 0.5f, 0.0f));
    ball.setVelocity(Math::vec3(0.0f, -10.0f, 0.0f));

    Contact contact;
    contact.a = &ground;
    contact.b = &ball;
    contact.friction = 0.0f;
    contact.restitution = 0.5f;
    contact.manifold.normal = Math::vec3(0, 1, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = Math::vec3(0.0f, 0.0f, 0.0f);
    contact.manifold.points[0].penetration = 0.01f;

    ContactSolver solver;
    solver.solve(&contact, 1, 1.0f / 60.0f);

    CHECK(ball.velocity().y > 3.0f);
    CHECK(ball.velocity().y < 7.0f);
}

void testSolverMomentumBetweenDynamics()
{
    const SphereShape shape(0.5f);
    RigidBody left = makeDynamic(shape, 2.0f, Math::vec3(-0.49f, 0.0f, 0.0f));
    RigidBody right = makeDynamic(shape, 2.0f, Math::vec3(0.49f, 0.0f, 0.0f));
    left.setVelocity(Math::vec3(4.0f, 0.0f, 0.0f));
    right.setVelocity(Math::vec3(-4.0f, 0.0f, 0.0f));

    const Math::vec3 before = left.velocity() * 2.0f + right.velocity() * 2.0f;

    Contact contact;
    contact.a = &left;
    contact.b = &right;
    contact.friction = 0.0f;
    contact.restitution = 0.0f;
    contact.manifold.normal = Math::vec3(1, 0, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = Math::vec3(0.0f);
    contact.manifold.points[0].penetration = 0.02f;

    ContactSolver solver;
    solver.solve(&contact, 1, 1.0f / 60.0f);

    const Math::vec3 after = left.velocity() * 2.0f + right.velocity() * 2.0f;
    CHECK(near(before, after, 1e-2f));
    CHECK(near(left.velocity().x, right.velocity().x, 1e-2f));
}

void testSolverFrictionStopsSliding()
{
    const BoxShape shape(Math::vec3(0.5f));
    RigidBody ground = makeStatic(Math::vec3(0.0f, -0.5f, 0.0f));
    RigidBody box = makeDynamic(shape, 1.0f, Math::vec3(0.0f, 0.5f, 0.0f));
    box.setVelocity(Math::vec3(3.0f, -1.0f, 0.0f));

    Contact contact;
    contact.a = &ground;
    contact.b = &box;
    contact.friction = 1.0f;
    contact.restitution = 0.0f;
    contact.manifold.normal = Math::vec3(0, 1, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = Math::vec3(0.0f);
    contact.manifold.points[0].penetration = 0.01f;

    ContactSolver solver;
    const f32 before = box.velocity().x;
    for (u32 i = 0; i < 20; ++i)
    {
        contact.manifold.points[0].penetration = 0.01f;
        solver.solve(&contact, 1, 1.0f / 60.0f);
    }
    // Friction removes the sideways speed and never reverses it (an overshoot drives the box backwards).
    CHECK(box.velocity().x < before);
    CHECK(box.velocity().x >= -0.05f);

    RigidBody slippery = makeDynamic(shape, 1.0f, Math::vec3(0.0f, 0.5f, 0.0f));
    slippery.setVelocity(Math::vec3(3.0f, -1.0f, 0.0f));
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
    RigidBody groundA = makeStatic(Math::vec3(0.0f));
    RigidBody groundB = makeStatic(Math::vec3(0.1f, 0.0f, 0.0f));

    Contact contact;
    contact.a = &groundA;
    contact.b = &groundB;
    contact.manifold.normal = Math::vec3(1, 0, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = Math::vec3(0.05f, 0.0f, 0.0f);
    contact.manifold.points[0].penetration = 0.5f;

    ContactSolver solver;
    // Two infinite masses: dividing by their total is a division by zero.
    solver.solve(&contact, 1, 1.0f / 60.0f);
    CHECK(near(groundA.position(), Math::vec3(0.0f)));
    CHECK(near(groundB.position(), Math::vec3(0.1f, 0.0f, 0.0f)));
    CHECK(std::isfinite(groundA.position().x));
}

void testWarmStartingCarriesImpulse()
{
    const BoxShape shape(Math::vec3(0.5f));
    RigidBody ground = makeStatic(Math::vec3(0.0f, -0.5f, 0.0f));
    RigidBody box = makeDynamic(shape, 1.0f, Math::vec3(0.0f, 0.5f, 0.0f));

    Contact contact;
    contact.a = &ground;
    contact.b = &box;
    contact.friction = 0.5f;
    contact.manifold.normal = Math::vec3(0, 1, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = Math::vec3(0.0f);
    contact.manifold.points[0].penetration = 0.005f;

    ContactSolver solver;
    box.setVelocity(Math::vec3(0.0f, -2.0f, 0.0f));
    solver.solve(&contact, 1, 1.0f / 60.0f);
    // The needed impulse is kept on the point as the next step's warm start.
    CHECK(contact.manifold.points[0].normalImpulse > 0.0f);
}

void testRigidBodyForcesAndImpulses()
{
    const BoxShape shape(Math::vec3(0.5f));
    RigidBody body = makeDynamic(shape, 2.0f, Math::vec3(0.0f));

    body.addForce(Math::vec3(10.0f, 0.0f, 0.0f));
    body.integrate(1.0f / 60.0f);
    CHECK(near(body.velocity().x, 10.0f / 2.0f / 60.0f, 1e-4f));

    body.applyLinearImpulse(Math::vec3(4.0f, 0.0f, 0.0f));
    CHECK(near(body.velocity().x, 10.0f / 2.0f / 60.0f + 4.0f / 2.0f, 1e-4f));

    body.setAcceleration(Math::vec3(0.0f, -9.81f, 0.0f));
    const f32 beforeY = body.velocity().y;
    body.integrate(1.0f / 60.0f);
    CHECK(near(body.velocity().y, beforeY - 9.81f / 60.0f, 1e-4f));

    body.clearAccumulators();
    body.addForceAtBodyPoint(Math::vec3(0.0f, 0.0f, 8.0f), Math::vec3(0.5f, 0.0f, 0.0f));
    body.integrate(1.0f / 60.0f);
    CHECK(body.angularVelocity().y != 0.0f);
}

void testRigidBodyOffCenterImpulseSpins()
{
    const BoxShape shape(Math::vec3(0.5f));
    RigidBody body = makeDynamic(shape, 1.0f, Math::vec3(0.0f));

    // r x I points +z for r on +x and I up.
    body.applyImpulseAtPoint(Math::vec3(0.0f, 5.0f, 0.0f), Math::vec3(0.5f, 0.0f, 0.0f));

    CHECK(near(body.velocity(), Math::vec3(0.0f, 5.0f, 0.0f), 1e-4f));
    CHECK(body.angularVelocity().z > 0.0f);
    const Math::vec3 surfaceVel = body.velocityAtPoint(Math::vec3(0.5f, 0.0f, 0.0f));
    CHECK(std::isfinite(surfaceVel.x));
}

void testRigidBodySleepsAndImpulseWakes()
{
    const BoxShape shape(Math::vec3(0.5f));
    RigidBody body = makeDynamic(shape, 1.0f, Math::vec3(0.0f));
    body.setCanSleep(true);
    body.setVelocity(Math::vec3(0.001f, 0.0f, 0.0f));

    bool slept = false;
    for (u32 i = 0; i < 60 && !slept; ++i)
    {
        body.integrate(1.0f / 60.0f);
        if (!body.awake())
            slept = true;
    }
    CHECK(slept);

    body.applyLinearImpulse(Math::vec3(1.0f, 0.0f, 0.0f));
    CHECK(body.awake());
}

void testRigidBodyStaticAndKinematic()
{
    const BoxShape shape(Math::vec3(0.5f));

    RigidBody statik = makeStatic(Math::vec3(1.0f, 2.0f, 3.0f));
    statik.setVelocity(Math::vec3(5.0f, 0.0f, 0.0f));
    statik.addForce(Math::vec3(100.0f, 0.0f, 0.0f));
    statik.integrate(1.0f / 60.0f);
    CHECK(!statik.isDynamic());
    CHECK(near(statik.position(), Math::vec3(1.0f, 2.0f, 3.0f)));

    RigidBody kinematic;
    kinematic.setBodyType(BodyType::Kinematic);
    kinematic.setPosition(Math::vec3(0.0f));
    kinematic.setVelocity(Math::vec3(3.0f, 0.0f, 0.0f));
    kinematic.addForce(Math::vec3(1000.0f, 0.0f, 0.0f));
    kinematic.integrate(1.0f / 60.0f);
    CHECK(near(kinematic.position().x, 3.0f / 60.0f, 1e-4f));
    CHECK(near(kinematic.velocity(), Math::vec3(3.0f, 0.0f, 0.0f), 1e-4f));
}

void testSolverOffCenterContactSpinsABox()
{
    // An impulse through a point off the centre of mass must spin the box.
    const BoxShape shape(Math::vec3(0.5f));
    RigidBody ground = makeStatic(Math::vec3(0.0f, -0.5f, 0.0f));
    RigidBody box = makeDynamic(shape, 1.0f, Math::vec3(0.0f, 0.45f, 0.0f));
    box.setVelocity(Math::vec3(0.0f, -5.0f, 0.0f));
    box.setAngularVelocity(Math::vec3(0.0f));

    const Math::vec3 contactPoint(0.4f, -0.05f, 0.0f);
    Contact contact;
    contact.a = &ground;
    contact.b = &box;
    contact.friction = 0.0f;
    contact.restitution = 0.0f;
    contact.manifold.normal = Math::vec3(0, 1, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = contactPoint;
    contact.manifold.points[0].penetration = 0.05f;

    ContactSolver solver;
    solver.solve(&contact, 1, 1.0f / 60.0f);

    // The solver cancels velocity AT the contact point, not the centre: the centre keeps falling a little while it spins.
    CHECK(box.velocityAtPoint(contactPoint).y > -1.0f);
    CHECK(std::abs(box.angularVelocity().z) > 1e-3f);
}

void testSolverSurvivesDeepPenetration()
{
    // A whole box-width of overlap must be corrected without exploding.
    const BoxShape shape(Math::vec3(0.5f));
    RigidBody ground = makeStatic(Math::vec3(0.0f, -0.5f, 0.0f));
    RigidBody box = makeDynamic(shape, 1.0f, Math::vec3(0.0f, -0.2f, 0.0f));
    box.setVelocity(Math::vec3(0.0f, -10.0f, 0.0f));

    Contact contact;
    contact.a = &ground;
    contact.b = &box;
    contact.friction = 0.5f;
    contact.restitution = 0.0f;
    contact.manifold.normal = Math::vec3(0, 1, 0);
    contact.manifold.count = 1;
    contact.manifold.points[0].position = Math::vec3(0.0f, -0.05f, 0.0f);
    contact.manifold.points[0].penetration = 0.7f;

    ContactSolver solver;
    const f32 startY = box.position().y;
    for (u32 i = 0; i < 5; ++i)
        solver.solve(&contact, 1, 1.0f / 60.0f);

    CHECK(box.position().y > startY);
    CHECK(std::abs(box.velocity().y) < 20.0f);
    CHECK(std::isfinite(box.position().y));
}

void testBoxSettlesOnGround()
{
    const BoxShape groundShape(Math::vec3(10.0f, 0.5f, 10.0f));
    const BoxShape boxShape(Math::vec3(0.5f));

    RigidBody ground = makeStatic(Math::vec3(0.0f, -0.5f, 0.0f));
    RigidBody box = makeDynamic(boxShape, 1.0f, Math::vec3(0.0f, 3.0f, 0.0f));
    box.setAcceleration(Math::vec3(0.0f, -9.81f, 0.0f));
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

    CHECK(box.position().y > 0.45f);
    CHECK(box.position().y < 0.55f);
    CHECK(std::abs(box.velocity().y) < 0.5f);
    CHECK(std::abs(box.position().x) < 0.05f);
    CHECK(std::abs(box.position().z) < 0.05f);
    CHECK(std::isfinite(box.position().y));
}

void testWorldStackStandsUp()
{
    // Five stacked boxes dropped from a small gap: exposes a missed broadphase pair, too few manifold points, or no warm starting.
    BoxShape groundShape(Math::vec3(20.0f, 0.5f, 20.0f));
    BoxShape boxShape(Math::vec3(0.5f));

    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::vec3(0.0f, -0.5f, 0.0f));

    constexpr u32 kCount = 5;
    RigidBody boxes[kCount];
    for (u32 i = 0; i < kCount; ++i)
    {
        boxes[i].setMass(1.0f);
        boxes[i].setInertiaTensor(boxShape.inertia(1.0f));
        boxes[i].setPosition(Math::vec3(0.0f, 0.5f + static_cast<f32>(i) * 1.02f, 0.0f));
        boxes[i].setDamping(0.999f, 0.999f);
    }

    ground.setShape(&groundShape);
    ground.setFriction(0.8f);
    for (u32 i = 0; i < kCount; ++i)
        boxes[i].setShape(&boxShape);

    Radion::Scene world;
    world.setGravity(Math::vec3(0.0f, -9.81f, 0.0f));
    world.setFixedStep(1.0f / 120.0f);

    world.addBody(ground);
    for (u32 i = 0; i < kCount; ++i)
        world.addBody(boxes[i]);

    for (u32 i = 0; i < 900; ++i)
        world.stepPhysics(1.0f / 120.0f);

    const int before = gFailures;
    for (u32 i = 0; i < kCount; ++i)
    {
        const f32 expected = 0.5f + static_cast<f32>(i) * 1.0f;
        CHECK(std::abs(boxes[i].position().y - expected) < 0.12f);
        CHECK(std::abs(boxes[i].position().x) < 0.15f);
        CHECK(std::abs(boxes[i].position().z) < 0.15f);
        CHECK(std::isfinite(boxes[i].position().y));
    }

    for (u32 i = 0; i < kCount; ++i)
        CHECK(Math::length(boxes[i].velocity()) < 0.35f);

    if (gFailures != before)
        for (u32 i = 0; i < kCount; ++i)
            std::fprintf(stderr, "    box %u: y %.4f (want %.2f) x %.4f z %.4f |v| %.4f\n", i,
                         boxes[i].position().y, 0.5 + double(i), boxes[i].position().x,
                         boxes[i].position().z, Math::length(boxes[i].velocity()));
}

void testStackSleepsTogether()
{
    // Per-body sleep freezes a half-settled stack and boxes hang in the air; bodies joined by contacts must sleep as one.
    BoxShape groundShape(Math::vec3(20.0f, 0.5f, 20.0f));
    BoxShape boxShape(Math::vec3(0.5f));

    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::vec3(0.0f, -0.5f, 0.0f));

    constexpr u32 kCount = 6;
    RigidBody boxes[kCount];

    ground.setShape(&groundShape);
    ground.setFriction(0.7f);

    Radion::Scene world;
    world.setGravity(Math::vec3(0.0f, -9.81f, 0.0f));
    world.setFixedStep(1.0f / 120.0f);

    world.addBody(ground);
    for (u32 i = 0; i < kCount; ++i)
    {
        boxes[i].setMass(1.0f);
        boxes[i].setInertiaTensor(boxShape.inertia(1.0f));
        boxes[i].setPosition(Math::vec3(0.0f, 0.5f + static_cast<f32>(i) * 1.005f, 0.0f));
        boxes[i].setDamping(0.999f, 0.999f);
        boxes[i].setShape(&boxShape);
        world.addBody(boxes[i]);
    }

    const int before = gFailures;
    u32 worstSplitStep = 0;
    u32 worstAsleep = 0;
    for (u32 step = 0; step < 1200; ++step)
    {
        world.stepPhysics(1.0f / 120.0f);

        // Checked EVERY step: the transient state is what hangs a box.
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

    BoxShape shape(Math::vec3(0.5f));
    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::vec3(0.0f, -0.5f, 0.0f));

    RigidBody box;
    box.setMass(1.0f);
    box.setInertiaTensor(shape.inertia(1.0f));
    box.setPosition(Math::vec3(0.0f, 3.0f, 0.0f));
    box.setCanSleep(false);

    ground.setShape(&shape);
    box.setShape(&shape);

    Radion::Scene world;
    world.setGravity(Math::vec3(0.0f, -9.81f, 0.0f));
    world.addBody(ground);
    world.addBody(box);

    world.setContactEventCallback(
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
        world.stepPhysics(1.0f / 120.0f);
        if (recorder.enters + recorder.stays == before && box.position().y < 2.0f)
            longestGap = Math::max(longestGap, ++gap);
        else
            gap = 0;
    }

    // Landing is one Enter then many Stays; an Enter every step means the cache is not remembering the pair.
    if (recorder.enters != 1 || recorder.exits != 0)
        std::fprintf(stderr,
                     "  landing: %u enters, %u stays, %u exits, resting y %.4f, longest gap %u\n",
                     recorder.enters, recorder.stays, recorder.exits, box.position().y, longestGap);
    CHECK(recorder.enters == 1);
    CHECK(recorder.stays > 100);
    CHECK(recorder.exits == 0);

    // Exit once after as many steps as the persistence window; a single missed step on landing must not read as separation.
    box.setBodyType(BodyType::Kinematic);
    box.setPosition(Math::vec3(0.0f, 20.0f, 0.0f));
    for (u32 i = 0; i < world.contactPersistence(); ++i)
        world.stepPhysics(1.0f / 120.0f);
    CHECK(recorder.exits == 1);
    const u32 after = recorder.exits;
    for (u32 i = 0; i < 5; ++i)
        world.stepPhysics(1.0f / 120.0f);
    CHECK(recorder.exits == after);
}

void testWorldFixedStepIsFrameRateIndependent()
{
    BoxShape shape(Math::vec3(0.5f));

    auto run = [&shape](f32 frame, u32 frames)
    {
        RigidBody box;
        box.setMass(1.0f);
        box.setInertiaTensor(shape.inertia(1.0f));
        box.setPosition(Math::vec3(0.0f, 10.0f, 0.0f));
        box.setCanSleep(false);
        box.setShape(&shape);
        Radion::Scene world;
        world.setGravity(Math::vec3(0.0f, -9.81f, 0.0f));
        world.setFixedStep(1.0f / 120.0f);
        world.addBody(box);
        for (u32 i = 0; i < frames; ++i)
            world.updatePhysics(frame);
        return box.position().y;
    };

    const f32 fine = run(1.0f / 120.0f, 60);
    const f32 coarse = run(1.0f / 30.0f, 15);
    CHECK(near(fine, coarse, 1e-3f));
}

void makeGroundMesh(std::vector<Math::vec3>& vertices, std::vector<u32>& indices)
{
    vertices = {Math::vec3(-5.0f, 0.0f, -5.0f), Math::vec3(5.0f, 0.0f, -5.0f),
                Math::vec3(5.0f, 0.0f, 5.0f), Math::vec3(-5.0f, 0.0f, 5.0f)};
    indices = {0, 2, 1, 0, 3, 2};
}

void testClosestPointOnTriangle()
{
    const Math::vec3 a(0.0f, 0.0f, 0.0f);
    const Math::vec3 b(1.0f, 0.0f, 0.0f);
    const Math::vec3 c(0.0f, 0.0f, 1.0f);

    CHECK(near(closestPointOnTriangle(a, b, c, Math::vec3(0.25f, 3.0f, 0.25f)),
               Math::vec3(0.25f, 0.0f, 0.25f)));
    CHECK(near(closestPointOnTriangle(a, b, c, Math::vec3(-2.0f, 0.0f, -2.0f)), a));
    CHECK(near(closestPointOnTriangle(a, b, c, Math::vec3(5.0f, 0.0f, 0.0f)), b));
    CHECK(near(closestPointOnTriangle(a, b, c, Math::vec3(0.5f, 1.0f, -1.0f)),
               Math::vec3(0.5f, 0.0f, 0.0f)));
}

void testTrimeshTreeFindsOnlyNearbyTriangles()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));
    CHECK(mesh.triangleCount() == 2);

    std::vector<u32> hits;
    AABB far;
    far.min = Math::vec3(100.0f);
    far.max = Math::vec3(101.0f);
    mesh.query(far, hits);
    CHECK(hits.empty());

    AABB over;
    over.min = Math::vec3(-1.0f, -1.0f, -1.0f);
    over.max = Math::vec3(1.0f, 1.0f, 1.0f);
    mesh.query(over, hits);
    CHECK(!hits.empty());
}

void testSphereOnTriangle()
{
    const SphereShape sphere(1.0f);
    const TriangleShape triangle(Math::vec3(-5.0f, 0.0f, -5.0f), Math::vec3(5.0f, 0.0f, 5.0f),
                                 Math::vec3(5.0f, 0.0f, -5.0f));

    ContactManifold manifold;
    CHECK(!Narrowphase::sphereTriangle(sphere, at(Math::vec3(0.0f, 3.0f, 0.0f)), triangle,
                                       Math::mat4(1.0f), manifold));

    CHECK(Narrowphase::sphereTriangle(sphere, at(Math::vec3(1.0f, 0.75f, -1.0f)), triangle,
                                      Math::mat4(1.0f), manifold));
    CHECK(manifold.count == 1);
    CHECK(near(manifold.points[0].penetration, 0.25f));
    CHECK(near(manifold.normal, Math::vec3(0.0f, -1.0f, 0.0f)));
}

void testBoxOnTriangleGetsAPatch()
{
    const BoxShape box(Math::vec3(1.0f));
    const TriangleShape triangle(Math::vec3(-5.0f, 0.0f, -5.0f), Math::vec3(5.0f, 0.0f, 5.0f),
                                 Math::vec3(5.0f, 0.0f, -5.0f));

    ContactManifold manifold;
    CHECK(Narrowphase::boxTriangle(box, at(Math::vec3(1.0f, 0.9f, -1.0f)), triangle, Math::mat4(1.0f),
                                   manifold));
    CHECK(near(manifold.normal, Math::vec3(0.0f, -1.0f, 0.0f)));
    CHECK(manifold.count > 1);
    CHECK(near(manifold.points[0].penetration, 0.1f, 1e-3f));

    CHECK(!Narrowphase::boxTriangle(box, at(Math::vec3(1.0f, 3.0f, -1.0f)), triangle,
                                    Math::mat4(1.0f), manifold));
}

void testCapsuleOnTriangle()
{
    const CapsuleShape capsule(0.5f, 1.0f);
    const TriangleShape triangle(Math::vec3(-5.0f, 0.0f, -5.0f), Math::vec3(5.0f, 0.0f, 5.0f),
                                 Math::vec3(5.0f, 0.0f, -5.0f));

    ContactManifold manifold;
    CHECK(Narrowphase::capsuleTriangle(capsule, at(Math::vec3(1.0f, 1.25f, -1.0f)), triangle,
                                       Math::mat4(1.0f), manifold));
    CHECK(near(manifold.points[0].penetration, 0.25f));
    CHECK(near(manifold.normal, Math::vec3(0.0f, -1.0f, 0.0f)));
}

void testConvexTrimeshSpansBothTriangles()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));
    const BoxShape box(Math::vec3(1.0f));

    std::vector<ContactManifold> manifolds;
    CHECK(Narrowphase::convexTrimesh(box, at(Math::vec3(0.0f, 0.9f, 0.0f)), mesh, Math::mat4(1.0f),
                                     manifolds));
    CHECK(manifolds.size() == 2);
    for (const ContactManifold& manifold : manifolds)
        CHECK(near(manifold.normal, Math::vec3(0.0f, -1.0f, 0.0f)));

    manifolds.clear();
    CHECK(!Narrowphase::convexTrimesh(box, at(Math::vec3(0.0f, 5.0f, 0.0f)), mesh, Math::mat4(1.0f),
                                      manifolds));
    CHECK(manifolds.empty());
}

void testTrimeshUnderRotation()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));
    const SphereShape sphere(1.0f);

    // A mesh rolled 90 degrees about Z is a wall in YZ; the sphere must be found through the inverse transform.
    const Math::mat4 meshTransform =
        at(Math::vec3(0.0f), Math::angleAxis(Math::radians(90.0f), Math::vec3(0.0f, 0.0f, 1.0f)));
    std::vector<ContactManifold> manifolds;
    CHECK(Narrowphase::convexTrimesh(sphere, at(Math::vec3(0.75f, 0.0f, 0.0f)), mesh, meshTransform,
                                     manifolds));
    CHECK(!manifolds.empty());
    CHECK(near(std::abs(manifolds[0].normal.x), 1.0f, 1e-3f));
}

void makeRoomMesh(std::vector<Math::vec3>& vertices, std::vector<u32>& indices)
{
    vertices = {// floor
                Math::vec3(-10.0f, 0.0f, -10.0f), Math::vec3(10.0f, 0.0f, -10.0f),
                Math::vec3(10.0f, 0.0f, 10.0f), Math::vec3(-10.0f, 0.0f, 10.0f),
                // wall facing -X
                Math::vec3(2.0f, 0.0f, -10.0f), Math::vec3(2.0f, 0.0f, 10.0f),
                Math::vec3(2.0f, 6.0f, 10.0f), Math::vec3(2.0f, 6.0f, -10.0f)};
    // A sweep is one-sided, unlike a push-out: a back-facing wall is not there.
    indices = {0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7};
}

void makeRoomWithCeilingMesh(std::vector<Math::vec3>& vertices, std::vector<u32>& indices)
{
    vertices = {// floor (+Y)
                Math::vec3(-10.0f, 0.0f, -10.0f), Math::vec3(10.0f, 0.0f, -10.0f),
                Math::vec3(10.0f, 0.0f, 10.0f), Math::vec3(-10.0f, 0.0f, 10.0f),
                // wall facing -X
                Math::vec3(2.0f, 0.0f, -10.0f), Math::vec3(2.0f, 0.0f, 10.0f),
                Math::vec3(2.0f, 3.0f, 10.0f), Math::vec3(2.0f, 3.0f, -10.0f),
                // ceiling (-Y)
                Math::vec3(-10.0f, 3.0f, -10.0f), Math::vec3(10.0f, 3.0f, -10.0f),
                Math::vec3(10.0f, 3.0f, 10.0f), Math::vec3(-10.0f, 3.0f, 10.0f)};
    indices = {0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 8, 9, 10, 8, 10, 11};
}

void testCharacterReportsWallAndCeiling()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeRoomWithCeilingMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::vec3(-3.0f, 2.0f, 0.0f));
    for (u32 i = 0; i < 240; ++i)
        character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));
    CHECK(character.isOnFloor());

    character.setMoveInput(Math::vec3(4.0f, 0.0f, 0.0f));
    bool sawWall = false;
    for (u32 i = 0; i < 120 && !sawWall; ++i)
    {
        character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));
        if (character.isOnWall())
        {
            sawWall = true;
            CHECK(character.wallNormal().x < -0.5f);
        }
    }
    CHECK(sawWall);
    CHECK(character.position().x < 2.0f);

    character.setMoveInput(Math::vec3(0.0f));
    character.jump(8.0f);
    bool sawCeiling = false;
    for (u32 i = 0; i < 120 && !sawCeiling; ++i)
    {
        character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));
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
    // The caller owns the velocity (gravity included); landing must zero the up component so gravity does not accumulate.
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::vec3(-4.0f, 4.0f, 0.0f));

    Math::vec3 velocity(2.0f, -3.0f, 0.0f);
    for (u32 i = 0; i < 240; ++i)
    {
        velocity.y -= 20.0f * (1.0f / 60.0f);
        velocity.y = Math::max(velocity.y, -50.0f);
        character.setVelocity(velocity);
        character.moveAndSlide(1.0f / 60.0f, mesh, Math::mat4(1.0f));
        velocity = character.velocity();
    }
    CHECK(character.isOnFloor());
    CHECK(near(character.verticalSpeed(), 0.0f, 1e-3f));
    CHECK(character.position().x > -3.0f);
}

void testCharacterSetVerticalSpeedJumpsAnytime()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::vec3(-4.0f, 3.0f, 0.0f));
    character.setVerticalSpeed(6.0f);
    const f32 startY = character.position().y;
    bool rose = false;
    for (u32 i = 0; i < 30; ++i)
    {
        character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));
        if (character.position().y > startY + 0.2f)
            rose = true;
    }
    CHECK(rose);
}

void testCharacterApplyFloorSnapIsPublic()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::vec3(-2.0f, 1.1f, 0.0f));
    CHECK(character.applyFloorSnap(mesh, Math::mat4(1.0f)));
    CHECK(character.isOnFloor());
    CHECK(near(character.position().y, 1.02f, 0.02f));
}

void testSlideCameraPullsBackFromAWall()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    const f32 radius = 0.3f;
    const Math::vec3 cam =
        mesh.slideCamera(Math::vec3(0.0f, 2.0f, 0.0f), Math::vec3(8.0f, 2.0f, 0.0f), radius);

    CHECK(cam.x < 2.0f);
    CHECK(cam.x > 1.0f);
    CHECK(cam.x + radius < 2.0f + 1e-3f);
}

void testSlideCameraReturnsTheDesiredPositionWhenClear()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    const Math::vec3 desired(5.0f, 2.0f, 0.0f);
    const Math::vec3 cam = mesh.slideCamera(Math::vec3(0.0f, 2.0f, 0.0f), desired, 0.3f);
    CHECK(near(cam, desired, 1e-3f));
}

void testCharacterLandsAndStandsOnTheFloor()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::vec3(-2.0f, 4.0f, 0.0f));

    for (u32 i = 0; i < 120; ++i)
        character.move(Math::vec3(0.0f, -0.05f, 0.0f), mesh, Math::mat4(1.0f));

    CHECK(character.grounded());
    const f32 expected = 0.4f + 0.6f;
    CHECK(character.position().y > expected - 0.1f);
    CHECK(character.position().y < expected + 0.15f);
    CHECK(near(character.groundNormal().y, 1.0f, 1e-2f));
}

void testCharacterDoesNotWalkThroughAWall()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::vec3(-2.0f, 1.0f, 0.0f));

    for (u32 i = 0; i < 120; ++i)
    {
        character.move(Math::vec3(0.12f, 0.0f, 0.0f), mesh, Math::mat4(1.0f));
        character.move(Math::vec3(0.0f, -0.05f, 0.0f), mesh, Math::mat4(1.0f));
    }
    CHECK(character.position().x < 2.0f);
    CHECK(character.position().x > 1.0f);
}

void testCharacterDoesNotTeleportToAFarWall()
{
    // Regression: a swept sphere's plane hit sits far past the path (t = 80); the running best used to start at infinity, not 1.0, so one 0.1 step moved the character to the wall. One step must move 0.1.
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::vec3(-6.0f, 1.0f, 0.0f));

    const Math::vec3 before = character.position();
    character.move(Math::vec3(0.1f, 0.0f, 0.0f), mesh, Math::mat4(1.0f));
    CHECK(Math::length(character.position() - before) < 0.2f);
    CHECK(character.position().x < -5.5f);
}

void testCharacterCrossesSeamsWithoutStopping()
{
    // A floor cut into a grid: walking crosses many shared edges, which used to stop a body dead.
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    constexpr int kCells = 12;
    constexpr f32 kStep = 1.0f;
    for (int r = 0; r <= kCells; ++r)
        for (int c = 0; c <= kCells; ++c)
            vertices.push_back(Math::vec3(static_cast<f32>(c) * kStep - 6.0f, 0.0f,
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
    character.setPosition(Math::vec3(-5.0f, 1.05f, 0.3f));

    const f32 startX = character.position().x;
    for (u32 i = 0; i < 200; ++i)
    {
        character.move(Math::vec3(0.05f, 0.0f, 0.0f), mesh, Math::mat4(1.0f));
        character.move(Math::vec3(0.0f, -0.02f, 0.0f), mesh, Math::mat4(1.0f));
    }

    const f32 travelled = character.position().x - startX;
    CHECK(travelled > 9.0f);
    CHECK(character.grounded());
}

void makeLedgeMesh(f32 height, std::vector<Math::vec3>& vertices, std::vector<u32>& indices)
{
    vertices = {Math::vec3(-10.0f, 0.0f, -10.0f), Math::vec3(0.0f, 0.0f, -10.0f),
                Math::vec3(0.0f, 0.0f, 10.0f),    Math::vec3(-10.0f, 0.0f, 10.0f),
                Math::vec3(0.0f, height, -10.0f), Math::vec3(0.0f, height, 10.0f),
                Math::vec3(10.0f, height, 10.0f), Math::vec3(10.0f, height, -10.0f),
                Math::vec3(0.0f, 0.0f, -10.0f),   Math::vec3(0.0f, 0.0f, 10.0f)};
    indices = {// lower floor, +Y
               0, 2, 1, 0, 3, 2,
               // upper floor, +Y
               4, 5, 6, 4, 6, 7,
               // the riser between them, -X
               8, 9, 4, 9, 5, 4};
}

f32 walkAtLedge(f32 ledgeHeight, f32 stepOffset)
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeLedgeMesh(ledgeHeight, vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setStepOffset(stepOffset);
    character.setPosition(Math::vec3(-3.0f, 1.05f, 0.0f));
    character.setMoveInput(Math::vec3(3.0f, 0.0f, 0.0f));
    for (u32 i = 0; i < 180; ++i)
    {
        const auto r = character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));
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

    // An ellipsoid rides a step on its curved base (no step-up pass): it climbs up to 0.7, 0.8+ is a wall; the ceiling sits under the vertical radius 1.0 since the widest cross-section is at its centre. Projecting with the world normal (as CharacterController) removed the wall launch; above 0.7 means a residual tangent only to the ellipsoid crept back.
    CHECK(walkAtLedge(0.3f, 0.0f) > 0.5f);
    CHECK(walkAtLedge(0.7f, 0.0f) > 0.5f);
    CHECK(walkAtLedge(0.8f, 0.0f) < 0.0f);
    CHECK(walkAtLedge(1.5f, 0.35f) < 0.0f);
}

void testCharacterDoesNotHopAtAPlatformSeam()
{
    // Two platforms at the SAME height butted at x = 0 without shared vertices: the seam edge reports a steep normal on level ground.
    std::vector<Math::vec3> vertices = {Math::vec3(-8.0f, 1.0f, -8.0f), Math::vec3(0.0f, 1.0f, -8.0f),
                                       Math::vec3(0.0f, 1.0f, 8.0f),   Math::vec3(-8.0f, 1.0f, 8.0f),
                                       Math::vec3(0.0f, 1.0f, -8.0f),  Math::vec3(8.0f, 1.0f, -8.0f),
                                       Math::vec3(8.0f, 1.0f, 8.0f),   Math::vec3(0.0f, 1.0f, 8.0f)};
    std::vector<u32> indices = {0, 2, 1, 0, 3, 2, 4, 6, 5, 4, 7, 6};

    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setStepOffset(0.35f);
    character.setPosition(Math::vec3(-4.0f, 3.0f, 0.0f));
    for (u32 i = 0; i < 180; ++i)
        character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));
    CHECK(character.grounded());

    const f32 settled = character.position().y;
    character.setMoveInput(Math::vec3(3.0f, 0.0f, 0.0f));
    f32 highest = settled;
    for (u32 i = 0; i < 180; ++i)
    {
        character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));
        highest = Math::max(highest, character.position().y);
    }

    // An offset firing at the seam shows as a 0.35 hop.
    CHECK(character.position().x > 1.0f);
    CHECK(highest < settled + 0.05f);
    CHECK(character.grounded());
}

void testCharacterIsNeverLaunchedByAContact()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    // Started INSIDE the wall at x = 2: the push-out must not be converted into velocity.
    character.setPosition(Math::vec3(2.0f, 1.0f, 0.0f));

    f32 highest = character.position().y;
    for (u32 i = 0; i < 180; ++i)
    {
        const Math::vec3 before = character.position();
        character.setMoveInput(Math::vec3(4.0f, 0.0f, 0.0f));
        character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));
        highest = Math::max(highest, character.position().y);
        CHECK(character.verticalSpeed() < 1.0f);
        // No frame may move him more than one push-out plus the requested step; iterating the push-out flings a wedged body.
        CHECK(Math::length(character.position() - before) < 1.5f);
    }
    CHECK(highest < 2.0f);
    CHECK(character.position().x < 2.0f);
}

void testGroundedNeverFlickersWhileStandingStill()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::vec3(-2.0f, 4.0f, 0.0f));
    for (u32 i = 0; i < 180; ++i)
        character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));
    CHECK(character.grounded());

    // Settled: he rests one skin width clear and a frame of falling is a quarter of it, so a bare downward sweep finds nothing and `grounded` flickers; every frame must report standing.
    const f32 settled = character.position().y;
    u32 airborneFrames = 0;
    for (u32 i = 0; i < 240; ++i)
    {
        character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));
        if (!character.grounded())
            ++airborneFrames;
    }
    CHECK(airborneFrames == 0);
    CHECK(near(character.position().y, settled, 1e-3f));

    character.setMoveInput(Math::vec3(2.0f, 0.0f, 0.0f));
    airborneFrames = 0;
    for (u32 i = 0; i < 180; ++i)
    {
        character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));
        if (!character.grounded())
            ++airborneFrames;
    }
    CHECK(airborneFrames == 0);
}

void testCharacterFallsAndLandsUnderGravity()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::vec3(-2.0f, 6.0f, 0.0f));
    CHECK(!character.grounded());

    for (u32 i = 0; i < 240; ++i)
        character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));

    CHECK(character.grounded());
    CHECK(near(character.verticalSpeed(), 0.0f, 1e-3f));
    CHECK(near(character.slopeAngle(), 0.0f, 2.0f));
}

void testCharacterJumpsOnlyFromTheGround()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::vec3(-2.0f, 6.0f, 0.0f));
    for (u32 i = 0; i < 240; ++i)
        character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));

    const f32 standing = character.position().y;
    character.jump(8.0f);
    f32 peak = standing;
    for (u32 i = 0; i < 40; ++i)
    {
        character.jump(8.0f);
        character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));
        peak = Math::max(peak, character.position().y);
    }
    CHECK(peak > standing + 1.0f);
    CHECK(peak < standing + 3.0f);

    for (u32 i = 0; i < 120; ++i)
        character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));
    CHECK(character.grounded());
    CHECK(near(character.position().y, standing, 0.05f));
}

void testTeleportClearsTheFall()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeRoomMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Physics::CharacterBody character;
    character.setShape(0.4f, 1.2f);
    character.setPosition(Math::vec3(-2.0f, 20.0f, 0.0f));
    for (u32 i = 0; i < 60; ++i)
        character.update(1.0f / 60.0f, mesh, Math::mat4(1.0f));
    CHECK(character.verticalSpeed() < -1.0f);

    character.teleport(Math::vec3(-4.0f, 3.0f, 0.0f));
    CHECK(near(character.verticalSpeed(), 0.0f));
    CHECK(near(character.position(), Math::vec3(-4.0f, 3.0f, 0.0f)));
}

void testTrimeshRaycastFindsTheNearestTriangle()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const usize base = vertices.size();
    for (usize i = 0; i < base; ++i)
        vertices.push_back(vertices[i] + Math::vec3(0.0f, 2.0f, 0.0f));
    const usize indexBase = indices.size();
    for (usize i = 0; i < indexBase; ++i)
        indices.push_back(indices[i] + static_cast<u32>(base));

    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Ray ray;
    ray.origin = Math::vec3(1.0f, 10.0f, 1.0f);
    ray.direction = Math::vec3(0.0f, -1.0f, 0.0f);

    TrimeshShape::RayHit hit;
    CHECK(mesh.raycast(ray, 100.0f, hit));
    CHECK(near(hit.point.y, 2.0f));
    CHECK(near(hit.distance, 8.0f));
    CHECK(near(std::abs(hit.normal.y), 1.0f));

    CHECK(!mesh.raycast(ray, 4.0f, hit));

    ray.origin = Math::vec3(1.0f, -10.0f, 1.0f);
    ray.direction = Math::vec3(0.0f, 1.0f, 0.0f);
    CHECK(mesh.raycast(ray, 100.0f, hit));
    CHECK(near(hit.point.y, 0.0f));
}

void testTrimeshRaycastFromInsideTheBounds()
{
    // A deep third floor stretches the root box far under the ray; the prune must not confuse the box exit distance with the ray budget (a short ray between floors must hit the floor beneath).
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const usize base = vertices.size();
    const usize indexBase = indices.size();
    for (usize i = 0; i < base; ++i)
        vertices.push_back(vertices[i] + Math::vec3(0.0f, 2.0f, 0.0f));
    for (usize i = 0; i < base; ++i)
        vertices.push_back(vertices[i] + Math::vec3(0.0f, -10.0f, 0.0f));
    for (usize i = 0; i < indexBase; ++i)
        indices.push_back(indices[i] + static_cast<u32>(base));
    for (usize i = 0; i < indexBase; ++i)
        indices.push_back(indices[i] + static_cast<u32>(base * 2));

    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    Ray ray;
    ray.origin = Math::vec3(1.0f, 1.0f, 1.0f);
    ray.direction = Math::vec3(0.0f, -1.0f, 0.0f);

    TrimeshShape::RayHit hit;
    CHECK(mesh.raycast(ray, 1.5f, hit));
    CHECK(near(hit.point.y, 0.0f));
    CHECK(near(hit.distance, 1.0f));
}

void testTrimeshOverlapSphereRejectsNearMisses()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

    std::vector<u32> hits;
    mesh.overlapSphere(Math::vec3(0.0f, 0.5f, 0.0f), 1.0f, hits);
    CHECK(!hits.empty());

    mesh.overlapSphere(Math::vec3(0.0f, 5.0f, 0.0f), 1.0f, hits);
    CHECK(hits.empty());

    mesh.overlapSphere(Math::vec3(7.0f, 0.0f, 0.0f), 1.0f, hits);
    CHECK(hits.empty());
}

void testSharedEdgesAreDetected()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));

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
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));
    const CapsuleShape capsule(0.4f, 0.8f);

    // Contacts across the diagonal seam must push straight up; a normal tilted towards it is a wall on flat ground.
    for (int step = -8; step <= 8; ++step)
    {
        const f32 x = static_cast<f32>(step) * 0.25f;
        const Math::vec3 position(x, 1.15f, -x);
        std::vector<ContactManifold> manifolds;
        if (!Narrowphase::convexTrimesh(capsule, at(position), mesh, Math::mat4(1.0f), manifolds))
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
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    makeGroundMesh(vertices, indices);
    const TrimeshShape mesh(vertices.data(), static_cast<u32>(vertices.size()), indices.data(),
                            static_cast<u32>(indices.size()));
    const SphereShape sphere(1.0f);

    // A real rim edge must not be flattened to the face normal, or nothing falls off.
    std::vector<ContactManifold> manifolds;
    CHECK(Narrowphase::convexTrimesh(sphere, at(Math::vec3(5.6f, 0.0f, 0.0f)), mesh, Math::mat4(1.0f),
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
    // Flipping one triangle makes both traverse the shared edge the same way: windingErrors must flag a backface.
    std::vector<Math::vec3> vertices = {Math::vec3(0, 0, 0), Math::vec3(2, 0, 0), Math::vec3(2, 0, 2),
                                       Math::vec3(0, 0, 2)};
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
    // A plane hit at t >> 1 must be rejected when the sweep cannot reach it; a full-length sweep reaches it with the wall's normal.
    std::vector<Math::vec3> vertices = {
        Math::vec3(-5, 0, -5), Math::vec3(5, 0, -5), Math::vec3(5, 0, 5), Math::vec3(-5, 0, 5),
        Math::vec3(-5, 0, 4), Math::vec3(5, 0, 4), Math::vec3(5, 6, 4), Math::vec3(-5, 6, 4)};
    const std::vector<u32> indices = {0, 2, 1, 0, 3, 2, 4, 6, 5, 4, 7, 6};
    const TrimeshShape mesh(vertices.data(), 8, indices.data(), 12);

    TrimeshShape::SweepHit hit;
    CHECK(!mesh.sweepSphere(Math::vec3(0.0f, 1.0f, 0.0f), 0.5f, Math::vec3(0.0f, 0.0f, 0.5f), hit));

    CHECK(mesh.sweepSphere(Math::vec3(0.0f, 1.0f, 0.0f), 0.5f, Math::vec3(0.0f, 0.0f, 4.0f), hit));
    CHECK(near(hit.t, 3.5f / 4.0f, 1e-3f));
    CHECK(near(hit.normal, Math::vec3(0.0f, 0.0f, -1.0f), 1e-3f));
}

void testNarrowphaseMarginReportsSpeculativeContacts()
{
    // No contact at margin 0; a speculative contact with negative penetration within a margin.
    const SphereShape a(0.5f);
    const SphereShape b(0.5f);
    const Math::mat4 ta = at(Math::vec3(0.0f));
    const Math::mat4 tb = at(Math::vec3(1.1f, 0.0f, 0.0f));

    ContactManifold manifold;
    CHECK(!Narrowphase::collide(a, ta, b, tb, manifold));

    CHECK(Narrowphase::collide(a, ta, b, tb, manifold, 0.2f));
    CHECK(manifold.count == 1);
    CHECK(manifold.points[0].penetration < 0.0f);
    CHECK(near(manifold.points[0].penetration, -0.1f, 1e-3f));
    CHECK(near(manifold.normal, Math::vec3(1.0f, 0.0f, 0.0f), 1e-3f));

    const BoxShape box(Math::vec3(0.5f));
    const Math::mat4 tbox = at(Math::vec3(1.1f, 0.0f, 0.0f));
    ContactManifold boxManifold;
    CHECK(Narrowphase::collide(a, ta, box, tbox, boxManifold, 0.2f));
    CHECK(boxManifold.points[0].penetration < 0.0f);
}

void testShapeInertiaTensors()
{
    const SphereShape sphere(1.0f);
    const Math::mat3 sphereI = sphere.inertia(5.0f);
    const f32 expected = 0.4f * 5.0f * 1.0f;
    CHECK(near(sphereI[0][0], expected));
    CHECK(near(sphereI[1][1], expected));
    CHECK(near(sphereI[2][2], expected));
    CHECK(near(sphereI[0][1], 0.0f, 1e-6f));

    // Box: m/12 * (dy^2 + dz^2) about x, using the FULL edge lengths.
    const BoxShape box(Math::vec3(1.0f, 2.0f, 3.0f));
    const f32 m = 6.0f;
    const Math::mat3 boxI = box.inertia(m);
    CHECK(near(boxI[0][0], m / 12.0f * (4.0f * 4.0f + 6.0f * 6.0f), 1e-3f));
    CHECK(near(boxI[1][1], m / 12.0f * (2.0f * 2.0f + 6.0f * 6.0f), 1e-3f));
    CHECK(near(boxI[2][2], m / 12.0f * (2.0f * 2.0f + 4.0f * 4.0f), 1e-3f));

    const CapsuleShape capsule(0.5f, 1.0f);
    const Math::mat3 capsuleI = capsule.inertia(2.0f);
    CHECK(near(capsuleI[0][1], 0.0f, 1e-6f));
    CHECK(capsuleI[0][0] > capsuleI[1][1]);
    CHECK(capsuleI[1][1] > 0.0f);
}

void testBoxFaceHelpers()
{
    const BoxShape box(Math::vec3(1.0f, 2.0f, 3.0f));
    const Math::mat4 identity = at(Math::vec3(0.0f));

    const Math::vec3 expected[6] = {Math::vec3(-1, 0, 0), Math::vec3(1, 0, 0),  Math::vec3(0, -1, 0),
                                   Math::vec3(0, 1, 0),  Math::vec3(0, 0, -1), Math::vec3(0, 0, 1)};
    for (u32 face = 0; face < 6; ++face)
    {
        CHECK(near(BoxShape::faceNormal(identity, face), expected[face], 1e-5f));
        const Math::vec3 normal = expected[face];
        const f32 halfExtent =
            1.0f * (normal.x != 0.0f) + 2.0f * (normal.y != 0.0f) + 3.0f * (normal.z != 0.0f);
        const u8* corners = BoxShape::faceCorners(face);
        Math::vec3 world[8];
        box.corners(identity, world);
        for (u32 c = 0; c < 4; ++c)
            CHECK(near(Math::dot(world[corners[c]], normal), halfExtent, 1e-5f));
    }

    const Math::mat4 turned =
        at(Math::vec3(0.0f), Math::angleAxis(Math::half_pi<f32>(), Math::vec3(0, 0, 1)));
    CHECK(near(BoxShape::faceNormal(turned, 1), Math::vec3(0.0f, 1.0f, 0.0f), 1e-3f));
}

void testTriangleFeatureIsInternal()
{
    // Two shared edges: the face is internal, shared edges report the face normal, a vertex is internal only when both its edges are shared.
    const TriangleShape triangle(Math::vec3(0, 0, 0), Math::vec3(1, 0, 0), Math::vec3(0, 1, 0),
                                 /*sharedEdges=*/0b011);
    CHECK(triangle.edgeIsShared(0));
    CHECK(triangle.edgeIsShared(1));
    CHECK(!triangle.edgeIsShared(2));

    CHECK(triangle.featureIsInternal(TriangleFeature::Face));
    CHECK(triangle.featureIsInternal(TriangleFeature::Edge0));
    CHECK(triangle.featureIsInternal(TriangleFeature::Edge1));
    CHECK(!triangle.featureIsInternal(TriangleFeature::Edge2));

    CHECK(triangle.featureIsInternal(TriangleFeature::Vertex1));
    CHECK(!triangle.featureIsInternal(TriangleFeature::Vertex2));

    CHECK(near(triangle.rawNormal(), Math::vec3(0.0f, 0.0f, 1.0f)));
}

void testConvexTrimeshBoxRestsOnFloorAndInCorner()
{
    // A box on the floor must report up normals; wedged in the corner it must report ONE manifold per touching triangle, never one averaged normal.
    std::vector<Math::vec3> vertices = {
        Math::vec3(-5, 0, -5), Math::vec3(5, 0, -5), Math::vec3(5, 0, 5), Math::vec3(-5, 0, 5),
        Math::vec3(2, 0, -5), Math::vec3(2, 0, 5), Math::vec3(2, 6, 5), Math::vec3(2, 6, -5)};
    const std::vector<u32> indices = {0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7};
    const TrimeshShape mesh(vertices.data(), 8, indices.data(), 12);

    const BoxShape box(Math::vec3(0.5f));
    std::vector<ContactManifold> manifolds;

    manifolds.clear();
    CHECK(Narrowphase::convexTrimesh(box, at(Math::vec3(0.0f, 0.49f, 0.0f)), mesh,
                                     at(Math::vec3(0.0f)), manifolds));
    CHECK(!manifolds.empty());
    for (const ContactManifold& manifold : manifolds)
        CHECK(near(manifold.normal, Math::vec3(0.0f, -1.0f, 0.0f), 1e-3f));

    manifolds.clear();
    CHECK(Narrowphase::convexTrimesh(box, at(Math::vec3(1.51f, 0.49f, 0.0f)), mesh,
                                     at(Math::vec3(0.0f)), manifolds));
    CHECK(manifolds.size() >= 2);
    bool up = false;
    bool wall = false;
    for (const ContactManifold& manifold : manifolds)
    {
        if (near(manifold.normal, Math::vec3(0.0f, -1.0f, 0.0f), 1e-3f))
            up = true;
        if (near(manifold.normal, Math::vec3(1.0f, 0.0f, 0.0f), 1e-3f))
            wall = true;
    }
    CHECK(up);
    CHECK(wall);
}

void testCharacterRigidBodyOnGround()
{
    BoxShape floor(Math::vec3(5.0f, 0.5f, 5.0f));
    RigidBody floorBody;
    floorBody.setBodyType(BodyType::Static);
    floorBody.setPosition(Math::vec3(0.0f, -0.5f, 0.0f));

    floorBody.setShape(&floor);

    Radion::Scene world;
    world.addBody(floorBody);

    CharacterRigidBody character;
    character.setShape(0.4f, 1.2f);
    character.addToWorld(world, Math::vec3(0.0f, 1.0f, 0.0f));
    character.postSimulation(0.05f);

    CHECK(character.groundState() == CharacterRigidBody::GroundState::OnGround);
    CHECK(character.isSupported());
    CHECK(near(character.groundNormal(), Math::vec3(0.0f, 1.0f, 0.0f), 1e-2f));

    character.removeFromWorld();
}

void testCharacterRigidBodyOnSteepGround()
{
    const f32 angleDegrees = 70.0f;
    const Math::quat rotation =
        Math::angleAxis(Math::radians(angleDegrees), Math::vec3(0.0f, 0.0f, 1.0f));
    const Math::vec3 normal = Math::normalize(rotation * Math::vec3(0.0f, 1.0f, 0.0f));

    BoxShape ramp(Math::vec3(5.0f, 0.5f, 5.0f));
    RigidBody rampBody;
    rampBody.setBodyType(BodyType::Static);
    rampBody.setPosition(Math::vec3(0.0f));
    rampBody.setOrientation(rotation);

    rampBody.setShape(&ramp);

    Radion::Scene world;
    world.addBody(rampBody);

    const Math::vec3 topFaceCentre = rotation * Math::vec3(0.0f, 0.5f, 0.0f);
    CharacterRigidBody character;
    character.setShape(0.4f, 0.0f);
    character.addToWorld(world, topFaceCentre + normal * 0.38f);
    character.postSimulation(0.05f);

    CHECK(character.groundState() == CharacterRigidBody::GroundState::OnSteepGround);
    CHECK(character.isSupported());
    CHECK(near(character.groundNormal(), normal, 1e-2f));

    character.removeFromWorld();
}

// Regression: a (near) 0 degree max slope means "check off", not "reject every slope"; the steep ramp must read OnGround.
void testCharacterRigidBodyMaxSlopeAngleZeroDisablesTheCheck()
{
    const f32 angleDegrees = 70.0f;
    const Math::quat rotation =
        Math::angleAxis(Math::radians(angleDegrees), Math::vec3(0.0f, 0.0f, 1.0f));
    const Math::vec3 normal = Math::normalize(rotation * Math::vec3(0.0f, 1.0f, 0.0f));

    BoxShape ramp(Math::vec3(5.0f, 0.5f, 5.0f));
    RigidBody rampBody;
    rampBody.setBodyType(BodyType::Static);
    rampBody.setPosition(Math::vec3(0.0f));
    rampBody.setOrientation(rotation);

    rampBody.setShape(&ramp);

    Radion::Scene world;
    world.addBody(rampBody);

    const Math::vec3 topFaceCentre = rotation * Math::vec3(0.0f, 0.5f, 0.0f);
    CharacterRigidBody character;
    character.setShape(0.4f, 0.0f);
    character.setMaxSlopeAngle(0.0f);
    character.addToWorld(world, topFaceCentre + normal * 0.38f);
    character.postSimulation(0.05f);

    CHECK(character.groundState() == CharacterRigidBody::GroundState::OnGround);
    CHECK(character.isSupported());

    character.removeFromWorld();
}

void testCharacterRigidBodyInAir()
{
    Radion::Scene world;
    CharacterRigidBody character;
    character.setShape(0.4f, 1.2f);
    character.addToWorld(world, Math::vec3(0.0f, 100.0f, 0.0f));
    character.postSimulation(0.05f);

    CHECK(character.groundState() == CharacterRigidBody::GroundState::InAir);
    CHECK(!character.isSupported());

    character.removeFromWorld();
}

void testCharacterRigidBodyPushesALightDynamicBox()
{
    Radion::Scene world;
    world.setGravity(Math::vec3(0.0f));

    BoxShape boxShape(Math::vec3(0.3f));
    RigidBody boxBody;
    boxBody.setMass(0.2f);
    boxBody.setInertiaTensor(boxShape.inertia(0.2f));
    boxBody.setPosition(Math::vec3(0.68f, 0.0f, 0.0f));
    boxBody.setShape(&boxShape);
    boxBody.setFriction(0.0f);
    world.addBody(boxBody);

    CharacterRigidBody character;
    character.setShape(0.4f, 1.2f);
    character.setFriction(0.0f);
    character.addToWorld(world, Math::vec3(0.0f, 0.0f, 0.0f));
    character.setLinearVelocity(Math::vec3(5.0f, 0.0f, 0.0f));

    world.stepPhysics(1.0f / 120.0f);
    character.postSimulation(0.05f);

    CHECK(boxBody.velocity().x > 0.01f);

    character.removeFromWorld();
}

void testCharacterRigidBodyMovesAfterFallingAsleep()
{
    BoxShape floorShape(Math::vec3(10.0f, 0.5f, 10.0f));
    RigidBody floor;
    floor.setBodyType(BodyType::Static);
    floor.setPosition(Math::vec3(0.0f, -0.5f, 0.0f));

    floor.setShape(&floorShape);

    Radion::Scene world;
    world.addBody(floor);

    CharacterRigidBody character;
    character.setShape(0.4f, 1.2f);
    character.addToWorld(world, Math::vec3(0.0f, 1.2f, 0.0f));

    for (u32 i = 0; i < 600; ++i)
        world.stepPhysics(1.0f / 120.0f);

    const Math::vec3 rested = character.position();

    for (u32 i = 0; i < 120; ++i)
    {
        character.setLinearVelocity(Math::vec3(3.0f, character.linearVelocity().y, 0.0f));
        world.stepPhysics(1.0f / 120.0f);
    }

    CHECK(character.position().x - rested.x > 1.0f);

    character.removeFromWorld();
}

void testDynamicBoxDroppedFromHeightRestsOnTrimesh()
{
    std::vector<Math::vec3> vertices;
    std::vector<u32> indices;
    const f32 extent = 12.0f;
    const u32 segments = 6;
    for (u32 z = 0; z <= segments; ++z)
        for (u32 x = 0; x <= segments; ++x)
            vertices.push_back(Math::vec3(-extent + 2.0f * extent * x / segments, 0.0f,
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
    floor.setPosition(Math::vec3(0.0f));
    floor.setShape(&floorMesh);
    floor.setFriction(0.8f);

    Radion::Scene world;
    world.setGravity(Math::vec3(0.0f, -20.0f, 0.0f));

    BoxShape boxShape(Math::vec3(0.4f));
    RigidBody box;
    box.setMass(4.0f);
    box.setInertiaTensor(boxShape.inertia(4.0f));
    box.setPosition(Math::vec3(1.7f, 8.0f, 1.3f));
    box.setDamping(0.999f, 0.999f);
    box.setShape(&boxShape);
    box.setFriction(0.6f);
    box.setRestitution(0.05f);
    world.addBody(box);
    world.addBody(floor);

    for (u32 i = 0; i < 600; ++i)
        world.stepPhysics(1.0f / 120.0f);

    CHECK(std::isfinite(box.position().y));
    CHECK(box.position().y > 0.3f);
    CHECK(box.position().y < 1.0f);
}

void testWorldRaycastFindsTheNearestBody()
{
    SphereShape sphere(1.0f);
    RigidBody nearBody;
    nearBody.setBodyType(BodyType::Static);
    nearBody.setPosition(Math::vec3(0.0f, 0.0f, 0.0f));
    RigidBody farBody;
    farBody.setBodyType(BodyType::Static);
    farBody.setPosition(Math::vec3(5.0f, 0.0f, 0.0f));
    nearBody.setShape(&sphere);
    farBody.setShape(&sphere);

    Radion::Scene world;
    world.addBody(nearBody);
    world.addBody(farBody);

    Ray ray;
    ray.origin = Math::vec3(-5.0f, 0.0f, 0.0f);
    ray.direction = Math::vec3(1.0f, 0.0f, 0.0f);

    WorldRayHit hit;
    CHECK(world.raycast(ray, 100.0f, QueryFilter(), hit));
    CHECK(hit.body == &nearBody);
    CHECK(near(hit.distance, 4.0f));
    CHECK(near(hit.point, Math::vec3(-1.0f, 0.0f, 0.0f)));
    CHECK(near(hit.normal, Math::vec3(-1.0f, 0.0f, 0.0f)));
}

void testWorldRaycastMaskExcludesLayer()
{
    SphereShape sphere(1.0f);
    RigidBody body;
    body.setBodyType(BodyType::Static);
    body.setPosition(Math::vec3(0.0f, 0.0f, 0.0f));

    body.setShape(&sphere);
    body.setCollisionGroup(2);

    Radion::Scene world;
    world.addBody(body);

    Ray ray;
    ray.origin = Math::vec3(-5.0f, 0.0f, 0.0f);
    ray.direction = Math::vec3(1.0f, 0.0f, 0.0f);

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
    body.setPosition(Math::vec3(0.0f, 0.0f, 0.0f));

    body.setShape(&shape);
    body.setCollisionGroup(4);

    Radion::Scene world;
    world.addBody(body);

    std::vector<RigidBody*> hits;
    world.overlapSphere(Math::vec3(0.5f, 0.0f, 0.0f), 1.0f, QueryFilter(), hits);
    CHECK(hits.size() == 1);
    CHECK(hits[0] == &body);

    QueryFilter onlyGroup1;
    onlyGroup1.collision.mask = 1u;
    world.overlapSphere(Math::vec3(0.5f, 0.0f, 0.0f), 1.0f, onlyGroup1, hits);
    CHECK(hits.empty());

    world.overlapSphere(Math::vec3(50.0f, 0.0f, 0.0f), 1.0f, QueryFilter(), hits);
    CHECK(hits.empty());
}

void testWorldStepSkipsIncompatibleMasks()
{
    BoxShape groundShape(Math::vec3(5.0f, 0.5f, 5.0f));
    BoxShape dropperShape(Math::vec3(0.5f));

    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::vec3(0.0f, 0.0f, 0.0f));

    RigidBody dropper;
    dropper.setMass(1.0f);
    dropper.setInertiaTensor(dropperShape.inertia(1.0f));
    dropper.setPosition(Math::vec3(0.0f, 1.5f, 0.0f));

    ground.setShape(&groundShape);
    ground.setFilter({1, 1});
    dropper.setShape(&dropperShape);
    dropper.setFilter({2, 2});

    Radion::Scene world;
    world.setGravity(Math::vec3(0.0f, -9.81f, 0.0f));
    world.addBody(ground);
    world.addBody(dropper);

    for (u32 i = 0; i < 200; ++i)
        world.stepPhysics(1.0f / 120.0f);

    CHECK(dropper.position().y < -1.0f);
}

void testWorldRemovalDetachesOneBodyAndKeepsOthersWorking()
{
    SphereShape shape(1.0f);
    RigidBody bodies[4];
    Radion::Scene world;

    for (u32 i = 0; i < 3; ++i)
    {
        bodies[i].setBodyType(BodyType::Static);
        bodies[i].setPosition(Math::vec3(static_cast<f32>(i) * 4.0f, 0.0f, 0.0f));
        bodies[i].setShape(&shape);
        world.addBody(bodies[i]);
    }

    world.removeBody(bodies[1]);
    CHECK(world.bodyCount() == 2);
    CHECK(bodies[1].scene() == nullptr);
    CHECK(bodies[0].scene() == &world);
    CHECK(bodies[2].scene() == &world);

    bodies[3].setBodyType(BodyType::Static);
    bodies[3].setShape(&shape);
    world.addBody(bodies[3]);
    CHECK(world.bodyCount() == 3);
    CHECK(bodies[3].scene() == &world);
    CHECK(bodies[2].scene() == &world);
}

void testWorldAllowsMutationFromCollisionCallback()
{
    struct Mutation
    {
        Radion::Scene* world = nullptr;
        RigidBody* spare = nullptr;
        RigidBody* removed = nullptr;
        bool done = false;
    };

    BoxShape shape(Math::vec3(0.5f));
    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::vec3(0.0f, -0.5f, 0.0f));
    ground.setShape(&shape);
    RigidBody box;
    box.setMass(1.0f);
    box.setInertiaTensor(shape.inertia(1.0f));
    box.setPosition(Math::vec3(0.0f, 0.45f, 0.0f));
    box.setShape(&shape);
    RigidBody spare;
    spare.setBodyType(BodyType::Static);
    spare.setPosition(Math::vec3(20.0f, 0.0f, 0.0f));
    spare.setShape(&shape);

    Radion::Scene world;
    world.addBody(ground);
    world.addBody(box);

    Mutation mutation;
    mutation.world = &world;
    mutation.spare = &spare;
    world.setContactEventCallback(
        [](const ContactEventInfo& info, void* user)
        {
            Mutation& mutation = *static_cast<Mutation*>(user);
            if (mutation.done || info.event != ContactEvent::Enter)
                return;
            mutation.done = true;
            mutation.removed = info.bodyB;
            mutation.world->removeBody(*mutation.removed);
            mutation.world->addBody(*mutation.spare);
            // Events are dispatched after the solver releases its temporary references, so mutation is safe.
            CHECK(mutation.removed->scene() == nullptr);
            CHECK(mutation.spare->scene() == mutation.world);
        },
        &mutation);

    world.stepPhysics(1.0f / 120.0f);
    CHECK(mutation.done);
    CHECK(mutation.removed == &box);
    CHECK(world.bodyCount() == 2);
    CHECK(ground.scene() == &world);
    CHECK(mutation.removed->scene() == nullptr);
    CHECK(spare.scene() == &world);
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

    Radion::Scene world;
    world.setGravity(Math::vec3(0.0f));
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
        bodies[i]->setPosition(Math::vec3(positions[i], 0.0f, 0.0f));
        bodies[i]->setShape(&shape);
        world.addBody(*bodies[i]);
    }

    CHECK(world.applyRadialImpulse(Math::vec3(0.0f), 5.0f, 10.0f) == 2);
    CHECK(nearBody.velocity().x > farBody.velocity().x);
    CHECK(farBody.velocity().x > 0.0f);
    CHECK(near(outsideBody.velocity(), Math::vec3(0.0f)));
    CHECK(near(staticBody.velocity(), Math::vec3(0.0f)));

    nearBody.setVelocity(Math::vec3(0.0f));
    farBody.setVelocity(Math::vec3(0.0f));
    CHECK(world.addRadialForce(Math::vec3(0.0f), 5.0f, -10.0f) == 2);
    world.stepPhysics(0.1f);
    CHECK(nearBody.velocity().x < farBody.velocity().x);
    CHECK(farBody.velocity().x < 0.0f);

    nearBody.setVelocity(Math::vec3(0.0f));
    farBody.setVelocity(Math::vec3(0.0f));
    CHECK(world.addDirectionalForce(Math::vec3(0.0f), 2.0f, Math::vec3(0.0f, 0.0f, 8.0f)) == 1);
    world.stepPhysics(0.1f);
    CHECK(nearBody.velocity().z > 0.0f);
    CHECK(near(farBody.velocity().z, 0.0f));
}

void testPlaneShape()
{
    const PlaneShape plane(Math::vec3(0.0f, 2.0f, 0.0f), 2.0f);
    const SphereShape sphere(1.0f);
    const Math::mat4 planeTransform = at(Math::vec3(0.0f, 1.0f, 0.0f));
    ContactManifold manifold;

    CHECK(Narrowphase::collide(sphere, at(Math::vec3(0.0f, 3.5f, 0.0f)), plane,
                               planeTransform, manifold));
    CHECK(near(manifold.normal, Math::vec3(0.0f, -1.0f, 0.0f)));
    CHECK(near(manifold.points[0].penetration, 0.5f));
    CHECK(near(manifold.points[0].position, Math::vec3(0.0f, 3.0f, 0.0f)));

    CHECK(Narrowphase::collide(plane, planeTransform, sphere,
                               at(Math::vec3(0.0f, 3.5f, 0.0f)), manifold));
    CHECK(near(manifold.normal, Math::vec3(0.0f, 1.0f, 0.0f)));

    CHECK(!Narrowphase::collide(sphere, at(Math::vec3(0.0f, 4.1f, 0.0f)), plane,
                                planeTransform, manifold));
    CHECK(Narrowphase::collide(sphere, at(Math::vec3(0.0f, 4.1f, 0.0f)), plane,
                               planeTransform, manifold, 0.2f));
    CHECK(near(manifold.points[0].penetration, -0.1f));

    Ray ray;
    ray.origin = Math::vec3(0.0f, 5.0f, 0.0f);
    ray.direction = Math::vec3(0.0f, -1.0f, 0.0f);
    ShapeRayHit hit;
    CHECK(Narrowphase::raycast(plane, planeTransform, ray, 10.0f, hit));
    CHECK(near(hit.distance, 2.0f));
    CHECK(near(hit.point, Math::vec3(0.0f, 3.0f, 0.0f)));
    CHECK(!Narrowphase::overlapSphere(plane, planeTransform, Math::vec3(0.0f, 4.2f, 0.0f),
                                     1.0f));
    CHECK(Narrowphase::overlapSphere(plane, planeTransform, Math::vec3(0.0f, 3.5f, 0.0f),
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
    moving.setPosition(Math::vec3(2.0f, 0.0f, 0.0f));
    moving.setCanSleep(false);

    fixed.setShape(&shape);
    fixed.setCollisionMask(0);
    moving.setShape(&shape);
    moving.setCollisionMask(0);

    Radion::Scene world;
    world.setGravity(Math::vec3(0.0f));
    world.addBody(fixed);
    world.addBody(moving);

    PointJoint joint(fixed, Math::vec3(0.0f), moving, Math::vec3(0.0f));
    world.addJoint(&joint);
    CHECK(world.jointCount() == 1);
    for (u32 i = 0; i < 120; ++i)
        world.stepPhysics(1.0f / 120.0f);
    CHECK(Math::length(joint.worldAnchorB() - joint.worldAnchorA()) < 0.01f);

    world.removeBody(moving);
    CHECK(world.jointCount() == 0);
}

void testPointJointCarMoves()
{
    PlaneShape groundShape(Math::vec3(0.0f, 1.0f, 0.0f));
    BoxShape chassisShape(Math::vec3(0.8f, 0.25f, 1.4f));
    SphereShape wheelShape(0.4f);
    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    RigidBody chassis;
    chassis.setMass(8.0f);
    chassis.setInertiaTensor(chassisShape.inertia(8.0f));
    chassis.setPosition(Math::vec3(0.0f, 0.85f, 0.0f));
    chassis.setCanSleep(false);
    RigidBody wheels[4];
    const Math::vec3 offsets[4] = {
        Math::vec3(-0.9f, -0.45f, -0.9f), Math::vec3(0.9f, -0.45f, -0.9f),
        Math::vec3(-0.9f, -0.45f, 0.9f), Math::vec3(0.9f, -0.45f, 0.9f)};

    ground.setShape(&groundShape);
    ground.setFriction(1.0f);

    Radion::Scene world;
    world.addBody(ground);
    CollisionFilter carFilter;
    carFilter.group = 2;
    carFilter.mask = 1;
    chassis.setShape(&chassisShape);
    chassis.setFilter(carFilter);
    chassis.setFriction(1.0f);
    world.addBody(chassis);

    std::vector<PointJoint> joints;
    joints.reserve(4);
    for (u32 i = 0; i < 4; ++i)
    {
        wheels[i].setMass(1.0f);
        wheels[i].setInertiaTensor(wheelShape.inertia(1.0f));
        wheels[i].setPosition(chassis.position() + offsets[i]);
        wheels[i].setCanSleep(false);
        wheels[i].setShape(&wheelShape);
        wheels[i].setFilter(carFilter);
        wheels[i].setFriction(1.0f);
        world.addBody(wheels[i]);
        joints.emplace_back(chassis, wheels[i], wheels[i].position());
        joints.back().setMotor(Math::vec3(1.0f, 0.0f, 0.0f), 14.0f, 25.0f);
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
                joints[i].setMotor(Math::vec3(1.0f, 0.0f, 0.0f), 10.0f + 5.0f * side,
                                   25.0f);
            }
        world.stepPhysics(1.0f / 120.0f);
        for (const PointJoint& joint : joints)
            maximumAnchorError =
                Math::max(maximumAnchorError,
                         Math::length(joint.worldAnchorB() - joint.worldAnchorA()));
    }

    CHECK(std::abs(chassis.position().z) > 0.5f);
    CHECK(maximumAnchorError < 0.1f);
    for (const PointJoint& joint : joints)
        CHECK(Math::length(joint.worldAnchorB() - joint.worldAnchorA()) < 0.1f);
}

void testConvexHullShapeMatchesBox()
{
    const Shard shard = buildCubeShard(1.0f);
    const ConvexHullShape hull(shard);
    const BoxShape box(Math::vec3(1.0f));
    const Math::mat4 identity = at(Math::vec3(0.0f));

    CHECK(near(hull.support(identity, Math::vec3(1, 1, 1)), box.support(identity, Math::vec3(1, 1, 1))));
    CHECK(near(hull.support(identity, Math::vec3(-1, 1, -1)),
              box.support(identity, Math::vec3(-1, 1, -1))));

    const AABB hullBounds = hull.bounds(identity);
    const AABB boxBounds = box.bounds(identity);
    CHECK(near(hullBounds.min, boxBounds.min));
    CHECK(near(hullBounds.max, boxBounds.max));

    const Math::mat3 hullI = hull.inertia(6.0f);
    const Math::mat3 boxI = box.inertia(6.0f);
    CHECK(near(hullI[0][0], boxI[0][0], 1e-2f));
    CHECK(near(hullI[1][1], boxI[1][1], 1e-2f));
    CHECK(near(hullI[2][2], boxI[2][2], 1e-2f));
    CHECK(near(hullI[0][1], 0.0f, 1e-2f));
    CHECK(near(hullI[0][2], 0.0f, 1e-2f));
    CHECK(near(hullI[1][2], 0.0f, 1e-2f));
}

void testConvexHullInertiaMatchesBoxClosedForm()
{
    // A non-cubic box, so a bug that appears only when the three axes differ cannot hide behind symmetry.
    const Shard shard = buildCubeShard(1.0f);
    Shard scaled = shard;
    for (Math::vec3& vertex : scaled.vertices)
        vertex *= Math::vec3(1.0f, 2.0f, 3.0f);
    const ConvexHullShape hull(scaled);
    const BoxShape box(Math::vec3(1.0f, 2.0f, 3.0f));

    const f32 mass = 6.0f;
    const Math::mat3 hullI = hull.inertia(mass);
    const Math::mat3 boxI = box.inertia(mass);
    CHECK(near(hullI[0][0], boxI[0][0], 1e-2f));
    CHECK(near(hullI[1][1], boxI[1][1], 1e-2f));
    CHECK(near(hullI[2][2], boxI[2][2], 1e-2f));
}

void testConvexHullBoxMatchesBoxBoxInvariants()
{
    const Shard shard = buildCubeShard(1.0f);
    const ConvexHullShape hull(shard);
    const BoxShape ground(Math::vec3(3.0f, 0.5f, 3.0f));

    const Math::mat4 hullTransform = at(Math::vec3(0.0f, 0.8f, 0.0f));
    const Math::mat4 groundTransform = at(Math::vec3(0.0f, -0.5f, 0.0f));

    ContactManifold manifold;
    CHECK(Narrowphase::collide(hull, hullTransform, ground, groundTransform, manifold));
    CHECK(manifold.count == 4);
    CHECK(near(manifold.normal, Math::vec3(0, -1, 0), 1e-3f));
    for (u32 i = 0; i < manifold.count; ++i)
        CHECK(near(manifold.points[i].penetration, 0.2f, 1e-2f));

    f32 spread = 0.0f;
    for (u32 i = 0; i < manifold.count; ++i)
        for (u32 j = i + 1; j < manifold.count; ++j)
            spread = Math::max(
                spread, Math::length(manifold.points[i].position - manifold.points[j].position));
    CHECK(spread > 1.5f);

    const Math::mat4 clear = at(Math::vec3(0.0f, 5.0f, 0.0f));
    CHECK(!Narrowphase::collide(hull, clear, ground, groundTransform, manifold));

    ContactManifold flipped;
    CHECK(Narrowphase::collide(ground, groundTransform, hull, hullTransform, flipped));
    CHECK(near(flipped.normal, Math::vec3(0, 1, 0), 1e-3f));
}

void testConvexHullSphereBasicContact()
{
    const Shard shard = buildCubeShard(1.0f);
    const ConvexHullShape hull(shard);
    const SphereShape sphere(0.5f);

    ContactManifold manifold;
    CHECK(Narrowphase::collide(hull, at(Math::vec3(0.0f)), sphere, at(Math::vec3(0.0f, 1.3f, 0.0f)),
                               manifold));
    CHECK(manifold.count == 1);
    CHECK(near(manifold.normal, Math::vec3(0, 1, 0), 1e-2f));
    CHECK(near(manifold.points[0].penetration, 0.2f, 1e-2f));

    CHECK(!Narrowphase::collide(hull, at(Math::vec3(0.0f)), sphere, at(Math::vec3(0.0f, 3.0f, 0.0f)),
                                manifold));
}

void testConvexHullCapsuleBasicContact()
{
    const Shard shard = buildCubeShard(1.0f);
    const ConvexHullShape hull(shard);
    const CapsuleShape capsule(0.3f, 0.6f);

    ContactManifold manifold;
    CHECK(Narrowphase::collide(hull, at(Math::vec3(0.0f)), capsule,
                               at(Math::vec3(0.0f, 1.85f, 0.0f)), manifold));
    CHECK(manifold.count >= 1);
    CHECK(std::abs(manifold.normal.y) > 0.9f);
    CHECK(manifold.points[0].penetration > 0.0f);

    CHECK(!Narrowphase::collide(hull, at(Math::vec3(0.0f)), capsule,
                                at(Math::vec3(0.0f, 4.0f, 0.0f)), manifold));
}

void testConvexHullConvexHullOverlapAndSeparation()
{
    // Cells split down the middle sit face to face; nudging one toward the other along the centroid line forces overlap.
    std::vector<Math::vec3> boxCorners;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2)
                boxCorners.push_back(Math::vec3(sx, sy, sz));
    std::vector<Math::vec3> voronoiPoints = {Math::vec3(-0.5f, 0, 0), Math::vec3(0.5f, 0, 0)};
    std::vector<Shard> shards;
    VoronoiShatter::shatter(boxCorners, voronoiPoints, shards);
    CHECK(shards.size() == 2);
    if (shards.size() != 2)
        return;

    const ConvexHullShape hullA(shards[0]);
    const ConvexHullShape hullB(shards[1]);
    const Math::vec3 direction = Math::normalize(shards[0].centroid - shards[1].centroid);

    ContactManifold manifold;
    const Math::vec3 farTransformB = shards[1].centroid - direction * 2.0f;
    CHECK(!Narrowphase::collide(hullA, at(shards[0].centroid), hullB, at(farTransformB), manifold));

    const Math::vec3 nearTransformB = shards[1].centroid + direction * 0.3f;
    CHECK(Narrowphase::collide(hullA, at(shards[0].centroid), hullB, at(nearTransformB), manifold));
    CHECK(manifold.count >= 1);
    CHECK(near(Math::length(manifold.normal), 1.0f, 1e-3f));
    CHECK(manifold.points[0].penetration > 0.0f);
    CHECK(std::isfinite(manifold.points[0].penetration));
}

void testConvexHullDegenerateShardIsFinite()
{
    // A thin sliver (glass pane shatter): support/bounds/inertia must not divide by near-zero volume into NaN.
    Shard shard = buildCubeShard(1.0f);
    for (Math::vec3& vertex : shard.vertices)
        vertex.z *= 0.0005f;
    const ConvexHullShape hull(shard);
    const Math::mat4 identity = at(Math::vec3(0.0f));

    const Math::vec3 support = hull.support(identity, Math::vec3(0.3f, 1.0f, 0.2f));
    CHECK(std::isfinite(support.x) && std::isfinite(support.y) && std::isfinite(support.z));

    const AABB bounds = hull.bounds(identity);
    CHECK(std::isfinite(bounds.min.x) && std::isfinite(bounds.max.x));

    const Math::mat3 inertia = hull.inertia(1.0f);
    for (u32 col = 0; col < 3; ++col)
        for (u32 row = 0; row < 3; ++row)
            CHECK(std::isfinite(inertia[col][row]));
}

void testConvexHullConvexHullCoincidentFacesDoNotCrash()
{
    // Zero gap, exactly face to face on every side: must report a collision and terminate.
    const Shard shard = buildCubeShard(1.0f);
    const ConvexHullShape hullA(shard);
    const ConvexHullShape hullB(shard);
    const Math::mat4 transform = at(Math::vec3(0.0f));

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
    testCharacterRigidBodyMaxSlopeAngleZeroDisablesTheCheck();
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
    testWorldRemovalDetachesOneBodyAndKeepsOthersWorking();
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
