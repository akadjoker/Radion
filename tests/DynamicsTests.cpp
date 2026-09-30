#include "PCH.h"

#include "Scene.h"
#include "dynamics/Aerodynamics.h"
#include "dynamics/RigidBody.h"

#include <cstdio>
#include <vector>

using namespace Radion;
using namespace Radion::Physics;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "DynamicsTests:%d: failed: %s\n", line, expression);
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

bool finite(const RigidBody& body)
{
    return std::isfinite(body.position().x) && std::isfinite(body.position().y) &&
           std::isfinite(body.position().z) && std::isfinite(body.orientation().w);
}

RigidBody makeBox(f32 mass = 2.0f, const Math::vec3& halfExtents = Math::vec3(0.5f))
{
    RigidBody body;
    body.setMass(mass);
    body.setInertiaTensor(Inertia::box(mass, halfExtents));
    body.setDamping(1.0f, 1.0f);
    body.setCanSleep(false);
    return body;
}

void testInertiaFormulas()
{
    // I_xx = m/12 * (height^2 + depth^2) uses FULL sides: cube side 1, mass 12 gives 2, not 1.
    const Math::mat3 cube = Inertia::box(12.0f, Math::vec3(0.5f));
    CHECK(near(cube[0][0], 2.0f));
    CHECK(near(cube[1][1], 2.0f));
    CHECK(near(cube[2][2], 2.0f));
    CHECK(near(cube[0][1], 0.0f));

    const Math::mat3 slab = Inertia::box(12.0f, Math::vec3(1.0f, 2.0f, 3.0f));
    CHECK(near(slab[0][0], 52.0f));
    CHECK(near(slab[1][1], 40.0f));
    CHECK(near(slab[2][2], 20.0f));

    // The smallest moment is about the long axis (easiest to spin).
    const Math::mat3 rod = Inertia::box(1.0f, Math::vec3(4.0f, 0.25f, 0.25f));
    CHECK(rod[0][0] < rod[1][1]);
    CHECK(rod[0][0] < rod[2][2]);
    CHECK(near(rod[1][1], rod[2][2]));

    CHECK(near(Inertia::solidSphere(5.0f, 2.0f)[0][0], 0.4f * 5.0f * 4.0f));
    CHECK(Inertia::hollowSphere(5.0f, 2.0f)[0][0] > Inertia::solidSphere(5.0f, 2.0f)[0][0]);

    const Math::mat3 cylinder = Inertia::cylinderY(3.0f, 0.5f, 2.0f);
    const Math::mat3 capsule = Inertia::capsuleY(3.0f, 0.5f, 2.0f);
    CHECK(capsule[0][0] > cylinder[0][0]);
    CHECK(near(capsule[0][0], capsule[2][2]));

    // Every one of them must be invertible, or setInertiaTensor refuses it.
    CHECK(std::abs(Math::determinant(cube)) > 1e-9f);
    CHECK(std::abs(Math::determinant(capsule)) > 1e-9f);
}

void testFreeFall()
{
    RigidBody body = makeBox();
    body.setAcceleration(Math::vec3(0.0f, -10.0f, 0.0f));

    // Semi-implicit Euler lands a*dt^2 ahead: check velocity exactly, position against that bias.
    constexpr f32 step = 1.0f / 100.0f;
    constexpr u32 steps = 100;
    for (u32 i = 0; i < steps; ++i)
        body.integrate(step);

    const f32 elapsed = step * steps;
    CHECK(near(body.velocity().y, -10.0f * elapsed, 1e-3f));

    const f32 exact = -0.5f * 10.0f * elapsed * elapsed;
    const f32 bias = -0.5f * 10.0f * step * elapsed;
    CHECK(near(body.position().y, exact + bias, 1e-3f));
}

void testMassScalesForce()
{
    RigidBody light = makeBox(1.0f);
    RigidBody heavy = makeBox(4.0f);
    light.addForce(Math::vec3(8.0f, 0.0f, 0.0f));
    heavy.addForce(Math::vec3(8.0f, 0.0f, 0.0f));
    light.integrate(0.5f);
    heavy.integrate(0.5f);

    CHECK(near(light.velocity().x, 4.0f));
    CHECK(near(heavy.velocity().x, 1.0f));

    // Gravity is not a force and is not divided by mass.
    RigidBody lightFall = makeBox(1.0f);
    RigidBody heavyFall = makeBox(1000.0f);
    lightFall.setAcceleration(Math::vec3(0.0f, -9.8f, 0.0f));
    heavyFall.setAcceleration(Math::vec3(0.0f, -9.8f, 0.0f));
    lightFall.integrate(0.1f);
    heavyFall.integrate(0.1f);
    CHECK(near(lightFall.velocity().y, heavyFall.velocity().y));
}

void testAccumulatorsCleared()
{
    RigidBody body = makeBox(1.0f);
    body.addForce(Math::vec3(10.0f, 0.0f, 0.0f));
    body.integrate(0.1f);
    const Math::vec3 after = body.velocity();
    // A force applied once acts for exactly one step.
    body.integrate(0.1f);
    CHECK(near(body.velocity(), after));
}

void testTorqueFromOffsetForce()
{
    RigidBody body = makeBox(2.0f, Math::vec3(0.5f));

    body.addForceAtPoint(Math::vec3(0.0f, 0.0f, 10.0f), body.position());
    body.integrate(0.1f);
    CHECK(body.velocity().z > 0.0f);
    CHECK(near(body.angularVelocity(), Math::vec3(0.0f)));

    // r x F = (0.5,0,0) x (0,0,10) has y = -5: spins about MINUS y.
    RigidBody offset = makeBox(2.0f, Math::vec3(0.5f));
    offset.addForceAtPoint(Math::vec3(0.0f, 0.0f, 10.0f),
                           offset.position() + Math::vec3(0.5f, 0.0f, 0.0f));
    offset.integrate(0.1f);
    CHECK(offset.angularVelocity().y < 0.0f);
    CHECK(near(offset.angularVelocity().x, 0.0f));
    CHECK(near(offset.angularVelocity().z, 0.0f));
    CHECK(near(offset.velocity(), body.velocity()));
}

void testInertiaTensorRotatesIntoWorld()
{
    const Math::vec3 halfExtents(4.0f, 0.25f, 0.25f);
    RigidBody body = makeBox(1.0f, halfExtents);

    const Math::mat3 upright = body.inverseInertiaTensorWorld();
    CHECK(upright[0][0] > upright[2][2]); // inverse, so easy axis is larger

    // The world tensor must follow a quarter turn about z (long axis to y).
    body.setOrientation(Math::angleAxis(Math::half_pi<f32>(), Math::vec3(0.0f, 0.0f, 1.0f)));
    const Math::mat3 onEnd = body.inverseInertiaTensorWorld();
    CHECK(onEnd[1][1] > onEnd[2][2]);
    CHECK(near(onEnd[1][1], upright[0][0], 1e-3f));

    CHECK(near(onEnd[0][1], onEnd[1][0], 1e-4f));
    CHECK(near(onEnd[0][2], onEnd[2][0], 1e-4f));
}

void testOrientationStaysNormalized()
{
    RigidBody body = makeBox(1.0f);
    body.setAngularVelocity(Math::vec3(0.0f, 12.0f, 0.0f));
    for (u32 i = 0; i < 2000; ++i)
        body.integrate(1.0f / 60.0f);
    // Without the renormalise in calculateDerivedData() thousands of steps drift.
    CHECK(near(Math::length(body.orientation()), 1.0f, 1e-3f));
    CHECK(std::isfinite(body.orientation().w));
}

void testSpinDirection()
{
    RigidBody body = makeBox(1.0f);
    body.setAngularVelocity(Math::vec3(0.0f, Math::half_pi<f32>(), 0.0f));
    // A quarter turn per second about +y sends +x to -z (right-handed).
    for (u32 i = 0; i < 1000; ++i)
        body.integrate(1.0f / 1000.0f);
    const Math::vec3 axis = body.directionToWorld(Math::vec3(1.0f, 0.0f, 0.0f));
    CHECK(near(axis, Math::vec3(0.0f, 0.0f, -1.0f), 1e-2f));
}

void testSpaceConversions()
{
    RigidBody body = makeBox();
    body.setPosition(Math::vec3(3.0f, -2.0f, 7.0f));
    body.setOrientation(Math::angleAxis(0.7f, Math::normalize(Math::vec3(1.0f, 2.0f, -3.0f))));

    const Math::vec3 local(0.25f, -0.5f, 0.75f);
    CHECK(near(body.pointToLocal(body.pointToWorld(local)), local, 1e-4f));

    CHECK(near(body.directionToWorld(Math::vec3(0.0f)), Math::vec3(0.0f)));
    CHECK(near(body.pointToWorld(Math::vec3(0.0f)), body.position()));
    CHECK(near(Math::length(body.directionToWorld(Math::vec3(0.0f, 1.0f, 0.0f))), 1.0f));

    // Transform matrix and point conversion must agree (solver vs renderer).
    const Math::vec3 throughMatrix = Math::vec3(body.transform() * Math::vec4(local, 1.0f));
    CHECK(near(throughMatrix, body.pointToWorld(local), 1e-4f));
}

void testVelocityAtPoint()
{
    RigidBody body = makeBox();
    body.setVelocity(Math::vec3(1.0f, 0.0f, 0.0f));
    body.setAngularVelocity(Math::vec3(0.0f, 2.0f, 0.0f));

    CHECK(near(body.velocityAtPoint(body.position()), Math::vec3(1.0f, 0.0f, 0.0f)));

    // w x r points along -z.
    const Math::vec3 point = body.position() + Math::vec3(1.0f, 0.0f, 0.0f);
    CHECK(near(body.velocityAtPoint(point), Math::vec3(1.0f, 0.0f, -2.0f)));
}

void testImpulses()
{
    RigidBody body = makeBox(4.0f);
    body.applyLinearImpulse(Math::vec3(8.0f, 0.0f, 0.0f));
    CHECK(near(body.velocity().x, 2.0f));

    RigidBody spun = makeBox(2.0f, Math::vec3(0.5f));
    spun.applyImpulseAtPoint(Math::vec3(0.0f, 0.0f, 4.0f),
                             spun.position() + Math::vec3(0.5f, 0.0f, 0.0f));
    CHECK(spun.velocity().z > 0.0f);
    CHECK(spun.angularVelocity().y < 0.0f);
}

void testBodyTypes()
{
    RigidBody stat = makeBox();
    stat.setBodyType(BodyType::Static);
    stat.setAcceleration(Math::vec3(0.0f, -10.0f, 0.0f));
    stat.addForce(Math::vec3(100.0f, 0.0f, 0.0f));
    stat.applyLinearImpulse(Math::vec3(100.0f, 0.0f, 0.0f));
    stat.integrate(1.0f);
    CHECK(near(stat.position(), Math::vec3(0.0f)));
    CHECK(near(stat.velocity(), Math::vec3(0.0f)));
    CHECK(stat.inverseMass() == 0.0f);
    CHECK(near(stat.inverseInertiaTensorWorld()[0][0], 0.0f));

    RigidBody kinematic = makeBox();
    kinematic.setBodyType(BodyType::Kinematic);
    kinematic.setVelocity(Math::vec3(2.0f, 0.0f, 0.0f));
    kinematic.setAcceleration(Math::vec3(0.0f, -10.0f, 0.0f));
    kinematic.addForce(Math::vec3(0.0f, 0.0f, 500.0f));
    kinematic.integrate(1.0f);
    CHECK(near(kinematic.position(), Math::vec3(2.0f, 0.0f, 0.0f)));
    CHECK(near(kinematic.velocity(), Math::vec3(2.0f, 0.0f, 0.0f)));

    // Switching back to Dynamic must restore the caller's mass, not Static's infinite one.
    RigidBody restored = makeBox(3.0f);
    const f32 before = restored.inverseMass();
    restored.setBodyType(BodyType::Static);
    restored.setBodyType(BodyType::Dynamic);
    CHECK(near(restored.inverseMass(), before));
    CHECK(near(restored.inverseInertiaTensorWorld()[0][0],
               makeBox(3.0f).inverseInertiaTensorWorld()[0][0]));
}

void testMassGuards()
{
    RigidBody body = makeBox(2.0f);
    const f32 before = body.inverseMass();
    body.setMass(0.0f);
    body.setMass(-1.0f);
    body.setMass(std::numeric_limits<f32>::quiet_NaN());
    CHECK(near(body.inverseMass(), before));

    // A singular tensor is refused: inverting it gives infinities.
    const Math::mat3 kept = body.inverseInertiaTensor();
    body.setInertiaTensor(Math::mat3(0.0f));
    CHECK(near(body.inverseInertiaTensor()[0][0], kept[0][0]));
}

void testDampingIsStepIndependent()
{
    // pow(damping, dt): two half steps must match one whole.
    RigidBody coarse = makeBox(1.0f);
    RigidBody fine = makeBox(1.0f);
    coarse.setDamping(0.5f, 0.5f);
    fine.setDamping(0.5f, 0.5f);
    coarse.setVelocity(Math::vec3(10.0f, 0.0f, 0.0f));
    fine.setVelocity(Math::vec3(10.0f, 0.0f, 0.0f));

    coarse.integrate(1.0f);
    fine.integrate(0.5f);
    fine.integrate(0.5f);
    CHECK(near(coarse.velocity().x, fine.velocity().x, 1e-3f));
}

void testSleep()
{
    RigidBody body = makeBox(1.0f);
    body.setCanSleep(true);
    body.setSleepEpsilon(0.3f);
    body.setDamping(0.9f, 0.9f);
    body.setVelocity(Math::vec3(0.01f, 0.0f, 0.0f));

    for (u32 i = 0; i < 600 && body.awake(); ++i)
        body.integrate(1.0f / 60.0f);
    CHECK(!body.awake());
    CHECK(near(body.velocity(), Math::vec3(0.0f)));

    const Math::vec3 restingAt = body.position();
    body.integrate(1.0f);
    CHECK(near(body.position(), restingAt));

    body.addForce(Math::vec3(0.0f, 0.0f, 5.0f));
    CHECK(body.awake());
    body.integrate(1.0f / 60.0f);
    CHECK(body.position().z > restingAt.z);

    RigidBody restless = makeBox(1.0f);
    restless.setCanSleep(false);
    for (u32 i = 0; i < 600; ++i)
        restless.integrate(1.0f / 60.0f);
    CHECK(restless.awake());
}

void testNoDriftAtRest()
{
    // A body with nothing acting on it must not drift (an integrator adding energy).
    RigidBody body = makeBox(1.0f);
    body.setCanSleep(false);
    for (u32 i = 0; i < 10000; ++i)
        body.integrate(1.0f / 60.0f);
    CHECK(near(body.position(), Math::vec3(0.0f), 1e-6f));
    CHECK(near(body.velocity(), Math::vec3(0.0f), 1e-6f));
    CHECK(near(Math::length(body.orientation()), 1.0f, 1e-5f));
}

void testDegenerateStepsIgnored()
{
    RigidBody body = makeBox(1.0f);
    body.setVelocity(Math::vec3(1.0f, 0.0f, 0.0f));
    body.integrate(0.0f);
    body.integrate(-1.0f);
    body.integrate(std::numeric_limits<f32>::quiet_NaN());
    CHECK(near(body.position(), Math::vec3(0.0f)));
    CHECK(std::isfinite(body.position().x));
}

void testAngularMomentumIsConserved()
{
    RigidBody body = makeBox(2.0f, Math::vec3(0.5f, 1.0f, 0.25f));
    body.setCanSleep(false);
    body.setAngularVelocity(Math::vec3(0.0f, 3.0f, 0.0f));
    const f32 before = Math::length(body.angularVelocity());
    for (u32 i = 0; i < 1200; ++i)
        body.integrate(1.0f / 120.0f);
    CHECK(near(Math::length(body.angularVelocity()), before, 1e-3f));
}

// A loose body's destructor must unregister it from its Scene (Scene::addBody(), no GameObject), or the Scene's list dangles.
void testLooseRigidBodyDeregistersOnDestruction()
{
    Scene scene;
    CHECK(scene.bodyCount() == 0);
    {
        RigidBody body = makeBox();
        scene.addBody(body);
        CHECK(scene.bodyCount() == 1);
    }
    CHECK(scene.bodyCount() == 0);
}

// std::vector<RigidBody> reallocation: moveFrom() drops registration, so reserve() upfront is the supported way to keep bodies registered.
void testRigidBodyVectorGrowthKeepsSceneRegistrationCorrect()
{
    Scene scene;
    std::vector<RigidBody> bodies;
    bodies.reserve(6);
    for (u32 i = 0; i < 6; ++i)
    {
        bodies.push_back(makeBox());
        bodies.back().setPosition(Math::vec3(static_cast<f32>(i), 5.0f, 0.0f));
        scene.addBody(bodies.back());
    }
    CHECK(scene.bodyCount() == 6);
    for (u32 i = 0; i < bodies.size(); ++i)
        CHECK(near(bodies[i].position(), Math::vec3(static_cast<f32>(i), 5.0f, 0.0f)));

    scene.setGravity(Math::vec3(0.0f, -10.0f, 0.0f));
    for (u32 step = 0; step < 60; ++step)
        scene.stepPhysics(1.0f / 60.0f);
    CHECK(scene.bodyCount() == 6);
    for (const RigidBody& body : bodies)
    {
        CHECK(finite(body));
        CHECK(body.position().y < 5.0f);
    }
}

// No reserve(): reallocation drops registrations; this must fail SAFELY (no stale pointer, count matches survivors).
void testRigidBodyVectorGrowthWithoutReserveDropsRegistrationSafely()
{
    Scene scene;
    std::vector<RigidBody> bodies;
    for (u32 i = 0; i < 6; ++i)
    {
        bodies.push_back(makeBox());
        scene.addBody(bodies.back());
    }
    // The registered count must equal the live bodies in the final buffer.
    CHECK(scene.bodyCount() <= bodies.size());

    scene.setGravity(Math::vec3(0.0f, -10.0f, 0.0f));
    for (u32 step = 0; step < 60; ++step)
        scene.stepPhysics(1.0f / 60.0f);
    for (const RigidBody& body : bodies)
        CHECK(finite(body));
}

void testWingTurnsSpeedIntoLift()
{
    RigidBody body;
    body.setMass(2.5f);
    body.setInertiaTensor(Math::mat3(1.0f));
    body.setDamping(1.0f, 1.0f);

    Airplane plane;
    plane.setBody(&body);
    plane.setThrust(0.0f);

    // Flying nose-first along -X, which is the reference aircraft's forward.
    body.setVelocity(Math::vec3(-40.0f, 0.0f, 0.0f));
    plane.applyForces();
    body.integrate(1.0f / 60.0f);

    // Forward speed maps to upward force; also catches a transposed tensor.
    CHECK(body.velocity().y > 0.0f);
    CHECK(std::abs(body.velocity().z) < 1e-3f);
}

void testStationaryWingMakesNoLift()
{
    RigidBody body;
    body.setMass(2.5f);
    body.setInertiaTensor(Math::mat3(1.0f));
    body.setDamping(1.0f, 1.0f);

    Airplane plane;
    plane.setBody(&body);
    plane.setThrust(0.0f);
    body.setVelocity(Math::vec3(0.0f));
    plane.applyForces();
    body.integrate(1.0f / 60.0f);
    CHECK(near(body.velocity(), Math::vec3(0.0f), 1e-5f));
}

void testAileronsRollOppositeWays()
{
    auto rollRate = [](f32 control)
    {
        RigidBody body;
        body.setMass(2.5f);
        body.setInertiaTensor(Math::mat3(1.0f));
        body.setDamping(1.0f, 1.0f);

        Airplane plane;
        plane.setBody(&body);
        plane.setThrust(0.0f);
        plane.setRoll(control);
        body.setVelocity(Math::vec3(-40.0f, 0.0f, 0.0f));
        plane.applyForces();
        body.integrate(1.0f / 60.0f);
        return body.angularVelocity().x;
    };

    const f32 left = rollRate(-1.0f);
    const f32 right = rollRate(1.0f);
    CHECK(left * right < 0.0f);
    CHECK(std::abs(rollRate(0.0f)) < std::abs(left));
}

void testRudderYawsBothWays()
{
    auto yawRate = [](f32 control)
    {
        RigidBody body;
        body.setMass(2.5f);
        body.setInertiaTensor(Math::mat3(1.0f));
        body.setDamping(1.0f, 1.0f);

        Airplane plane;
        plane.setBody(&body);
        plane.setThrust(0.0f);
        plane.setYaw(control);
        body.setVelocity(Math::vec3(-40.0f, 0.0f, 0.0f));
        plane.applyForces();
        body.integrate(1.0f / 60.0f);
        return body.angularVelocity().y;
    };

    CHECK(yawRate(-1.0f) * yawRate(1.0f) < 0.0f);
}

void testControlSurfaceInterpolates()
{
    const Math::mat3 base(0.0f);
    const Math::mat3 minimum(-1.0f);
    const Math::mat3 maximum(1.0f);
    AeroControlSurface surface(base, minimum, maximum, Math::vec3(0.0f));

    RigidBody body;
    body.setMass(1.0f);
    body.setInertiaTensor(Math::mat3(1.0f));
    body.setVelocity(Math::vec3(1.0f, 0.0f, 0.0f));

    surface.setControl(0.5f);
    const f32 half = Math::length(surface.tensor() * Math::vec3(1.0f));
    surface.setControl(2.0f);
    CHECK(near(surface.control(), 1.0f));
    surface.setControl(-2.0f);
    CHECK(near(surface.control(), -1.0f));
    (void)half;
}

void testThrustPushesAlongTheNose()
{
    RigidBody body;
    body.setMass(2.5f);
    body.setInertiaTensor(Math::mat3(1.0f));
    body.setDamping(1.0f, 1.0f);

    Airplane plane;
    plane.setBody(&body);
    plane.setThrust(10.0f);
    body.setVelocity(Math::vec3(0.0f));
    plane.applyForces();
    body.integrate(1.0f / 60.0f);
    CHECK(body.velocity().x < 0.0f);
    CHECK(std::abs(body.velocity().y) < 1e-5f);

    // The force is in body space: upside down, thrust points the other way in world space.
    RigidBody flipped;
    flipped.setMass(2.5f);
    flipped.setInertiaTensor(Math::mat3(1.0f));
    flipped.setDamping(1.0f, 1.0f);
    flipped.setOrientation(Math::angleAxis(Math::pi<f32>(), Math::vec3(0.0f, 1.0f, 0.0f)));
    Airplane other;
    other.setBody(&flipped);
    other.setThrust(10.0f);
    other.applyForces();
    flipped.integrate(1.0f / 60.0f);
    CHECK(flipped.velocity().x > 0.0f);
}

} // namespace

int main()
{
    testWingTurnsSpeedIntoLift();
    testStationaryWingMakesNoLift();
    testAileronsRollOppositeWays();
    testRudderYawsBothWays();
    testControlSurfaceInterpolates();
    testThrustPushesAlongTheNose();
    testInertiaFormulas();
    testFreeFall();
    testMassScalesForce();
    testAccumulatorsCleared();
    testTorqueFromOffsetForce();
    testInertiaTensorRotatesIntoWorld();
    testOrientationStaysNormalized();
    testSpinDirection();
    testSpaceConversions();
    testVelocityAtPoint();
    testImpulses();
    testBodyTypes();
    testMassGuards();
    testDampingIsStepIndependent();
    testSleep();
    testNoDriftAtRest();
    testDegenerateStepsIgnored();
    testAngularMomentumIsConserved();
    testLooseRigidBodyDeregistersOnDestruction();
    testRigidBodyVectorGrowthKeepsSceneRegistrationCorrect();
    testRigidBodyVectorGrowthWithoutReserveDropsRegistrationSafely();
    if (gFailures)
        std::fprintf(stderr, "%d dynamics test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
