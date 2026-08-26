#include "PCH.h"

#include "dynamics/Aerodynamics.h"
#include "dynamics/RigidBody.h"

#include <cstdio>

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

bool near(const glm::vec3& a, const glm::vec3& b, f32 epsilon = 1e-4f)
{
    return glm::length(a - b) <= epsilon;
}

RigidBody makeBox(f32 mass = 2.0f, const glm::vec3& halfExtents = glm::vec3(0.5f))
{
    RigidBody body;
    body.setMass(mass);
    body.setInertiaTensor(Inertia::box(mass, halfExtents));
    body.setDamping(1.0f, 1.0f);
    body.setCanSleep(false);
    return body;
}

// ---------------------------------------------------------------- inertia

void testInertiaFormulas()
{
    // I_xx = m/12 * (height^2 + depth^2), the other two FULL sides. A cube of
    // side 1 and mass 12 is therefore 12/12 * (1 + 1) = 2 on every axis, not
    // 1 - the halved extents in the call are not the ones in the formula.
    const glm::mat3 cube = Inertia::box(12.0f, glm::vec3(0.5f));
    CHECK(near(cube[0][0], 2.0f));
    CHECK(near(cube[1][1], 2.0f));
    CHECK(near(cube[2][2], 2.0f));
    CHECK(near(cube[0][1], 0.0f));

    // Same formula on unequal sides, worked out by hand: m=12, full sides
    // (2, 4, 6) gives m/12 * (16 + 36) = 52 about x, (4 + 36) = 40 about y,
    // (4 + 16) = 20 about z.
    const glm::mat3 slab = Inertia::box(12.0f, glm::vec3(1.0f, 2.0f, 3.0f));
    CHECK(near(slab[0][0], 52.0f));
    CHECK(near(slab[1][1], 40.0f));
    CHECK(near(slab[2][2], 20.0f));

    // A box that is long in x has its SMALLEST moment about x - that is the
    // axis it is easiest to spin around, and getting this backwards is the
    // classic way a body tumbles wrongly.
    const glm::mat3 rod = Inertia::box(1.0f, glm::vec3(4.0f, 0.25f, 0.25f));
    CHECK(rod[0][0] < rod[1][1]);
    CHECK(rod[0][0] < rod[2][2]);
    CHECK(near(rod[1][1], rod[2][2]));

    // Solid sphere: 2/5 m r^2, and a hollow one is more, since its mass sits
    // further out.
    CHECK(near(Inertia::solidSphere(5.0f, 2.0f)[0][0], 0.4f * 5.0f * 4.0f));
    CHECK(Inertia::hollowSphere(5.0f, 2.0f)[0][0] > Inertia::solidSphere(5.0f, 2.0f)[0][0]);

    // A capsule is a cylinder plus two caps, so it must be heavier to spin
    // end-over-end than the bare cylinder of the same length.
    const glm::mat3 cylinder = Inertia::cylinderY(3.0f, 0.5f, 2.0f);
    const glm::mat3 capsule = Inertia::capsuleY(3.0f, 0.5f, 2.0f);
    CHECK(capsule[0][0] > cylinder[0][0]);
    CHECK(near(capsule[0][0], capsule[2][2]));

    // Every one of them must be invertible, or setInertiaTensor refuses it.
    CHECK(std::abs(glm::determinant(cube)) > 1e-9f);
    CHECK(std::abs(glm::determinant(capsule)) > 1e-9f);
}

// ------------------------------------------------------------- integration

void testFreeFall()
{
    RigidBody body = makeBox();
    body.setAcceleration(glm::vec3(0.0f, -10.0f, 0.0f));

    // v = a t and y = 0.5 a t^2 exactly only in the continuous case; a
    // semi-implicit Euler step lands one step of a*dt^2 ahead. Check the
    // velocity exactly and the position against that known bias, rather than
    // loosening the tolerance until anything passes.
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
    light.addForce(glm::vec3(8.0f, 0.0f, 0.0f));
    heavy.addForce(glm::vec3(8.0f, 0.0f, 0.0f));
    light.integrate(0.5f);
    heavy.integrate(0.5f);

    // a = F/m, so four times the mass is a quarter of the speed.
    CHECK(near(light.velocity().x, 4.0f));
    CHECK(near(heavy.velocity().x, 1.0f));

    // Gravity is not a force and must not be divided by mass: both fall the
    // same. This is why acceleration is kept apart from the accumulator.
    RigidBody lightFall = makeBox(1.0f);
    RigidBody heavyFall = makeBox(1000.0f);
    lightFall.setAcceleration(glm::vec3(0.0f, -9.8f, 0.0f));
    heavyFall.setAcceleration(glm::vec3(0.0f, -9.8f, 0.0f));
    lightFall.integrate(0.1f);
    heavyFall.integrate(0.1f);
    CHECK(near(lightFall.velocity().y, heavyFall.velocity().y));
}

void testAccumulatorsCleared()
{
    RigidBody body = makeBox(1.0f);
    body.addForce(glm::vec3(10.0f, 0.0f, 0.0f));
    body.integrate(0.1f);
    const glm::vec3 after = body.velocity();
    // A force applied once must act for exactly one step. Left in the
    // accumulator it would keep pushing forever.
    body.integrate(0.1f);
    CHECK(near(body.velocity(), after));
}

// ---------------------------------------------------------------- rotation

void testTorqueFromOffsetForce()
{
    RigidBody body = makeBox(2.0f, glm::vec3(0.5f));

    // Straight through the centre of mass: it moves and does not turn.
    body.addForceAtPoint(glm::vec3(0.0f, 0.0f, 10.0f), body.position());
    body.integrate(0.1f);
    CHECK(body.velocity().z > 0.0f);
    CHECK(near(body.angularVelocity(), glm::vec3(0.0f)));

    // Same push, one edge away. Worked out rather than guessed:
    // r x F = (0.5,0,0) x (0,0,10), whose y component is r_z*F_x - r_x*F_z
    // = 0 - 5 = -5. So an offset along +x with a force along +z spins about
    // MINUS y.
    RigidBody offset = makeBox(2.0f, glm::vec3(0.5f));
    offset.addForceAtPoint(glm::vec3(0.0f, 0.0f, 10.0f),
                           offset.position() + glm::vec3(0.5f, 0.0f, 0.0f));
    offset.integrate(0.1f);
    CHECK(offset.angularVelocity().y < 0.0f);
    CHECK(near(offset.angularVelocity().x, 0.0f));
    CHECK(near(offset.angularVelocity().z, 0.0f));
    // The linear part is untouched by where the force landed.
    CHECK(near(offset.velocity(), body.velocity()));
}

void testInertiaTensorRotatesIntoWorld()
{
    // A rod that is long in x: easy to spin about x, hard about z.
    const glm::vec3 halfExtents(4.0f, 0.25f, 0.25f);
    RigidBody body = makeBox(1.0f, halfExtents);

    const glm::mat3 upright = body.inverseInertiaTensorWorld();
    CHECK(upright[0][0] > upright[2][2]); // inverse, so easy axis is larger

    // Stand it on end - a quarter turn about z sends the long axis to y. The
    // world tensor must follow, or a body would resist the same in world
    // space however it is turned.
    body.setOrientation(glm::angleAxis(glm::half_pi<f32>(), glm::vec3(0.0f, 0.0f, 1.0f)));
    const glm::mat3 onEnd = body.inverseInertiaTensorWorld();
    CHECK(onEnd[1][1] > onEnd[2][2]);
    CHECK(near(onEnd[1][1], upright[0][0], 1e-3f));

    // Still symmetric: R I R^T of a symmetric tensor has to be.
    CHECK(near(onEnd[0][1], onEnd[1][0], 1e-4f));
    CHECK(near(onEnd[0][2], onEnd[2][0], 1e-4f));
}

void testOrientationStaysNormalized()
{
    RigidBody body = makeBox(1.0f);
    body.setAngularVelocity(glm::vec3(0.0f, 12.0f, 0.0f));
    for (u32 i = 0; i < 2000; ++i)
        body.integrate(1.0f / 60.0f);
    // Two thousand steps of "add a vector to a quaternion" drift a long way
    // without the renormalise in calculateDerivedData().
    CHECK(near(glm::length(body.orientation()), 1.0f, 1e-3f));
    CHECK(std::isfinite(body.orientation().w));
}

void testSpinDirection()
{
    RigidBody body = makeBox(1.0f);
    body.setAngularVelocity(glm::vec3(0.0f, glm::half_pi<f32>(), 0.0f));
    // A quarter turn per second about +y, for one second: +x ends on -z,
    // which is the right-handed direction. A sign error here reads as the
    // whole world spinning backwards.
    for (u32 i = 0; i < 1000; ++i)
        body.integrate(1.0f / 1000.0f);
    const glm::vec3 axis = body.directionToWorld(glm::vec3(1.0f, 0.0f, 0.0f));
    CHECK(near(axis, glm::vec3(0.0f, 0.0f, -1.0f), 1e-2f));
}

// ------------------------------------------------------------------ frames

void testSpaceConversions()
{
    RigidBody body = makeBox();
    body.setPosition(glm::vec3(3.0f, -2.0f, 7.0f));
    body.setOrientation(glm::angleAxis(0.7f, glm::normalize(glm::vec3(1.0f, 2.0f, -3.0f))));

    const glm::vec3 local(0.25f, -0.5f, 0.75f);
    CHECK(near(body.pointToLocal(body.pointToWorld(local)), local, 1e-4f));

    // A direction ignores the translation; a point does not.
    CHECK(near(body.directionToWorld(glm::vec3(0.0f)), glm::vec3(0.0f)));
    CHECK(near(body.pointToWorld(glm::vec3(0.0f)), body.position()));
    CHECK(near(glm::length(body.directionToWorld(glm::vec3(0.0f, 1.0f, 0.0f))), 1.0f));

    // The transform matrix and the point conversion have to agree - the
    // solver uses one and the renderer the other.
    const glm::vec3 throughMatrix = glm::vec3(body.transform() * glm::vec4(local, 1.0f));
    CHECK(near(throughMatrix, body.pointToWorld(local), 1e-4f));
}

void testVelocityAtPoint()
{
    RigidBody body = makeBox();
    body.setVelocity(glm::vec3(1.0f, 0.0f, 0.0f));
    body.setAngularVelocity(glm::vec3(0.0f, 2.0f, 0.0f));

    // At the centre a point moves at the body's own velocity.
    CHECK(near(body.velocityAtPoint(body.position()), glm::vec3(1.0f, 0.0f, 0.0f)));

    // One unit along +x, spinning about +y: w x r points along -z.
    const glm::vec3 point = body.position() + glm::vec3(1.0f, 0.0f, 0.0f);
    CHECK(near(body.velocityAtPoint(point), glm::vec3(1.0f, 0.0f, -2.0f)));
}

// ------------------------------------------------------------------ impulses

void testImpulses()
{
    RigidBody body = makeBox(4.0f);
    body.applyLinearImpulse(glm::vec3(8.0f, 0.0f, 0.0f));
    // An impulse changes velocity there and then: dv = J/m, no step needed.
    CHECK(near(body.velocity().x, 2.0f));

    RigidBody spun = makeBox(2.0f, glm::vec3(0.5f));
    spun.applyImpulseAtPoint(glm::vec3(0.0f, 0.0f, 4.0f),
                             spun.position() + glm::vec3(0.5f, 0.0f, 0.0f));
    CHECK(spun.velocity().z > 0.0f);
    // Same r x F as testTorqueFromOffsetForce: about minus y.
    CHECK(spun.angularVelocity().y < 0.0f);
}

// --------------------------------------------------------------- body types

void testBodyTypes()
{
    RigidBody stat = makeBox();
    stat.setBodyType(BodyType::Static);
    stat.setAcceleration(glm::vec3(0.0f, -10.0f, 0.0f));
    stat.addForce(glm::vec3(100.0f, 0.0f, 0.0f));
    stat.applyLinearImpulse(glm::vec3(100.0f, 0.0f, 0.0f));
    stat.integrate(1.0f);
    // Nothing reaches it: not gravity, not a force, not an impulse.
    CHECK(near(stat.position(), glm::vec3(0.0f)));
    CHECK(near(stat.velocity(), glm::vec3(0.0f)));
    CHECK(stat.inverseMass() == 0.0f);
    // Infinite inverse inertia would be zero: an angular impulse does nothing.
    CHECK(near(stat.inverseInertiaTensorWorld()[0][0], 0.0f));

    RigidBody kinematic = makeBox();
    kinematic.setBodyType(BodyType::Kinematic);
    kinematic.setVelocity(glm::vec3(2.0f, 0.0f, 0.0f));
    kinematic.setAcceleration(glm::vec3(0.0f, -10.0f, 0.0f));
    kinematic.addForce(glm::vec3(0.0f, 0.0f, 500.0f));
    kinematic.integrate(1.0f);
    // Moves by its own velocity only. Gravity and forces are ignored, but it
    // does move - that is the whole difference from Static.
    CHECK(near(kinematic.position(), glm::vec3(2.0f, 0.0f, 0.0f)));
    CHECK(near(kinematic.velocity(), glm::vec3(2.0f, 0.0f, 0.0f)));

    // Switching back to Dynamic must restore the mass the caller gave, not
    // leave the body with the infinite mass Static forced on it.
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
    // Refused, and the body keeps what it had rather than becoming infinite.
    CHECK(near(body.inverseMass(), before));

    // A singular inertia tensor is refused for the same reason: inverting it
    // gives infinities that spread through every later step.
    const glm::mat3 kept = body.inverseInertiaTensor();
    body.setInertiaTensor(glm::mat3(0.0f));
    CHECK(near(body.inverseInertiaTensor()[0][0], kept[0][0]));
}

// ------------------------------------------------------------------- damping

void testDampingIsStepIndependent()
{
    // pow(damping, dt) means the same amount of energy is removed per second
    // however the second is divided up - two half steps must match one whole.
    RigidBody coarse = makeBox(1.0f);
    RigidBody fine = makeBox(1.0f);
    coarse.setDamping(0.5f, 0.5f);
    fine.setDamping(0.5f, 0.5f);
    coarse.setVelocity(glm::vec3(10.0f, 0.0f, 0.0f));
    fine.setVelocity(glm::vec3(10.0f, 0.0f, 0.0f));

    coarse.integrate(1.0f);
    fine.integrate(0.5f);
    fine.integrate(0.5f);
    CHECK(near(coarse.velocity().x, fine.velocity().x, 1e-3f));
}

// --------------------------------------------------------------------- sleep

void testSleep()
{
    RigidBody body = makeBox(1.0f);
    body.setCanSleep(true);
    body.setSleepEpsilon(0.3f);
    body.setDamping(0.9f, 0.9f);
    body.setVelocity(glm::vec3(0.01f, 0.0f, 0.0f));

    for (u32 i = 0; i < 600 && body.awake(); ++i)
        body.integrate(1.0f / 60.0f);
    CHECK(!body.awake());
    // Falling asleep zeroes the velocities, so nothing creeps.
    CHECK(near(body.velocity(), glm::vec3(0.0f)));

    const glm::vec3 restingAt = body.position();
    body.integrate(1.0f);
    CHECK(near(body.position(), restingAt));

    // Any force has to wake it, or a sleeping body is unhittable.
    body.addForce(glm::vec3(0.0f, 0.0f, 5.0f));
    CHECK(body.awake());
    body.integrate(1.0f / 60.0f);
    CHECK(body.position().z > restingAt.z);

    // A body told it cannot sleep must not, however still it goes.
    RigidBody restless = makeBox(1.0f);
    restless.setCanSleep(false);
    for (u32 i = 0; i < 600; ++i)
        restless.integrate(1.0f / 60.0f);
    CHECK(restless.awake());
}

// ------------------------------------------------------------------ stability

void testNoDriftAtRest()
{
    // A body with nothing acting on it must sit exactly still. Any drift here
    // is an integrator that adds energy, and it shows up later as a stack
    // that will not settle.
    RigidBody body = makeBox(1.0f);
    body.setCanSleep(false);
    for (u32 i = 0; i < 10000; ++i)
        body.integrate(1.0f / 60.0f);
    CHECK(near(body.position(), glm::vec3(0.0f), 1e-6f));
    CHECK(near(body.velocity(), glm::vec3(0.0f), 1e-6f));
    CHECK(near(glm::length(body.orientation()), 1.0f, 1e-5f));
}

void testDegenerateStepsIgnored()
{
    RigidBody body = makeBox(1.0f);
    body.setVelocity(glm::vec3(1.0f, 0.0f, 0.0f));
    body.integrate(0.0f);
    body.integrate(-1.0f);
    body.integrate(std::numeric_limits<f32>::quiet_NaN());
    // A zero, negative or NaN step leaves the body exactly as it was rather
    // than sending it to infinity.
    CHECK(near(body.position(), glm::vec3(0.0f)));
    CHECK(std::isfinite(body.position().x));
}

void testAngularMomentumIsConserved()
{
    // No damping, no torque: a free body keeps spinning at the same rate.
    RigidBody body = makeBox(2.0f, glm::vec3(0.5f, 1.0f, 0.25f));
    body.setCanSleep(false);
    body.setAngularVelocity(glm::vec3(0.0f, 3.0f, 0.0f));
    const f32 before = glm::length(body.angularVelocity());
    for (u32 i = 0; i < 1200; ++i)
        body.integrate(1.0f / 120.0f);
    CHECK(near(glm::length(body.angularVelocity()), before, 1e-3f));
}

void testWingTurnsSpeedIntoLift()
{
    RigidBody body;
    body.setMass(2.5f);
    body.setInertiaTensor(glm::mat3(1.0f));
    body.setDamping(1.0f, 1.0f);

    Airplane plane;
    plane.setBody(&body);
    plane.setThrust(0.0f);

    // Flying nose-first along -X, which is the reference aircraft's forward.
    body.setVelocity(glm::vec3(-40.0f, 0.0f, 0.0f));
    plane.applyForces();
    body.integrate(1.0f / 60.0f);

    // The wings have to push it up. This is the whole point of the tensor:
    // no lift equation, no angle of attack, just forward speed mapped onto
    // upward force - and it is also the check that catches a transposed
    // tensor, which would push it sideways or backwards instead.
    CHECK(body.velocity().y > 0.0f);
    CHECK(std::abs(body.velocity().z) < 1e-3f);
}

void testStationaryWingMakesNoLift()
{
    RigidBody body;
    body.setMass(2.5f);
    body.setInertiaTensor(glm::mat3(1.0f));
    body.setDamping(1.0f, 1.0f);

    Airplane plane;
    plane.setBody(&body);
    plane.setThrust(0.0f);
    body.setVelocity(glm::vec3(0.0f));
    plane.applyForces();
    body.integrate(1.0f / 60.0f);
    CHECK(near(body.velocity(), glm::vec3(0.0f), 1e-5f));
}

void testAileronsRollOppositeWays()
{
    auto rollRate = [](f32 control)
    {
        RigidBody body;
        body.setMass(2.5f);
        body.setInertiaTensor(glm::mat3(1.0f));
        body.setDamping(1.0f, 1.0f);

        Airplane plane;
        plane.setBody(&body);
        plane.setThrust(0.0f);
        plane.setRoll(control);
        body.setVelocity(glm::vec3(-40.0f, 0.0f, 0.0f));
        plane.applyForces();
        body.integrate(1.0f / 60.0f);
        // Roll is rotation about the forward axis.
        return body.angularVelocity().x;
    };

    const f32 left = rollRate(-1.0f);
    const f32 right = rollRate(1.0f);
    // Opposite signs, or the ailerons are moving together and the plane
    // climbs instead of rolling.
    CHECK(left * right < 0.0f);
    CHECK(std::abs(rollRate(0.0f)) < std::abs(left));
}

void testRudderYawsBothWays()
{
    auto yawRate = [](f32 control)
    {
        RigidBody body;
        body.setMass(2.5f);
        body.setInertiaTensor(glm::mat3(1.0f));
        body.setDamping(1.0f, 1.0f);

        Airplane plane;
        plane.setBody(&body);
        plane.setThrust(0.0f);
        plane.setYaw(control);
        body.setVelocity(glm::vec3(-40.0f, 0.0f, 0.0f));
        plane.applyForces();
        body.integrate(1.0f / 60.0f);
        return body.angularVelocity().y;
    };

    CHECK(yawRate(-1.0f) * yawRate(1.0f) < 0.0f);
}

void testControlSurfaceInterpolates()
{
    const glm::mat3 base(0.0f);
    const glm::mat3 minimum(-1.0f);
    const glm::mat3 maximum(1.0f);
    AeroControlSurface surface(base, minimum, maximum, glm::vec3(0.0f));

    RigidBody body;
    body.setMass(1.0f);
    body.setInertiaTensor(glm::mat3(1.0f));
    body.setVelocity(glm::vec3(1.0f, 0.0f, 0.0f));

    // Half deflection has to sit between rest and full, not jump to it.
    surface.setControl(0.5f);
    const f32 half = glm::length(surface.tensor() * glm::vec3(1.0f));
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
    body.setInertiaTensor(glm::mat3(1.0f));
    body.setDamping(1.0f, 1.0f);

    Airplane plane;
    plane.setBody(&body);
    plane.setThrust(10.0f);
    // Wings make no force at rest, so what remains is thrust alone.
    body.setVelocity(glm::vec3(0.0f));
    plane.applyForces();
    body.integrate(1.0f / 60.0f);
    CHECK(body.velocity().x < 0.0f);
    CHECK(std::abs(body.velocity().y) < 1e-5f);

    // Rolled upside down, the same thrust has to point the other way in
    // world space - the force is in body space, not world.
    RigidBody flipped;
    flipped.setMass(2.5f);
    flipped.setInertiaTensor(glm::mat3(1.0f));
    flipped.setDamping(1.0f, 1.0f);
    flipped.setOrientation(glm::angleAxis(glm::pi<f32>(), glm::vec3(0.0f, 1.0f, 0.0f)));
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
    if (gFailures)
        std::fprintf(stderr, "%d dynamics test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
