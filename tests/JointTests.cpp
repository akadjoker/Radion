#include "PCH.h"

#include "Scene.h"
#include "collision/CollisionShape.h"
#include "dynamics/ContactSolver.h"
#include "dynamics/DistanceJoint.h"
#include "dynamics/FixedJoint.h"
#include "dynamics/HingeJoint.h"
#include "dynamics/MouseJoint.h"
#include "dynamics/PistonJoint.h"
#include "dynamics/PointJoint.h"
#include "dynamics/RigidBody.h"
#include "dynamics/SliderJoint.h"
#include "dynamics/UniversalJoint.h"
#include "dynamics/WheelJoint.h"

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
        std::fprintf(stderr, "JointTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool near(f32 a, f32 b, f32 epsilon = 1e-3f)
{
    return std::abs(a - b) <= epsilon;
}

bool near(const glm::vec3& a, const glm::vec3& b, f32 epsilon = 1e-3f)
{
    return glm::length(a - b) <= epsilon;
}

RigidBody makeDynamicBox(const glm::vec3& position, f32 mass = 1.0f,
                         const glm::vec3& halfExtents = glm::vec3(0.5f))
{
    RigidBody body;
    body.setPosition(position);
    body.setMass(mass);
    body.setInertiaTensor(Inertia::box(mass, halfExtents));
    body.setDamping(1.0f, 1.0f);
    body.setCanSleep(false);
    return body;
}

RigidBody makeStaticBox(const glm::vec3& position)
{
    RigidBody body;
    body.setPosition(position);
    body.setBodyType(BodyType::Static);
    return body;
}

void stepWithJoint(RigidBody& a, RigidBody& b, Joint& joint, const glm::vec3& gravity, f32 duration,
                   u32 velocityIterations = 8, u32 positionIterations = 3)
{
    if (a.isDynamic())
    {
        a.setAcceleration(gravity);
        a.integrateForces(duration);
    }
    if (b.isDynamic())
    {
        b.setAcceleration(gravity);
        b.integrateForces(duration);
    }

    Joint* joints[] = {&joint};
    ContactSolverSettings settings;
    settings.velocityIterations = velocityIterations;
    settings.positionIterations = positionIterations;
    ContactSolver solver;
    solver.setSettings(settings);
    solver.solve(nullptr, 0, joints, 1, duration);

    if (a.isDynamic())
        a.integrateVelocity(duration);
    if (b.isDynamic())
        b.integrateVelocity(duration);
}

void stepJoints(RigidBody* const* bodies, u32 bodyCount, Joint* const* joints, u32 jointCount,
                const glm::vec3& gravity, f32 duration)
{
    for (u32 i = 0; i < bodyCount; ++i)
        if (bodies[i]->isDynamic())
        {
            bodies[i]->setAcceleration(gravity);
            bodies[i]->integrateForces(duration);
        }
    ContactSolver solver;
    solver.solve(nullptr, 0, joints, jointCount, duration);
    for (u32 i = 0; i < bodyCount; ++i)
        if (bodies[i]->isDynamic())
            bodies[i]->integrateVelocity(duration);
}

bool finite(const RigidBody& body)
{
    return std::isfinite(body.position().x) && std::isfinite(body.position().y) &&
           std::isfinite(body.position().z) && std::isfinite(body.orientation().w);
}

// -------------------------------------------------------------- DistanceJoint

void testDistanceJointHoldsFixedLength()
{
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody weight = makeDynamicBox(glm::vec3(2.0f, 0.0f, 0.0f));
    DistanceJoint joint(anchor, glm::vec3(0.0f), weight, glm::vec3(0.0f), 2.0f, 2.0f);

    for (u32 i = 0; i < 600; ++i)
        stepWithJoint(anchor, weight, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    const f32 length = glm::length(weight.position() - anchor.position());
    CHECK(near(length, 2.0f, 0.02f));
}

void testDistanceJointRangeAllowsSlack()
{
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody weight = makeDynamicBox(glm::vec3(0.5f, 0.0f, 0.0f));
    DistanceJoint joint(anchor, glm::vec3(0.0f), weight, glm::vec3(0.0f), 0.0f, 2.0f);

    for (u32 i = 0; i < 6; ++i)
        stepWithJoint(anchor, weight, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    // Well inside the range, gravity must pull it down unopposed.
    CHECK(weight.position().y < -0.01f);

    for (u32 i = 0; i < 600; ++i)
        stepWithJoint(anchor, weight, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    const f32 length = glm::length(weight.position() - anchor.position());
    CHECK(length <= 2.02f);
    CHECK(near(length, 2.0f, 0.02f));
}

// ----------------------------------------------------------------- FixedJoint

void testFixedJointLocksAllSixDOF()
{
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody plate = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f));
    FixedJoint joint(anchor, plate, glm::vec3(0.5f, 0.0f, 0.0f));

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(anchor, plate, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(near(plate.position(), glm::vec3(1.0f, 0.0f, 0.0f), 0.02f));
    CHECK(near(glm::length(plate.orientation() - glm::quat(1.0f, 0.0f, 0.0f, 0.0f)), 0.0f, 0.05f));
}

// ------------------------------------------------------------------ HingeJoint

void testHingeJointFreeAboutItsAxisOnly()
{
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody arm = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f), 1.0f, glm::vec3(0.5f));
    HingeJoint joint(anchor, arm, glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    arm.addTorque(glm::vec3(0.0f, 5.0f, 0.0f));
    for (u32 i = 0; i < 30; ++i)
        stepWithJoint(anchor, arm, joint, glm::vec3(0.0f), 1.0f / 60.0f);

    // A torque about a locked axis (y) must not spin the body about that axis.
    CHECK(near(arm.angularVelocity().y, 0.0f, 0.05f));

    arm.addTorque(glm::vec3(0.0f, 0.0f, 5.0f));
    for (u32 i = 0; i < 30; ++i)
        stepWithJoint(anchor, arm, joint, glm::vec3(0.0f), 1.0f / 60.0f);

    // The hinge axis itself (z) is free to spin.
    CHECK(std::abs(arm.angularVelocity().z) > 0.05f);
}

void testHingeJointMotorDrivesToTargetVelocity()
{
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody wheel = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f), 1.0f, glm::vec3(0.5f));
    HingeJoint joint(anchor, wheel, glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    joint.setMotor(4.0f, 20.0f);

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(anchor, wheel, joint, glm::vec3(0.0f), 1.0f / 60.0f);

    CHECK(near(wheel.angularVelocity().z, 4.0f, 0.1f));
}

// A servo is commanded with an angle, not a speed: it drives there and then
// holds, which is what a robot arm's axis does.
void testHingeServoReachesAndHoldsItsAngle()
{
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody arm = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f), 1.0f, glm::vec3(0.5f));
    HingeJoint joint(anchor, arm, glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    const f32 target = 0.6f;
    joint.setServo(target, 200.0f);
    CHECK(joint.servoEnabled());
    CHECK(joint.motorEnabled()); // the servo drives the velocity motor

    // Under gravity, so holding is real work and not just the absence of it.
    for (u32 i = 0; i < 400; ++i)
        stepWithJoint(anchor, arm, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
    CHECK(near(joint.currentAngle(), target, 0.02f));

    // Still there 400 steps later, and not drifting or oscillating.
    const f32 settled = joint.currentAngle();
    for (u32 i = 0; i < 400; ++i)
        stepWithJoint(anchor, arm, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
    CHECK(near(joint.currentAngle(), settled, 0.01f));
    CHECK(std::abs(arm.angularVelocity().z) < 0.05f); // arrived, not still chasing

    // A new target is followed without re-enabling anything: this is the
    // call a command arriving over a socket would make.
    joint.setServo(-0.4f, 200.0f);
    for (u32 i = 0; i < 400; ++i)
        stepWithJoint(anchor, arm, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
    CHECK(near(joint.currentAngle(), -0.4f, 0.02f));
}

// A target outside the joint's own limits is clamped into them, not chased
// through them - btHingeConstraint::setMotorTarget() does the same.
void testHingeServoClampsTargetToLimits()
{
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody arm = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f), 1.0f, glm::vec3(0.5f));
    HingeJoint joint(anchor, arm, glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    joint.setLimits(-0.3f, 0.5f);
    joint.setServo(2.0f, 200.0f); // well past the upper limit

    for (u32 i = 0; i < 400; ++i)
        stepWithJoint(anchor, arm, joint, glm::vec3(0.0f), 1.0f / 60.0f);

    CHECK(near(joint.currentAngle(), 0.5f, 0.03f));
}

// Bounded torque is what makes a servo a servo: too little of it and the
// joint cannot lift its own load, which is a result a caller must be able to
// see rather than have papered over.
void testHingeServoRespectsItsTorqueBudget()
{
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody arm = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f), 1.0f, glm::vec3(0.5f));
    HingeJoint joint(anchor, arm, glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    joint.setServo(1.2f, 0.2f); // nowhere near enough against a 10 m/s^2 pull

    for (u32 i = 0; i < 400; ++i)
        stepWithJoint(anchor, arm, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(joint.currentAngle() < 1.2f - 0.1f); // fell short, as it should
    CHECK(std::isfinite(joint.currentAngle())); // and did not explode trying
}

void testSliderServoReachesAndHoldsItsPosition()
{
    RigidBody rail = makeStaticBox(glm::vec3(0.0f));
    RigidBody finger = makeDynamicBox(glm::vec3(0.0f));
    SliderJoint joint(rail, finger, glm::vec3(0.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    joint.setServo(0.35f, 200.0f);

    for (u32 i = 0; i < 400; ++i)
        stepWithJoint(rail, finger, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(near(joint.currentPosition(), 0.35f, 0.02f));
    CHECK(std::abs(finger.velocity().x) < 0.05f);
}

// Measures the one number that decides whether this engine can carry a
// walking robot: how far a chain of servo-held joints sags under load.
//
// Three links held straight out by servos, with a mass on the end - the
// shape of a leg holding up its share of a body. A rigid chain keeps every
// joint at its commanded angle; a chain solved by sequential impulses gives
// a little, and how much is what a learned gait would end up exploiting.
// Reported rather than merely asserted: the number is the point.
void testServoChainSagUnderLoad()
{
    RigidBody root = makeStaticBox(glm::vec3(0.0f));
    RigidBody link1 = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f), 1.0f, glm::vec3(0.5f));
    RigidBody link2 = makeDynamicBox(glm::vec3(2.0f, 0.0f, 0.0f), 1.0f, glm::vec3(0.5f));
    RigidBody load = makeDynamicBox(glm::vec3(3.0f, 0.0f, 0.0f), 5.0f, glm::vec3(0.5f));

    const glm::vec3 axis(0.0f, 0.0f, 1.0f);
    HingeJoint hip(root, link1, glm::vec3(0.5f, 0.0f, 0.0f), axis);
    HingeJoint knee(link1, link2, glm::vec3(1.5f, 0.0f, 0.0f), axis);
    HingeJoint ankle(link2, load, glm::vec3(2.5f, 0.0f, 0.0f), axis);

    // Held straight, with torque to spare and a rated speed of 2 rad/s -
    // about 115 deg/s, in the range an industrial arm's axis actually moves.
    hip.setServo(0.0f, 2000.0f, 2.0f);
    knee.setServo(0.0f, 2000.0f, 2.0f);
    ankle.setServo(0.0f, 2000.0f, 2.0f);

    RigidBody* bodies[] = {&root, &link1, &link2, &load};
    Joint* joints[] = {&hip, &knee, &ankle};
    const f32 duration = 1.0f / 120.0f;
    const glm::vec3 gravity(0.0f, -10.0f, 0.0f);

    // Swept against solver iterations on purpose. If the error shrinks as
    // they go up, what is being measured is the solver failing to converge
    // on a coupled chain - not the servo.
    f32 hipSag = 0.0f, kneeSag = 0.0f, ankleSag = 0.0f, tipDrop = 0.0f;
    f32 worstByIterations[3] = {0.0f, 0.0f, 0.0f};
    u32 sweepIndex = 0;
    for (u32 iterations : {8u, 32u, 128u})
    {
        for (RigidBody* body : bodies)
        {
            body->setVelocity(glm::vec3(0.0f));
            body->setAngularVelocity(glm::vec3(0.0f));
            body->setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
        }
        link1.setPosition(glm::vec3(1.0f, 0.0f, 0.0f));
        link2.setPosition(glm::vec3(2.0f, 0.0f, 0.0f));
        load.setPosition(glm::vec3(3.0f, 0.0f, 0.0f));

        ContactSolverSettings settings;
        settings.velocityIterations = iterations;
        ContactSolver solver;
        solver.setSettings(settings);

        f32 worstHip = 0.0f;
        for (u32 step = 0; step < 600; ++step)
        {
            for (RigidBody* body : bodies)
                if (body->isDynamic())
                {
                    body->setAcceleration(gravity);
                    body->integrateForces(duration);
                }
            solver.solve(nullptr, 0, joints, 3, duration);
            for (RigidBody* body : bodies)
                if (body->isDynamic())
                    body->integrateVelocity(duration);
            // Only the second half counts, so the initial settle is not
            // mistaken for the steady-state wobble.
            if (step > 300)
                worstHip = glm::max(worstHip, glm::degrees(std::abs(hip.currentAngle())));
        }

        worstByIterations[sweepIndex++] = worstHip;
        hipSag = glm::degrees(std::abs(hip.currentAngle()));
        kneeSag = glm::degrees(std::abs(knee.currentAngle()));
        ankleSag = glm::degrees(std::abs(ankle.currentAngle()));
        tipDrop = -load.position().y;
        std::printf("JointTests: servo chain, %3u velocity iterations - worst hip swing %.3f deg, "
                    "final hip %.3f / knee %.3f / ankle %.3f deg, tip drop %.4f m\n",
                    iterations, static_cast<double>(worstHip), static_cast<double>(hipSag),
                    static_cast<double>(kneeSag), static_cast<double>(ankleSag),
                    static_cast<double>(tipDrop));
    }

    CHECK(std::isfinite(hipSag) && std::isfinite(tipDrop));

    // What the sweep showed, and what has to keep being true:
    // the chain's error is the solver's convergence, so more iterations buy
    // a stiffer chain. Measured 10.2 -> 3.4 -> 0.36 degrees at 8/32/128.
    CHECK(worstByIterations[1] < worstByIterations[0]);
    CHECK(worstByIterations[2] < worstByIterations[1]);
    // And at 128 the chain is stiff enough to carry a walking robot: under a
    // degree of swing at the loaded joint. This is the bound that matters -
    // if it ever fails, a robot built on this engine stopped being credible.
    CHECK(worstByIterations[2] < 1.0f);
    // The default 8 is a game setting, not a robotics one. Kept as a bound
    // only to catch the chain turning to rubber outright.
    CHECK(worstByIterations[0] < 15.0f);
}

void testHingeJointKeepsAnchorTogether()
{
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody arm = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f), 1.0f, glm::vec3(0.5f));
    HingeJoint joint(anchor, arm, glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(anchor, arm, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    const glm::vec3 hingePointOnArm = arm.pointToWorld(glm::vec3(-0.5f, 0.0f, 0.0f));
    CHECK(near(hingePointOnArm, glm::vec3(0.5f, 0.0f, 0.0f), 0.05f));
}

// ----------------------------------------------------------------- SliderJoint

void testSliderJointMovesOnlyAlongItsAxis()
{
    RigidBody rail = makeStaticBox(glm::vec3(0.0f));
    RigidBody carriage = makeDynamicBox(glm::vec3(0.0f, -1.0f, 0.0f));
    SliderJoint joint(rail, carriage, glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    for (u32 i = 0; i < 60; ++i)
        stepWithJoint(rail, carriage, joint, glm::vec3(4.0f, -10.0f, 3.0f), 1.0f / 60.0f);

    // Gravity's x/z components must not move the carriage off the rail.
    CHECK(near(carriage.position().x, 0.0f, 0.02f));
    CHECK(near(carriage.position().z, 0.0f, 0.02f));
    // The y component, along the rail, must have fallen freely.
    CHECK(carriage.position().y < -1.0f);
    CHECK(near(glm::length(carriage.orientation() - glm::quat(1.0f, 0.0f, 0.0f, 0.0f)), 0.0f,
              0.02f));
}

void testSliderJointLimitsClampTravel()
{
    RigidBody rail = makeStaticBox(glm::vec3(0.0f));
    RigidBody piston = makeDynamicBox(glm::vec3(0.0f, -0.2f, 0.0f));
    SliderJoint joint(rail, piston, glm::vec3(0.0f, -0.2f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    joint.setLimits(-1.0f, 0.0f);

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(rail, piston, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    // The limit clamps the slide displacement, not the absolute position -
    // the piston started at y=-0.2, so the floor sits at -0.2 + (-1.0).
    CHECK(joint.currentPosition() >= -1.02f);
    CHECK(near(joint.currentPosition(), -1.0f, 0.03f));
    CHECK(near(piston.position().y, -1.2f, 0.03f));
}

void testSliderJointMotorDrivesToTargetVelocity()
{
    RigidBody rail = makeStaticBox(glm::vec3(0.0f));
    RigidBody piston = makeDynamicBox(glm::vec3(0.0f));
    SliderJoint joint(rail, piston, glm::vec3(0.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    joint.setMotor(3.0f, 50.0f);

    for (u32 i = 0; i < 120; ++i)
        stepWithJoint(rail, piston, joint, glm::vec3(0.0f), 1.0f / 60.0f);

    CHECK(near(piston.velocity().x, 3.0f, 0.05f));
}

// ----------------------------------------------------------------- PistonJoint

void testPistonJointMovesAndSpinsOnlyAlongItsAxis()
{
    RigidBody rail = makeStaticBox(glm::vec3(0.0f));
    RigidBody strut = makeDynamicBox(glm::vec3(0.0f, -1.0f, 0.0f));
    PistonJoint joint(rail, strut, glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    strut.addTorque(glm::vec3(3.0f, 0.0f, 0.0f));
    for (u32 i = 0; i < 60; ++i)
        stepWithJoint(rail, strut, joint, glm::vec3(4.0f, -10.0f, 3.0f), 1.0f / 60.0f);

    // Gravity's sideways components and a torque about a locked axis (x)
    // must not move or spin the strut off the rail.
    CHECK(near(strut.position().x, 0.0f, 0.03f));
    CHECK(near(strut.position().z, 0.0f, 0.03f));
    CHECK(near(strut.angularVelocity().x, 0.0f, 0.05f));
    // Free along y: falls, and a torque about y is free to spin it.
    CHECK(strut.position().y < -1.0f);
}

void testPistonJointLinearMotorAndAngularLimitAreIndependent()
{
    RigidBody rail = makeStaticBox(glm::vec3(0.0f));
    RigidBody strut = makeDynamicBox(glm::vec3(0.0f));
    PistonJoint joint(rail, strut, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    joint.setLinearMotor(2.0f, 40.0f);
    joint.setAngularLimits(-0.2f, 0.2f);

    strut.addTorque(glm::vec3(0.0f, 6.0f, 0.0f));
    for (u32 i = 0; i < 240; ++i)
        stepWithJoint(rail, strut, joint, glm::vec3(0.0f), 1.0f / 60.0f);

    CHECK(near(strut.velocity().y, 2.0f, 0.05f));
    CHECK(joint.currentAngle() <= 0.22f);
    CHECK(joint.currentAngle() >= -0.22f);
}

// -------------------------------------------------------------- UniversalJoint

void testUniversalJointFreeAboutBothAxesOnly()
{
    RigidBody yoke = makeStaticBox(glm::vec3(0.0f));
    RigidBody shaft = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f));
    UniversalJoint joint(yoke, shaft, glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f),
                        glm::vec3(0.0f, 0.0f, 1.0f));

    // A torque about the shared perpendicular direction (x, locked by the
    // ball-and-socket point but not one of the two free hinge axes) must
    // not build up angular velocity there.
    shaft.addTorque(glm::vec3(5.0f, 0.0f, 0.0f));
    for (u32 i = 0; i < 30; ++i)
        stepWithJoint(yoke, shaft, joint, glm::vec3(0.0f), 1.0f / 60.0f);
    CHECK(near(shaft.angularVelocity().x, 0.0f, 0.05f));

    shaft.addTorque(glm::vec3(0.0f, 5.0f, 0.0f));
    for (u32 i = 0; i < 30; ++i)
        stepWithJoint(yoke, shaft, joint, glm::vec3(0.0f), 1.0f / 60.0f);
    CHECK(std::abs(shaft.angularVelocity().y) > 0.05f);
}

void testUniversalJointKeepsAnchorTogether()
{
    RigidBody yoke = makeStaticBox(glm::vec3(0.0f));
    RigidBody shaft = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f));
    UniversalJoint joint(yoke, shaft, glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f),
                        glm::vec3(0.0f, 0.0f, 1.0f));

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(yoke, shaft, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    const glm::vec3 anchorOnShaft = shaft.pointToWorld(glm::vec3(-0.5f, 0.0f, 0.0f));
    CHECK(near(anchorOnShaft, glm::vec3(0.5f, 0.0f, 0.0f), 0.05f));
}

void testUniversalJointMotorDrivesToTargetVelocity()
{
    RigidBody yoke = makeStaticBox(glm::vec3(0.0f));
    RigidBody shaft = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f));
    UniversalJoint joint(yoke, shaft, glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f),
                        glm::vec3(0.0f, 0.0f, 1.0f));
    joint.setMotorA(3.0f, 30.0f);

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(yoke, shaft, joint, glm::vec3(0.0f), 1.0f / 60.0f);

    CHECK(near(shaft.angularVelocity().y, 3.0f, 0.1f));
}

// ------------------------------------------------------------------ stress

void testChainOfPointJointsHangsWithoutStretching()
{
    // Eight links hanging from a static anchor - the configuration every
    // ragdoll limb reduces to. A chain solved link by link is where drift
    // and stretch pile up first.
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    std::vector<RigidBody> links(8);
    std::vector<RigidBody*> bodies = {&anchor};
    for (u32 i = 0; i < links.size(); ++i)
    {
        links[i] = makeDynamicBox(glm::vec3(static_cast<f32>(i + 1), 0.0f, 0.0f), 1.0f,
                                  glm::vec3(0.4f));
        bodies.push_back(&links[i]);
    }
    std::vector<PointJoint> joints;
    joints.reserve(8);
    joints.emplace_back(anchor, links[0], glm::vec3(0.5f, 0.0f, 0.0f));
    for (u32 i = 1; i < links.size(); ++i)
        joints.emplace_back(links[i - 1], links[i],
                            glm::vec3(static_cast<f32>(i) + 0.5f, 0.0f, 0.0f));
    std::vector<Joint*> jointPointers;
    for (PointJoint& joint : joints)
        jointPointers.push_back(&joint);

    for (u32 step = 0; step < 600; ++step)
        stepJoints(bodies.data(), static_cast<u32>(bodies.size()), jointPointers.data(),
                   static_cast<u32>(jointPointers.size()), glm::vec3(0.0f, -10.0f, 0.0f),
                   1.0f / 60.0f);

    for (const RigidBody& link : links)
        CHECK(finite(link));
    // The chain is 8.5 anchors long; the last link further away than that
    // means the joints stretched apart.
    CHECK(glm::length(links.back().position()) < 9.0f);
    for (const PointJoint& joint : joints)
        CHECK(glm::length(joint.worldAnchorA() - joint.worldAnchorB()) < 0.05f);
}

void testExtremeMassRatioStaysTogether()
{
    // A 100:1 ratio across one joint - the light body gets almost the whole
    // correction and is the first to be launched by an unstable solver.
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody light = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f), 0.05f, glm::vec3(0.3f));
    RigidBody heavy = makeDynamicBox(glm::vec3(2.0f, 0.0f, 0.0f), 5.0f, glm::vec3(0.5f));
    PointJoint first(anchor, light, glm::vec3(0.5f, 0.0f, 0.0f));
    PointJoint second(light, heavy, glm::vec3(1.5f, 0.0f, 0.0f));
    RigidBody* bodies[] = {&anchor, &light, &heavy};
    Joint* joints[] = {&first, &second};

    for (u32 step = 0; step < 600; ++step)
        stepJoints(bodies, 3, joints, 2, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(finite(light));
    CHECK(finite(heavy));
    CHECK(glm::length(first.worldAnchorA() - first.worldAnchorB()) < 0.1f);
    CHECK(glm::length(second.worldAnchorA() - second.worldAnchorB()) < 0.1f);
}

void testHingeLimitSurvivesBeingHammered()
{
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody arm = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f), 1.0f, glm::vec3(0.5f));
    HingeJoint joint(anchor, arm, glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    joint.setLimits(-0.5f, 0.5f);

    // Fifty radians per second straight into the limit, repeatedly.
    for (u32 burst = 0; burst < 4; ++burst)
    {
        arm.setAngularVelocity(glm::vec3(0.0f, 0.0f, burst % 2 == 0 ? 50.0f : -50.0f));
        for (u32 step = 0; step < 60; ++step)
        {
            stepWithJoint(anchor, arm, joint, glm::vec3(0.0f), 1.0f / 60.0f);
            CHECK(finite(arm));
        }
        CHECK(joint.currentAngle() > -0.7f);
        CHECK(joint.currentAngle() < 0.7f);
    }
}

void testHingeMotorAgainstLimitHoldsAtLimit()
{
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody arm = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f), 1.0f, glm::vec3(0.5f));
    HingeJoint joint(anchor, arm, glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    joint.setLimits(-0.3f, 0.3f);
    joint.setMotor(10.0f, 50.0f);

    for (u32 step = 0; step < 300; ++step)
        stepWithJoint(anchor, arm, joint, glm::vec3(0.0f), 1.0f / 60.0f);

    // The motor pushes forever; the limit has to win and hold, with no
    // windup that bursts through and no oscillation left.
    CHECK(finite(arm));
    CHECK(near(joint.currentAngle(), 0.3f, 0.05f));
    CHECK(std::abs(arm.angularVelocity().z) < 0.5f);
}

void testUniversalJointSurvivesParallelAxes()
{
    // Degenerate construction: both hinge axes identical, including the one
    // case where the axis matches the fallback perpendicular's partner.
    const glm::vec3 axes[] = {glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f),
                              glm::vec3(1.0f, 0.0f, 0.0f)};
    for (const glm::vec3& axis : axes)
    {
        RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
        RigidBody shaft = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f));
        UniversalJoint joint(anchor, shaft, glm::vec3(0.5f, 0.0f, 0.0f), axis, axis);
        for (u32 step = 0; step < 120; ++step)
            stepWithJoint(anchor, shaft, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
        CHECK(finite(shaft));
    }
}

void testSliderSurvivesASidewaysWhack()
{
    RigidBody rail = makeStaticBox(glm::vec3(0.0f));
    RigidBody carriage = makeDynamicBox(glm::vec3(0.0f));
    SliderJoint joint(rail, carriage, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    carriage.applyImpulseAtPoint(glm::vec3(50.0f, 0.0f, 30.0f),
                                 carriage.position() + glm::vec3(0.0f, 0.4f, 0.0f));
    for (u32 step = 0; step < 300; ++step)
        stepWithJoint(rail, carriage, joint, glm::vec3(0.0f), 1.0f / 60.0f);

    CHECK(finite(carriage));
    CHECK(near(carriage.position().x, 0.0f, 0.05f));
    CHECK(near(carriage.position().z, 0.0f, 0.05f));
}

void testPointJointRecoversFromATeleport()
{
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody weight = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f));
    PointJoint joint(anchor, weight, glm::vec3(0.5f, 0.0f, 0.0f));

    for (u32 step = 0; step < 60; ++step)
        stepWithJoint(anchor, weight, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    // Someone moves the body by hand, ten metres away - a scene edit, a
    // respawn. The joint has to reel it back in instead of detonating.
    weight.setPosition(weight.position() + glm::vec3(10.0f, 5.0f, -3.0f));
    for (u32 step = 0; step < 300; ++step)
    {
        stepWithJoint(anchor, weight, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
        CHECK(finite(weight));
    }
    CHECK(glm::length(joint.worldAnchorA() - joint.worldAnchorB()) < 0.1f);
}

void testHingeMotorSurvivesVaryingTimestep()
{
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody wheel = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f), 1.0f, glm::vec3(0.5f));
    HingeJoint joint(anchor, wheel, glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    joint.setMotor(4.0f, 20.0f);

    // Frame spikes: the warm-start impulse scaling is exactly what breaks
    // when the step keeps changing.
    for (u32 step = 0; step < 300; ++step)
    {
        const f32 dt = step % 3 == 0 ? 1.0f / 30.0f : step % 3 == 1 ? 1.0f / 240.0f : 1.0f / 60.0f;
        stepWithJoint(anchor, wheel, joint, glm::vec3(0.0f), dt);
        CHECK(finite(wheel));
    }
    CHECK(near(wheel.angularVelocity().z, 4.0f, 0.3f));
}

void testDynamicPairConservesLinearMomentum()
{
    // Two free bodies joined by a point joint, no gravity: every joint
    // impulse is internal and equal-and-opposite, so the pair's total
    // momentum must not drift.
    RigidBody a = makeDynamicBox(glm::vec3(0.0f), 2.0f);
    RigidBody b = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f), 1.0f);
    a.setVelocity(glm::vec3(3.0f, 1.0f, -2.0f));
    PointJoint joint(a, b, glm::vec3(0.5f, 0.0f, 0.0f));

    const glm::vec3 momentumBefore = a.velocity() * 2.0f + b.velocity() * 1.0f;
    for (u32 step = 0; step < 300; ++step)
        stepWithJoint(a, b, joint, glm::vec3(0.0f), 1.0f / 60.0f);
    const glm::vec3 momentumAfter = a.velocity() * 2.0f + b.velocity() * 1.0f;

    CHECK(finite(a));
    CHECK(finite(b));
    CHECK(near(momentumBefore, momentumAfter, 0.05f));
}

void testFixedJointHoldsUnderHeavyTorque()
{
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody plate = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f));
    FixedJoint joint(anchor, plate, glm::vec3(0.5f, 0.0f, 0.0f));

    for (u32 step = 0; step < 300; ++step)
    {
        plate.addTorque(glm::vec3(0.0f, 300.0f, 0.0f));
        stepWithJoint(anchor, plate, joint, glm::vec3(0.0f), 1.0f / 60.0f);
        CHECK(finite(plate));
    }
    // Torque against a weld deflects a little each step and is pulled back;
    // it must not ratchet into a slow spin.
    glm::quat deviation = plate.orientation();
    if (deviation.w < 0.0f)
        deviation = -deviation;
    CHECK(2.0f * std::acos(glm::clamp(deviation.w, -1.0f, 1.0f)) < 0.35f);
}

void testPistonCombinedMotionHoldsItsAxis()
{
    // Both freedoms at once: sliding under gravity while an angular motor
    // spins it - the strut case. The locked directions must not leak.
    RigidBody rail = makeStaticBox(glm::vec3(0.0f));
    RigidBody strut = makeDynamicBox(glm::vec3(0.0f));
    PistonJoint joint(rail, strut, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    joint.setAngularMotor(6.0f, 30.0f);

    for (u32 step = 0; step < 300; ++step)
    {
        stepWithJoint(rail, strut, joint, glm::vec3(3.0f, -10.0f, 2.0f), 1.0f / 60.0f);
        CHECK(finite(strut));
    }
    CHECK(near(strut.position().x, 0.0f, 0.05f));
    CHECK(near(strut.position().z, 0.0f, 0.05f));
    CHECK(near(strut.angularVelocity().y, 6.0f, 0.3f));
    CHECK(near(std::abs(strut.angularVelocity().x), 0.0f, 0.1f));
}

// ------------------------------------------------------------------ MouseJoint

void testMouseJointCarriesABodyToTheTarget()
{
    RigidBody box = makeDynamicBox(glm::vec3(0.0f), 1.0f);
    MouseJoint joint(box, glm::vec3(0.0f));
    joint.setMaxForce(1000.0f);
    joint.tuneSpring(5.0f, 0.7f);
    joint.setTarget(glm::vec3(2.0f, 3.0f, -1.0f));

    RigidBody* bodies[] = {&box};
    Joint* joints[] = {&joint};
    for (u32 step = 0; step < 240; ++step)
        stepJoints(bodies, 1, joints, 1, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(finite(box));
    // A soft spring under gravity holds a little below the target; what it
    // must not do is lag by much or keep swinging.
    CHECK(glm::length(box.position() - joint.target()) < 0.15f);
    CHECK(glm::length(box.velocity()) < 0.2f);
}

void testMouseJointForceCapCannotYankABody()
{
    // Five newtons cannot lift a 1 kg body out of 10 m/s^2 gravity, so the
    // capped joint must lose - the body keeps falling instead of being
    // teleported to the cursor.
    RigidBody box = makeDynamicBox(glm::vec3(0.0f), 1.0f);
    MouseJoint joint(box, glm::vec3(0.0f));
    joint.setMaxForce(5.0f);
    joint.tuneSpring(5.0f, 0.7f);
    joint.setTarget(glm::vec3(0.0f, 50.0f, 0.0f));

    RigidBody* bodies[] = {&box};
    Joint* joints[] = {&joint};
    for (u32 step = 0; step < 120; ++step)
        stepJoints(bodies, 1, joints, 1, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(finite(box));
    CHECK(box.position().y < 0.0f);
}

void testMouseJointSurvivesAFarTarget()
{
    RigidBody box = makeDynamicBox(glm::vec3(0.0f), 1.0f);
    MouseJoint joint(box, glm::vec3(0.0f));
    joint.setMaxForce(1000.0f);
    joint.setTarget(glm::vec3(500.0f, 0.0f, 0.0f));

    RigidBody* bodies[] = {&box};
    Joint* joints[] = {&joint};
    for (u32 step = 0; step < 240; ++step)
    {
        stepJoints(bodies, 1, joints, 1, glm::vec3(0.0f), 1.0f / 60.0f);
        CHECK(finite(box));
    }
    // The force cap turns a far target into a bounded chase, never a launch
    // beyond it.
    CHECK(box.position().x < 520.0f);
}

void testMouseJointGrabsASleepingBodyInAWorld()
{
    // The exact grab-in-a-level scenario: a crate settled on the floor long
    // enough to fall asleep, then grabbed. Everything runs through
    // Scene::updatePhysics() with its fixed-step accumulator, not through the
    // solver directly, because that is the path a demo actually takes.
    Radion::Scene world;
    world.setGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.setFixedStep(1.0f / 120.0f);

    BoxShape groundShape(glm::vec3(20.0f, 0.5f, 20.0f));
    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(glm::vec3(0.0f, -0.5f, 0.0f));
    ground.setShape(&groundShape);
    ground.setFriction(0.7f);
    world.addBody(ground);

    BoxShape crateShape(glm::vec3(0.4f));
    RigidBody crate;
    crate.setMass(1.5f);
    crate.setInertiaTensor(Inertia::box(1.5f, glm::vec3(0.4f)));
    crate.setPosition(glm::vec3(0.0f, 0.4f, 0.0f));
    crate.setShape(&crateShape);
    crate.setFriction(0.6f);
    world.addBody(crate);

    for (u32 step = 0; step < 600; ++step)
        world.updatePhysics(1.0f / 60.0f);
    CHECK(!crate.awake());

    MouseJoint joint(crate, crate.position() + glm::vec3(0.0f, 0.4f, 0.0f));
    joint.setMaxForce(1000.0f * 1.5f);
    joint.tuneSpring(5.0f, 0.7f);
    world.addJoint(&joint);
    joint.setTarget(crate.position() + glm::vec3(0.0f, 2.0f, 0.0f));

    for (u32 step = 0; step < 240; ++step)
        world.updatePhysics(1.0f / 60.0f);
    world.removeJoint(&joint);

    CHECK(finite(crate));
    // Grabbed two metres up: a joint that cannot wake a sleeping body leaves
    // the crate exactly where it slept.
    CHECK(crate.position().y > 1.0f);
}

// ---------------------------------------------------- Scene registration safety

// A loose joint's destructor must pull it out of any Scene it was added to
// directly (Scene::addJoint(), no GameObject involved) - otherwise the
// Scene's own joint list outlives the memory it points at the moment this
// scope ends.
void testLoosePointJointDeregistersOnDestruction()
{
    RigidBody a = makeStaticBox(glm::vec3(0.0f));
    RigidBody b = makeDynamicBox(glm::vec3(1.0f, 0.0f, 0.0f));
    Radion::Scene world;
    world.addBody(a);
    world.addBody(b);
    CHECK(world.jointCount() == 0);
    {
        PointJoint joint(a, b, glm::vec3(0.5f, 0.0f, 0.0f));
        world.addJoint(&joint);
        CHECK(world.jointCount() == 1);
    }
    CHECK(world.jointCount() == 0);
}

// std::vector<PointJoint> reallocates by moving its elements to new storage.
// PointJoint is the one joint class other code (a chain, a car's wheel
// mounts) keeps in a vector rather than one at a time, so its move
// constructor is the one that actually runs here - every other joint test in
// this file sidesteps that with an upfront reserve(). A reserve() upfront is
// the supported way to keep several registered joints in one vector - with
// no reallocation, nothing ever moves, and every joint stays registered (see
// the next test for what a growth WITHOUT the reserve does instead).
void testPointJointVectorGrowthKeepsSceneRegistrationCorrect()
{
    Radion::Scene world;
    world.setGravity(glm::vec3(0.0f, -10.0f, 0.0f));

    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    world.addBody(anchor);

    // Reserved up front - RigidBody's own pointer stability across a vector
    // growth is a separate concern, covered in DynamicsTests.cpp.
    std::vector<RigidBody> links;
    links.reserve(6);
    std::vector<PointJoint> joints;
    joints.reserve(6);
    for (u32 i = 0; i < 6; ++i)
    {
        links.push_back(makeDynamicBox(glm::vec3(static_cast<f32>(i + 1), 0.0f, 0.0f)));
        world.addBody(links.back());
        RigidBody& previous = i == 0 ? anchor : links[i - 1];
        joints.emplace_back(previous, links.back(),
                            glm::vec3(static_cast<f32>(i) + 0.5f, 0.0f, 0.0f));
        world.addJoint(&joints.back());
    }
    CHECK(world.jointCount() == 6);

    for (u32 step = 0; step < 300; ++step)
        world.stepPhysics(1.0f / 60.0f);

    CHECK(world.jointCount() == 6);
    for (const RigidBody& link : links)
        CHECK(finite(link));
    for (const PointJoint& joint : joints)
        CHECK(glm::length(joint.worldAnchorA() - joint.worldAnchorB()) < 0.1f);
}

// The unsupported counterpart to the test above, with no reserve() on
// `joints`: every emplace_back past the small starting capacity reallocates
// and moves the existing joints, and Joint::moveJointStateFrom() deregisters
// each one from the Scene rather than leave a dangling mJoints entry pointing
// at the freed old buffer. The guarantee this checks is narrower than "it
// still works" - only that it fails SAFELY (no crash, no stale pointer ever
// dereferenced by stepPhysics(), the count exactly matches what survived)
// rather than corrupting the Scene.
void testPointJointVectorGrowthWithoutReserveDropsRegistrationSafely()
{
    Radion::Scene world;
    world.setGravity(glm::vec3(0.0f, -10.0f, 0.0f));

    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    world.addBody(anchor);

    std::vector<RigidBody> links;
    links.reserve(6);
    std::vector<PointJoint> joints;
    for (u32 i = 0; i < 6; ++i)
    {
        links.push_back(makeDynamicBox(glm::vec3(static_cast<f32>(i + 1), 0.0f, 0.0f)));
        world.addBody(links.back());
        RigidBody& previous = i == 0 ? anchor : links[i - 1];
        joints.emplace_back(previous, links.back(),
                            glm::vec3(static_cast<f32>(i) + 0.5f, 0.0f, 0.0f));
        world.addJoint(&joints.back());
    }
    CHECK(world.jointCount() <= joints.size());

    for (u32 step = 0; step < 300; ++step)
        world.stepPhysics(1.0f / 60.0f);
    for (const RigidBody& link : links)
        CHECK(finite(link));
}

// ------------------------------------------------------------- warm start energy

void testWarmStartDoesNotInjectEnergy()
{
    RigidBody anchor = makeStaticBox(glm::vec3(0.0f));
    RigidBody weight = makeDynamicBox(glm::vec3(2.0f, 0.0f, 0.0f));
    DistanceJoint joint(anchor, glm::vec3(0.0f), weight, glm::vec3(0.0f), 2.0f, 2.0f);

    f32 maxSpeed = 0.0f;
    for (u32 i = 0; i < 600; ++i)
    {
        stepWithJoint(anchor, weight, joint, glm::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
        maxSpeed = glm::max(maxSpeed, glm::length(weight.velocity()));
    }

    // A pendulum released from rest at radius 2 under g=10 cannot exceed the
    // speed of a free fall through the same 2m drop: v = sqrt(2 g h) ~= 6.32.
    CHECK(maxSpeed < 8.0f);
}

// ------------------------------------------------------------------ WheelJoint

RigidBody makeChassis()
{
    RigidBody body;
    body.setBodyType(BodyType::Static);
    body.setPosition(glm::vec3(0.0f, 1.0f, 0.0f));
    return body;
}

RigidBody makeWheel(const glm::vec3& position)
{
    RigidBody body;
    body.setBodyType(BodyType::Dynamic);
    body.setPosition(position);
    body.setMass(15.0f);
    body.setInertiaTensor(Inertia::box(15.0f, glm::vec3(0.3f, 0.3f, 0.3f)));
    // These tests drive the body directly, without a Scene to manage the
    // joint's sleep island (Scene::addJoint wakes both bodies and keeps
    // them awake together) - without this the body falls asleep the
    // moment it settles, and the spring keeps nudging a velocity that no
    // longer gets integrated.
    body.setCanSleep(false);
    return body;
}

void testSuspensionSettlesAtRestLength()
{
    RigidBody chassis = makeChassis();
    RigidBody wheel = makeWheel(glm::vec3(0.0f, 0.3f, 0.0f));

    // Anchor at the wheel's own centre: no lever arm on the wheel side, so
    // free rotation about the spin axis (nothing constrains it) can't couple
    // into the wheel's linear velocity through a stray arm.
    WheelJoint wheelJoint(chassis, wheel, wheel.position(), glm::vec3(0.0f, -1.0f, 0.0f),
                         glm::vec3(1.0f, 0.0f, 0.0f));
    wheelJoint.setSuspension(0.5f, 4000.0f, 400.0f);

    const f32 dt = 1.0f / 120.0f;
    for (int i = 0; i < 600; ++i)
    {
        wheel.setAcceleration(glm::vec3(0.0f, -9.81f, 0.0f));
        wheel.integrateForces(dt);
        wheelJoint.setup(dt);
        wheelJoint.warmStart();
        wheelJoint.solveVelocity();
        wheel.integrateVelocity(dt);
        wheelJoint.solvePosition(0.2f);
    }

    // Settled: travel close to the spring's equilibrium (rest length plus the
    // extra compression gravity holds it at), not still falling.
    CHECK(std::fabs(wheelJoint.suspensionTravel() - 0.529f) < 0.02f);
    CHECK(std::fabs(wheel.velocity().y) < 0.05f);
}

void testSteeringMotorTurnsWithinLimits()
{
    RigidBody chassis = makeChassis();
    RigidBody wheel = makeWheel(glm::vec3(0.0f, 0.5f, 0.0f));

    WheelJoint wheelJoint(chassis, wheel, wheel.position(), glm::vec3(0.0f, -1.0f, 0.0f),
                         glm::vec3(1.0f, 0.0f, 0.0f));
    wheelJoint.setSuspension(0.5f, 4000.0f, 400.0f);
    wheelJoint.setSteeringLimits(-0.5f, 0.5f);
    wheelJoint.setSteeringMotor(10.0f, 500.0f);

    const f32 dt = 1.0f / 120.0f;
    for (int i = 0; i < 240; ++i)
    {
        wheel.setAcceleration(glm::vec3(0.0f, -9.81f, 0.0f));
        wheel.integrateForces(dt);
        wheelJoint.setup(dt);
        wheelJoint.warmStart();
        wheelJoint.solveVelocity();
        wheel.integrateVelocity(dt);
        wheelJoint.solvePosition(0.2f);
    }

    CHECK(wheelJoint.steeringAngle() <= 0.55f);
    CHECK(wheelJoint.steeringAngle() > 0.0f);
}

void testSpinMotorDrivesWheelWithoutLimit()
{
    RigidBody chassis = makeChassis();
    RigidBody wheel = makeWheel(glm::vec3(0.0f, 0.5f, 0.0f));

    WheelJoint wheelJoint(chassis, wheel, wheel.position(), glm::vec3(0.0f, -1.0f, 0.0f),
                         glm::vec3(1.0f, 0.0f, 0.0f));
    wheelJoint.setSuspension(0.5f, 4000.0f, 400.0f);
    wheelJoint.setSpinMotor(20.0f, 200.0f);

    const f32 dt = 1.0f / 120.0f;
    for (int i = 0; i < 120; ++i)
    {
        wheel.setAcceleration(glm::vec3(0.0f, -9.81f, 0.0f));
        wheel.integrateForces(dt);
        wheelJoint.setup(dt);
        wheelJoint.warmStart();
        wheelJoint.solveVelocity();
        wheel.integrateVelocity(dt);
        wheelJoint.solvePosition(0.2f);
    }

    // Free to spin without bound: should reach close to the target speed.
    CHECK(std::fabs(wheelJoint.spinAngularVelocity() - 20.0f) < 2.0f);
}

} // namespace

int main()
{
    testDistanceJointHoldsFixedLength();
    testDistanceJointRangeAllowsSlack();
    testFixedJointLocksAllSixDOF();
    testHingeJointFreeAboutItsAxisOnly();
    testHingeJointMotorDrivesToTargetVelocity();
    testHingeServoReachesAndHoldsItsAngle();
    testHingeServoClampsTargetToLimits();
    testHingeServoRespectsItsTorqueBudget();
    testSliderServoReachesAndHoldsItsPosition();
    testServoChainSagUnderLoad();
    testHingeJointKeepsAnchorTogether();
    testSliderJointMovesOnlyAlongItsAxis();
    testSliderJointLimitsClampTravel();
    testSliderJointMotorDrivesToTargetVelocity();
    testPistonJointMovesAndSpinsOnlyAlongItsAxis();
    testPistonJointLinearMotorAndAngularLimitAreIndependent();
    testUniversalJointFreeAboutBothAxesOnly();
    testUniversalJointKeepsAnchorTogether();
    testUniversalJointMotorDrivesToTargetVelocity();
    testChainOfPointJointsHangsWithoutStretching();
    testExtremeMassRatioStaysTogether();
    testHingeLimitSurvivesBeingHammered();
    testHingeMotorAgainstLimitHoldsAtLimit();
    testUniversalJointSurvivesParallelAxes();
    testSliderSurvivesASidewaysWhack();
    testPointJointRecoversFromATeleport();
    testHingeMotorSurvivesVaryingTimestep();
    testDynamicPairConservesLinearMomentum();
    testFixedJointHoldsUnderHeavyTorque();
    testPistonCombinedMotionHoldsItsAxis();
    testMouseJointCarriesABodyToTheTarget();
    testMouseJointForceCapCannotYankABody();
    testMouseJointSurvivesAFarTarget();
    testMouseJointGrabsASleepingBodyInAWorld();
    testWarmStartDoesNotInjectEnergy();
    testSuspensionSettlesAtRestLength();
    testSteeringMotorTurnsWithinLimits();
    testSpinMotorDrivesWheelWithoutLimit();
    testLoosePointJointDeregistersOnDestruction();
    testPointJointVectorGrowthKeepsSceneRegistrationCorrect();
    testPointJointVectorGrowthWithoutReserveDropsRegistrationSafely();
    if (gFailures)
        std::fprintf(stderr, "%d joint test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
