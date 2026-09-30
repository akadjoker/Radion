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
#include <limits>
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

bool near(const Math::vec3& a, const Math::vec3& b, f32 epsilon = 1e-3f)
{
    return Math::length(a - b) <= epsilon;
}

RigidBody makeDynamicBox(const Math::vec3& position, f32 mass = 1.0f,
                         const Math::vec3& halfExtents = Math::vec3(0.5f))
{
    RigidBody body;
    body.setPosition(position);
    body.setMass(mass);
    body.setInertiaTensor(Inertia::box(mass, halfExtents));
    body.setDamping(1.0f, 1.0f);
    body.setCanSleep(false);
    return body;
}

RigidBody makeStaticBox(const Math::vec3& position)
{
    RigidBody body;
    body.setPosition(position);
    body.setBodyType(BodyType::Static);
    return body;
}

void stepWithJoint(RigidBody& a, RigidBody& b, Joint& joint, const Math::vec3& gravity, f32 duration,
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
                const Math::vec3& gravity, f32 duration)
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

bool finiteVec(const Math::vec3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

void testDistanceJointHoldsFixedLength()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody weight = makeDynamicBox(Math::vec3(2.0f, 0.0f, 0.0f));
    DistanceJoint joint(anchor, Math::vec3(0.0f), weight, Math::vec3(0.0f), 2.0f, 2.0f);

    for (u32 i = 0; i < 600; ++i)
        stepWithJoint(anchor, weight, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    const f32 length = Math::length(weight.position() - anchor.position());
    CHECK(near(length, 2.0f, 0.02f));
}

void testDistanceJointRangeAllowsSlack()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody weight = makeDynamicBox(Math::vec3(0.5f, 0.0f, 0.0f));
    DistanceJoint joint(anchor, Math::vec3(0.0f), weight, Math::vec3(0.0f), 0.0f, 2.0f);

    for (u32 i = 0; i < 6; ++i)
        stepWithJoint(anchor, weight, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(weight.position().y < -0.01f);

    for (u32 i = 0; i < 600; ++i)
        stepWithJoint(anchor, weight, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    const f32 length = Math::length(weight.position() - anchor.position());
    CHECK(length <= 2.02f);
    CHECK(near(length, 2.0f, 0.02f));
}

void testFixedJointLocksAllSixDOF()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody plate = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f));
    FixedJoint joint(anchor, plate, Math::vec3(0.5f, 0.0f, 0.0f));

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(anchor, plate, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(near(plate.position(), Math::vec3(1.0f, 0.0f, 0.0f), 0.02f));
    CHECK(near(Math::length(plate.orientation() - Math::quat(1.0f, 0.0f, 0.0f, 0.0f)), 0.0f, 0.05f));
}

void testHingeJointFreeAboutItsAxisOnly()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody arm = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
    HingeJoint joint(anchor, arm, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f));

    arm.addTorque(Math::vec3(0.0f, 5.0f, 0.0f));
    for (u32 i = 0; i < 30; ++i)
        stepWithJoint(anchor, arm, joint, Math::vec3(0.0f), 1.0f / 60.0f);

    CHECK(near(arm.angularVelocity().y, 0.0f, 0.05f));

    arm.addTorque(Math::vec3(0.0f, 0.0f, 5.0f));
    for (u32 i = 0; i < 30; ++i)
        stepWithJoint(anchor, arm, joint, Math::vec3(0.0f), 1.0f / 60.0f);

    CHECK(std::abs(arm.angularVelocity().z) > 0.05f);
}

void testHingeJointMotorDrivesToTargetVelocity()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody wheel = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
    HingeJoint joint(anchor, wheel, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f));
    joint.setMotor(4.0f, 20.0f);

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(anchor, wheel, joint, Math::vec3(0.0f), 1.0f / 60.0f);

    CHECK(near(wheel.angularVelocity().z, 4.0f, 0.1f));
}

void testHingeServoReachesAndHoldsItsAngle()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody arm = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
    HingeJoint joint(anchor, arm, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f));

    const f32 target = 0.6f;
    joint.setServo(target, 200.0f);
    CHECK(joint.servoEnabled());
    CHECK(joint.motorEnabled());

    for (u32 i = 0; i < 400; ++i)
        stepWithJoint(anchor, arm, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
    CHECK(near(joint.currentAngle(), target, 0.02f));

    const f32 settled = joint.currentAngle();
    for (u32 i = 0; i < 400; ++i)
        stepWithJoint(anchor, arm, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
    CHECK(near(joint.currentAngle(), settled, 0.01f));
    CHECK(std::abs(arm.angularVelocity().z) < 0.05f);

    joint.setServo(-0.4f, 200.0f);
    for (u32 i = 0; i < 400; ++i)
        stepWithJoint(anchor, arm, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
    CHECK(near(joint.currentAngle(), -0.4f, 0.02f));
}

// A target outside the limits is clamped, not chased through them (as btHingeConstraint::setMotorTarget()).
void testHingeServoClampsTargetToLimits()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody arm = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
    HingeJoint joint(anchor, arm, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f));
    joint.setLimits(-0.3f, 0.5f);
    joint.setServo(2.0f, 200.0f);

    for (u32 i = 0; i < 400; ++i)
        stepWithJoint(anchor, arm, joint, Math::vec3(0.0f), 1.0f / 60.0f);

    CHECK(near(joint.currentAngle(), 0.5f, 0.03f));
}

// Bounded torque: with too little the joint cannot lift its load, and a caller must see that.
void testHingeServoRespectsItsTorqueBudget()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody arm = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
    HingeJoint joint(anchor, arm, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f));
    joint.setServo(1.2f, 0.2f);

    for (u32 i = 0; i < 400; ++i)
        stepWithJoint(anchor, arm, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(joint.currentAngle() < 1.2f - 0.1f);
    CHECK(std::isfinite(joint.currentAngle()));
}

void testSliderServoReachesAndHoldsItsPosition()
{
    RigidBody rail = makeStaticBox(Math::vec3(0.0f));
    RigidBody finger = makeDynamicBox(Math::vec3(0.0f));
    SliderJoint joint(rail, finger, Math::vec3(0.0f), Math::vec3(1.0f, 0.0f, 0.0f));
    joint.setServo(0.35f, 200.0f);

    for (u32 i = 0; i < 400; ++i)
        stepWithJoint(rail, finger, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(near(joint.currentPosition(), 0.35f, 0.02f));
    CHECK(std::abs(finger.velocity().x) < 0.05f);
}

// Servo sag of a chain of three servo-held links with an end mass (a leg holding its share of a body); reported, not merely asserted.
void testServoChainSagUnderLoad()
{
    RigidBody root = makeStaticBox(Math::vec3(0.0f));
    RigidBody link1 = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
    RigidBody link2 = makeDynamicBox(Math::vec3(2.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
    RigidBody load = makeDynamicBox(Math::vec3(3.0f, 0.0f, 0.0f), 5.0f, Math::vec3(0.5f));

    const Math::vec3 axis(0.0f, 0.0f, 1.0f);
    HingeJoint hip(root, link1, Math::vec3(0.5f, 0.0f, 0.0f), axis);
    HingeJoint knee(link1, link2, Math::vec3(1.5f, 0.0f, 0.0f), axis);
    HingeJoint ankle(link2, load, Math::vec3(2.5f, 0.0f, 0.0f), axis);

    // Rated speed 2 rad/s, about 115 deg/s.
    hip.setServo(0.0f, 2000.0f, 2.0f);
    knee.setServo(0.0f, 2000.0f, 2.0f);
    ankle.setServo(0.0f, 2000.0f, 2.0f);

    RigidBody* bodies[] = {&root, &link1, &link2, &load};
    Joint* joints[] = {&hip, &knee, &ankle};
    const f32 duration = 1.0f / 120.0f;
    const Math::vec3 gravity(0.0f, -10.0f, 0.0f);

    // Swept against solver iterations: error shrinking with them means solver convergence is measured, not the servo.
    f32 hipSag = 0.0f, kneeSag = 0.0f, ankleSag = 0.0f, tipDrop = 0.0f;
    f32 worstByIterations[3] = {0.0f, 0.0f, 0.0f};
    u32 sweepIndex = 0;
    for (u32 iterations : {8u, 32u, 128u})
    {
        for (RigidBody* body : bodies)
        {
            body->setVelocity(Math::vec3(0.0f));
            body->setAngularVelocity(Math::vec3(0.0f));
            body->setOrientation(Math::quat(1.0f, 0.0f, 0.0f, 0.0f));
        }
        link1.setPosition(Math::vec3(1.0f, 0.0f, 0.0f));
        link2.setPosition(Math::vec3(2.0f, 0.0f, 0.0f));
        load.setPosition(Math::vec3(3.0f, 0.0f, 0.0f));

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
            // Only the second half counts, so the initial settle is not mistaken for wobble.
            if (step > 300)
                worstHip = Math::max(worstHip, Math::degrees(std::abs(hip.currentAngle())));
        }

        worstByIterations[sweepIndex++] = worstHip;
        hipSag = Math::degrees(std::abs(hip.currentAngle()));
        kneeSag = Math::degrees(std::abs(knee.currentAngle()));
        ankleSag = Math::degrees(std::abs(ankle.currentAngle()));
        tipDrop = -load.position().y;
        std::printf("JointTests: servo chain, %3u velocity iterations - worst hip swing %.3f deg, "
                    "final hip %.3f / knee %.3f / ankle %.3f deg, tip drop %.4f m\n",
                    iterations, static_cast<double>(worstHip), static_cast<double>(hipSag),
                    static_cast<double>(kneeSag), static_cast<double>(ankleSag),
                    static_cast<double>(tipDrop));
    }

    CHECK(std::isfinite(hipSag) && std::isfinite(tipDrop));

    // Chain error is solver convergence: measured 10.2 -> 3.4 -> 0.36 degrees at 8/32/128 iterations.
    CHECK(worstByIterations[1] < worstByIterations[0]);
    CHECK(worstByIterations[2] < worstByIterations[1]);
    // At 128 the swing at the loaded joint must be under a degree, or a robot built on this engine is not credible.
    CHECK(worstByIterations[2] < 1.0f);
    // The default 8 is a game setting: the bound only catches the chain turning to rubber.
    CHECK(worstByIterations[0] < 15.0f);
}

void testHingeJointKeepsAnchorTogether()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody arm = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
    HingeJoint joint(anchor, arm, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f));

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(anchor, arm, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    const Math::vec3 hingePointOnArm = arm.pointToWorld(Math::vec3(-0.5f, 0.0f, 0.0f));
    CHECK(near(hingePointOnArm, Math::vec3(0.5f, 0.0f, 0.0f), 0.05f));
}

void testSliderJointMovesOnlyAlongItsAxis()
{
    RigidBody rail = makeStaticBox(Math::vec3(0.0f));
    RigidBody carriage = makeDynamicBox(Math::vec3(0.0f, -1.0f, 0.0f));
    SliderJoint joint(rail, carriage, Math::vec3(0.0f, -1.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f));

    for (u32 i = 0; i < 60; ++i)
        stepWithJoint(rail, carriage, joint, Math::vec3(4.0f, -10.0f, 3.0f), 1.0f / 60.0f);

    CHECK(near(carriage.position().x, 0.0f, 0.02f));
    CHECK(near(carriage.position().z, 0.0f, 0.02f));
    CHECK(carriage.position().y < -1.0f);
    CHECK(near(Math::length(carriage.orientation() - Math::quat(1.0f, 0.0f, 0.0f, 0.0f)), 0.0f,
              0.02f));
}

void testSliderJointLimitsClampTravel()
{
    RigidBody rail = makeStaticBox(Math::vec3(0.0f));
    RigidBody piston = makeDynamicBox(Math::vec3(0.0f, -0.2f, 0.0f));
    SliderJoint joint(rail, piston, Math::vec3(0.0f, -0.2f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f));
    joint.setLimits(-1.0f, 0.0f);

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(rail, piston, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    // The limit clamps displacement, not position: started at y=-0.2, so the floor is -0.2 + (-1.0).
    CHECK(joint.currentPosition() >= -1.02f);
    CHECK(near(joint.currentPosition(), -1.0f, 0.03f));
    CHECK(near(piston.position().y, -1.2f, 0.03f));
}

void testSliderJointMotorDrivesToTargetVelocity()
{
    RigidBody rail = makeStaticBox(Math::vec3(0.0f));
    RigidBody piston = makeDynamicBox(Math::vec3(0.0f));
    SliderJoint joint(rail, piston, Math::vec3(0.0f), Math::vec3(1.0f, 0.0f, 0.0f));
    joint.setMotor(3.0f, 50.0f);

    for (u32 i = 0; i < 120; ++i)
        stepWithJoint(rail, piston, joint, Math::vec3(0.0f), 1.0f / 60.0f);

    CHECK(near(piston.velocity().x, 3.0f, 0.05f));
}

void testPistonJointMovesAndSpinsOnlyAlongItsAxis()
{
    RigidBody rail = makeStaticBox(Math::vec3(0.0f));
    RigidBody strut = makeDynamicBox(Math::vec3(0.0f, -1.0f, 0.0f));
    PistonJoint joint(rail, strut, Math::vec3(0.0f, -1.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f));

    strut.addTorque(Math::vec3(3.0f, 0.0f, 0.0f));
    for (u32 i = 0; i < 60; ++i)
        stepWithJoint(rail, strut, joint, Math::vec3(4.0f, -10.0f, 3.0f), 1.0f / 60.0f);

    CHECK(near(strut.position().x, 0.0f, 0.03f));
    CHECK(near(strut.position().z, 0.0f, 0.03f));
    CHECK(near(strut.angularVelocity().x, 0.0f, 0.05f));
    CHECK(strut.position().y < -1.0f);
}

void testPistonJointLinearMotorAndAngularLimitAreIndependent()
{
    RigidBody rail = makeStaticBox(Math::vec3(0.0f));
    RigidBody strut = makeDynamicBox(Math::vec3(0.0f));
    PistonJoint joint(rail, strut, Math::vec3(0.0f), Math::vec3(0.0f, 1.0f, 0.0f));
    joint.setLinearMotor(2.0f, 40.0f);
    joint.setAngularLimits(-0.2f, 0.2f);

    strut.addTorque(Math::vec3(0.0f, 6.0f, 0.0f));
    for (u32 i = 0; i < 240; ++i)
        stepWithJoint(rail, strut, joint, Math::vec3(0.0f), 1.0f / 60.0f);

    CHECK(near(strut.velocity().y, 2.0f, 0.05f));
    CHECK(joint.currentAngle() <= 0.22f);
    CHECK(joint.currentAngle() >= -0.22f);
}

void testUniversalJointFreeAboutBothAxesOnly()
{
    RigidBody yoke = makeStaticBox(Math::vec3(0.0f));
    RigidBody shaft = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f));
    UniversalJoint joint(yoke, shaft, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f),
                        Math::vec3(0.0f, 0.0f, 1.0f));

    // A torque about the shared perpendicular (x, locked by the point) must not build angular velocity.
    shaft.addTorque(Math::vec3(5.0f, 0.0f, 0.0f));
    for (u32 i = 0; i < 30; ++i)
        stepWithJoint(yoke, shaft, joint, Math::vec3(0.0f), 1.0f / 60.0f);
    CHECK(near(shaft.angularVelocity().x, 0.0f, 0.05f));

    shaft.addTorque(Math::vec3(0.0f, 5.0f, 0.0f));
    for (u32 i = 0; i < 30; ++i)
        stepWithJoint(yoke, shaft, joint, Math::vec3(0.0f), 1.0f / 60.0f);
    CHECK(std::abs(shaft.angularVelocity().y) > 0.05f);
}

void testUniversalJointKeepsAnchorTogether()
{
    RigidBody yoke = makeStaticBox(Math::vec3(0.0f));
    RigidBody shaft = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f));
    UniversalJoint joint(yoke, shaft, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f),
                        Math::vec3(0.0f, 0.0f, 1.0f));

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(yoke, shaft, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    const Math::vec3 anchorOnShaft = shaft.pointToWorld(Math::vec3(-0.5f, 0.0f, 0.0f));
    CHECK(near(anchorOnShaft, Math::vec3(0.5f, 0.0f, 0.0f), 0.05f));
}

void testUniversalJointMotorDrivesToTargetVelocity()
{
    RigidBody yoke = makeStaticBox(Math::vec3(0.0f));
    RigidBody shaft = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f));
    UniversalJoint joint(yoke, shaft, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f),
                        Math::vec3(0.0f, 0.0f, 1.0f));
    joint.setMotorA(3.0f, 30.0f);

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(yoke, shaft, joint, Math::vec3(0.0f), 1.0f / 60.0f);

    CHECK(near(shaft.angularVelocity().y, 3.0f, 0.1f));
}

void testChainOfPointJointsHangsWithoutStretching()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    std::vector<RigidBody> links(8);
    std::vector<RigidBody*> bodies = {&anchor};
    for (u32 i = 0; i < links.size(); ++i)
    {
        links[i] = makeDynamicBox(Math::vec3(static_cast<f32>(i + 1), 0.0f, 0.0f), 1.0f,
                                  Math::vec3(0.4f));
        bodies.push_back(&links[i]);
    }
    std::vector<PointJoint> joints;
    joints.reserve(8);
    joints.emplace_back(anchor, links[0], Math::vec3(0.5f, 0.0f, 0.0f));
    for (u32 i = 1; i < links.size(); ++i)
        joints.emplace_back(links[i - 1], links[i],
                            Math::vec3(static_cast<f32>(i) + 0.5f, 0.0f, 0.0f));
    std::vector<Joint*> jointPointers;
    for (PointJoint& joint : joints)
        jointPointers.push_back(&joint);

    for (u32 step = 0; step < 600; ++step)
        stepJoints(bodies.data(), static_cast<u32>(bodies.size()), jointPointers.data(),
                   static_cast<u32>(jointPointers.size()), Math::vec3(0.0f, -10.0f, 0.0f),
                   1.0f / 60.0f);

    for (const RigidBody& link : links)
        CHECK(finite(link));
    // The chain is 8.5 anchors long; a farther last link means the joints stretched.
    CHECK(Math::length(links.back().position()) < 9.0f);
    for (const PointJoint& joint : joints)
        CHECK(Math::length(joint.worldAnchorA() - joint.worldAnchorB()) < 0.05f);
}

void testExtremeMassRatioStaysTogether()
{
    // 100:1 across one joint: the light body takes the correction and is first launched by an unstable solver.
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody light = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 0.05f, Math::vec3(0.3f));
    RigidBody heavy = makeDynamicBox(Math::vec3(2.0f, 0.0f, 0.0f), 5.0f, Math::vec3(0.5f));
    PointJoint first(anchor, light, Math::vec3(0.5f, 0.0f, 0.0f));
    PointJoint second(light, heavy, Math::vec3(1.5f, 0.0f, 0.0f));
    RigidBody* bodies[] = {&anchor, &light, &heavy};
    Joint* joints[] = {&first, &second};

    for (u32 step = 0; step < 600; ++step)
        stepJoints(bodies, 3, joints, 2, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(finite(light));
    CHECK(finite(heavy));
    CHECK(Math::length(first.worldAnchorA() - first.worldAnchorB()) < 0.1f);
    CHECK(Math::length(second.worldAnchorA() - second.worldAnchorB()) < 0.1f);
}

void testHingeLimitSurvivesBeingHammered()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody arm = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
    HingeJoint joint(anchor, arm, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f));
    joint.setLimits(-0.5f, 0.5f);

    for (u32 burst = 0; burst < 4; ++burst)
    {
        arm.setAngularVelocity(Math::vec3(0.0f, 0.0f, burst % 2 == 0 ? 50.0f : -50.0f));
        for (u32 step = 0; step < 60; ++step)
        {
            stepWithJoint(anchor, arm, joint, Math::vec3(0.0f), 1.0f / 60.0f);
            CHECK(finite(arm));
        }
        CHECK(joint.currentAngle() > -0.7f);
        CHECK(joint.currentAngle() < 0.7f);
    }
}

void testHingeMotorAgainstLimitHoldsAtLimit()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody arm = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
    HingeJoint joint(anchor, arm, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f));
    joint.setLimits(-0.3f, 0.3f);
    joint.setMotor(10.0f, 50.0f);

    for (u32 step = 0; step < 300; ++step)
        stepWithJoint(anchor, arm, joint, Math::vec3(0.0f), 1.0f / 60.0f);

    // The limit must win against a forever-pushing motor, with no windup or leftover oscillation.
    CHECK(finite(arm));
    CHECK(near(joint.currentAngle(), 0.3f, 0.05f));
    CHECK(std::abs(arm.angularVelocity().z) < 0.5f);
}

void testUniversalJointSurvivesParallelAxes()
{
    // Both hinge axes identical, including when the axis matches the fallback perpendicular's partner.
    const Math::vec3 axes[] = {Math::vec3(0.0f, 1.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f),
                              Math::vec3(1.0f, 0.0f, 0.0f)};
    for (const Math::vec3& axis : axes)
    {
        RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
        RigidBody shaft = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f));
        UniversalJoint joint(anchor, shaft, Math::vec3(0.5f, 0.0f, 0.0f), axis, axis);
        for (u32 step = 0; step < 120; ++step)
            stepWithJoint(anchor, shaft, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
        CHECK(finite(shaft));
    }
}

void testSliderSurvivesASidewaysWhack()
{
    RigidBody rail = makeStaticBox(Math::vec3(0.0f));
    RigidBody carriage = makeDynamicBox(Math::vec3(0.0f));
    SliderJoint joint(rail, carriage, Math::vec3(0.0f), Math::vec3(0.0f, 1.0f, 0.0f));

    carriage.applyImpulseAtPoint(Math::vec3(50.0f, 0.0f, 30.0f),
                                 carriage.position() + Math::vec3(0.0f, 0.4f, 0.0f));
    for (u32 step = 0; step < 300; ++step)
        stepWithJoint(rail, carriage, joint, Math::vec3(0.0f), 1.0f / 60.0f);

    CHECK(finite(carriage));
    CHECK(near(carriage.position().x, 0.0f, 0.05f));
    CHECK(near(carriage.position().z, 0.0f, 0.05f));
}

void testPointJointRecoversFromATeleport()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody weight = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f));
    PointJoint joint(anchor, weight, Math::vec3(0.5f, 0.0f, 0.0f));

    for (u32 step = 0; step < 60; ++step)
        stepWithJoint(anchor, weight, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    // Body moved 10 m by hand (scene edit, respawn): the joint must reel it in, not detonate.
    weight.setPosition(weight.position() + Math::vec3(10.0f, 5.0f, -3.0f));
    for (u32 step = 0; step < 300; ++step)
    {
        stepWithJoint(anchor, weight, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
        CHECK(finite(weight));
    }
    CHECK(Math::length(joint.worldAnchorA() - joint.worldAnchorB()) < 0.1f);
}

void testHingeMotorSurvivesVaryingTimestep()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody wheel = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
    HingeJoint joint(anchor, wheel, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f));
    joint.setMotor(4.0f, 20.0f);

    // Frame spikes break the warm-start impulse scaling.
    for (u32 step = 0; step < 300; ++step)
    {
        const f32 dt = step % 3 == 0 ? 1.0f / 30.0f : step % 3 == 1 ? 1.0f / 240.0f : 1.0f / 60.0f;
        stepWithJoint(anchor, wheel, joint, Math::vec3(0.0f), dt);
        CHECK(finite(wheel));
    }
    CHECK(near(wheel.angularVelocity().z, 4.0f, 0.3f));
}

void testDynamicPairConservesLinearMomentum()
{
    // No gravity: joint impulses are internal and equal-opposite, so total momentum must not drift.
    RigidBody a = makeDynamicBox(Math::vec3(0.0f), 2.0f);
    RigidBody b = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f);
    a.setVelocity(Math::vec3(3.0f, 1.0f, -2.0f));
    PointJoint joint(a, b, Math::vec3(0.5f, 0.0f, 0.0f));

    const Math::vec3 momentumBefore = a.velocity() * 2.0f + b.velocity() * 1.0f;
    for (u32 step = 0; step < 300; ++step)
        stepWithJoint(a, b, joint, Math::vec3(0.0f), 1.0f / 60.0f);
    const Math::vec3 momentumAfter = a.velocity() * 2.0f + b.velocity() * 1.0f;

    CHECK(finite(a));
    CHECK(finite(b));
    CHECK(near(momentumBefore, momentumAfter, 0.05f));
}

void testFixedJointHoldsUnderHeavyTorque()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody plate = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f));
    FixedJoint joint(anchor, plate, Math::vec3(0.5f, 0.0f, 0.0f));

    for (u32 step = 0; step < 300; ++step)
    {
        plate.addTorque(Math::vec3(0.0f, 300.0f, 0.0f));
        stepWithJoint(anchor, plate, joint, Math::vec3(0.0f), 1.0f / 60.0f);
        CHECK(finite(plate));
    }
    Math::quat deviation = plate.orientation();
    if (deviation.w < 0.0f)
        deviation = -deviation;
    CHECK(2.0f * std::acos(Math::clamp(deviation.w, -1.0f, 1.0f)) < 0.35f);
}

void testPistonCombinedMotionHoldsItsAxis()
{
    // Sliding under gravity while an angular motor spins it (the strut case): locked directions must not leak.
    RigidBody rail = makeStaticBox(Math::vec3(0.0f));
    RigidBody strut = makeDynamicBox(Math::vec3(0.0f));
    PistonJoint joint(rail, strut, Math::vec3(0.0f), Math::vec3(0.0f, 1.0f, 0.0f));
    joint.setAngularMotor(6.0f, 30.0f);

    for (u32 step = 0; step < 300; ++step)
    {
        stepWithJoint(rail, strut, joint, Math::vec3(3.0f, -10.0f, 2.0f), 1.0f / 60.0f);
        CHECK(finite(strut));
    }
    CHECK(near(strut.position().x, 0.0f, 0.05f));
    CHECK(near(strut.position().z, 0.0f, 0.05f));
    CHECK(near(strut.angularVelocity().y, 6.0f, 0.3f));
    CHECK(near(std::abs(strut.angularVelocity().x), 0.0f, 0.1f));
}

void testMouseJointCarriesABodyToTheTarget()
{
    RigidBody box = makeDynamicBox(Math::vec3(0.0f), 1.0f);
    MouseJoint joint(box, Math::vec3(0.0f));
    joint.setMaxForce(1000.0f);
    joint.tuneSpring(5.0f, 0.7f);
    joint.setTarget(Math::vec3(2.0f, 3.0f, -1.0f));

    RigidBody* bodies[] = {&box};
    Joint* joints[] = {&joint};
    for (u32 step = 0; step < 240; ++step)
        stepJoints(bodies, 1, joints, 1, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(finite(box));
    CHECK(Math::length(box.position() - joint.target()) < 0.15f);
    CHECK(Math::length(box.velocity()) < 0.2f);
}

void testMouseJointForceCapCannotYankABody()
{
    // 5 N cannot lift 1 kg against 10 m/s^2, so the capped joint must lose, not teleport the body.
    RigidBody box = makeDynamicBox(Math::vec3(0.0f), 1.0f);
    MouseJoint joint(box, Math::vec3(0.0f));
    joint.setMaxForce(5.0f);
    joint.tuneSpring(5.0f, 0.7f);
    joint.setTarget(Math::vec3(0.0f, 50.0f, 0.0f));

    RigidBody* bodies[] = {&box};
    Joint* joints[] = {&joint};
    for (u32 step = 0; step < 120; ++step)
        stepJoints(bodies, 1, joints, 1, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(finite(box));
    CHECK(box.position().y < 0.0f);
}

void testMouseJointSurvivesAFarTarget()
{
    RigidBody box = makeDynamicBox(Math::vec3(0.0f), 1.0f);
    MouseJoint joint(box, Math::vec3(0.0f));
    joint.setMaxForce(1000.0f);
    joint.setTarget(Math::vec3(500.0f, 0.0f, 0.0f));

    RigidBody* bodies[] = {&box};
    Joint* joints[] = {&joint};
    for (u32 step = 0; step < 240; ++step)
    {
        stepJoints(bodies, 1, joints, 1, Math::vec3(0.0f), 1.0f / 60.0f);
        CHECK(finite(box));
    }
    CHECK(box.position().x < 520.0f);
}

void testMouseJointGrabsASleepingBodyInAWorld()
{
    // Crate asleep on the floor, then grabbed, through Scene::updatePhysics() with its fixed-step accumulator, as a demo does.
    Radion::Scene world;
    world.setGravity(Math::vec3(0.0f, -9.81f, 0.0f));
    world.setFixedStep(1.0f / 120.0f);

    BoxShape groundShape(Math::vec3(20.0f, 0.5f, 20.0f));
    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::vec3(0.0f, -0.5f, 0.0f));
    ground.setShape(&groundShape);
    ground.setFriction(0.7f);
    world.addBody(ground);

    BoxShape crateShape(Math::vec3(0.4f));
    RigidBody crate;
    crate.setMass(1.5f);
    crate.setInertiaTensor(Inertia::box(1.5f, Math::vec3(0.4f)));
    crate.setPosition(Math::vec3(0.0f, 0.4f, 0.0f));
    crate.setShape(&crateShape);
    crate.setFriction(0.6f);
    world.addBody(crate);

    for (u32 step = 0; step < 600; ++step)
        world.updatePhysics(1.0f / 60.0f);
    CHECK(!crate.awake());

    MouseJoint joint(crate, crate.position() + Math::vec3(0.0f, 0.4f, 0.0f));
    joint.setMaxForce(1000.0f * 1.5f);
    joint.tuneSpring(5.0f, 0.7f);
    world.addJoint(&joint);
    joint.setTarget(crate.position() + Math::vec3(0.0f, 2.0f, 0.0f));

    for (u32 step = 0; step < 240; ++step)
        world.updatePhysics(1.0f / 60.0f);
    world.removeJoint(&joint);

    CHECK(finite(crate));
    // A joint that cannot wake a sleeping body leaves the crate where it slept.
    CHECK(crate.position().y > 1.0f);
}

// A loose joint's destructor must unregister it from its Scene (Scene::addJoint(), no GameObject), or the Scene's list dangles.
void testLoosePointJointDeregistersOnDestruction()
{
    RigidBody a = makeStaticBox(Math::vec3(0.0f));
    RigidBody b = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f));
    Radion::Scene world;
    world.addBody(a);
    world.addBody(b);
    CHECK(world.jointCount() == 0);
    {
        PointJoint joint(a, b, Math::vec3(0.5f, 0.0f, 0.0f));
        world.addJoint(&joint);
        CHECK(world.jointCount() == 1);
    }
    CHECK(world.jointCount() == 0);
}

// std::vector<PointJoint> reallocation runs the move constructor; reserve() upfront is the supported way to keep joints registered (next test: without it).
void testPointJointVectorGrowthKeepsSceneRegistrationCorrect()
{
    Radion::Scene world;
    world.setGravity(Math::vec3(0.0f, -10.0f, 0.0f));

    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    world.addBody(anchor);

    std::vector<RigidBody> links;
    links.reserve(6);
    std::vector<PointJoint> joints;
    joints.reserve(6);
    for (u32 i = 0; i < 6; ++i)
    {
        links.push_back(makeDynamicBox(Math::vec3(static_cast<f32>(i + 1), 0.0f, 0.0f)));
        world.addBody(links.back());
        RigidBody& previous = i == 0 ? anchor : links[i - 1];
        joints.emplace_back(previous, links.back(),
                            Math::vec3(static_cast<f32>(i) + 0.5f, 0.0f, 0.0f));
        world.addJoint(&joints.back());
    }
    CHECK(world.jointCount() == 6);

    for (u32 step = 0; step < 300; ++step)
        world.stepPhysics(1.0f / 60.0f);

    CHECK(world.jointCount() == 6);
    for (const RigidBody& link : links)
        CHECK(finite(link));
    for (const PointJoint& joint : joints)
        CHECK(Math::length(joint.worldAnchorA() - joint.worldAnchorB()) < 0.1f);
}

// No reserve(): reallocation deregisters moved joints (Joint::moveJointStateFrom()); must fail SAFELY: no stale pointer in stepPhysics(), count matches survivors.
void testPointJointVectorGrowthWithoutReserveDropsRegistrationSafely()
{
    Radion::Scene world;
    world.setGravity(Math::vec3(0.0f, -10.0f, 0.0f));

    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    world.addBody(anchor);

    std::vector<RigidBody> links;
    links.reserve(6);
    std::vector<PointJoint> joints;
    for (u32 i = 0; i < 6; ++i)
    {
        links.push_back(makeDynamicBox(Math::vec3(static_cast<f32>(i + 1), 0.0f, 0.0f)));
        world.addBody(links.back());
        RigidBody& previous = i == 0 ? anchor : links[i - 1];
        joints.emplace_back(previous, links.back(),
                            Math::vec3(static_cast<f32>(i) + 0.5f, 0.0f, 0.0f));
        world.addJoint(&joints.back());
    }
    CHECK(world.jointCount() <= joints.size());

    for (u32 step = 0; step < 300; ++step)
        world.stepPhysics(1.0f / 60.0f);
    for (const RigidBody& link : links)
        CHECK(finite(link));
}

void testWarmStartDoesNotInjectEnergy()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody weight = makeDynamicBox(Math::vec3(2.0f, 0.0f, 0.0f));
    DistanceJoint joint(anchor, Math::vec3(0.0f), weight, Math::vec3(0.0f), 2.0f, 2.0f);

    f32 maxSpeed = 0.0f;
    for (u32 i = 0; i < 600; ++i)
    {
        stepWithJoint(anchor, weight, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
        maxSpeed = Math::max(maxSpeed, Math::length(weight.velocity()));
    }

    // A pendulum released from rest at radius 2 under g=10 cannot exceed the
    // speed of a free fall through the same 2m drop: v = sqrt(2 g h) ~= 6.32.
    CHECK(maxSpeed < 8.0f);
}

RigidBody makeChassis()
{
    RigidBody body;
    body.setBodyType(BodyType::Static);
    body.setPosition(Math::vec3(0.0f, 1.0f, 0.0f));
    return body;
}

RigidBody makeWheel(const Math::vec3& position)
{
    RigidBody body;
    body.setBodyType(BodyType::Dynamic);
    body.setPosition(position);
    body.setMass(15.0f);
    body.setInertiaTensor(Inertia::box(15.0f, Math::vec3(0.3f, 0.3f, 0.3f)));
    // No Scene manages the joint's sleep island, so keep the body awake or it sleeps on settling and the spring nudges a velocity that is no longer integrated.
    body.setCanSleep(false);
    return body;
}

void testSuspensionSettlesAtRestLength()
{
    RigidBody chassis = makeChassis();
    RigidBody wheel = makeWheel(Math::vec3(0.0f, 0.3f, 0.0f));

    // Anchor at the wheel's centre: no lever arm couples free spin into linear velocity.
    WheelJoint wheelJoint(chassis, wheel, wheel.position(), Math::vec3(0.0f, -1.0f, 0.0f),
                         Math::vec3(1.0f, 0.0f, 0.0f));
    wheelJoint.setSuspension(0.5f, 4000.0f, 400.0f);

    const f32 dt = 1.0f / 120.0f;
    for (int i = 0; i < 600; ++i)
    {
        wheel.setAcceleration(Math::vec3(0.0f, -9.81f, 0.0f));
        wheel.integrateForces(dt);
        wheelJoint.setup(dt);
        wheelJoint.warmStart();
        wheelJoint.solveVelocity();
        wheel.integrateVelocity(dt);
        wheelJoint.solvePosition(0.2f);
    }

    CHECK(std::fabs(wheelJoint.suspensionTravel() - 0.529f) < 0.02f);
    CHECK(std::fabs(wheel.velocity().y) < 0.05f);
}

// Real car springs are stiff (~75 kN/m per corner); an explicit spring blows up above k*dt^2/m, a constraint row has no ceiling. Sweeps stiffness by decades.
void testSuspensionHoldsStiffSprings()
{
    const f32 dt = 1.0f / 120.0f;
    bool allSettled = true;

    for (f32 stiffness : {4.0e3f, 4.0e4f, 1.0e5f, 1.0e6f})
    {
        RigidBody chassis = makeChassis();
        RigidBody wheel = makeWheel(Math::vec3(0.0f, 0.3f, 0.0f));
        WheelJoint wheelJoint(chassis, wheel, wheel.position(), Math::vec3(0.0f, -1.0f, 0.0f),
                             Math::vec3(1.0f, 0.0f, 0.0f));
        // Damping at a tenth of stiffness, as the existing settling test, so only one thing changes.
        wheelJoint.setSuspension(0.5f, stiffness, stiffness * 0.1f);

        for (int i = 0; i < 600; ++i)
        {
            wheel.setAcceleration(Math::vec3(0.0f, -9.81f, 0.0f));
            wheel.integrateForces(dt);
            wheelJoint.setup(dt);
            wheelJoint.warmStart();
            wheelJoint.solveVelocity();
            wheel.integrateVelocity(dt);
            wheelJoint.solvePosition(0.2f);
        }

        const f32 travel = wheelJoint.suspensionTravel();
        const f32 speed = std::fabs(wheel.velocity().y);
        const bool settled = std::isfinite(travel) && std::isfinite(speed) && speed < 0.05f;
        allSettled = allSettled && settled;
        std::printf("JointTests: suspension k=%9.0f N/m - travel %.4f m, speed %.5f m/s%s\n",
                    static_cast<double>(stiffness), static_cast<double>(travel),
                    static_cast<double>(speed), settled ? "" : "   NOT SETTLED");
    }

    // Every stiffness must settle; failure means the row stopped being solved implicitly.
    CHECK(allSettled);
}

void testSteeringMotorTurnsWithinLimits()
{
    RigidBody chassis = makeChassis();
    RigidBody wheel = makeWheel(Math::vec3(0.0f, 0.5f, 0.0f));

    WheelJoint wheelJoint(chassis, wheel, wheel.position(), Math::vec3(0.0f, -1.0f, 0.0f),
                         Math::vec3(1.0f, 0.0f, 0.0f));
    wheelJoint.setSuspension(0.5f, 4000.0f, 400.0f);
    wheelJoint.setSteeringLimits(-0.5f, 0.5f);
    wheelJoint.setSteeringMotor(10.0f, 500.0f);

    const f32 dt = 1.0f / 120.0f;
    for (int i = 0; i < 240; ++i)
    {
        wheel.setAcceleration(Math::vec3(0.0f, -9.81f, 0.0f));
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

// A zero axis normalises to NaN and spreads through the impulses into both bodies; setters refused it, constructors must too.
void testJointsRejectDegenerateAxes()
{
    const Math::vec3 zero(0.0f);
    const f32 dt = 1.0f / 60.0f;

    {
        RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
        RigidBody arm = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
        HingeJoint joint(anchor, arm, Math::vec3(0.5f, 0.0f, 0.0f), zero);
        joint.setMotor(2.0f, 100.0f);
        for (u32 i = 0; i < 120; ++i)
            stepWithJoint(anchor, arm, joint, Math::vec3(0.0f, -10.0f, 0.0f), dt);
        CHECK(finite(arm));
        CHECK(finiteVec(arm.velocity()));
        CHECK(finiteVec(arm.angularVelocity()));
        CHECK(std::isfinite(joint.currentAngle()));
    }

    {
        RigidBody rail = makeStaticBox(Math::vec3(0.0f));
        RigidBody carriage = makeDynamicBox(Math::vec3(0.0f));
        PistonJoint joint(rail, carriage, Math::vec3(0.0f), zero);
        for (u32 i = 0; i < 120; ++i)
            stepWithJoint(rail, carriage, joint, Math::vec3(0.0f, -10.0f, 0.0f), dt);
        CHECK(finite(carriage));
        CHECK(finiteVec(carriage.velocity()));
    }

    {
        RigidBody chassis = makeChassis();
        RigidBody wheel = makeWheel(Math::vec3(0.0f, 0.5f, 0.0f));
        WheelJoint joint(chassis, wheel, wheel.position(), zero, zero);
        joint.setSuspension(0.5f, 4000.0f, 400.0f);
        for (int i = 0; i < 120; ++i)
        {
            wheel.setAcceleration(Math::vec3(0.0f, -9.81f, 0.0f));
            wheel.integrateForces(dt);
            joint.setup(dt);
            joint.warmStart();
            joint.solveVelocity();
            wheel.integrateVelocity(dt);
            joint.solvePosition(0.2f);
        }
        CHECK(finite(wheel));
        CHECK(finiteVec(wheel.velocity()));
        CHECK(std::isfinite(joint.suspensionTravel()));
    }
}

void testServoRejectsNonFiniteInput()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody arm = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
    HingeJoint joint(anchor, arm, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f));

    joint.setServo(0.5f, 100.0f, 2.0f);
    const f32 good = joint.servoTargetAngle();

    const f32 nan = std::numeric_limits<f32>::quiet_NaN();
    const f32 inf = std::numeric_limits<f32>::infinity();
    joint.setServo(nan, 100.0f, 2.0f);
    CHECK(joint.servoTargetAngle() == good);
    joint.setServo(1.0f, nan, 2.0f);
    CHECK(joint.servoTargetAngle() == good);
    joint.setServo(1.0f, 100.0f, inf);
    CHECK(joint.servoTargetAngle() == good);

    for (u32 i = 0; i < 120; ++i)
        stepWithJoint(anchor, arm, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
    CHECK(std::isfinite(joint.currentAngle()));
    CHECK(finiteVec(arm.angularVelocity()));
}

void testServoWithNoTorqueIsOff()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody arm = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
    HingeJoint joint(anchor, arm, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f));

    joint.setServo(1.0f, 0.0f);
    CHECK(!joint.servoEnabled());
    CHECK(!joint.motorEnabled());

    for (u32 i = 0; i < 240; ++i)
        stepWithJoint(anchor, arm, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
    CHECK(joint.currentAngle() < 0.0f);
    CHECK(std::isfinite(joint.currentAngle()));
}

void testServoSurvivesZeroTimestep()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody arm = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
    HingeJoint joint(anchor, arm, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f));
    joint.setServo(0.5f, 200.0f, 2.0f);

    for (u32 i = 0; i < 10; ++i)
        stepWithJoint(anchor, arm, joint, Math::vec3(0.0f, -10.0f, 0.0f), 0.0f);

    CHECK(std::isfinite(joint.currentAngle()));
    CHECK(finiteVec(arm.angularVelocity()));
    CHECK(finiteVec(arm.position()));
}

// setLimits() clamps each side into its own half, so inverted limits open no hole; no spin or NaN either.
void testServoWithInvertedLimits()
{
    RigidBody anchor = makeStaticBox(Math::vec3(0.0f));
    RigidBody arm = makeDynamicBox(Math::vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::vec3(0.5f));
    HingeJoint joint(anchor, arm, Math::vec3(0.5f, 0.0f, 0.0f), Math::vec3(0.0f, 0.0f, 1.0f));

    joint.setLimits(0.8f, -0.8f);
    joint.setServo(0.4f, 200.0f, 2.0f);

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(anchor, arm, joint, Math::vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(std::isfinite(joint.currentAngle()));
    CHECK(std::abs(joint.currentAngle()) < Math::pi<f32>());
    CHECK(finiteVec(arm.angularVelocity()));
}

// Damping without spring: a pure damper, a valid setup.
void testSuspensionDamperWithoutSpring()
{
    RigidBody chassis = makeChassis();
    RigidBody wheel = makeWheel(Math::vec3(0.0f, 0.3f, 0.0f));
    WheelJoint wheelJoint(chassis, wheel, wheel.position(), Math::vec3(0.0f, -1.0f, 0.0f),
                         Math::vec3(1.0f, 0.0f, 0.0f));
    wheelJoint.setSuspension(0.5f, 0.0f, 500.0f);

    const f32 dt = 1.0f / 120.0f;
    for (int i = 0; i < 240; ++i)
    {
        wheel.setAcceleration(Math::vec3(0.0f, -9.81f, 0.0f));
        wheel.integrateForces(dt);
        wheelJoint.setup(dt);
        wheelJoint.warmStart();
        wheelJoint.solveVelocity();
        wheel.integrateVelocity(dt);
        wheelJoint.solvePosition(0.2f);
    }

    CHECK(std::isfinite(wheelJoint.suspensionTravel()));
    CHECK(finiteVec(wheel.position()));
}

// Parallel wheel axes, which the constructor's comment says to avoid: must degrade, not explode.
void testWheelJointWithDegenerateAxes()
{
    RigidBody chassis = makeChassis();
    RigidBody wheel = makeWheel(Math::vec3(0.0f, 0.5f, 0.0f));
    WheelJoint wheelJoint(chassis, wheel, wheel.position(), Math::vec3(0.0f, -1.0f, 0.0f),
                         Math::vec3(0.0f, -1.0f, 0.0f));
    wheelJoint.setSuspension(0.5f, 4000.0f, 400.0f);
    wheelJoint.setSpinMotor(10.0f, 100.0f);

    const f32 dt = 1.0f / 120.0f;
    for (int i = 0; i < 120; ++i)
    {
        wheel.setAcceleration(Math::vec3(0.0f, -9.81f, 0.0f));
        wheel.integrateForces(dt);
        wheelJoint.setup(dt);
        wheelJoint.warmStart();
        wheelJoint.solveVelocity();
        wheel.integrateVelocity(dt);
        wheelJoint.solvePosition(0.2f);
    }

    CHECK(finiteVec(wheel.position()));
    CHECK(finiteVec(wheel.velocity()));
    CHECK(finiteVec(wheel.angularVelocity()));
}

void testSteeringServoHoldsCommandedAngle()
{
    RigidBody chassis = makeChassis();
    RigidBody wheel = makeWheel(Math::vec3(0.0f, 0.5f, 0.0f));

    WheelJoint wheelJoint(chassis, wheel, wheel.position(), Math::vec3(0.0f, -1.0f, 0.0f),
                         Math::vec3(1.0f, 0.0f, 0.0f));
    wheelJoint.setSuspension(0.5f, 4000.0f, 400.0f);
    wheelJoint.setSteeringLimits(Math::radians(-35.0f), Math::radians(35.0f));

    const f32 dt = 1.0f / 120.0f;
    const auto drive = [&](int steps) {
        for (int i = 0; i < steps; ++i)
        {
            wheel.setAcceleration(Math::vec3(0.0f, -9.81f, 0.0f));
            wheel.integrateForces(dt);
            wheelJoint.setup(dt);
            wheelJoint.warmStart();
            wheelJoint.solveVelocity();
            wheel.integrateVelocity(dt);
            wheelJoint.solvePosition(0.2f);
        }
    };

    wheelJoint.setSteeringServo(Math::radians(20.0f), 600.0f, 4.0f);
    CHECK(wheelJoint.steeringServoEnabled());
    drive(240);
    CHECK(near(wheelJoint.steeringAngle(), Math::radians(20.0f), 0.02f));

    drive(240);
    CHECK(near(wheelJoint.steeringAngle(), Math::radians(20.0f), 0.02f));

    wheelJoint.setSteeringServo(Math::radians(-25.0f), 600.0f, 4.0f);
    drive(360);
    CHECK(near(wheelJoint.steeringAngle(), Math::radians(-25.0f), 0.02f));

    wheelJoint.setSteeringServo(Math::radians(80.0f), 600.0f, 4.0f);
    drive(480);
    CHECK(wheelJoint.steeringAngle() <= Math::radians(36.0f));
    CHECK(wheelJoint.steeringAngle() > Math::radians(30.0f));
}

void testSpinMotorDrivesWheelWithoutLimit()
{
    RigidBody chassis = makeChassis();
    RigidBody wheel = makeWheel(Math::vec3(0.0f, 0.5f, 0.0f));

    WheelJoint wheelJoint(chassis, wheel, wheel.position(), Math::vec3(0.0f, -1.0f, 0.0f),
                         Math::vec3(1.0f, 0.0f, 0.0f));
    wheelJoint.setSuspension(0.5f, 4000.0f, 400.0f);
    wheelJoint.setSpinMotor(20.0f, 200.0f);

    const f32 dt = 1.0f / 120.0f;
    for (int i = 0; i < 120; ++i)
    {
        wheel.setAcceleration(Math::vec3(0.0f, -9.81f, 0.0f));
        wheel.integrateForces(dt);
        wheelJoint.setup(dt);
        wheelJoint.warmStart();
        wheelJoint.solveVelocity();
        wheel.integrateVelocity(dt);
        wheelJoint.solvePosition(0.2f);
    }

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
    testSuspensionHoldsStiffSprings();
    testSteeringServoHoldsCommandedAngle();
    testJointsRejectDegenerateAxes();
    testServoRejectsNonFiniteInput();
    testServoWithNoTorqueIsOff();
    testServoSurvivesZeroTimestep();
    testServoWithInvertedLimits();
    testSuspensionDamperWithoutSpring();
    testWheelJointWithDegenerateAxes();
    testSteeringMotorTurnsWithinLimits();
    testSpinMotorDrivesWheelWithoutLimit();
    testLoosePointJointDeregistersOnDestruction();
    testPointJointVectorGrowthKeepsSceneRegistrationCorrect();
    testPointJointVectorGrowthWithoutReserveDropsRegistrationSafely();
    if (gFailures)
        std::fprintf(stderr, "%d joint test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
