#include "PCH.h"

#include "collision/CollisionShape.h"
#include "dynamics/ContactSolver.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/DistanceJoint.h"
#include "dynamics/FixedJoint.h"
#include "dynamics/HingeJoint.h"
#include "dynamics/MouseJoint.h"
#include "dynamics/PistonJoint.h"
#include "dynamics/PointJoint.h"
#include "dynamics/RigidBody.h"
#include "dynamics/SliderJoint.h"
#include "dynamics/UniversalJoint.h"

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

bool near(const Math::Vec3& a, const Math::Vec3& b, f32 epsilon = 1e-3f)
{
    return glm::length(a - b) <= epsilon;
}

RigidBody makeDynamicBox(const Math::Vec3& position, f32 mass = 1.0f,
                         const Math::Vec3& halfExtents = Math::Vec3(0.5f))
{
    RigidBody body;
    body.setPosition(position);
    body.setMass(mass);
    body.setInertiaTensor(Inertia::box(mass, halfExtents));
    body.setDamping(1.0f, 1.0f);
    body.setCanSleep(false);
    return body;
}

RigidBody makeStaticBox(const Math::Vec3& position)
{
    RigidBody body;
    body.setPosition(position);
    body.setBodyType(BodyType::Static);
    return body;
}

void stepWithJoint(RigidBody& a, RigidBody& b, Joint& joint, const Math::Vec3& gravity, f32 duration,
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
                const Math::Vec3& gravity, f32 duration)
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
    RigidBody anchor = makeStaticBox(Math::Vec3(0.0f));
    RigidBody weight = makeDynamicBox(Math::Vec3(2.0f, 0.0f, 0.0f));
    DistanceJoint joint(anchor, Math::Vec3(0.0f), weight, Math::Vec3(0.0f), 2.0f, 2.0f);

    for (u32 i = 0; i < 600; ++i)
        stepWithJoint(anchor, weight, joint, Math::Vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    const f32 length = glm::length(weight.position() - anchor.position());
    CHECK(near(length, 2.0f, 0.02f));
}

void testDistanceJointRangeAllowsSlack()
{
    RigidBody anchor = makeStaticBox(Math::Vec3(0.0f));
    RigidBody weight = makeDynamicBox(Math::Vec3(0.5f, 0.0f, 0.0f));
    DistanceJoint joint(anchor, Math::Vec3(0.0f), weight, Math::Vec3(0.0f), 0.0f, 2.0f);

    for (u32 i = 0; i < 6; ++i)
        stepWithJoint(anchor, weight, joint, Math::Vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    // Well inside the range, gravity must pull it down unopposed.
    CHECK(weight.position().y < -0.01f);

    for (u32 i = 0; i < 600; ++i)
        stepWithJoint(anchor, weight, joint, Math::Vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    const f32 length = glm::length(weight.position() - anchor.position());
    CHECK(length <= 2.02f);
    CHECK(near(length, 2.0f, 0.02f));
}

// ----------------------------------------------------------------- FixedJoint

void testFixedJointLocksAllSixDOF()
{
    RigidBody anchor = makeStaticBox(Math::Vec3(0.0f));
    RigidBody plate = makeDynamicBox(Math::Vec3(1.0f, 0.0f, 0.0f));
    FixedJoint joint(anchor, plate, Math::Vec3(0.5f, 0.0f, 0.0f));

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(anchor, plate, joint, Math::Vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(near(plate.position(), Math::Vec3(1.0f, 0.0f, 0.0f), 0.02f));
    CHECK(near(glm::length(plate.orientation() - Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f)), 0.0f, 0.05f));
}

// ------------------------------------------------------------------ HingeJoint

void testHingeJointFreeAboutItsAxisOnly()
{
    RigidBody anchor = makeStaticBox(Math::Vec3(0.0f));
    RigidBody arm = makeDynamicBox(Math::Vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::Vec3(0.5f));
    HingeJoint joint(anchor, arm, Math::Vec3(0.5f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 1.0f));

    arm.addTorque(Math::Vec3(0.0f, 5.0f, 0.0f));
    for (u32 i = 0; i < 30; ++i)
        stepWithJoint(anchor, arm, joint, Math::Vec3(0.0f), 1.0f / 60.0f);

    // A torque about a locked axis (y) must not spin the body about that axis.
    CHECK(near(arm.angularVelocity().y, 0.0f, 0.05f));

    arm.addTorque(Math::Vec3(0.0f, 0.0f, 5.0f));
    for (u32 i = 0; i < 30; ++i)
        stepWithJoint(anchor, arm, joint, Math::Vec3(0.0f), 1.0f / 60.0f);

    // The hinge axis itself (z) is free to spin.
    CHECK(std::abs(arm.angularVelocity().z) > 0.05f);
}

void testHingeJointMotorDrivesToTargetVelocity()
{
    RigidBody anchor = makeStaticBox(Math::Vec3(0.0f));
    RigidBody wheel = makeDynamicBox(Math::Vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::Vec3(0.5f));
    HingeJoint joint(anchor, wheel, Math::Vec3(0.5f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 1.0f));
    joint.setMotor(4.0f, 20.0f);

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(anchor, wheel, joint, Math::Vec3(0.0f), 1.0f / 60.0f);

    CHECK(near(wheel.angularVelocity().z, 4.0f, 0.1f));
}

void testHingeJointKeepsAnchorTogether()
{
    RigidBody anchor = makeStaticBox(Math::Vec3(0.0f));
    RigidBody arm = makeDynamicBox(Math::Vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::Vec3(0.5f));
    HingeJoint joint(anchor, arm, Math::Vec3(0.5f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 1.0f));

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(anchor, arm, joint, Math::Vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    const Math::Vec3 hingePointOnArm = arm.pointToWorld(Math::Vec3(-0.5f, 0.0f, 0.0f));
    CHECK(near(hingePointOnArm, Math::Vec3(0.5f, 0.0f, 0.0f), 0.05f));
}

// ----------------------------------------------------------------- SliderJoint

void testSliderJointMovesOnlyAlongItsAxis()
{
    RigidBody rail = makeStaticBox(Math::Vec3(0.0f));
    RigidBody carriage = makeDynamicBox(Math::Vec3(0.0f, -1.0f, 0.0f));
    SliderJoint joint(rail, carriage, Math::Vec3(0.0f, -1.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f));

    for (u32 i = 0; i < 60; ++i)
        stepWithJoint(rail, carriage, joint, Math::Vec3(4.0f, -10.0f, 3.0f), 1.0f / 60.0f);

    // Gravity's x/z components must not move the carriage off the rail.
    CHECK(near(carriage.position().x, 0.0f, 0.02f));
    CHECK(near(carriage.position().z, 0.0f, 0.02f));
    // The y component, along the rail, must have fallen freely.
    CHECK(carriage.position().y < -1.0f);
    CHECK(near(glm::length(carriage.orientation() - Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f)), 0.0f,
              0.02f));
}

void testSliderJointLimitsClampTravel()
{
    RigidBody rail = makeStaticBox(Math::Vec3(0.0f));
    RigidBody piston = makeDynamicBox(Math::Vec3(0.0f, -0.2f, 0.0f));
    SliderJoint joint(rail, piston, Math::Vec3(0.0f, -0.2f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f));
    joint.setLimits(-1.0f, 0.0f);

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(rail, piston, joint, Math::Vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    // The limit clamps the slide displacement, not the absolute position -
    // the piston started at y=-0.2, so the floor sits at -0.2 + (-1.0).
    CHECK(joint.currentPosition() >= -1.02f);
    CHECK(near(joint.currentPosition(), -1.0f, 0.03f));
    CHECK(near(piston.position().y, -1.2f, 0.03f));
}

void testSliderJointMotorDrivesToTargetVelocity()
{
    RigidBody rail = makeStaticBox(Math::Vec3(0.0f));
    RigidBody piston = makeDynamicBox(Math::Vec3(0.0f));
    SliderJoint joint(rail, piston, Math::Vec3(0.0f), Math::Vec3(1.0f, 0.0f, 0.0f));
    joint.setMotor(3.0f, 50.0f);

    for (u32 i = 0; i < 120; ++i)
        stepWithJoint(rail, piston, joint, Math::Vec3(0.0f), 1.0f / 60.0f);

    CHECK(near(piston.velocity().x, 3.0f, 0.05f));
}

// ----------------------------------------------------------------- PistonJoint

void testPistonJointMovesAndSpinsOnlyAlongItsAxis()
{
    RigidBody rail = makeStaticBox(Math::Vec3(0.0f));
    RigidBody strut = makeDynamicBox(Math::Vec3(0.0f, -1.0f, 0.0f));
    PistonJoint joint(rail, strut, Math::Vec3(0.0f, -1.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f));

    strut.addTorque(Math::Vec3(3.0f, 0.0f, 0.0f));
    for (u32 i = 0; i < 60; ++i)
        stepWithJoint(rail, strut, joint, Math::Vec3(4.0f, -10.0f, 3.0f), 1.0f / 60.0f);

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
    RigidBody rail = makeStaticBox(Math::Vec3(0.0f));
    RigidBody strut = makeDynamicBox(Math::Vec3(0.0f));
    PistonJoint joint(rail, strut, Math::Vec3(0.0f), Math::Vec3(0.0f, 1.0f, 0.0f));
    joint.setLinearMotor(2.0f, 40.0f);
    joint.setAngularLimits(-0.2f, 0.2f);

    strut.addTorque(Math::Vec3(0.0f, 6.0f, 0.0f));
    for (u32 i = 0; i < 240; ++i)
        stepWithJoint(rail, strut, joint, Math::Vec3(0.0f), 1.0f / 60.0f);

    CHECK(near(strut.velocity().y, 2.0f, 0.05f));
    CHECK(joint.currentAngle() <= 0.22f);
    CHECK(joint.currentAngle() >= -0.22f);
}

// -------------------------------------------------------------- UniversalJoint

void testUniversalJointFreeAboutBothAxesOnly()
{
    RigidBody yoke = makeStaticBox(Math::Vec3(0.0f));
    RigidBody shaft = makeDynamicBox(Math::Vec3(1.0f, 0.0f, 0.0f));
    UniversalJoint joint(yoke, shaft, Math::Vec3(0.5f, 0.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f),
                        Math::Vec3(0.0f, 0.0f, 1.0f));

    // A torque about the shared perpendicular direction (x, locked by the
    // ball-and-socket point but not one of the two free hinge axes) must
    // not build up angular velocity there.
    shaft.addTorque(Math::Vec3(5.0f, 0.0f, 0.0f));
    for (u32 i = 0; i < 30; ++i)
        stepWithJoint(yoke, shaft, joint, Math::Vec3(0.0f), 1.0f / 60.0f);
    CHECK(near(shaft.angularVelocity().x, 0.0f, 0.05f));

    shaft.addTorque(Math::Vec3(0.0f, 5.0f, 0.0f));
    for (u32 i = 0; i < 30; ++i)
        stepWithJoint(yoke, shaft, joint, Math::Vec3(0.0f), 1.0f / 60.0f);
    CHECK(std::abs(shaft.angularVelocity().y) > 0.05f);
}

void testUniversalJointKeepsAnchorTogether()
{
    RigidBody yoke = makeStaticBox(Math::Vec3(0.0f));
    RigidBody shaft = makeDynamicBox(Math::Vec3(1.0f, 0.0f, 0.0f));
    UniversalJoint joint(yoke, shaft, Math::Vec3(0.5f, 0.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f),
                        Math::Vec3(0.0f, 0.0f, 1.0f));

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(yoke, shaft, joint, Math::Vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    const Math::Vec3 anchorOnShaft = shaft.pointToWorld(Math::Vec3(-0.5f, 0.0f, 0.0f));
    CHECK(near(anchorOnShaft, Math::Vec3(0.5f, 0.0f, 0.0f), 0.05f));
}

void testUniversalJointMotorDrivesToTargetVelocity()
{
    RigidBody yoke = makeStaticBox(Math::Vec3(0.0f));
    RigidBody shaft = makeDynamicBox(Math::Vec3(1.0f, 0.0f, 0.0f));
    UniversalJoint joint(yoke, shaft, Math::Vec3(0.5f, 0.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f),
                        Math::Vec3(0.0f, 0.0f, 1.0f));
    joint.setMotorA(3.0f, 30.0f);

    for (u32 i = 0; i < 300; ++i)
        stepWithJoint(yoke, shaft, joint, Math::Vec3(0.0f), 1.0f / 60.0f);

    CHECK(near(shaft.angularVelocity().y, 3.0f, 0.1f));
}

// ------------------------------------------------------------------ stress

void testChainOfPointJointsHangsWithoutStretching()
{
    // Eight links hanging from a static anchor - the configuration every
    // ragdoll limb reduces to. A chain solved link by link is where drift
    // and stretch pile up first.
    RigidBody anchor = makeStaticBox(Math::Vec3(0.0f));
    std::vector<RigidBody> links(8);
    std::vector<RigidBody*> bodies = {&anchor};
    for (u32 i = 0; i < links.size(); ++i)
    {
        links[i] = makeDynamicBox(Math::Vec3(static_cast<f32>(i + 1), 0.0f, 0.0f), 1.0f,
                                  Math::Vec3(0.4f));
        bodies.push_back(&links[i]);
    }
    std::vector<PointJoint> joints;
    joints.reserve(8);
    joints.emplace_back(anchor, links[0], Math::Vec3(0.5f, 0.0f, 0.0f));
    for (u32 i = 1; i < links.size(); ++i)
        joints.emplace_back(links[i - 1], links[i],
                            Math::Vec3(static_cast<f32>(i) + 0.5f, 0.0f, 0.0f));
    std::vector<Joint*> jointPointers;
    for (PointJoint& joint : joints)
        jointPointers.push_back(&joint);

    for (u32 step = 0; step < 600; ++step)
        stepJoints(bodies.data(), static_cast<u32>(bodies.size()), jointPointers.data(),
                   static_cast<u32>(jointPointers.size()), Math::Vec3(0.0f, -10.0f, 0.0f),
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
    RigidBody anchor = makeStaticBox(Math::Vec3(0.0f));
    RigidBody light = makeDynamicBox(Math::Vec3(1.0f, 0.0f, 0.0f), 0.05f, Math::Vec3(0.3f));
    RigidBody heavy = makeDynamicBox(Math::Vec3(2.0f, 0.0f, 0.0f), 5.0f, Math::Vec3(0.5f));
    PointJoint first(anchor, light, Math::Vec3(0.5f, 0.0f, 0.0f));
    PointJoint second(light, heavy, Math::Vec3(1.5f, 0.0f, 0.0f));
    RigidBody* bodies[] = {&anchor, &light, &heavy};
    Joint* joints[] = {&first, &second};

    for (u32 step = 0; step < 600; ++step)
        stepJoints(bodies, 3, joints, 2, Math::Vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(finite(light));
    CHECK(finite(heavy));
    CHECK(glm::length(first.worldAnchorA() - first.worldAnchorB()) < 0.1f);
    CHECK(glm::length(second.worldAnchorA() - second.worldAnchorB()) < 0.1f);
}

void testHingeLimitSurvivesBeingHammered()
{
    RigidBody anchor = makeStaticBox(Math::Vec3(0.0f));
    RigidBody arm = makeDynamicBox(Math::Vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::Vec3(0.5f));
    HingeJoint joint(anchor, arm, Math::Vec3(0.5f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 1.0f));
    joint.setLimits(-0.5f, 0.5f);

    // Fifty radians per second straight into the limit, repeatedly.
    for (u32 burst = 0; burst < 4; ++burst)
    {
        arm.setAngularVelocity(Math::Vec3(0.0f, 0.0f, burst % 2 == 0 ? 50.0f : -50.0f));
        for (u32 step = 0; step < 60; ++step)
        {
            stepWithJoint(anchor, arm, joint, Math::Vec3(0.0f), 1.0f / 60.0f);
            CHECK(finite(arm));
        }
        CHECK(joint.currentAngle() > -0.7f);
        CHECK(joint.currentAngle() < 0.7f);
    }
}

void testHingeMotorAgainstLimitHoldsAtLimit()
{
    RigidBody anchor = makeStaticBox(Math::Vec3(0.0f));
    RigidBody arm = makeDynamicBox(Math::Vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::Vec3(0.5f));
    HingeJoint joint(anchor, arm, Math::Vec3(0.5f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 1.0f));
    joint.setLimits(-0.3f, 0.3f);
    joint.setMotor(10.0f, 50.0f);

    for (u32 step = 0; step < 300; ++step)
        stepWithJoint(anchor, arm, joint, Math::Vec3(0.0f), 1.0f / 60.0f);

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
    const Math::Vec3 axes[] = {Math::Vec3(0.0f, 1.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 1.0f),
                              Math::Vec3(1.0f, 0.0f, 0.0f)};
    for (const Math::Vec3& axis : axes)
    {
        RigidBody anchor = makeStaticBox(Math::Vec3(0.0f));
        RigidBody shaft = makeDynamicBox(Math::Vec3(1.0f, 0.0f, 0.0f));
        UniversalJoint joint(anchor, shaft, Math::Vec3(0.5f, 0.0f, 0.0f), axis, axis);
        for (u32 step = 0; step < 120; ++step)
            stepWithJoint(anchor, shaft, joint, Math::Vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
        CHECK(finite(shaft));
    }
}

void testSliderSurvivesASidewaysWhack()
{
    RigidBody rail = makeStaticBox(Math::Vec3(0.0f));
    RigidBody carriage = makeDynamicBox(Math::Vec3(0.0f));
    SliderJoint joint(rail, carriage, Math::Vec3(0.0f), Math::Vec3(0.0f, 1.0f, 0.0f));

    carriage.applyImpulseAtPoint(Math::Vec3(50.0f, 0.0f, 30.0f),
                                 carriage.position() + Math::Vec3(0.0f, 0.4f, 0.0f));
    for (u32 step = 0; step < 300; ++step)
        stepWithJoint(rail, carriage, joint, Math::Vec3(0.0f), 1.0f / 60.0f);

    CHECK(finite(carriage));
    CHECK(near(carriage.position().x, 0.0f, 0.05f));
    CHECK(near(carriage.position().z, 0.0f, 0.05f));
}

void testPointJointRecoversFromATeleport()
{
    RigidBody anchor = makeStaticBox(Math::Vec3(0.0f));
    RigidBody weight = makeDynamicBox(Math::Vec3(1.0f, 0.0f, 0.0f));
    PointJoint joint(anchor, weight, Math::Vec3(0.5f, 0.0f, 0.0f));

    for (u32 step = 0; step < 60; ++step)
        stepWithJoint(anchor, weight, joint, Math::Vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    // Someone moves the body by hand, ten metres away - a scene edit, a
    // respawn. The joint has to reel it back in instead of detonating.
    weight.setPosition(weight.position() + Math::Vec3(10.0f, 5.0f, -3.0f));
    for (u32 step = 0; step < 300; ++step)
    {
        stepWithJoint(anchor, weight, joint, Math::Vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
        CHECK(finite(weight));
    }
    CHECK(glm::length(joint.worldAnchorA() - joint.worldAnchorB()) < 0.1f);
}

void testHingeMotorSurvivesVaryingTimestep()
{
    RigidBody anchor = makeStaticBox(Math::Vec3(0.0f));
    RigidBody wheel = makeDynamicBox(Math::Vec3(1.0f, 0.0f, 0.0f), 1.0f, Math::Vec3(0.5f));
    HingeJoint joint(anchor, wheel, Math::Vec3(0.5f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 1.0f));
    joint.setMotor(4.0f, 20.0f);

    // Frame spikes: the warm-start impulse scaling is exactly what breaks
    // when the step keeps changing.
    for (u32 step = 0; step < 300; ++step)
    {
        const f32 dt = step % 3 == 0 ? 1.0f / 30.0f : step % 3 == 1 ? 1.0f / 240.0f : 1.0f / 60.0f;
        stepWithJoint(anchor, wheel, joint, Math::Vec3(0.0f), dt);
        CHECK(finite(wheel));
    }
    CHECK(near(wheel.angularVelocity().z, 4.0f, 0.3f));
}

void testDynamicPairConservesLinearMomentum()
{
    // Two free bodies joined by a point joint, no gravity: every joint
    // impulse is internal and equal-and-opposite, so the pair's total
    // momentum must not drift.
    RigidBody a = makeDynamicBox(Math::Vec3(0.0f), 2.0f);
    RigidBody b = makeDynamicBox(Math::Vec3(1.0f, 0.0f, 0.0f), 1.0f);
    a.setVelocity(Math::Vec3(3.0f, 1.0f, -2.0f));
    PointJoint joint(a, b, Math::Vec3(0.5f, 0.0f, 0.0f));

    const Math::Vec3 momentumBefore = a.velocity() * 2.0f + b.velocity() * 1.0f;
    for (u32 step = 0; step < 300; ++step)
        stepWithJoint(a, b, joint, Math::Vec3(0.0f), 1.0f / 60.0f);
    const Math::Vec3 momentumAfter = a.velocity() * 2.0f + b.velocity() * 1.0f;

    CHECK(finite(a));
    CHECK(finite(b));
    CHECK(near(momentumBefore, momentumAfter, 0.05f));
}

void testFixedJointHoldsUnderHeavyTorque()
{
    RigidBody anchor = makeStaticBox(Math::Vec3(0.0f));
    RigidBody plate = makeDynamicBox(Math::Vec3(1.0f, 0.0f, 0.0f));
    FixedJoint joint(anchor, plate, Math::Vec3(0.5f, 0.0f, 0.0f));

    for (u32 step = 0; step < 300; ++step)
    {
        plate.addTorque(Math::Vec3(0.0f, 300.0f, 0.0f));
        stepWithJoint(anchor, plate, joint, Math::Vec3(0.0f), 1.0f / 60.0f);
        CHECK(finite(plate));
    }
    // Torque against a weld deflects a little each step and is pulled back;
    // it must not ratchet into a slow spin.
    Math::Quaternion deviation = plate.orientation();
    if (deviation.w < 0.0f)
        deviation = -deviation;
    CHECK(2.0f * std::acos(glm::clamp(deviation.w, -1.0f, 1.0f)) < 0.35f);
}

void testPistonCombinedMotionHoldsItsAxis()
{
    // Both freedoms at once: sliding under gravity while an angular motor
    // spins it - the strut case. The locked directions must not leak.
    RigidBody rail = makeStaticBox(Math::Vec3(0.0f));
    RigidBody strut = makeDynamicBox(Math::Vec3(0.0f));
    PistonJoint joint(rail, strut, Math::Vec3(0.0f), Math::Vec3(0.0f, 1.0f, 0.0f));
    joint.setAngularMotor(6.0f, 30.0f);

    for (u32 step = 0; step < 300; ++step)
    {
        stepWithJoint(rail, strut, joint, Math::Vec3(3.0f, -10.0f, 2.0f), 1.0f / 60.0f);
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
    RigidBody box = makeDynamicBox(Math::Vec3(0.0f), 1.0f);
    MouseJoint joint(box, Math::Vec3(0.0f));
    joint.setMaxForce(1000.0f);
    joint.tuneSpring(5.0f, 0.7f);
    joint.setTarget(Math::Vec3(2.0f, 3.0f, -1.0f));

    RigidBody* bodies[] = {&box};
    Joint* joints[] = {&joint};
    for (u32 step = 0; step < 240; ++step)
        stepJoints(bodies, 1, joints, 1, Math::Vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

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
    RigidBody box = makeDynamicBox(Math::Vec3(0.0f), 1.0f);
    MouseJoint joint(box, Math::Vec3(0.0f));
    joint.setMaxForce(5.0f);
    joint.tuneSpring(5.0f, 0.7f);
    joint.setTarget(Math::Vec3(0.0f, 50.0f, 0.0f));

    RigidBody* bodies[] = {&box};
    Joint* joints[] = {&joint};
    for (u32 step = 0; step < 120; ++step)
        stepJoints(bodies, 1, joints, 1, Math::Vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);

    CHECK(finite(box));
    CHECK(box.position().y < 0.0f);
}

void testMouseJointSurvivesAFarTarget()
{
    RigidBody box = makeDynamicBox(Math::Vec3(0.0f), 1.0f);
    MouseJoint joint(box, Math::Vec3(0.0f));
    joint.setMaxForce(1000.0f);
    joint.setTarget(Math::Vec3(500.0f, 0.0f, 0.0f));

    RigidBody* bodies[] = {&box};
    Joint* joints[] = {&joint};
    for (u32 step = 0; step < 240; ++step)
    {
        stepJoints(bodies, 1, joints, 1, Math::Vec3(0.0f), 1.0f / 60.0f);
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
    // PhysicsWorld::update with its fixed-step accumulator, not through the
    // solver directly, because that is the path a demo actually takes.
    PhysicsWorld world;
    world.setGravity(Math::Vec3(0.0f, -9.81f, 0.0f));
    world.setFixedStep(1.0f / 120.0f);

    BoxShape groundShape(Math::Vec3(20.0f, 0.5f, 20.0f));
    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));
    BodyEntry groundEntry;
    groundEntry.body = &ground;
    groundEntry.shape = &groundShape;
    groundEntry.friction = 0.7f;
    world.addBody(groundEntry);

    BoxShape crateShape(Math::Vec3(0.4f));
    RigidBody crate;
    crate.setMass(1.5f);
    crate.setInertiaTensor(Inertia::box(1.5f, Math::Vec3(0.4f)));
    crate.setPosition(Math::Vec3(0.0f, 0.4f, 0.0f));
    BodyEntry crateEntry;
    crateEntry.body = &crate;
    crateEntry.shape = &crateShape;
    crateEntry.friction = 0.6f;
    world.addBody(crateEntry);

    for (u32 step = 0; step < 600; ++step)
        world.update(1.0f / 60.0f);
    CHECK(!crate.awake());

    MouseJoint joint(crate, crate.position() + Math::Vec3(0.0f, 0.4f, 0.0f));
    joint.setMaxForce(1000.0f * 1.5f);
    joint.tuneSpring(5.0f, 0.7f);
    world.addJoint(&joint);
    joint.setTarget(crate.position() + Math::Vec3(0.0f, 2.0f, 0.0f));

    for (u32 step = 0; step < 240; ++step)
        world.update(1.0f / 60.0f);
    world.removeJoint(&joint);

    CHECK(finite(crate));
    // Grabbed two metres up: a joint that cannot wake a sleeping body leaves
    // the crate exactly where it slept.
    CHECK(crate.position().y > 1.0f);
}

// ------------------------------------------------------------- warm start energy

void testWarmStartDoesNotInjectEnergy()
{
    RigidBody anchor = makeStaticBox(Math::Vec3(0.0f));
    RigidBody weight = makeDynamicBox(Math::Vec3(2.0f, 0.0f, 0.0f));
    DistanceJoint joint(anchor, Math::Vec3(0.0f), weight, Math::Vec3(0.0f), 2.0f, 2.0f);

    f32 maxSpeed = 0.0f;
    for (u32 i = 0; i < 600; ++i)
    {
        stepWithJoint(anchor, weight, joint, Math::Vec3(0.0f, -10.0f, 0.0f), 1.0f / 60.0f);
        maxSpeed = glm::max(maxSpeed, glm::length(weight.velocity()));
    }

    // A pendulum released from rest at radius 2 under g=10 cannot exceed the
    // speed of a free fall through the same 2m drop: v = sqrt(2 g h) ~= 6.32.
    CHECK(maxSpeed < 8.0f);
}

} // namespace

int main()
{
    testDistanceJointHoldsFixedLength();
    testDistanceJointRangeAllowsSlack();
    testFixedJointLocksAllSixDOF();
    testHingeJointFreeAboutItsAxisOnly();
    testHingeJointMotorDrivesToTargetVelocity();
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
    if (gFailures)
        std::fprintf(stderr, "%d joint test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
