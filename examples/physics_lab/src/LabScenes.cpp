#include "LabScenes.h"

#include "Agent.h"
#include "AssetManager.h"
#include "Camera.h"
#include "CameraControllers.h"
#include "GameObject.h"
#include "Light.h"
#include "MeshRenderer.h"
#include "ObstacleComponent.h"
#include "Scene.h"
#include "Steering.h"
#include "dynamics/DistanceJoint.h"
#include "dynamics/HingeJoint.h"
#include "dynamics/PistonJoint.h"
#include "dynamics/PointJoint.h"
#include "dynamics/RigidBody.h"
#include "dynamics/SliderJoint.h"
#include "dynamics/UniversalJoint.h"
#include "dynamics/WheelJoint.h"

#include <cmath>

namespace Radion::Lab
{

// The joints live in Physics::; the components this demo puts on objects do
// not, so both names are in play here and the shorter one wins for the ones
// used most.
using Physics::DistanceJoint;
using Physics::HingeJoint;
using Physics::PistonJoint;
using Physics::PointJoint;
using Physics::SliderJoint;
using Physics::UniversalJoint;
using Physics::WheelJoint;

namespace
{

// Every body in here is a primitive, and the visual is built to match the
// collider rather than loaded - so anything that looks wrong is the physics
// and not an asset.
GameObject* makeBox(Scene& scene, const char* name, const glm::vec3& position,
                    const glm::vec3& halfExtents, f32 mass)
{
    GameObject* object = scene.createGameObject(name);
    object->setPosition(position);
    if (MeshRenderer* renderer = object->addComponent<MeshRenderer>())
        renderer->setMesh(Assets().createMesh(MeshDesc::box(halfExtents * 2.0f)));

    Physics::RigidBody* body = object->addComponent<Physics::RigidBody>();
    body->setBox(halfExtents);
    if (mass > 0.0f)
    {
        body->setMass(mass);
        body->setInertiaTensor(Physics::Inertia::box(mass, halfExtents));
    }
    else
        body->setBodyType(Physics::BodyType::Static);
    return object;
}

GameObject* makeSphere(Scene& scene, const char* name, const glm::vec3& position, f32 radius,
                       f32 mass)
{
    GameObject* object = scene.createGameObject(name);
    object->setPosition(position);
    if (MeshRenderer* renderer = object->addComponent<MeshRenderer>())
        renderer->setMesh(Assets().createMesh(MeshDesc::sphere(radius, 16, 24)));

    Physics::RigidBody* body = object->addComponent<Physics::RigidBody>();
    body->setSphere(radius);
    if (mass > 0.0f)
    {
        body->setMass(mass);
        body->setInertiaTensor(Physics::Inertia::solidSphere(mass, radius));
    }
    else
        body->setBodyType(Physics::BodyType::Static);
    return object;
}

void addGroundAndSky(Scene& scene, const glm::vec3& cameraPosition, const glm::vec3& lookAt)
{
    GameObject* ground = scene.createGameObject("Ground");
    ground->setPosition(glm::vec3(0.0f, -0.5f, 0.0f));
    if (MeshRenderer* renderer = ground->addComponent<MeshRenderer>())
        renderer->setMesh(Assets().createMesh(MeshDesc::box(glm::vec3(120.0f, 1.0f, 120.0f))));
    Physics::RigidBody* groundBody = ground->addComponent<Physics::RigidBody>();
    groundBody->setBox(glm::vec3(60.0f, 0.5f, 60.0f));
    groundBody->setBodyType(Physics::BodyType::Static);
    groundBody->setFriction(0.9f);

    GameObject* cameraObject = scene.createGameObject("Camera");
    cameraObject->setPosition(cameraPosition);
    cameraObject->lookAt(lookAt);
    Camera* camera = cameraObject->addComponent<Camera>();
    camera->setPerspective(60.0f, 16.0f / 9.0f, 0.1f, 500.0f);
    if (FreeFly* fly = cameraObject->addComponent<FreeFly>())
        fly->setSprintMultiplier(4.0f);
    scene.setActiveCamera(camera);

    GameObject* sunObject = scene.createGameObject("Sun");
    DirectionalLight* sun = sunObject->addComponent<DirectionalLight>();
    sun->setColor(glm::vec3(1.0f, 0.96f, 0.9f));
    sun->setIntensity(1.2f);
    sunObject->setPosition(glm::vec3(-20.0f, 30.0f, -20.0f));
    sunObject->lookAt(glm::vec3(0.0f));
    scene.setSunLight(sun);
}

// ------------------------------------------------------------ joint gallery

void buildJointGallery(Scene& scene)
{
    addGroundAndSky(scene, glm::vec3(0.0f, 8.0f, 22.0f), glm::vec3(0.0f, 3.0f, 0.0f));

    // A door on a hinge, with stops: swings, and cannot pass its frame.
    GameObject* frame = makeBox(scene, "DoorFrame", glm::vec3(-12.0f, 3.0f, 0.0f),
                                glm::vec3(0.15f, 1.5f, 0.15f), 0.0f);
    GameObject* door = makeBox(scene, "Door", glm::vec3(-11.0f, 3.0f, 0.0f),
                               glm::vec3(0.9f, 1.4f, 0.08f), 12.0f);
    HingeJoint* hinge = door->addComponent<HingeJoint>();
    hinge->setConnectedBody(frame);
    hinge->setAuthoredAxis(glm::vec3(0.0f, 1.0f, 0.0f));
    hinge->setLimits(0.0f, glm::radians(110.0f));

    // A lift on a slider, held at a floor by a servo.
    GameObject* shaft = makeBox(scene, "LiftShaft", glm::vec3(-7.0f, 0.2f, 0.0f),
                                glm::vec3(0.2f, 0.2f, 0.2f), 0.0f);
    GameObject* platform = makeBox(scene, "LiftPlatform", glm::vec3(-7.0f, 0.6f, 0.0f),
                                   glm::vec3(1.2f, 0.1f, 1.2f), 60.0f);
    SliderJoint* lift = platform->addComponent<SliderJoint>();
    lift->setConnectedBody(shaft);
    lift->setAuthoredAxis(glm::vec3(0.0f, 1.0f, 0.0f));
    lift->setLimits(0.0f, 6.0f);
    lift->setServo(4.0f, 40000.0f, 1.2f);
    // Something to carry, so the servo has real work to do.
    makeBox(scene, "LiftCargo", glm::vec3(-7.0f, 1.2f, 0.0f), glm::vec3(0.4f, 0.4f, 0.4f), 40.0f);

    // A pendulum on a ball joint: free in every direction.
    GameObject* pivot = makeBox(scene, "PendulumPivot", glm::vec3(-2.5f, 6.0f, 0.0f),
                                glm::vec3(0.15f), 0.0f);
    GameObject* bob =
        makeSphere(scene, "PendulumBob", glm::vec3(-1.0f, 4.5f, 0.0f), 0.45f, 15.0f);
    PointJoint* ball = bob->addComponent<PointJoint>();
    ball->setConnectedBody(pivot);

    // A crane cable: a distance joint holding a load that swings.
    GameObject* boom =
        makeBox(scene, "CraneBoom", glm::vec3(2.5f, 6.5f, 0.0f), glm::vec3(0.2f), 0.0f);
    GameObject* load = makeBox(scene, "CraneLoad", glm::vec3(2.5f, 3.0f, 0.0f),
                               glm::vec3(0.5f, 0.5f, 0.5f), 80.0f);
    DistanceJoint* cable = load->addComponent<DistanceJoint>();
    cable->setConnectedBody(boom);
    cable->setAuthoredDistance(3.0f, 3.5f);

    // A piston: slides and spins on the same axis.
    GameObject* rail = makeBox(scene, "PistonRail", glm::vec3(7.0f, 3.0f, 0.0f),
                               glm::vec3(0.2f), 0.0f);
    GameObject* rod = makeBox(scene, "PistonRod", glm::vec3(7.0f, 3.0f, 0.0f),
                              glm::vec3(0.25f, 0.25f, 1.0f), 20.0f);
    PistonJoint* piston = rod->addComponent<PistonJoint>();
    piston->setConnectedBody(rail);
    piston->setAuthoredAxis(glm::vec3(0.0f, 0.0f, 1.0f));
    piston->setLinearLimits(-1.5f, 1.5f);
    piston->setLinearMotor(1.5f, 4000.0f);

    // A universal joint: two axes, like a hip.
    GameObject* yoke =
        makeBox(scene, "UniversalYoke", glm::vec3(11.0f, 4.0f, 0.0f), glm::vec3(0.2f), 0.0f);
    GameObject* shaftBody = makeBox(scene, "UniversalShaft", glm::vec3(11.0f, 3.0f, 0.0f),
                                    glm::vec3(0.18f, 1.0f, 0.18f), 10.0f);
    UniversalJoint* universal = shaftBody->addComponent<UniversalJoint>();
    universal->setConnectedBody(yoke);
    universal->setAuthoredAxis(glm::vec3(1.0f, 0.0f, 0.0f));

    // A stack, to watch contacts settle and sleep.
    for (u32 i = 0; i < 6; ++i)
        makeBox(scene, "StackBox", glm::vec3(15.0f, 0.35f + static_cast<f32>(i) * 0.72f, 0.0f),
                glm::vec3(0.35f), 8.0f);
}

// ---------------------------------------------------------------- robot arm

void buildRobotArm(Scene& scene)
{
    addGroundAndSky(scene, glm::vec3(6.0f, 4.0f, 8.0f), glm::vec3(0.0f, 2.0f, 0.0f));

    GameObject* base = makeBox(scene, "ArmBase", glm::vec3(0.0f, 0.3f, 0.0f),
                               glm::vec3(0.6f, 0.3f, 0.6f), 0.0f);

    // Axis 1: the whole arm turns about vertical.
    GameObject* column = makeBox(scene, "ArmColumn", glm::vec3(0.0f, 1.0f, 0.0f),
                                 glm::vec3(0.25f, 0.5f, 0.25f), 30.0f);
    HingeJoint* axis1 = column->addComponent<HingeJoint>();
    axis1->setConnectedBody(base);
    axis1->setAuthoredAxis(glm::vec3(0.0f, 1.0f, 0.0f));
    axis1->setServo(0.0f, 6000.0f, 1.2f);

    // Axis 2: the shoulder.
    GameObject* upperArm = makeBox(scene, "ArmUpper", glm::vec3(0.0f, 2.0f, 0.0f),
                                   glm::vec3(0.18f, 0.7f, 0.18f), 18.0f);
    HingeJoint* axis2 = upperArm->addComponent<HingeJoint>();
    axis2->setConnectedBody(column);
    axis2->setAuthoredAxis(glm::vec3(0.0f, 0.0f, 1.0f));
    axis2->setLimits(glm::radians(-60.0f), glm::radians(90.0f));
    axis2->setServo(0.0f, 6000.0f, 1.5f);

    // Axis 3: the elbow.
    GameObject* foreArm = makeBox(scene, "ArmFore", glm::vec3(0.0f, 3.2f, 0.0f),
                                  glm::vec3(0.15f, 0.6f, 0.15f), 10.0f);
    HingeJoint* axis3 = foreArm->addComponent<HingeJoint>();
    axis3->setConnectedBody(upperArm);
    axis3->setAuthoredAxis(glm::vec3(0.0f, 0.0f, 1.0f));
    axis3->setLimits(0.0f, glm::radians(150.0f));
    axis3->setServo(0.0f, 4000.0f, 1.5f);

    // Two fingers on sliders: the gripper.
    for (u32 side = 0; side < 2; ++side)
    {
        const f32 sign = side == 0 ? -1.0f : 1.0f;
        GameObject* finger =
            makeBox(scene, side == 0 ? "GripperLeft" : "GripperRight",
                    glm::vec3(sign * 0.12f, 3.95f, 0.0f), glm::vec3(0.05f, 0.18f, 0.05f), 1.0f);
        SliderJoint* slide = finger->addComponent<SliderJoint>();
        slide->setConnectedBody(foreArm);
        slide->setAuthoredAxis(glm::vec3(sign, 0.0f, 0.0f));
        slide->setLimits(0.0f, 0.25f);
        slide->setServo(0.2f, 400.0f, 0.4f);
    }

    // Something to reach for.
    makeBox(scene, "WorkPiece", glm::vec3(1.4f, 0.2f, 0.0f), glm::vec3(0.12f, 0.12f, 0.12f), 1.5f);
}

// ------------------------------------------------------------------ vehicle

void buildVehicle(Scene& scene)
{
    addGroundAndSky(scene, glm::vec3(8.0f, 4.0f, 10.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    GameObject* chassis = makeBox(scene, "Chassis", glm::vec3(0.0f, 1.2f, 0.0f),
                                  glm::vec3(0.9f, 0.35f, 1.9f), 1200.0f);

    const glm::vec3 corners[4] = {glm::vec3(-0.95f, 0.75f, 1.4f), glm::vec3(0.95f, 0.75f, 1.4f),
                                  glm::vec3(-0.95f, 0.75f, -1.4f), glm::vec3(0.95f, 0.75f, -1.4f)};
    const bool isFront[4] = {true, true, false, false};
    const char* names[4] = {"WheelFL", "WheelFR", "WheelRL", "WheelRR"};

    for (u32 i = 0; i < 4; ++i)
    {
        GameObject* wheel = makeSphere(scene, names[i], corners[i], 0.35f, 20.0f);
        WheelJoint* joint = wheel->addComponent<WheelJoint>();
        joint->setConnectedBody(chassis);
        joint->setAuthoredSuspensionAxis(glm::vec3(0.0f, -1.0f, 0.0f));
        joint->setAuthoredSpinAxis(glm::vec3(1.0f, 0.0f, 0.0f));
        // 300 kg a corner settling 20 cm is about 15 kN/m; damping a tenth
        // of that. Stable at any stiffness now that the spring is solved in
        // the constraint rather than pushed in as a force.
        joint->setSuspension(0.45f, 15000.0f, 1500.0f);
        if (isFront[i])
        {
            joint->setSteeringLimits(glm::radians(-35.0f), glm::radians(35.0f));
            joint->setSteeringServo(0.0f, 900.0f, 4.0f);
        }
        else
            joint->setSteeringLimits(0.0f, 0.0f);
        joint->setSpinMotor(0.0f, 0.0f);
    }

    // A ramp and a few obstacles, so the suspension has something to do.
    GameObject* ramp = makeBox(scene, "Ramp", glm::vec3(0.0f, 0.35f, -14.0f),
                               glm::vec3(4.0f, 0.2f, 3.0f), 0.0f);
    ramp->setRotation(glm::angleAxis(glm::radians(-12.0f), glm::vec3(1.0f, 0.0f, 0.0f)));
    for (u32 i = 0; i < 5; ++i)
        makeBox(scene, "Kerb",
                glm::vec3(-6.0f + static_cast<f32>(i) * 3.0f, 0.15f, -6.0f),
                glm::vec3(0.6f, 0.15f, 0.6f), 0.0f);
}

// -------------------------------------------------------------- agent crowd

void buildAgentCrowd(Scene& scene)
{
    addGroundAndSky(scene, glm::vec3(0.0f, 26.0f, 26.0f), glm::vec3(0.0f, 0.0f, 0.0f));

    // Pillars to walk around, each one an Obstacle the avoidance reads from
    // the scene - no group assembled by hand.
    const glm::vec3 pillars[5] = {glm::vec3(0.0f, 1.5f, 0.0f), glm::vec3(6.0f, 1.5f, -4.0f),
                                  glm::vec3(-6.0f, 1.5f, -4.0f), glm::vec3(4.0f, 1.5f, 6.0f),
                                  glm::vec3(-4.0f, 1.5f, 6.0f)};
    for (u32 i = 0; i < 5; ++i)
    {
        GameObject* pillar = scene.createGameObject("Pillar");
        pillar->setPosition(pillars[i]);
        if (MeshRenderer* renderer = pillar->addComponent<MeshRenderer>())
            renderer->setMesh(Assets().createMesh(MeshDesc::sphere(1.5f, 12, 18)));
        Radion::Obstacle* obstacle = pillar->addComponent<Radion::Obstacle>();
        obstacle->setSphere(1.5f);
    }

    // The goal every agent walks toward.
    GameObject* goal = scene.createGameObject("Goal");
    goal->setPosition(glm::vec3(0.0f, 0.5f, -16.0f));
    if (MeshRenderer* renderer = goal->addComponent<MeshRenderer>())
        renderer->setMesh(Assets().createMesh(MeshDesc::box(glm::vec3(1.0f, 1.0f, 1.0f))));

    Agent::Settings settings;
    settings.type = 1;
    settings.senseRange = 6.0f;
    settings.maxVelocityChange = 3.0f;
    settings.maxSpeed = 4.0f;
    settings.desiredSpeed = 3.0f;
    settings.radius = 0.4f;

    for (u32 i = 0; i < 12; ++i)
    {
        const f32 lane = static_cast<f32>(i % 6) * 2.2f - 5.5f;
        const f32 row = static_cast<f32>(i / 6) * 2.0f;
        GameObject* object = scene.createGameObject("Agent");
        object->setPosition(glm::vec3(lane, 0.4f, 14.0f + row));
        if (MeshRenderer* renderer = object->addComponent<MeshRenderer>())
            renderer->setMesh(Assets().createMesh(MeshDesc::capsule(0.35f, 1.2f, 8, 12)));

        Agent* agent = object->addComponent<Agent>();
        agent->applySettings(settings);
        agent->setGroupId(1);
        agent->setPosition(object->position());
        agent->setGoal(glm::vec3(0.0f, 0.4f, -16.0f));
        agent->setGoalRadius(1.5f);
        // Separation keeps them out of each other; obstacle avoidance reads
        // the scene's own obstacle list.
        agent->addBehavior<AI::SeparationBehavior>(1.6f, 0.2f, 1.0f);
        agent->addBehavior<AI::SeekBehavior>(glm::vec3(0.0f, 0.4f, -16.0f));
        agent->addBehavior<AI::ObstacleAvoidanceBehavior>(2.5f);
    }
}

} // namespace

const char* sceneName(LabScene scene)
{
    switch (scene)
    {
    case LabScene::JointGallery: return "Joint gallery";
    case LabScene::RobotArm: return "Robot arm";
    case LabScene::Vehicle: return "Vehicle";
    case LabScene::AgentCrowd: return "Agent crowd";
    case LabScene::Count: break;
    }
    return "";
}

void buildScene(Scene& scene, LabScene which)
{
    switch (which)
    {
    case LabScene::JointGallery: buildJointGallery(scene); break;
    case LabScene::RobotArm: buildRobotArm(scene); break;
    case LabScene::Vehicle: buildVehicle(scene); break;
    case LabScene::AgentCrowd: buildAgentCrowd(scene); break;
    case LabScene::Count: break;
    }
}

void updateScene(Scene& scene, LabScene which, f32 elapsed)
{
    switch (which)
    {
    case LabScene::RobotArm:
    {
        // Walks the three axes through a slow cycle and works the gripper, so
        // the servos are always doing something to watch.
        GameObject* column = scene.findGameObject("ArmColumn");
        GameObject* upper = scene.findGameObject("ArmUpper");
        GameObject* fore = scene.findGameObject("ArmFore");
        if (column)
            if (HingeJoint* axis = column->getComponent<HingeJoint>())
                axis->setServo(std::sin(elapsed * 0.5f) * 1.2f, 6000.0f, 1.2f);
        if (upper)
            if (HingeJoint* axis = upper->getComponent<HingeJoint>())
                axis->setServo(glm::radians(30.0f) + std::sin(elapsed * 0.7f) * 0.5f, 6000.0f,
                               1.5f);
        if (fore)
            if (HingeJoint* axis = fore->getComponent<HingeJoint>())
                axis->setServo(glm::radians(60.0f) + std::sin(elapsed * 0.9f) * 0.6f, 4000.0f,
                               1.5f);

        const f32 opening = 0.05f + (std::sin(elapsed * 1.3f) * 0.5f + 0.5f) * 0.18f;
        for (const char* name : {"GripperLeft", "GripperRight"})
            if (GameObject* finger = scene.findGameObject(name))
                if (SliderJoint* slide = finger->getComponent<SliderJoint>())
                    slide->setServo(opening, 400.0f, 0.4f);
        break;
    }
    case LabScene::Vehicle:
    {
        // Gentle throttle with the wheel weaving, which is what makes the
        // suspension and the steering servo visible.
        const f32 steer = std::sin(elapsed * 0.4f) * glm::radians(22.0f);
        const char* front[2] = {"WheelFL", "WheelFR"};
        for (const char* name : front)
            if (GameObject* wheel = scene.findGameObject(name))
                if (WheelJoint* joint = wheel->getComponent<WheelJoint>())
                    joint->setSteeringServo(steer, 900.0f, 4.0f);
        const char* driven[2] = {"WheelRL", "WheelRR"};
        for (const char* name : driven)
            if (GameObject* wheel = scene.findGameObject(name))
                if (WheelJoint* joint = wheel->getComponent<WheelJoint>())
                    joint->setSpinMotor(18.0f, 400.0f);
        break;
    }
    case LabScene::JointGallery:
    {
        // The lift rides between two floors, so the slider servo is doing
        // work rather than just holding.
        if (GameObject* platform = scene.findGameObject("LiftPlatform"))
            if (SliderJoint* lift = platform->getComponent<SliderJoint>())
                lift->setServo(3.0f + std::sin(elapsed * 0.35f) * 2.5f, 40000.0f, 1.2f);
        break;
    }
    case LabScene::AgentCrowd:
    case LabScene::Count:
        break;
    }
}

} // namespace Radion::Lab
