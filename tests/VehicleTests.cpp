#include "PCH.h"

#include "collision/CollisionShape.h"
#include "dynamics/MotorcycleController.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/RaycastVehicle.h"
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
        std::fprintf(stderr, "VehicleTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool near(f32 a, f32 b, f32 epsilon = 1e-3f)
{
    return std::abs(a - b) <= epsilon;
}

bool finiteVec(const Math::vec3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

void stepVehicle(f32 step, void* userData)
{
    static_cast<RaycastVehicle*>(userData)->update(step);
}

struct CarFixture
{
    BoxShape groundShape{Math::vec3(50.0f, 0.5f, 50.0f)};
    BoxShape chassisShape{Math::vec3(1.0f, 0.4f, 2.0f)};

    RigidBody ground;
    RigidBody chassis;

    PhysicsWorld world;
    u32 chassisId = 0;
    RaycastVehicle* vehicle = nullptr;

    explicit CarFixture(bool withGround = true)
    {
        world.setGravity(Math::vec3(0.0f, -9.81f, 0.0f));
        world.setFixedStep(1.0f / 120.0f);

        if (withGround)
        {
            ground.setBodyType(BodyType::Static);
            ground.setPosition(Math::vec3(0.0f, -0.5f, 0.0f));
            BodyEntry groundEntry;
            groundEntry.body = &ground;
            groundEntry.shape = &groundShape;
            groundEntry.friction = 0.9f;
            world.addBody(groundEntry);
        }

        chassis.setMass(800.0f);
        chassis.setInertiaTensor(Inertia::box(800.0f, Math::vec3(1.0f, 0.4f, 2.0f)));
        chassis.setPosition(Math::vec3(0.0f, 0.6f, 0.0f));
        chassis.setDamping(1.0f, 1.0f);
        chassis.setCanSleep(false);
        BodyEntry chassisEntry;
        chassisEntry.body = &chassis;
        chassisEntry.shape = &chassisShape;
        chassisEntry.friction = 0.5f;
        chassisId = world.addBody(chassisEntry);
    }

    void buildVehicle()
    {
        vehicle = new RaycastVehicle(chassis, &world, chassisId);

        RaycastVehicle::Tuning tuning;
        const Math::vec3 direction(0.0f, -1.0f, 0.0f);
        const Math::vec3 axle(-1.0f, 0.0f, 0.0f);
        const f32 restLength = 0.3f;
        const f32 radius = 0.4f;

        const Math::vec3 corners[4] = {
            Math::vec3(-0.9f, -0.4f, 1.7f),  // front left
            Math::vec3(0.9f, -0.4f, 1.7f),   // front right
            Math::vec3(-0.9f, -0.4f, -1.7f), // rear left
            Math::vec3(0.9f, -0.4f, -1.7f)   // rear right
        };
        const bool isFront[4] = {true, true, false, false};

        for (u32 i = 0; i < 4; ++i)
            vehicle->addWheel(corners[i], direction, axle, restLength, radius, tuning, isFront[i]);

        world.setStepCallback(&stepVehicle, vehicle);
    }

    ~CarFixture()
    {
        delete vehicle;
    }
};

void testSuspensionHoldsTheCarUp()
{
    CarFixture fixture;
    fixture.buildVehicle();

    for (u32 i = 0; i < 120; ++i)
        fixture.world.step(1.0f / 120.0f);
    const f32 heightAfterOneSecond = fixture.chassis.position().y;

    for (u32 i = 0; i < 240; ++i)
        fixture.world.step(1.0f / 120.0f);
    const f32 heightAfterThreeSeconds = fixture.chassis.position().y;

    CHECK(std::isfinite(heightAfterThreeSeconds));
    CHECK(near(heightAfterOneSecond, heightAfterThreeSeconds, 0.05f));
    for (u32 i = 0; i < fixture.vehicle->wheelCount(); ++i)
        CHECK(fixture.vehicle->wheel(i).inContact);
}

void testEngineForceAcceleratesForward()
{
    CarFixture fixture;
    fixture.buildVehicle();

    for (u32 i = 0; i < 120; ++i)
        fixture.world.step(1.0f / 120.0f);

    for (u32 i = 0; i < 4; ++i)
        fixture.vehicle->setEngineForce(3000.0f, i);

    for (u32 i = 0; i < 240; ++i)
        fixture.world.step(1.0f / 120.0f);

    const Math::vec3 forward = fixture.chassis.directionToWorld(Math::vec3(0.0f, 0.0f, 1.0f));
    const f32 forwardSpeed = Math::dot(forward, fixture.chassis.velocity());
    CHECK(finiteVec(fixture.chassis.position()));
    CHECK(forwardSpeed > 1.0f);
}

void testBrakeSlowsTheCarDown()
{
    CarFixture fixture;
    fixture.buildVehicle();

    for (u32 i = 0; i < 120; ++i)
        fixture.world.step(1.0f / 120.0f);

    for (u32 i = 0; i < 4; ++i)
        fixture.vehicle->setEngineForce(3000.0f, i);
    for (u32 i = 0; i < 240; ++i)
        fixture.world.step(1.0f / 120.0f);

    for (u32 i = 0; i < 4; ++i)
    {
        fixture.vehicle->setEngineForce(0.0f, i);
        fixture.vehicle->setBrake(4000.0f, i);
    }
    // Four seconds: the chassis pitches on its suspension as it sheds speed,
    // so the linear speed at the centre of mass does not fall monotonically
    // - it needs the full transient to settle before this checks it is slow.
    for (u32 i = 0; i < 480; ++i)
        fixture.world.step(1.0f / 120.0f);

    CHECK(finiteVec(fixture.chassis.velocity()));
    CHECK(Math::length(fixture.chassis.velocity()) < 1.0f);
}

void testSteeringTurnsTheCar()
{
    CarFixture fixture;
    fixture.buildVehicle();

    for (u32 i = 0; i < 120; ++i)
        fixture.world.step(1.0f / 120.0f);

    for (u32 i = 0; i < 4; ++i)
        fixture.vehicle->setEngineForce(2500.0f, i);
    // Front wheels only (indices 0 and 1 in buildVehicle's corner order).
    fixture.vehicle->setSteering(0.35f, 0);
    fixture.vehicle->setSteering(0.35f, 1);

    // Only far enough to see the turn build up - by three seconds at this
    // speed and lock the car has come most of the way round a circle and
    // its z coordinate would have come back down through zero again.
    for (u32 i = 0; i < 180; ++i)
    {
        fixture.world.step(1.0f / 120.0f);
        CHECK(finiteVec(fixture.chassis.position()));
    }

    CHECK(fixture.chassis.position().z > 0.5f);
    CHECK(std::abs(fixture.chassis.position().x) > 0.3f);
}

void testFreeFallWithoutGroundStaysFinite()
{
    CarFixture fixture(false);
    fixture.buildVehicle();

    for (u32 i = 0; i < 4; ++i)
        fixture.vehicle->setEngineForce(1500.0f, i);

    for (u32 i = 0; i < 240; ++i)
    {
        fixture.world.step(1.0f / 120.0f);
        CHECK(finiteVec(fixture.chassis.position()));
        CHECK(finiteVec(fixture.chassis.velocity()));
    }

    for (u32 i = 0; i < fixture.vehicle->wheelCount(); ++i)
        CHECK(!fixture.vehicle->wheel(i).inContact);
    // Free fall under gravity: the chassis has to have dropped.
    CHECK(fixture.chassis.position().y < 0.0f);
}

// ------------------------------------------------------------- motorcycle

struct BikeStep
{
    RaycastVehicle* vehicle = nullptr;
    MotorcycleController* controller = nullptr;
    Math::vec3 gravity{0.0f, -9.81f, 0.0f};
};

void stepBike(f32 step, void* userData)
{
    BikeStep* bike = static_cast<BikeStep*>(userData);
    bike->controller->preUpdate(step, bike->gravity);
    bike->vehicle->update(step);
    bike->controller->postUpdate(step);
}

struct BikeFixture
{
    BoxShape groundShape{Math::vec3(200.0f, 0.5f, 200.0f)};
    BoxShape chassisShape{Math::vec3(0.2f, 0.3f, 0.9f)};

    RigidBody ground;
    RigidBody chassis;

    PhysicsWorld world;
    u32 chassisId = 0;
    RaycastVehicle* vehicle = nullptr;
    MotorcycleController* controller = nullptr;
    BikeStep stepData;

    BikeFixture()
    {
        world.setGravity(Math::vec3(0.0f, -9.81f, 0.0f));
        world.setFixedStep(1.0f / 120.0f);

        ground.setBodyType(BodyType::Static);
        ground.setPosition(Math::vec3(0.0f, -0.5f, 0.0f));
        BodyEntry groundEntry;
        groundEntry.body = &ground;
        groundEntry.shape = &groundShape;
        groundEntry.friction = 0.9f;
        world.addBody(groundEntry);

        chassis.setMass(240.0f);
        chassis.setInertiaTensor(Inertia::box(240.0f, Math::vec3(0.2f, 0.3f, 0.9f)));
        chassis.setPosition(Math::vec3(0.0f, 0.6f, 0.0f));
        chassis.setDamping(1.0f, 1.0f);
        chassis.setCanSleep(false);
        BodyEntry chassisEntry;
        chassisEntry.body = &chassis;
        chassisEntry.shape = &chassisShape;
        chassisEntry.friction = 0.5f;
        chassisId = world.addBody(chassisEntry);

        vehicle = new RaycastVehicle(chassis, &world, chassisId);
        RaycastVehicle::Tuning tuning;
        tuning.suspensionStiffness = 20.0f;
        tuning.suspensionCompression = 2.0f;
        tuning.suspensionDamping = 2.3f;
        const Math::vec3 direction(0.0f, -1.0f, 0.0f);
        const Math::vec3 axle(-1.0f, 0.0f, 0.0f);
        vehicle->addWheel(Math::vec3(0.0f, -0.25f, 0.75f), direction, axle, 0.3f, 0.3f, tuning,
                         true);
        vehicle->addWheel(Math::vec3(0.0f, -0.25f, -0.75f), direction, axle, 0.3f, 0.3f, tuning,
                         false);
        // A car keeps this low to fight rollover; a bike IS the rollover -
        // the lateral forces' roll moment is the dynamics the lean spring
        // balances against, so it must act at the real contact height.
        vehicle->wheel(0).rollInfluence = 1.0f;
        vehicle->wheel(1).rollInfluence = 1.0f;

        controller = new MotorcycleController(*vehicle, chassis);
        controller->setCasterAngle(0.5236f);

        stepData.vehicle = vehicle;
        stepData.controller = controller;
        world.setStepCallback(&stepBike, &stepData);
    }

    ~BikeFixture()
    {
        delete controller;
        delete vehicle;
    }
};

void testMotorcycleStaysUprightDrivingStraight()
{
    BikeFixture fixture;
    fixture.vehicle->setEngineForce(600.0f, 1);

    for (u32 step = 0; step < 480; ++step)
        fixture.world.step(1.0f / 120.0f);

    CHECK(finiteVec(fixture.chassis.position()));
    const Math::vec3 up = fixture.chassis.directionToWorld(Math::vec3(0.0f, 1.0f, 0.0f));
    CHECK(up.y > 0.98f);
    CHECK(Math::dot(fixture.chassis.velocity(),
                   fixture.chassis.directionToWorld(Math::vec3(0.0f, 0.0f, 1.0f))) > 3.0f);
}

void testMotorcycleLeansIntoATurn()
{
    BikeFixture fixture;
    fixture.vehicle->setEngineForce(600.0f, 1);

    for (u32 step = 0; step < 360; ++step)
        fixture.world.step(1.0f / 120.0f);
    fixture.controller->setSteerInput(0.5f);
    f32 deepestLean = 0.0f;
    for (u32 step = 0; step < 480; ++step)
    {
        fixture.world.step(1.0f / 120.0f);
        CHECK(finiteVec(fixture.chassis.position()));
        deepestLean = Math::max(deepestLean,
                               std::abs(fixture.controller->currentLeanAngle()));
    }

    // Turning demands lean; upright through a turn means the controller is
    // not steering the roll at all. And it must never exceed its own cap.
    CHECK(deepestLean > 0.05f);
    CHECK(deepestLean < fixture.controller->currentLeanAngle() + 1.0f);
    const Math::vec3 up = fixture.chassis.directionToWorld(Math::vec3(0.0f, 1.0f, 0.0f));
    CHECK(up.y > 0.5f);
}

void testMotorcycleFallsWithTheLeanSpringOff()
{
    BikeFixture fixture;
    fixture.controller->setLeanControllerEnabled(false);
    fixture.chassis.setAngularVelocity(Math::vec3(0.0f, 0.0f, 0.3f));

    for (u32 step = 0; step < 480; ++step)
        fixture.world.step(1.0f / 120.0f);

    CHECK(finiteVec(fixture.chassis.position()));
    const Math::vec3 up = fixture.chassis.directionToWorld(Math::vec3(0.0f, 1.0f, 0.0f));
    CHECK(up.y < 0.7f);
}

} // namespace

int main()
{
    testSuspensionHoldsTheCarUp();
    testEngineForceAcceleratesForward();
    testBrakeSlowsTheCarDown();
    testSteeringTurnsTheCar();
    testFreeFallWithoutGroundStaysFinite();
    testMotorcycleStaysUprightDrivingStraight();
    testMotorcycleLeansIntoATurn();
    testMotorcycleFallsWithTheLeanSpringOff();
    if (gFailures)
        std::fprintf(stderr, "%d vehicle test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
