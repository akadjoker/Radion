#include "PCH.h"

#include "Scene.h"
#include "collision/CollisionShape.h"
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
        std::fprintf(stderr, "PhysicsCcdTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool near(f32 a, f32 b, f32 epsilon = 1e-3f)
{
    return std::abs(a - b) <= epsilon;
}

RigidBody makeBullet(const glm::vec3& position, const glm::vec3& velocity, const CollisionShape& shape)
{
    RigidBody body;
    body.setMass(1.0f);
    body.setInertiaTensor(shape.inertia(1.0f));
    body.setPosition(position);
    body.setVelocity(velocity);
    body.setDamping(1.0f, 1.0f);
    body.setCanSleep(false);
    body.setBullet(true);
    return body;
}

// A thin static wall a fast body would cross entirely within one step if
// swept only by its own AABB - the classic tunnelling setup.
void testBulletStopsAtThinWall()
{
    BoxShape wallShape(glm::vec3(0.05f, 1.0f, 1.0f));
    BoxShape bulletShape(glm::vec3(0.1f));

    RigidBody wall;
    wall.setBodyType(BodyType::Static);
    wall.setPosition(glm::vec3(5.0f, 0.0f, 0.0f));
    wall.setShape(&wallShape);

    RigidBody bullet = makeBullet(glm::vec3(0.0f), glm::vec3(200.0f, 0.0f, 0.0f), bulletShape);
    bullet.setShape(&bulletShape);

    Radion::Scene world;
    world.setGravity(glm::vec3(0.0f));
    world.addBody(wall);
    world.addBody(bullet);

    // 200 * 0.1 = 20 units of travel against a wall 0.1 units thick at x in [4.95, 5.05].
    world.stepPhysics(0.1f);

    CHECK(bullet.position().x < 5.0f);
    CHECK(near(bullet.position().x, 4.945f, 1e-3f));
    CHECK(near(bullet.position().y, 0.0f, 1e-3f));
    CHECK(near(bullet.position().z, 0.0f, 1e-3f));
}

// Same scene, no setBullet(true): proves the wall really is thin enough to
// tunnel through, so the previous test is measuring the sweep and not
// something else (contact margin, a slow enough wall, and so on).
void testNonBulletTunnelsThroughSameWall()
{
    BoxShape wallShape(glm::vec3(0.05f, 1.0f, 1.0f));
    BoxShape bulletShape(glm::vec3(0.1f));

    RigidBody wall;
    wall.setBodyType(BodyType::Static);
    wall.setPosition(glm::vec3(5.0f, 0.0f, 0.0f));
    wall.setShape(&wallShape);

    RigidBody bullet;
    bullet.setMass(1.0f);
    bullet.setInertiaTensor(bulletShape.inertia(1.0f));
    bullet.setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
    bullet.setVelocity(glm::vec3(200.0f, 0.0f, 0.0f));
    bullet.setDamping(1.0f, 1.0f);
    bullet.setCanSleep(false);
    bullet.setShape(&bulletShape);

    Radion::Scene world;
    world.setGravity(glm::vec3(0.0f));
    world.addBody(wall);
    world.addBody(bullet);

    world.stepPhysics(0.1f);

    CHECK(bullet.position().x > 5.05f);
    CHECK(near(bullet.position().x, 20.0f, 1e-3f));
}

// A bullet-marked body settling under ordinary gravity has to rest exactly
// like any other body: the sweep must stay out of the way once motion per
// step drops below the slop threshold.
void testBulletRestsNormallyAtLowSpeed()
{
    BoxShape groundShape(glm::vec3(10.0f, 0.5f, 10.0f));
    SphereShape ballShape(0.5f);

    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(glm::vec3(0.0f, -0.5f, 0.0f));
    ground.setShape(&groundShape);
    ground.setFriction(0.5f);

    RigidBody ball;
    ball.setMass(1.0f);
    ball.setInertiaTensor(ballShape.inertia(1.0f));
    ball.setPosition(glm::vec3(0.0f, 2.0f, 0.0f));
    ball.setDamping(0.999f, 0.999f);
    ball.setBullet(true);
    ball.setShape(&ballShape);

    Radion::Scene world;
    world.setGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    world.addBody(ground);
    world.addBody(ball);

    for (u32 i = 0; i < 300; ++i)
        world.stepPhysics(1.0f / 120.0f);

    CHECK(std::abs(ball.position().y - 0.5f) < 0.05f);
    CHECK(glm::length(ball.velocity()) < 0.5f);
}

// Two fast dynamic bodies closing on each other: the reference sweep skips
// any hit against a Dynamic body and leaves it to the ordinary contact
// solver, so a bullet-marked body must still cross a dynamic obstacle within
// the step that first brings them together.
void testBulletSweepSkipsDynamicObstacles()
{
    BoxShape obstacleShape(glm::vec3(0.05f, 1.0f, 1.0f));
    BoxShape bulletShape(glm::vec3(0.1f));

    RigidBody obstacle;
    obstacle.setMass(1.0f);
    obstacle.setInertiaTensor(obstacleShape.inertia(1.0f));
    obstacle.setPosition(glm::vec3(5.0f, 0.0f, 0.0f));
    obstacle.setDamping(1.0f, 1.0f);
    obstacle.setCanSleep(false);
    obstacle.setShape(&obstacleShape);

    RigidBody bullet = makeBullet(glm::vec3(0.0f), glm::vec3(200.0f, 0.0f, 0.0f), bulletShape);
    bullet.setShape(&bulletShape);

    Radion::Scene world;
    world.setGravity(glm::vec3(0.0f));
    world.addBody(obstacle);
    world.addBody(bullet);

    world.stepPhysics(0.1f);

    CHECK(bullet.position().x > 5.05f);
}

} // namespace

int main()
{
    testBulletStopsAtThinWall();
    testNonBulletTunnelsThroughSameWall();
    testBulletRestsNormallyAtLowSpeed();
    testBulletSweepSkipsDynamicObstacles();
    if (gFailures)
        std::fprintf(stderr, "%d physics ccd test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
