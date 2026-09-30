#include "PCH.h"

#include "Collider.h"
#include "CollisionWorld.h"
#include "GameObject.h"
#include "Octree.h"
#include "Scene.h"
#include "SceneSerializer.h"

#include <cstdio>

using namespace Radion;

namespace
{

int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (condition)
        return;
    std::fprintf(stderr, "SceneCollisionTests:%d: failed: %s\n", line, expression);
    ++gFailures;
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

// Flat world-space quad at y = 0 spanning [-5, 5] in X and Z, normal up.
void buildGroundOctree(TriangleOctree& octree)
{
    CollisionMesh mesh;
    mesh.positions = {Math::vec3(-5.0f, 0.0f, -5.0f), Math::vec3(5.0f, 0.0f, -5.0f),
                      Math::vec3(5.0f, 0.0f, 5.0f), Math::vec3(-5.0f, 0.0f, 5.0f)};
    mesh.indices = {0, 3, 2, 0, 2, 1};
    octree.addCollisionMesh(mesh, Math::mat4(1.0f));
    octree.build();
}

GameObject* makeBoxWall(Scene& scene, const char* name, const Math::vec3& center,
                        const Math::vec3& halfExtents, u32 type)
{
    GameObject* object = scene.createGameObject(name);
    object->setPosition(center);
    Collider* collider = object->addComponent<Collider>();
    collider->setBox(halfExtents);
    collider->setType(type);
    return object;
}

GameObject* makeSphereWall(Scene& scene, const char* name, const Math::vec3& center, f32 radius,
                           u32 type)
{
    GameObject* object = scene.createGameObject(name);
    object->setPosition(center);
    Collider* collider = object->addComponent<Collider>();
    collider->setSphere(radius);
    collider->setType(type);
    return object;
}

GameObject* makeCapsuleWall(Scene& scene, const char* name, const Math::vec3& center, f32 radius,
                            f32 height, u32 type)
{
    GameObject* object = scene.createGameObject(name);
    object->setPosition(center);
    Collider* collider = object->addComponent<Collider>();
    collider->setCapsule(radius, height);
    collider->setType(type);
    return object;
}

void testAttachAndReadBackComponent()
{
    Scene scene;
    GameObject* object = scene.createGameObject("Body");
    CHECK(object != nullptr);

    Collider* collider = object->addComponent<Collider>();
    CHECK(collider != nullptr);
    collider->setCapsule(0.4f, 1.8f);
    collider->setType(7);

    Collider* fetched = object->getComponent<Collider>();
    CHECK(fetched == collider);
    CHECK(fetched->shape() == ColliderShape::Capsule);
    CHECK(fetched->type() == 7);
    CHECK(near(fetched->radius(), 0.4f));
    CHECK(near(fetched->height(), 1.8f));
}

void testWorldBoundsFollowTransform()
{
    Scene scene;
    GameObject* box = scene.createGameObject("Box");
    Collider* boxCollider = box->addComponent<Collider>();
    boxCollider->setBox(Math::vec3(0.5f, 0.5f, 0.5f));

    GameObject* ball = scene.createGameObject("Ball");
    Collider* ballCollider = ball->addComponent<Collider>();
    ballCollider->setSphere(0.5f);
    scene.update(1.0f / 60.0f);

    CHECK(near(boxCollider->worldBounds().center(), Math::vec3(0.0f)));
    CHECK(near(boxCollider->worldBounds().extents(), Math::vec3(0.5f)));
    CHECK(near(ballCollider->worldBounds().extents(), Math::vec3(0.5f)));

    box->setPosition(Math::vec3(5.0f, 0.0f, 0.0f));
    box->setScale(Math::vec3(2.0f, 2.0f, 2.0f));
    ball->setPosition(Math::vec3(-5.0f, 0.0f, 0.0f));
    ball->setScale(Math::vec3(2.0f, 2.0f, 2.0f));
    scene.update(1.0f / 60.0f);

    const AABB movedBox = boxCollider->worldBounds();
    CHECK(near(movedBox.center(), Math::vec3(5.0f, 0.0f, 0.0f)));
    CHECK(near(movedBox.extents(), Math::vec3(1.0f)));

    const AABB movedBall = ballCollider->worldBounds();
    CHECK(near(movedBall.center(), Math::vec3(-5.0f, 0.0f, 0.0f)));
    CHECK(near(movedBall.extents(), Math::vec3(1.0f)));

    ball->setScale(Math::vec3(3.0f, 1.0f, 1.0f));
    scene.update(1.0f / 60.0f);
    CHECK(near(ballCollider->worldBounds().extents(), Math::vec3(1.5f, 0.5f, 0.5f)));
}

void testOverlappingSpheresProduceDirectionalContacts()
{
    Scene scene;
    GameObject* a = scene.createGameObject("SphereA");
    GameObject* b = scene.createGameObject("SphereB");
    b->setPosition(Math::vec3(1.5f, 0.0f, 0.0f));

    Collider* colliderA = a->addComponent<Collider>();
    colliderA->setSphere(1.0f);
    colliderA->setType(1);
    Collider* colliderB = b->addComponent<Collider>();
    colliderB->setSphere(1.0f);
    colliderB->setType(2);

    scene.update(1.0f / 60.0f);
    scene.collisions().enable(1, 2, CollisionResponse::None);
    scene.collisions().step();

    CHECK(colliderA->contactCount() == 1);
    CHECK(colliderB->contactCount() == 1);
    if (colliderA->contactCount() == 1)
    {
        CHECK(colliderA->contactAt(0).other == colliderB);
        // B sits in +X from A - A's push-out direction points back at -X.
        CHECK(colliderA->contactAt(0).normal.x < 0.0f);
    }
    if (colliderB->contactCount() == 1)
    {
        CHECK(colliderB->contactAt(0).other == colliderA);
        CHECK(colliderB->contactAt(0).normal.x > 0.0f);
    }
}

void testUnregisteredPairProducesNoContacts()
{
    Scene scene;
    GameObject* a = scene.createGameObject("SphereA");
    GameObject* b = scene.createGameObject("SphereB");
    b->setPosition(Math::vec3(0.5f, 0.0f, 0.0f));

    Collider* colliderA = a->addComponent<Collider>();
    colliderA->setSphere(1.0f);
    colliderA->setType(3);
    Collider* colliderB = b->addComponent<Collider>();
    colliderB->setSphere(1.0f);
    colliderB->setType(4);

    scene.update(1.0f / 60.0f);
    scene.collisions().step();

    CHECK(colliderA->contactCount() == 0);
    CHECK(colliderB->contactCount() == 0);
}

void testDisableAfterEnableStopsContacts()
{
    Scene scene;
    GameObject* a = scene.createGameObject("SphereA");
    GameObject* b = scene.createGameObject("SphereB");
    b->setPosition(Math::vec3(0.5f, 0.0f, 0.0f));

    Collider* colliderA = a->addComponent<Collider>();
    colliderA->setSphere(1.0f);
    colliderA->setType(5);
    Collider* colliderB = b->addComponent<Collider>();
    colliderB->setSphere(1.0f);
    colliderB->setType(6);

    scene.update(1.0f / 60.0f);
    scene.collisions().enable(5, 6, CollisionResponse::Stop);
    scene.collisions().step();
    CHECK(colliderA->contactCount() == 1);
    CHECK(colliderB->contactCount() == 1);

    scene.collisions().disable(5, 6);
    scene.collisions().step();
    CHECK(colliderA->contactCount() == 0);
    CHECK(colliderB->contactCount() == 0);
}

void testSphereVsBoxAndCapsuleVsSphere()
{
    Scene scene;

    GameObject* box = scene.createGameObject("Box");
    Collider* boxCollider = box->addComponent<Collider>();
    boxCollider->setBox(Math::vec3(1.0f, 1.0f, 1.0f));
    boxCollider->setType(10);

    GameObject* sphereNearBox = scene.createGameObject("SphereNearBox");
    sphereNearBox->setPosition(Math::vec3(1.5f, 0.0f, 0.0f));
    Collider* sphereBoxCollider = sphereNearBox->addComponent<Collider>();
    sphereBoxCollider->setSphere(1.0f);
    sphereBoxCollider->setType(11);

    GameObject* capsule = scene.createGameObject("Capsule");
    Collider* capsuleCollider = capsule->addComponent<Collider>();
    capsuleCollider->setCapsule(0.4f, 1.8f); // segment half-height = 0.5, caps to y = +-0.9
    capsuleCollider->setType(12);

    GameObject* sphereNearCapsule = scene.createGameObject("SphereNearCapsule");
    sphereNearCapsule->setPosition(Math::vec3(0.0f, 1.0f, 0.0f));
    Collider* sphereCapsuleCollider = sphereNearCapsule->addComponent<Collider>();
    sphereCapsuleCollider->setSphere(0.3f);
    sphereCapsuleCollider->setType(13);

    scene.update(1.0f / 60.0f);
    scene.collisions().enable(10, 11, CollisionResponse::None);
    scene.collisions().enable(12, 13, CollisionResponse::None);
    scene.collisions().step();

    CHECK(boxCollider->contactCount() == 1);
    CHECK(sphereBoxCollider->contactCount() == 1);
    CHECK(capsuleCollider->contactCount() == 1);
    CHECK(sphereCapsuleCollider->contactCount() == 1);
}

void testMeshColliderReportsContactAgainstSphere()
{
    TriangleOctree ground;
    buildGroundOctree(ground);
    CHECK(!ground.empty());

    Scene scene;
    GameObject* groundObject = scene.createGameObject("Ground");
    Collider* groundCollider = groundObject->addComponent<Collider>();
    groundCollider->setMesh(&ground);
    groundCollider->setType(20);

    GameObject* ball = scene.createGameObject("Ball");
    ball->setPosition(Math::vec3(0.0f, 0.3f, 0.0f));
    Collider* ballCollider = ball->addComponent<Collider>();
    ballCollider->setSphere(0.5f); // reaches from y = -0.2 to y = 0.8, embeds the y = 0 plane
    ballCollider->setType(21);

    scene.update(1.0f / 60.0f);
    scene.collisions().enable(20, 21, CollisionResponse::None);
    scene.collisions().step();

    CHECK(ballCollider->contactCount() >= 1);
    CHECK(groundCollider->contactCount() >= 1);
    if (ballCollider->contactCount() >= 1)
    {
        CHECK(ballCollider->contactAt(0).other == groundCollider);
        CHECK(ballCollider->contactAt(0).normal.y > 0.0f);
    }
    if (groundCollider->contactCount() >= 1)
        CHECK(groundCollider->contactAt(0).normal.y < 0.0f);
}

void testContactsAreClearedBetweenSteps()
{
    Scene scene;
    GameObject* a = scene.createGameObject("SphereA");
    GameObject* b = scene.createGameObject("SphereB");
    b->setPosition(Math::vec3(0.5f, 0.0f, 0.0f));

    Collider* colliderA = a->addComponent<Collider>();
    colliderA->setSphere(1.0f);
    colliderA->setType(8);
    Collider* colliderB = b->addComponent<Collider>();
    colliderB->setSphere(1.0f);
    colliderB->setType(9);

    scene.update(1.0f / 60.0f);
    scene.collisions().enable(8, 9, CollisionResponse::None);
    scene.collisions().step();
    CHECK(colliderA->contactCount() == 1);
    CHECK(colliderB->contactCount() == 1);

    b->setPosition(Math::vec3(100.0f, 0.0f, 0.0f));
    scene.update(1.0f / 60.0f);
    scene.collisions().step();
    CHECK(colliderA->contactCount() == 0);
    CHECK(colliderB->contactCount() == 0);
}

// Objects join a Scene lazily, so scene.update() runs once before serializing.
void testSerializerRoundTripForEveryShapeKind()
{
    TriangleOctree dummyOctree;

    Scene scene;
    GameObject* sphereObject = scene.createGameObject("SphereObj");
    Collider* sphereCollider = sphereObject->addComponent<Collider>();
    sphereCollider->setSphere(0.7f);
    sphereCollider->setType(101);
    sphereCollider->setResponse(CollisionResponse::Slide);

    GameObject* boxObject = scene.createGameObject("BoxObj");
    Collider* boxCollider = boxObject->addComponent<Collider>();
    boxCollider->setBox(Math::vec3(1.0f, 2.0f, 3.0f));
    boxCollider->setType(102);
    boxCollider->setResponse(CollisionResponse::Stop);

    GameObject* capsuleObject = scene.createGameObject("CapsuleObj");
    Collider* capsuleCollider = capsuleObject->addComponent<Collider>();
    capsuleCollider->setCapsule(0.4f, 1.8f);
    capsuleCollider->setType(103);
    capsuleCollider->setResponse(CollisionResponse::SlideXZ);

    GameObject* meshObject = scene.createGameObject("MeshObj");
    Collider* meshCollider = meshObject->addComponent<Collider>();
    meshCollider->setMesh(&dummyOctree);
    meshCollider->setType(104);
    meshCollider->setResponse(CollisionResponse::None);

    scene.update(1.0f / 60.0f);

    SceneSerializer serializer;
    const nlohmann::json document = serializer.toJson(scene);

    Scene reloaded;
    SceneLoadResult result;
    CHECK(serializer.fromJson(document, reloaded, result));
    CHECK(result.success());

    GameObject* reSphereObject = reloaded.findGameObject(sphereObject->id());
    Collider* reSphere = reSphereObject ? reSphereObject->getComponent<Collider>() : nullptr;
    CHECK(reSphere != nullptr);
    if (reSphere)
    {
        CHECK(reSphere->shape() == ColliderShape::Sphere);
        CHECK(near(reSphere->radius(), 0.7f));
        CHECK(reSphere->type() == 101);
        CHECK(reSphere->response() == CollisionResponse::Slide);
    }

    GameObject* reBoxObject = reloaded.findGameObject(boxObject->id());
    Collider* reBox = reBoxObject ? reBoxObject->getComponent<Collider>() : nullptr;
    CHECK(reBox != nullptr);
    if (reBox)
    {
        CHECK(reBox->shape() == ColliderShape::Box);
        CHECK(near(reBox->halfExtents(), Math::vec3(1.0f, 2.0f, 3.0f)));
        CHECK(reBox->type() == 102);
        CHECK(reBox->response() == CollisionResponse::Stop);
    }

    GameObject* reCapsuleObject = reloaded.findGameObject(capsuleObject->id());
    Collider* reCapsule = reCapsuleObject ? reCapsuleObject->getComponent<Collider>() : nullptr;
    CHECK(reCapsule != nullptr);
    if (reCapsule)
    {
        CHECK(reCapsule->shape() == ColliderShape::Capsule);
        CHECK(near(reCapsule->radius(), 0.4f));
        CHECK(near(reCapsule->height(), 1.8f));
        CHECK(reCapsule->type() == 103);
        CHECK(reCapsule->response() == CollisionResponse::SlideXZ);
    }

    GameObject* reMeshObject = reloaded.findGameObject(meshObject->id());
    Collider* reMesh = reMeshObject ? reMeshObject->getComponent<Collider>() : nullptr;
    CHECK(reMesh != nullptr);
    if (reMesh)
    {
        CHECK(reMesh->shape() == ColliderShape::Mesh);
        CHECK(reMesh->type() == 104);
        CHECK(reMesh->response() == CollisionResponse::None);
    }
}

void testFreeMoveReturnsDestinationUnchanged()
{
    Scene scene;
    scene.update(1.0f / 60.0f);

    const Math::vec3 from(0.0f, 0.0f, 0.0f);
    const Math::vec3 to(5.0f, 0.0f, 0.0f);
    CollisionWorld::MoveResult result = scene.collisions().moveSphere(from, to, 0.5f, 1);

    CHECK(near(result.position, to));
    CHECK(result.hitCount == 0);
    CHECK(!result.collided);
}

// Stop rejects the whole step if any part of [from, to] crosses the target; the mover never advances past `from`.
void testStopAgainstWallStaysAtStart()
{
    Scene scene;
    makeBoxWall(scene, "Wall", Math::vec3(3.0f, 0.0f, 0.0f), Math::vec3(0.1f, 2.0f, 2.0f), 2);
    scene.update(1.0f / 60.0f);
    scene.collisions().enable(1, 2, CollisionResponse::Stop);

    const Math::vec3 from(0.0f, 0.0f, 0.0f);
    const Math::vec3 to(5.0f, 0.0f, 0.0f);
    CollisionWorld::MoveResult result = scene.collisions().moveSphere(from, to, 0.5f, 1);

    CHECK(result.hitCount == 1);
    CHECK(result.collided);
    CHECK(near(result.position, from));
    CHECK(result.position.x < 2.4f);
}

// Hand-verified: hit t = 0.48, hitPos (2.4, 2.4, 0), normal (-1, 0, 0); Y stays 5.0, X clips to 2.4 minus the epsilon push-out.
void testSlideAlongWallKeepsTangentLosesNormal()
{
    Scene scene;
    makeBoxWall(scene, "Wall", Math::vec3(3.0f, 0.0f, 0.0f), Math::vec3(0.1f, 2.0f, 2.0f), 3);
    scene.update(1.0f / 60.0f);
    scene.collisions().enable(1, 3, CollisionResponse::Slide);

    const Math::vec3 from(0.0f, 0.0f, 0.0f);
    const Math::vec3 to(5.0f, 5.0f, 0.0f);
    CollisionWorld::MoveResult result = scene.collisions().moveSphere(from, to, 0.5f, 1);

    CHECK(result.hitCount == 1);
    CHECK(near(result.position, Math::vec3(2.399f, 5.0f, 0.0f), 1e-3f));
    CHECK(near(result.lastNormal, Math::vec3(-1.0f, 0.0f, 0.0f)));
}

// Curved wall on an almost-vertical path: XZ length balloons past 0.2 while the 3D length (5.69) fits the plain-Slide budget (10.0); SlideXZ clamps, Slide does not.
void testSlideXZClampsHorizontallyNotVertically()
{
    Scene scene;
    makeSphereWall(scene, "Bulge", Math::vec3(1.0f, 3.0f, 0.0f), 1.2f, 4);
    scene.update(1.0f / 60.0f);
    scene.collisions().enable(1, 4, CollisionResponse::SlideXZ);

    const Math::vec3 from(0.0f, 0.0f, 0.0f);
    const Math::vec3 to(0.2f, 10.0f, 0.0f);
    CollisionWorld::MoveResult result = scene.collisions().moveSphere(from, to, 0.5f, 1);

    CHECK(result.hitCount == 1);
    CHECK(near(result.position, Math::vec3(-0.2006f, 0.2235f, 0.0f), 2e-3f));

    Scene plainScene;
    makeSphereWall(plainScene, "Bulge", Math::vec3(1.0f, 3.0f, 0.0f), 1.2f, 4);
    plainScene.update(1.0f / 60.0f);
    plainScene.collisions().enable(1, 4, CollisionResponse::Slide);
    CollisionWorld::MoveResult plainResult = plainScene.collisions().moveSphere(from, to, 0.5f, 1);

    CHECK(plainResult.hitCount == 1);
    CHECK(near(plainResult.position, Math::vec3(-3.7857f, 4.2455f, 0.0f), 2e-3f));
    CHECK(result.position.y < plainResult.position.y);
}

// Two walls at 90 degrees: both planes registered (hitCount == 2); a second identical move advances less than epsilon, so plane history is remembered.
void testCornerRestsWithoutOscillating()
{
    Scene scene;
    makeBoxWall(scene, "WallX", Math::vec3(3.5f, 0.0f, 0.0f), Math::vec3(0.5f, 5.0f, 5.0f), 5);
    makeBoxWall(scene, "WallZ", Math::vec3(0.0f, 0.0f, 3.5f), Math::vec3(5.0f, 5.0f, 0.5f), 6);
    scene.update(1.0f / 60.0f);
    scene.collisions().enable(1, 5, CollisionResponse::Slide);
    scene.collisions().enable(1, 6, CollisionResponse::Slide);

    const Math::vec3 to(10.0f, 0.0f, 10.0f);
    CollisionWorld::MoveResult first = scene.collisions().moveSphere(Math::vec3(0.0f), to, 0.5f, 1);

    CHECK(first.hitCount == 2);
    CHECK(first.position.x < 2.5f);
    CHECK(first.position.z < 2.5f);

    CollisionWorld::MoveResult second = scene.collisions().moveSphere(first.position, to, 0.5f, 1);
    CHECK(Math::distance(second.position, first.position) < 0.01f);
}

void testUnenabledPairDoesNotStopTheMove()
{
    Scene scene;
    makeBoxWall(scene, "Wall", Math::vec3(3.0f, 0.0f, 0.0f), Math::vec3(0.1f, 2.0f, 2.0f), 2);
    scene.update(1.0f / 60.0f);

    const Math::vec3 from(0.0f, 0.0f, 0.0f);
    const Math::vec3 to(5.0f, 0.0f, 0.0f);
    CollisionWorld::MoveResult result = scene.collisions().moveSphere(from, to, 0.5f, 1);

    CHECK(near(result.position, to));
    CHECK(result.hitCount == 0);
    CHECK(!result.collided);
}

void testAllTargetShapesStopOrSlideCorrectly()
{
    const Math::vec3 from(0.0f, 0.0f, 0.0f);
    const Math::vec3 to(5.0f, 0.0f, 0.0f);

    {
        Scene scene;
        makeSphereWall(scene, "SphereWall", Math::vec3(3.0f, 0.0f, 0.0f), 1.0f, 2);
        scene.update(1.0f / 60.0f);
        scene.collisions().enable(1, 2, CollisionResponse::Stop);
        CollisionWorld::MoveResult result = scene.collisions().moveSphere(from, to, 0.5f, 1);
        CHECK(result.hitCount == 1);
        CHECK(near(result.position, from));
    }
    {
        Scene scene;
        makeBoxWall(scene, "BoxWall", Math::vec3(3.0f, 0.0f, 0.0f), Math::vec3(0.5f, 2.0f, 2.0f), 2);
        scene.update(1.0f / 60.0f);
        scene.collisions().enable(1, 2, CollisionResponse::Stop);
        CollisionWorld::MoveResult result = scene.collisions().moveSphere(from, to, 0.5f, 1);
        CHECK(result.hitCount == 1);
        CHECK(near(result.position, from));
    }
    {
        Scene scene;
        makeCapsuleWall(scene, "CapsuleWall", Math::vec3(3.0f, 0.0f, 0.0f), 0.5f, 4.0f, 2);
        scene.update(1.0f / 60.0f);
        scene.collisions().enable(1, 2, CollisionResponse::Stop);
        CollisionWorld::MoveResult result = scene.collisions().moveSphere(from, to, 0.5f, 1);
        CHECK(result.hitCount == 1);
        CHECK(near(result.position, from));
    }
    {
        TriangleOctree ground;
        buildGroundOctree(ground);

        Scene scene;
        GameObject* groundObject = scene.createGameObject("Ground");
        Collider* groundCollider = groundObject->addComponent<Collider>();
        groundCollider->setMesh(&ground);
        groundCollider->setType(2);
        scene.update(1.0f / 60.0f);
        scene.collisions().enable(1, 2, CollisionResponse::Slide);

        CollisionWorld::MoveResult result = scene.collisions().moveSphere(
            Math::vec3(0.0f, 2.0f, 0.0f), Math::vec3(0.0f, -5.0f, 0.0f), 0.5f, 1);
        CHECK(result.hitCount == 1);
        CHECK(near(result.position, Math::vec3(0.0f, 0.501f, 0.0f), 1e-3f));
    }
}

// maxHits reached mid-corner: the result is the last SAFE position (just off the first wall), never inside either wall.
void testMaxHitsReturnsLastSafePosition()
{
    Scene scene;
    makeBoxWall(scene, "WallX", Math::vec3(3.5f, 0.0f, 0.0f), Math::vec3(0.5f, 5.0f, 5.0f), 5);
    makeBoxWall(scene, "WallZ", Math::vec3(0.0f, 0.0f, 3.5f), Math::vec3(5.0f, 5.0f, 0.5f), 6);
    scene.update(1.0f / 60.0f);
    scene.collisions().enable(1, 5, CollisionResponse::Slide);
    scene.collisions().enable(1, 6, CollisionResponse::Slide);

    const Math::vec3 from(0.0f, 0.0f, 0.0f);
    const Math::vec3 to(10.0f, 0.0f, 10.0f);
    CollisionWorld::MoveResult capped = scene.collisions().moveSphere(from, to, 0.5f, 1, 1);

    CHECK(capped.hitCount == 1);
    CHECK(near(capped.position, Math::vec3(-0.001f, 0.0f, 0.0f), 1e-3f));
    CHECK(capped.position.x < 3.0f);
    CHECK(capped.position.z < 3.0f);

    CollisionWorld::MoveResult uncapped = scene.collisions().moveSphere(from, to, 0.5f, 1);
    CHECK(uncapped.hitCount == 2);
    CHECK(Math::distance(capped.position, uncapped.position) > 0.5f);
}

} // namespace

int main()
{
    testAttachAndReadBackComponent();
    testWorldBoundsFollowTransform();
    testOverlappingSpheresProduceDirectionalContacts();
    testUnregisteredPairProducesNoContacts();
    testDisableAfterEnableStopsContacts();
    testSphereVsBoxAndCapsuleVsSphere();
    testMeshColliderReportsContactAgainstSphere();
    testContactsAreClearedBetweenSteps();
    testSerializerRoundTripForEveryShapeKind();
    testFreeMoveReturnsDestinationUnchanged();
    testStopAgainstWallStaysAtStart();
    testSlideAlongWallKeepsTangentLosesNormal();
    testSlideXZClampsHorizontallyNotVertically();
    testCornerRestsWithoutOscillating();
    testUnenabledPairDoesNotStopTheMove();
    testAllTargetShapesStopOrSlideCorrectly();
    testMaxHitsReturnsLastSafePosition();

    if (gFailures)
        std::fprintf(stderr, "%d scene collision test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
