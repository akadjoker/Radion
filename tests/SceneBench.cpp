#include "PCH.h"

#include "Collider.h"
#include "CollisionWorld.h"
#include "GameObject.h"
#include "Scene.h"

#include <chrono>
#include <cstdio>

using namespace Radion;

namespace
{

f64 milliseconds(std::chrono::steady_clock::time_point begin,
                 std::chrono::steady_clock::time_point end)
{
    return std::chrono::duration<f64, std::milli>(end - begin).count();
}

// Objects with no components at all: what one Scene::update() costs before a
// single line of game logic runs. The scene is flat, so nothing here pays for
// a deep hierarchy either - this is the floor.
void benchIdleUpdate(u32 count, u32 frames)
{
    Scene scene;
    for (u32 i = 0; i < count; ++i)
        scene.createGameObject("object");
    scene.update(0.0f);

    const auto begin = std::chrono::steady_clock::now();
    for (u32 frame = 0; frame < frames; ++frame)
        scene.update(1.0f / 60.0f);
    const auto end = std::chrono::steady_clock::now();

    const f64 perFrame = milliseconds(begin, end) / static_cast<f64>(frames);
    std::printf("  idle update  %6u objects  %8.3f ms/frame  %7.1f us/1k objects\n", count,
                perFrame, perFrame * 1000.0 / (static_cast<f64>(count) / 1000.0));
}

// The same scene with one moving object, to separate what the update costs
// because something changed from what it costs regardless.
void benchOneMoverUpdate(u32 count, u32 frames)
{
    Scene scene;
    GameObject* mover = scene.createGameObject("mover");
    for (u32 i = 0; i < count; ++i)
        scene.createGameObject("object");
    scene.update(0.0f);

    const auto begin = std::chrono::steady_clock::now();
    for (u32 frame = 0; frame < frames; ++frame)
    {
        mover->setPosition(glm::vec3(static_cast<f32>(frame), 0.0f, 0.0f));
        scene.update(1.0f / 60.0f);
    }
    const auto end = std::chrono::steady_clock::now();

    std::printf("  one mover    %6u objects  %8.3f ms/frame\n", count,
                milliseconds(begin, end) / static_cast<f64>(frames));
}

// Spheres on a wide grid, spaced far enough apart that almost no pair ever
// touches. Whatever this costs is pair bookkeeping, not contact solving.
void benchCollisionStep(u32 count, u32 frames)
{
    Scene scene;
    scene.collisions().enable(0, 0, CollisionResponse::None);

    const u32 side = static_cast<u32>(std::sqrt(static_cast<f64>(count))) + 1;
    for (u32 i = 0; i < count; ++i)
    {
        GameObject* object = scene.createGameObject("collider");
        object->setPosition(glm::vec3(static_cast<f32>(i % side) * 20.0f, 0.0f,
                                      static_cast<f32>(i / side) * 20.0f));
        Collider* collider = object->addComponent<Collider>();
        collider->setSphere(1.0f);
        collider->setType(0);
    }
    scene.update(0.0f);

    const auto begin = std::chrono::steady_clock::now();
    for (u32 frame = 0; frame < frames; ++frame)
        scene.collisions().step();
    const auto end = std::chrono::steady_clock::now();

    const f64 perFrame = milliseconds(begin, end) / static_cast<f64>(frames);
    const f64 pairs = static_cast<f64>(count) * static_cast<f64>(count - 1) / 2.0;
    std::printf("  collision    %6u colliders %8.3f ms/frame  %.0f pairs tested\n", count,
                perFrame, pairs);
}

} // namespace

int main()
{
    std::printf("Scene::update, no components\n");
    benchIdleUpdate(1000, 300);
    benchIdleUpdate(10000, 100);
    benchIdleUpdate(50000, 40);

    std::printf("\nScene::update, one object moving\n");
    benchOneMoverUpdate(10000, 100);
    benchOneMoverUpdate(50000, 40);

    std::printf("\nCollisionWorld::step, spheres too far apart to touch\n");
    benchCollisionStep(200, 200);
    benchCollisionStep(500, 100);
    benchCollisionStep(1000, 40);
    benchCollisionStep(2000, 20);
    return 0;
}
