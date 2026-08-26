#include "PCH.h"

#include "collision/Broadphase.h"
#include "collision/CollisionShape.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/RigidBody.h"
#include "softbody/SoftBody.h"

#include <chrono>
#include <cstring>
#include <cstdio>
#include <deque>

using namespace Radion;
using namespace Radion::Physics;

namespace
{

f64 milliseconds(std::chrono::steady_clock::time_point begin,
                 std::chrono::steady_clock::time_point end)
{
    return std::chrono::duration<f64, std::milli>(end - begin).count();
}

// A pile of falling boxes: broadphase, narrowphase and solver all under
// load at once. Reported per step over the second half of the run, once the
// pile has mostly landed and the contact count is at its worst.
void benchBoxPile(u32 count)
{
    BoxShape groundShape(Math::vec3(60.0f, 0.5f, 60.0f));
    BoxShape boxShape(Math::vec3(0.5f));

    PhysicsWorld world;
    world.setGravity(Math::vec3(0.0f, -9.81f, 0.0f));
    world.setFixedStep(1.0f / 60.0f);

    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::vec3(0.0f, -0.5f, 0.0f));
    BodyEntry groundEntry;
    groundEntry.body = &ground;
    groundEntry.shape = &groundShape;
    groundEntry.friction = 0.8f;
    world.addBody(groundEntry);

    std::deque<RigidBody> bodies;
    const u32 side = static_cast<u32>(std::cbrt(static_cast<f64>(count))) + 1;
    u32 spawned = 0;
    for (u32 y = 0; y < side && spawned < count; ++y)
        for (u32 x = 0; x < side && spawned < count; ++x)
            for (u32 z = 0; z < side && spawned < count; ++z)
            {
                bodies.emplace_back();
                RigidBody& body = bodies.back();
                body.setMass(1.0f);
                body.setInertiaTensor(Inertia::box(1.0f, Math::vec3(0.5f)));
                body.setPosition(Math::vec3(static_cast<f32>(x) * 1.1f -
                                               static_cast<f32>(side) * 0.55f,
                                           1.0f + static_cast<f32>(y) * 1.1f,
                                           static_cast<f32>(z) * 1.1f -
                                               static_cast<f32>(side) * 0.55f));
                BodyEntry entry;
                entry.body = &body;
                entry.shape = &boxShape;
                entry.friction = 0.6f;
                world.addBody(entry);
                ++spawned;
            }

    constexpr u32 kSteps = 240;
    f64 total = 0.0;
    f64 worst = 0.0;
    for (u32 step = 0; step < kSteps; ++step)
    {
        const auto begin = std::chrono::steady_clock::now();
        world.step(1.0f / 60.0f);
        const auto end = std::chrono::steady_clock::now();
        if (step >= kSteps / 2)
        {
            const f64 elapsed = milliseconds(begin, end);
            total += elapsed;
            worst = elapsed > worst ? elapsed : worst;
        }
    }
    std::printf("box pile %4u bodies: %6.3f ms/step avg, %6.3f ms worst\n", count,
                total / static_cast<f64>(kSteps / 2), worst);
}

void countContactEvent(const ContactEventInfo&, void* userData)
{
    ++*static_cast<u64*>(userData);
}

void benchContactEvents(u32 count)
{
    BoxShape groundShape(Math::vec3(60.0f, 0.5f, 60.0f));
    BoxShape boxShape(Math::vec3(0.5f));

    PhysicsWorld world;
    world.setGravity(Math::vec3(0.0f, -9.81f, 0.0f));
    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::vec3(0.0f, -0.5f, 0.0f));
    BodyEntry groundEntry;
    groundEntry.body = &ground;
    groundEntry.shape = &groundShape;
    world.addBody(groundEntry);

    std::deque<RigidBody> bodies;
    const u32 side = static_cast<u32>(std::cbrt(static_cast<f64>(count))) + 1;
    u32 spawned = 0;
    for (u32 y = 0; y < side && spawned < count; ++y)
        for (u32 x = 0; x < side && spawned < count; ++x)
            for (u32 z = 0; z < side && spawned < count; ++z)
            {
                bodies.emplace_back();
                RigidBody& body = bodies.back();
                body.setMass(1.0f);
                body.setInertiaTensor(Inertia::box(1.0f, Math::vec3(0.5f)));
                body.setPosition(Math::vec3(static_cast<f32>(x) * 1.01f -
                                               static_cast<f32>(side) * 0.505f,
                                           1.0f + static_cast<f32>(y) * 1.01f,
                                           static_cast<f32>(z) * 1.01f -
                                               static_cast<f32>(side) * 0.505f));
                BodyEntry entry;
                entry.body = &body;
                entry.shape = &boxShape;
                world.addBody(entry);
                ++spawned;
            }

    u64 eventCount = 0;
    world.setEventCallback(countContactEvent, &eventCount);
    constexpr u32 kSteps = 240;
    f64 total = 0.0;
    for (u32 step = 0; step < kSteps; ++step)
    {
        const auto begin = std::chrono::steady_clock::now();
        world.step(1.0f / 60.0f);
        const auto end = std::chrono::steady_clock::now();
        if (step >= kSteps / 2)
            total += milliseconds(begin, end);
    }
    std::printf("contact events %4u bodies: %6.3f ms/step avg, %llu callbacks\n", count,
                total / static_cast<f64>(kSteps / 2),
                static_cast<unsigned long long>(eventCount));
}

void benchBroadphase(u32 count, bool overlapping, bool movable = true)
{
    Broadphase broadphase;
    broadphase.reserve(count);
    for (u32 index = 0; index < count; ++index)
    {
        const Math::vec3 center = overlapping
                                     ? Math::vec3(0.0f)
                                     : Math::vec3(static_cast<f32>(index % 64) * 3.0f,
                                                 static_cast<f32>((index / 64) % 32) * 3.0f,
                                                 static_cast<f32>(index / 2048) * 3.0f);
        BroadphaseProxy proxy;
        proxy.id = index;
        proxy.movable = movable;
        proxy.bounds.min = center - Math::vec3(0.5f);
        proxy.bounds.max = center + Math::vec3(0.5f);
        broadphase.add(proxy);
    }

    std::vector<BroadphasePair> pairs;
    constexpr u32 kRuns = 32;
    f64 total = 0.0;
    for (u32 run = 0; run < kRuns; ++run)
    {
        const auto begin = std::chrono::steady_clock::now();
        broadphase.findPairs(pairs);
        total += milliseconds(begin, std::chrono::steady_clock::now());
    }
    std::printf("broadphase %s %s %5u proxies: %7.3f ms/findPairs, %zu pairs\n",
                movable ? "moving" : "static", overlapping ? "overlap" : "sparse ", count,
                total / kRuns, pairs.size());
}

void benchStaticBvh(u32 staticCount, u32 kinematicCount)
{
    BoxShape shape(Math::vec3(0.5f));
    PhysicsWorld world;
    std::deque<RigidBody> staticBodies;
    for (u32 index = 0; index < staticCount; ++index)
    {
        staticBodies.emplace_back();
        RigidBody& body = staticBodies.back();
        body.setBodyType(BodyType::Static);
        body.setPosition(Math::vec3(static_cast<f32>(index % 64) * 3.0f,
                                   static_cast<f32>((index / 64) % 32) * 3.0f,
                                   static_cast<f32>(index / 2048) * 3.0f));
        BodyEntry entry;
        entry.body = &body;
        entry.shape = &shape;
        world.addBody(entry);
    }

    std::deque<RigidBody> kinematicBodies;
    for (u32 index = 0; index < kinematicCount; ++index)
    {
        kinematicBodies.emplace_back();
        RigidBody& body = kinematicBodies.back();
        body.setBodyType(BodyType::Kinematic);
        body.setPosition(Math::vec3(static_cast<f32>(index) * 3.0f + 0.75f, 100.0f, 0.0f));
        BodyEntry entry;
        entry.body = &body;
        entry.shape = &shape;
        world.addBody(entry);
    }

    constexpr u32 kSteps = 120;
    f64 total = 0.0;
    for (u32 step = 0; step < kSteps; ++step)
    {
        const auto begin = std::chrono::steady_clock::now();
        world.step(1.0f / 60.0f);
        total += milliseconds(begin, std::chrono::steady_clock::now());
    }
    std::printf("static BVH %u static + %u kinematic: %7.3f ms/step avg\n", staticCount,
                kinematicCount, total / kSteps);
}

void benchRaycasts(u32 bodyCount, u32 rayCount)
{
    BoxShape groundShape(Math::vec3(60.0f, 0.5f, 60.0f));
    BoxShape boxShape(Math::vec3(0.5f));

    PhysicsWorld world;
    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::vec3(0.0f, -0.5f, 0.0f));
    BodyEntry groundEntry;
    groundEntry.body = &ground;
    groundEntry.shape = &groundShape;
    world.addBody(groundEntry);

    std::deque<RigidBody> bodies;
    for (u32 i = 0; i < bodyCount; ++i)
    {
        bodies.emplace_back();
        RigidBody& body = bodies.back();
        body.setBodyType(BodyType::Static);
        body.setPosition(Math::vec3(static_cast<f32>(i % 32) * 2.0f - 32.0f, 0.5f,
                                   static_cast<f32>(i / 32) * 2.0f - 32.0f));
        BodyEntry entry;
        entry.body = &body;
        entry.shape = &boxShape;
        world.addBody(entry);
    }

    u32 hits = 0;
    const auto begin = std::chrono::steady_clock::now();
    for (u32 i = 0; i < rayCount; ++i)
    {
        Ray ray;
        ray.origin = Math::vec3(static_cast<f32>(i % 64) - 32.0f, 10.0f,
                               static_cast<f32>((i / 64) % 64) - 32.0f);
        ray.direction = Math::vec3(0.0f, -1.0f, 0.0f);
        WorldRayHit hit;
        if (world.raycast(ray, 50.0f, QueryFilter(), hit))
            ++hits;
    }
    const auto end = std::chrono::steady_clock::now();
    const f64 elapsed = milliseconds(begin, end);
    std::printf("raycasts %u vs %u bodies: %6.3f ms total, %.2f us/ray (%u hits)\n", rayCount,
                bodyCount + 1, elapsed, 1000.0 * elapsed / static_cast<f64>(rayCount), hits);
}

void benchSoftBody()
{
    std::vector<Math::vec3> positions;
    std::vector<u32> indices;
    const u32 side = 45;
    for (u32 r = 0; r < side; ++r)
        for (u32 c = 0; c < side; ++c)
            positions.push_back(Math::vec3(static_cast<f32>(c) * 0.136f - 3.0f, 2.0f,
                                          static_cast<f32>(r) * 0.136f - 3.0f));
    for (u32 r = 0; r + 1 < side; ++r)
        for (u32 c = 0; c + 1 < side; ++c)
        {
            const u32 i0 = r * side + c;
            const u32 i1 = i0 + 1;
            const u32 i2 = i0 + side;
            const u32 i3 = i2 + 1;
            indices.insert(indices.end(), {i0, i2, i1, i1, i2, i3});
        }

    SoftBody body;
    body.setParticles(positions.data(), static_cast<u32>(positions.size()), 4.0f);
    body.buildFromMesh(indices.data(), static_cast<u32>(indices.size()), 0.0f, 1.0e-4f);
    body.setCollisionMargin(0.02f);

    PhysicsWorld world;
    PlaneShape groundShape(Math::vec3(0.0f, 1.0f, 0.0f));
    RigidBody ground;
    ground.setBodyType(BodyType::Static);
    ground.setPosition(Math::vec3(0.0f));
    BodyEntry groundEntry;
    groundEntry.body = &ground;
    groundEntry.shape = &groundShape;
    groundEntry.friction = 0.6f;
    world.addBody(groundEntry);
    body.setCollisionWorld(&world);

    constexpr u32 kSteps = 240;
    f64 total = 0.0;
    for (u32 step = 0; step < kSteps; ++step)
    {
        const auto begin = std::chrono::steady_clock::now();
        body.step(1.0f / 120.0f, 8);
        const auto end = std::chrono::steady_clock::now();
        if (step >= kSteps / 2)
            total += milliseconds(begin, end);
    }
    std::printf("soft body 45x45, 8 substeps: %6.3f ms/step avg\n",
                total / static_cast<f64>(kSteps / 2));
}

void buildTerrainGrid(u32 quadsPerSide, std::vector<Math::vec3>& positions,
                      std::vector<u32>& indices)
{
    const u32 verticesPerSide = quadsPerSide + 1;
    positions.clear();
    positions.reserve(static_cast<usize>(verticesPerSide) * verticesPerSide);
    for (u32 z = 0; z < verticesPerSide; ++z)
        for (u32 x = 0; x < verticesPerSide; ++x)
        {
            const f32 fx = static_cast<f32>(x) * 0.1f - static_cast<f32>(quadsPerSide) * 0.05f;
            const f32 fz = static_cast<f32>(z) * 0.1f - static_cast<f32>(quadsPerSide) * 0.05f;
            positions.push_back(Math::vec3(fx, 0.3f * std::sin(fx * 0.7f) * std::cos(fz * 0.6f),
                                          fz));
        }
    indices.clear();
    indices.reserve(static_cast<usize>(quadsPerSide) * quadsPerSide * 6);
    for (u32 z = 0; z < quadsPerSide; ++z)
        for (u32 x = 0; x < quadsPerSide; ++x)
        {
            const u32 i0 = z * verticesPerSide + x;
            const u32 i1 = i0 + 1;
            const u32 i2 = i0 + verticesPerSide;
            const u32 i3 = i2 + 1;
            indices.insert(indices.end(), {i0, i2, i1, i1, i2, i3});
        }
}

// Free soft-body particles thrown against a level-sized trimesh - the blood
// splash zombie_night spawns per hit, several of them alive at once. What it
// measures is the per-particle contact search: a search radius tied to
// anything other than the particle's own travel turns each of these into a
// walk over thousands of the level's triangles.
void benchSoftBodySplashes(u32 splashCount, u32 particlesPerSplash)
{
    std::vector<Math::vec3> positions;
    std::vector<u32> indices;
    buildTerrainGrid(400, positions, indices);
    TrimeshShape mesh(positions.data(), static_cast<u32>(positions.size()), indices.data(),
                      static_cast<u32>(indices.size()));

    PhysicsWorld world;
    world.setGravity(Math::vec3(0.0f, -20.0f, 0.0f));
    RigidBody meshBody;
    meshBody.setBodyType(BodyType::Static);
    meshBody.setPosition(Math::vec3(0.0f));
    BodyEntry meshEntry;
    meshEntry.body = &meshBody;
    meshEntry.shape = &mesh;
    meshEntry.friction = 0.8f;
    world.addBody(meshEntry);

    std::deque<SoftBody> splashes;
    for (u32 s = 0; s < splashCount; ++s)
    {
        std::vector<Math::vec3> particles(particlesPerSplash,
                                         Math::vec3(static_cast<f32>(s) * 0.5f, 1.5f, 0.0f));
        splashes.emplace_back();
        SoftBody& splash = splashes.back();
        splash.setParticles(particles.data(), particlesPerSplash, 0.05f);
        splash.setGravity(world.gravity());
        splash.setDamping(0.98f);
        splash.setCollisionMargin(0.02f);
        splash.setCollisionWorld(&world);
        for (u32 p = 0; p < particlesPerSplash; ++p)
        {
            const f32 angle = static_cast<f32>(p) * 0.8f;
            splash.particle(p).velocity =
                3.0f * Math::vec3(std::cos(angle), 1.2f, std::sin(angle));
        }
    }

    constexpr u32 kSteps = 180;
    f64 total = 0.0;
    f64 worst = 0.0;
    for (u32 step = 0; step < kSteps; ++step)
    {
        const auto begin = std::chrono::steady_clock::now();
        for (SoftBody& splash : splashes)
            splash.step(1.0f / 60.0f, 4);
        const auto end = std::chrono::steady_clock::now();
        const f64 elapsed = milliseconds(begin, end);
        total += elapsed;
        worst = Math::max(worst, elapsed);
    }

    // Without this the timing means nothing: a search radius small enough to
    // find no surface at all is the fastest possible answer and the wrong
    // one. Every particle is thrown upward from above the mesh, so by the
    // last step every one of them must be resting on it.
    u32 landed = 0;
    for (const SoftBody& splash : splashes)
        for (u32 p = 0; p < splash.particleCount(); ++p)
            if (splash.contact(p).active)
                ++landed;

    std::printf("%u splashes x %u particles vs 320k-tri mesh: %6.3f ms/frame avg, %6.3f ms "
                "worst, %u/%u resting on it\n",
                splashCount, particlesPerSplash, total / static_cast<f64>(kSteps), worst, landed,
                splashCount * particlesPerSplash);
}

// A million-triangle static mesh, the scale of a whole scanned-in level
// used as one collider. Build cost, ray cost through its tree, and dynamic
// boxes resting on it are the three numbers that decide whether a big map
// can simply BE the collision world.
void benchLargeTrimesh(u32 quadsPerSide)
{
    std::vector<Math::vec3> positions;
    std::vector<u32> indices;
    buildTerrainGrid(quadsPerSide, positions, indices);

    const auto buildBegin = std::chrono::steady_clock::now();
    TrimeshShape mesh(positions.data(), static_cast<u32>(positions.size()), indices.data(),
                      static_cast<u32>(indices.size()));
    const auto buildEnd = std::chrono::steady_clock::now();

    PhysicsWorld world;
    world.setGravity(Math::vec3(0.0f, -9.81f, 0.0f));
    world.setFixedStep(1.0f / 60.0f);
    RigidBody meshBody;
    meshBody.setBodyType(BodyType::Static);
    meshBody.setPosition(Math::vec3(0.0f));
    BodyEntry meshEntry;
    meshEntry.body = &meshBody;
    meshEntry.shape = &mesh;
    meshEntry.friction = 0.7f;
    world.addBody(meshEntry);

    constexpr u32 kRays = 10000;
    u32 hits = 0;
    const auto rayBegin = std::chrono::steady_clock::now();
    for (u32 i = 0; i < kRays; ++i)
    {
        Ray ray;
        ray.origin = Math::vec3(static_cast<f32>(i % 100) * 0.37f - 18.0f, 10.0f,
                               static_cast<f32>(i / 100) * 0.31f - 15.0f);
        ray.direction = Math::vec3(0.0f, -1.0f, 0.0f);
        WorldRayHit hit;
        if (world.raycast(ray, 50.0f, QueryFilter(), hit))
            ++hits;
    }
    const auto rayEnd = std::chrono::steady_clock::now();

    BoxShape boxShape(Math::vec3(0.25f));
    std::deque<RigidBody> boxes;
    for (u32 i = 0; i < 128; ++i)
    {
        boxes.emplace_back();
        RigidBody& body = boxes.back();
        body.setMass(1.0f);
        body.setInertiaTensor(Inertia::box(1.0f, Math::vec3(0.25f)));
        body.setPosition(Math::vec3(static_cast<f32>(i % 12) * 0.6f - 3.6f,
                                   2.0f + static_cast<f32>(i / 12) * 0.6f,
                                   static_cast<f32>(i % 7) * 0.5f - 1.75f));
        BodyEntry entry;
        entry.body = &body;
        entry.shape = &boxShape;
        entry.friction = 0.6f;
        world.addBody(entry);
    }

    constexpr u32 kSteps = 240;
    f64 stepTotal = 0.0;
    f64 stepWorst = 0.0;
    for (u32 step = 0; step < kSteps; ++step)
    {
        const auto begin = std::chrono::steady_clock::now();
        world.step(1.0f / 60.0f);
        const auto end = std::chrono::steady_clock::now();
        if (step >= kSteps / 2)
        {
            const f64 elapsed = milliseconds(begin, end);
            stepTotal += elapsed;
            stepWorst = elapsed > stepWorst ? elapsed : stepWorst;
        }
    }

    const u32 triangleCount = quadsPerSide * quadsPerSide * 2;
    std::printf("trimesh %u tris: build %.1f ms, ray %.2f us (%u/%u hits), "
                "128 boxes on it %6.3f ms/step avg %6.3f worst\n",
                triangleCount, milliseconds(buildBegin, buildEnd),
                1000.0 * milliseconds(rayBegin, rayEnd) / static_cast<f64>(kRays), hits, kRays,
                stepTotal / static_cast<f64>(kSteps / 2), stepWorst);
}

} // namespace

int main(int argc, char** argv)
{
    if (argc == 2 && std::strcmp(argv[1], "softbody") == 0)
    {
        benchSoftBody();
        benchSoftBodySplashes(6, 8);
        return 0;
    }
    if (argc == 2 && std::strcmp(argv[1], "events") == 0)
    {
        benchContactEvents(128);
        return 0;
    }
    if (argc == 2 && std::strcmp(argv[1], "broadphase") == 0)
    {
        benchBroadphase(1000, false);
        benchBroadphase(5000, false);
        benchBroadphase(512, true);
        benchBroadphase(5000, false, false);
        return 0;
    }
    if (argc == 2 && std::strcmp(argv[1], "static-bvh") == 0)
    {
        benchStaticBvh(5000, 64);
        return 0;
    }

    benchLargeTrimesh(708);
    benchBoxPile(128);
    benchBoxPile(512);
    benchBoxPile(1024);
    benchRaycasts(512, 100000);
    benchSoftBody();
    benchSoftBodySplashes(6, 8);
    return 0;
}
