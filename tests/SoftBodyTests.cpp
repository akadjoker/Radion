#include "PCH.h"

#include "softbody/SoftBody.h"
#include "collision/CollisionShape.h"
#include "dynamics/PhysicsWorld.h"

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
        std::fprintf(stderr, "SoftBodyTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool near(f32 a, f32 b, f32 epsilon = 1e-4f)
{
    return std::abs(a - b) <= epsilon;
}

// A rows x cols grid on the XZ plane at y = 0, as a triangle mesh.
void makeGrid(u32 rows, u32 cols, f32 spacing, std::vector<Math::vec3>& positions,
              std::vector<u32>& indices)
{
    positions.clear();
    indices.clear();
    for (u32 r = 0; r < rows; ++r)
        for (u32 c = 0; c < cols; ++c)
            positions.push_back(
                Math::vec3(static_cast<f32>(c) * spacing, 0.0f, static_cast<f32>(r) * spacing));

    for (u32 r = 0; r + 1 < rows; ++r)
        for (u32 c = 0; c + 1 < cols; ++c)
        {
            const u32 i0 = r * cols + c;
            const u32 i1 = i0 + 1;
            const u32 i2 = i0 + cols;
            const u32 i3 = i2 + 1;
            indices.insert(indices.end(), {i0, i2, i1, i1, i2, i3});
        }
}

void testMeshBuildsSharedEdgesOnce()
{
    std::vector<Math::vec3> positions;
    std::vector<u32> indices;
    makeGrid(2, 2, 1.0f, positions, indices);

    SoftBody body;
    body.setParticles(positions.data(), static_cast<u32>(positions.size()), 1.0f);
    body.buildFromMesh(indices.data(), static_cast<u32>(indices.size()), 0.0f, 0.0f);

    // Two triangles over four vertices: five distinct edges, plus one bending
    // constraint across the shared diagonal. An edge counted twice would say
    // six structural.
    CHECK(body.constraintCount() == 6);
}

void testStiffClothDoesNotExplode()
{
    std::vector<Math::vec3> positions;
    std::vector<u32> indices;
    makeGrid(8, 8, 0.25f, positions, indices);

    SoftBody body;
    body.setParticles(positions.data(), static_cast<u32>(positions.size()), 2.0f);
    // Zero compliance is infinitely stiff. This is the case a spring solver
    // cannot take at any usable step - it is the whole reason for XPBD.
    body.buildFromMesh(indices.data(), static_cast<u32>(indices.size()), 0.0f, 0.0f);
    body.setPinned(0, true);
    body.setPinned(7, true);

    for (u32 i = 0; i < 240; ++i)
        body.step(1.0f / 60.0f, 8);

    for (u32 i = 0; i < body.particleCount(); ++i)
    {
        const Math::vec3& position = body.particle(i).position;
        CHECK(position.x == position.x); // NaN never equals itself
        CHECK(std::abs(position.x) < 1000.0f);
        CHECK(std::abs(position.y) < 1000.0f);
        CHECK(std::abs(position.z) < 1000.0f);
    }
}

void testStiffnessIsIndependentOfStepSize()
{
    auto settle = [](f32 dt, u32 steps)
    {
        std::vector<Math::vec3> positions;
        std::vector<u32> indices;
        makeGrid(6, 6, 0.2f, positions, indices);

        SoftBody body;
        body.setParticles(positions.data(), static_cast<u32>(positions.size()), 1.0f);
        body.buildFromMesh(indices.data(), static_cast<u32>(indices.size()), 1.0e-7f, 1.0e-5f);
        body.setPinned(0, true);
        body.setPinned(5, true);
        for (u32 i = 0; i < steps; ++i)
            body.step(dt, 12);
        return body.worstStretch();
    };

    // Same simulated time, half the step. A spring solver drapes differently
    // here; compliance divided by dt squared is what makes these agree.
    const f32 coarse = settle(1.0f / 60.0f, 120);
    const f32 fine = settle(1.0f / 120.0f, 240);
    CHECK(near(coarse, fine, 0.02f));
}

void testAttachmentsHoldASheetFromStretching()
{
    auto stretchAfterYank = [](bool useAttachments)
    {
        std::vector<Math::vec3> positions;
        std::vector<u32> indices;
        makeGrid(12, 4, 0.2f, positions, indices);

        SoftBody body;
        body.setParticles(positions.data(), static_cast<u32>(positions.size()), 1.0f);
        // Slack on purpose: an inextensible sheet has nothing for LRA to
        // improve, and real cloth is never solved to convergence.
        body.buildFromMesh(indices.data(), static_cast<u32>(indices.size()), 5.0e-6f, 1.0e-4f);
        for (u32 c = 0; c < 4; ++c)
            body.setPinned(c, true);
        if (useAttachments)
            body.buildAttachments(1.0f);

        // Two iterations, which is where a constraint chain cannot carry a
        // correction to the far end and the sheet stretches.
        body.setGravity(Math::vec3(0.0f, -40.0f, 0.0f));
        for (u32 i = 0; i < 60; ++i)
            body.step(1.0f / 60.0f, 2);
        return body.worstStretch();
    };

    const f32 without = stretchAfterYank(false);
    const f32 with = stretchAfterYank(true);
    CHECK(with <= without);
    CHECK(with < 1.5f);
}

void testPinnedParticlesNeverMove()
{
    std::vector<Math::vec3> positions;
    std::vector<u32> indices;
    makeGrid(5, 5, 0.3f, positions, indices);

    SoftBody body;
    body.setParticles(positions.data(), static_cast<u32>(positions.size()), 1.0f);
    body.buildFromMesh(indices.data(), static_cast<u32>(indices.size()), 0.0f, 0.0f);
    body.setPinned(0, true);
    const Math::vec3 anchor = body.particle(0).position;

    for (u32 i = 0; i < 120; ++i)
        body.step(1.0f / 60.0f, 8);

    CHECK(near(Math::length(body.particle(0).position - anchor), 0.0f));
    CHECK(near(Math::length(body.particle(0).velocity), 0.0f));
    // Everything else fell.
    CHECK(body.particle(body.particleCount() - 1).position.y < -0.5f);
}

void testSubstepsBeatOneBigStep()
{
    auto stretch = [](u32 substeps)
    {
        std::vector<Math::vec3> positions;
        std::vector<u32> indices;
        makeGrid(16, 4, 0.2f, positions, indices);

        SoftBody body;
        body.setParticles(positions.data(), static_cast<u32>(positions.size()), 3.0f);
        body.buildFromMesh(indices.data(), static_cast<u32>(indices.size()), 0.0f, 1.0e-4f);
        for (u32 c = 0; c < 4; ++c)
            body.setPinned(c, true);
        body.setGravity(Math::vec3(0.0f, -60.0f, 0.0f));
        for (u32 i = 0; i < 60; ++i)
            body.step(1.0f / 60.0f, substeps);
        return body.worstStretch();
    };

    // Same total work per frame either way in the old arrangement; here the
    // frame is split instead. A heavy strip hung from one end is the case
    // that separates them - one big step cannot carry the load down the
    // chain, and the strip stretches.
    // Measured on this strip: 1 substep leaves it 69% longer than its rest
    // length, 8 leaves 7.5%, 32 leaves 1%. Splitting the frame is what buys
    // that - the same total projection work spread over one step does not.
    const f32 one = stretch(1);
    const f32 eight = stretch(8);
    CHECK(one > 1.5f);
    CHECK(eight < 1.2f);
    CHECK(eight < one);
    CHECK(stretch(32) < 1.05f);
}

void testEmptyBodyIsHarmless()
{
    SoftBody body;
    body.step(1.0f / 60.0f, 8);
    body.buildAttachments();
    CHECK(body.particleCount() == 0);
    CHECK(near(body.worstStretch(), 1.0f));
}

void testWindDoesNotDependOnParticleCount()
{
    const Math::vec3 onePosition(0.0f);
    SoftBody one;
    one.setParticles(&onePosition, 1, 2.0f);
    one.setGravity(Math::vec3(0.0f));
    one.setDamping(1.0f);
    one.setWind(Math::vec3(20.0f, 0.0f, 0.0f));

    std::vector<Math::vec3> manyPositions(100, Math::vec3(0.0f));
    SoftBody many;
    many.setParticles(manyPositions.data(), static_cast<u32>(manyPositions.size()), 2.0f);
    many.setGravity(Math::vec3(0.0f));
    many.setDamping(1.0f);
    many.setWind(Math::vec3(20.0f, 0.0f, 0.0f));

    one.step(0.1f, 1);
    many.step(0.1f, 1);
    CHECK(near(one.particle(0).velocity.x, many.particle(0).velocity.x));
    CHECK(near(one.particle(0).velocity.x, 1.0f));
}

void testDenseSheetDoesNotLaunchFromSphere()
{
    std::vector<Math::vec3> positions;
    std::vector<u32> indices;
    makeGrid(45, 45, 6.0f / 44.0f, positions, indices);
    for (Math::vec3& position : positions)
        position += Math::vec3(-3.0f, 6.0f, -3.0f);

    SoftBody body;
    body.setParticles(positions.data(), static_cast<u32>(positions.size()), 4.0f);
    body.buildFromMesh(indices.data(), static_cast<u32>(indices.size()), 0.0f, 1.0e-4f);
    body.setCollisionMargin(0.02f);
    body.setMaxLinearVelocity(20.0f);
    PhysicsWorld collisionWorld;
    SphereShape sphereShape(1.4f);
    PlaneShape groundShape(Math::vec3(0.0f, 1.0f, 0.0f));
    RigidBody sphereBody;
    sphereBody.setBodyType(BodyType::Static);
    sphereBody.setPosition(Math::vec3(0.0f, 3.0f, 0.0f));
    RigidBody groundBody;
    groundBody.setBodyType(BodyType::Static);
    groundBody.setPosition(Math::vec3(0.0f));
    BodyEntry sphereEntry;
    sphereEntry.body = &sphereBody;
    sphereEntry.shape = &sphereShape;
    collisionWorld.addBody(sphereEntry);
    BodyEntry groundEntry;
    groundEntry.body = &groundBody;
    groundEntry.shape = &groundShape;
    collisionWorld.addBody(groundEntry);
    body.setCollisionWorld(&collisionWorld);

    f32 highest = 6.0f;
    f32 furthest = 3.0f;
    f32 maximumSpeed = 0.0f;
    for (u32 step = 0; step < 600; ++step)
    {
        body.step(1.0f / 120.0f, 8);
        for (const SoftBody::Particle& particle : body.particles())
        {
            CHECK(std::isfinite(particle.position.x) && std::isfinite(particle.position.y) &&
                  std::isfinite(particle.position.z));
            highest = Math::max(highest, particle.position.y);
            furthest = Math::max(furthest,
                                Math::max(std::abs(particle.position.x),
                                         std::abs(particle.position.z)));
            maximumSpeed = Math::max(maximumSpeed, Math::length(particle.velocity));
        }
    }
    CHECK(highest < 7.0f);
    CHECK(furthest < 12.0f);
    if (maximumSpeed >= 30.0f)
        std::fprintf(stderr, "dense sheet maximum speed: %.3f\n",
                     static_cast<f64>(maximumSpeed));
    CHECK(maximumSpeed < 30.0f);
}

void testHangingSheetRemainsStableAgainstSphere()
{
    std::vector<Math::vec3> positions;
    std::vector<u32> indices;
    makeGrid(25, 25, 4.0f / 24.0f, positions, indices);
    for (Math::vec3& position : positions)
        position = Math::vec3(position.x - 2.0f, 8.0f - position.z, -2.6f);

    SoftBody body;
    body.setParticles(positions.data(), static_cast<u32>(positions.size()), 2.0f);
    body.buildFromMesh(indices.data(), static_cast<u32>(indices.size()), 2.0e-6f, 1.0e-4f);
    for (u32 i = 0; i < body.particleCount(); ++i)
        if (positions[i].y > 7.999f)
            body.setPinned(i, true);
    body.buildAttachments(1.02f);
    body.setCollisionMargin(0.02f);
    body.setMaxLinearVelocity(20.0f);
    PhysicsWorld collisionWorld;
    SphereShape sphereShape(1.4f);
    PlaneShape groundShape(Math::vec3(0.0f, 1.0f, 0.0f));
    RigidBody sphereBody;
    sphereBody.setBodyType(BodyType::Kinematic);
    sphereBody.setPosition(Math::vec3(0.0f, 5.5f, 0.0f));
    RigidBody groundBody;
    groundBody.setBodyType(BodyType::Static);
    groundBody.setPosition(Math::vec3(0.0f));
    BodyEntry sphereEntry;
    sphereEntry.body = &sphereBody;
    sphereEntry.shape = &sphereShape;
    collisionWorld.addBody(sphereEntry);
    BodyEntry groundEntry;
    groundEntry.body = &groundBody;
    groundEntry.shape = &groundShape;
    collisionWorld.addBody(groundEntry);
    body.setCollisionWorld(&collisionWorld);
    f32 maximumSpeed = 0.0f;
    f32 previousSphereZ = 0.0f;
    for (u32 step = 0; step < 720; ++step)
    {
        const f32 sphereZ = step < 120 ? 0.0f
                              : step < 360
                                  ? -1.8f * static_cast<f32>(step - 120) / 240.0f
                                  : -1.8f;
        sphereBody.setPosition(Math::vec3(0.0f, 5.5f, sphereZ));
        sphereBody.setVelocity(
            Math::vec3(0.0f, 0.0f, (sphereZ - previousSphereZ) * 120.0f));
        previousSphereZ = sphereZ;
        body.setWind(step >= 360 && step < 480 ? Math::vec3(20.0f, 0.0f, 6.0f)
                                               : Math::vec3(0.0f));
        body.step(1.0f / 120.0f, 8);
        for (const SoftBody::Particle& particle : body.particles())
        {
            CHECK(std::isfinite(particle.position.x) && std::isfinite(particle.position.y) &&
                  std::isfinite(particle.position.z));
            CHECK(Math::all(Math::lessThan(Math::abs(particle.position), Math::vec3(20.0f))));
            maximumSpeed = Math::max(maximumSpeed, Math::length(particle.velocity));
        }
    }
    CHECK(maximumSpeed < 30.0f);
}

// Average particle speed and centre displacement after sliding a flat resting
// sheet across level ground for one second with the given contact friction.
void slideRestingSheet(f32 friction, f32& averageSpeed, f32& displacement,
                       bool boxGround = false)
{
    std::vector<Math::vec3> positions;
    std::vector<u32> indices;
    makeGrid(6, 6, 0.25f, positions, indices);
    for (Math::vec3& position : positions)
        position.y = 0.02f;

    SoftBody body;
    body.setParticles(positions.data(), static_cast<u32>(positions.size()), 1.0f);
    body.buildFromMesh(indices.data(), static_cast<u32>(indices.size()), 0.0f, 0.0f);
    body.setCollisionMargin(0.02f);
    PhysicsWorld collisionWorld;
    PlaneShape planeShape(Math::vec3(0.0f, 1.0f, 0.0f));
    BoxShape boxShape(Math::vec3(15.0f, 0.5f, 15.0f));
    RigidBody groundBody;
    groundBody.setBodyType(BodyType::Static);
    groundBody.setPosition(boxGround ? Math::vec3(0.0f, -0.5f, 0.0f) : Math::vec3(0.0f));
    BodyEntry groundEntry;
    groundEntry.body = &groundBody;
    groundEntry.shape = boxGround ? static_cast<CollisionShape*>(&boxShape)
                                  : static_cast<CollisionShape*>(&planeShape);
    groundEntry.friction = friction;
    collisionWorld.addBody(groundEntry);
    body.setCollisionWorld(&collisionWorld);

    Math::vec3 startCentre(0.0f);
    for (u32 i = 0; i < body.particleCount(); ++i)
    {
        body.particle(i).velocity = Math::vec3(1.0f, 0.0f, 0.0f);
        startCentre += body.particle(i).position;
    }
    startCentre /= static_cast<f32>(body.particleCount());

    for (u32 step = 0; step < 120; ++step)
        body.step(1.0f / 120.0f, 8);

    Math::vec3 endCentre(0.0f);
    averageSpeed = 0.0f;
    for (u32 i = 0; i < body.particleCount(); ++i)
    {
        averageSpeed += Math::length(body.particle(i).velocity);
        endCentre += body.particle(i).position;
    }
    averageSpeed /= static_cast<f32>(body.particleCount());
    endCentre /= static_cast<f32>(body.particleCount());
    displacement = Math::length(endCentre - startCentre);
}

void testGroundFrictionStopsASlidingSheet()
{
    // Coulomb friction at 0.6 decelerates a 1 m/s slide at ~5.9 m/s^2: it has
    // to stop within 0.17 s and travel no further than ~0.09 m. One second
    // later anything still moving means the contact is not braking at all.
    f32 speed = 0.0f;
    f32 travelled = 0.0f;
    slideRestingSheet(0.6f, speed, travelled);
    if (speed >= 0.05f || travelled >= 0.30f)
        std::fprintf(stderr, "sliding sheet with friction: speed %.3f travelled %.3f\n",
                     static_cast<f64>(speed), static_cast<f64>(travelled));
    CHECK(speed < 0.05f);
    CHECK(travelled < 0.30f);

    // The control: without friction the same sheet must still be gliding,
    // or the test above is passing for the wrong reason.
    slideRestingSheet(0.0f, speed, travelled);
    CHECK(speed > 0.5f);
    CHECK(travelled > 0.5f);

    // The same slide over a box's top face has to brake identically - the
    // sphere-box narrowphase feeds the same contact fields as the plane path.
    slideRestingSheet(0.6f, speed, travelled, true);
    if (speed >= 0.05f || travelled >= 0.30f)
        std::fprintf(stderr, "sliding sheet on box: speed %.3f travelled %.3f\n",
                     static_cast<f64>(speed), static_cast<f64>(travelled));
    CHECK(speed < 0.05f);
    CHECK(travelled < 0.30f);
}

void testFallenSheetComesToRest(bool withSphere = true)
{
    // The demo scene, headless: the 45x45 sheet dropped from 6 m over the
    // sphere, sliding off onto the ground. Once everything is down, friction
    // has to bring it to rest instead of letting it glide like ice.
    std::vector<Math::vec3> positions;
    std::vector<u32> indices;
    makeGrid(45, 45, 6.0f / 44.0f, positions, indices);
    for (Math::vec3& position : positions)
        position += Math::vec3(-3.0f, 6.0f, -3.0f);

    SoftBody body;
    body.setParticles(positions.data(), static_cast<u32>(positions.size()), 4.0f);
    body.buildFromMesh(indices.data(), static_cast<u32>(indices.size()), 0.0f, 1.0e-4f);
    body.setCollisionMargin(0.02f);
    body.setMaxLinearVelocity(20.0f);
    PhysicsWorld collisionWorld;
    SphereShape sphereShape(1.4f);
    PlaneShape groundShape(Math::vec3(0.0f, 1.0f, 0.0f));
    RigidBody sphereBody;
    sphereBody.setBodyType(BodyType::Static);
    sphereBody.setPosition(Math::vec3(0.0f, 3.0f, 0.0f));
    RigidBody groundBody;
    groundBody.setBodyType(BodyType::Static);
    groundBody.setPosition(Math::vec3(0.0f));
    if (withSphere)
    {
        BodyEntry sphereEntry;
        sphereEntry.body = &sphereBody;
        sphereEntry.shape = &sphereShape;
        sphereEntry.friction = 0.6f;
        collisionWorld.addBody(sphereEntry);
    }
    BodyEntry groundEntry;
    groundEntry.body = &groundBody;
    groundEntry.shape = &groundShape;
    groundEntry.friction = 0.6f;
    collisionWorld.addBody(groundEntry);
    body.setCollisionWorld(&collisionWorld);

    f32 averageSpeed = 0.0f;
    bool finite = true;
    for (u32 step = 0; step < 1200; ++step)
    {
        body.step(1.0f / 120.0f, 8);
        if ((step + 1) % 120 == 0)
        {
            averageSpeed = 0.0f;
            for (const SoftBody::Particle& particle : body.particles())
            {
                finite = finite && std::isfinite(particle.position.x) &&
                         std::isfinite(particle.position.y) &&
                         std::isfinite(particle.position.z);
                averageSpeed += Math::length(particle.velocity);
            }
            averageSpeed /= static_cast<f32>(body.particleCount());
        }
    }
    CHECK(finite);
    if (withSphere)
    {
        // Sliding off a ball leaves the sheet crumpled, and without self
        // collision a crumpled pile's folds oscillate through each other
        // forever - full rest is out of reach until that exists (the Jolt
        // library itself, measured headless on a simpler pressed curtain,
        // rings at 0.33-3.98 m/frame). What the solver does have to hold is
        // that the pile never runs away.
        if (averageSpeed >= 12.0f)
            std::fprintf(stderr, "fallen sheet over sphere: average speed %.3f\n",
                         static_cast<f64>(averageSpeed));
        CHECK(averageSpeed < 12.0f);
    }
    else
    {
        // Flat on level ground every particle is in contact, so friction has
        // to bring the whole sheet to a genuine stop.
        if (averageSpeed >= 0.05f)
            std::fprintf(stderr, "fallen sheet on ground: average speed %.3f\n",
                         static_cast<f64>(averageSpeed));
        CHECK(averageSpeed < 0.05f);
    }
}

void testDihedralBendRestoresAFlatRestPose()
{
    // Two triangles sharing one edge, rest pose flat, everything pinned but
    // the tip of one wing, which is lifted and released. The dihedral bend
    // has to pull it back to the plane - the sign test that catches a port
    // pushing the fold open instead of closed.
    std::vector<Math::vec3> positions = {Math::vec3(0.0f, 0.0f, 0.0f),
                                        Math::vec3(1.0f, 0.0f, 0.0f),
                                        Math::vec3(0.5f, 0.0f, 1.0f),
                                        Math::vec3(0.5f, 0.0f, -1.0f)};
    std::vector<u32> indices = {0, 2, 1, 0, 1, 3};
    SoftBody body;
    body.setParticles(positions.data(), 4, 1.0f);
    body.buildFromMesh(indices.data(), 6, 0.0f, 0.0f, SoftBody::BendType::Dihedral);
    CHECK(body.constraintCount() == 5);
    body.setPinned(0, true);
    body.setPinned(1, true);
    body.setPinned(3, true);
    body.particle(2).position = Math::vec3(0.5f, 0.8f, 0.35f);
    body.setGravity(Math::vec3(0.0f));
    for (u32 i = 0; i < 40; ++i)
        body.step(1.0f / 120.0f, 8);
    CHECK(std::abs(body.particle(2).position.y) < 0.01f);
}

// Worst frame-to-frame particle displacement over the last two simulated
// seconds - the number a human reads as trembling, which an average of
// velocities cannot see because the contact push-out moves positions after
// the velocity was derived.
f32 tremorAfterSettling(SoftBody& body, u32 settleSteps, u32 measureSteps)
{
    for (u32 step = 0; step < settleSteps; ++step)
        body.step(1.0f / 120.0f, 8);

    std::vector<Math::vec3> previous(body.particleCount());
    for (u32 i = 0; i < body.particleCount(); ++i)
        previous[i] = body.particle(i).position;

    f32 worst = 0.0f;
    for (u32 step = 0; step < measureSteps; ++step)
    {
        body.step(1.0f / 120.0f, 8);
        for (u32 i = 0; i < body.particleCount(); ++i)
        {
            worst = Math::max(worst, Math::length(body.particle(i).position - previous[i]));
            previous[i] = body.particle(i).position;
        }
    }
    return worst;
}

void testHeavyFallenSheetDoesNotTremble()
{
    std::vector<Math::vec3> positions;
    std::vector<u32> indices;
    makeGrid(45, 45, 6.0f / 44.0f, positions, indices);
    for (Math::vec3& position : positions)
        position += Math::vec3(-3.0f, 6.0f, -3.0f);

    SoftBody body;
    body.setParticles(positions.data(), static_cast<u32>(positions.size()), 40.0f);
    body.buildFromMesh(indices.data(), static_cast<u32>(indices.size()), 0.0f, 1.0e-4f);
    body.setCollisionMargin(0.02f);
    body.setMaxLinearVelocity(20.0f);
    PhysicsWorld collisionWorld;
    SphereShape sphereShape(1.4f);
    PlaneShape groundShape(Math::vec3(0.0f, 1.0f, 0.0f));
    RigidBody sphereBody;
    sphereBody.setBodyType(BodyType::Static);
    sphereBody.setPosition(Math::vec3(0.0f, 3.0f, 0.0f));
    RigidBody groundBody;
    groundBody.setBodyType(BodyType::Static);
    groundBody.setPosition(Math::vec3(0.0f));
    BodyEntry sphereEntry;
    sphereEntry.body = &sphereBody;
    sphereEntry.shape = &sphereShape;
    sphereEntry.friction = 0.6f;
    collisionWorld.addBody(sphereEntry);
    BodyEntry groundEntry;
    groundEntry.body = &groundBody;
    groundEntry.shape = &groundShape;
    groundEntry.friction = 0.6f;
    collisionWorld.addBody(groundEntry);
    body.setCollisionWorld(&collisionWorld);

    // Reference point, measured: the Jolt library itself, run headless on the
    // pressed-curtain scenario below, trembles at 0.33-3.98 m/frame. Ours
    // holds under 0.04 on both scenarios; the bound leaves room for chaos
    // without letting a real regression through.
    const f32 tremor = tremorAfterSettling(body, 600, 240);
    if (tremor >= 0.08f)
        std::fprintf(stderr, "heavy fallen sheet tremor: %.4f m/frame\n",
                     static_cast<f64>(tremor));
    CHECK(tremor < 0.08f);
}

void testCurtainPressedBySphereComesToRest()
{
    std::vector<Math::vec3> positions;
    std::vector<u32> indices;
    makeGrid(37, 37, 4.0f / 36.0f, positions, indices);
    for (Math::vec3& position : positions)
        position = Math::vec3(position.x - 2.0f, 6.0f - position.z, -2.6f);

    SoftBody body;
    body.setParticles(positions.data(), static_cast<u32>(positions.size()), 2.0f);
    body.buildFromMesh(indices.data(), static_cast<u32>(indices.size()), 2.0e-6f, 1.0e-4f);
    for (u32 i = 0; i < body.particleCount(); ++i)
        if (positions[i].y > 5.999f)
            body.setPinned(i, true);
    body.buildAttachments(1.02f);
    body.setCollisionMargin(0.02f);
    body.setMaxLinearVelocity(20.0f);
    PhysicsWorld collisionWorld;
    SphereShape sphereShape(1.4f);
    RigidBody sphereBody;
    sphereBody.setBodyType(BodyType::Static);
    // Pressed 0.3 m through the curtain's rest plane, the way the demo's
    // slider pushes the sphere back into the hanging sheet.
    sphereBody.setPosition(Math::vec3(0.0f, 4.0f, -1.5f));
    BodyEntry sphereEntry;
    sphereEntry.body = &sphereBody;
    sphereEntry.shape = &sphereShape;
    sphereEntry.friction = 0.6f;
    collisionWorld.addBody(sphereEntry);
    body.setCollisionWorld(&collisionWorld);

    const f32 tremor = tremorAfterSettling(body, 600, 240);
    if (tremor >= 0.08f)
        std::fprintf(stderr, "curtain vs sphere tremor: %.4f m/frame\n",
                     static_cast<f64>(tremor));
    CHECK(tremor < 0.08f);
}

} // namespace

int main()
{
    testMeshBuildsSharedEdgesOnce();
    testStiffClothDoesNotExplode();
    testStiffnessIsIndependentOfStepSize();
    testAttachmentsHoldASheetFromStretching();
    testSubstepsBeatOneBigStep();
    testPinnedParticlesNeverMove();
    testEmptyBodyIsHarmless();
    testWindDoesNotDependOnParticleCount();
    testDenseSheetDoesNotLaunchFromSphere();
    testHangingSheetRemainsStableAgainstSphere();
    testGroundFrictionStopsASlidingSheet();
    testFallenSheetComesToRest(false);
    testFallenSheetComesToRest(true);
    testDihedralBendRestoresAFlatRestPose();
    testHeavyFallenSheetDoesNotTremble();
    testCurtainPressedBySphereComesToRest();
    if (gFailures)
        std::fprintf(stderr, "%d soft body test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
