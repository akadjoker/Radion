#include "PCH.h"

#include "AssetManager.h"
#include "Mesh.h"
#include "VolumeCSG.h"
#include "VolumeMeshSource.h"
#include "VolumeMesher.h"

#include <cstdio>
#include <vector>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "VolumeMeshSourceTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

MeshData buildBox(const Math::vec3& size)
{
    MeshData mesh;
    AssetManager::getSingleton().buildMeshData(MeshDesc::box(size), mesh);
    return mesh;
}

MeshData buildSphere(f32 radius, u32 rings, u32 slices)
{
    MeshData mesh;
    AssetManager::getSingleton().buildMeshData(MeshDesc::sphere(radius, rings, slices), mesh);
    return mesh;
}

void testBuildRejectsNothing()
{
    Volume::MeshSource source;
    MeshData empty;
    CHECK(!source.build(empty));
    CHECK(!source.valid());
    CHECK(source.sampleDensity(Math::vec3(0.0f)) < 0.0f);

    MeshData box = buildBox(Math::vec3(2.0f));
    CHECK(source.build(box));
    CHECK(source.valid());
    CHECK(source.triangleCount() == box.indices.size() / 3);
}

// Reference: BoxSource computes the same field in closed form; agreement across a grid proves distance and sign.
void testMatchesTheAnalyticBox()
{
    MeshData box = buildBox(Math::vec3(2.0f, 2.0f, 2.0f));
    Volume::MeshSource mesh;
    CHECK(mesh.build(box));

    // MeshDesc::box takes the full size, so the half extents are half of it.
    const Volume::BoxSource analytic(Math::vec3(0.0f), Math::vec3(1.0f));

    u32 signMismatches = 0;
    f32 worst = 0.0f;
    u32 compared = 0;

    for (s32 z = -6; z <= 6; ++z)
        for (s32 y = -6; y <= 6; ++y)
            for (s32 x = -6; x <= 6; ++x)
            {
                const Math::vec3 p(static_cast<f32>(x) * 0.3f, static_cast<f32>(y) * 0.3f,
                                  static_cast<f32>(z) * 0.3f);

                const f32 fromMesh = mesh.sampleDensity(p);
                const f32 fromAnalytic = analytic.sampleDensity(p);

                // Near the surface the sign legitimately differs; compare it only away from it.
                if (Math::abs(fromAnalytic) > 0.05f)
                {
                    if ((fromMesh > 0.0f) != (fromAnalytic > 0.0f))
                        ++signMismatches;
                    worst = Math::max(worst, Math::abs(fromMesh - fromAnalytic));
                    ++compared;
                }
            }

    CHECK(compared > 1000);
    CHECK(signMismatches == 0);
    CHECK(worst < 0.01f);
}

// Facets approximate a sphere (sagitta); comparing still catches an inverted sign or too large an error.
void testAgreesWithTheAnalyticSphere()
{
    MeshData sphere = buildSphere(1.0f, 48, 96);
    Volume::MeshSource mesh;
    CHECK(mesh.build(sphere));

    const Volume::SphereSource analytic(Math::vec3(0.0f), 1.0f);

    u32 signMismatches = 0;
    f32 worst = 0.0f;
    for (s32 z = -4; z <= 4; ++z)
        for (s32 y = -4; y <= 4; ++y)
            for (s32 x = -4; x <= 4; ++x)
            {
                const Math::vec3 p(static_cast<f32>(x) * 0.4f, static_cast<f32>(y) * 0.4f,
                                  static_cast<f32>(z) * 0.4f);
                const f32 fromMesh = mesh.sampleDensity(p);
                const f32 fromAnalytic = analytic.sampleDensity(p);
                if (Math::abs(fromAnalytic) > 0.1f)
                {
                    if ((fromMesh > 0.0f) != (fromAnalytic > 0.0f))
                        ++signMismatches;
                    worst = Math::max(worst, Math::abs(fromMesh - fromAnalytic));
                }
            }

    CHECK(signMismatches == 0);
    CHECK(worst < 0.05f);
}

// Positive inside is the convention of every Source; backwards turns Difference into Intersection.
void testSignConvention()
{
    MeshData box = buildBox(Math::vec3(2.0f));
    Volume::MeshSource mesh;
    CHECK(mesh.build(box));

    CHECK(mesh.sampleDensity(Math::vec3(0.0f)) > 0.0f);
    CHECK(Math::abs(mesh.sampleDensity(Math::vec3(0.0f)) - 1.0f) < 0.01f);

    CHECK(mesh.sampleDensity(Math::vec3(5.0f, 0.0f, 0.0f)) < 0.0f);
    CHECK(Math::abs(mesh.sampleDensity(Math::vec3(5.0f, 0.0f, 0.0f)) + 4.0f) < 0.01f);

    CHECK(mesh.sampleDensity(Math::vec3(0.98f, 0.0f, 0.0f)) > 0.0f);
    CHECK(mesh.sampleDensity(Math::vec3(1.02f, 0.0f, 0.0f)) < 0.0f);

    const f32 corner = mesh.sampleDensity(Math::vec3(2.0f, 2.0f, 2.0f));
    CHECK(corner < 0.0f);
    CHECK(Math::abs(Math::abs(corner) - Math::length(Math::vec3(1.0f))) < 0.01f);
}

// Off-origin mesh: an implementation assuming it is centred fails here.
void testOffCentreMesh()
{
    MeshData box = buildBox(Math::vec3(2.0f));
    AssetManager::getSingleton().translate(box, Math::vec3(10.0f, -5.0f, 3.0f));

    Volume::MeshSource mesh;
    CHECK(mesh.build(box));

    CHECK(mesh.sampleDensity(Math::vec3(10.0f, -5.0f, 3.0f)) > 0.0f);
    CHECK(mesh.sampleDensity(Math::vec3(0.0f)) < 0.0f);
    CHECK(Math::abs(mesh.sampleDensity(Math::vec3(10.0f, -5.0f, 3.0f)) - 1.0f) < 0.01f);
}

void testDifferenceAgainstAMesh()
{
    MeshData box = buildBox(Math::vec3(2.0f));
    Volume::MeshSource solid;
    CHECK(solid.build(box));

    const Volume::SphereSource drill(Math::vec3(0.0f), 0.5f);
    const Volume::DifferenceSource drilled(solid, drill);

    CHECK(drilled.sampleDensity(Math::vec3(0.0f)) < 0.0f);
    CHECK(drilled.sampleDensity(Math::vec3(0.8f, 0.0f, 0.0f)) > 0.0f);
    CHECK(drilled.sampleDensity(Math::vec3(3.0f, 0.0f, 0.0f)) < 0.0f);

    const Volume::IntersectionSource common(solid, drill);
    CHECK(common.sampleDensity(Math::vec3(0.0f)) > 0.0f);
    CHECK(common.sampleDensity(Math::vec3(0.8f, 0.0f, 0.0f)) < 0.0f);

    // Named: BinarySource keeps references to its operands.
    const Volume::SphereSource beside(Math::vec3(2.0f, 0.0f, 0.0f), 0.5f);
    const Volume::UnionSource both(solid, beside);
    CHECK(both.sampleDensity(Math::vec3(0.0f)) > 0.0f);
    CHECK(both.sampleDensity(Math::vec3(2.0f, 0.0f, 0.0f)) > 0.0f);
}

void testMeshesBackOut()
{
    MeshData box = buildBox(Math::vec3(2.0f));
    Volume::MeshSource source;
    CHECK(source.build(box));

    Volume::MeshingSettings settings;
    settings.bounds = source.bounds();
    settings.bounds.min -= Math::vec3(0.5f);
    settings.bounds.max += Math::vec3(0.5f);
    settings.voxelSize = 0.2f;

    MeshData out;
    Volume::MeshingStats stats;
    CHECK(Volume::buildMesh(source, settings, out, &stats));

    CHECK(out.positions.size() > 0);
    CHECK(out.indices.size() >= 3);
    CHECK(stats.triangles > 0);

    const Math::vec3 size = out.bounds.max - out.bounds.min;
    CHECK(Math::abs(size.x - 2.0f) < 0.4f);
    CHECK(Math::abs(size.y - 2.0f) < 0.4f);
    CHECK(Math::abs(size.z - 2.0f) < 0.4f);
}

} // namespace

int main()
{
    testBuildRejectsNothing();
    testMatchesTheAnalyticBox();
    testAgreesWithTheAnalyticSphere();
    testSignConvention();
    testOffCentreMesh();
    testDifferenceAgainstAMesh();
    testMeshesBackOut();

    if (gFailures)
        std::fprintf(stderr, "%d volume mesh source test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
