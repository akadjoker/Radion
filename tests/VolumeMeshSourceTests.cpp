// VolumeMeshSourceTests.cpp - a loaded mesh as a density field, which is what
// lets the CSG in VolumeCSG.h combine modelled geometry instead of only
// spheres and boxes. The magnitudes are checked against the analytic sources
// already in the engine rather than against numbers written out by hand.
// Nothing here touches the GPU.

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

MeshData buildBox(const glm::vec3& size)
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
    // A source with nothing in it still has to answer, and answer "outside".
    CHECK(source.sampleDensity(glm::vec3(0.0f)) < 0.0f);

    MeshData box = buildBox(glm::vec3(2.0f));
    CHECK(source.build(box));
    CHECK(source.valid());
    CHECK(source.triangleCount() == box.indices.size() / 3);
}

// The whole thing measured against BoxSource, which computes the same field
// in closed form. If the mesh version agrees with it across a grid then both
// the distance and the sign are right, and neither number came from me.
void testMatchesTheAnalyticBox()
{
    MeshData box = buildBox(glm::vec3(2.0f, 2.0f, 2.0f));
    Volume::MeshSource mesh;
    CHECK(mesh.build(box));

    // MeshDesc::box takes the full size, so the half extents are half of it.
    const Volume::BoxSource analytic(glm::vec3(0.0f), glm::vec3(1.0f));

    u32 signMismatches = 0;
    f32 worst = 0.0f;
    u32 compared = 0;

    for (s32 z = -6; z <= 6; ++z)
        for (s32 y = -6; y <= 6; ++y)
            for (s32 x = -6; x <= 6; ++x)
            {
                const glm::vec3 p(static_cast<f32>(x) * 0.3f, static_cast<f32>(y) * 0.3f,
                                  static_cast<f32>(z) * 0.3f);

                const f32 fromMesh = mesh.sampleDensity(p);
                const f32 fromAnalytic = analytic.sampleDensity(p);

                // Points sitting all but exactly on the surface are where the
                // two disagree for reasons that are not bugs, so the sign is
                // only compared away from it.
                if (glm::abs(fromAnalytic) > 0.05f)
                {
                    if ((fromMesh > 0.0f) != (fromAnalytic > 0.0f))
                        ++signMismatches;
                    worst = glm::max(worst, glm::abs(fromMesh - fromAnalytic));
                    ++compared;
                }
            }

    CHECK(compared > 1000);
    CHECK(signMismatches == 0);
    // The box's own distance field is exact, and so is the distance to its
    // triangles: they should agree to well under a voxel.
    CHECK(worst < 0.01f);
}

// A sphere's triangles only approximate it, so the two fields differ by the
// sagitta of the facets. Comparing anyway catches a sign that is inverted or
// a distance that is wrong by more than tessellation explains.
void testAgreesWithTheAnalyticSphere()
{
    MeshData sphere = buildSphere(1.0f, 48, 96);
    Volume::MeshSource mesh;
    CHECK(mesh.build(sphere));

    const Volume::SphereSource analytic(glm::vec3(0.0f), 1.0f);

    u32 signMismatches = 0;
    f32 worst = 0.0f;
    for (s32 z = -4; z <= 4; ++z)
        for (s32 y = -4; y <= 4; ++y)
            for (s32 x = -4; x <= 4; ++x)
            {
                const glm::vec3 p(static_cast<f32>(x) * 0.4f, static_cast<f32>(y) * 0.4f,
                                  static_cast<f32>(z) * 0.4f);
                const f32 fromMesh = mesh.sampleDensity(p);
                const f32 fromAnalytic = analytic.sampleDensity(p);
                if (glm::abs(fromAnalytic) > 0.1f)
                {
                    if ((fromMesh > 0.0f) != (fromAnalytic > 0.0f))
                        ++signMismatches;
                    worst = glm::max(worst, glm::abs(fromMesh - fromAnalytic));
                }
            }

    CHECK(signMismatches == 0);
    CHECK(worst < 0.05f);
}

// Positive inside is the convention every other Source follows; getting it
// backwards would turn every Difference into an Intersection.
void testSignConvention()
{
    MeshData box = buildBox(glm::vec3(2.0f));
    Volume::MeshSource mesh;
    CHECK(mesh.build(box));

    CHECK(mesh.sampleDensity(glm::vec3(0.0f)) > 0.0f);
    CHECK(glm::abs(mesh.sampleDensity(glm::vec3(0.0f)) - 1.0f) < 0.01f);

    CHECK(mesh.sampleDensity(glm::vec3(5.0f, 0.0f, 0.0f)) < 0.0f);
    CHECK(glm::abs(mesh.sampleDensity(glm::vec3(5.0f, 0.0f, 0.0f)) + 4.0f) < 0.01f);

    // Just inside and just outside one face.
    CHECK(mesh.sampleDensity(glm::vec3(0.98f, 0.0f, 0.0f)) > 0.0f);
    CHECK(mesh.sampleDensity(glm::vec3(1.02f, 0.0f, 0.0f)) < 0.0f);

    // Off a corner, where the nearest feature is a vertex rather than a face.
    const f32 corner = mesh.sampleDensity(glm::vec3(2.0f, 2.0f, 2.0f));
    CHECK(corner < 0.0f);
    CHECK(glm::abs(glm::abs(corner) - glm::length(glm::vec3(1.0f))) < 0.01f);
}

// A mesh that is not centred on the origin: an implementation that quietly
// assumes it is passes everything above and fails here.
void testOffCentreMesh()
{
    MeshData box = buildBox(glm::vec3(2.0f));
    AssetManager::getSingleton().translate(box, glm::vec3(10.0f, -5.0f, 3.0f));

    Volume::MeshSource mesh;
    CHECK(mesh.build(box));

    CHECK(mesh.sampleDensity(glm::vec3(10.0f, -5.0f, 3.0f)) > 0.0f);
    CHECK(mesh.sampleDensity(glm::vec3(0.0f)) < 0.0f);
    CHECK(glm::abs(mesh.sampleDensity(glm::vec3(10.0f, -5.0f, 3.0f)) - 1.0f) < 0.01f);
}

// What this was built for: the combinators that already existed, driven by a
// mesh on one side. A hole through a solid is the case that fails if the sign
// is wrong anywhere.
void testDifferenceAgainstAMesh()
{
    MeshData box = buildBox(glm::vec3(2.0f));
    Volume::MeshSource solid;
    CHECK(solid.build(box));

    const Volume::SphereSource drill(glm::vec3(0.0f), 0.5f);
    const Volume::DifferenceSource drilled(solid, drill);

    // The middle is gone, the shell around it is not.
    CHECK(drilled.sampleDensity(glm::vec3(0.0f)) < 0.0f);
    CHECK(drilled.sampleDensity(glm::vec3(0.8f, 0.0f, 0.0f)) > 0.0f);
    // Still outside the box entirely.
    CHECK(drilled.sampleDensity(glm::vec3(3.0f, 0.0f, 0.0f)) < 0.0f);

    const Volume::IntersectionSource common(solid, drill);
    CHECK(common.sampleDensity(glm::vec3(0.0f)) > 0.0f);
    CHECK(common.sampleDensity(glm::vec3(0.8f, 0.0f, 0.0f)) < 0.0f);

    // Named, not a temporary: BinarySource keeps references to its operands,
    // so one built in the argument list is dead before it is ever sampled.
    const Volume::SphereSource beside(glm::vec3(2.0f, 0.0f, 0.0f), 0.5f);
    const Volume::UnionSource both(solid, beside);
    CHECK(both.sampleDensity(glm::vec3(0.0f)) > 0.0f);
    CHECK(both.sampleDensity(glm::vec3(2.0f, 0.0f, 0.0f)) > 0.0f);
}

// End to end: mesh in, density field, marching cubes, mesh out. The result
// only has to be a closed solid of roughly the right size - the voxel grid
// decides the rest.
void testMeshesBackOut()
{
    MeshData box = buildBox(glm::vec3(2.0f));
    Volume::MeshSource source;
    CHECK(source.build(box));

    Volume::MeshingSettings settings;
    settings.bounds = source.bounds();
    settings.bounds.min -= glm::vec3(0.5f);
    settings.bounds.max += glm::vec3(0.5f);
    settings.voxelSize = 0.2f;

    MeshData out;
    Volume::MeshingStats stats;
    CHECK(Volume::buildMesh(source, settings, out, &stats));

    CHECK(out.positions.size() > 0);
    CHECK(out.indices.size() >= 3);
    CHECK(stats.triangles > 0);

    // Within a voxel of the box it came from on every axis.
    const glm::vec3 size = out.bounds.max - out.bounds.min;
    CHECK(glm::abs(size.x - 2.0f) < 0.4f);
    CHECK(glm::abs(size.y - 2.0f) < 0.4f);
    CHECK(glm::abs(size.z - 2.0f) < 0.4f);
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
