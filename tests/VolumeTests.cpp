#include "PCH.h"

#include "VolumeCSG.h"
#include "VolumeGrid.h"
#include "VolumeMesher.h"

#include <cstdio>

using namespace Radion;
using namespace Radion::Volume;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "VolumeTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool near(f32 a, f32 b, f32 epsilon = 0.0001f) { return std::abs(a - b) <= epsilon; }
bool near(const glm::vec3& a, const glm::vec3& b, f32 epsilon = 0.0001f)
{
    return glm::length(a - b) <= epsilon;
}

void testPrimitives()
{
    SphereSource sphere(glm::vec3(1.0f, 2.0f, 3.0f), 2.0f);
    CHECK(near(sphere.sampleDensity(glm::vec3(1.0f, 2.0f, 3.0f)), 2.0f));
    CHECK(near(sphere.sampleDensity(glm::vec3(3.0f, 2.0f, 3.0f)), 0.0f));
    CHECK(near(sphere.sample(glm::vec3(3.0f, 2.0f, 3.0f)).gradient, glm::vec3(1, 0, 0)));

    PlaneSource plane(glm::vec3(0, 2, 0), -2.0f);
    CHECK(near(plane.sampleDensity(glm::vec3(0, 3, 0)), 1.0f));
    CHECK(near(plane.sample(glm::vec3(0)).gradient, glm::vec3(0, 1, 0)));

    BoxSource box(glm::vec3(0), glm::vec3(1));
    CHECK(box.sampleDensity(glm::vec3(0)) > 0.0f);
    CHECK(box.sampleDensity(glm::vec3(2, 0, 0)) < 0.0f);
}

void testOperationsAndNoise()
{
    const SphereSource a(glm::vec3(-0.5f, 0, 0), 1.0f);
    const SphereSource b(glm::vec3(0.5f, 0, 0), 1.0f);
    UnionSource united(a, b);
    IntersectionSource overlap(a, b);
    DifferenceSource difference(a, b);
    NegateSource negated(a);
    ScaleSource scaled(a, 2.0f);
    CHECK(united.sampleDensity(glm::vec3(-1.4f, 0, 0)) > 0.0f);
    CHECK(overlap.sampleDensity(glm::vec3(0, 0, 0)) > 0.0f);
    CHECK(difference.sampleDensity(glm::vec3(0.5f, 0, 0)) < 0.0f);
    CHECK(near(negated.sampleDensity(glm::vec3(-0.5f, 0, 0)), -1.0f));
    CHECK(near(scaled.sampleDensity(glm::vec3(-1.0f, 0, 0)), 2.0f));

    NoiseSource first(42, 0.25f), second(42, 0.25f), other(43, 0.25f);
    const glm::vec3 point(1.2f, -0.7f, 3.4f);
    CHECK(near(first.sampleDensity(point), second.sampleDensity(point)));
    CHECK(!near(first.sampleDensity(point), other.sampleDensity(point), 1e-6f));
}

void testMesher()
{
    SphereSource sphere(glm::vec3(0), 1.0f);
    MeshingSettings settings;
    settings.bounds.min = glm::vec3(-1.5f);
    settings.bounds.max = glm::vec3(1.5f);
    settings.voxelSize = 0.25f;
    MeshData mesh;
    mesh.positions.push_back(glm::vec3(99));
    MeshingStats stats;
    CHECK(buildMesh(sphere, settings, mesh, &stats));
    CHECK(!mesh.positions.empty());
    CHECK(mesh.indices.size() % 3 == 0);
    CHECK(mesh.submeshes.size() == 1);
    CHECK(stats.samples > 0 && stats.triangles > 0);
    CHECK(mesh.bounds.min.x >= -1.51f && mesh.bounds.max.x <= 1.51f);
    for (u32 index : mesh.indices) CHECK(index < mesh.positions.size());

    MeshData preserved;
    preserved.positions.push_back(glm::vec3(7));
    MeshingSettings invalid = settings;
    invalid.voxelSize = 0.0f;
    CHECK(!buildMesh(sphere, invalid, preserved));
    CHECK(preserved.positions.size() == 1 && near(preserved.positions[0], glm::vec3(7)));
}

void testCaveGeneration()
{
    SphereSource terrain(glm::vec3(0.0f), 4.0f);
    SphereSource cave(glm::vec3(1.25f, 0.0f, 0.0f), 1.5f);
    DifferenceSource terrainWithCave(terrain, cave);

    MeshingSettings settings;
    settings.bounds.min = glm::vec3(-4.5f);
    settings.bounds.max = glm::vec3(4.5f);
    settings.voxelSize = 0.5f;

    MeshData mesh;
    MeshingStats stats;
    CHECK(buildMesh(terrainWithCave, settings, mesh, &stats));
    CHECK(!mesh.positions.empty());
    CHECK(stats.triangles > 0);
    CHECK(mesh.indices.size() == static_cast<usize>(stats.triangles) * 3);
    for (u32 index : mesh.indices) CHECK(index < mesh.positions.size());

    // The subtraction must be empty at the cave centre while the terrain
    // remains solid at a point away from the carved region.
    CHECK(terrainWithCave.sampleDensity(glm::vec3(1.25f, 0.0f, 0.0f)) < 0.0f);
    CHECK(terrainWithCave.sampleDensity(glm::vec3(-2.5f, 0.0f, 0.0f)) > 0.0f);
}

void testGrid()
{
    GridSource grid(glm::uvec3(3, 3, 3), glm::vec3(0), 1.0f, -1.0f);
    CHECK(grid.valid());
    CHECK(near(grid.bounds().min, glm::vec3(0)));
    CHECK(near(grid.bounds().max, glm::vec3(2)));
    CHECK(near(grid.voxelPosition(2, 1, 0), glm::vec3(2, 1, 0)));
    glm::uvec3 voxel;
    CHECK(grid.worldToVoxel(glm::vec3(1.9f, 0.1f, 2.0f), voxel));
    CHECK(voxel == glm::uvec3(1, 0, 2));
    CHECK(!grid.worldToVoxel(glm::vec3(3, 0, 0), voxel));
    MeshData gridMesh;
    MeshingStats gridStats;
    CHECK(grid.buildMesh(gridMesh, &gridStats));
    CHECK(gridStats.samples > 0);
    CHECK(grid.setVoxel(1, 1, 1, 1.0f));
    CHECK(near(grid.voxel(1, 1, 1), 1.0f));
    CHECK(!grid.setVoxel(3, 0, 0, 1.0f));
    CHECK(!grid.setVoxel(0, 0, 0, std::numeric_limits<f32>::quiet_NaN()));
    CHECK(std::isinf(grid.voxel(3, 0, 0)));
    CHECK(near(grid.sampleDensity(glm::vec3(1)), 1.0f));
    CHECK(grid.sampleDensity(glm::vec3(0)) < 0.0f);

    PlaneSource ramp(glm::vec3(1, 0, 0), -1.0f);
    grid.fill(ramp);
    CHECK(near(grid.voxel(0, 1, 1), -1.0f));
    CHECK(near(grid.voxel(2, 1, 1), 1.0f));
    CHECK(grid.sample(glm::vec3(1, 1, 1)).gradient.x > 0.9f);

    GridSource brushTarget(glm::uvec3(4, 4, 4), glm::vec3(0), 1.0f, -1.0f);
    SphereSource brush(glm::vec3(1), 1.1f);
    AABB affected;
    affected.min = glm::vec3(0);
    affected.max = glm::vec3(2);
    const AABB changed = brushTarget.apply(VolumeOperation::Union, brush, affected);
    CHECK(!changed.empty());
    CHECK(changed.min.x >= 0.0f && changed.max.x <= 2.0f);
    CHECK(brushTarget.voxel(1, 1, 1) > 0.0f);
    CHECK(brushTarget.voxel(3, 3, 3) < 0.0f);

    const AABB noChange = brushTarget.apply(VolumeOperation::Difference, brush, AABB{});
    CHECK(noChange.empty());

    ByteArray encoded;
    CHECK(brushTarget.save(encoded));
    CHECK(encoded.size() > 0);
    encoded.seek(0);
    GridSource restored(glm::uvec3(1), glm::vec3(99), 2.0f, 0.0f);
    CHECK(GridSource::load(encoded, restored));
    CHECK(restored.dimensions() == brushTarget.dimensions());
    CHECK(near(restored.origin(), brushTarget.origin()));
    CHECK(near(restored.cellSize(), brushTarget.cellSize()));
    CHECK(near(restored.voxel(1, 1, 1), brushTarget.voxel(1, 1, 1)));

    ByteArray truncated;
    CHECK(truncated.writeString("RVOL"));
    truncated.seek(0);
    GridSource unchanged(glm::uvec3(1), glm::vec3(4), 3.0f, 7.0f);
    CHECK(!GridSource::load(truncated, unchanged));
    CHECK(near(unchanged.voxel(0, 0, 0), 7.0f));
}
void testBoxDistance()
{
    const BoxSource box(glm::vec3(0.0f), glm::vec3(1.0f));

    CHECK(std::abs(box.sampleDensity(glm::vec3(0.0f)) - 1.0f) < 1e-4f);
    CHECK(std::abs(box.sampleDensity(glm::vec3(1.0f, 0.0f, 0.0f))) < 1e-4f);
    CHECK(std::abs(box.sampleDensity(glm::vec3(0.5f, 0.0f, 0.0f)) - 0.5f) < 1e-4f);

    CHECK(std::abs(box.sampleDensity(glm::vec3(3.0f, 0.0f, 0.0f)) + 2.0f) < 1e-4f);
    CHECK(std::abs(box.sampleDensity(glm::vec3(0.0f, -4.0f, 0.0f)) + 3.0f) < 1e-4f);

    const f32 corner = box.sampleDensity(glm::vec3(2.0f, 2.0f, 1.0f));
    CHECK(std::abs(corner + std::sqrt(2.0f)) < 1e-4f);
}

void testWeldedAndClosed()
{
    const SphereSource sphere(glm::vec3(0.0f), 2.0f);
    MeshingSettings settings;
    settings.bounds.min = glm::vec3(-3.0f);
    settings.bounds.max = glm::vec3(3.0f);
    settings.voxelSize = 0.25f;

    MeshData mesh;
    CHECK(buildMesh(sphere, settings, mesh));
    CHECK(!mesh.indices.empty());

    CHECK(mesh.positions.size() * 3 < mesh.indices.size() * 2);

    HashMap<u64, u32> edges;
    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
        for (u32 e = 0; e < 3; ++e)
        {
            const u32 a = mesh.indices[i + e];
            const u32 b = mesh.indices[i + (e + 1) % 3];
            const u64 key = (static_cast<u64>(std::min(a, b)) << 32) | std::max(a, b);
            ++edges[key];
        }
    u32 unshared = 0;
    u32 boundary = 0;
    u32 excess = 0;
    for (const auto& entry : edges)
    {
        if (entry.second == 2) continue;
        ++unshared;
        if (entry.second == 1) ++boundary; else ++excess;
    }
    if (unshared)
        std::fprintf(stderr, "  %u of %zu edges not shared by exactly 2 triangles (%u used once, %u used 3+)\n",
                     unshared, edges.size(), boundary, excess);
    CHECK(unshared == 0);
}

} // namespace

int main()
{
    testBoxDistance();
    testWeldedAndClosed();
    testPrimitives();
    testOperationsAndNoise();
    testMesher();
    testCaveGeneration();
    testGrid();
    if (gFailures) std::fprintf(stderr, "%d volume test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
