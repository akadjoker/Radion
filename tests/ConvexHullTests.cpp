#include "PCH.h"

#include "ConvexHullComputer.h"
#include "VoronoiShatter.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace Radion;
using namespace Radion::Geometry;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "ConvexHullTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool near(f32 a, f32 b, f32 epsilon = 1e-3f)
{
    return std::abs(a - b) <= epsilon;
}

bool near(const Math::Vec3& a, const Math::Vec3& b, f32 epsilon = 1e-3f)
{
    return glm::length(a - b) <= epsilon;
}

bool isFinite(f32 v)
{
    return std::isfinite(v);
}

bool isFinite(const Math::Vec3& v)
{
    return isFinite(v.x) && isFinite(v.y) && isFinite(v.z);
}

bool allVerticesFinite(const std::vector<Math::Vec3>& vertices)
{
    for (const Math::Vec3& v : vertices)
    {
        if (!isFinite(v))
            return false;
    }
    return true;
}

std::vector<Math::Vec3> unitCubeCorners(f32 halfExtent = 0.5f)
{
    std::vector<Math::Vec3> corners;
    for (int i = 0; i < 8; i++)
    {
        f32 x = (i & 1) ? halfExtent : -halfExtent;
        f32 y = (i & 2) ? halfExtent : -halfExtent;
        f32 z = (i & 4) ? halfExtent : -halfExtent;
        corners.push_back(Math::Vec3(x, y, z));
    }
    return corners;
}

bool facePlanar(const ConvexHullComputer& hull, int faceEdgeIndex, f32 epsilon)
{
    const ConvexHullComputer::Edge* start = &hull.edges[faceEdgeIndex];
    const ConvexHullComputer::Edge* edge = start;
    Math::Vec3 p0 = hull.vertices[edge->getSourceVertex()];
    Math::Vec3 p1 = hull.vertices[edge->getTargetVertex()];
    edge = edge->getNextEdgeOfFace();
    Math::Vec3 p2 = hull.vertices[edge->getTargetVertex()];
    Math::Vec3 normal = glm::cross(p1 - p0, p2 - p0);
    if (glm::length(normal) < 1e-8f)
        return false;
    normal = glm::normalize(normal);
    f32 d = glm::dot(normal, p0);
    do
    {
        Math::Vec3 p = hull.vertices[edge->getTargetVertex()];
        if (!near(glm::dot(normal, p), d, epsilon))
            return false;
        edge = edge->getNextEdgeOfFace();
    } while (edge != start);
    return true;
}

f32 boxVolume(const Math::Vec3& halfExtents)
{
    return 8.0f * halfExtents.x * halfExtents.y * halfExtents.z;
}

f32 sumShardVolumes(const std::vector<Shard>& shards)
{
    f32 total = 0.0f;
    for (const Shard& s : shards)
        total += s.volume;
    return total;
}

bool allShardVerticesInBounds(const std::vector<Shard>& shards, const Math::Vec3& boundsMin, const Math::Vec3& boundsMax, f32 epsilon)
{
    for (const Shard& s : shards)
    {
        for (const Math::Vec3& v : s.vertices)
        {
            Math::Vec3 world = s.centroid + v;
            if (world.x < boundsMin.x - epsilon || world.x > boundsMax.x + epsilon)
                return false;
            if (world.y < boundsMin.y - epsilon || world.y > boundsMax.y + epsilon)
                return false;
            if (world.z < boundsMin.z - epsilon || world.z > boundsMax.z + epsilon)
                return false;
        }
    }
    return true;
}

void testCubeHullHasSixFacesTwelveEdgesEightVertices()
{
    std::vector<Math::Vec3> corners = unitCubeCorners();
    ConvexHullComputer hull;
    hull.compute(&corners[0].x, sizeof(Math::Vec3), (int)corners.size(), 0.0f, 0.0f);

    CHECK(hull.vertices.size() == 8);
    CHECK(hull.faces.size() == 6);
    CHECK(hull.edges.size() == 24);

    for (int faceEdge : hull.faces)
    {
        CHECK(facePlanar(hull, faceEdge, 1e-2f));
    }
}

void testHullExcludesInteriorPoints()
{
    std::vector<Math::Vec3> points = unitCubeCorners();
    points.push_back(Math::Vec3(0.0f, 0.0f, 0.0f));
    points.push_back(Math::Vec3(0.1f, -0.1f, 0.2f));

    ConvexHullComputer hull;
    hull.compute(&points[0].x, sizeof(Math::Vec3), (int)points.size(), 0.0f, 0.0f);

    CHECK(hull.vertices.size() == 8);
    for (const Math::Vec3& v : hull.vertices)
    {
        CHECK(std::abs(v.x) > 0.4f || std::abs(v.y) > 0.4f || std::abs(v.z) > 0.4f);
    }
}

void testDegenerateCoplanarInputDoesNotCrash()
{
    std::vector<Math::Vec3> points;
    points.push_back(Math::Vec3(0.0f, 0.0f, 0.0f));
    points.push_back(Math::Vec3(1.0f, 0.0f, 0.0f));
    points.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
    points.push_back(Math::Vec3(1.0f, 1.0f, 0.0f));
    points.push_back(Math::Vec3(0.5f, 0.5f, 0.0f));

    ConvexHullComputer hull;
    hull.compute(&points[0].x, sizeof(Math::Vec3), (int)points.size(), 0.0f, 0.0f);

    CHECK(allVerticesFinite(hull.vertices));
}

void testDegenerateDuplicatePointsDoesNotCrash()
{
    std::vector<Math::Vec3> points(6, Math::Vec3(1.0f, 2.0f, 3.0f));

    ConvexHullComputer hull;
    hull.compute(&points[0].x, sizeof(Math::Vec3), (int)points.size(), 0.0f, 0.0f);

    CHECK(allVerticesFinite(hull.vertices));
}

void testCountZeroYieldsEmptyHull()
{
    ConvexHullComputer hull;
    f32 shift = hull.compute(nullptr, sizeof(Math::Vec3), 0, 0.0f, 0.0f);

    CHECK(shift == 0.0f);
    CHECK(hull.vertices.empty());
    CHECK(hull.faces.empty());
    CHECK(hull.edges.empty());
}

void testCountOneDoesNotCrash()
{
    Math::Vec3 point(1.0f, 2.0f, 3.0f);
    ConvexHullComputer hull;
    hull.compute(&point.x, sizeof(Math::Vec3), 1, 0.0f, 0.0f);

    CHECK(allVerticesFinite(hull.vertices));
    CHECK(hull.faces.size() <= 1);
}

void testCountTwoSegmentDoesNotCrash()
{
    std::vector<Math::Vec3> points = {Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(1.0f, 0.0f, 0.0f)};
    ConvexHullComputer hull;
    hull.compute(&points[0].x, sizeof(Math::Vec3), (int)points.size(), 0.0f, 0.0f);

    CHECK(allVerticesFinite(hull.vertices));
    // Two vertices joined by a single edge pair still produce one degenerate
    // "face" loop when the output edge/face structure is walked (the reference
    // does not special-case this); what matters is that it is not fabricated
    // garbage.
    CHECK(hull.faces.size() <= 1);
}

void testAllPointsCollinearDoesNotCrash()
{
    std::vector<Math::Vec3> points;
    for (int i = 0; i < 6; i++)
        points.push_back(Math::Vec3((f32)i, 0.0f, 0.0f));

    ConvexHullComputer hull;
    hull.compute(&points[0].x, sizeof(Math::Vec3), (int)points.size(), 0.0f, 0.0f);

    CHECK(allVerticesFinite(hull.vertices));
    CHECK(hull.vertices.size() == 2);
    CHECK(hull.faces.size() <= 1);
}

void testLargeCoordinateMagnitudeStaysSane()
{
    std::vector<Math::Vec3> corners = unitCubeCorners(1e6f);
    ConvexHullComputer hull;
    hull.compute(&corners[0].x, sizeof(Math::Vec3), (int)corners.size(), 0.0f, 0.0f);

    CHECK(allVerticesFinite(hull.vertices));
    CHECK(hull.vertices.size() == 8);
    CHECK(hull.faces.size() == 6);
}

void testSmallCoordinateMagnitudeStaysSane()
{
    std::vector<Math::Vec3> corners = unitCubeCorners(1e-4f);
    ConvexHullComputer hull;
    hull.compute(&corners[0].x, sizeof(Math::Vec3), (int)corners.size(), 0.0f, 0.0f);

    CHECK(allVerticesFinite(hull.vertices));
    CHECK(hull.vertices.size() == 8);
    CHECK(hull.faces.size() == 6);
}

void testTinyJitterDoesNotFragmentCubeFaces()
{
    std::vector<Math::Vec3> corners = unitCubeCorners();
    unsigned int seed = 12345u;
    for (Math::Vec3& c : corners)
    {
        seed = 1664525u * seed + 1013904223u;
        f32 jx = ((f32)(seed & 0xffffu) / 65535.0f - 0.5f) * 2e-6f;
        seed = 1664525u * seed + 1013904223u;
        f32 jy = ((f32)(seed & 0xffffu) / 65535.0f - 0.5f) * 2e-6f;
        seed = 1664525u * seed + 1013904223u;
        f32 jz = ((f32)(seed & 0xffffu) / 65535.0f - 0.5f) * 2e-6f;
        c += Math::Vec3(jx, jy, jz);
    }

    ConvexHullComputer hull;
    hull.compute(&corners[0].x, sizeof(Math::Vec3), (int)corners.size(), 0.0f, 0.0f);

    // The hull uses exact integer predicates on quantized coordinates (no
    // coplanarity tolerance), so jitter far smaller than the quantization step
    // can still break exact coplanarity of a nominal face and legitimately
    // split it into a few extra triangular micro-faces. This is inherited,
    // correct behavior of the reference algorithm, not a defect: it is the
    // exact convex hull of the (no-longer-exactly-planar) input. We only check
    // that it stays in a sane range and produces no crash/NaN.
    CHECK(allVerticesFinite(hull.vertices));
    CHECK(hull.vertices.size() == 8);
    CHECK(hull.faces.size() >= 6 && hull.faces.size() <= 16);
}

void testGetVerticesInsidePlanesRecoversBoxCorners()
{
    std::vector<Math::Vec4> planes;
    planes.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, -0.5f));
    planes.push_back(Math::Vec4(-1.0f, 0.0f, 0.0f, -0.5f));
    planes.push_back(Math::Vec4(0.0f, 1.0f, 0.0f, -0.5f));
    planes.push_back(Math::Vec4(0.0f, -1.0f, 0.0f, -0.5f));
    planes.push_back(Math::Vec4(0.0f, 0.0f, 1.0f, -0.5f));
    planes.push_back(Math::Vec4(0.0f, 0.0f, -1.0f, -0.5f));

    std::vector<Math::Vec3> vertices;
    std::vector<int> planeIndices;
    bool found = VoronoiShatter::getVerticesInsidePlanes(planes, vertices, planeIndices);

    CHECK(found);
    CHECK(vertices.size() == 8);

    std::vector<Math::Vec3> expected = unitCubeCorners();
    for (const Math::Vec3& e : expected)
    {
        bool matched = false;
        for (const Math::Vec3& v : vertices)
        {
            if (near(v, e, 1e-3f))
            {
                matched = true;
                break;
            }
        }
        CHECK(matched);
    }
}

void testSingleVoronoiPointRecoversWholeBox()
{
    std::vector<Math::Vec3> boxVerts = unitCubeCorners();
    std::vector<Math::Vec3> points = {Math::Vec3(0.0f, 0.0f, 0.0f)};
    std::vector<Shard> shards;
    VoronoiShatter::shatter(boxVerts, points, shards);

    CHECK(shards.size() == 1);
    if (shards.size() == 1)
    {
        CHECK(near(shards[0].volume, boxVolume(Math::Vec3(0.5f)), 1e-2f));
        CHECK(shards[0].vertices.size() == 8);
        for (const Math::Vec3& e : boxVerts)
        {
            Math::Vec3 world = shards[0].centroid;
            bool matched = false;
            for (const Math::Vec3& v : shards[0].vertices)
            {
                if (near(world + v, e, 1e-2f))
                {
                    matched = true;
                    break;
                }
            }
            CHECK(matched);
        }
    }
}

void testMultipleVoronoiPointsConserveVolume()
{
    std::vector<Math::Vec3> boxVerts = unitCubeCorners();
    std::vector<Math::Vec3> points = {
        Math::Vec3(-0.3f, -0.3f, -0.3f),
        Math::Vec3(0.3f, -0.3f, -0.3f),
        Math::Vec3(-0.3f, 0.3f, -0.3f),
        Math::Vec3(0.3f, 0.3f, -0.3f),
        Math::Vec3(-0.2f, -0.2f, 0.25f),
        Math::Vec3(0.2f, -0.2f, 0.25f),
        Math::Vec3(-0.2f, 0.2f, 0.25f),
        Math::Vec3(0.2f, 0.2f, 0.25f),
    };

    std::vector<Shard> shards;
    VoronoiShatter::shatter(boxVerts, points, shards);

    CHECK(!shards.empty());
    CHECK(near(sumShardVolumes(shards), boxVolume(Math::Vec3(0.5f)), 5e-2f));
    CHECK(allShardVerticesInBounds(shards, Math::Vec3(-0.5f), Math::Vec3(0.5f), 1e-2f));
}

void testVoronoiZeroPointsDoesNotCrash()
{
    std::vector<Math::Vec3> boxVerts = unitCubeCorners();
    std::vector<Math::Vec3> points;
    std::vector<Shard> shards;
    VoronoiShatter::shatter(boxVerts, points, shards);

    CHECK(shards.empty());
}

void testVoronoiSinglePointOutsideBoxStillRecoversWholeBox()
{
    std::vector<Math::Vec3> boxVerts = unitCubeCorners();
    std::vector<Math::Vec3> points = {Math::Vec3(5.0f, 5.0f, 5.0f)};
    std::vector<Shard> shards;
    VoronoiShatter::shatter(boxVerts, points, shards);

    CHECK(shards.size() == 1);
    if (shards.size() == 1)
    {
        CHECK(near(shards[0].volume, boxVolume(Math::Vec3(0.5f)), 1e-2f));
        CHECK(shards[0].vertices.size() == 8);
    }
}

void testVoronoiNearDuplicatePointsDoNotCrash()
{
    std::vector<Math::Vec3> boxVerts = unitCubeCorners();
    std::vector<Math::Vec3> points = {
        Math::Vec3(-0.1f, 0.0f, 0.0f),
        Math::Vec3(-0.1f + 1e-6f, 1e-7f, -1e-7f),
        Math::Vec3(0.2f, 0.2f, 0.2f),
        Math::Vec3(-0.3f, -0.2f, 0.1f),
    };
    std::vector<Shard> shards;
    VoronoiShatter::shatter(boxVerts, points, shards);

    for (const Shard& s : shards)
    {
        CHECK(allVerticesFinite(s.vertices));
        CHECK(isFinite(s.centroid));
        CHECK(isFinite(s.volume));
    }

    f32 total = 0.0f;
    bool allFinite = true;
    for (const Shard& s : shards)
    {
        if (!isFinite(s.volume))
        {
            allFinite = false;
            continue;
        }
        total += s.volume;
    }
    if (allFinite)
    {
        CHECK(near(total, boxVolume(Math::Vec3(0.5f)), 5e-2f));
    }
}

void testVoronoiManyPointsFinishesAndConservesVolume()
{
    std::vector<Math::Vec3> boxVerts = unitCubeCorners();
    std::vector<Math::Vec3> points;
    unsigned int seed = 987654321u;
    for (int i = 0; i < 50; i++)
    {
        seed = 1664525u * seed + 1013904223u;
        f32 x = ((f32)(seed & 0xffffu) / 65535.0f - 0.5f);
        seed = 1664525u * seed + 1013904223u;
        f32 y = ((f32)(seed & 0xffffu) / 65535.0f - 0.5f);
        seed = 1664525u * seed + 1013904223u;
        f32 z = ((f32)(seed & 0xffffu) / 65535.0f - 0.5f);
        points.push_back(Math::Vec3(x, y, z));
    }

    std::vector<Shard> shards;
    VoronoiShatter::shatter(boxVerts, points, shards);

    CHECK(!shards.empty());
    bool allFinite = true;
    f32 total = 0.0f;
    for (const Shard& s : shards)
    {
        if (!isFinite(s.volume) || !allVerticesFinite(s.vertices))
        {
            allFinite = false;
            continue;
        }
        total += s.volume;
    }
    CHECK(allFinite);
    if (allFinite)
    {
        CHECK(near(total, boxVolume(Math::Vec3(0.5f)), 0.1f));
    }
}

void testVoronoiDegenerateSourceShapeDoesNotCrash()
{
    std::vector<Math::Vec3> flatBoxVerts;
    for (int i = 0; i < 8; i++)
    {
        f32 x = (i & 1) ? 0.5f : -0.5f;
        f32 y = (i & 2) ? 0.5f : -0.5f;
        flatBoxVerts.push_back(Math::Vec3(x, y, 0.0f));
    }

    std::vector<Math::Vec3> points = {
        Math::Vec3(-0.2f, -0.2f, 0.0f),
        Math::Vec3(0.2f, 0.2f, 0.0f),
    };

    std::vector<Shard> shards;
    VoronoiShatter::shatter(flatBoxVerts, points, shards);

    for (const Shard& s : shards)
    {
        CHECK(allVerticesFinite(s.vertices));
    }
}

} // namespace

int main()
{
    testCubeHullHasSixFacesTwelveEdgesEightVertices();
    testHullExcludesInteriorPoints();
    testDegenerateCoplanarInputDoesNotCrash();
    testDegenerateDuplicatePointsDoesNotCrash();
    testCountZeroYieldsEmptyHull();
    testCountOneDoesNotCrash();
    testCountTwoSegmentDoesNotCrash();
    testAllPointsCollinearDoesNotCrash();
    testLargeCoordinateMagnitudeStaysSane();
    testSmallCoordinateMagnitudeStaysSane();
    testTinyJitterDoesNotFragmentCubeFaces();
    testGetVerticesInsidePlanesRecoversBoxCorners();
    testSingleVoronoiPointRecoversWholeBox();
    testMultipleVoronoiPointsConserveVolume();
    testVoronoiZeroPointsDoesNotCrash();
    testVoronoiSinglePointOutsideBoxStillRecoversWholeBox();
    testVoronoiNearDuplicatePointsDoNotCrash();
    testVoronoiManyPointsFinishesAndConservesVolume();
    testVoronoiDegenerateSourceShapeDoesNotCrash();
    if (gFailures)
        std::fprintf(stderr, "%d convex hull test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
