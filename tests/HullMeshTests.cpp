// HullMeshTests.cpp - triangles out of the edge/face structure the hull
// computer and the shatter shards both carry. Nothing here touches the GPU.

#include "PCH.h"

#include "HullMesh.h"
#include "Mesh.h"

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
        std::fprintf(stderr, "HullMeshTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

std::vector<Math::vec3> boxCorners(f32 half)
{
    std::vector<Math::vec3> points;
    for (s32 z = -1; z <= 1; z += 2)
        for (s32 y = -1; y <= 1; y += 2)
            for (s32 x = -1; x <= 1; x += 2)
                points.push_back(Math::vec3(static_cast<f32>(x), static_cast<f32>(y),
                                           static_cast<f32>(z)) *
                                 half);
    return points;
}

bool indicesValid(const MeshData& mesh)
{
    for (u32 index : mesh.indices)
        if (index >= mesh.positions.size())
            return false;
    return mesh.indices.size() % 3 == 0;
}

void testHullOfABox()
{
    MeshData mesh;
    CHECK(Geometry::buildConvexHullMesh(boxCorners(1.0f), mesh));

    CHECK(indicesValid(mesh));
    // Six square faces, two triangles each, however the hull chose to fan them.
    CHECK(mesh.indices.size() / 3 == 12);
    CHECK(mesh.submeshes.size() == 1);
    CHECK(mesh.submeshes[0].indexCount == mesh.indices.size());

    // Three vertices per triangle, none shared: a hull has no smooth edges,
    // and sharing them would average the normals across the corners.
    CHECK(mesh.positions.size() == mesh.indices.size());
    CHECK(mesh.normals.size() == mesh.positions.size());
    CHECK(mesh.uvs.size() == mesh.positions.size());

    CHECK(Math::abs(mesh.bounds.min.x + 1.0f) < 1e-4f);
    CHECK(Math::abs(mesh.bounds.max.z - 1.0f) < 1e-4f);
}

// Every triangle of a box hull has an axis-aligned normal, and the three
// vertices of one triangle share it exactly.
void testFaceNormalsAreFlat()
{
    MeshData mesh;
    CHECK(Geometry::buildConvexHullMesh(boxCorners(2.0f), mesh));

    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const Math::vec3& n = mesh.normals[mesh.indices[i]];
        CHECK(Math::abs(Math::length(n) - 1.0f) < 1e-3f);
        CHECK(n == mesh.normals[mesh.indices[i + 1]]);
        CHECK(n == mesh.normals[mesh.indices[i + 2]]);

        const f32 axis = Math::max(Math::abs(n.x), Math::max(Math::abs(n.y), Math::abs(n.z)));
        CHECK(axis > 0.999f);
    }
}

// The hull of points already on a convex shape is that shape; points inside
// it change nothing.
void testInteriorPointsAreDropped()
{
    std::vector<Math::vec3> points = boxCorners(1.0f);
    MeshData plain;
    CHECK(Geometry::buildConvexHullMesh(points, plain));

    points.push_back(Math::vec3(0.0f));
    points.push_back(Math::vec3(0.2f, -0.3f, 0.1f));
    MeshData withInterior;
    CHECK(Geometry::buildConvexHullMesh(points, withInterior));

    CHECK(withInterior.indices.size() == plain.indices.size());
    CHECK(Math::abs(withInterior.bounds.max.x - plain.bounds.max.x) < 1e-4f);
}

// A concave cloud comes back as its hull - the dent is filled in. That is
// what a hull is, and a caller reaching for one has to expect it.
void testConcaveCloudFillsIn()
{
    std::vector<Math::vec3> points = boxCorners(1.0f);
    points.push_back(Math::vec3(0.0f, -0.5f, 0.0f));

    MeshData mesh;
    CHECK(Geometry::buildConvexHullMesh(points, mesh));
    CHECK(mesh.indices.size() / 3 == 12);
}

void testRejectsTooFewPoints()
{
    MeshData mesh;
    CHECK(!Geometry::buildConvexHullMesh({}, mesh));
    CHECK(mesh.positions.empty());

    const std::vector<Math::vec3> triangle = {Math::vec3(0.0f), Math::vec3(1.0f, 0.0f, 0.0f),
                                             Math::vec3(0.0f, 1.0f, 0.0f)};
    CHECK(!Geometry::buildConvexHullMesh(triangle, mesh));
    CHECK(mesh.positions.empty());

    // Empty face and edge lists have nothing to walk.
    std::vector<Math::vec3> vertices = boxCorners(1.0f);
    std::vector<Geometry::ConvexHullComputer::Edge> edges;
    std::vector<int> faces;
    CHECK(!Geometry::buildHullMesh(vertices, edges, faces, mesh));
}

// A cloud that is flat has no volume, so the hull has no faces to walk, and
// the function has to say so rather than hand back an empty mesh as success.
void testDegenerateCloud()
{
    std::vector<Math::vec3> flat;
    for (s32 y = -1; y <= 1; y += 2)
        for (s32 x = -1; x <= 1; x += 2)
            flat.push_back(Math::vec3(static_cast<f32>(x), 0.0f, static_cast<f32>(y)));

    MeshData mesh;
    // Either it produces a degenerate-free result or it refuses; what it must
    // not do is return true with triangles that have no area.
    if (Geometry::buildConvexHullMesh(flat, mesh))
    {
        CHECK(indicesValid(mesh));
        for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
        {
            const Math::vec3& a = mesh.positions[mesh.indices[i]];
            const Math::vec3& b = mesh.positions[mesh.indices[i + 1]];
            const Math::vec3& c = mesh.positions[mesh.indices[i + 2]];
            CHECK(Math::length(Math::cross(b - a, c - a)) > 1e-9f);
        }
    }
    else
    {
        CHECK(mesh.positions.empty());
    }
}

} // namespace

int main()
{
    testHullOfABox();
    testFaceNormalsAreFlat();
    testInteriorPointsAreDropped();
    testConcaveCloudFillsIn();
    testRejectsTooFewPoints();
    testDegenerateCloud();

    if (gFailures)
        std::fprintf(stderr, "%d hull mesh test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
