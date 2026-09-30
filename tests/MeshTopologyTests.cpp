#include "PCH.h"

#include "mesh/MeshTopology.h"

#include <cstdio>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "MeshTopologyTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

MeshData splitCube()
{
    MeshData mesh;
    const Math::vec3 corner[8] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                                 {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    const u32 faces[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4},
                             {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}};
    for (const auto& face : faces)
    {
        const u32 base = static_cast<u32>(mesh.positions.size());
        for (const u32 index : face)
            mesh.positions.push_back(corner[index]);
        mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    return mesh;
}

MeshData grid(u32 n)
{
    MeshData mesh;
    for (u32 z = 0; z <= n; ++z)
        for (u32 x = 0; x <= n; ++x)
            mesh.positions.push_back(Math::vec3(static_cast<f32>(x), 0, static_cast<f32>(z)));
    const u32 row = n + 1;
    for (u32 z = 0; z < n; ++z)
    {
        for (u32 x = 0; x < n; ++x)
        {
            const u32 a = z * row + x;
            mesh.indices.insert(mesh.indices.end(), {a, a + row, a + row + 1, a, a + row + 1, a + 1});
        }
    }
    return mesh;
}

void testSplitCubeIsClosed()
{
    const MeshData mesh = splitCube();
    MeshTopology topology;
    topology.build(mesh);

    std::vector<u32> ids;
    for (u32 v = 0; v < mesh.positions.size(); ++v)
        ids.push_back(topology.canonical(v));
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    CHECK(ids.size() == 8);
    for (u32 v = 0; v < mesh.positions.size(); ++v)
        CHECK(topology.canonical(v) <= v);

    CHECK(topology.edges().size() == 18);
    for (u32 e = 0; e < topology.edges().size(); ++e)
    {
        CHECK(topology.edges()[e].faces.size() == 2);
        CHECK(!topology.isBoundary(e));
        CHECK(!topology.isNonManifold(e));
    }
    CHECK(topology.boundaryLoops(mesh).empty());
}

void testNeighboursAcrossSeams()
{
    const MeshData mesh = splitCube();
    MeshTopology topology;
    topology.build(mesh);

    std::vector<u32> neighbours;
    topology.faceNeighbors(mesh, 0, neighbours);
    CHECK(neighbours.size() == 3);
    CHECK(std::find(neighbours.begin(), neighbours.end(), 1u) != neighbours.end());
}

void testCoincident()
{
    const MeshData mesh = splitCube();
    MeshTopology topology;
    topology.build(mesh);
    const std::vector<u32> group = topology.coincident(0);
    CHECK(group.size() == 3);
    CHECK(group.front() == 0);
    for (const u32 v : group)
        CHECK(topology.canonical(v) == 0);
}

void testGridBoundary()
{
    const MeshData mesh = grid(2);
    MeshTopology topology;
    topology.build(mesh);

    const std::vector<std::vector<u32>> loops = topology.boundaryLoops(mesh);
    CHECK(loops.size() == 1);
    CHECK(loops[0].size() == 8);

    u32 boundaryEdges = 0;
    for (u32 e = 0; e < topology.edges().size(); ++e)
        boundaryEdges += topology.isBoundary(e) ? 1u : 0u;
    CHECK(boundaryEdges == 8);

    const std::vector<u32>& loop = loops[0];
    for (usize i = 0; i < loop.size(); ++i)
    {
        const u32 from = loop[i];
        const u32 to = loop[(i + 1) % loop.size()];
        const s32 edge = topology.findEdge(from, to);
        CHECK(edge >= 0 && topology.isBoundary(static_cast<u32>(edge)));
        if (edge < 0)
            continue;
        const u32 face = topology.edges()[static_cast<usize>(edge)].faces[0];
        bool runsForward = false;
        for (u32 c = 0; c < 3; ++c)
        {
            if (topology.canonical(mesh.indices[face * 3 + c]) == from &&
                topology.canonical(mesh.indices[face * 3 + (c + 1) % 3]) == to)
                runsForward = true;
        }
        CHECK(runsForward);
    }
}

void testTwoHolesGiveTwoLoops()
{
    MeshData mesh = grid(3);
    const usize quad = 1 * 3 + 1;
    mesh.indices.erase(mesh.indices.begin() + static_cast<long>(quad * 6),
                       mesh.indices.begin() + static_cast<long>(quad * 6 + 6));
    MeshTopology topology;
    topology.build(mesh);
    const auto loops = topology.boundaryLoops(mesh);
    CHECK(loops.size() == 2);
    const usize a = loops[0].size();
    const usize b = loops[1].size();
    CHECK((a == 12 && b == 4) || (a == 4 && b == 12));
}

void testNonManifoldAndDegenerate()
{
    MeshData fan;
    fan.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, -1, 0}};
    fan.indices = {0, 1, 2, 0, 1, 3, 0, 1, 4};
    MeshTopology topology;
    topology.build(fan);
    const s32 shared = topology.findEdge(0, 1);
    CHECK(shared >= 0);
    CHECK(topology.isNonManifold(static_cast<u32>(shared)));
    CHECK(topology.edges()[static_cast<usize>(shared)].faces.size() == 3);
    CHECK(topology.boundaryLoops(fan).empty());

    // A triangle with two corners at one point has a side with no length.
    MeshData collapsed;
    collapsed.positions = {{0, 0, 0}, {0, 0, 0}, {1, 0, 0}};
    collapsed.indices = {0, 1, 2};
    topology.build(collapsed);
    CHECK(topology.faceEdges(0)[0] == -1);
    CHECK(topology.faceEdges(0)[1] >= 0);
    CHECK(topology.faceEdges(0)[2] >= 0);
}

void testToleranceAndDeterminism()
{
    MeshData mesh;
    mesh.positions = {{0, 0, 0}, {1.0f, 0, 0}, {1.0f + 2.0e-6f, 0, 0}, {0, 1, 0}};
    mesh.indices = {0, 1, 3, 2, 3, 1};
    MeshTopology a;
    MeshTopology b;
    a.build(mesh);
    b.build(mesh);
    CHECK(a.canonical(2) == 1);
    CHECK(a.edges().size() == b.edges().size());
    for (usize i = 0; i < a.edges().size(); ++i)
        CHECK(a.edges()[i].a == b.edges()[i].a && a.edges()[i].b == b.edges()[i].b);

    MeshTopology strict;
    strict.build(mesh, 1.0e-7f);
    CHECK(strict.canonical(2) == 2);
}

void testInvalidIndicesAreSkipped()
{
    MeshData mesh;
    mesh.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    mesh.indices = {0, 1, 2, 0, 1, 99};
    MeshTopology topology;
    topology.build(mesh);
    CHECK(topology.faceCount() == 2);
    CHECK(topology.faceEdges(1)[0] >= 0);
    CHECK(topology.faceEdges(1)[1] == -1);
}

} // namespace

int main()
{
    testSplitCubeIsClosed();
    testNeighboursAcrossSeams();
    testCoincident();
    testGridBoundary();
    testTwoHolesGiveTwoLoops();
    testNonManifoldAndDegenerate();
    testToleranceAndDeterminism();
    testInvalidIndicesAreSkipped();

    if (gFailures)
        std::fprintf(stderr, "%d mesh topology test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
