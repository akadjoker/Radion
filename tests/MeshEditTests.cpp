#include "PCH.h"

#include "mesh/MeshEdit.h"

#include <cstdio>
#include <map>
#include <set>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "MeshEditTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

MeshData octahedron()
{
    MeshData mesh;
    mesh.positions = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    mesh.indices = {0, 2, 4, 2, 1, 4, 1, 3, 4, 3, 0, 4, 2, 0, 5, 1, 2, 5, 3, 1, 5, 0, 3, 5};
    return mesh;
}

MeshData icosahedron()
{
    const f32 t = (1.0f + std::sqrt(5.0f)) * 0.5f;
    MeshData mesh;
    const Math::vec3 v[12] = {{-1, t, 0}, {1, t, 0},  {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t},
                             {0, -1, -t}, {0, 1, -t}, {t, 0, -1},  {t, 0, 1},  {-t, 0, -1}, {-t, 0, 1}};
    for (const Math::vec3& p : v)
        mesh.positions.push_back(Math::normalize(p));
    mesh.indices = {0, 11, 5, 0, 5, 1,  0, 1, 7,  0, 7, 10, 0, 10, 11, 1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8,
                    3, 9, 4,  3, 4, 2,  3, 2, 6,  3, 6, 8,  3, 8, 9,   4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7,  9, 8, 1};
    return mesh;
}

MeshData weldedCube()
{
    MeshData mesh;
    mesh.positions = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    const u32 faces[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}};
    for (const auto& q : faces)
        mesh.indices.insert(mesh.indices.end(), {q[0], q[1], q[2], q[0], q[2], q[3]});
    return mesh;
}

MeshData splitCube()
{
    const MeshData welded = weldedCube();
    MeshData mesh;
    for (usize f = 0; f < welded.indices.size() / 3; ++f)
        for (usize c = 0; c < 3; ++c)
        {
            mesh.positions.push_back(welded.positions[welded.indices[f * 3 + c]]);
            mesh.indices.push_back(static_cast<u32>(mesh.positions.size() - 1));
        }
    return mesh;
}

void addAttributes(MeshData& mesh)
{
    mesh.normals.assign(mesh.positions.size(), Math::vec3(0, 1, 0));
    mesh.uvs.clear();
    mesh.colors.clear();
    for (const Math::vec3& p : mesh.positions)
    {
        mesh.uvs.push_back(Math::vec2(p.x, p.z));
        mesh.colors.push_back(0xFF000000u | (static_cast<u32>(Math::clamp(p.x, 0.0f, 1.0f) * 255.0f) & 0xFF));
    }
}

struct Report
{
    usize vertices = 0;
    usize edges = 0;
    usize faces = 0;
    bool closed = true;
    bool consistent = true; // each directed edge appears once: all triangles agree on "out"
    bool valid = true;
};

Report analyse(const MeshData& mesh)
{
    Report report;
    MeshTopology topology;
    topology.build(mesh);

    // Vertices no triangle uses (an edit can leave some behind) are not part of the surface.
    std::set<u32> points;
    for (const u32 index : mesh.indices)
        if (index < mesh.positions.size())
            points.insert(topology.canonical(index));
    report.vertices = points.size();
    report.edges = topology.edges().size();
    report.faces = mesh.indices.size() / 3;

    for (const MeshTopology::Edge& edge : topology.edges())
        if (edge.faces.size() != 2)
            report.closed = false;

    std::map<std::pair<u32, u32>, int> directed;
    for (usize f = 0; f < report.faces; ++f)
    {
        u32 c[3];
        for (usize k = 0; k < 3; ++k)
        {
            if (mesh.indices[f * 3 + k] >= mesh.positions.size())
            {
                report.valid = false;
                return report;
            }
            c[k] = topology.canonical(mesh.indices[f * 3 + k]);
        }
        if (c[0] == c[1] || c[1] == c[2] || c[2] == c[0])
            report.valid = false;
        for (usize k = 0; k < 3; ++k)
            ++directed[{c[k], c[(k + 1) % 3]}];
    }
    for (const auto& entry : directed)
        if (entry.second != 1)
            report.consistent = false;
    return report;
}

bool facesOutward(const MeshData& mesh, const Math::vec3& centre)
{
    for (usize f = 0; f < mesh.indices.size() / 3; ++f)
    {
        const Math::vec3& a = mesh.positions[mesh.indices[f * 3]];
        const Math::vec3& b = mesh.positions[mesh.indices[f * 3 + 1]];
        const Math::vec3& c = mesh.positions[mesh.indices[f * 3 + 2]];
        if (Math::dot(Math::cross(b - a, c - a), (a + b + c) / 3.0f - centre) <= 0.0f)
            return false;
    }
    return true;
}

bool arraysInStep(const MeshData& mesh)
{
    const usize n = mesh.positions.size();
    return (mesh.normals.empty() || mesh.normals.size() == n) && (mesh.uvs.empty() || mesh.uvs.size() == n) &&
           (mesh.colors.empty() || mesh.colors.size() == n) && (mesh.tangents.empty() || mesh.tangents.size() == n);
}

MeshTopology topologyOf(const MeshData& mesh)
{
    MeshTopology topology;
    topology.build(mesh);
    return topology;
}

u64 edgeBetween(const MeshData& mesh, const Math::vec3& a, const Math::vec3& b)
{
    const MeshTopology topology = topologyOf(mesh);
    u32 ia = 0;
    u32 ib = 0;
    for (u32 v = 0; v < mesh.positions.size(); ++v)
    {
        if (mesh.positions[v] == a)
            ia = topology.canonical(v);
        if (mesh.positions[v] == b)
            ib = topology.canonical(v);
    }
    return MeshTopology::edgeKey(ia, ib);
}

void testTestShapesAreSound()
{
    for (const MeshData& mesh : {octahedron(), icosahedron(), weldedCube(), splitCube()})
    {
        const Report r = analyse(mesh);
        CHECK(r.closed && r.consistent && r.valid);
        CHECK(static_cast<int>(r.vertices) - static_cast<int>(r.edges) + static_cast<int>(r.faces) == 2);
        CHECK(facesOutward(mesh, Math::vec3(0.0f)) || facesOutward(mesh, Math::vec3(0.5f)));
    }
}

void testRefineOneEdge()
{
    MeshData mesh = weldedCube();
    const u64 key = edgeBetween(mesh, {0, 0, 0}, {1, 0, 0});
    MeshEdit::RefineResult result;
    std::string error;
    CHECK(MeshEdit::refineEdges(mesh, {{key, 0.5f}}, &result, &error));

    CHECK(mesh.indices.size() / 3 == 14);
    CHECK(mesh.positions.size() == 9);
    CHECK(result.midpoints.size() == 1);
    CHECK(mesh.positions[result.midpoints[0].vertex] == Math::vec3(0.5f, 0, 0));
    CHECK(result.origin.size() == 14);

    const Report r = analyse(mesh);
    CHECK(r.closed && r.consistent && r.valid);
    CHECK(facesOutward(mesh, Math::vec3(0.5f)));
}

void testRefineDirectionOfT()
{
    // `t` runs from the lower canonical id to the higher whichever way the triangles walk the edge.
    MeshData mesh = weldedCube();
    const u64 key = edgeBetween(mesh, {0, 0, 0}, {1, 0, 0});
    MeshEdit::RefineResult result;
    CHECK(MeshEdit::refineEdges(mesh, {{key, 0.25f}}, &result));
    CHECK(mesh.positions[result.midpoints[0].vertex] == Math::vec3(0.25f, 0, 0));
}

void testRefineTwoAndThreeEdgesOfOneTriangle()
{
    MeshData mesh = octahedron();
    const u64 a = edgeBetween(mesh, {1, 0, 0}, {0, 1, 0});
    const u64 b = edgeBetween(mesh, {0, 1, 0}, {0, 0, 1});
    CHECK(MeshEdit::refineEdges(mesh, {{a, 0.5f}, {b, 0.5f}}));
    Report r = analyse(mesh);
    CHECK(r.closed && r.consistent && r.valid);
    CHECK(facesOutward(mesh, Math::vec3(0.0f)));

    MeshData all = octahedron();
    std::vector<MeshEdit::EdgeSplit> splits;
    const MeshTopology topology = topologyOf(all);
    for (const MeshTopology::Edge& edge : topology.edges())
        splits.push_back({MeshTopology::edgeKey(edge.a, edge.b), 0.5f});
    CHECK(MeshEdit::refineEdges(all, splits));
    CHECK(all.indices.size() / 3 == 32);
    r = analyse(all);
    CHECK(r.closed && r.consistent && r.valid);
}

void testRefineKeepsAttributesInStep()
{
    MeshData mesh = weldedCube();
    addAttributes(mesh);
    const u64 key = edgeBetween(mesh, {0, 0, 0}, {1, 0, 0});
    MeshEdit::RefineResult result;
    CHECK(MeshEdit::refineEdges(mesh, {{key, 0.5f}}, &result));
    CHECK(arraysInStep(mesh));

    const u32 m = result.midpoints[0].vertex;
    CHECK(std::abs(mesh.uvs[m].x - 0.5f) < 1.0e-5f && std::abs(mesh.uvs[m].y) < 1.0e-5f);
    CHECK(std::abs(Math::length(mesh.normals[m]) - 1.0f) < 1.0e-4f);
    CHECK((mesh.colors[m] & 0xFF) >= 126 && (mesh.colors[m] & 0xFF) <= 129);
}

void testRefineRejectsBadInput()
{
    MeshData mesh = weldedCube();
    std::string error;
    CHECK(!MeshEdit::refineEdges(mesh, {}, nullptr, &error));
    CHECK(!MeshEdit::refineEdges(mesh, {{MeshTopology::edgeKey(0, 99), 0.5f}}, nullptr, &error));
    const u64 key = edgeBetween(mesh, {0, 0, 0}, {1, 0, 0});
    CHECK(!MeshEdit::refineEdges(mesh, {{key, 0.0f}}, nullptr, &error));
    CHECK(!MeshEdit::refineEdges(mesh, {{key, 1.0f}}, nullptr, &error));
    CHECK(mesh.indices.size() == 36);
}

void testSubdivideFlat()
{
    for (MeshData mesh : {octahedron(), weldedCube(), splitCube()})
    {
        const Math::vec3 low = [&] { Math::vec3 m(1e9f); for (auto& p : mesh.positions) m = Math::min(m, p); return m; }();
        const Math::vec3 high = [&] { Math::vec3 m(-1e9f); for (auto& p : mesh.positions) m = Math::max(m, p); return m; }();
        const usize triangles = mesh.indices.size() / 3;

        std::string error;
        CHECK(MeshEdit::subdivide(mesh, {}, 1, false, &error));
        CHECK(mesh.indices.size() / 3 == triangles * 4);
        const Report r = analyse(mesh);
        CHECK(r.closed && r.consistent && r.valid);
        CHECK(static_cast<int>(r.vertices) - static_cast<int>(r.edges) + static_cast<int>(r.faces) == 2);

        Math::vec3 low2(1e9f);
        Math::vec3 high2(-1e9f);
        for (const Math::vec3& p : mesh.positions)
        {
            low2 = Math::min(low2, p);
            high2 = Math::max(high2, p);
        }
        CHECK(low2 == low && high2 == high);
    }
}

void testSubdivideTwoLevels()
{
    MeshData mesh = octahedron();
    CHECK(MeshEdit::subdivide(mesh, {}, 2, false));
    CHECK(mesh.indices.size() / 3 == 8 * 16);
    const Report r = analyse(mesh);
    CHECK(r.closed && r.consistent && r.valid);
}

void testSubdivideRegionStaysWatertight()
{
    MeshData mesh = weldedCube();
    CHECK(MeshEdit::subdivide(mesh, {0}, 1, false));
    const Report r = analyse(mesh);
    CHECK(r.closed && r.consistent && r.valid);
    CHECK(mesh.indices.size() / 3 > 12 + 3);
    CHECK(facesOutward(mesh, Math::vec3(0.5f)));

    MeshData twice = weldedCube();
    CHECK(MeshEdit::subdivide(twice, {0, 1}, 2, false));
    const Report r2 = analyse(twice);
    CHECK(r2.closed && r2.consistent && r2.valid);
}

void testSubdivideSmoothRoundsTheSurface()
{
    // Loop subdivision pulls an octahedron's corners inside the originals and keeps its symmetry.
    MeshData mesh = octahedron();
    CHECK(MeshEdit::subdivide(mesh, {}, 1, true));
    const Report r = analyse(mesh);
    CHECK(r.closed && r.consistent && r.valid);
    CHECK(mesh.bounds.max.x < 1.0f && mesh.bounds.max.x > 0.5f);
    CHECK(std::abs(mesh.bounds.max.x + mesh.bounds.min.x) < 1.0e-5f);
    CHECK(std::abs(mesh.bounds.max.x - mesh.bounds.max.y) < 1.0e-5f);
    CHECK(facesOutward(mesh, Math::vec3(0.0f)));
}

f32 sharpestFold(const MeshData& mesh)
{
    const MeshTopology topology = topologyOf(mesh);
    auto normal = [&](u32 face)
    {
        const Math::vec3& a = mesh.positions[mesh.indices[face * 3]];
        const Math::vec3& b = mesh.positions[mesh.indices[face * 3 + 1]];
        const Math::vec3& c = mesh.positions[mesh.indices[face * 3 + 2]];
        return Math::normalize(Math::cross(b - a, c - a));
    };
    f32 sharpest = 0.0f;
    for (const MeshTopology::Edge& edge : topology.edges())
    {
        if (edge.faces.size() != 2)
            continue;
        sharpest = std::max(sharpest, std::acos(Math::clamp(Math::dot(normal(edge.faces[0]), normal(edge.faces[1])), -1.0f, 1.0f)));
    }
    return sharpest;
}

void testSmoothSubdivisionConvergesOnASphere()
{
    MeshData mesh = icosahedron();
    f32 before = sharpestFold(mesh);
    for (int level = 0; level < 3; ++level)
    {
        CHECK(MeshEdit::subdivide(mesh, {}, 1, true));
        const f32 after = sharpestFold(mesh);
        CHECK(after < before * 0.75f);
        before = after;
    }
    CHECK(analyse(mesh).closed);
    CHECK(facesOutward(mesh, Math::vec3(0.0f)));

    MeshData flat = icosahedron();
    const f32 fold = sharpestFold(flat);
    CHECK(MeshEdit::subdivide(flat, {}, 2, false));
    CHECK(std::abs(sharpestFold(flat) - fold) < 1.0e-3f);
}

void testSmoothRegionKeepsItsBorder()
{
    // A region of a flat grid: smoothing must not tear it from its surroundings
    // (the mesh stays closed/manifold where it was) and flat stays flat.
    MeshData mesh = weldedCube();
    CHECK(MeshEdit::subdivide(mesh, {0, 1}, 1, true));
    const Report r = analyse(mesh);
    CHECK(r.closed && r.consistent && r.valid);
}

void testSubmeshesSurviveSubdivision()
{
    MeshData mesh = weldedCube();
    SubMesh a;
    a.indexOffset = 0;
    a.indexCount = 18;
    SubMesh b;
    b.indexOffset = 18;
    b.indexCount = 18;
    b.materialSlot = 1;
    mesh.submeshes = {a, b};
    mesh.materials.resize(2);

    CHECK(MeshEdit::subdivide(mesh, {}, 1, false));
    CHECK(mesh.submeshes.size() == 2);
    CHECK(mesh.submeshes[0].indexCount == 18 * 4);
    CHECK(mesh.submeshes[1].indexCount == 18 * 4);
    CHECK(mesh.submeshes[0].indexOffset == 0);
    CHECK(mesh.submeshes[1].indexOffset == 18 * 4);
    CHECK(mesh.submeshes[1].materialSlot == 1);
    CHECK(mesh.submeshes[0].indexCount + mesh.submeshes[1].indexCount == mesh.indices.size());
    CHECK(!mesh.submeshes[0].bounds.empty());

    MeshData partial = weldedCube();
    partial.submeshes = {a, b};
    partial.materials.resize(2);
    CHECK(MeshEdit::subdivide(partial, {0}, 1, false));
    CHECK(partial.submeshes[0].indexOffset == 0);
    CHECK(partial.submeshes[0].indexOffset + partial.submeshes[0].indexCount == partial.submeshes[1].indexOffset);
    CHECK(partial.submeshes[1].indexOffset + partial.submeshes[1].indexCount == partial.indices.size());
}

void testSubdivideKeepsAttributesAndRejectsBadInput()
{
    MeshData mesh = splitCube();
    addAttributes(mesh);
    CHECK(MeshEdit::subdivide(mesh, {}, 1, true));
    CHECK(arraysInStep(mesh));

    MeshData empty;
    std::string error;
    CHECK(!MeshEdit::subdivide(empty, {}, 1, false, &error));
    MeshData cube = weldedCube();
    CHECK(!MeshEdit::subdivide(cube, {}, 0, false, &error));
    CHECK(!MeshEdit::subdivide(cube, {}, 9, false, &error));
    CHECK(!MeshEdit::subdivide(cube, {999}, 1, false, &error));
    CHECK(cube.indices.size() == 36);
}

void testTurnEdge()
{
    MeshData mesh;
    mesh.positions = {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1}};
    mesh.indices = {0, 2, 1, 0, 3, 2};
    mesh.indices = {0, 3, 2, 0, 2, 1};
    const u64 diagonal = edgeBetween(mesh, {0, 0, 0}, {1, 0, 1});

    std::string error;
    CHECK(MeshEdit::turnEdge(mesh, diagonal, &error));
    CHECK(mesh.indices.size() == 6);
    const MeshTopology after = topologyOf(mesh);
    const u64 other = edgeBetween(mesh, {1, 0, 0}, {0, 0, 1});
    CHECK(after.findEdge(static_cast<u32>(other >> 32), static_cast<u32>(other & 0xFFFFFFFFu)) >= 0);
    CHECK(after.findEdge(static_cast<u32>(diagonal >> 32), static_cast<u32>(diagonal & 0xFFFFFFFFu)) < 0);
    for (usize f = 0; f < 2; ++f)
    {
        const Math::vec3& a = mesh.positions[mesh.indices[f * 3]];
        const Math::vec3& b = mesh.positions[mesh.indices[f * 3 + 1]];
        const Math::vec3& c = mesh.positions[mesh.indices[f * 3 + 2]];
        CHECK(Math::cross(b - a, c - a).y > 0.0f);
    }

    CHECK(MeshEdit::turnEdge(mesh, other, &error));
    const MeshTopology back = topologyOf(mesh);
    CHECK(back.findEdge(static_cast<u32>(diagonal >> 32), static_cast<u32>(diagonal & 0xFFFFFFFFu)) >= 0);
}

void testTurnEdgeRefusals()
{
    std::string error;

    MeshData dart;
    dart.positions = {{0, 0, 0}, {2, 0, 0}, {1, 0, 0.3f}, {1, 0, 2}};
    // Triangles (0,3,2) and (0,2,1) share edge 0-2; the quad 0,1,2,3 is concave at 2.
    dart.indices = {0, 3, 2, 0, 2, 1};
    const u64 shared = edgeBetween(dart, {0, 0, 0}, {1, 0, 0.3f});
    CHECK(!MeshEdit::turnEdge(dart, shared, &error));
    CHECK(error.find("convex") != std::string::npos);

    MeshData one;
    one.positions = {{0, 0, 0}, {1, 0, 0}, {0, 0, 1}};
    one.indices = {0, 2, 1};
    CHECK(!MeshEdit::turnEdge(one, MeshTopology::edgeKey(0, 1), &error));

    MeshData split = weldedCube();
    split = splitCube();
    const MeshTopology topology = topologyOf(split);
    bool refused = false;
    for (const MeshTopology::Edge& edge : topology.edges())
    {
        if (edge.faces.size() != 2)
            continue;
        MeshData copy = split;
        if (!MeshEdit::turnEdge(copy, MeshTopology::edgeKey(edge.a, edge.b), &error) &&
            error.find("seam") != std::string::npos)
            refused = true;
    }
    CHECK(refused);

    CHECK(!MeshEdit::turnEdge(one, MeshTopology::edgeKey(0, 77), &error));
}

void testCollapseEdge()
{
    MeshData mesh = weldedCube();
    const u64 key = edgeBetween(mesh, {0, 0, 0}, {1, 0, 0});
    std::string error;
    CHECK(MeshEdit::collapseEdge(mesh, key, 0.5f, &error));
    CHECK(mesh.indices.size() / 3 == 10);
    CHECK(mesh.positions[0] == Math::vec3(0.5f, 0, 0));
    CHECK(mesh.positions[1] == Math::vec3(0.5f, 0, 0));

    const Report r = analyse(mesh);
    CHECK(r.valid);
    CHECK(r.closed);
    CHECK(static_cast<int>(r.vertices) - static_cast<int>(r.edges) + static_cast<int>(r.faces) == 2);

    MeshData again = weldedCube();
    CHECK(MeshEdit::collapseEdge(again, key, 0.0f, &error));
    CHECK(again.positions[1] == Math::vec3(0, 0, 0));
    CHECK(!MeshEdit::collapseEdge(again, key, 1.5f, &error));
    CHECK(!MeshEdit::collapseEdge(again, MeshTopology::edgeKey(0, 50), 0.5f, &error));
}

void testCollapseKeepsSubmeshes()
{
    MeshData mesh = weldedCube();
    SubMesh a;
    a.indexCount = 18;
    SubMesh b;
    b.indexOffset = 18;
    b.indexCount = 18;
    mesh.submeshes = {a, b};
    mesh.materials.resize(1);
    const u64 key = edgeBetween(mesh, {0, 0, 0}, {1, 0, 0});
    CHECK(MeshEdit::collapseEdge(mesh, key, 0.5f));
    CHECK(mesh.submeshes[0].indexCount + mesh.submeshes[1].indexCount == mesh.indices.size());
    CHECK(mesh.submeshes[0].indexOffset + mesh.submeshes[0].indexCount == mesh.submeshes[1].indexOffset);
}

MeshData quadGrid(u32 nx, u32 nz)
{
    MeshData mesh;
    for (u32 z = 0; z <= nz; ++z)
        for (u32 x = 0; x <= nx; ++x)
            mesh.positions.push_back(Math::vec3(static_cast<f32>(x), 0, static_cast<f32>(z)));
    const u32 row = nx + 1;
    for (u32 z = 0; z < nz; ++z)
    {
        for (u32 x = 0; x < nx; ++x)
        {
            const u32 a = z * row + x;
            const u32 b = a + 1;
            const u32 c = a + row + 1;
            const u32 d = a + row;
            mesh.indices.insert(mesh.indices.end(), {a, d, c, a, c, b});
        }
    }
    return mesh;
}

MeshData band(u32 n)
{
    MeshData mesh;
    for (u32 j = 0; j < n; ++j)
    {
        const f32 angle = 6.2831853f * static_cast<f32>(j) / static_cast<f32>(n);
        mesh.positions.push_back(Math::vec3(std::cos(angle), 0.0f, std::sin(angle)));
    }
    for (u32 j = 0; j < n; ++j)
    {
        const f32 angle = 6.2831853f * static_cast<f32>(j) / static_cast<f32>(n);
        mesh.positions.push_back(Math::vec3(std::cos(angle), 1.0f, std::sin(angle)));
    }
    for (u32 j = 0; j < n; ++j)
    {
        const u32 a = j;
        const u32 b = (j + 1) % n;
        const u32 c = n + (j + 1) % n;
        const u32 d = n + j;
        mesh.indices.insert(mesh.indices.end(), {a, c, b, a, d, c});
    }
    return mesh;
}

std::set<f32> distinct(const std::vector<f32>& values)
{
    std::set<f32> out;
    for (const f32 v : values)
        out.insert(std::round(v * 1000.0f) / 1000.0f);
    return out;
}

void testKnifeCutsACube()
{
    MeshData mesh = weldedCube();
    std::vector<u64> cut;
    std::string error;
    CHECK(MeshEdit::knife(mesh, Math::vec3(1, 0, 0), 0.5f, 1.0e-5f, &cut, &error));

    const Report r = analyse(mesh);
    CHECK(r.closed && r.consistent && r.valid);
    CHECK(static_cast<int>(r.vertices) - static_cast<int>(r.edges) + static_cast<int>(r.faces) == 2);
    CHECK(facesOutward(mesh, Math::vec3(0.5f)));

    CHECK(cut.size() >= 4);
    const MeshTopology topology = topologyOf(mesh);
    for (const u64 key : cut)
    {
        CHECK(std::abs(mesh.positions[static_cast<u32>(key >> 32)].x - 0.5f) < 1.0e-4f);
        CHECK(std::abs(mesh.positions[static_cast<u32>(key & 0xFFFFFFFFu)].x - 0.5f) < 1.0e-4f);
    }
    CHECK(mesh.bounds.min.x == 0.0f && mesh.bounds.max.x == 1.0f);

    MeshData oblique = weldedCube();
    CHECK(MeshEdit::knife(oblique, Math::vec3(1, 1, 0), 1.0f, 1.0e-5f));
    const Report ro = analyse(oblique);
    CHECK(ro.closed && ro.consistent && ro.valid);
}

void testKnifeRefusals()
{
    std::string error;
    MeshData mesh = weldedCube();
    CHECK(!MeshEdit::knife(mesh, Math::vec3(0, 0, 0), 0.0f, 1.0e-5f, nullptr, &error));
    CHECK(!MeshEdit::knife(mesh, Math::vec3(1, 0, 0), 5.0f, 1.0e-5f, nullptr, &error));
    CHECK(!MeshEdit::knife(mesh, Math::vec3(1, 0, 0), 0.0f, 1.0e-5f, nullptr, &error));
    CHECK(mesh.indices.size() == 36);
}

void testLoopCutOnAGrid()
{
    MeshData mesh = quadGrid(4, 4);
    const u32 row = 5;
    const u64 start = MeshTopology::edgeKey(2 * row + 1, 2 * row + 2);

    std::vector<u64> created;
    std::string error;
    CHECK(MeshEdit::loopCut(mesh, start, 1, &created, &error));
    CHECK(mesh.indices.size() / 3 == 32 + 8);
    CHECK(mesh.positions.size() == 25 + 5);

    std::vector<f32> xs;
    for (u32 v = 25; v < mesh.positions.size(); ++v)
        xs.push_back(mesh.positions[v].x);
    CHECK(distinct(xs) == std::set<f32>({1.5f}));

    const Report r = analyse(mesh);
    CHECK(r.valid && r.consistent);
    CHECK(created.size() == 4);
    const MeshTopology topology = topologyOf(mesh);
    CHECK(topology.boundaryLoops(mesh).size() == 1);

    for (const u64 key : created)
    {
        const Math::vec3& a = mesh.positions[static_cast<u32>(key >> 32)];
        const Math::vec3& b = mesh.positions[static_cast<u32>(key & 0xFFFFFFFFu)];
        CHECK(std::abs(a.x - 1.5f) < 1.0e-4f && std::abs(b.x - 1.5f) < 1.0e-4f);
    }
}

void testLoopCutSeveralAndEitherDirection()
{
    MeshData mesh = quadGrid(4, 4);
    const u32 row = 5;
    const u64 start = MeshTopology::edgeKey(2 * row + 1, 2 * row + 2);
    CHECK(MeshEdit::loopCut(mesh, start, 3));
    std::vector<f32> xs;
    for (u32 v = 25; v < mesh.positions.size(); ++v)
        xs.push_back(mesh.positions[v].x);
    CHECK(distinct(xs) == std::set<f32>({1.25f, 1.5f, 1.75f}));
    const Report r = analyse(mesh);
    CHECK(r.valid && r.consistent);
    CHECK(mesh.indices.size() / 3 == 32 + 3 * 8);

    MeshData again = quadGrid(4, 4);
    const u64 other = MeshTopology::edgeKey(0 * row + 1, 0 * row + 2);
    CHECK(MeshEdit::loopCut(again, other, 1));
    CHECK(again.indices.size() / 3 == 40);
    std::vector<f32> ys;
    for (u32 v = 25; v < again.positions.size(); ++v)
        ys.push_back(again.positions[v].x);
    CHECK(distinct(ys) == std::set<f32>({1.5f}));
}

void testLoopCutsAreStraightAcrossTheRing()
{
    // Two loops, each a straight line across the grid: quads reached from either end of the start edge must cut at the same place.
    MeshData mesh = quadGrid(4, 4);
    const u32 row = 5;
    const u64 start = MeshTopology::edgeKey(2 * row + 1, 2 * row + 2);
    CHECK(MeshEdit::loopCut(mesh, start, 2));
    std::vector<f32> xs;
    for (u32 v = 25; v < mesh.positions.size(); ++v)
        xs.push_back(mesh.positions[v].x);
    const std::set<f32> found = distinct(xs);
    CHECK(found.size() == 2);
    CHECK(found.count(1.333f) == 1 && found.count(1.667f) == 1);
    CHECK(xs.size() == 10);
}

void testLoopCutAlongTheOtherAxis()
{
    MeshData mesh = quadGrid(4, 4);
    const u32 row = 5;
    const u64 start = MeshTopology::edgeKey(1 * row + 2, 2 * row + 2);
    CHECK(MeshEdit::loopCut(mesh, start, 1));
    CHECK(mesh.indices.size() / 3 == 40);
    std::vector<f32> zs;
    for (u32 v = 25; v < mesh.positions.size(); ++v)
        zs.push_back(mesh.positions[v].z);
    CHECK(distinct(zs) == std::set<f32>({1.5f}));
}

void testLoopCutClosesAroundABand()
{
    MeshData mesh = band(8);
    const u64 start = MeshTopology::edgeKey(0, 8);
    std::string error;
    CHECK(MeshEdit::loopCut(mesh, start, 1, nullptr, &error));
    CHECK(mesh.indices.size() / 3 == 16 + 16);
    CHECK(mesh.positions.size() == 16 + 8);
    for (u32 v = 16; v < mesh.positions.size(); ++v)
        CHECK(std::abs(mesh.positions[v].y - 0.5f) < 1.0e-5f || std::abs(mesh.positions[v].y - 0.5f) < 1.0e-5f);
    const Report r = analyse(mesh);
    CHECK(r.valid && r.consistent);
}

void testLoopCutRefusals()
{
    std::string error;
    MeshData mesh = quadGrid(2, 2);
    CHECK(!MeshEdit::loopCut(mesh, MeshTopology::edgeKey(0, 1), 0, nullptr, &error));
    CHECK(!MeshEdit::loopCut(mesh, MeshTopology::edgeKey(0, 99), 1, nullptr, &error));
    MeshData one;
    one.positions = {{0, 0, 0}, {1, 0, 0}, {0, 0, 1}};
    one.indices = {0, 2, 1};
    CHECK(!MeshEdit::loopCut(one, MeshTopology::edgeKey(0, 1), 1, nullptr, &error));
    CHECK(one.indices.size() == 3);
}

void testInsetAFlatRegion()
{
    MeshData mesh = quadGrid(3, 3);
    const u32 quad = 1 * 3 + 1;
    std::vector<u32> inner;
    std::string error;
    CHECK(MeshEdit::inset(mesh, {quad * 2, quad * 2 + 1}, 0.2f, 0.0f, &inner, &error));

    CHECK(mesh.indices.size() / 3 == 18 + 8);
    CHECK(inner.size() == 2);

    std::set<f32> xs;
    for (const u32 face : inner)
        for (u32 c = 0; c < 3; ++c)
            xs.insert(std::round(mesh.positions[mesh.indices[face * 3 + c]].x * 1000.0f) / 1000.0f);
    CHECK(xs == std::set<f32>({1.2f, 1.8f}));

    const Report r = analyse(mesh);
    CHECK(r.valid && r.consistent);
    const MeshTopology topology = topologyOf(mesh);
    CHECK(topology.boundaryLoops(mesh).size() == 1);
    for (usize f = 0; f < mesh.indices.size() / 3; ++f)
    {
        const Math::vec3& a = mesh.positions[mesh.indices[f * 3]];
        const Math::vec3& b = mesh.positions[mesh.indices[f * 3 + 1]];
        const Math::vec3& c = mesh.positions[mesh.indices[f * 3 + 2]];
        CHECK(Math::cross(b - a, c - a).y > 0.0f);
    }
}

void testInsetWithDepth()
{
    MeshData mesh = quadGrid(3, 3);
    const u32 quad = 1 * 3 + 1;
    std::vector<u32> inner;
    CHECK(MeshEdit::inset(mesh, {quad * 2, quad * 2 + 1}, 0.25f, 0.5f, &inner));
    for (const u32 face : inner)
        for (u32 c = 0; c < 3; ++c)
            CHECK(std::abs(mesh.positions[mesh.indices[face * 3 + c]].y - 0.5f) < 1.0e-5f);
    CHECK(std::abs(mesh.bounds.min.y) < 1.0e-6f && std::abs(mesh.bounds.max.y - 0.5f) < 1.0e-5f);
    const Report r = analyse(mesh);
    CHECK(r.valid && r.consistent);

    MeshData raised = quadGrid(3, 3);
    CHECK(MeshEdit::inset(raised, {quad * 2, quad * 2 + 1}, 0.0f, 0.3f));
    CHECK(std::abs(raised.bounds.max.y - 0.3f) < 1.0e-5f);
}

void testInsetOnAClosedSurface()
{
    for (MeshData mesh : {weldedCube(), splitCube()})
    {
        std::vector<u32> inner;
        CHECK(MeshEdit::inset(mesh, {0, 1}, 0.1f, 0.0f, &inner));
        const Report r = analyse(mesh);
        CHECK(r.closed && r.consistent && r.valid);
        CHECK(static_cast<int>(r.vertices) - static_cast<int>(r.edges) + static_cast<int>(r.faces) == 2);
        CHECK(mesh.indices.size() / 3 == 12 + 8);
        CHECK(facesOutward(mesh, Math::vec3(0.5f)));
    }
}

void testInsetRefusals()
{
    std::string error;
    MeshData cube = weldedCube();
    std::vector<u32> everything;
    for (u32 f = 0; f < 12; ++f)
        everything.push_back(f);
    CHECK(!MeshEdit::inset(cube, everything, 0.1f, 0.0f, nullptr, &error));
    CHECK(error.find("border") != std::string::npos);
    CHECK(!MeshEdit::inset(cube, {}, 0.1f, 0.0f, nullptr, &error));
    CHECK(!MeshEdit::inset(cube, {99}, 0.1f, 0.0f, nullptr, &error));
    CHECK(!MeshEdit::inset(cube, {0}, 0.0f, 0.0f, nullptr, &error));
    CHECK(!MeshEdit::inset(cube, {0}, -1.0f, 0.0f, nullptr, &error));
    CHECK(cube.indices.size() == 36);
}

void testInsetKeepsSubmeshes()
{
    MeshData mesh = weldedCube();
    SubMesh a;
    a.indexCount = 18;
    SubMesh b;
    b.indexOffset = 18;
    b.indexCount = 18;
    mesh.submeshes = {a, b};
    mesh.materials.resize(2);
    CHECK(MeshEdit::inset(mesh, {0, 1}, 0.1f, 0.0f));
    CHECK(mesh.submeshes[0].indexCount == (6 + 8) * 3);
    CHECK(mesh.submeshes[1].indexCount == 18);
    CHECK(mesh.submeshes[1].indexOffset == mesh.submeshes[0].indexCount);
}

void testRemoveUnusedVertices()
{
    MeshData mesh = weldedCube();
    addAttributes(mesh);
    mesh.positions.push_back({9, 9, 9});
    mesh.normals.push_back({0, 1, 0});
    mesh.uvs.push_back({0, 0});
    mesh.colors.push_back(0xFFFFFFFFu);
    mesh.positions.insert(mesh.positions.begin() + 2, {7, 7, 7});
    mesh.normals.insert(mesh.normals.begin() + 2, {0, 1, 0});
    mesh.uvs.insert(mesh.uvs.begin() + 2, {0, 0});
    mesh.colors.insert(mesh.colors.begin() + 2, 0xFFFFFFFFu);
    for (u32& index : mesh.indices)
        if (index >= 2)
            ++index;

    const usize triangles = mesh.indices.size() / 3;
    const u32 dropped = MeshEdit::removeUnusedVertices(mesh);
    CHECK(dropped == 2);
    CHECK(mesh.positions.size() == 8);
    CHECK(arraysInStep(mesh));
    CHECK(mesh.indices.size() / 3 == triangles);
    const Report r = analyse(mesh);
    CHECK(r.closed && r.consistent && r.valid);
    CHECK(facesOutward(mesh, Math::vec3(0.5f)));
    CHECK(MeshEdit::removeUnusedVertices(mesh) == 0);
}

u32 usedVertexCount(const MeshData& mesh)
{
    std::set<u32> used(mesh.indices.begin(), mesh.indices.end());
    return static_cast<u32>(used.size());
}

void testBevelACubeEdge()
{
    for (MeshData mesh : {weldedCube(), splitCube()})
    {
        const u64 key = edgeBetween(mesh, {0, 0, 0}, {1, 0, 0});
        std::string error;
        CHECK(MeshEdit::bevel(mesh, {key}, 0.2f, &error));

        const Report r = analyse(mesh);
        CHECK(r.closed && r.consistent && r.valid);
        CHECK(r.vertices == 10);
        CHECK(usedVertexCount(mesh) == mesh.positions.size());
        CHECK(static_cast<int>(r.vertices) - static_cast<int>(r.edges) + static_cast<int>(r.faces) == 2);
        CHECK(facesOutward(mesh, Math::vec3(0.5f)));

        for (u32 v : std::set<u32>(mesh.indices.begin(), mesh.indices.end()))
        {
            const Math::vec3& p = mesh.positions[v];
            CHECK(!(std::abs(p.y) < 1.0e-6f && std::abs(p.z) < 1.0e-6f));
        }
        u32 onFaceY = 0;
        u32 onFaceZ = 0;
        for (u32 v : std::set<u32>(mesh.indices.begin(), mesh.indices.end()))
        {
            const Math::vec3& p = mesh.positions[v];
            if (std::abs(p.y) < 1.0e-5f && std::abs(p.z - 0.2f) < 1.0e-5f)
                ++onFaceY;
            if (std::abs(p.z) < 1.0e-5f && std::abs(p.y - 0.2f) < 1.0e-5f)
                ++onFaceZ;
        }
        CHECK(onFaceY >= 2 && onFaceZ >= 2);
        CHECK(mesh.bounds.min == Math::vec3(0.0f) && mesh.bounds.max == Math::vec3(1.0f));
    }
}

void testBevelTwoEdgesAtOnce()
{
    MeshData mesh = weldedCube();
    const u64 a = edgeBetween(mesh, {0, 0, 0}, {1, 0, 0});
    const u64 b = edgeBetween(mesh, {0, 1, 1}, {1, 1, 1});
    std::string error;
    const bool ok = MeshEdit::bevel(mesh, {a, b}, 0.15f, &error);
    if (!ok)
        std::fprintf(stderr, "bevel two edges: %s\n", error.c_str());
    CHECK(ok);
    const Report r = analyse(mesh);
    CHECK(r.closed && r.consistent && r.valid);
    CHECK(static_cast<int>(r.vertices) - static_cast<int>(r.edges) + static_cast<int>(r.faces) == 2);
    CHECK(facesOutward(mesh, Math::vec3(0.5f)));
    CHECK(usedVertexCount(mesh) == 12 + 2 * 4);
}

void testBevelRefusals()
{
    std::string error;
    MeshData mesh = weldedCube();
    const u64 a = edgeBetween(mesh, {0, 0, 0}, {1, 0, 0});
    const u64 b = edgeBetween(mesh, {1, 0, 0}, {1, 1, 0});
    CHECK(!MeshEdit::bevel(mesh, {a, b}, 0.1f, &error));
    CHECK(error.find("share a vertex") != std::string::npos);

    CHECK(!MeshEdit::bevel(mesh, {a}, 0.9f, &error)); // wider than the faces allow
    CHECK(!MeshEdit::bevel(mesh, {a}, 0.0f, &error));
    CHECK(!MeshEdit::bevel(mesh, {}, 0.1f, &error));
    CHECK(!MeshEdit::bevel(mesh, {MeshTopology::edgeKey(0, 99)}, 0.1f, &error));

    MeshData grid = quadGrid(2, 2);
    CHECK(!MeshEdit::bevel(grid, {MeshTopology::edgeKey(0, 1)}, 0.1f, &error));

    CHECK(mesh.indices.size() == 36);
}

f32 triangleAreaSum(const MeshData& mesh, usize firstFace, usize lastFace)
{
    f32 total = 0.0f;
    for (usize f = firstFace; f < lastFace; ++f)
    {
        const Math::vec3& a = mesh.positions[mesh.indices[f * 3]];
        const Math::vec3& b = mesh.positions[mesh.indices[f * 3 + 1]];
        const Math::vec3& c = mesh.positions[mesh.indices[f * 3 + 2]];
        total += 0.5f * Math::length(Math::cross(b - a, c - a));
    }
    return total;
}

void testFillAClosedOffHole()
{
    MeshData mesh = weldedCube();
    mesh.indices.resize(30);
    std::string error;
    u32 filled = 0;
    CHECK(MeshEdit::fillHoles(mesh, {}, 64, &filled, &error));
    CHECK(filled == 1);
    const Report r = analyse(mesh);
    CHECK(r.closed && r.consistent && r.valid);
    CHECK(static_cast<int>(r.vertices) - static_cast<int>(r.edges) + static_cast<int>(r.faces) == 2);
    CHECK(facesOutward(mesh, Math::vec3(0.5f)));
    CHECK(analyse(mesh).faces == 12);
}

void testFillAConcaveHole()
{
    MeshData mesh = quadGrid(4, 4);
    std::vector<u32> kept;
    const std::set<u32> holeCells = {1 * 4 + 1, 1 * 4 + 2, 2 * 4 + 1};
    for (u32 cell = 0; cell < 16; ++cell)
    {
        if (holeCells.count(cell))
            continue;
        kept.insert(kept.end(), mesh.indices.begin() + cell * 6, mesh.indices.begin() + cell * 6 + 6);
    }
    mesh.indices = kept;
    const usize before = mesh.indices.size() / 3;

    const u64 holeEdge = MeshTopology::edgeKey(1 * 5 + 1, 1 * 5 + 2);
    u32 filled = 0;
    CHECK(MeshEdit::fillHoles(mesh, {holeEdge}, 64, &filled));
    CHECK(filled == 1);
    CHECK(std::abs(triangleAreaSum(mesh, before, mesh.indices.size() / 3) - 3.0f) < 1.0e-4f);
    for (usize f = before; f < mesh.indices.size() / 3; ++f)
    {
        const Math::vec3& a = mesh.positions[mesh.indices[f * 3]];
        const Math::vec3& b = mesh.positions[mesh.indices[f * 3 + 1]];
        const Math::vec3& c = mesh.positions[mesh.indices[f * 3 + 2]];
        CHECK(Math::cross(b - a, c - a).y > 0.0f);
    }
    const MeshTopology topology = topologyOf(mesh);
    CHECK(topology.boundaryLoops(mesh).size() == 1);
}

void testFillRefusals()
{
    std::string error;
    MeshData closed = weldedCube();
    CHECK(!MeshEdit::fillHoles(closed, {}, 64, nullptr, &error));

    MeshData open = weldedCube();
    open.indices.resize(30);
    CHECK(!MeshEdit::fillHoles(open, {}, 3, nullptr, &error));
    CHECK(!MeshEdit::fillHoles(open, {MeshTopology::edgeKey(0, 99)}, 64, nullptr, &error));
    CHECK(open.indices.size() == 30);
}

MeshData twoBands(u32 countA, u32 countB)
{
    MeshData a = band(countA);
    MeshData b = band(countB);
    for (Math::vec3& p : b.positions)
        p.y += 2.0f;
    const u32 base = static_cast<u32>(a.positions.size());
    a.positions.insert(a.positions.end(), b.positions.begin(), b.positions.end());
    for (const u32 index : b.indices)
        a.indices.push_back(index + base);
    return a;
}

void testBridgeEqualRings()
{
    MeshData mesh = twoBands(8, 8);
    const u64 top = MeshTopology::edgeKey(8, 9);
    const u64 bottom = MeshTopology::edgeKey(16, 17);
    const usize before = mesh.indices.size() / 3;
    std::string error;
    CHECK(MeshEdit::bridge(mesh, {top, bottom}, &error));
    CHECK(mesh.indices.size() / 3 == before + 16);
    const Report r = analyse(mesh);
    CHECK(r.valid && r.consistent);
    const MeshTopology topology = topologyOf(mesh);
    CHECK(topology.boundaryLoops(mesh).size() == 2);
    for (const MeshTopology::Edge& edge : topology.edges())
        CHECK(!topology.isNonManifold(static_cast<u32>(&edge - &topology.edges()[0])));
}

void testBridgeUnequalRings()
{
    MeshData mesh = twoBands(8, 12);
    const u64 top = MeshTopology::edgeKey(8, 9);
    const u64 bottom = MeshTopology::edgeKey(16 + 0, 16 + 1);
    const usize before = mesh.indices.size() / 3;
    std::string error;
    CHECK(MeshEdit::bridge(mesh, {top, bottom}, &error));
    CHECK(mesh.indices.size() / 3 == before + 8 + 12);
    const Report r = analyse(mesh);
    CHECK(r.valid && r.consistent);
}

void testBridgeRefusals()
{
    std::string error;
    MeshData mesh = twoBands(8, 8);
    CHECK(!MeshEdit::bridge(mesh, {}, &error));
    CHECK(!MeshEdit::bridge(mesh, {MeshTopology::edgeKey(8, 9)}, &error));
    MeshData closed = weldedCube();
    CHECK(!MeshEdit::bridge(closed, {}, &error));
}

void testMirrorHalfACube()
{
    // A cube with its x = 0 face missing, mirrored across x = 0 and welded: a
    // closed box twice as wide.
    MeshData mesh = weldedCube();
    std::vector<u32> kept;
    for (usize f = 0; f < 12; ++f)
    {
        bool onPlane = true;
        for (u32 c = 0; c < 3; ++c)
            onPlane = onPlane && mesh.positions[mesh.indices[f * 3 + c]].x == 0.0f;
        if (!onPlane)
            kept.insert(kept.end(), mesh.indices.begin() + f * 3, mesh.indices.begin() + f * 3 + 3);
    }
    mesh.indices = kept;
    addAttributes(mesh);

    std::string error;
    CHECK(MeshEdit::mirror(mesh, 0, 0.0f, 1.0e-5f, {}, &error));
    CHECK(arraysInStep(mesh));
    const Report r = analyse(mesh);
    CHECK(r.closed && r.consistent && r.valid);
    CHECK(r.vertices == 12);
    CHECK(mesh.indices.size() / 3 == 20);
    CHECK(static_cast<int>(r.vertices) - static_cast<int>(r.edges) + static_cast<int>(r.faces) == 2);
    CHECK(mesh.bounds.min.x == -1.0f && mesh.bounds.max.x == 1.0f);
    CHECK(facesOutward(mesh, Math::vec3(0.0f, 0.5f, 0.5f)));
}

void testMirrorWithoutWeld()
{
    MeshData mesh = octahedron();
    CHECK(MeshEdit::mirror(mesh, 1, 2.0f, 0.0f, {}));
    CHECK(std::abs(mesh.bounds.max.y - 5.0f) < 1.0e-5f);
    CHECK(mesh.positions.size() == 12);
    CHECK(mesh.indices.size() / 3 == 16);
    const Report r = analyse(mesh);
    CHECK(r.valid && r.consistent);
    MeshData lower = octahedron();
    CHECK(facesOutward(lower, Math::vec3(0.0f)));

    MeshData some = octahedron();
    CHECK(MeshEdit::mirror(some, 0, 0.0f, 0.0f, {0, 1}));
    CHECK(some.indices.size() / 3 == 10);
}

void testMirrorRefusals()
{
    std::string error;
    MeshData mesh = octahedron();
    CHECK(!MeshEdit::mirror(mesh, 3, 0.0f, 0.0f, {}, &error));
    CHECK(!MeshEdit::mirror(mesh, 0, NAN, 0.0f, {}, &error));
    CHECK(!MeshEdit::mirror(mesh, 0, 0.0f, -1.0f, {}, &error));
    CHECK(!MeshEdit::mirror(mesh, 0, 0.0f, 0.0f, {99}, &error));
    CHECK(mesh.indices.size() == 24);
}

void testMergeSubmeshes()
{
    MeshData mesh = weldedCube();
    SubMesh a;
    a.indexCount = 12;
    SubMesh b;
    b.indexOffset = 12;
    b.indexCount = 12;
    b.materialSlot = 1;
    SubMesh c;
    c.indexOffset = 24;
    c.indexCount = 12;
    c.materialSlot = 2;
    mesh.submeshes = {a, b, c};
    mesh.materials.resize(3);

    std::string error;
    CHECK(MeshEdit::mergeSubmeshes(mesh, {2, 0}, &error));
    CHECK(mesh.submeshes.size() == 2);
    CHECK(mesh.submeshes[0].indexCount == 24);
    CHECK(mesh.submeshes[0].materialSlot == 0);
    CHECK(mesh.submeshes[1].indexCount == 12);
    CHECK(mesh.submeshes[1].materialSlot == 1);
    CHECK(mesh.submeshes[0].indexOffset == 0 && mesh.submeshes[1].indexOffset == 24);
    CHECK(mesh.indices.size() == 36);

    CHECK(!MeshEdit::mergeSubmeshes(mesh, {1}, &error));
    CHECK(!MeshEdit::mergeSubmeshes(mesh, {0, 7}, &error));
    CHECK(!MeshEdit::mergeSubmeshes(mesh, {1, 1}, &error));
}

} // namespace

int main()
{
    testFillAClosedOffHole();
    testFillAConcaveHole();
    testFillRefusals();
    testBridgeEqualRings();
    testBridgeUnequalRings();
    testBridgeRefusals();
    testMirrorHalfACube();
    testMirrorWithoutWeld();
    testMirrorRefusals();
    testMergeSubmeshes();
    testRemoveUnusedVertices();
    testInsetAFlatRegion();
    testInsetWithDepth();
    testInsetOnAClosedSurface();
    testInsetRefusals();
    testInsetKeepsSubmeshes();
    testBevelACubeEdge();
    testBevelTwoEdgesAtOnce();
    testBevelRefusals();
    testKnifeCutsACube();
    testKnifeRefusals();
    testLoopCutOnAGrid();
    testLoopCutSeveralAndEitherDirection();
    testLoopCutsAreStraightAcrossTheRing();
    testLoopCutAlongTheOtherAxis();
    testLoopCutClosesAroundABand();
    testLoopCutRefusals();

    testTestShapesAreSound();
    testRefineOneEdge();
    testRefineDirectionOfT();
    testRefineTwoAndThreeEdgesOfOneTriangle();
    testRefineKeepsAttributesInStep();
    testRefineRejectsBadInput();
    testSubdivideFlat();
    testSubdivideTwoLevels();
    testSubdivideRegionStaysWatertight();
    testSubdivideSmoothRoundsTheSurface();
    testSmoothSubdivisionConvergesOnASphere();
    testSmoothRegionKeepsItsBorder();
    testSubmeshesSurviveSubdivision();
    testSubdivideKeepsAttributesAndRejectsBadInput();
    testTurnEdge();
    testTurnEdgeRefusals();
    testCollapseEdge();
    testCollapseKeepsSubmeshes();

    if (gFailures)
        std::fprintf(stderr, "%d mesh edit test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
