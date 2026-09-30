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

// ------------------------------------------------------------------ shapes

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
    const glm::vec3 v[12] = {{-1, t, 0}, {1, t, 0},  {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t},
                             {0, -1, -t}, {0, 1, -t}, {t, 0, -1},  {t, 0, 1},  {-t, 0, -1}, {-t, 0, 1}};
    for (const glm::vec3& p : v)
        mesh.positions.push_back(glm::normalize(p));
    mesh.indices = {0, 11, 5, 0, 5, 1,  0, 1, 7,  0, 7, 10, 0, 10, 11, 1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8,
                    3, 9, 4,  3, 4, 2,  3, 2, 6,  3, 6, 8,  3, 8, 9,   4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7,  9, 8, 1};
    return mesh;
}

// The unit cube with shared corners: 8 vertices, 12 triangles.
MeshData weldedCube()
{
    MeshData mesh;
    mesh.positions = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    const u32 faces[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}};
    for (const auto& q : faces)
        mesh.indices.insert(mesh.indices.end(), {q[0], q[1], q[2], q[0], q[2], q[3]});
    return mesh;
}

// The same cube the way the engine builds one: each face owns its corners.
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
    mesh.normals.assign(mesh.positions.size(), glm::vec3(0, 1, 0));
    mesh.uvs.clear();
    mesh.colors.clear();
    for (const glm::vec3& p : mesh.positions)
    {
        mesh.uvs.push_back(glm::vec2(p.x, p.z));
        mesh.colors.push_back(0xFF000000u | (static_cast<u32>(glm::clamp(p.x, 0.0f, 1.0f) * 255.0f) & 0xFF));
    }
}

// ------------------------------------------------------------------ checks

struct Report
{
    usize vertices = 0; // canonical
    usize edges = 0;
    usize faces = 0;
    bool closed = true;     // every edge has exactly two triangles
    bool consistent = true; // each directed edge appears once: all triangles agree on "out"
    bool valid = true;      // indices in range, no collapsed triangle
};

Report analyse(const MeshData& mesh)
{
    Report report;
    MeshTopology topology;
    topology.build(mesh);

    std::set<u32> points;
    for (u32 v = 0; v < mesh.positions.size(); ++v)
        points.insert(topology.canonical(v));
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

// Triangles of a convex shape face away from its centre.
bool facesOutward(const MeshData& mesh, const glm::vec3& centre)
{
    for (usize f = 0; f < mesh.indices.size() / 3; ++f)
    {
        const glm::vec3& a = mesh.positions[mesh.indices[f * 3]];
        const glm::vec3& b = mesh.positions[mesh.indices[f * 3 + 1]];
        const glm::vec3& c = mesh.positions[mesh.indices[f * 3 + 2]];
        if (glm::dot(glm::cross(b - a, c - a), (a + b + c) / 3.0f - centre) <= 0.0f)
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

u64 edgeBetween(const MeshData& mesh, const glm::vec3& a, const glm::vec3& b)
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

// ------------------------------------------------------------------- tests

void testTestShapesAreSound()
{
    for (const MeshData& mesh : {octahedron(), icosahedron(), weldedCube(), splitCube()})
    {
        const Report r = analyse(mesh);
        CHECK(r.closed && r.consistent && r.valid);
        CHECK(static_cast<int>(r.vertices) - static_cast<int>(r.edges) + static_cast<int>(r.faces) == 2);
        CHECK(facesOutward(mesh, glm::vec3(0.0f)) || facesOutward(mesh, glm::vec3(0.5f)));
    }
}

void testRefineOneEdge()
{
    MeshData mesh = weldedCube();
    const u64 key = edgeBetween(mesh, {0, 0, 0}, {1, 0, 0});
    MeshEdit::RefineResult result;
    std::string error;
    CHECK(MeshEdit::refineEdges(mesh, {{key, 0.5f}}, &result, &error));

    // The two triangles on the edge are cut in two: 12 -> 14, one new vertex.
    CHECK(mesh.indices.size() / 3 == 14);
    CHECK(mesh.positions.size() == 9);
    CHECK(result.midpoints.size() == 1);
    CHECK(mesh.positions[result.midpoints[0].vertex] == glm::vec3(0.5f, 0, 0));
    CHECK(result.origin.size() == 14);

    const Report r = analyse(mesh);
    CHECK(r.closed && r.consistent && r.valid);
    CHECK(facesOutward(mesh, glm::vec3(0.5f)));
}

void testRefineDirectionOfT()
{
    // t runs from the lower canonical id to the higher one whichever way the
    // triangles walk the edge.
    MeshData mesh = weldedCube();
    const u64 key = edgeBetween(mesh, {0, 0, 0}, {1, 0, 0}); // ids 0 and 1
    MeshEdit::RefineResult result;
    CHECK(MeshEdit::refineEdges(mesh, {{key, 0.25f}}, &result));
    CHECK(mesh.positions[result.midpoints[0].vertex] == glm::vec3(0.25f, 0, 0));
}

void testRefineTwoAndThreeEdgesOfOneTriangle()
{
    MeshData mesh = octahedron();
    // Two edges of triangle (+x,+y,+z): x-y and y-z.
    const u64 a = edgeBetween(mesh, {1, 0, 0}, {0, 1, 0});
    const u64 b = edgeBetween(mesh, {0, 1, 0}, {0, 0, 1});
    CHECK(MeshEdit::refineEdges(mesh, {{a, 0.5f}, {b, 0.5f}}));
    Report r = analyse(mesh);
    CHECK(r.closed && r.consistent && r.valid);
    CHECK(facesOutward(mesh, glm::vec3(0.0f)));

    MeshData all = octahedron();
    std::vector<MeshEdit::EdgeSplit> splits;
    const MeshTopology topology = topologyOf(all);
    for (const MeshTopology::Edge& edge : topology.edges())
        splits.push_back({MeshTopology::edgeKey(edge.a, edge.b), 0.5f});
    CHECK(MeshEdit::refineEdges(all, splits));
    CHECK(all.indices.size() / 3 == 32); // every triangle became four
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
    CHECK(std::abs(glm::length(mesh.normals[m]) - 1.0f) < 1.0e-4f);
    // Half way between a colour of 0 and 255 is about 128.
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
    CHECK(mesh.indices.size() == 36); // untouched
}

void testSubdivideFlat()
{
    for (MeshData mesh : {octahedron(), weldedCube(), splitCube()})
    {
        const glm::vec3 low = [&] { glm::vec3 m(1e9f); for (auto& p : mesh.positions) m = glm::min(m, p); return m; }();
        const glm::vec3 high = [&] { glm::vec3 m(-1e9f); for (auto& p : mesh.positions) m = glm::max(m, p); return m; }();
        const usize triangles = mesh.indices.size() / 3;

        std::string error;
        CHECK(MeshEdit::subdivide(mesh, {}, 1, false, &error));
        CHECK(mesh.indices.size() / 3 == triangles * 4);
        const Report r = analyse(mesh);
        CHECK(r.closed && r.consistent && r.valid);
        CHECK(static_cast<int>(r.vertices) - static_cast<int>(r.edges) + static_cast<int>(r.faces) == 2);

        // Flat subdivision does not move the surface.
        glm::vec3 low2(1e9f);
        glm::vec3 high2(-1e9f);
        for (const glm::vec3& p : mesh.positions)
        {
            low2 = glm::min(low2, p);
            high2 = glm::max(high2, p);
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
    // One face of a cube: its neighbours must be cut along the shared edges or a
    // T-junction is left behind.
    MeshData mesh = weldedCube();
    CHECK(MeshEdit::subdivide(mesh, {0}, 1, false));
    const Report r = analyse(mesh);
    CHECK(r.closed && r.consistent && r.valid);
    CHECK(mesh.indices.size() / 3 > 12 + 3); // the face became 4; its neighbours were cut too
    CHECK(facesOutward(mesh, glm::vec3(0.5f)));

    // A second level on the same region, then everything else untouched.
    MeshData twice = weldedCube();
    CHECK(MeshEdit::subdivide(twice, {0, 1}, 2, false));
    const Report r2 = analyse(twice);
    CHECK(r2.closed && r2.consistent && r2.valid);
}

void testSubdivideSmoothRoundsTheSurface()
{
    // Loop subdivision of an octahedron pulls its corners in and its edge
    // points out, toward a sphere: the corners end up strictly inside the
    // original ones, and the shape keeps its symmetry.
    MeshData mesh = octahedron();
    CHECK(MeshEdit::subdivide(mesh, {}, 1, true));
    const Report r = analyse(mesh);
    CHECK(r.closed && r.consistent && r.valid);
    CHECK(mesh.bounds.max.x < 1.0f && mesh.bounds.max.x > 0.5f);
    CHECK(std::abs(mesh.bounds.max.x + mesh.bounds.min.x) < 1.0e-5f);
    CHECK(std::abs(mesh.bounds.max.x - mesh.bounds.max.y) < 1.0e-5f);
    CHECK(facesOutward(mesh, glm::vec3(0.0f)));
}

// The sharpest turn between two neighbouring triangles, in radians: a measure of
// how faceted the surface still is.
f32 sharpestFold(const MeshData& mesh)
{
    const MeshTopology topology = topologyOf(mesh);
    auto normal = [&](u32 face)
    {
        const glm::vec3& a = mesh.positions[mesh.indices[face * 3]];
        const glm::vec3& b = mesh.positions[mesh.indices[face * 3 + 1]];
        const glm::vec3& c = mesh.positions[mesh.indices[face * 3 + 2]];
        return glm::normalize(glm::cross(b - a, c - a));
    };
    f32 sharpest = 0.0f;
    for (const MeshTopology::Edge& edge : topology.edges())
    {
        if (edge.faces.size() != 2)
            continue;
        sharpest = std::max(sharpest, std::acos(glm::clamp(glm::dot(normal(edge.faces[0]), normal(edge.faces[1])), -1.0f, 1.0f)));
    }
    return sharpest;
}

void testSmoothSubdivisionConvergesOnASphere()
{
    // Each round of Loop subdivision makes the icosahedron smoother.
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
    CHECK(facesOutward(mesh, glm::vec3(0.0f)));

    // Flat subdivision adds triangles but never smooths anything.
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

    // A region inside one submesh only grows that one.
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
    // A flat quad of two triangles: turning the diagonal swaps it for the other.
    MeshData mesh;
    mesh.positions = {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1}};
    mesh.indices = {0, 2, 1, 0, 3, 2}; // CCW seen from +Y... either way: both agree
    // Face up: make the triangles face +Y.
    mesh.indices = {0, 3, 2, 0, 2, 1};
    const u64 diagonal = edgeBetween(mesh, {0, 0, 0}, {1, 0, 1});

    std::string error;
    CHECK(MeshEdit::turnEdge(mesh, diagonal, &error));
    CHECK(mesh.indices.size() == 6);
    const MeshTopology after = topologyOf(mesh);
    const u64 other = edgeBetween(mesh, {1, 0, 0}, {0, 0, 1});
    CHECK(after.findEdge(static_cast<u32>(other >> 32), static_cast<u32>(other & 0xFFFFFFFFu)) >= 0);
    CHECK(after.findEdge(static_cast<u32>(diagonal >> 32), static_cast<u32>(diagonal & 0xFFFFFFFFu)) < 0);
    // Both triangles still face +Y.
    for (usize f = 0; f < 2; ++f)
    {
        const glm::vec3& a = mesh.positions[mesh.indices[f * 3]];
        const glm::vec3& b = mesh.positions[mesh.indices[f * 3 + 1]];
        const glm::vec3& c = mesh.positions[mesh.indices[f * 3 + 2]];
        CHECK(glm::cross(b - a, c - a).y > 0.0f);
    }

    // Turning it back restores the original diagonal.
    CHECK(MeshEdit::turnEdge(mesh, other, &error));
    const MeshTopology back = topologyOf(mesh);
    CHECK(back.findEdge(static_cast<u32>(diagonal >> 32), static_cast<u32>(diagonal & 0xFFFFFFFFu)) >= 0);
}

void testTurnEdgeRefusals()
{
    std::string error;

    // A concave quad: flipping would fold the surface.
    MeshData dart;
    dart.positions = {{0, 0, 0}, {2, 0, 0}, {1, 0, 0.3f}, {1, 0, 2}};
    // Triangles (0,3,2) and (0,2,1) share edge 0-2; the quad 0,1,2,3 is concave at 2.
    dart.indices = {0, 3, 2, 0, 2, 1};
    const u64 shared = edgeBetween(dart, {0, 0, 0}, {1, 0, 0.3f});
    CHECK(!MeshEdit::turnEdge(dart, shared, &error));
    CHECK(error.find("convex") != std::string::npos);

    // A boundary edge has one triangle.
    MeshData one;
    one.positions = {{0, 0, 0}, {1, 0, 0}, {0, 0, 1}};
    one.indices = {0, 2, 1};
    CHECK(!MeshEdit::turnEdge(one, MeshTopology::edgeKey(0, 1), &error));

    // A seam (triangles that do not share vertex indices along it) is refused.
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
    // The two triangles on the edge are gone.
    CHECK(mesh.indices.size() / 3 == 10);
    // Both ends now stand at the middle of where they were.
    CHECK(mesh.positions[0] == glm::vec3(0.5f, 0, 0));
    CHECK(mesh.positions[1] == glm::vec3(0.5f, 0, 0));

    const Report r = analyse(mesh);
    CHECK(r.valid);
    CHECK(r.closed);
    CHECK(static_cast<int>(r.vertices) - static_cast<int>(r.edges) + static_cast<int>(r.faces) == 2);

    MeshData again = weldedCube();
    CHECK(MeshEdit::collapseEdge(again, key, 0.0f, &error));
    CHECK(again.positions[1] == glm::vec3(0, 0, 0));
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

} // namespace

int main()
{
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
