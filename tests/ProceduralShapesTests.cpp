#include "PCH.h"

#include "ProceduralShapes.h"
#include "mesh/MeshTopology.h"

#include <cstdio>
#include <map>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "ProceduralShapesTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

Math::vec3 centerOf(const MeshData& mesh)
{
    return mesh.bounds.center();
}

// Every triangle of a convex shape must face away from its centre, and every
// index must land inside the vertex arrays. That is the whole contract of the
// winding: a shape built inside out renders black and culls from the outside.
void checkOutwardFacing(const MeshData& mesh, int line)
{
    check(!mesh.positions.empty() && !mesh.indices.empty(), "has geometry", line);
    check(mesh.indices.size() % 3 == 0, "whole triangles", line);
    check(mesh.normals.size() == mesh.positions.size(), "normals per vertex", line);
    check(mesh.uvs.size() == mesh.positions.size(), "uvs per vertex", line);

    const Math::vec3 center = centerOf(mesh);
    u32 inward = 0;
    u32 degenerate = 0;
    u32 outOfRange = 0;
    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const u32 a = mesh.indices[i];
        const u32 b = mesh.indices[i + 1];
        const u32 c = mesh.indices[i + 2];
        if (a >= mesh.positions.size() || b >= mesh.positions.size() || c >= mesh.positions.size())
        {
            ++outOfRange;
            continue;
        }
        const Math::vec3 normal = Math::cross(mesh.positions[b] - mesh.positions[a],
                                            mesh.positions[c] - mesh.positions[a]);
        if (Math::length(normal) < 1.0e-9f)
        {
            ++degenerate;
            continue;
        }
        const Math::vec3 centroid = (mesh.positions[a] + mesh.positions[b] + mesh.positions[c]) / 3.0f;
        if (Math::dot(normal, centroid - center) <= 0.0f)
            ++inward;
    }
    check(outOfRange == 0, "no index out of range", line);
    check(degenerate == 0, "no degenerate triangle", line);
    check(inward == 0, "every triangle faces outward", line);

    // Vertex normals agree with the faces: none points back into the shape.
    u32 badNormals = 0;
    for (usize i = 0; i < mesh.positions.size(); ++i)
    {
        if (std::abs(Math::length(mesh.normals[i]) - 1.0f) > 1.0e-3f)
            ++badNormals;
        else if (Math::dot(mesh.normals[i], mesh.positions[i] - center) < -1.0e-4f)
            ++badNormals;
    }
    check(badNormals == 0, "vertex normals are unit and outward", line);
}

#define CHECK_OUTWARD(mesh) checkOutwardFacing((mesh), __LINE__)

void testLatheSphere()
{
    // A sphere of radius 1 as a profile, pole to pole, rising.
    LatheParams params;
    params.slices = 16;
    for (int i = 0; i <= 8; ++i)
    {
        const f32 angle = 3.14159265f * static_cast<f32>(i) / 8.0f;
        params.profile.push_back(Math::vec2(std::sin(angle), -std::cos(angle)));
    }

    MeshData mesh;
    std::string error;
    CHECK(buildLathe(params, mesh, &error));
    CHECK_OUTWARD(mesh);
    CHECK(mesh.submeshes.size() == 1);
    CHECK(mesh.submeshes[0].indexCount == mesh.indices.size());
    CHECK(std::abs(mesh.bounds.max.y - 1.0f) < 1.0e-4f);
    CHECK(std::abs(mesh.bounds.min.y + 1.0f) < 1.0e-4f);
    // Pole to pole has no open end, so no caps were added.
    CHECK(mesh.positions.size() == 9 * 17);
}

void testLatheDescendingProfileStillFacesOutward()
{
    LatheParams params;
    params.slices = 12;
    // The same sphere walked from the top down.
    for (int i = 8; i >= 0; --i)
    {
        const f32 angle = 3.14159265f * static_cast<f32>(i) / 8.0f;
        params.profile.push_back(Math::vec2(std::sin(angle), -std::cos(angle)));
    }

    MeshData mesh;
    CHECK(buildLathe(params, mesh));
    CHECK_OUTWARD(mesh);
}

void testLatheCylinderCaps()
{
    LatheParams params;
    params.slices = 10;
    params.profile = {Math::vec2(1.0f, 0.0f), Math::vec2(1.0f, 2.0f)};

    MeshData capped;
    CHECK(buildLathe(params, capped));
    CHECK_OUTWARD(capped);
    // 2 rings of 11 + two caps of (1 centre + 10 rim).
    CHECK(capped.positions.size() == 22 + 22);

    params.capStart = false;
    params.capEnd = false;
    MeshData open;
    CHECK(buildLathe(params, open));
    CHECK(open.positions.size() == 22);
    CHECK(open.indices.size() == 10 * 2 * 3);
}

void testLatheRejectsBadInput()
{
    MeshData mesh;
    std::string error;

    LatheParams tooShort;
    tooShort.profile = {Math::vec2(1.0f, 0.0f)};
    CHECK(!buildLathe(tooShort, mesh, &error));
    CHECK(!error.empty());

    LatheParams negative;
    negative.profile = {Math::vec2(-1.0f, 0.0f), Math::vec2(1.0f, 1.0f)};
    CHECK(!buildLathe(negative, mesh));

    LatheParams flat;
    flat.slices = 2;
    flat.profile = {Math::vec2(1.0f, 0.0f), Math::vec2(1.0f, 1.0f)};
    CHECK(!buildLathe(flat, mesh));

    LatheParams notANumber;
    notANumber.profile = {Math::vec2(1.0f, 0.0f), Math::vec2(1.0f, NAN)};
    CHECK(!buildLathe(notANumber, mesh));
}

LoftParams capsuleLoft(s32 axis)
{
    LoftParams params;
    params.axis = axis;
    params.segments = 16;
    params.sections = {{0.0f, 0.0f, 0.0f}, {0.5f, 1.0f, 1.0f}, {2.0f, 1.0f, 1.0f},
                       {2.5f, 0.0f, 0.0f}};
    return params;
}

void testLoftOnEveryAxis()
{
    for (s32 axis = 0; axis < 3; ++axis)
    {
        MeshData mesh;
        std::string error;
        CHECK(buildLoft(capsuleLoft(axis), mesh, &error));
        CHECK_OUTWARD(mesh);

        // It runs from 0 to 2.5 along the chosen axis and is 1 across in the others.
        CHECK(std::abs(mesh.bounds.min[axis]) < 1.0e-4f);
        CHECK(std::abs(mesh.bounds.max[axis] - 2.5f) < 1.0e-4f);
        for (s32 other = 0; other < 3; ++other)
        {
            if (other == axis)
                continue;
            CHECK(std::abs(mesh.bounds.max[other] - 0.5f) < 1.0e-3f);
            CHECK(std::abs(mesh.bounds.min[other] + 0.5f) < 1.0e-3f);
        }
    }
}

void testLoftOffsetsAndCaps()
{
    LoftParams params;
    params.axis = 2;
    params.segments = 12;
    // A tapering boom that also rises: the end section is smaller and higher.
    LoftSection start;
    start.at = 0.0f;
    start.width = 2.0f;
    start.height = 2.0f;
    LoftSection end;
    end.at = 4.0f;
    end.width = 0.5f;
    end.height = 0.5f;
    end.offset = Math::vec2(0.0f, 1.0f);
    params.sections = {start, end};

    MeshData mesh;
    CHECK(buildLoft(params, mesh));
    // Both ends are open rings, so both get a cap.
    CHECK(mesh.positions.size() == 2 * 13 + 2 * 13);
    CHECK(std::abs(mesh.bounds.max.y - 1.25f) < 1.0e-3f);

    // The far end sits around y = 1.
    f32 lowestAtEnd = 1.0e9f;
    for (const Math::vec3& position : mesh.positions)
        if (std::abs(position.z - 4.0f) < 1.0e-4f)
            lowestAtEnd = std::min(lowestAtEnd, position.y);
    CHECK(std::abs(lowestAtEnd - 0.75f) < 1.0e-3f);
}

void testLoftSuperellipseIsBoxier()
{
    LoftParams round = capsuleLoft(2);
    LoftParams boxy = round;
    for (LoftSection& section : boxy.sections)
        section.exponent = 6.0f;

    MeshData roundMesh;
    MeshData boxyMesh;
    CHECK(buildLoft(round, roundMesh));
    CHECK(buildLoft(boxy, boxyMesh));

    // On the 45-degree diagonal a circle sits 0.354 from each axis; a squarer
    // section pushes that corner further out.
    auto farthestCorner = [](const MeshData& mesh)
    {
        f32 best = 0.0f;
        for (const Math::vec3& position : mesh.positions)
            best = std::max(best, std::min(std::abs(position.x), std::abs(position.y)));
        return best;
    };
    CHECK(farthestCorner(boxyMesh) > farthestCorner(roundMesh) + 0.05f);
}

void testLoftRejectsBadInput()
{
    MeshData mesh;
    std::string error;

    LoftParams one;
    one.sections = {{0.0f, 1.0f, 1.0f}};
    CHECK(!buildLoft(one, mesh, &error));

    LoftParams unordered = capsuleLoft(2);
    std::swap(unordered.sections[1], unordered.sections[2]);
    CHECK(!buildLoft(unordered, mesh, &error));
    CHECK(error.find("increasing") != std::string::npos);

    LoftParams badAxis = capsuleLoft(2);
    badAxis.axis = 3;
    CHECK(!buildLoft(badAxis, mesh));

    LoftParams negative = capsuleLoft(2);
    negative.sections[1].width = -1.0f;
    CHECK(!buildLoft(negative, mesh));

    LoftParams badExponent = capsuleLoft(2);
    badExponent.sections[1].exponent = 100.0f;
    CHECK(!buildLoft(badExponent, mesh));
}

// ------------------------------------------------------------- solids

f32 signedVolume(const MeshData& mesh)
{
    f32 volume = 0.0f;
    for (usize f = 0; f + 2 < mesh.indices.size(); f += 3)
    {
        const Math::vec3& a = mesh.positions[mesh.indices[f]];
        const Math::vec3& b = mesh.positions[mesh.indices[f + 1]];
        const Math::vec3& c = mesh.positions[mesh.indices[f + 2]];
        volume += Math::dot(a, Math::cross(b, c)) / 6.0f;
    }
    return volume;
}

// A closed solid: every edge shared by exactly two triangles that walk it in
// opposite directions, facing out (positive volume), with flat normals that agree
// with their triangle. `volume` is what it should enclose.
void checkSolid(const MeshData& mesh, f32 volume, f32 tolerance, int line)
{
    check(!mesh.positions.empty() && mesh.indices.size() % 3 == 0, "has geometry", line);
    check(mesh.normals.size() == mesh.positions.size() && mesh.uvs.size() == mesh.positions.size(),
          "normals and uvs per vertex", line);
    check(mesh.submeshes.size() == 1 && mesh.submeshes[0].indexCount == mesh.indices.size(), "one submesh", line);

    MeshTopology topology;
    topology.build(mesh);
    u32 notClosed = 0;
    for (const MeshTopology::Edge& edge : topology.edges())
        if (edge.faces.size() != 2)
            ++notClosed;
    check(notClosed == 0, "every edge has two triangles", line);

    std::map<std::pair<u32, u32>, int> directed;
    for (usize f = 0; f + 2 < mesh.indices.size(); f += 3)
        for (usize c = 0; c < 3; ++c)
            ++directed[{topology.canonical(mesh.indices[f + c]), topology.canonical(mesh.indices[f + (c + 1) % 3])}];
    u32 conflicting = 0;
    for (const auto& entry : directed)
        if (entry.second != 1)
            ++conflicting;
    check(conflicting == 0, "triangles agree on which way is out", line);

    check(std::abs(signedVolume(mesh) - volume) <= tolerance * volume, "encloses the expected volume", line);

    u32 badNormals = 0;
    for (usize f = 0; f + 2 < mesh.indices.size(); f += 3)
    {
        const Math::vec3& a = mesh.positions[mesh.indices[f]];
        const Math::vec3& b = mesh.positions[mesh.indices[f + 1]];
        const Math::vec3& c = mesh.positions[mesh.indices[f + 2]];
        const Math::vec3 geometric = Math::normalize(Math::cross(b - a, c - a));
        for (usize k = 0; k < 3; ++k)
            if (Math::dot(mesh.normals[mesh.indices[f + k]], geometric) < 0.99f)
                ++badNormals;
    }
    check(badNormals == 0, "flat normals agree with their triangle", line);
}

#define CHECK_SOLID(mesh, volume, tolerance) checkSolid((mesh), (volume), (tolerance), __LINE__)

f32 polygonArea(u32 sides, f32 radius)
{
    return 0.5f * static_cast<f32>(sides) * radius * radius * std::sin(2.0f * 3.14159265f / static_cast<f32>(sides));
}

void testPrism()
{
    PrismParams params;
    params.sides = 6;
    params.radius = 1.0f;
    params.height = 2.0f;
    MeshData mesh;
    std::string error;
    CHECK(buildPrism(params, mesh, &error));
    CHECK_SOLID(mesh, polygonArea(6, 1.0f) * 2.0f, 1.0e-3f);
    CHECK(std::abs(mesh.bounds.min.y + 1.0f) < 1.0e-5f && std::abs(mesh.bounds.max.y - 1.0f) < 1.0e-5f);

    PrismParams bad;
    bad.sides = 2;
    CHECK(!buildPrism(bad, mesh, &error));
    bad.sides = 6;
    bad.height = 0.0f;
    CHECK(!buildPrism(bad, mesh, &error));
}

void testTube()
{
    TubeParams params;
    params.outerRadius = 1.0f;
    params.innerRadius = 0.5f;
    params.height = 2.0f;
    params.slices = 24;
    MeshData mesh;
    std::string error;
    CHECK(buildTube(params, mesh, &error));
    CHECK_SOLID(mesh, (polygonArea(24, 1.0f) - polygonArea(24, 0.5f)) * 2.0f, 1.0e-3f);
    CHECK(std::abs(mesh.bounds.min.y + 1.0f) < 1.0e-5f && std::abs(mesh.bounds.max.y - 1.0f) < 1.0e-5f);
    CHECK(std::abs(mesh.bounds.max.x - 1.0f) < 1.0e-5f);

    TubeParams bad = params;
    bad.innerRadius = 1.0f;
    CHECK(!buildTube(bad, mesh, &error));
    bad = params;
    bad.slices = 2;
    CHECK(!buildTube(bad, mesh, &error));
}

void testDisc()
{
    DiscParams params;
    params.radius = 2.0f;
    params.slices = 16;
    MeshData mesh;
    CHECK(buildDisc(params, mesh));
    CHECK(mesh.indices.size() == 16 * 3);
    f32 area = 0.0f;
    for (usize f = 0; f + 2 < mesh.indices.size(); f += 3)
    {
        const Math::vec3& a = mesh.positions[mesh.indices[f]];
        const Math::vec3& b = mesh.positions[mesh.indices[f + 1]];
        const Math::vec3& c = mesh.positions[mesh.indices[f + 2]];
        const Math::vec3 n = Math::cross(b - a, c - a);
        CHECK(n.y > 0.0f); // faces up
        area += 0.5f * Math::length(n);
    }
    CHECK(std::abs(area - polygonArea(16, 2.0f)) < 1.0e-3f);
    DiscParams bad;
    bad.radius = -1.0f;
    CHECK(!buildDisc(bad, mesh));
}

void testStairs()
{
    StairsParams params;
    params.steps = 5;
    params.width = 2.0f;
    params.stepHeight = 0.2f;
    params.stepDepth = 0.3f;
    MeshData mesh;
    std::string error;
    CHECK(buildStairs(params, mesh, &error));
    // Volume: width x sum over steps of (i + 1) * rise * run.
    CHECK_SOLID(mesh, 2.0f * 0.2f * 0.3f * (1 + 2 + 3 + 4 + 5), 1.0e-3f);
    CHECK(std::abs(mesh.bounds.min.x + 1.0f) < 1.0e-5f && std::abs(mesh.bounds.max.x - 1.0f) < 1.0e-5f);
    CHECK(std::abs(mesh.bounds.min.y + 0.5f) < 1.0e-5f && std::abs(mesh.bounds.max.y - 0.5f) < 1.0e-5f);
    CHECK(std::abs(mesh.bounds.min.z + 0.75f) < 1.0e-5f && std::abs(mesh.bounds.max.z - 0.75f) < 1.0e-5f);
    // The top is at the +Z end: that is where the stairs go up to.
    for (const Math::vec3& p : mesh.positions)
        if (p.y > 0.49f)
            CHECK(p.z > 0.4f);

    StairsParams bad;
    bad.steps = 0;
    CHECK(!buildStairs(bad, mesh, &error));
    bad.steps = 3;
    bad.stepDepth = 0.0f;
    CHECK(!buildStairs(bad, mesh, &error));
}

void testArch()
{
    ArchParams params;
    params.width = 2.0f;
    params.height = 2.5f;
    params.depth = 0.5f;
    params.thickness = 0.4f;
    params.segments = 16;
    MeshData mesh;
    std::string error;
    CHECK(buildArch(params, mesh, &error));

    // Outer minus inner, each a rectangle under a half polygon.
    const f32 spring = 1.5f;
    const f32 outer = 2.0f * spring + 0.5f * polygonArea(32, 1.0f);
    const f32 inner = 1.2f * spring + 0.5f * polygonArea(32, 0.6f);
    CHECK_SOLID(mesh, (outer - inner) * 0.5f, 2.0e-3f);
    CHECK(std::abs(mesh.bounds.min.x + 1.0f) < 1.0e-5f && std::abs(mesh.bounds.max.x - 1.0f) < 1.0e-5f);
    CHECK(std::abs(mesh.bounds.min.y + 1.25f) < 1.0e-5f && std::abs(mesh.bounds.max.y - 1.25f) < 1.0e-5f);
    CHECK(std::abs(mesh.bounds.min.z + 0.25f) < 1.0e-5f && std::abs(mesh.bounds.max.z - 0.25f) < 1.0e-5f);

    ArchParams bad = params;
    bad.thickness = 1.0f; // as wide as half the arch: no opening
    CHECK(!buildArch(bad, mesh, &error));
    bad = params;
    bad.height = 0.5f; // lower than the semicircle
    CHECK(!buildArch(bad, mesh, &error));
}

void testExtrusionOfAConcaveOutline()
{
    // An L, given clockwise: it is reversed, and still comes out a solid.
    ExtrusionParams params;
    params.depth = 2.0f;
    params.profile = {{0, 0}, {0, 3}, {1, 3}, {1, 1}, {3, 1}, {3, 0}};
    MeshData mesh;
    std::string error;
    CHECK(buildExtrusion(params, mesh, &error));
    // Area of the L: 3x1 + 1x2 = 5.
    // The mesh is not centred on X/Y (the outline is used as given), so measure volume
    // by the divergence theorem, which does not care.
    CHECK_SOLID(mesh, 5.0f * 2.0f, 1.0e-3f);
    CHECK(std::abs(mesh.bounds.min.z + 1.0f) < 1.0e-5f && std::abs(mesh.bounds.max.z - 1.0f) < 1.0e-5f);

    ExtrusionParams bad;
    bad.profile = {{0, 0}, {1, 0}};
    CHECK(!buildExtrusion(bad, mesh, &error));
    bad.profile = {{0, 0}, {1, 0}, {2, 0}}; // a line, no area
    CHECK(!buildExtrusion(bad, mesh, &error));
    bad.profile = {{0, 0}, {1, 0}, {0, 1}};
    bad.depth = 0.0f;
    CHECK(!buildExtrusion(bad, mesh, &error));
    bad.depth = 1.0f;
    bad.profile = {{0, 0}, {1, 0}, {NAN, 1}};
    CHECK(!buildExtrusion(bad, mesh, &error));
}

} // namespace

int main()
{
    testPrism();
    testTube();
    testDisc();
    testStairs();
    testArch();
    testExtrusionOfAConcaveOutline();
    testLatheSphere();
    testLatheDescendingProfileStillFacesOutward();
    testLatheCylinderCaps();
    testLatheRejectsBadInput();
    testLoftOnEveryAxis();
    testLoftOffsetsAndCaps();
    testLoftSuperellipseIsBoxier();
    testLoftRejectsBadInput();

    if (gFailures)
        std::fprintf(stderr, "%d procedural shape test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
