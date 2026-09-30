#include "PCH.h"

#include "ProceduralShapes.h"

#include <cstdio>

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

glm::vec3 centerOf(const MeshData& mesh)
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

    const glm::vec3 center = centerOf(mesh);
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
        const glm::vec3 normal = glm::cross(mesh.positions[b] - mesh.positions[a],
                                            mesh.positions[c] - mesh.positions[a]);
        if (glm::length(normal) < 1.0e-9f)
        {
            ++degenerate;
            continue;
        }
        const glm::vec3 centroid = (mesh.positions[a] + mesh.positions[b] + mesh.positions[c]) / 3.0f;
        if (glm::dot(normal, centroid - center) <= 0.0f)
            ++inward;
    }
    check(outOfRange == 0, "no index out of range", line);
    check(degenerate == 0, "no degenerate triangle", line);
    check(inward == 0, "every triangle faces outward", line);

    // Vertex normals agree with the faces: none points back into the shape.
    u32 badNormals = 0;
    for (usize i = 0; i < mesh.positions.size(); ++i)
    {
        if (std::abs(glm::length(mesh.normals[i]) - 1.0f) > 1.0e-3f)
            ++badNormals;
        else if (glm::dot(mesh.normals[i], mesh.positions[i] - center) < -1.0e-4f)
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
        params.profile.push_back(glm::vec2(std::sin(angle), -std::cos(angle)));
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
        params.profile.push_back(glm::vec2(std::sin(angle), -std::cos(angle)));
    }

    MeshData mesh;
    CHECK(buildLathe(params, mesh));
    CHECK_OUTWARD(mesh);
}

void testLatheCylinderCaps()
{
    LatheParams params;
    params.slices = 10;
    params.profile = {glm::vec2(1.0f, 0.0f), glm::vec2(1.0f, 2.0f)};

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
    tooShort.profile = {glm::vec2(1.0f, 0.0f)};
    CHECK(!buildLathe(tooShort, mesh, &error));
    CHECK(!error.empty());

    LatheParams negative;
    negative.profile = {glm::vec2(-1.0f, 0.0f), glm::vec2(1.0f, 1.0f)};
    CHECK(!buildLathe(negative, mesh));

    LatheParams flat;
    flat.slices = 2;
    flat.profile = {glm::vec2(1.0f, 0.0f), glm::vec2(1.0f, 1.0f)};
    CHECK(!buildLathe(flat, mesh));

    LatheParams notANumber;
    notANumber.profile = {glm::vec2(1.0f, 0.0f), glm::vec2(1.0f, NAN)};
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
    end.offset = glm::vec2(0.0f, 1.0f);
    params.sections = {start, end};

    MeshData mesh;
    CHECK(buildLoft(params, mesh));
    // Both ends are open rings, so both get a cap.
    CHECK(mesh.positions.size() == 2 * 13 + 2 * 13);
    CHECK(std::abs(mesh.bounds.max.y - 1.25f) < 1.0e-3f);

    // The far end sits around y = 1.
    f32 lowestAtEnd = 1.0e9f;
    for (const glm::vec3& position : mesh.positions)
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
        for (const glm::vec3& position : mesh.positions)
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

} // namespace

int main()
{
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
