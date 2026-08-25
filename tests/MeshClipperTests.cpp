#include "PCH.h"

#include "MeshClipper.h"

#include <cstdio>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "MeshClipperTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

MeshData makeQuad()
{
    MeshData mesh;
    mesh.positions.push_back(glm::vec3(-1.0f, 0.0f, -1.0f));
    mesh.positions.push_back(glm::vec3(1.0f, 0.0f, -1.0f));
    mesh.positions.push_back(glm::vec3(-1.0f, 0.0f, 1.0f));
    mesh.positions.push_back(glm::vec3(1.0f, 0.0f, 1.0f));
    mesh.normals.assign(4, glm::vec3(0.0f, 1.0f, 0.0f));
    mesh.uvs.push_back(glm::vec2(0.0f, 0.0f));
    mesh.uvs.push_back(glm::vec2(1.0f, 0.0f));
    mesh.uvs.push_back(glm::vec2(0.0f, 1.0f));
    mesh.uvs.push_back(glm::vec2(1.0f, 1.0f));
    mesh.indices.push_back(0);
    mesh.indices.push_back(2);
    mesh.indices.push_back(1);
    mesh.indices.push_back(1);
    mesh.indices.push_back(2);
    mesh.indices.push_back(3);
    SubMesh submesh;
    submesh.indexCount = static_cast<u32>(mesh.indices.size());
    submesh.materialSlot = 3;
    mesh.submeshes.push_back(submesh);
    return mesh;
}

void testPlaneCut()
{
    MeshData input = makeQuad();
    MeshData positive;
    CHECK(clipMeshByPlane(input, glm::vec3(1.0f, 0.0f, 0.0f), 0.0f, true, positive));
    CHECK(!positive.positions.empty());
    CHECK(positive.indices.size() % 3 == 0);
    CHECK(positive.normals.size() == positive.positions.size());
    CHECK(positive.uvs.size() == positive.positions.size());
    CHECK(positive.submeshes.size() == 1);
    CHECK(positive.submeshes[0].materialSlot == 3);

    bool foundBoundary = false;
    for (const glm::vec3& position : positive.positions)
    {
        CHECK(position.x >= -0.0001f);
        if (std::abs(position.x) < 0.0001f)
            foundBoundary = true;
    }
    CHECK(foundBoundary);

    MeshData negative;
    CHECK(clipMeshByPlane(input, glm::vec3(1.0f, 0.0f, 0.0f), 0.0f, false, negative));
    CHECK(!negative.positions.empty());
    for (const glm::vec3& position : negative.positions)
        CHECK(position.x <= 0.0001f);
}

void testRejectedInputs()
{
    MeshData input = makeQuad();
    MeshData output;
    CHECK(!clipMeshByPlane(input, glm::vec3(0.0f), 0.0f, true, output));
    input.skin.resize(input.positions.size());
    CHECK(!clipMeshByPlane(input, glm::vec3(1.0f, 0.0f, 0.0f), 0.0f, true, output));
}
}

int main()
{
    testPlaneCut();
    testRejectedInputs();
    if (gFailures)
        std::fprintf(stderr, "%d mesh clipper test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
