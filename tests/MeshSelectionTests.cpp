#include "PCH.h"

#include "AssetManager.h"
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
        std::fprintf(stderr, "MeshSelectionTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool ascendingAndUnique(const std::vector<u32>& values)
{
    for (usize i = 1; i < values.size(); ++i)
        if (values[i] <= values[i - 1])
            return false;
    return true;
}

bool contains(const std::vector<u32>& values, u32 wanted)
{
    for (usize i = 0; i < values.size(); ++i)
        if (values[i] == wanted)
            return true;
    return false;
}

// A 3x1 strip of quads: vertices 0..7, laid out as two rows of four along x.
//
//   4---5---6---7
//   | \ | \ | \ |
//   0---1---2---3
MeshData makeStrip()
{
    MeshData mesh;
    for (u32 x = 0; x < 4; ++x)
        mesh.positions.push_back(Math::vec3(static_cast<f32>(x), 0.0f, 0.0f));
    for (u32 x = 0; x < 4; ++x)
        mesh.positions.push_back(Math::vec3(static_cast<f32>(x), 0.0f, 1.0f));

    for (u32 x = 0; x < 3; ++x)
    {
        mesh.indices.push_back(x);
        mesh.indices.push_back(x + 4);
        mesh.indices.push_back(x + 5);

        mesh.indices.push_back(x);
        mesh.indices.push_back(x + 5);
        mesh.indices.push_back(x + 1);
    }

    SubMesh submesh;
    submesh.indexOffset = 0;
    submesh.indexCount = static_cast<u32>(mesh.indices.size());
    mesh.submeshes.push_back(submesh);
    return mesh;
}

MeshData makeTwoIslands()
{
    MeshData mesh = makeStrip();
    const u32 base = static_cast<u32>(mesh.positions.size());
    const usize indexBase = mesh.indices.size();

    MeshData second = makeStrip();
    for (usize i = 0; i < second.positions.size(); ++i)
        mesh.positions.push_back(second.positions[i] + Math::vec3(0.0f, 0.0f, 10.0f));
    for (usize i = 0; i < second.indices.size(); ++i)
        mesh.indices.push_back(second.indices[i] + base);

    mesh.submeshes.clear();
    SubMesh first;
    first.indexOffset = 0;
    first.indexCount = static_cast<u32>(indexBase);
    first.materialSlot = 0;
    SubMesh other;
    other.indexOffset = static_cast<u32>(indexBase);
    other.indexCount = static_cast<u32>(mesh.indices.size() - indexBase);
    other.materialSlot = 1;
    mesh.submeshes.push_back(first);
    mesh.submeshes.push_back(other);
    return mesh;
}

// Grow widens by exactly one ring: reading and writing one set would let the selection run away in one pass.
void testGrowWidensByOneRing()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeStrip();

    std::vector<u32> grown;
    assets.growVertexSelection(mesh, {0}, grown);

    CHECK(ascendingAndUnique(grown));
    CHECK(grown.size() == 4);
    CHECK(contains(grown, 0) && contains(grown, 1) && contains(grown, 4) && contains(grown, 5));
    CHECK(!contains(grown, 2));
    CHECK(!contains(grown, 6));

    std::vector<u32> again;
    assets.growVertexSelection(mesh, grown, again);
    CHECK(contains(again, 2) && contains(again, 6));
    CHECK(!contains(again, 3) && !contains(again, 7));
}

void testShrinkPeelsTheBorder()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeStrip();

    std::vector<u32> all;
    for (u32 v = 0; v < 8; ++v)
        all.push_back(v);
    std::vector<u32> shrunk;
    assets.shrinkVertexSelection(mesh, all, shrunk);
    CHECK(shrunk.size() == 8);

    std::vector<u32> grown;
    assets.growVertexSelection(mesh, {0}, grown);
    assets.shrinkVertexSelection(mesh, grown, shrunk);
    CHECK(contains(shrunk, 0));
    CHECK(!contains(shrunk, 2));
    CHECK(shrunk.size() < grown.size());
}

void testGrowFaces()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeStrip();

    std::vector<u32> grown;
    assets.growFaceSelection(mesh, {0}, grown);

    CHECK(ascendingAndUnique(grown));
    CHECK(contains(grown, 0));
    CHECK(contains(grown, 1));
    CHECK(!contains(grown, 5));
}

void testLinkedStopsAtTheIsland()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeTwoIslands();

    std::vector<u32> linked;
    assets.selectLinkedVertices(mesh, {0}, linked);

    CHECK(ascendingAndUnique(linked));
    CHECK(linked.size() == 8);
    CHECK(contains(linked, 7));
    CHECK(!contains(linked, 8));

    assets.selectLinkedVertices(mesh, {8}, linked);
    CHECK(linked.size() == 8);
    CHECK(contains(linked, 15));
    CHECK(!contains(linked, 0));

    assets.selectLinkedVertices(mesh, {0, 8}, linked);
    CHECK(linked.size() == 16);
}

void testLinkedFaces()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeTwoIslands();

    std::vector<u32> linked;
    assets.selectLinkedFaces(mesh, {0}, linked);

    CHECK(ascendingAndUnique(linked));
    CHECK(linked.size() == 6);
    for (usize i = 0; i < linked.size(); ++i)
        CHECK(linked[i] < 6);

    assets.selectLinkedFaces(mesh, {7}, linked);
    CHECK(linked.size() == 6);
    for (usize i = 0; i < linked.size(); ++i)
        CHECK(linked[i] >= 6);
}

void testSubmeshFaces()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeTwoIslands();

    std::vector<u32> faces;
    assets.submeshFaces(mesh, 0, faces);
    CHECK(faces.size() == 6);
    CHECK(faces.front() == 0 && faces.back() == 5);

    assets.submeshFaces(mesh, 1, faces);
    CHECK(faces.size() == 6);
    CHECK(faces.front() == 6 && faces.back() == 11);

    assets.submeshFaces(mesh, 9, faces);
    CHECK(faces.empty());
}

void testStaleIndicesAreIgnored()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeStrip();

    std::vector<u32> out;
    assets.growVertexSelection(mesh, {0, 500}, out);
    CHECK(contains(out, 0));
    CHECK(out.size() == 4);

    assets.growFaceSelection(mesh, {0, 900}, out);
    CHECK(contains(out, 0));

    assets.selectLinkedVertices(mesh, {999}, out);
    CHECK(out.empty());
}

void testEmptyInputs()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeStrip();

    std::vector<u32> out;
    assets.growVertexSelection(mesh, {}, out);
    CHECK(out.empty());
    assets.shrinkVertexSelection(mesh, {}, out);
    CHECK(out.empty());
    assets.growFaceSelection(mesh, {}, out);
    CHECK(out.empty());
    assets.selectLinkedVertices(mesh, {}, out);
    CHECK(out.empty());

    MeshData empty;
    assets.growVertexSelection(empty, {0}, out);
    CHECK(out.empty());
    assets.selectLinkedFaces(empty, {0}, out);
    CHECK(out.empty());
}

void testQueriesLeaveTheMeshAlone()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeTwoIslands();
    const usize vertices = mesh.positions.size();
    const usize indices = mesh.indices.size();
    const usize submeshes = mesh.submeshes.size();

    std::vector<u32> out;
    assets.growVertexSelection(mesh, {0}, out);
    assets.shrinkVertexSelection(mesh, {0, 1, 4, 5}, out);
    assets.growFaceSelection(mesh, {0}, out);
    assets.selectLinkedVertices(mesh, {0}, out);
    assets.selectLinkedFaces(mesh, {0}, out);
    assets.submeshFaces(mesh, 0, out);

    CHECK(mesh.positions.size() == vertices);
    CHECK(mesh.indices.size() == indices);
    CHECK(mesh.submeshes.size() == submeshes);
}

} // namespace

int main()
{
    testGrowWidensByOneRing();
    testShrinkPeelsTheBorder();
    testGrowFaces();
    testLinkedStopsAtTheIsland();
    testLinkedFaces();
    testSubmeshFaces();
    testStaleIndicesAreIgnored();
    testEmptyInputs();
    testQueriesLeaveTheMeshAlone();

    if (gFailures)
        std::fprintf(stderr, "%d mesh selection test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
