#include "PCH.h"

#include "AssetManager.h"
#include "Mesh.h"

#include <cstdio>
#include <set>
#include <vector>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "MeshExtrudeTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool near(f32 a, f32 b, f32 tolerance = 1e-4f)
{
    return Math::abs(a - b) <= tolerance;
}

// Two triangles forming a flat quad in XZ, normal +Y; the shared diagonal 0-2 must not grow a wall when both are extruded.
MeshData makeQuad()
{
    MeshData mesh;
    mesh.positions = {
        Math::vec3(0.0f, 0.0f, 0.0f), Math::vec3(1.0f, 0.0f, 0.0f),
        Math::vec3(1.0f, 0.0f, 1.0f), Math::vec3(0.0f, 0.0f, 1.0f),
    };
    mesh.normals.assign(4, Math::vec3(0.0f, 1.0f, 0.0f));
    mesh.uvs = {Math::vec2(0.0f, 0.0f), Math::vec2(1.0f, 0.0f), Math::vec2(1.0f, 1.0f),
                Math::vec2(0.0f, 1.0f)};
    mesh.colors.assign(4, 0xff00ff00u);
    mesh.indices = {0, 3, 2, 0, 2, 1};

    SubMesh submesh;
    submesh.indexOffset = 0;
    submesh.indexCount = 6;
    mesh.submeshes.push_back(submesh);
    return mesh;
}

bool indicesValid(const MeshData& mesh)
{
    for (u32 index : mesh.indices)
        if (index >= mesh.positions.size())
            return false;
    return true;
}

bool submeshesCoverIndices(const MeshData& mesh)
{
    usize total = 0;
    for (usize i = 0; i < mesh.submeshes.size(); ++i)
    {
        if (mesh.submeshes[i].indexOffset != total)
            return false;
        total += mesh.submeshes[i].indexCount;
    }
    return total == mesh.indices.size();
}

// An edge shared by two extruded faces is interior and gets no wall; otherwise a wall is buried in the solid.
void testInteriorEdgeGetsNoWall()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeQuad();

    const std::vector<u32> both = {0, 1};
    std::vector<u32> raised;
    CHECK(assets.extrudeFaces(mesh, both, 2.0f, &raised));

    // 2 caps + 4 boundary edges x 2 triangles; a wall on the diagonal would make 12.
    CHECK(mesh.indices.size() / 3 == 10);
    CHECK(raised.size() == 2);
    CHECK(indicesValid(mesh));
    CHECK(submeshesCoverIndices(mesh));

    CHECK(mesh.positions.size() == 8);
    CHECK(mesh.normals.size() == 8);
    CHECK(mesh.uvs.size() == 8);
    CHECK(mesh.colors.size() == 8);
}

void testEdgeSharedWithAnUnselectedFaceIsBoundary()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeQuad();

    const std::vector<u32> one = {0};
    CHECK(assets.extrudeFaces(mesh, one, 1.0f));

    CHECK(mesh.indices.size() / 3 == 1 + 1 + 6);
    CHECK(indicesValid(mesh));
    CHECK(submeshesCoverIndices(mesh));
    CHECK(mesh.positions.size() == 7);
}

void testCapMovesAlongTheNormal()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeQuad();

    std::vector<u32> raised;
    const std::vector<u32> both = {0, 1};
    CHECK(assets.extrudeFaces(mesh, both, 2.0f, &raised));
    CHECK(raised.size() == 2);

    for (usize f = 0; f < raised.size(); ++f)
    {
        const u32 face = raised[f];
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 index = mesh.indices[face * 3 + corner];
            CHECK(near(mesh.positions[index].y, 2.0f));
            CHECK(index >= 4);
        }
    }

    for (u32 v = 0; v < 4; ++v)
        CHECK(near(mesh.positions[v].y, 0.0f));

    CHECK(mesh.colors[4] == 0xff00ff00u);
}

void testNegativeDistance()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeQuad();

    std::vector<u32> raised;
    const std::vector<u32> both = {0, 1};
    CHECK(assets.extrudeFaces(mesh, both, -1.5f, &raised));

    CHECK(mesh.indices.size() / 3 == 10);
    CHECK(indicesValid(mesh));
    for (usize f = 0; f < raised.size(); ++f)
        for (u32 corner = 0; corner < 3; ++corner)
            CHECK(near(mesh.positions[mesh.indices[raised[f] * 3 + corner]].y, -1.5f));
}

// Re-extruding the returned faces: the index buffer was rebuilt, so old face numbers are stale.
void testExtrudingTheResultAgain()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeQuad();

    std::vector<u32> raised;
    const std::vector<u32> both = {0, 1};
    CHECK(assets.extrudeFaces(mesh, both, 1.0f, &raised));

    std::vector<u32> raisedAgain;
    CHECK(assets.extrudeFaces(mesh, raised, 1.0f, &raisedAgain));

    CHECK(raisedAgain.size() == 2);
    CHECK(indicesValid(mesh));
    CHECK(submeshesCoverIndices(mesh));
    for (usize f = 0; f < raisedAgain.size(); ++f)
        for (u32 corner = 0; corner < 3; ++corner)
            CHECK(near(mesh.positions[mesh.indices[raisedAgain[f] * 3 + corner]].y, 2.0f));
}

void testWallsStayInTheirSubmesh()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeQuad();
    mesh.submeshes.clear();

    SubMesh first;
    first.indexOffset = 0;
    first.indexCount = 3;
    first.materialSlot = 0;
    SubMesh second;
    second.indexOffset = 3;
    second.indexCount = 3;
    second.materialSlot = 1;
    mesh.submeshes.push_back(first);
    mesh.submeshes.push_back(second);

    const std::vector<u32> one = {1};
    CHECK(assets.extrudeFaces(mesh, one, 1.0f));

    CHECK(mesh.submeshes.size() == 2);
    CHECK(submeshesCoverIndices(mesh));
    CHECK(mesh.submeshes[0].indexCount == 3);
    CHECK(mesh.submeshes[1].indexCount == (1 + 6) * 3);
    CHECK(mesh.submeshes[1].materialSlot == 1);
}

void testRejectsNothingToDo()
{
    AssetManager& assets = AssetManager::getSingleton();

    MeshData mesh = makeQuad();
    const MeshData before = mesh;
    CHECK(!assets.extrudeFaces(mesh, {}, 1.0f));
    CHECK(mesh.indices.size() == before.indices.size());
    CHECK(mesh.positions.size() == before.positions.size());

    const std::vector<u32> bogus = {17, 900};
    CHECK(!assets.extrudeFaces(mesh, bogus, 1.0f));
    CHECK(mesh.indices.size() == before.indices.size());

    MeshData empty;
    CHECK(!assets.extrudeFaces(empty, {0}, 1.0f));
}

// With no optional streams none may come back half-filled (a later upload reads past the end).
void testMeshWithoutOptionalStreams()
{
    AssetManager& assets = AssetManager::getSingleton();

    MeshData mesh;
    mesh.positions = {
        Math::vec3(0.0f, 0.0f, 0.0f), Math::vec3(1.0f, 0.0f, 0.0f),
        Math::vec3(1.0f, 0.0f, 1.0f),
    };
    mesh.indices = {0, 2, 1};

    CHECK(assets.extrudeFaces(mesh, {0}, 1.0f));
    CHECK(mesh.positions.size() == 6);
    CHECK(mesh.normals.empty());
    CHECK(mesh.uvs.empty());
    CHECK(mesh.colors.empty());
    CHECK(indicesValid(mesh));
    CHECK(submeshesCoverIndices(mesh));
}

// A wall uses two originals and two duplicates, else it is a flat sliver in the old surface.
void testWallsJoinBaseToCap()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeQuad();

    std::vector<u32> raised;
    const std::vector<u32> both = {0, 1};
    CHECK(assets.extrudeFaces(mesh, both, 1.0f, &raised));

    std::set<u32> capFaces(raised.begin(), raised.end());
    const usize faceCount = mesh.indices.size() / 3;
    usize wallCount = 0;
    for (usize face = 0; face < faceCount; ++face)
    {
        if (capFaces.count(static_cast<u32>(face)))
            continue;

        u32 base = 0;
        u32 cap = 0;
        for (u32 corner = 0; corner < 3; ++corner)
        {
            if (mesh.indices[face * 3 + corner] < 4)
                ++base;
            else
                ++cap;
        }
        CHECK(base > 0 && cap > 0);
        ++wallCount;
    }
    CHECK(wallCount == 8);
}

} // namespace

int main()
{
    testInteriorEdgeGetsNoWall();
    testEdgeSharedWithAnUnselectedFaceIsBoundary();
    testCapMovesAlongTheNormal();
    testNegativeDistance();
    testExtrudingTheResultAgain();
    testWallsStayInTheirSubmesh();
    testRejectsNothingToDo();
    testMeshWithoutOptionalStreams();
    testWallsJoinBaseToCap();

    if (gFailures)
        std::fprintf(stderr, "%d mesh extrude test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
