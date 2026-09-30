// MeshExtrudeTests.cpp - extrudeFaces(): a region of faces raised along its
// own normals, walled in along its boundary. Nothing here touches the GPU.

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

// Two triangles making one flat quad in XZ, both wound so their normal is +Y.
// The diagonal 0-2 is shared, which is the edge that must NOT grow a wall
// when both triangles are extruded together.
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

// The one property that separates a real region extrude from a naive one: an
// edge shared by two extruded faces is inside the region and gets no wall.
// Getting this wrong buries a wall inside the solid, where it is invisible
// until something shades or collides against it.
void testInteriorEdgeGetsNoWall()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeQuad();

    const std::vector<u32> both = {0, 1};
    std::vector<u32> raised;
    CHECK(assets.extrudeFaces(mesh, both, 2.0f, &raised));

    // 2 cap triangles, plus 4 boundary edges walled with 2 triangles each.
    // A wall on the shared diagonal too would make it 12.
    CHECK(mesh.indices.size() / 3 == 10);
    CHECK(raised.size() == 2);
    CHECK(indicesValid(mesh));
    CHECK(submeshesCoverIndices(mesh));

    // Four originals plus one duplicate each.
    CHECK(mesh.positions.size() == 8);
    CHECK(mesh.normals.size() == 8);
    CHECK(mesh.uvs.size() == 8);
    CHECK(mesh.colors.size() == 8);
}

// The same two triangles, extruded one at a time: now the diagonal is used by
// one selected face and one unselected one, so it IS on the boundary.
void testEdgeSharedWithAnUnselectedFaceIsBoundary()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeQuad();

    const std::vector<u32> one = {0};
    CHECK(assets.extrudeFaces(mesh, one, 1.0f));

    // The untouched triangle, the cap, and three walls of two triangles.
    CHECK(mesh.indices.size() / 3 == 1 + 1 + 6);
    CHECK(indicesValid(mesh));
    CHECK(submeshesCoverIndices(mesh));
    // Only that triangle's three vertices were duplicated.
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

    // Every vertex of a raised face sits 2 units up, and nowhere else.
    for (usize f = 0; f < raised.size(); ++f)
    {
        const u32 face = raised[f];
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 index = mesh.indices[face * 3 + corner];
            CHECK(near(mesh.positions[index].y, 2.0f));
            // Duplicates come after the originals.
            CHECK(index >= 4);
        }
    }

    // The originals stayed where they were.
    for (u32 v = 0; v < 4; ++v)
        CHECK(near(mesh.positions[v].y, 0.0f));

    // A duplicate carries its source's attributes across.
    CHECK(mesh.colors[4] == 0xff00ff00u);
}

// A negative distance pulls the region the other way; the walls have to
// follow rather than be built for the outward case only.
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

// Extruding again on what the first one returned is the whole point of the
// out parameter - the index buffer is rebuilt, so the old face numbers point
// at whatever inherited them.
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
    // Two units up in total, not one: the second extrude found the cap.
    for (usize f = 0; f < raisedAgain.size(); ++f)
        for (u32 corner = 0; corner < 3; ++corner)
            CHECK(near(mesh.positions[mesh.indices[raisedAgain[f] * 3 + corner]].y, 2.0f));
}

// Walls belong to the material of the face that raised them, or an extrusion
// on a two-material mesh comes out wearing the wrong one.
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

    // Face 1 lives in the second submesh.
    const std::vector<u32> one = {1};
    CHECK(assets.extrudeFaces(mesh, one, 1.0f));

    CHECK(mesh.submeshes.size() == 2);
    CHECK(submeshesCoverIndices(mesh));
    // The first submesh keeps its single untouched triangle; everything the
    // extrude produced landed in the second.
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

    // Face numbers past the end are ignored, and a selection of nothing but
    // those leaves the mesh alone.
    const std::vector<u32> bogus = {17, 900};
    CHECK(!assets.extrudeFaces(mesh, bogus, 1.0f));
    CHECK(mesh.indices.size() == before.indices.size());

    MeshData empty;
    CHECK(!assets.extrudeFaces(empty, {0}, 1.0f));
}

// A mesh carrying none of the optional streams must not come back with
// half-filled ones, which is what makes a later upload read past the end.
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

// Every wall triangle must use two originals and two duplicates - a wall
// built from four originals would be a flat sliver lying in the old surface.
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
