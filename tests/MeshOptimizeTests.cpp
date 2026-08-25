// MeshOptimizeTests.cpp - CPU-side mesh optimization passes on MeshData:
// weld, vertex cache / overdraw / vertex fetch reorder, and simplification.
// Nothing here touches the GPU.

#include "PCH.h"

#include "AssetManager.h"
#include "Mesh.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <set>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "MeshOptimizeTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool indicesValid(const MeshData& mesh)
{
    for (u32 index : mesh.indices)
        if (index >= mesh.positions.size())
            return false;
    for (const SubMesh& submesh : mesh.submeshes)
        if (usize(submesh.indexOffset) + submesh.indexCount > mesh.indices.size())
            return false;
    return true;
}

// The set of triangles by vertex position, winding-normalized by rotating the
// smallest corner first - index reordering passes must preserve it exactly.
std::multiset<std::array<f32, 9>> triangleSet(const MeshData& mesh)
{
    std::multiset<std::array<f32, 9>> set;
    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        std::array<std::array<f32, 3>, 3> tri;
        for (int corner = 0; corner < 3; ++corner)
        {
            const glm::vec3& p = mesh.positions[mesh.indices[i + corner]];
            tri[corner] = {p.x, p.y, p.z};
        }
        while (tri[1] < tri[0] || tri[2] < tri[0])
            std::rotate(tri.begin(), tri.begin() + 1, tri.end());
        set.insert({tri[0][0], tri[0][1], tri[0][2], tri[1][0], tri[1][1], tri[1][2], tri[2][0],
                    tri[2][1], tri[2][2]});
    }
    return set;
}

// A flat XZ grid emitted OBJ-style: every quad writes its own four corners,
// so shared corners are duplicated - exactly what weld exists to clean up.
MeshData makeGridWithDuplicates(u32 quads)
{
    MeshData mesh;
    for (u32 z = 0; z < quads; ++z)
        for (u32 x = 0; x < quads; ++x)
        {
            const u32 base = static_cast<u32>(mesh.positions.size());
            const f32 fx = static_cast<f32>(x);
            const f32 fz = static_cast<f32>(z);
            mesh.positions.push_back(glm::vec3(fx, 0.0f, fz));
            mesh.positions.push_back(glm::vec3(fx + 1.0f, 0.0f, fz));
            mesh.positions.push_back(glm::vec3(fx + 1.0f, 0.0f, fz + 1.0f));
            mesh.positions.push_back(glm::vec3(fx, 0.0f, fz + 1.0f));
            for (int i = 0; i < 4; ++i)
            {
                mesh.normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
                mesh.uvs.push_back(glm::vec2(0.0f, 0.0f));
            }
            const u32 quad[6] = {base, base + 2, base + 1, base, base + 3, base + 2};
            mesh.indices.insert(mesh.indices.end(), quad, quad + 6);
        }

    SubMesh submesh;
    submesh.indexOffset = 0;
    submesh.indexCount = static_cast<u32>(mesh.indices.size());
    mesh.submeshes.push_back(submesh);
    return mesh;
}

void testWeld()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeGridWithDuplicates(4);

    const auto before = triangleSet(mesh);
    const usize beforeVerts = mesh.positions.size(); // 64
    const u32 removed = assets.weldVertices(mesh);

    // A 4x4 quad grid has 25 unique corners.
    CHECK(removed == beforeVerts - 25);
    CHECK(mesh.positions.size() == 25);
    CHECK(mesh.normals.size() == 25);
    CHECK(mesh.uvs.size() == 25);
    CHECK(indicesValid(mesh));
    CHECK(triangleSet(mesh) == before);

    // Welding an already-welded mesh removes nothing.
    CHECK(assets.weldVertices(mesh) == 0);
}

void testIndexReorderPasses()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeGridWithDuplicates(4);
    assets.weldVertices(mesh);

    const auto before = triangleSet(mesh);
    const std::vector<SubMesh> submeshesBefore = mesh.submeshes;
    const std::vector<glm::vec3> positionsBefore = mesh.positions;

    assets.optimizeVertexCache(mesh);
    assets.optimizeOverdraw(mesh, 1.05f);

    // Pure index reorders: same triangles, same vertices, same ranges.
    CHECK(triangleSet(mesh) == before);
    CHECK(mesh.positions == positionsBefore);
    CHECK(mesh.submeshes.size() == submeshesBefore.size());
    CHECK(mesh.submeshes[0].indexOffset == submeshesBefore[0].indexOffset);
    CHECK(mesh.submeshes[0].indexCount == submeshesBefore[0].indexCount);
    CHECK(indicesValid(mesh));

    // Vertex fetch reorders the streams and drops unreferenced vertices; an
    // extra orphan vertex must disappear.
    mesh.positions.push_back(glm::vec3(99.0f, 99.0f, 99.0f));
    mesh.normals.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
    mesh.uvs.push_back(glm::vec2(0.0f, 0.0f));
    assets.optimizeVertexFetch(mesh);
    CHECK(mesh.positions.size() == 25);
    CHECK(mesh.normals.size() == 25);
    CHECK(mesh.uvs.size() == 25);
    CHECK(triangleSet(mesh) == before);
    CHECK(indicesValid(mesh));
}

void testSimplify()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeGridWithDuplicates(16);
    assets.weldVertices(mesh);
    assets.computeBounds(mesh);

    const usize beforeTris = mesh.indices.size() / 3; // 512
    f32 reachedError = -1.0f;
    CHECK(assets.simplifyMesh(mesh, 0.25f, 0.05f, &reachedError));

    // A flat plane collapses essentially for free.
    CHECK(mesh.indices.size() / 3 < beforeTris / 2);
    CHECK(reachedError >= 0.0f);
    CHECK(reachedError <= 0.05f);
    CHECK(indicesValid(mesh));
    CHECK(mesh.submeshes.size() == 1);
    CHECK(mesh.submeshes[0].indexOffset == 0);
    CHECK(mesh.submeshes[0].indexCount == mesh.indices.size());
    CHECK(mesh.indices.size() % 3 == 0);

    // Compaction afterwards drops the vertices simplification orphaned.
    const usize vertsBeforeFetch = mesh.positions.size();
    assets.optimizeVertexFetch(mesh);
    CHECK(mesh.positions.size() < vertsBeforeFetch);
    CHECK(indicesValid(mesh));

    // Degenerate input fails instead of crashing.
    MeshData empty;
    CHECK(!assets.simplifyMesh(empty, 0.5f, 0.01f, nullptr));
}

} // namespace

int main()
{
    testWeld();
    testIndexReorderPasses();
    testSimplify();

    if (gFailures)
        std::fprintf(stderr, "%d mesh optimize test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
