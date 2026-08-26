// MeshUVTests.cpp - transformFaceUVs(): retiling the UVs already on a set of
// faces, and the vertex split at the edge of that set which keeps the edit
// from spreading. Nothing here touches the GPU.

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
        std::fprintf(stderr, "MeshUVTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool near(const glm::vec2& a, const glm::vec2& b, f32 tolerance = 1e-4f)
{
    return glm::abs(a.x - b.x) <= tolerance && glm::abs(a.y - b.y) <= tolerance;
}

// One quad, two triangles. Face 0 is (0,3,2) and face 1 is (0,2,1): they
// share vertices 0 and 2, which is what has to be split when only one of them
// is retiled.
MeshData makeQuad()
{
    MeshData mesh;
    mesh.positions = {
        glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f),
        glm::vec3(1.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f),
    };
    mesh.uvs = {glm::vec2(0.0f, 0.0f), glm::vec2(1.0f, 0.0f), glm::vec2(1.0f, 1.0f),
                glm::vec2(0.0f, 1.0f)};
    mesh.indices = {0, 3, 2, 0, 2, 1};

    SubMesh submesh;
    submesh.indexOffset = 0;
    submesh.indexCount = 6;
    mesh.submeshes.push_back(submesh);
    return mesh;
}

// The whole reason the function splits vertices: retiling one face must leave
// the face joined to it exactly as it was.
void testNeighbourKeepsItsUVs()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeQuad();

    CHECK(assets.transformFaceUVs(mesh, {0}, glm::vec2(2.0f), 0.0f, glm::vec2(0.0f)));

    // Vertices 0 and 2 belong to both faces, so each gained a copy; vertex 3
    // is face 0's alone and did not.
    CHECK(mesh.positions.size() == 6);
    CHECK(mesh.uvs.size() == 6);

    // Face 1 still names the originals and their UVs are untouched.
    CHECK(mesh.indices[3] == 0 && mesh.indices[4] == 2 && mesh.indices[5] == 1);
    CHECK(near(mesh.uvs[0], glm::vec2(0.0f, 0.0f)));
    CHECK(near(mesh.uvs[2], glm::vec2(1.0f, 1.0f)));
    CHECK(near(mesh.uvs[1], glm::vec2(1.0f, 0.0f)));

    // Face 0 now names the copies, and the one vertex it had to itself.
    CHECK(mesh.indices[0] == 4);
    CHECK(mesh.indices[1] == 3);
    CHECK(mesh.indices[2] == 5);

    // Scaled by two about the centre of (0,0)..(1,1), which is (0.5,0.5).
    CHECK(near(mesh.uvs[4], glm::vec2(-0.5f, -0.5f)));
    CHECK(near(mesh.uvs[3], glm::vec2(-0.5f, 1.5f)));
    CHECK(near(mesh.uvs[5], glm::vec2(1.5f, 1.5f)));

    // A split copies the position, not just the UV.
    CHECK(mesh.positions[4] == mesh.positions[0]);
    CHECK(mesh.positions[5] == mesh.positions[2]);
}

// Nothing is shared with anything left out, so nothing needs splitting.
void testWholeMeshSplitsNothing()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeQuad();

    CHECK(assets.transformFaceUVs(mesh, {}, glm::vec2(2.0f), 0.0f, glm::vec2(0.0f)));
    CHECK(mesh.positions.size() == 4);

    // Same centre, so the corners spread symmetrically about it.
    CHECK(near(mesh.uvs[0], glm::vec2(-0.5f, -0.5f)));
    CHECK(near(mesh.uvs[2], glm::vec2(1.5f, 1.5f)));

    // Naming every face by hand is the same as naming none.
    MeshData explicitAll = makeQuad();
    CHECK(assets.transformFaceUVs(explicitAll, {0, 1}, glm::vec2(2.0f), 0.0f, glm::vec2(0.0f)));
    CHECK(explicitAll.positions.size() == 4);
    CHECK(near(explicitAll.uvs[0], mesh.uvs[0]));
}

// Scaling about the origin instead of the island's own centre would slide the
// texture away by however far the island sits from (0,0). The centre staying
// put is what makes the amount mean the same thing anywhere.
void testScaleKeepsTheCentre()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeQuad();
    for (usize i = 0; i < mesh.uvs.size(); ++i)
        mesh.uvs[i] += glm::vec2(10.0f, 10.0f);

    CHECK(assets.transformFaceUVs(mesh, {}, glm::vec2(3.0f), 0.0f, glm::vec2(0.0f)));

    glm::vec2 min = mesh.uvs[0];
    glm::vec2 max = mesh.uvs[0];
    for (usize i = 1; i < mesh.uvs.size(); ++i)
    {
        min = glm::min(min, mesh.uvs[i]);
        max = glm::max(max, mesh.uvs[i]);
    }
    CHECK(near((min + max) * 0.5f, glm::vec2(10.5f, 10.5f)));
    // Three times as wide as it was.
    CHECK(glm::abs((max.x - min.x) - 3.0f) < 1e-4f);
}

void testOffsetAndRotation()
{
    AssetManager& assets = AssetManager::getSingleton();

    MeshData moved = makeQuad();
    CHECK(assets.transformFaceUVs(moved, {}, glm::vec2(1.0f), 0.0f, glm::vec2(0.25f, -0.5f)));
    CHECK(near(moved.uvs[0], glm::vec2(0.25f, -0.5f)));
    CHECK(near(moved.uvs[2], glm::vec2(1.25f, 0.5f)));

    // 90 degrees about the centre (0.5,0.5) takes the corner at (0,0) to
    // (1,0): x' = cx + (x-cx)cos - (y-cy)sin.
    MeshData turned = makeQuad();
    CHECK(assets.transformFaceUVs(turned, {}, glm::vec2(1.0f), 90.0f, glm::vec2(0.0f)));
    CHECK(near(turned.uvs[0], glm::vec2(1.0f, 0.0f)));
    CHECK(near(turned.uvs[2], glm::vec2(0.0f, 1.0f)));

    // Turning it four times gets back where it started.
    MeshData full = makeQuad();
    for (u32 i = 0; i < 4; ++i)
        CHECK(assets.transformFaceUVs(full, {}, glm::vec2(1.0f), 90.0f, glm::vec2(0.0f)));
    CHECK(near(full.uvs[0], glm::vec2(0.0f, 0.0f), 1e-3f));
    CHECK(near(full.uvs[2], glm::vec2(1.0f, 1.0f), 1e-3f));
}

void testFlip()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeQuad();

    CHECK(assets.transformFaceUVs(mesh, {}, glm::vec2(-1.0f, 1.0f), 0.0f, glm::vec2(0.0f)));
    // Mirrored about the centre: the two ends swap and nothing moves off it.
    CHECK(near(mesh.uvs[0], glm::vec2(1.0f, 0.0f)));
    CHECK(near(mesh.uvs[1], glm::vec2(0.0f, 0.0f)));
}

void testRejectsMeshWithoutUVs()
{
    AssetManager& assets = AssetManager::getSingleton();

    MeshData mesh = makeQuad();
    mesh.uvs.clear();
    CHECK(!assets.transformFaceUVs(mesh, {}, glm::vec2(2.0f), 0.0f, glm::vec2(0.0f)));
    CHECK(mesh.positions.size() == 4);

    // A UV array that is present but the wrong length is refused too rather
    // than read past its end.
    MeshData short_ = makeQuad();
    short_.uvs.pop_back();
    CHECK(!assets.transformFaceUVs(short_, {}, glm::vec2(2.0f), 0.0f, glm::vec2(0.0f)));

    MeshData empty;
    CHECK(!assets.transformFaceUVs(empty, {}, glm::vec2(2.0f), 0.0f, glm::vec2(0.0f)));
}

// A selection left over from a bigger mesh names faces that are gone; it must
// come back false rather than split or move anything.
void testRejectsSelectionOfNothing()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeQuad();
    const std::vector<glm::vec2> before = mesh.uvs;

    CHECK(!assets.transformFaceUVs(mesh, {17, 99}, glm::vec2(2.0f), 0.0f, glm::vec2(0.0f)));
    CHECK(mesh.positions.size() == 4);
    for (usize i = 0; i < mesh.uvs.size(); ++i)
        CHECK(near(mesh.uvs[i], before[i]));
}

// A split has to carry every stream across, or the copy renders with someone
// else's normal or skin weights.
void testSplitCarriesEveryStream()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeQuad();
    mesh.normals.assign(4, glm::vec3(0.0f, 1.0f, 0.0f));
    mesh.colors.assign(4, 0xff112233u);
    mesh.uvs2.assign(4, glm::vec2(0.25f, 0.75f));

    CHECK(assets.transformFaceUVs(mesh, {0}, glm::vec2(2.0f), 0.0f, glm::vec2(0.0f)));

    CHECK(mesh.positions.size() == 6);
    CHECK(mesh.normals.size() == 6);
    CHECK(mesh.colors.size() == 6);
    CHECK(mesh.uvs2.size() == 6);
    CHECK(mesh.colors[4] == 0xff112233u);
    CHECK(near(mesh.uvs2[4], glm::vec2(0.25f, 0.75f)));
    // The second UV set is a lightmap's own unwrap and is not what was asked
    // to be retiled.
    CHECK(near(mesh.uvs2[0], glm::vec2(0.25f, 0.75f)));
}

} // namespace

int main()
{
    testNeighbourKeepsItsUVs();
    testWholeMeshSplitsNothing();
    testScaleKeepsTheCentre();
    testOffsetAndRotation();
    testFlip();
    testRejectsMeshWithoutUVs();
    testRejectsSelectionOfNothing();
    testSplitCarriesEveryStream();

    if (gFailures)
        std::fprintf(stderr, "%d mesh uv test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
