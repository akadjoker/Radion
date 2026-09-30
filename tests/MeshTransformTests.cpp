#include "PCH.h"

#include "AssetManager.h"
#include "Mesh.h"

#include <cstdio>
#include "Math.h"
#include <vector>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "MeshTransformTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool near(f32 a, f32 b, f32 tolerance = 1e-4f)
{
    return Math::abs(a - b) <= tolerance;
}

bool near(const Math::vec3& a, const Math::vec3& b, f32 tolerance = 1e-4f)
{
    return near(a.x, b.x, tolerance) && near(a.y, b.y, tolerance) && near(a.z, b.z, tolerance);
}

// Well away from the origin: a pivot bug slides the mesh toward (0,0,0), which an origin-centred mesh would hide.
MeshData makeOffsetQuad()
{
    MeshData mesh;
    mesh.positions = {
        Math::vec3(100.0f, 0.0f, 100.0f), Math::vec3(102.0f, 0.0f, 100.0f),
        Math::vec3(102.0f, 0.0f, 102.0f), Math::vec3(100.0f, 0.0f, 102.0f),
    };
    mesh.normals = {
        Math::vec3(0.0f, 1.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f),
        Math::vec3(0.0f, 1.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f),
    };
    mesh.uvs = {Math::vec2(0.0f), Math::vec2(1.0f, 0.0f), Math::vec2(1.0f), Math::vec2(0.0f, 1.0f)};
    mesh.indices = {0, 1, 2, 0, 2, 3};

    SubMesh submesh;
    submesh.indexOffset = 0;
    submesh.indexCount = 6;
    mesh.submeshes.push_back(submesh);
    return mesh;
}

void testWholeMeshScalesAboutItsMedian()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeOffsetQuad();

    assets.transformVertices(mesh, Math::scale(Math::mat4(1.0f), Math::vec3(2.0f)));

    CHECK(near(mesh.positions[0], Math::vec3(99.0f, 0.0f, 99.0f)));
    CHECK(near(mesh.positions[1], Math::vec3(103.0f, 0.0f, 99.0f)));
    CHECK(near(mesh.positions[2], Math::vec3(103.0f, 0.0f, 103.0f)));
    CHECK(near(mesh.positions[3], Math::vec3(99.0f, 0.0f, 103.0f)));

    Math::vec3 median(0.0f);
    for (usize i = 0; i < mesh.positions.size(); ++i)
        median += mesh.positions[i];
    median /= static_cast<f32>(mesh.positions.size());
    CHECK(near(median, Math::vec3(101.0f, 0.0f, 101.0f)));

    // Bounds must follow, or framing and culling use the old box.
    CHECK(near(mesh.bounds.min, Math::vec3(99.0f, 0.0f, 99.0f)));
    CHECK(near(mesh.bounds.max, Math::vec3(103.0f, 0.0f, 103.0f)));
}

void testSubsetLeavesTheRestAlone()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeOffsetQuad();
    const std::vector<Math::vec3> before = mesh.positions;

    const std::vector<u32> selection = {0, 1};
    assets.transformVertices(mesh, Math::scale(Math::mat4(1.0f), Math::vec3(2.0f)), selection);

    CHECK(near(mesh.positions[0], Math::vec3(99.0f, 0.0f, 100.0f)));
    CHECK(near(mesh.positions[1], Math::vec3(103.0f, 0.0f, 100.0f)));
    CHECK(near(mesh.positions[2], before[2]));
    CHECK(near(mesh.positions[3], before[3]));
}

void testTranslationMovesEverythingEqually()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeOffsetQuad();
    const std::vector<Math::vec3> before = mesh.positions;

    const Math::vec3 delta(5.0f, -2.0f, 0.5f);
    assets.transformVertices(mesh, Math::translate(Math::mat4(1.0f), delta));

    for (usize i = 0; i < mesh.positions.size(); ++i)
        CHECK(near(mesh.positions[i], before[i] + delta));
}

// Rotating positions without normals leaves the surface lit as if unturned.
void testRotationCarriesNormals()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeOffsetQuad();

    // Right-handed, so 90 degrees about +X takes +Y to +Z.
    assets.transformVertices(
        mesh, Math::rotate(Math::mat4(1.0f), Math::radians(90.0f), Math::vec3(1.0f, 0.0f, 0.0f)));

    for (usize i = 0; i < mesh.normals.size(); ++i)
        CHECK(near(mesh.normals[i], Math::vec3(0.0f, 0.0f, 1.0f)));

    CHECK(near(Math::length(mesh.normals[0]), 1.0f));
}

void testRotationOfSubsetKeepsOtherNormals()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeOffsetQuad();

    const std::vector<u32> selection = {0};
    assets.transformVertices(
        mesh, Math::rotate(Math::mat4(1.0f), Math::radians(90.0f), Math::vec3(1.0f, 0.0f, 0.0f)),
        selection);

    CHECK(near(mesh.normals[0], Math::vec3(0.0f, 0.0f, 1.0f)));
    CHECK(near(mesh.normals[1], Math::vec3(0.0f, 1.0f, 0.0f)));
    CHECK(near(mesh.normals[2], Math::vec3(0.0f, 1.0f, 0.0f)));

    CHECK(near(mesh.positions[0], Math::vec3(100.0f, 0.0f, 100.0f)));
}

// A BlenderSelection may outlive its mesh: out-of-range indices must not read past the positions.
void testOutOfRangeIndicesAreIgnored()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeOffsetQuad();
    const std::vector<Math::vec3> before = mesh.positions;

    const std::vector<u32> selection = {0, 9999, 4};
    assets.transformVertices(mesh, Math::translate(Math::mat4(1.0f), Math::vec3(1.0f, 0.0f, 0.0f)),
                             selection);

    CHECK(near(mesh.positions[0], before[0] + Math::vec3(1.0f, 0.0f, 0.0f)));
    CHECK(near(mesh.positions[1], before[1]));

    MeshData untouched = makeOffsetQuad();
    const std::vector<u32> allBad = {500, 501};
    assets.transformVertices(untouched, Math::scale(Math::mat4(1.0f), Math::vec3(3.0f)), allBad);
    for (usize i = 0; i < untouched.positions.size(); ++i)
        CHECK(near(untouched.positions[i], before[i]));
}

void testEmptyMeshIsSafe()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh;
    assets.transformVertices(mesh, Math::scale(Math::mat4(1.0f), Math::vec3(2.0f)));
    CHECK(mesh.positions.empty());
}

void testIdentityChangesNothing()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeOffsetQuad();
    const std::vector<Math::vec3> before = mesh.positions;

    assets.transformVertices(mesh, Math::mat4(1.0f));

    for (usize i = 0; i < mesh.positions.size(); ++i)
        CHECK(near(mesh.positions[i], before[i], 1e-3f));
}

// A mirrored transform must reverse the winding or faces render backwards.
void testMirrorFlipsWinding()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeOffsetQuad();
    const std::vector<u32> before = mesh.indices;

    assets.transformVertices(mesh, Math::scale(Math::mat4(1.0f), Math::vec3(-1.0f, 1.0f, 1.0f)));

    CHECK(mesh.indices.size() == before.size());
    bool reversed = false;
    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
        if (mesh.indices[i + 1] != before[i + 1])
            reversed = true;
    CHECK(reversed);
}

} // namespace

int main()
{
    testWholeMeshScalesAboutItsMedian();
    testSubsetLeavesTheRestAlone();
    testTranslationMovesEverythingEqually();
    testRotationCarriesNormals();
    testRotationOfSubsetKeepsOtherNormals();
    testOutOfRangeIndicesAreIgnored();
    testEmptyMeshIsSafe();
    testIdentityChangesNothing();
    testMirrorFlipsWinding();

    if (gFailures)
        std::fprintf(stderr, "%d mesh transform test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
