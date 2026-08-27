// MeshTransformTests.cpp - transformVertices(): a matrix baked into a subset
// of a mesh's vertices around their own median point. Nothing here touches
// the GPU.

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

// Two triangles, well away from the origin: a pivot bug shows up as the whole
// thing sliding toward or away from (0,0,0), which a mesh built around the
// origin would hide completely.
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

// Scaling the whole mesh must grow it where it stands. About the origin a
// quad at x=100 would land at x=200; about its own median it stays centred.
void testWholeMeshScalesAboutItsMedian()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeOffsetQuad();

    assets.transformVertices(mesh, Math::scale(Math::mat4(1.0f), Math::vec3(2.0f)));

    CHECK(near(mesh.positions[0], Math::vec3(99.0f, 0.0f, 99.0f)));
    CHECK(near(mesh.positions[1], Math::vec3(103.0f, 0.0f, 99.0f)));
    CHECK(near(mesh.positions[2], Math::vec3(103.0f, 0.0f, 103.0f)));
    CHECK(near(mesh.positions[3], Math::vec3(99.0f, 0.0f, 103.0f)));

    // The median is exactly where it was.
    Math::vec3 median(0.0f);
    for (usize i = 0; i < mesh.positions.size(); ++i)
        median += mesh.positions[i];
    median /= static_cast<f32>(mesh.positions.size());
    CHECK(near(median, Math::vec3(101.0f, 0.0f, 101.0f)));

    // Bounds follow, or anything that frames or culls the mesh is left with
    // the old box.
    CHECK(near(mesh.bounds.min, Math::vec3(99.0f, 0.0f, 99.0f)));
    CHECK(near(mesh.bounds.max, Math::vec3(103.0f, 0.0f, 103.0f)));
}

// The point of the selection argument: unselected geometry does not move.
void testSubsetLeavesTheRestAlone()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeOffsetQuad();
    const std::vector<Math::vec3> before = mesh.positions;

    const std::vector<u32> selection = {0, 1};
    assets.transformVertices(mesh, Math::scale(Math::mat4(1.0f), Math::vec3(2.0f)), selection);

    // Median of the two selected corners is (101, 0, 100); they move apart
    // along x around it and stay put on z.
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

    // A translation is unaffected by which pivot it is applied around.
    for (usize i = 0; i < mesh.positions.size(); ++i)
        CHECK(near(mesh.positions[i], before[i] + delta));
}

// Rotating positions without rotating normals leaves the surface lit as if it
// never turned - the failure that makes a rotated mesh look wrong rather than
// broken, so it is easy to ship.
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

    // A single vertex is its own median, so rotating it moves it nowhere.
    CHECK(near(mesh.positions[0], Math::vec3(100.0f, 0.0f, 100.0f)));
}

// The selection comes from a BlenderSelection that may outlive the mesh it
// was made against - deleting vertices then transforming must not read past
// the end of the position array.
void testOutOfRangeIndicesAreIgnored()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeOffsetQuad();
    const std::vector<Math::vec3> before = mesh.positions;

    const std::vector<u32> selection = {0, 9999, 4};
    assets.transformVertices(mesh, Math::translate(Math::mat4(1.0f), Math::vec3(1.0f, 0.0f, 0.0f)),
                             selection);

    // Only vertex 0 was real, and it is its own median, so a translation
    // still moves it and nothing else.
    CHECK(near(mesh.positions[0], before[0] + Math::vec3(1.0f, 0.0f, 0.0f)));
    CHECK(near(mesh.positions[1], before[1]));

    // A selection of nothing but garbage leaves the mesh untouched.
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

// Scaling by one is the identity, whatever the pivot maths does in between.
void testIdentityChangesNothing()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeOffsetQuad();
    const std::vector<Math::vec3> before = mesh.positions;

    assets.transformVertices(mesh, Math::mat4(1.0f));

    for (usize i = 0; i < mesh.positions.size(); ++i)
        CHECK(near(mesh.positions[i], before[i], 1e-3f));
}

// A mirrored whole-mesh transform turns the geometry inside out; the winding
// has to be reversed to match, or every face renders backwards.
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
