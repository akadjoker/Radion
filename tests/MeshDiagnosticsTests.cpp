// MeshDiagnosticsTests.cpp - analyzeMesh(): what a mesh is made of and what
// is wrong with it. A diagnostic that reports a fault where there is none, or
// misses one, is worse than no diagnostic, so each fault gets a mesh built to
// carry exactly it. Nothing here touches the GPU.

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
        std::fprintf(stderr, "MeshDiagnosticsTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

// A closed tetrahedron: four vertices, four faces, every edge shared by
// exactly two triangles. The clean baseline every fault below is measured
// against.
MeshData makeTetrahedron()
{
    MeshData mesh;
    mesh.positions = {
        Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(1.0f, 0.0f, 0.0f),
        Math::Vec3(0.0f, 1.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 1.0f),
    };
    mesh.normals.assign(4, Math::Vec3(0.0f, 1.0f, 0.0f));
    mesh.uvs.assign(4, Math::Vec2(0.0f));
    mesh.indices = {0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};

    SubMesh submesh;
    submesh.indexOffset = 0;
    submesh.indexCount = 12;
    mesh.submeshes.push_back(submesh);
    return mesh;
}

void testCleanMeshReportsNothing()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeTetrahedron();

    AssetManager::Diagnostics d;
    assets.analyzeMesh(mesh, d);

    CHECK(d.vertexCount == 4);
    CHECK(d.triangleCount == 4);
    CHECK(d.submeshCount == 1);

    CHECK(d.outOfRangeIndices == 0);
    CHECK(d.degenerateTriangles == 0);
    CHECK(d.orphanVertices == 0);
    CHECK(d.nonManifoldEdges == 0);
    CHECK(d.exactDuplicatePositions == 0);
    CHECK(!d.streamsMismatched);
    CHECK(!d.trianglesTruncated);
    CHECK(!d.submeshRangesInvalid);

    // Closed: every one of its six edges is shared by two faces.
    CHECK(d.boundaryEdges == 0);

    CHECK(d.hasNormals && d.hasUvs);
    CHECK(!d.hasTangents && !d.hasColors && !d.hasSkin && !d.hasUvs2);
    CHECK(d.memoryBytes == mesh.memoryBytes());
}

// An open surface has boundary edges and that is not a fault - reporting it
// as one would cry wolf on every flat plane in existence.
void testOpenSurfaceHasBoundaryEdges()
{
    AssetManager& assets = AssetManager::getSingleton();

    MeshData mesh;
    mesh.positions = {Math::Vec3(0.0f), Math::Vec3(1.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 1.0f)};
    mesh.indices = {0, 2, 1};

    AssetManager::Diagnostics d;
    assets.analyzeMesh(mesh, d);

    CHECK(d.boundaryEdges == 3);
    CHECK(d.nonManifoldEdges == 0);
    CHECK(d.degenerateTriangles == 0);
}

void testDegenerateTriangles()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeTetrahedron();

    // A repeated corner.
    mesh.indices.push_back(0);
    mesh.indices.push_back(1);
    mesh.indices.push_back(1);

    AssetManager::Diagnostics d;
    assets.analyzeMesh(mesh, d);
    CHECK(d.degenerateTriangles == 1);

    // Three distinct vertices on one line: no repeated index, no area. This
    // is the one an index-only check misses.
    MeshData sliver;
    sliver.positions = {Math::Vec3(0.0f), Math::Vec3(1.0f, 0.0f, 0.0f), Math::Vec3(2.0f, 0.0f, 0.0f)};
    sliver.indices = {0, 1, 2};
    assets.analyzeMesh(sliver, d);
    CHECK(d.degenerateTriangles == 1);
    CHECK(d.triangleCount == 1);
}

void testOrphanVertices()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeTetrahedron();

    mesh.positions.push_back(Math::Vec3(9.0f, 9.0f, 9.0f));
    mesh.positions.push_back(Math::Vec3(8.0f, 8.0f, 8.0f));
    mesh.normals.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
    mesh.normals.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
    mesh.uvs.push_back(Math::Vec2(0.0f));
    mesh.uvs.push_back(Math::Vec2(0.0f));

    AssetManager::Diagnostics d;
    assets.analyzeMesh(mesh, d);
    CHECK(d.orphanVertices == 2);
    CHECK(!d.streamsMismatched);
}

void testNonManifoldEdge()
{
    AssetManager& assets = AssetManager::getSingleton();

    // Three triangles hinged on the same edge 0-1.
    MeshData mesh;
    mesh.positions = {
        Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(1.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f),
        Math::Vec3(0.0f, 0.0f, 1.0f), Math::Vec3(0.0f, -1.0f, 0.0f),
    };
    mesh.indices = {0, 1, 2, 0, 1, 3, 0, 1, 4};

    AssetManager::Diagnostics d;
    assets.analyzeMesh(mesh, d);
    CHECK(d.nonManifoldEdges == 1);
    CHECK(d.degenerateTriangles == 0);
}

void testOutOfRangeIndices()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeTetrahedron();
    mesh.indices.push_back(0);
    mesh.indices.push_back(1);
    mesh.indices.push_back(99);

    AssetManager::Diagnostics d;
    assets.analyzeMesh(mesh, d);
    CHECK(d.outOfRangeIndices == 1);
    // The bad triangle is skipped rather than counted as degenerate too.
    CHECK(d.degenerateTriangles == 0);
}

void testDuplicatePositions()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeTetrahedron();

    // Two more vertices on top of vertex 0, both referenced so they are not
    // orphans as well.
    mesh.positions.push_back(mesh.positions[0]);
    mesh.positions.push_back(mesh.positions[0]);
    mesh.normals.resize(mesh.positions.size(), Math::Vec3(0.0f, 1.0f, 0.0f));
    mesh.uvs.resize(mesh.positions.size(), Math::Vec2(0.0f));
    mesh.indices.push_back(4);
    mesh.indices.push_back(5);
    mesh.indices.push_back(1);

    AssetManager::Diagnostics d;
    assets.analyzeMesh(mesh, d);
    // Three vertices share one point: two of them are the duplicates.
    CHECK(d.exactDuplicatePositions == 2);
    CHECK(d.orphanVertices == 0);
}

void testStreamMismatch()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeTetrahedron();
    mesh.normals.pop_back();

    AssetManager::Diagnostics d;
    assets.analyzeMesh(mesh, d);
    CHECK(d.streamsMismatched);
    CHECK(d.hasNormals);

    // A stream that is simply absent is not a mismatch.
    MeshData clean = makeTetrahedron();
    clean.normals.clear();
    assets.analyzeMesh(clean, d);
    CHECK(!d.streamsMismatched);
    CHECK(!d.hasNormals);
}

void testTruncatedAndBadSubmeshes()
{
    AssetManager& assets = AssetManager::getSingleton();

    MeshData truncated = makeTetrahedron();
    truncated.indices.push_back(0);
    AssetManager::Diagnostics d;
    assets.analyzeMesh(truncated, d);
    CHECK(d.trianglesTruncated);
    CHECK(d.triangleCount == 4);

    MeshData overrun = makeTetrahedron();
    overrun.submeshes[0].indexCount = 999;
    assets.analyzeMesh(overrun, d);
    CHECK(d.submeshRangesInvalid);

    MeshData misaligned = makeTetrahedron();
    misaligned.submeshes[0].indexCount = 10;
    assets.analyzeMesh(misaligned, d);
    CHECK(d.submeshRangesInvalid);
}

// Every field has to be written on each call, or a second analysis inherits
// the first one's faults and the panel keeps showing a problem that is fixed.
void testResultDoesNotCarryOver()
{
    AssetManager& assets = AssetManager::getSingleton();

    MeshData broken = makeTetrahedron();
    broken.indices.push_back(0);
    broken.indices.push_back(1);
    broken.indices.push_back(1);

    AssetManager::Diagnostics d;
    assets.analyzeMesh(broken, d);
    CHECK(d.degenerateTriangles == 1);

    MeshData clean = makeTetrahedron();
    assets.analyzeMesh(clean, d);
    CHECK(d.degenerateTriangles == 0);
    CHECK(d.orphanVertices == 0);
    CHECK(d.boundaryEdges == 0);
}

void testEmptyMesh()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh;

    AssetManager::Diagnostics d;
    assets.analyzeMesh(mesh, d);
    CHECK(d.vertexCount == 0);
    CHECK(d.triangleCount == 0);
    CHECK(d.orphanVertices == 0);
    CHECK(d.degenerateTriangles == 0);
    CHECK(!d.streamsMismatched);
}

// It diagnoses; it must not repair, reorder, or otherwise touch the mesh.
void testAnalysisLeavesTheMeshAlone()
{
    AssetManager& assets = AssetManager::getSingleton();
    MeshData mesh = makeTetrahedron();
    const usize vertices = mesh.positions.size();
    const usize indices = mesh.indices.size();

    AssetManager::Diagnostics d;
    assets.analyzeMesh(mesh, d);

    CHECK(mesh.positions.size() == vertices);
    CHECK(mesh.indices.size() == indices);
}

} // namespace

int main()
{
    testCleanMeshReportsNothing();
    testOpenSurfaceHasBoundaryEdges();
    testDegenerateTriangles();
    testOrphanVertices();
    testNonManifoldEdge();
    testOutOfRangeIndices();
    testDuplicatePositions();
    testStreamMismatch();
    testTruncatedAndBadSubmeshes();
    testResultDoesNotCarryOver();
    testEmptyMesh();
    testAnalysisLeavesTheMeshAlone();

    if (gFailures)
        std::fprintf(stderr, "%d mesh diagnostics test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
