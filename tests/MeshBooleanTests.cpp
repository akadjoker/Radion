#include "PCH.h"

#include "mesh/MeshBoolean.h"

#include <cstdio>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "MeshBooleanTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

// A closed unit-cube-shaped box from `low` to `high`, wound outward.
MeshData box(const Math::vec3& low, const Math::vec3& high)
{
    MeshData mesh;
    for (int i = 0; i < 8; ++i)
        mesh.positions.push_back(Math::vec3((i & 1) ? high.x : low.x, (i & 2) ? high.y : low.y, (i & 4) ? high.z : low.z));
    // Corner index bits: x = 1, y = 2, z = 4. Faces as outward quads.
    const u32 faces[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
    for (const auto& q : faces)
        mesh.indices.insert(mesh.indices.end(), {q[0], q[1], q[2], q[0], q[2], q[3]});
    return mesh;
}

// Volume by the divergence theorem; positive for outward-facing triangles.
f32 signedVolume(const MeshData& mesh)
{
    f32 volume = 0.0f;
    for (usize f = 0; f + 2 < mesh.indices.size(); f += 3)
    {
        const Math::vec3& a = mesh.positions[mesh.indices[f]];
        const Math::vec3& b = mesh.positions[mesh.indices[f + 1]];
        const Math::vec3& c = mesh.positions[mesh.indices[f + 2]];
        volume += Math::dot(a, Math::cross(b, c)) / 6.0f;
    }
    return volume;
}

void testBoxesHaveTheVolumeWeExpect()
{
    // The fixture itself: a unit cube is 1, and faces out.
    CHECK(std::abs(signedVolume(box(Math::vec3(0.0f), Math::vec3(1.0f))) - 1.0f) < 1.0e-5f);
}

void testUnionDifferenceIntersection()
{
    // A unit cube and a slab 2 wide in y and z that overlaps its right half (no
    // coplanar faces, which the field cannot tell apart). Union 1 + 4 - 0.5;
    // the cube with the slab cut out 0.5; what they share 0.5.
    const MeshData a = box(Math::vec3(0.0f), Math::vec3(1.0f));
    const MeshData b = box(Math::vec3(0.5f, -0.5f, -0.5f), Math::vec3(1.5f, 1.5f, 1.5f));

    struct Case
    {
        MeshEdit::BooleanOp op;
        f32 volume;
    };
    for (const Case& test : {Case{MeshEdit::BooleanOp::Union, 4.5f}, Case{MeshEdit::BooleanOp::Difference, 0.5f},
                             Case{MeshEdit::BooleanOp::Intersection, 0.5f}})
    {
        MeshData result;
        std::string error;
        CHECK(MeshEdit::booleanMeshes(a, b, test.op, 64, result, &error));
        CHECK(!result.positions.empty() && !result.indices.empty());
        // A remesh on a grid: close, not exact, and outward facing.
        const f32 volume = signedVolume(result);
        CHECK(std::abs(volume - test.volume) < 0.08f * test.volume);
    }
}

void testBoundsOfTheResult()
{
    const MeshData a = box(Math::vec3(0.0f), Math::vec3(1.0f));
    const MeshData b = box(Math::vec3(0.5f, -0.5f, -0.5f), Math::vec3(1.5f, 1.5f, 1.5f));
    MeshData united;
    CHECK(MeshEdit::booleanMeshes(a, b, MeshEdit::BooleanOp::Union, 48, united));
    Math::vec3 low(1e9f);
    Math::vec3 high(-1e9f);
    for (const Math::vec3& p : united.positions)
    {
        low = Math::min(low, p);
        high = Math::max(high, p);
    }
    // Within a cell of the true box (x 0..1.5, y and z -0.5..1.5).
    const f32 cell = 2.0f / 48.0f;
    CHECK(std::abs(low.x) < cell * 1.5f && std::abs(high.x - 1.5f) < cell * 1.5f);
    CHECK(std::abs(low.y + 0.5f) < cell * 1.5f && std::abs(high.y - 1.5f) < cell * 1.5f);

    // A difference never reaches past the first shape.
    MeshData cut;
    CHECK(MeshEdit::booleanMeshes(a, b, MeshEdit::BooleanOp::Difference, 48, cut));
    f32 maxX = -1e9f;
    for (const Math::vec3& p : cut.positions)
        maxX = std::max(maxX, p.x);
    CHECK(maxX < 0.5f + cell * 1.5f);
}

void testNormalsAndWindingFaceOutward()
{
    const MeshData a = box(Math::vec3(0.0f), Math::vec3(1.0f));
    const MeshData b = box(Math::vec3(0.5f, -0.5f, -0.5f), Math::vec3(1.5f, 1.5f, 1.5f));
    MeshData result;
    CHECK(MeshEdit::booleanMeshes(a, b, MeshEdit::BooleanOp::Intersection, 48, result));
    // The intersection is the box x 0.5..1, y 0..1, z 0..1: every vertex normal
    // points away from its centre (0.75, 0.5, 0.5), and so does every triangle.
    const Math::vec3 centre(0.75f, 0.5f, 0.5f);
    CHECK(result.normals.size() == result.positions.size());
    u32 wrongNormals = 0;
    for (usize i = 0; i < result.positions.size(); ++i)
        if (Math::dot(result.normals[i], result.positions[i] - centre) <= 0.0f)
            ++wrongNormals;
    CHECK(wrongNormals == 0);
    u32 inward = 0;
    for (usize f = 0; f + 2 < result.indices.size(); f += 3)
    {
        const Math::vec3& p0 = result.positions[result.indices[f]];
        const Math::vec3& p1 = result.positions[result.indices[f + 1]];
        const Math::vec3& p2 = result.positions[result.indices[f + 2]];
        if (Math::dot(Math::cross(p1 - p0, p2 - p0), (p0 + p1 + p2) / 3.0f - centre) <= 0.0f)
            ++inward;
    }
    CHECK(inward == 0);
}

void testRefusals()
{
    const MeshData a = box(Math::vec3(0.0f), Math::vec3(1.0f));
    const MeshData far = box(Math::vec3(5.0f), Math::vec3(6.0f));
    MeshData out;
    std::string error;
    // Disjoint: nothing is shared.
    CHECK(!MeshEdit::booleanMeshes(a, far, MeshEdit::BooleanOp::Intersection, 32, out, &error));
    CHECK(error.find("overlap") != std::string::npos);

    MeshData empty;
    CHECK(!MeshEdit::booleanMeshes(empty, a, MeshEdit::BooleanOp::Union, 32, out, &error));
    CHECK(!MeshEdit::booleanMeshes(a, empty, MeshEdit::BooleanOp::Union, 32, out, &error));
    CHECK(!MeshEdit::booleanMeshes(a, a, MeshEdit::BooleanOp::Union, 2, out, &error));
    CHECK(!MeshEdit::booleanMeshes(a, a, MeshEdit::BooleanOp::Union, 100000, out, &error));
    CHECK(out.indices.empty());
}

} // namespace

int main()
{
    testBoxesHaveTheVolumeWeExpect();
    testUnionDifferenceIntersection();
    testBoundsOfTheResult();
    testNormalsAndWindingFaceOutward();
    testRefusals();

    if (gFailures)
        std::fprintf(stderr, "%d mesh boolean test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
