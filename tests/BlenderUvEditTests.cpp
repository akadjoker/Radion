#include "PCH.h"

#include "mesh/MeshEdit.h"
#include "mesh/MeshUv.h"

#include <cmath>
#include <cstdio>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "BlenderUvEditTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

bool near(const glm::vec2& a, const glm::vec2& b, f32 tolerance = 1e-4f)
{
    return glm::length(a - b) <= tolerance;
}

// Two quads in UV space that share no vertex: two islands of two triangles.
// Quad A spans (0,0)-(0.25,0.25), quad B spans (0.5,0.5)-(1,1).
MeshData twoIslands()
{
    MeshData mesh;
    mesh.positions = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {2, 0, 0}, {3, 0, 0}, {3, 1, 0}, {2, 1, 0}};
    mesh.uvs = {{0, 0}, {0.25f, 0}, {0.25f, 0.25f}, {0, 0.25f}, {0.5f, 0.5f}, {1, 0.5f}, {1, 1}, {0.5f, 1}};
    mesh.indices = {0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7};
    return mesh;
}

// A unit cube with per-face vertices (24), faces wound outward.
MeshData cube()
{
    MeshData mesh;
    const glm::vec3 normals[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (const glm::vec3& n : normals)
    {
        glm::vec3 u = std::fabs(n.y) > 0.5f ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
        glm::vec3 v = glm::cross(n, u);
        u = glm::cross(v, n);
        const u32 base = static_cast<u32>(mesh.positions.size());
        for (int i = 0; i < 4; ++i)
        {
            const f32 su = (i == 0 || i == 3) ? -0.5f : 0.5f;
            const f32 sv = (i >= 2) ? 0.5f : -0.5f;
            mesh.positions.push_back(n * 0.5f + u * su + v * sv);
            mesh.normals.push_back(n);
        }
        mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    // Wind every face so the geometric normal agrees with the stored one.
    for (usize t = 0; t < mesh.indices.size() / 3; ++t)
    {
        const glm::vec3 g = glm::cross(mesh.positions[mesh.indices[t * 3 + 1]] - mesh.positions[mesh.indices[t * 3]],
                                       mesh.positions[mesh.indices[t * 3 + 2]] - mesh.positions[mesh.indices[t * 3]]);
        if (glm::dot(g, mesh.normals[mesh.indices[t * 3]]) < 0.0f)
            std::swap(mesh.indices[t * 3 + 1], mesh.indices[t * 3 + 2]);
    }
    mesh.uvs.assign(mesh.positions.size(), glm::vec2(0.0f));
    return mesh;
}

void testIslands()
{
    const MeshData mesh = twoIslands();
    u32 count = 0;
    const std::vector<u32> island = MeshUv::islands(mesh, &count);
    CHECK(count == 2);
    CHECK(island.size() == 4 && island[0] == island[1] && island[2] == island[3] && island[0] != island[2]);

    const std::vector<u32> of = MeshUv::islandVertices(mesh, {3});
    CHECK(of == std::vector<u32>({4, 5, 6, 7}));
    CHECK(MeshUv::verticesOfTriangles(mesh, {0, 99}) == std::vector<u32>({0, 1, 2}));

    // One shared vertex merges two quads into one island.
    MeshData joined = mesh;
    joined.indices[6] = 2;
    MeshUv::islands(joined, &count);
    CHECK(count == 1);
}

void testBoundsAndTransform()
{
    MeshData mesh = twoIslands();
    const MeshUv::Rect rect = MeshUv::bounds(mesh, {0, 1, 2, 3});
    CHECK(rect.valid && near(rect.min, {0, 0}) && near(rect.max, {0.25f, 0.25f}));
    CHECK(!MeshUv::bounds(mesh, {}).valid);

    MeshUv::Transform move;
    move.translate = {0.1f, 0.2f};
    CHECK(MeshUv::transform(mesh, {0, 1, 2, 3}, nullptr, rect.center(), move) == 4);
    CHECK(near(mesh.uvs[0], {0.1f, 0.2f}) && near(mesh.uvs[4], {0.5f, 0.5f}));

    // Rotating 90 degrees about the centre keeps the centre, and turns (1,0)-relative into (0,1)-relative.
    MeshData rot = twoIslands();
    MeshUv::Transform turn;
    turn.rotateDegrees = 90.0f;
    MeshUv::transform(rot, {0, 1, 2, 3}, nullptr, {0.125f, 0.125f}, turn);
    CHECK(near(rot.uvs[0], {0.25f, 0.0f}));  // bottom left goes to bottom right
    CHECK(near(rot.uvs[1], {0.25f, 0.25f}));

    // Flip in u about the centre: the quad stays where it is, mirrored.
    MeshData flip = twoIslands();
    MeshUv::Transform mirror;
    mirror.scale = {-1.0f, 1.0f};
    MeshUv::transform(flip, {0, 1, 2, 3}, nullptr, {0.125f, 0.125f}, mirror);
    CHECK(near(flip.uvs[0], {0.25f, 0.0f}) && near(flip.uvs[1], {0.0f, 0.0f}));

    // Pinned vertices stay.
    MeshData pin = twoIslands();
    std::vector<u8> pinned(8, 0);
    pinned[1] = 1;
    CHECK(MeshUv::transform(pin, {0, 1, 2, 3}, &pinned, {0, 0}, move) == 3);
    CHECK(near(pin.uvs[1], {0.25f, 0.0f}));
}

void testFit()
{
    MeshData mesh = twoIslands();
    // The second quad is 0.5 x 0.5 at (0.5..1): fitted with no margin it fills the square.
    CHECK(MeshUv::fit(mesh, {4, 5, 6, 7}, nullptr, true, 0.0f) == 4);
    CHECK(near(mesh.uvs[4], {0, 0}) && near(mesh.uvs[6], {1, 1}));

    // A wide layout keeps its aspect and is centred when asked.
    MeshData wide;
    wide.positions.assign(2, glm::vec3(0));
    wide.uvs = {{0, 0}, {2, 1}};
    MeshUv::fit(wide, {0, 1}, nullptr, true, 0.0f);
    CHECK(near(wide.uvs[0], {0, 0.25f}) && near(wide.uvs[1], {1, 0.75f}));
    MeshData stretch;
    stretch.positions.assign(2, glm::vec3(0));
    stretch.uvs = {{0, 0}, {2, 1}};
    MeshUv::fit(stretch, {0, 1}, nullptr, false, 0.1f);
    CHECK(near(stretch.uvs[0], {0.1f, 0.1f}) && near(stretch.uvs[1], {0.9f, 0.9f}));
    CHECK(MeshUv::fit(stretch, {}, nullptr, true, 0.0f) == 0);
}

void testBoxMap()
{
    MeshData mesh = cube();
    std::vector<u32> all;
    for (u32 t = 0; t < 12; ++t)
        all.push_back(t);
    std::vector<u32> touched;
    const u32 added = MeshUv::boxMap(mesh, all, 1.0f, {0, 0}, &touched);
    CHECK(added == 0); // a cube with per-face vertices needs none
    CHECK(touched.size() == 24);
    // Every face is mapped flat and undistorted: its UV size is 1 x 1.
    for (u32 face = 0; face < 6; ++face)
    {
        std::vector<u32> face4 = {face * 4, face * 4 + 1, face * 4 + 2, face * 4 + 3};
        const MeshUv::Rect rect = MeshUv::bounds(mesh, face4);
        CHECK(near(rect.size(), {1, 1}));
    }

    // Tiling and offset.
    MeshData tiled = cube();
    MeshUv::boxMap(tiled, all, 2.0f, {0.5f, 0.0f});
    CHECK(near(MeshUv::bounds(tiled, {0, 1, 2, 3}).size(), {2, 2}));

    // Welded corners: a single triangle fan over a corner shared by faces of
    // different planes must be split so each plane keeps its UV.
    MeshData welded;
    welded.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    welded.normals.assign(4, glm::vec3(0, 0, 1));
    welded.indices = {0, 1, 2, 0, 3, 1, 0, 2, 3}; // faces facing z, y, x
    const u32 addedWelded = MeshUv::boxMap(welded, {0, 1, 2}, 1.0f, {0, 0}, nullptr);
    CHECK(addedWelded > 0);
    CHECK(welded.positions.size() == 4 + addedWelded);
    CHECK(welded.uvs.size() == welded.positions.size() && welded.normals.size() == welded.positions.size());
    // No triangle shares a vertex with a triangle of another plane now.
    CHECK(MeshUv::islands(welded).size() == 3);
    u32 count = 0;
    MeshUv::islands(welded, &count);
    CHECK(count == 3);

    // Triangles outside the list keep their UVs.
    MeshData partial = twoIslands();
    partial.indices[6] = 2; // triangle 2 now shares vertex 2 with triangle 0
    const glm::vec2 before = partial.uvs[2];
    MeshUv::boxMap(partial, {2, 3}, 1.0f, {0, 0});
    CHECK(near(partial.uvs[2], before));
}

void testRenderLayout()
{
    const MeshData mesh = twoIslands();
    const std::vector<u8> image = MeshUv::renderLayout(mesh, {0, 1, 2, 3}, 64, {}, 0);
    CHECK(image.size() == 64u * 64u * 4u);
    // The wire colour is in the image, and alpha is opaque everywhere.
    bool wire = false;
    bool opaque = true;
    for (usize i = 0; i < image.size(); i += 4)
    {
        if (image[i] == 255 && image[i + 1] == 200 && image[i + 2] == 60)
            wire = true;
        if (image[i + 3] != 255)
            opaque = false;
    }
    CHECK(wire && opaque);

    // With a background the pixels come from it (dimmed): a solid white 4x4 image.
    const std::vector<u8> white(4 * 4 * 4, 255);
    const std::vector<u8> over = MeshUv::renderLayout(mesh, {}, 32, white, 4);
    CHECK(over[(10 * 32 + 10) * 4] == 153);
}
} // namespace

int main()
{
    testIslands();
    testBoundsAndTransform();
    testFit();
    testBoxMap();
    testRenderLayout();
    if (gFailures)
        std::fprintf(stderr, "%d mesh uv test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
