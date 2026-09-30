#include "PCH.h"

#include "mesh/MeshPaint.h"

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
        std::fprintf(stderr, "MeshPaintTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

// Five vertices on the X axis at 0, 1, 2, 3, 4.
MeshData line()
{
    MeshData mesh;
    for (int i = 0; i < 5; ++i)
        mesh.positions.push_back(glm::vec3(static_cast<f32>(i), 0, 0));
    return mesh;
}

void testPackRoundTrip()
{
    const u32 packed = MeshPaint::pack(glm::vec4(1.0f, 0.0f, 0.5f, 1.0f));
    CHECK((packed & 0xFF) == 255);            // r is the first byte in memory
    CHECK(((packed >> 8) & 0xFF) == 0);
    CHECK(((packed >> 16) & 0xFF) == 128);
    CHECK(((packed >> 24) & 0xFF) == 255);
    const glm::vec4 back = MeshPaint::unpack(packed);
    CHECK(std::fabs(back.b - 128.0f / 255.0f) < 1e-6f);
    CHECK(MeshPaint::pack(glm::vec4(2.0f, -1.0f, 0.0f, 1.0f)) == MeshPaint::pack(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)));
}

void testSrgbToLinear()
{
    const glm::vec3 mid = MeshPaint::toLinear(glm::vec3(0.5f));
    CHECK(std::fabs(mid.r - 0.2140f) < 1e-3f);
    const glm::vec3 ends = MeshPaint::toLinear(glm::vec3(0.0f, 1.0f, 0.0f));
    CHECK(ends.r == 0.0f && std::fabs(ends.g - 1.0f) < 1e-6f);
}

void testPaintVertices()
{
    MeshData mesh = line();
    CHECK(!MeshPaint::hasColors(mesh));
    const glm::vec4 red(1, 0, 0, 1);
    CHECK(MeshPaint::paintVertices(mesh, {1, 3, 99}, red, 1.0f) == 2); // 99 is ignored
    CHECK(mesh.colors.size() == 5);
    CHECK(mesh.colors[0] == 0xFFFFFFFFu && mesh.colors[2] == 0xFFFFFFFFu);
    CHECK(MeshPaint::unpack(mesh.colors[1]).g == 0.0f);
    CHECK(MeshPaint::hasColors(mesh));

    // Half opacity from white towards black.
    CHECK(MeshPaint::paintVertices(mesh, {0}, glm::vec4(0, 0, 0, 1), 0.5f) == 1);
    CHECK(std::fabs(MeshPaint::unpack(mesh.colors[0]).r - 0.5f) < 1.0f / 255.0f);
    // Painting the colour a vertex already has changes nothing.
    CHECK(MeshPaint::paintVertices(mesh, {1}, red, 1.0f) == 0);
}

void testSphereBrush()
{
    MeshData mesh = line();
    const glm::vec4 blue(0, 0, 1, 1);
    // Hard brush: full colour inside, nothing outside the radius.
    CHECK(MeshPaint::paintSphere(mesh, nullptr, glm::vec3(2, 0, 0), 1.5f, 1.0f, blue, 1.0f) == 3);
    CHECK(MeshPaint::unpack(mesh.colors[2]).r == 0.0f);
    CHECK(MeshPaint::unpack(mesh.colors[1]).r == 0.0f);
    CHECK(mesh.colors[0] == 0xFFFFFFFFu && mesh.colors[4] == 0xFFFFFFFFu);

    // Soft brush: the centre is full, the rim is a blend, beyond the radius untouched.
    MeshData soft = line();
    MeshPaint::paintSphere(soft, nullptr, glm::vec3(2, 0, 0), 2.0f, 0.0f, blue, 1.0f);
    const f32 centre = MeshPaint::unpack(soft.colors[2]).r;
    const f32 near = MeshPaint::unpack(soft.colors[3]).r;
    CHECK(centre == 0.0f);
    CHECK(near > 0.0f && near < 1.0f);
    CHECK(soft.colors[0] == 0xFFFFFFFFu); // exactly at the radius: untouched

    // A subset limits the brush.
    MeshData sub = line();
    const std::vector<u32> only = {2};
    CHECK(MeshPaint::paintSphere(sub, &only, glm::vec3(2, 0, 0), 5.0f, 1.0f, blue, 1.0f) == 1);
    CHECK(MeshPaint::paintSphere(sub, nullptr, glm::vec3(2, 0, 0), 0.0f, 1.0f, blue, 1.0f) == 0);
}

void testClear()
{
    MeshData mesh = line();
    MeshPaint::paintVertices(mesh, {1, 2}, glm::vec4(1, 0, 0, 1), 1.0f);
    const std::vector<u32> one = {1};
    MeshPaint::clear(mesh, &one);
    CHECK(mesh.colors.size() == 5 && mesh.colors[1] == 0xFFFFFFFFu && mesh.colors[2] != 0xFFFFFFFFu);
    const std::vector<u32> two = {2};
    MeshPaint::clear(mesh, &two);
    CHECK(mesh.colors.empty()); // nothing left but white: the array is dropped
    MeshPaint::paintVertices(mesh, {0}, glm::vec4(0, 1, 0, 1), 1.0f);
    MeshPaint::clear(mesh, nullptr);
    CHECK(mesh.colors.empty());
}
} // namespace

int main()
{
    testPackRoundTrip();
    testSrgbToLinear();
    testPaintVertices();
    testSphereBrush();
    testClear();
    if (gFailures)
        std::fprintf(stderr, "%d mesh paint test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
