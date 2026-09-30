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

MeshData line()
{
    MeshData mesh;
    for (int i = 0; i < 5; ++i)
        mesh.positions.push_back(Math::vec3(static_cast<f32>(i), 0, 0));
    return mesh;
}

void testPackRoundTrip()
{
    const u32 packed = MeshPaint::pack(Math::vec4(1.0f, 0.0f, 0.5f, 1.0f));
    CHECK((packed & 0xFF) == 255);            // r is the first byte in memory
    CHECK(((packed >> 8) & 0xFF) == 0);
    CHECK(((packed >> 16) & 0xFF) == 128);
    CHECK(((packed >> 24) & 0xFF) == 255);
    const Math::vec4 back = MeshPaint::unpack(packed);
    CHECK(std::fabs(back.z - 128.0f / 255.0f) < 1e-6f);
    CHECK(MeshPaint::pack(Math::vec4(2.0f, -1.0f, 0.0f, 1.0f)) == MeshPaint::pack(Math::vec4(1.0f, 0.0f, 0.0f, 1.0f)));
}

void testSrgbToLinear()
{
    const Math::vec3 mid = MeshPaint::toLinear(Math::vec3(0.5f));
    CHECK(std::fabs(mid.x - 0.2140f) < 1e-3f);
    const Math::vec3 ends = MeshPaint::toLinear(Math::vec3(0.0f, 1.0f, 0.0f));
    CHECK(ends.x == 0.0f && std::fabs(ends.y - 1.0f) < 1e-6f);
}

void testPaintVertices()
{
    MeshData mesh = line();
    CHECK(!MeshPaint::hasColors(mesh));
    const Math::vec4 red(1, 0, 0, 1);
    CHECK(MeshPaint::paintVertices(mesh, {1, 3, 99}, red, 1.0f) == 2);
    CHECK(mesh.colors.size() == 5);
    CHECK(mesh.colors[0] == 0xFFFFFFFFu && mesh.colors[2] == 0xFFFFFFFFu);
    CHECK(MeshPaint::unpack(mesh.colors[1]).y == 0.0f);
    CHECK(MeshPaint::hasColors(mesh));

    CHECK(MeshPaint::paintVertices(mesh, {0}, Math::vec4(0, 0, 0, 1), 0.5f) == 1);
    CHECK(std::fabs(MeshPaint::unpack(mesh.colors[0]).x - 0.5f) < 1.0f / 255.0f);
    CHECK(MeshPaint::paintVertices(mesh, {1}, red, 1.0f) == 0);
}

void testSphereBrush()
{
    MeshData mesh = line();
    const Math::vec4 blue(0, 0, 1, 1);
    CHECK(MeshPaint::paintSphere(mesh, nullptr, Math::vec3(2, 0, 0), 1.5f, 1.0f, blue, 1.0f) == 3);
    CHECK(MeshPaint::unpack(mesh.colors[2]).x == 0.0f);
    CHECK(MeshPaint::unpack(mesh.colors[1]).x == 0.0f);
    CHECK(mesh.colors[0] == 0xFFFFFFFFu && mesh.colors[4] == 0xFFFFFFFFu);

    MeshData soft = line();
    MeshPaint::paintSphere(soft, nullptr, Math::vec3(2, 0, 0), 2.0f, 0.0f, blue, 1.0f);
    const f32 centre = MeshPaint::unpack(soft.colors[2]).x;
    const f32 near = MeshPaint::unpack(soft.colors[3]).x;
    CHECK(centre == 0.0f);
    CHECK(near > 0.0f && near < 1.0f);
    CHECK(soft.colors[0] == 0xFFFFFFFFu);

    MeshData sub = line();
    const std::vector<u32> only = {2};
    CHECK(MeshPaint::paintSphere(sub, &only, Math::vec3(2, 0, 0), 5.0f, 1.0f, blue, 1.0f) == 1);
    CHECK(MeshPaint::paintSphere(sub, nullptr, Math::vec3(2, 0, 0), 0.0f, 1.0f, blue, 1.0f) == 0);
}

void testClear()
{
    MeshData mesh = line();
    MeshPaint::paintVertices(mesh, {1, 2}, Math::vec4(1, 0, 0, 1), 1.0f);
    const std::vector<u32> one = {1};
    MeshPaint::clear(mesh, &one);
    CHECK(mesh.colors.size() == 5 && mesh.colors[1] == 0xFFFFFFFFu && mesh.colors[2] != 0xFFFFFFFFu);
    const std::vector<u32> two = {2};
    MeshPaint::clear(mesh, &two);
    CHECK(mesh.colors.empty());
    MeshPaint::paintVertices(mesh, {0}, Math::vec4(0, 1, 0, 1), 1.0f);
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
