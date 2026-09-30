#include "PCH.h"
#include "mesh/MeshPaint.h"

#include <algorithm>
#include <cmath>

namespace Radion::MeshPaint
{

namespace
{
constexpr u32 kWhite = 0xFFFFFFFFu;

void ensureColors(MeshData& mesh)
{
    if (mesh.colors.size() != mesh.positions.size())
        mesh.colors.resize(mesh.positions.size(), kWhite);
}

f32 channelToLinear(f32 value)
{
    return value <= 0.04045f ? value / 12.92f : std::pow((value + 0.055f) / 1.055f, 2.4f);
}
} // namespace

Math::vec3 toLinear(const Math::vec3& srgb)
{
    return Math::vec3(channelToLinear(srgb.x), channelToLinear(srgb.y), channelToLinear(srgb.z));
}

Math::vec4 unpack(u32 packed)
{
    return Math::vec4(static_cast<f32>(packed & 0xFF), static_cast<f32>((packed >> 8) & 0xFF),
                     static_cast<f32>((packed >> 16) & 0xFF), static_cast<f32>((packed >> 24) & 0xFF)) /
           255.0f;
}

u32 pack(const Math::vec4& color)
{
    const Math::vec4 clamped = Math::clamp(color, Math::vec4(0.0f), Math::vec4(1.0f));
    u32 packed = 0;
    for (int channel = 0; channel < 4; ++channel)
        packed |= static_cast<u32>(std::lround(clamped[channel] * 255.0f)) << (8 * channel);
    return packed;
}

u32 paintVertices(MeshData& mesh, const std::vector<u32>& vertices, const Math::vec4& color, f32 opacity)
{
    opacity = Math::clamp(opacity, 0.0f, 1.0f);
    ensureColors(mesh);
    u32 changed = 0;
    for (const u32 vertex : vertices)
    {
        if (vertex >= mesh.colors.size())
            continue;
        const u32 before = mesh.colors[vertex];
        mesh.colors[vertex] = pack(Math::mix(unpack(before), color, opacity));
        if (mesh.colors[vertex] != before)
            ++changed;
    }
    return changed;
}

u32 paintSphere(MeshData& mesh, const std::vector<u32>* subset, const Math::vec3& center, f32 radius, f32 hardness,
                const Math::vec4& color, f32 opacity)
{
    if (radius <= 0.0f)
        return 0;
    hardness = Math::clamp(hardness, 0.0f, 1.0f);
    ensureColors(mesh);

    u32 changed = 0;
    auto paint = [&](u32 vertex)
    {
        if (vertex >= mesh.positions.size())
            return;
        const f32 distance = Math::length(mesh.positions[vertex] - center) / radius;
        if (distance >= 1.0f)
            return;
        // Flat out to `hardness`, smoothstep to nothing at the rim.
        f32 falloff = 1.0f;
        if (distance > hardness)
        {
            const f32 t = (distance - hardness) / std::max(1.0f - hardness, 1e-6f);
            falloff = 1.0f - t * t * (3.0f - 2.0f * t);
        }
        const u32 before = mesh.colors[vertex];
        mesh.colors[vertex] = pack(Math::mix(unpack(before), color, Math::clamp(opacity, 0.0f, 1.0f) * falloff));
        if (mesh.colors[vertex] != before)
            ++changed;
    };

    if (subset)
    {
        for (const u32 vertex : *subset)
            paint(vertex);
    }
    else
    {
        for (u32 vertex = 0; vertex < static_cast<u32>(mesh.positions.size()); ++vertex)
            paint(vertex);
    }
    return changed;
}

void clear(MeshData& mesh, const std::vector<u32>* vertices)
{
    if (!vertices)
    {
        mesh.colors.clear();
        return;
    }
    if (mesh.colors.size() != mesh.positions.size())
        return;
    for (const u32 vertex : *vertices)
    {
        if (vertex < mesh.colors.size())
            mesh.colors[vertex] = kWhite;
    }
    if (!hasColors(mesh))
        mesh.colors.clear();
}

bool hasColors(const MeshData& mesh)
{
    if (mesh.colors.size() != mesh.positions.size())
        return false;
    return std::any_of(mesh.colors.begin(), mesh.colors.end(), [](u32 c) { return c != kWhite; });
}

} // namespace Radion::MeshPaint
