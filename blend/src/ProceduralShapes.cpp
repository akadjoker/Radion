#include "PCH.h"
#include "ProceduralShapes.h"

#include <cmath>

using namespace Radion;

namespace
{
constexpr f32 kPi = 3.14159265358979323846f;
constexpr usize kMaxVertices = 200000;
constexpr f32 kPinchEpsilon = 1.0e-6f;

bool fail(std::string* error, const std::string& message)
{
    if (error)
        *error = message;
    return false;
}

// One ring of `columns + 1` vertices (the last repeats the first so the UV
// seam has its own vertex), described by where each of its points sits.
struct Surface
{
    std::vector<glm::vec3> positions;
    std::vector<glm::vec2> uvs;
    std::vector<u32> indices;
    // A ring whose points all coincide: the surface comes to a point there.
    std::vector<bool> pinched;
    u32 columns = 0;
    u32 rings = 0;

    u32 vertex(u32 ring, u32 column) const
    {
        return ring * (columns + 1) + column;
    }
};

// Joins neighbouring rings. A ring that is pinched contributes a single
// triangle per column instead of a quad, so no degenerate triangle is emitted.
// `flip` reverses the winding for shapes whose rings run the other way round.
void stitchRings(Surface& surface, bool flip)
{
    for (u32 ring = 0; ring + 1 < surface.rings; ++ring)
    {
        const bool lowPinched = surface.pinched[ring];
        const bool highPinched = surface.pinched[ring + 1];
        if (lowPinched && highPinched)
            continue;

        for (u32 column = 0; column < surface.columns; ++column)
        {
            const u32 a0 = surface.vertex(ring, column);
            const u32 a1 = surface.vertex(ring, column + 1);
            const u32 b0 = surface.vertex(ring + 1, column);
            const u32 b1 = surface.vertex(ring + 1, column + 1);

            auto triangle = [&](u32 x, u32 y, u32 z)
            {
                surface.indices.push_back(x);
                surface.indices.push_back(flip ? z : y);
                surface.indices.push_back(flip ? y : z);
            };

            if (lowPinched)
            {
                triangle(a0, b1, b0);
            }
            else if (highPinched)
            {
                triangle(a0, a1, b0);
            }
            else
            {
                triangle(a0, a1, b1);
                triangle(a0, b1, b0);
            }
        }
    }
}

// A flat disc closing the ring `ring`, facing `outward` (+1/-1 along the ring's
// own winding). Its vertices are its own, so its normal stays flat.
void addCap(Surface& surface, MeshData& out, u32 ring, bool facesBackward, bool flip)
{
    const u32 centerIndex = static_cast<u32>(out.positions.size());
    glm::vec3 center(0.0f);
    for (u32 column = 0; column < surface.columns; ++column)
        center += surface.positions[surface.vertex(ring, column)];
    center /= static_cast<f32>(surface.columns);

    out.positions.push_back(center);
    out.uvs.push_back(glm::vec2(0.5f, 0.5f));

    const u32 firstRim = static_cast<u32>(out.positions.size());
    for (u32 column = 0; column < surface.columns; ++column)
    {
        const glm::vec3 position = surface.positions[surface.vertex(ring, column)];
        out.positions.push_back(position);
        const glm::vec3 fromCenter = position - center;
        out.uvs.push_back(glm::vec2(0.5f + 0.5f * fromCenter.x, 0.5f + 0.5f * fromCenter.z));
    }

    for (u32 column = 0; column < surface.columns; ++column)
    {
        const u32 current = firstRim + column;
        const u32 next = firstRim + (column + 1) % surface.columns;
        const bool reverse = facesBackward != flip;
        out.indices.push_back(centerIndex);
        out.indices.push_back(reverse ? next : current);
        out.indices.push_back(reverse ? current : next);
    }
}

// Smooth normals from the triangles. The seam vertices and the points of a
// pinched ring are separate vertices at one position, so their normals are
// merged afterwards or a shading crack would follow the seam.
void computeNormals(const Surface& surface, MeshData& out)
{
    out.normals.assign(out.positions.size(), glm::vec3(0.0f));
    for (usize i = 0; i + 2 < out.indices.size(); i += 3)
    {
        const u32 i0 = out.indices[i];
        const u32 i1 = out.indices[i + 1];
        const u32 i2 = out.indices[i + 2];
        // Not normalised: the cross product's length is twice the area, which
        // is exactly the weight a smooth normal wants.
        const glm::vec3 faceNormal = glm::cross(out.positions[i1] - out.positions[i0],
                                                out.positions[i2] - out.positions[i0]);
        out.normals[i0] += faceNormal;
        out.normals[i1] += faceNormal;
        out.normals[i2] += faceNormal;
    }

    for (u32 ring = 0; ring < surface.rings; ++ring)
    {
        if (surface.pinched[ring])
        {
            glm::vec3 sum(0.0f);
            for (u32 column = 0; column <= surface.columns; ++column)
                sum += out.normals[surface.vertex(ring, column)];
            for (u32 column = 0; column <= surface.columns; ++column)
                out.normals[surface.vertex(ring, column)] = sum;
        }
        else
        {
            const u32 first = surface.vertex(ring, 0);
            const u32 last = surface.vertex(ring, surface.columns);
            const glm::vec3 seam = out.normals[first] + out.normals[last];
            out.normals[first] = seam;
            out.normals[last] = seam;
        }
    }

    for (glm::vec3& normal : out.normals)
    {
        const f32 length = glm::length(normal);
        normal = length > 1.0e-12f ? normal / length : glm::vec3(0.0f, 1.0f, 0.0f);
    }
}

void finish(Surface& surface, MeshData& out, bool capStart, bool capEnd, bool flip)
{
    out.positions = surface.positions;
    out.uvs = surface.uvs;
    out.indices = surface.indices;

    // Caps are added after the side surface so the side's ring vertices keep
    // their numbers; the caps' own vertices come last.
    const usize sideVertexCount = out.positions.size();
    if (capStart && !surface.pinched.front())
        addCap(surface, out, 0, true, flip);
    if (capEnd && !surface.pinched.back())
        addCap(surface, out, surface.rings - 1, false, flip);

    computeNormals(surface, out);
    // computeNormals sized `normals` to every vertex including the caps', and
    // gave the caps whatever their triangles say - which is their flat normal,
    // because no other triangle shares those vertices.
    (void)sideVertexCount;

    SubMesh submesh;
    submesh.indexOffset = 0;
    submesh.indexCount = static_cast<u32>(out.indices.size());
    out.submeshes.assign(1, submesh);
    out.materials.assign(1, Material());
    out.bounds = AABB();
    for (const glm::vec3& position : out.positions)
        out.bounds.expand(position);
    out.submeshes[0].bounds = out.bounds;
}
} // namespace

bool Radion::buildLathe(const LatheParams& params, MeshData& out, std::string* error)
{
    if (params.profile.size() < 2)
        return fail(error, "a lathe profile needs at least 2 points");
    if (params.slices < 3 || params.slices > 256)
        return fail(error, "slices must be between 3 and 256");
    if (static_cast<usize>(params.slices + 1) * params.profile.size() > kMaxVertices)
        return fail(error, "the lathe would have too many vertices");

    for (const glm::vec2& point : params.profile)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
            return fail(error, "profile points must be finite numbers");
        // A hair below zero is rounding from computing the profile (sin(pi)),
        // not a request for a negative radius.
        if (point.x < -kPinchEpsilon)
            return fail(error, "profile radii must not be negative");
    }

    Surface surface;
    surface.columns = params.slices;
    surface.rings = static_cast<u32>(params.profile.size());

    // Length along the profile drives V, so a long tapering section is not
    // textured as if it were as short as a stubby one.
    std::vector<f32> along(params.profile.size(), 0.0f);
    for (usize i = 1; i < params.profile.size(); ++i)
        along[i] = along[i - 1] + glm::length(params.profile[i] - params.profile[i - 1]);
    const f32 total = along.back() > 0.0f ? along.back() : 1.0f;

    for (u32 ring = 0; ring < surface.rings; ++ring)
    {
        glm::vec2 point = params.profile[ring];
        point.x = std::max(point.x, 0.0f);
        surface.pinched.push_back(point.x <= kPinchEpsilon);
        for (u32 column = 0; column <= surface.columns; ++column)
        {
            const f32 u = static_cast<f32>(column) / static_cast<f32>(surface.columns);
            const f32 angle = u * 2.0f * kPi;
            surface.positions.push_back(
                glm::vec3(point.x * std::cos(angle), point.y, point.x * std::sin(angle)));
            surface.uvs.push_back(glm::vec2(u, along[ring] / total));
        }
    }

    // The angle runs from +X toward +Z, which is clockwise seen from above, so
    // a profile that rises along +Y must have its triangles turned round to face
    // outward, and one that descends already does.
    const bool risingProfile = params.profile.back().y >= params.profile.front().y;
    const bool flip = risingProfile;
    stitchRings(surface, flip);
    finish(surface, out, params.capStart, params.capEnd, flip);
    return true;
}

bool Radion::buildLoft(const LoftParams& params, MeshData& out, std::string* error)
{
    if (params.axis < 0 || params.axis > 2)
        return fail(error, "axis must be 0, 1 or 2");
    if (params.sections.size() < 2)
        return fail(error, "a loft needs at least 2 sections");
    if (params.segments < 3 || params.segments > 256)
        return fail(error, "segments must be between 3 and 256");
    if (static_cast<usize>(params.segments + 1) * params.sections.size() > kMaxVertices)
        return fail(error, "the loft would have too many vertices");

    for (usize i = 0; i < params.sections.size(); ++i)
    {
        const LoftSection& section = params.sections[i];
        if (!std::isfinite(section.at) || !std::isfinite(section.width) ||
            !std::isfinite(section.height) || !std::isfinite(section.offset.x) ||
            !std::isfinite(section.offset.y) || !std::isfinite(section.exponent))
            return fail(error, "section values must be finite numbers");
        if (section.width < 0.0f || section.height < 0.0f)
            return fail(error, "section width and height must not be negative");
        if (section.exponent < 0.5f || section.exponent > 16.0f)
            return fail(error, "section exponent must be between 0.5 and 16");
        if (i > 0 && section.at <= params.sections[i - 1].at)
            return fail(error, "sections must be listed in increasing 'at' order");
    }

    // The two axes the cross-section lives in, in x-y-z order.
    const s32 firstAxis = params.axis == 0 ? 1 : 0;
    const s32 secondAxis = params.axis == 2 ? 1 : 2;

    Surface surface;
    surface.columns = params.segments;
    surface.rings = static_cast<u32>(params.sections.size());

    const f32 span = params.sections.back().at - params.sections.front().at;
    for (u32 ring = 0; ring < surface.rings; ++ring)
    {
        const LoftSection& section = params.sections[ring];
        surface.pinched.push_back(section.width <= kPinchEpsilon || section.height <= kPinchEpsilon);
        const f32 v = (section.at - params.sections.front().at) / span;
        const f32 power = 2.0f / section.exponent;

        for (u32 column = 0; column <= surface.columns; ++column)
        {
            const f32 u = static_cast<f32>(column) / static_cast<f32>(surface.columns);
            const f32 angle = u * 2.0f * kPi;
            const f32 c = std::cos(angle);
            const f32 s = std::sin(angle);
            // Superellipse: |x|^n + |y|^n = 1, in its parametric form.
            const f32 x = (c < 0.0f ? -1.0f : 1.0f) * std::pow(std::abs(c), power);
            const f32 y = (s < 0.0f ? -1.0f : 1.0f) * std::pow(std::abs(s), power);

            glm::vec3 position(0.0f);
            position[params.axis] = section.at;
            position[firstAxis] = section.offset.x + 0.5f * section.width * x;
            position[secondAxis] = section.offset.y + 0.5f * section.height * y;
            surface.positions.push_back(position);
            surface.uvs.push_back(glm::vec2(u, v));
        }
    }

    // Which way the rings must be stitched depends on whether (first, second,
    // axis) is a right-handed triple: X,Y,Z and Y,Z,X are; Z,X,Y is too, but
    // the axes above are taken in x-y-z order, so axis Y gives (X, Z, Y) - left
    // handed.
    const bool flip = params.axis == 1;
    stitchRings(surface, flip);
    finish(surface, out, params.capStart, params.capEnd, flip);
    return true;
}
