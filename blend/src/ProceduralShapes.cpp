#include "PCH.h"
#include "ProceduralShapes.h"

#include "mesh/MeshEdit.h"

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

// One ring of `columns + 1` vertices; the last repeats the first so the UV seam has its own vertex.
struct Surface
{
    std::vector<Math::vec3> positions;
    std::vector<Math::vec2> uvs;
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

// Joins neighbouring rings; a pinched ring gives one triangle per column, not a degenerate quad. `flip` reverses
// winding.
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

// A flat disc closing ring `ring`, facing `outward` (+1/-1 along its winding), with its own vertices so the normal stays
// flat.
void addCap(Surface& surface, MeshData& out, u32 ring, bool facesBackward, bool flip)
{
    const u32 centerIndex = static_cast<u32>(out.positions.size());
    Math::vec3 center(0.0f);
    for (u32 column = 0; column < surface.columns; ++column)
        center += surface.positions[surface.vertex(ring, column)];
    center /= static_cast<f32>(surface.columns);

    out.positions.push_back(center);
    out.uvs.push_back(Math::vec2(0.5f, 0.5f));

    const u32 firstRim = static_cast<u32>(out.positions.size());
    for (u32 column = 0; column < surface.columns; ++column)
    {
        const Math::vec3 position = surface.positions[surface.vertex(ring, column)];
        out.positions.push_back(position);
        const Math::vec3 fromCenter = position - center;
        out.uvs.push_back(Math::vec2(0.5f + 0.5f * fromCenter.x, 0.5f + 0.5f * fromCenter.z));
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

// Seam and pinched-ring vertices share positions, so their normals are merged or a shading crack follows the seam.
void computeNormals(const Surface& surface, MeshData& out)
{
    out.normals.assign(out.positions.size(), Math::vec3(0.0f));
    for (usize i = 0; i + 2 < out.indices.size(); i += 3)
    {
        const u32 i0 = out.indices[i];
        const u32 i1 = out.indices[i + 1];
        const u32 i2 = out.indices[i + 2];
        // Not normalised: the cross product's length is twice the area, the right weight for a smooth normal.
        const Math::vec3 faceNormal = Math::cross(out.positions[i1] - out.positions[i0],
                                                out.positions[i2] - out.positions[i0]);
        out.normals[i0] += faceNormal;
        out.normals[i1] += faceNormal;
        out.normals[i2] += faceNormal;
    }

    for (u32 ring = 0; ring < surface.rings; ++ring)
    {
        if (surface.pinched[ring])
        {
            Math::vec3 sum(0.0f);
            for (u32 column = 0; column <= surface.columns; ++column)
                sum += out.normals[surface.vertex(ring, column)];
            for (u32 column = 0; column <= surface.columns; ++column)
                out.normals[surface.vertex(ring, column)] = sum;
        }
        else
        {
            const u32 first = surface.vertex(ring, 0);
            const u32 last = surface.vertex(ring, surface.columns);
            const Math::vec3 seam = out.normals[first] + out.normals[last];
            out.normals[first] = seam;
            out.normals[last] = seam;
        }
    }

    for (Math::vec3& normal : out.normals)
    {
        const f32 length = Math::length(normal);
        normal = length > 1.0e-12f ? normal / length : Math::vec3(0.0f, 1.0f, 0.0f);
    }
}

void finish(Surface& surface, MeshData& out, bool capStart, bool capEnd, bool flip)
{
    out.positions = surface.positions;
    out.uvs = surface.uvs;
    out.indices = surface.indices;

    // Caps come after the side so its ring vertices keep their numbers.
    const usize sideVertexCount = out.positions.size();
    if (capStart && !surface.pinched.front())
        addCap(surface, out, 0, true, flip);
    if (capEnd && !surface.pinched.back())
        addCap(surface, out, surface.rings - 1, false, flip);

    computeNormals(surface, out);
    // The caps get their flat normal, since no other triangle shares their vertices.
    (void)sideVertexCount;

    SubMesh submesh;
    submesh.indexOffset = 0;
    submesh.indexCount = static_cast<u32>(out.indices.size());
    out.submeshes.assign(1, submesh);
    out.materials.assign(1, Material());
    out.bounds = AABB();
    for (const Math::vec3& position : out.positions)
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

    for (const Math::vec2& point : params.profile)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
            return fail(error, "profile points must be finite numbers");
        // A hair below zero is rounding from the profile (sin(pi)), not a negative radius.
        if (point.x < -kPinchEpsilon)
            return fail(error, "profile radii must not be negative");
    }

    Surface surface;
    surface.columns = params.slices;
    surface.rings = static_cast<u32>(params.profile.size());

    // Length along the profile drives V, so a long taper is not textured like a stubby section.
    std::vector<f32> along(params.profile.size(), 0.0f);
    for (usize i = 1; i < params.profile.size(); ++i)
        along[i] = along[i - 1] + Math::length(params.profile[i] - params.profile[i - 1]);
    const f32 total = along.back() > 0.0f ? along.back() : 1.0f;

    for (u32 ring = 0; ring < surface.rings; ++ring)
    {
        Math::vec2 point = params.profile[ring];
        point.x = std::max(point.x, 0.0f);
        surface.pinched.push_back(point.x <= kPinchEpsilon);
        for (u32 column = 0; column <= surface.columns; ++column)
        {
            const f32 u = static_cast<f32>(column) / static_cast<f32>(surface.columns);
            const f32 angle = u * 2.0f * kPi;
            surface.positions.push_back(
                Math::vec3(point.x * std::cos(angle), point.y, point.x * std::sin(angle)));
            surface.uvs.push_back(Math::vec2(u, along[ring] / total));
        }
    }

    // The angle runs +X toward +Z (clockwise from above), so a rising profile's triangles must be turned to face
    // outward.
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

            Math::vec3 position(0.0f);
            position[params.axis] = section.at;
            position[firstAxis] = section.offset.x + 0.5f * section.width * x;
            position[secondAxis] = section.offset.y + 0.5f * section.height * y;
            surface.positions.push_back(position);
            surface.uvs.push_back(Math::vec2(u, v));
        }
    }

    // Stitch direction depends on whether (first, second, axis) is right-handed; axis Y gives (X, Z, Y), left-handed.
    const bool flip = params.axis == 1;
    stitchRings(surface, flip);
    finish(surface, out, params.capStart, params.capEnd, flip);
    return true;
}

namespace
{
void finishSingleSubmesh(MeshData& out)
{
    SubMesh submesh;
    submesh.indexOffset = 0;
    submesh.indexCount = static_cast<u32>(out.indices.size());
    out.submeshes.assign(1, submesh);
    out.materials.assign(1, Material());
    out.bounds = AABB();
    for (const Math::vec3& position : out.positions)
        out.bounds.expand(position);
    out.submeshes[0].bounds = out.bounds;
}

// a, b, c, d counter-clockwise seen from outside.
void addFlatQuad(MeshData& out, const Math::vec3& a, const Math::vec3& b, const Math::vec3& c, const Math::vec3& d,
                 const Math::vec2& uvA, const Math::vec2& uvB, const Math::vec2& uvC, const Math::vec2& uvD)
{
    const Math::vec3 normal = Math::normalize(Math::cross(b - a, c - a));
    const u32 base = static_cast<u32>(out.positions.size());
    for (const Math::vec3& p : {a, b, c, d})
    {
        out.positions.push_back(p);
        out.normals.push_back(normal);
    }
    out.uvs.insert(out.uvs.end(), {uvA, uvB, uvC, uvD});
    out.indices.insert(out.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
}
} // namespace

bool Radion::buildExtrusion(const ExtrusionParams& params, MeshData& out, std::string* error)
{
    const usize n = params.profile.size();
    if (n < 3)
        return fail(error, "an outline needs at least 3 points");
    if (n > 512)
        return fail(error, "an outline may have at most 512 points");
    if (!(params.depth > 0.0f) || !std::isfinite(params.depth))
        return fail(error, "the depth must be greater than zero");
    for (const Math::vec2& p : params.profile)
        if (!std::isfinite(p.x) || !std::isfinite(p.y))
            return fail(error, "outline points must be finite numbers");

    // Counter-clockwise, whichever way it came in.
    std::vector<Math::vec2> profile = params.profile;
    f64 area = 0.0;
    for (usize i = 0; i < n; ++i)
    {
        const Math::vec2& a = profile[i];
        const Math::vec2& b = profile[(i + 1) % n];
        area += static_cast<f64>(a.x) * b.y - static_cast<f64>(b.x) * a.y;
    }
    if (std::abs(area) < 1.0e-12)
        return fail(error, "the outline encloses no area");
    if (area < 0.0)
        std::reverse(profile.begin(), profile.end());

    const f32 half = params.depth * 0.5f;
    MeshData mesh;

    f32 perimeter = 0.0f;
    for (usize i = 0; i < n; ++i)
        perimeter += Math::distance(profile[i], profile[(i + 1) % n]);
    f32 walked = 0.0f;
    for (usize i = 0; i < n; ++i)
    {
        const Math::vec2& p = profile[i];
        const Math::vec2& q = profile[(i + 1) % n];
        const f32 length = Math::distance(p, q);
        if (length < 1.0e-9f)
            continue;
        const f32 u0 = walked / perimeter;
        const f32 u1 = (walked + length) / perimeter;
        walked += length;
        addFlatQuad(mesh, Math::vec3(p, -half), Math::vec3(q, -half), Math::vec3(q, half), Math::vec3(p, half),
                    Math::vec2(u0, 0.0f), Math::vec2(u1, 0.0f), Math::vec2(u1, 1.0f), Math::vec2(u0, 1.0f));
    }

    std::vector<Math::vec3> ring(n);
    for (usize i = 0; i < n; ++i)
        ring[i] = Math::vec3(profile[i], 0.0f);
    const auto triangles = MeshEdit::triangulatePolygon(ring);
    for (const f32 z : {half, -half})
    {
        const u32 base = static_cast<u32>(mesh.positions.size());
        for (usize i = 0; i < n; ++i)
        {
            mesh.positions.push_back(Math::vec3(profile[i], z));
            mesh.normals.push_back(Math::vec3(0.0f, 0.0f, z > 0.0f ? 1.0f : -1.0f));
            mesh.uvs.push_back(profile[i]);
        }
        for (const auto& tri : triangles)
        {
            if (z > 0.0f)
                mesh.indices.insert(mesh.indices.end(), {base + tri[0], base + tri[1], base + tri[2]});
            else
                mesh.indices.insert(mesh.indices.end(), {base + tri[0], base + tri[2], base + tri[1]});
        }
    }

    finishSingleSubmesh(mesh);
    out = std::move(mesh);
    return true;
}

bool Radion::buildDisc(const DiscParams& params, MeshData& out, std::string* error)
{
    if (!(params.radius > 0.0f) || !std::isfinite(params.radius))
        return fail(error, "the radius must be greater than zero");
    if (params.slices < 3 || params.slices > 256)
        return fail(error, "slices must be between 3 and 256");

    MeshData mesh;
    mesh.positions.push_back(Math::vec3(0.0f));
    mesh.normals.push_back(Math::vec3(0.0f, 1.0f, 0.0f));
    mesh.uvs.push_back(Math::vec2(0.5f));
    for (u32 i = 0; i < params.slices; ++i)
    {
        const f32 angle = 2.0f * kPi * static_cast<f32>(i) / static_cast<f32>(params.slices);
        const Math::vec3 p(params.radius * std::cos(angle), 0.0f, params.radius * std::sin(angle));
        mesh.positions.push_back(p);
        mesh.normals.push_back(Math::vec3(0.0f, 1.0f, 0.0f));
        mesh.uvs.push_back(Math::vec2(0.5f + 0.5f * std::cos(angle), 0.5f + 0.5f * std::sin(angle)));
    }
    // Angle runs +X toward +Z, clockwise seen from above; up-facing needs the reverse.
    for (u32 i = 0; i < params.slices; ++i)
        mesh.indices.insert(mesh.indices.end(), {0, 1 + (i + 1) % params.slices, 1 + i});

    finishSingleSubmesh(mesh);
    out = std::move(mesh);
    return true;
}

bool Radion::buildTube(const TubeParams& params, MeshData& out, std::string* error)
{
    if (!(params.outerRadius > 0.0f) || !(params.innerRadius > 0.0f) || !(params.height > 0.0f) ||
        !std::isfinite(params.outerRadius) || !std::isfinite(params.innerRadius) || !std::isfinite(params.height))
        return fail(error, "radii and height must be greater than zero");
    if (params.innerRadius >= params.outerRadius)
        return fail(error, "the inner radius must be smaller than the outer");
    if (params.slices < 3 || params.slices > 256)
        return fail(error, "slices must be between 3 and 256");

    // Four bands, each a ring of quads with its own vertices so the edges stay sharp.
    MeshData mesh;
    const f32 half = params.height * 0.5f;
    auto ring = [&](f32 radius, f32 y, f32 angle) { return Math::vec3(radius * std::cos(angle), y, radius * std::sin(angle)); };
    for (u32 i = 0; i < params.slices; ++i)
    {
        const f32 a0 = 2.0f * kPi * static_cast<f32>(i) / static_cast<f32>(params.slices);
        const f32 a1 = 2.0f * kPi * static_cast<f32>(i + 1) / static_cast<f32>(params.slices);
        const f32 u0 = static_cast<f32>(i) / static_cast<f32>(params.slices);
        const f32 u1 = static_cast<f32>(i + 1) / static_cast<f32>(params.slices);
        // The angle runs clockwise seen from above, which decides each quad's vertex order.
        addFlatQuad(mesh, ring(params.outerRadius, -half, a1), ring(params.outerRadius, -half, a0),
                    ring(params.outerRadius, half, a0), ring(params.outerRadius, half, a1), {u1, 0}, {u0, 0}, {u0, 1}, {u1, 1});
        addFlatQuad(mesh, ring(params.innerRadius, -half, a0), ring(params.innerRadius, -half, a1),
                    ring(params.innerRadius, half, a1), ring(params.innerRadius, half, a0), {u0, 0}, {u1, 0}, {u1, 1}, {u0, 1});
        addFlatQuad(mesh, ring(params.innerRadius, half, a0), ring(params.innerRadius, half, a1),
                    ring(params.outerRadius, half, a1), ring(params.outerRadius, half, a0), {u0, 0}, {u1, 0}, {u1, 1}, {u0, 1});
        addFlatQuad(mesh, ring(params.innerRadius, -half, a1), ring(params.innerRadius, -half, a0),
                    ring(params.outerRadius, -half, a0), ring(params.outerRadius, -half, a1), {u1, 0}, {u0, 0}, {u0, 1}, {u1, 1});
    }

    finishSingleSubmesh(mesh);
    out = std::move(mesh);
    return true;
}

bool Radion::buildPrism(const PrismParams& params, MeshData& out, std::string* error)
{
    if (params.sides < 3 || params.sides > 64)
        return fail(error, "sides must be between 3 and 64");
    if (!(params.radius > 0.0f) || !(params.height > 0.0f) || !std::isfinite(params.radius) ||
        !std::isfinite(params.height))
        return fail(error, "radius and height must be greater than zero");

    // Outline in X-Y pushed along Z, then stood up so the height runs along Y.
    ExtrusionParams extrusion;
    extrusion.depth = params.height;
    for (u32 i = 0; i < params.sides; ++i)
    {
        const f32 angle = 2.0f * kPi * static_cast<f32>(i) / static_cast<f32>(params.sides);
        extrusion.profile.push_back(Math::vec2(params.radius * std::cos(angle), params.radius * std::sin(angle)));
    }
    MeshData mesh;
    if (!buildExtrusion(extrusion, mesh, error))
        return false;

    // Rotate -90 degrees about X: (x, y, z) -> (x, z, -y). A rotation keeps the winding.
    for (Math::vec3& p : mesh.positions)
        p = Math::vec3(p.x, p.z, -p.y);
    for (Math::vec3& n : mesh.normals)
        n = Math::vec3(n.x, n.z, -n.y);
    finishSingleSubmesh(mesh);
    out = std::move(mesh);
    return true;
}

bool Radion::buildStairs(const StairsParams& params, MeshData& out, std::string* error)
{
    if (params.steps < 1 || params.steps > 128)
        return fail(error, "steps must be between 1 and 128");
    if (!(params.width > 0.0f) || !(params.stepHeight > 0.0f) || !(params.stepDepth > 0.0f) ||
        !std::isfinite(params.width) || !std::isfinite(params.stepHeight) || !std::isfinite(params.stepDepth))
        return fail(error, "width, step height and step depth must be greater than zero");

    const f32 run = params.stepDepth;
    const f32 rise = params.stepHeight;
    const u32 n = params.steps;

    // Side profile, x = run, y = rise: the floor, the back wall, then down the treads.
    ExtrusionParams extrusion;
    extrusion.depth = params.width;
    extrusion.profile = {{0.0f, 0.0f}, {run * n, 0.0f}, {run * n, rise * n}};
    for (u32 i = n; i-- > 0;)
    {
        extrusion.profile.push_back({run * static_cast<f32>(i), rise * static_cast<f32>(i + 1)});
        if (i > 0)
            extrusion.profile.push_back({run * static_cast<f32>(i), rise * static_cast<f32>(i)});
    }
    MeshData mesh;
    if (!buildExtrusion(extrusion, mesh, error))
        return false;

    // Centre the box, then rotate -90 degrees about Y, (x, y, z) -> (-z, y, x): run onto +Z, width onto X.
    const Math::vec3 centre(run * n * 0.5f, rise * n * 0.5f, 0.0f);
    for (Math::vec3& p : mesh.positions)
    {
        p -= centre;
        p = Math::vec3(-p.z, p.y, p.x);
    }
    for (Math::vec3& normal : mesh.normals)
        normal = Math::vec3(-normal.z, normal.y, normal.x);
    finishSingleSubmesh(mesh);
    out = std::move(mesh);
    return true;
}

bool Radion::buildArch(const ArchParams& params, MeshData& out, std::string* error)
{
    if (!(params.width > 0.0f) || !(params.height > 0.0f) || !(params.depth > 0.0f) || !(params.thickness > 0.0f) ||
        !std::isfinite(params.width) || !std::isfinite(params.height) || !std::isfinite(params.depth) ||
        !std::isfinite(params.thickness))
        return fail(error, "width, height, depth and thickness must be greater than zero");
    if (params.segments < 3 || params.segments > 128)
        return fail(error, "segments must be between 3 and 128");

    const f32 outerRadius = params.width * 0.5f;
    const f32 innerRadius = outerRadius - params.thickness;
    if (!(innerRadius > 0.0f))
        return fail(error, "the thickness must be less than half the width");
    if (params.height < outerRadius)
        return fail(error, "the height must be at least half the width (the arch is a semicircle on top)");

    const f32 spring = params.height - outerRadius;
    ExtrusionParams extrusion;
    extrusion.depth = params.depth;
    auto& profile = extrusion.profile;
    profile.push_back({-outerRadius, 0.0f});
    profile.push_back({-outerRadius, spring});
    for (u32 i = 1; i < params.segments; ++i)
    {
        const f32 angle = kPi - kPi * static_cast<f32>(i) / static_cast<f32>(params.segments);
        profile.push_back({outerRadius * std::cos(angle), spring + outerRadius * std::sin(angle)});
    }
    profile.push_back({outerRadius, spring});
    profile.push_back({outerRadius, 0.0f});
    profile.push_back({innerRadius, 0.0f});
    profile.push_back({innerRadius, spring});
    for (u32 i = 1; i < params.segments; ++i)
    {
        const f32 angle = kPi * static_cast<f32>(i) / static_cast<f32>(params.segments);
        profile.push_back({innerRadius * std::cos(angle), spring + innerRadius * std::sin(angle)});
    }
    profile.push_back({-innerRadius, spring});
    profile.push_back({-innerRadius, 0.0f});

    MeshData mesh;
    if (!buildExtrusion(extrusion, mesh, error))
        return false;

    const Math::vec3 centre(0.0f, params.height * 0.5f, 0.0f);
    for (Math::vec3& p : mesh.positions)
        p -= centre;
    finishSingleSubmesh(mesh);
    out = std::move(mesh);
    return true;
}
