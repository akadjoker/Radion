#include "PCH.h"

#include "VolumeMesher.h"


namespace Radion::Volume
{
namespace
{
struct GridSample { f32 density; Math::vec3 gradient; };
struct Hit { Math::vec3 position; Math::vec3 gradient; u64 edge; };

bool finiteVec(const Math::vec3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

u64 edgeKey(u64 a, u64 b)
{
    return a < b ? (a << 32) | b : (b << 32) | a;
}

u32 weldVertex(MeshData& mesh, HashMap<u64, u32>& weld, const Hit& hit,
               const Math::vec3& faceNormal, bool uvs)
{
    const auto found = weld.find(hit.edge);
    if (found != weld.end())
    {
        const u32 index = found->second;
        mesh.normals[index] += Math::length(hit.gradient) > 1e-6f ? Math::normalize(hit.gradient)
                                                                 : faceNormal;
        return index;
    }
    const u32 index = static_cast<u32>(mesh.positions.size());
    mesh.positions.push_back(hit.position);
    mesh.normals.push_back(Math::length(hit.gradient) > 1e-6f ? Math::normalize(hit.gradient)
                                                            : faceNormal);
    if (uvs) mesh.uvs.push_back(Math::vec2(hit.position.x, hit.position.z));
    weld[hit.edge] = index;
    return index;
}

void emitTriangle(MeshData& mesh, HashMap<u64, u32>& weld, const Hit& a, const Hit& b,
                  const Hit& c, bool uvs)
{
    Math::vec3 normal = Math::cross(b.position - a.position, c.position - a.position);
    const Math::vec3 gradient = a.gradient + b.gradient + c.gradient;
    if (Math::dot(normal, gradient) < 0.0f) normal = -normal;
    const u32 i0 = weldVertex(mesh, weld, a, normal, uvs);
    const u32 i1 = weldVertex(mesh, weld, b, normal, uvs);
    const u32 i2 = weldVertex(mesh, weld, c, normal, uvs);
    mesh.indices.insert(mesh.indices.end(), {i0, i1, i2});
}

Hit crossing(const Math::vec3 positions[4], const GridSample samples[4], const u64 points[4],
             f32 iso, u8 a, u8 b)
{
    const f32 da = samples[a].density; const f32 db = samples[b].density;
    const f32 t = (iso - da) / (db - da);
    return {Math::mix(positions[a], positions[b], t),
            Math::mix(samples[a].gradient, samples[b].gradient, t),
            edgeKey(points[a], points[b])};
}

void polygonizeTetra(const Math::vec3 positions[4], const GridSample samples[4],
                     const u64 points[4], f32 iso, MeshData& mesh, HashMap<u64, u32>& weld,
                     bool uvs)
{
    u8 inside[4]; u8 outside[4];
    u32 insideCount = 0; u32 outsideCount = 0;
    for (u8 i = 0; i < 4; ++i)
    {
        if (samples[i].density >= iso) inside[insideCount++] = i;
        else outside[outsideCount++] = i;
    }
    if (insideCount == 0 || outsideCount == 0) return;

    if (insideCount == 1 || outsideCount == 1)
    {
        const u8 apex = insideCount == 1 ? inside[0] : outside[0];
        const u8* others = insideCount == 1 ? outside : inside;
        emitTriangle(mesh, weld, crossing(positions, samples, points, iso, apex, others[0]),
                     crossing(positions, samples, points, iso, apex, others[1]),
                     crossing(positions, samples, points, iso, apex, others[2]), uvs);
        return;
    }

    const u8 p = inside[0]; const u8 q = inside[1];
    const u8 r = outside[0]; const u8 s = outside[1];
    const Hit pr = crossing(positions, samples, points, iso, p, r);
    const Hit ps = crossing(positions, samples, points, iso, p, s);
    const Hit qs = crossing(positions, samples, points, iso, q, s);
    const Hit qr = crossing(positions, samples, points, iso, q, r);
    emitTriangle(mesh, weld, pr, ps, qs, uvs);
    emitTriangle(mesh, weld, pr, qs, qr, uvs);
}
} // namespace

bool buildMesh(const Source& source, const MeshingSettings& settings,
               MeshData& out, MeshingStats* stats)
{
    MeshingStats local{};
    if (stats) *stats = {};
    if (settings.voxelSize <= 0.0f || !std::isfinite(settings.voxelSize) ||
        settings.bounds.empty() || !finiteVec(settings.bounds.min) || !finiteVec(settings.bounds.max) ||
        settings.bounds.min.x > settings.bounds.max.x || settings.bounds.min.y > settings.bounds.max.y ||
        settings.bounds.min.z > settings.bounds.max.z || !std::isfinite(settings.isoLevel)) return false;

    const Math::vec3 extent = settings.bounds.max - settings.bounds.min;
    const Math::uvec3 cells(static_cast<u32>(std::ceil(extent.x / settings.voxelSize)),
                           static_cast<u32>(std::ceil(extent.y / settings.voxelSize)),
                           static_cast<u32>(std::ceil(extent.z / settings.voxelSize)));
    constexpr u64 maxCells = 16ull * 1024ull * 1024ull;
    const u64 cellCount = u64(cells.x) * cells.y * cells.z;
    if (cells.x == 0 || cells.y == 0 || cells.z == 0 || cellCount > maxCells) return false;

    const Math::uvec3 points = cells + Math::uvec3(1);
    const u64 pointCount = u64(points.x) * points.y * points.z;
    if (pointCount > maxCells + 1) return false;
    std::vector<GridSample> grid(static_cast<usize>(pointCount));
    const auto index = [points](u32 x, u32 y, u32 z) { return (u64(z) * points.y + y) * points.x + x; };
    const f32 isoEpsilon = settings.voxelSize * 1e-4f;
    for (u32 z = 0; z < points.z; ++z) for (u32 y = 0; y < points.y; ++y) for (u32 x = 0; x < points.x; ++x)
    {
        const Math::vec3 p = settings.bounds.min + Math::vec3(x, y, z) * settings.voxelSize;
        const Sample sample = source.sample(p);
        f32 density = sample.density;
        if (std::abs(density - settings.isoLevel) < isoEpsilon) density = settings.isoLevel + isoEpsilon;
        grid[static_cast<usize>(index(x,y,z))] = {density, sample.gradient}; ++local.samples;
    }

    MeshData result;
    HashMap<u64, u32> weld;
    static constexpr u8 tetrahedra[6][4] = {{0,1,3,7},{0,1,5,7},{0,2,3,7},
                                            {0,2,6,7},{0,4,5,7},{0,4,6,7}};
    for (u32 z = 0; z < cells.z; ++z) for (u32 y = 0; y < cells.y; ++y) for (u32 x = 0; x < cells.x; ++x)
    {
        Math::vec3 p[8] = {}; GridSample s[8] = {}; u64 g[8] = {};
        for (u32 i = 0; i < 8; ++i)
        {
            const u32 dx = i & 1, dy = (i >> 1) & 1, dz = (i >> 2) & 1;
            p[i] = settings.bounds.min + Math::vec3(x+dx,y+dy,z+dz) * settings.voxelSize;
            g[i] = index(x+dx,y+dy,z+dz);
            s[i] = grid[static_cast<usize>(g[i])];
        }
        for (const auto& tetra : tetrahedra)
        {
            Math::vec3 tp[4]; GridSample ts[4]; u64 tg[4];
            for (u32 i = 0; i < 4; ++i) { tp[i] = p[tetra[i]]; ts[i] = s[tetra[i]]; tg[i] = g[tetra[i]]; }
            const usize before = result.indices.size();
            polygonizeTetra(tp, ts, tg, settings.isoLevel, result, weld, settings.generateUVs);
            local.triangles += static_cast<u32>((result.indices.size() - before) / 3);
        }
        ++local.cells;
    }
    for (Math::vec3& n : result.normals)
        n = Math::length(n) > 1e-6f ? Math::normalize(n) : Math::vec3(0.0f, 1.0f, 0.0f);
    for (const Math::vec3& p : result.positions) result.bounds.expand(p);
    if (!result.positions.empty()) result.submeshes.push_back({0, static_cast<u32>(result.indices.size()), 0, 0, result.bounds});
    out = std::move(result);
    if (stats) *stats = local;
    return true;
}
} // namespace Radion::Volume
