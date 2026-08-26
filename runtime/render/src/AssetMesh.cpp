#include "PCH.h"

#include <chrono>

#include "AssetManager.h"
#include "FileSystem.h"
#include "MaterialManager.h"
#include "Pixmap.h"
#include "RadionMeshImporter.h"

#include <meshoptimizer/meshoptimizer.h>

namespace Radion
{

namespace
{

// The one gate every mesh - procedural, imported, or handed in by a caller -
// passes through on the way to the GPU. Individual importers validate
// different subsets of this (RadionMeshImporter checks its chunk sizes,
// ObjImporter checks its own indices), but nothing stopped custom/procedural
// data from reaching upload() with an index past the vertex count or a
// submesh range past the index buffer - both read GPU memory that was never
// allocated for this mesh, silently, on whichever driver does not happen to
// bounds-check.
bool validateMeshData(const MeshData& data, std::string& error)
{
    for (usize i = 0; i < data.indices.size(); ++i)
    {
        if (data.indices[i] >= data.positions.size())
        {
            error = "index " + std::to_string(data.indices[i]) + " at position " +
                    std::to_string(i) + " is out of range for " +
                    std::to_string(data.positions.size()) + " vertices";
            return false;
        }
    }

    for (usize i = 0; i < data.submeshes.size(); ++i)
    {
        const SubMesh& submesh = data.submeshes[i];
        // u64, not u32 + u32: a caller building MeshData directly (not
        // through an importer's own count cap) can hand in two values that
        // overflow the u32 sum before it is ever compared to indices.size().
        const u64 end = static_cast<u64>(submesh.indexOffset) + submesh.indexCount;
        if (end > data.indices.size())
        {
            error = "submesh " + std::to_string(i) + " range [" +
                    std::to_string(submesh.indexOffset) + ", " + std::to_string(end) +
                    ") exceeds " + std::to_string(data.indices.size()) + " indices";
            return false;
        }
    }

    for (usize i = 0; i < data.positions.size(); ++i)
    {
        const glm::vec3& p = data.positions[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        {
            error = "non-finite position at vertex " + std::to_string(i);
            return false;
        }
    }

    return true;
}

// Grid resolution for splitSubMeshes(): one axis per extent, scaled so the
// product of the three lands near triangleCount/targetTriangles. An axis
// with near-zero extent (a flat wall, a floor) is forced to a single cell
// instead of feeding a near-zero value into the scale, which would blow the
// other two axes up trying to compensate.
void computeSplitGrid(const AABB& bounds, u32 triangleCount, u32 targetTriangles, int& gx, int& gy,
                      int& gz)
{
    const f32 minExtent = 1e-4f;
    const glm::vec3 extents = bounds.extents();
    const bool flatX = extents.x < minExtent;
    const bool flatY = extents.y < minExtent;
    const bool flatZ = extents.z < minExtent;

    const f32 ex = std::max(extents.x, minExtent);
    const f32 ey = std::max(extents.y, minExtent);
    const f32 ez = std::max(extents.z, minExtent);

    const f32 targetCells = static_cast<f32>(triangleCount) / static_cast<f32>(targetTriangles);
    const f32 scale = std::cbrt(targetCells / (ex * ey * ez));

    gx = flatX ? 1 : std::clamp(static_cast<int>(std::round(ex * scale)), 1, 64);
    gy = flatY ? 1 : std::clamp(static_cast<int>(std::round(ey * scale)), 1, 64);
    gz = flatZ ? 1 : std::clamp(static_cast<int>(std::round(ez * scale)), 1, 64);
}

// Reorders mesh.indices within submesh's own range by a uniform grid over
// each triangle's centroid, and appends one SubMesh per non-empty cell to
// `out`. Leaves both mesh and out untouched and returns false when the
// submesh is already small enough or the grid comes out 1x1x1 - the caller
// keeps the original SubMesh in that case.
bool splitSubMeshGrid(MeshData& mesh, const SubMesh& submesh, u32 targetTriangles,
                      std::vector<SubMesh>& out)
{
    const u32 triangleCount = submesh.indexCount / 3;
    if (triangleCount == 0 || triangleCount <= targetTriangles)
        return false;

    AABB bounds;
    for (u32 i = 0; i < submesh.indexCount; ++i)
    {
        const u32 index = mesh.indices[submesh.indexOffset + i];
        if (index < mesh.positions.size())
            bounds.expand(mesh.positions[index]);
    }
    if (bounds.empty())
        return false;

    int gx = 1, gy = 1, gz = 1;
    computeSplitGrid(bounds, triangleCount, targetTriangles, gx, gy, gz);
    if (gx <= 1 && gy <= 1 && gz <= 1)
        return false;

    const glm::vec3 extents = bounds.extents();
    const f32 invEx = extents.x > 1e-4f ? 1.0f / extents.x : 0.0f;
    const f32 invEy = extents.y > 1e-4f ? 1.0f / extents.y : 0.0f;
    const f32 invEz = extents.z > 1e-4f ? 1.0f / extents.z : 0.0f;

    const u32 cellCount = static_cast<u32>(gx) * static_cast<u32>(gy) * static_cast<u32>(gz);
    const u32 invalidCell = cellCount;

    // Counting sort by cell index: cellOf[t] is each triangle's bucket,
    // offsets[c] becomes bucket c's start once prefix-summed. Two flat
    // arrays and one pass to place every triangle - no per-cell allocation,
    // and iterating cells in ascending index order makes the result
    // deterministic without ever hashing anything.
    std::vector<u32> cellOf(triangleCount, invalidCell);
    std::vector<u32> offsets(cellCount + 1, 0);

    for (u32 t = 0; t < triangleCount; ++t)
    {
        const u32 base = submesh.indexOffset + t * 3;
        const u32 i0 = mesh.indices[base + 0];
        const u32 i1 = mesh.indices[base + 1];
        const u32 i2 = mesh.indices[base + 2];
        if (i0 >= mesh.positions.size() || i1 >= mesh.positions.size() ||
            i2 >= mesh.positions.size())
            continue;

        const glm::vec3 centroid =
            (mesh.positions[i0] + mesh.positions[i1] + mesh.positions[i2]) / 3.0f;

        const int cx =
            gx > 1
                ? std::clamp(static_cast<int>((centroid.x - bounds.min.x) * invEx * gx), 0, gx - 1)
                : 0;
        const int cy =
            gy > 1
                ? std::clamp(static_cast<int>((centroid.y - bounds.min.y) * invEy * gy), 0, gy - 1)
                : 0;
        const int cz =
            gz > 1
                ? std::clamp(static_cast<int>((centroid.z - bounds.min.z) * invEz * gz), 0, gz - 1)
                : 0;

        const u32 cell =
            static_cast<u32>(cx) + static_cast<u32>(cy) * gx + static_cast<u32>(cz) * gx * gy;
        cellOf[t] = cell;
        offsets[cell + 1]++;
    }

    for (u32 c = 0; c < cellCount; ++c)
        offsets[c + 1] += offsets[c];

    const u32 validCount = offsets[cellCount];
    u32 nonEmptyCells = 0;
    for (u32 c = 0; c < cellCount; ++c)
        if (offsets[c + 1] > offsets[c])
            ++nonEmptyCells;

    if (nonEmptyCells <= 1)
        return false;

    std::vector<u32> cursor(offsets.begin(), offsets.begin() + cellCount);
    std::vector<u32> order(validCount);
    for (u32 t = 0; t < triangleCount; ++t)
    {
        if (cellOf[t] == invalidCell)
            continue;
        order[cursor[cellOf[t]]++] = t;
    }

    std::vector<u32> reordered;
    reordered.reserve(validCount * 3);
    for (u32 t : order)
    {
        const u32 base = submesh.indexOffset + t * 3;
        reordered.push_back(mesh.indices[base + 0]);
        reordered.push_back(mesh.indices[base + 1]);
        reordered.push_back(mesh.indices[base + 2]);
    }
    std::copy(reordered.begin(), reordered.end(), mesh.indices.begin() + submesh.indexOffset);

    for (u32 c = 0; c < cellCount; ++c)
    {
        const u32 count = offsets[c + 1] - offsets[c];
        if (count == 0)
            continue;

        SubMesh piece;
        piece.indexOffset = submesh.indexOffset + offsets[c] * 3;
        piece.indexCount = count * 3;
        piece.materialSlot = submesh.materialSlot;
        piece.lightmapPage = submesh.lightmapPage;
        out.push_back(piece);
    }

    return true;
}

// Every primitive below that revolves around Y (sphere, cylinder/cone,
// capsule, torus) shares this as its tangent direction: whatever the radius
// at a given theta, position always carries (cos theta, sin theta) into x/z,
// so d(position)/d(theta) always points this way regardless of which ring or
// how far out it is. Computed inline, per vertex, alongside the normal - no
// separate pass over the mesh once it already exists.
glm::vec4 revolveTangent(f32 theta)
{
    return glm::vec4(-std::sin(theta), 0.0f, std::cos(theta), 1.0f);
}

glm::vec3 faceNormal(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    const glm::vec3 normal = glm::cross(v1 - v0, v2 - v0);
    const float length = glm::length(normal);
    return length > 0.0f ? normal / length : glm::vec3(0.0f);
}

// The interior angle at each of the three vertices, from the law of cosines.
// Weighting by angle stops a corner shared by many small triangles from
// dragging the smooth normal towards them.
glm::vec3 angleWeights(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    const float a = glm::dot(v1 - v2, v1 - v2);
    const float b = glm::dot(v0 - v2, v0 - v2);
    const float c = glm::dot(v0 - v1, v0 - v1);

    const float aSqrt = std::sqrt(a);
    const float bSqrt = std::sqrt(b);
    const float cSqrt = std::sqrt(c);

    if (aSqrt <= 0.0f || bSqrt <= 0.0f || cSqrt <= 0.0f)
        return glm::vec3(1.0f);

    return glm::vec3(std::acos(glm::clamp((b + c - a) / (2.0f * bSqrt * cSqrt), -1.0f, 1.0f)),
                     std::acos(glm::clamp((-b + c + a) / (2.0f * aSqrt * cSqrt), -1.0f, 1.0f)),
                     std::acos(glm::clamp((b - c + a) / (2.0f * bSqrt * aSqrt), -1.0f, 1.0f)));
}

// ------------------------------------------------------------ primitives

// One quad's worth of shared vertices per face, so each face keeps its own
// flat normal and uv island - a cube needs hard edges, not smooth ones.
void appendCubeFace(MeshData& data, const glm::vec3& normal, const glm::vec3& v0,
                    const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& v3)
{
    const u32 base = static_cast<u32>(data.positions.size());
    const glm::vec3 corners[4] = {v0, v1, v2, v3};
    const glm::vec2 uvs[4] = {{0.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 1.0f}};
    // uv[0] -> uv[1] is exactly one step in U with none in V, so that edge
    // already IS the tangent direction - nothing to derive.
    const glm::vec4 tangent(glm::normalize(v1 - v0), 1.0f);
    for (u32 i = 0; i < 4; ++i)
    {
        data.positions.push_back(corners[i]);
        data.normals.push_back(normal);
        data.uvs.push_back(uvs[i]);
        data.tangents.push_back(tangent);
    }
    const u32 quad[6] = {base, base + 1, base + 2, base, base + 2, base + 3};
    data.indices.insert(data.indices.end(), quad, quad + 6);
}

void buildBox(MeshData& data, const glm::vec3& size)
{
    const glm::vec3 h = size * 0.5f;
    appendCubeFace(data, {0, 0, 1}, {-h.x, -h.y, h.z}, {h.x, -h.y, h.z}, {h.x, h.y, h.z},
                   {-h.x, h.y, h.z}); // +Z
    appendCubeFace(data, {0, 0, -1}, {h.x, -h.y, -h.z}, {-h.x, -h.y, -h.z}, {-h.x, h.y, -h.z},
                   {h.x, h.y, -h.z}); // -Z
    appendCubeFace(data, {1, 0, 0}, {h.x, -h.y, h.z}, {h.x, -h.y, -h.z}, {h.x, h.y, -h.z},
                   {h.x, h.y, h.z}); // +X
    appendCubeFace(data, {-1, 0, 0}, {-h.x, -h.y, -h.z}, {-h.x, -h.y, h.z}, {-h.x, h.y, h.z},
                   {-h.x, h.y, -h.z}); // -X
    appendCubeFace(data, {0, 1, 0}, {-h.x, h.y, h.z}, {h.x, h.y, h.z}, {h.x, h.y, -h.z},
                   {-h.x, h.y, -h.z}); // +Y
    appendCubeFace(data, {0, -1, 0}, {-h.x, -h.y, -h.z}, {h.x, -h.y, -h.z}, {h.x, -h.y, h.z},
                   {-h.x, -h.y, h.z}); // -Y
}

void buildPlane(MeshData& data, f32 width, f32 depth, u32 segX, u32 segZ, f32 uvTiles)
{
    segX = segX < 1 ? 1 : segX;
    segZ = segZ < 1 ? 1 : segZ;
    const u32 nx = segX + 1, nz = segZ + 1;
    const f32 x0 = -width * 0.5f, z0 = -depth * 0.5f;
    const f32 dx = width / static_cast<f32>(segX), dz = depth / static_cast<f32>(segZ);

    data.positions.resize(static_cast<usize>(nx) * nz);
    data.normals.resize(data.positions.size());
    data.uvs.resize(data.positions.size());
    // Flat and axis-aligned: U always runs along world +X, so the tangent is
    // the same constant for every vertex - no per-vertex derivative needed.
    data.tangents.assign(data.positions.size(), glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
    for (u32 j = 0; j < nz; ++j)
    {
        for (u32 i = 0; i < nx; ++i)
        {
            const usize v = static_cast<usize>(j) * nx + i;
            data.positions[v] =
                glm::vec3(x0 + static_cast<f32>(i) * dx, 0.0f, z0 + static_cast<f32>(j) * dz);
            data.normals[v] = glm::vec3(0.0f, 1.0f, 0.0f);
            data.uvs[v] =
                glm::vec2(static_cast<f32>(i) / segX, static_cast<f32>(j) / segZ) * uvTiles;
        }
    }
    for (u32 j = 0; j < segZ; ++j)
    {
        for (u32 i = 0; i < segX; ++i)
        {
            const u32 a = j * nx + i, b = a + 1;
            const u32 c = a + nx, d = c + 1;
            const u32 quad[6] = {a, c, b, b, c, d};
            data.indices.insert(data.indices.end(), quad, quad + 6);
        }
    }
}

void buildSphere(MeshData& data, f32 radius, u32 rings, u32 slices)
{
    const f32 pi = glm::pi<f32>();
    data.positions.reserve(static_cast<usize>(rings + 1) * (slices + 1));
    for (u32 r = 0; r <= rings; ++r)
    {
        const f32 v = static_cast<f32>(r) / rings;
        const f32 phi = v * pi; // 0 = top pole
        for (u32 s = 0; s <= slices; ++s)
        {
            const f32 u = static_cast<f32>(s) / slices;
            const f32 theta = u * 2.0f * pi;
            const glm::vec3 n(std::sin(phi) * std::cos(theta), std::cos(phi),
                              std::sin(phi) * std::sin(theta));
            data.positions.push_back(n * radius);
            data.normals.push_back(n);
            data.uvs.push_back(glm::vec2(u, v));
            data.tangents.push_back(revolveTangent(theta));
        }
    }
    for (u32 r = 0; r < rings; ++r)
    {
        for (u32 s = 0; s < slices; ++s)
        {
            const u32 a = r * (slices + 1) + s, b = a + slices + 1;
            const u32 quad[6] = {a, a + 1, b, a + 1, b + 1, b};
            data.indices.insert(data.indices.end(), quad, quad + 6);
        }
    }
}

// Shared side+caps builder: topScale=1 cylinder, 0 cone (apex ring).
void buildTube(MeshData& data, f32 radius, f32 height, u32 slices, f32 topScale)
{
    const f32 pi = glm::pi<f32>();
    // Side normals lean outward for cones: slope = radius shrink over height.
    const f32 slope = (radius - radius * topScale) / height;
    for (u32 cap = 0; cap <= 1; ++cap) // 0 = bottom ring, 1 = top ring
    {
        const f32 y = cap ? height : 0.0f;
        const f32 r = cap ? radius * topScale : radius;
        for (u32 s = 0; s <= slices; ++s)
        {
            const f32 u = static_cast<f32>(s) / slices;
            const f32 theta = u * 2.0f * pi;
            const f32 cx = std::cos(theta), cz = std::sin(theta);
            const glm::vec3 n = glm::normalize(glm::vec3(cx, slope, cz));
            data.positions.push_back(glm::vec3(cx * r, y, cz * r));
            data.normals.push_back(n);
            data.uvs.push_back(glm::vec2(u, static_cast<f32>(cap)));
            data.tangents.push_back(revolveTangent(theta));
        }
    }
    for (u32 s = 0; s < slices; ++s)
    {
        const u32 a = s, b = s + slices + 1;
        const u32 quad[6] = {a, b, a + 1, a + 1, b, b + 1};
        data.indices.insert(data.indices.end(), quad, quad + 6);
    }

    // Caps: centre fans (bottom always; top only when it has area).
    for (u32 cap = 0; cap <= 1; ++cap)
    {
        const f32 r = cap ? radius * topScale : radius;
        if (r < 1e-5f)
            continue;
        const f32 y = cap ? height : 0.0f;
        const f32 ny = cap ? 1.0f : -1.0f;
        const u32 center = static_cast<u32>(data.positions.size());
        data.positions.push_back(glm::vec3(0.0f, y, 0.0f));
        data.normals.push_back(glm::vec3(0.0f, ny, 0.0f));
        data.uvs.push_back(glm::vec2(0.5f, 0.5f));
        data.tangents.push_back(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
        for (u32 s = 0; s <= slices; ++s)
        {
            const f32 theta = static_cast<f32>(s) / slices * 2.0f * pi;
            data.positions.push_back(glm::vec3(std::cos(theta) * r, y, std::sin(theta) * r));
            data.normals.push_back(glm::vec3(0.0f, ny, 0.0f));
            data.uvs.push_back(
                glm::vec2(std::cos(theta) * 0.5f + 0.5f, std::sin(theta) * 0.5f + 0.5f));
            data.tangents.push_back(revolveTangent(theta));
        }
        for (u32 s = 0; s < slices; ++s)
        {
            const u32 a = center + 1 + s, b = a + 1;
            if (cap)
            {
                data.indices.push_back(center);
                data.indices.push_back(b);
                data.indices.push_back(a);
            }
            else
            {
                data.indices.push_back(center);
                data.indices.push_back(a);
                data.indices.push_back(b);
            }
        }
    }
}

// Bottom hemisphere (pole..equator) + an explicit second equator ring at
// y=radius+height (the straight cylindrical body between the two) + top
// hemisphere (equator..pole). uv.v runs 0 (bottom pole) to 1 (top pole).
void buildCapsule(MeshData& data, f32 radius, f32 height, u32 rings, u32 slices)
{
    const f32 pi = glm::pi<f32>();
    const u32 totalRings = rings * 2 + 2; // + the explicit top-equator row
    const f32 vScale = 1.0f / static_cast<f32>(totalRings - 1);

    u32 row = 0;
    auto addRing = [&](f32 y, f32 ringRadius, f32 ny, f32 nRingScale)
    {
        const f32 v = static_cast<f32>(row) * vScale;
        for (u32 s = 0; s <= slices; ++s)
        {
            const f32 u = static_cast<f32>(s) / slices;
            const f32 theta = u * 2.0f * pi;
            const f32 cx = std::cos(theta), cz = std::sin(theta);
            const glm::vec3 n = glm::normalize(glm::vec3(cx * nRingScale, ny, cz * nRingScale));
            data.positions.push_back(glm::vec3(cx * ringRadius, y, cz * ringRadius));
            data.normals.push_back(n);
            data.uvs.push_back(glm::vec2(u, v));
            data.tangents.push_back(revolveTangent(theta));
        }
        ++row;
    };

    // Bottom hemisphere: pole (phi=pi, y=0) down to the equator (phi=pi/2).
    // Its centre sits at y=radius, so the pole lands exactly on y=0.
    for (u32 r = 0; r <= rings; ++r)
    {
        const f32 phi = pi * (1.0f - 0.5f * static_cast<f32>(r) / rings);
        addRing(radius + radius * std::cos(phi), radius * std::sin(phi), std::cos(phi),
                std::sin(phi));
    }
    // Straight body: the same equator, lifted to y=radius+height.
    addRing(radius + height, radius, 0.0f, 1.0f);
    // Top hemisphere: equator (phi=pi/2) up to the pole (phi=0).
    for (u32 r = 1; r <= rings; ++r)
    {
        const f32 phi = pi * 0.5f * (1.0f - static_cast<f32>(r) / rings);
        addRing(radius + height + radius * std::cos(phi), radius * std::sin(phi), std::cos(phi),
                std::sin(phi));
    }

    for (u32 r = 0; r < totalRings - 1; ++r)
    {
        for (u32 s = 0; s < slices; ++s)
        {
            const u32 a = r * (slices + 1) + s, b = a + slices + 1;
            const u32 quad[6] = {a, a + 1, b, a + 1, b + 1, b};
            data.indices.insert(data.indices.end(), quad, quad + 6);
        }
    }
}

// Ring in the XZ plane, hole along Y: major angle theta sweeps the big circle
// around Y, minor angle phi sweeps the tube cross-section around the tangent.
void buildTorus(MeshData& data, f32 majorRadius, f32 minorRadius, u32 majorSegments,
                u32 minorSegments)
{
    const f32 pi = glm::pi<f32>();
    data.positions.reserve(static_cast<usize>(majorSegments + 1) * (minorSegments + 1));
    for (u32 i = 0; i <= majorSegments; ++i)
    {
        const f32 u = static_cast<f32>(i) / majorSegments;
        const f32 theta = u * 2.0f * pi;
        const f32 rx = std::cos(theta), rz = std::sin(theta);
        const glm::vec3 center(rx * majorRadius, 0.0f, rz * majorRadius);
        for (u32 j = 0; j <= minorSegments; ++j)
        {
            const f32 v = static_cast<f32>(j) / minorSegments;
            const f32 phi = v * 2.0f * pi;
            const glm::vec3 n(rx * std::cos(phi), std::sin(phi), rz * std::cos(phi));
            data.positions.push_back(center + n * minorRadius);
            data.normals.push_back(n);
            data.uvs.push_back(glm::vec2(u, v));
            data.tangents.push_back(revolveTangent(theta));
        }
    }
    for (u32 i = 0; i < majorSegments; ++i)
    {
        for (u32 j = 0; j < minorSegments; ++j)
        {
            const u32 a = i * (minorSegments + 1) + j, b = a + minorSegments + 1;
            const u32 quad[6] = {a, b, a + 1, a + 1, b, b + 1};
            data.indices.insert(data.indices.end(), quad, quad + 6);
        }
    }
}

// Height at a point of the plane, read out of the heightmap. The image is
// stretched across the plane's extent and sampled bilinearly, so the image's
// resolution and the mesh's segment count stay independent; grayscale comes
// off the red channel as 0..1, scaled by `scale`. Outside the plane - the
// neighbour taps the normals need at the border - clamps to the edge pixel
// rather than wrapping, so the rim does not fold over.
f32 sampleHeightmap(const Pixmap& image, f32 u, f32 v, f32 scale)
{
    const f32 px = glm::clamp(u, 0.0f, 1.0f) * static_cast<f32>(image.width - 1);
    const f32 pz = glm::clamp(v, 0.0f, 1.0f) * static_cast<f32>(image.height - 1);
    const int x0 = static_cast<int>(px), z0 = static_cast<int>(pz);
    const int x1 = x0 + 1 < image.width ? x0 + 1 : x0;
    const int z1 = z0 + 1 < image.height ? z0 + 1 : z0;
    const f32 fx = px - static_cast<f32>(x0), fz = pz - static_cast<f32>(z0);

    const u8* pixels = image.pixels;
    const int stride = image.components;
    const f32 h00 = static_cast<f32>(pixels[(z0 * image.width + x0) * stride]);
    const f32 h10 = static_cast<f32>(pixels[(z0 * image.width + x1) * stride]);
    const f32 h01 = static_cast<f32>(pixels[(z1 * image.width + x0) * stride]);
    const f32 h11 = static_cast<f32>(pixels[(z1 * image.width + x1) * stride]);
    return glm::mix(glm::mix(h00, h10, fx), glm::mix(h01, h11, fx), fz) / 255.0f * scale;
}

void buildHillsPlane(MeshData& data, f32 width, f32 depth, u32 segX, u32 segZ,
                     const Pixmap& heightmap, f32 heightScale, f32 uvTiles)
{
    segX = segX < 1 ? 1 : segX;
    segZ = segZ < 1 ? 1 : segZ;
    const u32 nx = segX + 1, nz = segZ + 1;
    const f32 x0 = -width * 0.5f, z0 = -depth * 0.5f;
    const f32 dx = width / static_cast<f32>(segX), dz = depth / static_cast<f32>(segZ);

    data.positions.resize(static_cast<usize>(nx) * nz);
    data.normals.resize(data.positions.size());
    data.uvs.resize(data.positions.size());
    data.tangents.resize(data.positions.size());
    for (u32 j = 0; j < nz; ++j)
    {
        for (u32 i = 0; i < nx; ++i)
        {
            const usize v = static_cast<usize>(j) * nx + i;
            const f32 wx = x0 + static_cast<f32>(i) * dx, wz = z0 + static_cast<f32>(j) * dz;
            data.positions[v] = glm::vec3(
                wx, sampleHeightmap(heightmap, (wx - x0) / width, (wz - z0) / depth, heightScale),
                wz);
            data.uvs[v] =
                glm::vec2(static_cast<f32>(i) / segX, static_cast<f32>(j) / segZ) * uvTiles;
        }
    }
    // Central-difference normals - cheap here since segX/segZ is small
    // (decorative ground, not open-world terrain). The same hl/hr sample
    // that tilts the normal along X is also the slope U (world +X) climbs
    // at, so the tangent falls out of the same two heightmap taps.
    for (u32 j = 0; j < nz; ++j)
    {
        for (u32 i = 0; i < nx; ++i)
        {
            const usize v = static_cast<usize>(j) * nx + i;
            const glm::vec3& p = data.positions[v];
            const f32 hl = sampleHeightmap(heightmap, (p.x - dx - x0) / width, (p.z - z0) / depth,
                                           heightScale);
            const f32 hr = sampleHeightmap(heightmap, (p.x + dx - x0) / width, (p.z - z0) / depth,
                                           heightScale);
            const f32 hd = sampleHeightmap(heightmap, (p.x - x0) / width, (p.z - dz - z0) / depth,
                                           heightScale);
            const f32 hu = sampleHeightmap(heightmap, (p.x - x0) / width, (p.z + dz - z0) / depth,
                                           heightScale);
            data.normals[v] =
                glm::normalize(glm::vec3(hl - hr, 2.0f * ((dx + dz) * 0.5f), hd - hu));
            data.tangents[v] = glm::vec4(glm::normalize(glm::vec3(2.0f * dx, hr - hl, 0.0f)), 1.0f);
        }
    }
    for (u32 j = 0; j < segZ; ++j)
    {
        for (u32 i = 0; i < segX; ++i)
        {
            const u32 a = j * nx + i, b = a + 1;
            const u32 c = a + nx, d = c + 1;
            const u32 quad[6] = {a, c, b, b, c, d};
            data.indices.insert(data.indices.end(), quad, quad + 6);
        }
    }
}

void buildHeightfield(MeshData& data, const f32* heights, u32 w, u32 h, f32 cellSize, f32 uvTiles)
{
    data.positions.resize(static_cast<usize>(w) * h);
    data.normals.resize(data.positions.size());
    data.uvs.resize(data.positions.size());
    data.tangents.resize(data.positions.size());
    for (u32 j = 0; j < h; ++j)
    {
        for (u32 i = 0; i < w; ++i)
        {
            const usize v = static_cast<usize>(j) * w + i;
            data.positions[v] = glm::vec3(static_cast<f32>(i) * cellSize, heights[v],
                                          static_cast<f32>(j) * cellSize);
            data.uvs[v] =
                glm::vec2(static_cast<f32>(i) / (w - 1), static_cast<f32>(j) / (h - 1)) * uvTiles;

            const f32 hl = heights[j * w + (i > 0 ? i - 1 : i)];
            const f32 hr = heights[j * w + (i < w - 1 ? i + 1 : i)];
            const f32 hd = heights[(j > 0 ? j - 1 : j) * w + i];
            const f32 hu = heights[(j < h - 1 ? j + 1 : j) * w + i];
            data.normals[v] = glm::normalize(glm::vec3(hl - hr, 2.0f * cellSize, hd - hu));
            data.tangents[v] =
                glm::vec4(glm::normalize(glm::vec3(2.0f * cellSize, hr - hl, 0.0f)), 1.0f);
        }
    }
    for (u32 j = 0; j < h - 1; ++j)
    {
        for (u32 i = 0; i < w - 1; ++i)
        {
            const u32 a = j * w + i, b = a + 1;
            const u32 c = a + w, d = c + 1;
            const u32 quad[6] = {a, c, b, b, c, d};
            data.indices.insert(data.indices.end(), quad, quad + 6);
        }
    }
}

// One vertex per pixel: here the image's resolution is the mesh's, unlike
// buildHillsPlane where the image is stretched over a plane that keeps its
// own segment count.
void buildHeightfieldFromPixmap(MeshData& data, const Pixmap& heightmap, f32 cellSize,
                                f32 heightScale, f32 uvTiles)
{
    const u32 w = static_cast<u32>(heightmap.width), h = static_cast<u32>(heightmap.height);
    const int stride = heightmap.components;
    std::vector<f32> world(static_cast<usize>(w) * h);
    for (usize i = 0; i < world.size(); ++i)
        world[i] = static_cast<f32>(heightmap.pixels[i * stride]) / 255.0f * heightScale;
    buildHeightfield(data, world.data(), w, h, cellSize, uvTiles);
}

} // namespace

// --------------------------------------------------------------- mesh data

usize MeshData::vertexCount() const
{
    return positions.size();
}

usize MeshData::triangleCount() const
{
    return indices.size() / 3;
}

usize MeshData::memoryBytes() const
{
    return positions.size() * sizeof(glm::vec3) + normals.size() * sizeof(glm::vec3) +
           tangents.size() * sizeof(glm::vec4) + uvs.size() * sizeof(glm::vec2) +
           uvs2.size() * sizeof(glm::vec2) + colors.size() * sizeof(u32) +
           skin.size() * sizeof(MeshSkinVertex) + indices.size() * sizeof(u32) +
           submeshes.size() * sizeof(SubMesh);
}

void MeshData::clear()
{
    positions.clear();
    normals.clear();
    tangents.clear();
    uvs.clear();
    uvs2.clear();
    colors.clear();
    skin.clear();
    indices.clear();
    submeshes.clear();
    materials.clear();
    materialTextureFiles.clear();
    materialNormalFiles.clear();
    bounds = AABB();
}

void MeshData::resizeVertices(usize count)
{
    positions.resize(count);
    if (!normals.empty())
        normals.resize(count);
    if (!tangents.empty())
        tangents.resize(count);
    if (!uvs.empty())
        uvs.resize(count);
    if (!uvs2.empty())
        uvs2.resize(count);
    if (!colors.empty())
        colors.resize(count);
    if (!skin.empty())
        skin.resize(count);
}

// -------------------------------------------------------- mesh descriptions

namespace
{
// Indexed by MeshSource. These names reach a saved scene through
// MeshDesc::key(), so renaming one - or reordering a recipe's parameters in
// the factories below - is a format change, not a refactor.
const char* const kMeshSourceNames[] = {"None",   "File",       "Box",        "Plane",
                                        "Sphere", "Cylinder",   "Cone",       "Capsule",
                                        "Torus",  "HillsPlane", "Heightfield"};

MeshDesc makeDesc(MeshSource source, std::initializer_list<f32> params)
{
    MeshDesc desc;
    desc.source = source;
    usize i = 0;
    for (f32 value : params)
        desc.params[i++] = value;
    return desc;
}
} // namespace

const char* meshSourceName(MeshSource source)
{
    const u8 index = static_cast<u8>(source);
    return index < (sizeof(kMeshSourceNames) / sizeof(kMeshSourceNames[0]))
               ? kMeshSourceNames[index]
               : "None";
}

bool meshSourceFromName(const std::string& name, MeshSource& out)
{
    for (usize i = 0; i < sizeof(kMeshSourceNames) / sizeof(kMeshSourceNames[0]); ++i)
    {
        if (name == kMeshSourceNames[i])
        {
            out = static_cast<MeshSource>(i);
            return true;
        }
    }
    return false;
}

MeshDesc MeshDesc::fromFile(const std::string& file)
{
    MeshDesc desc;
    desc.source = MeshSource::File;
    desc.file = file;
    return desc;
}

MeshDesc MeshDesc::box(const glm::vec3& size)
{
    return makeDesc(MeshSource::Box, {size.x, size.y, size.z});
}

MeshDesc MeshDesc::plane(f32 width, f32 depth, u32 segX, u32 segZ, f32 uvTiles)
{
    return makeDesc(MeshSource::Plane,
                    {width, depth, static_cast<f32>(segX), static_cast<f32>(segZ), uvTiles});
}

MeshDesc MeshDesc::sphere(f32 radius, u32 rings, u32 slices)
{
    return makeDesc(MeshSource::Sphere,
                    {radius, static_cast<f32>(rings), static_cast<f32>(slices)});
}

MeshDesc MeshDesc::cylinder(f32 radius, f32 height, u32 slices)
{
    return makeDesc(MeshSource::Cylinder, {radius, height, static_cast<f32>(slices)});
}

MeshDesc MeshDesc::cone(f32 radius, f32 height, u32 slices)
{
    return makeDesc(MeshSource::Cone, {radius, height, static_cast<f32>(slices)});
}

MeshDesc MeshDesc::capsule(f32 radius, f32 height, u32 rings, u32 slices)
{
    return makeDesc(MeshSource::Capsule,
                    {radius, height, static_cast<f32>(rings), static_cast<f32>(slices)});
}

MeshDesc MeshDesc::torus(f32 majorRadius, f32 minorRadius, u32 majorSegments, u32 minorSegments)
{
    return makeDesc(MeshSource::Torus, {majorRadius, minorRadius, static_cast<f32>(majorSegments),
                                        static_cast<f32>(minorSegments)});
}

MeshDesc MeshDesc::hillsPlane(f32 width, f32 depth, u32 segX, u32 segZ,
                              const std::string& heightmapFile, f32 heightScale, f32 uvTiles)
{
    MeshDesc desc =
        makeDesc(MeshSource::HillsPlane, {width, depth, static_cast<f32>(segX),
                                          static_cast<f32>(segZ), heightScale, uvTiles});
    desc.file = heightmapFile;
    return desc;
}

MeshDesc MeshDesc::heightfield(const std::string& heightmapFile, f32 cellSize, f32 heightScale,
                               f32 uvTiles)
{
    MeshDesc desc = makeDesc(MeshSource::Heightfield, {cellSize, heightScale, uvTiles});
    desc.file = heightmapFile;
    return desc;
}

bool MeshDesc::operator==(const MeshDesc& other) const
{
    if (source != other.source || file != other.file)
        return false;
    for (usize i = 0; i < 8; ++i)
        if (params[i] != other.params[i])
            return false;
    return true;
}

std::string MeshDesc::key() const
{
    std::string out = kMeshSourceNames[static_cast<u8>(source)];
    out += '|';
    out += file;
    // A File mesh is identified by its path alone; its parameter slots are
    // unused and would only put noise in the key.
    if (source == MeshSource::File)
        return out;
    char number[32];
    for (usize i = 0; i < 8; ++i)
    {
        std::snprintf(number, sizeof(number), "|%g", params[i]);
        out += number;
    }
    return out;
}

MeshHandle AssetManager::createMesh(const MeshDesc& desc)
{
    if (desc.source == MeshSource::None)
    {
        Log::error("AssetManager: createMesh() got a description with no source");
        return MeshHandle();
    }

    const std::string key = desc.key();
    const auto cached = mMeshByKey.find(key);
    if (cached != mMeshByKey.end() && mMeshes.get(cached->second))
        return cached->second;

    const MeshHandle handle = buildFromDesc(desc);
    if (!handle.valid())
        return handle;

    mMeshDescs[packHandle(handle)] = desc;
    mMeshByKey[key] = handle;
    return handle;
}

const MeshDesc& AssetManager::meshDesc(MeshHandle handle) const
{
    static const MeshDesc none;
    const auto entry = mMeshDescs.find(packHandle(handle));
    return entry != mMeshDescs.end() ? entry->second : none;
}

void AssetManager::registerMeshDesc(MeshHandle handle, const MeshDesc& desc)
{
    if (!handle.valid() || desc.source == MeshSource::None)
        return;
    mMeshDescs[packHandle(handle)] = desc;
    mMeshByKey[desc.key()] = handle;
}

namespace
{
// A texture reference an importer already joined against the mesh's own
// directory (an OBJ's map_Kd, e.g.) but that still does not resolve gets one
// more try with a textures/ subfolder inserted before the filename -
// Sponza's own layout (map_Kd lines carry only the bare filename, the actual
// files sit one level down in textures/). FileSystem's search-path list
// cannot express this on its own: resolveOnDisk() only ever prepends a whole
// extra root to the name as given, it never inserts a subfolder into a name
// that already carries a directory of its own (see
// AssetManager::importMeshFileData()'s own search-path registration right
// above this, which is why that alone was not enough for a name shaped like
// this).
void retryUnderTexturesSubfolder(FileSystem& files, std::vector<std::string>& paths)
{
    for (std::string& path : paths)
    {
        if (path.empty() || files.exists(path))
            continue;
        const usize slash = path.find_last_of('/');
        const std::string candidate = slash == std::string::npos
                                          ? "textures/" + path
                                          : path.substr(0, slash) + "/textures" + path.substr(slash);
        if (files.exists(candidate))
            path = candidate;
    }
}
} // namespace

bool AssetManager::importMeshFileData(const std::string& file, MeshData& data)
{
    registerMeshSearchPaths(file);
    if (!importMeshGeometry(file, data))
        return false;
    applyMeshFileMaterials(file, data);
    return true;
}

void AssetManager::registerMeshSearchPaths(const std::string& file)
{
    // A foreign export commonly ships as "modelname/modelname.obj" next to
    // "modelname/textures/" (Sponza is the standard example), and its own
    // texture references are relative to the mesh's own directory, not that
    // textures/ subfolder - only findable once that subfolder is itself a
    // search path. This used to only happen inside AssetsPanel's Import
    // button (see its own comment); anything else that loads the same mesh
    // file later - reopening a saved scene, Mesh Tools' reimport - went
    // through importMesh() directly and never got it, so every texture
    // lookup failed silently (checker fallback) outside that one popup.
    // Registered here instead, the one choke point every mesh load already
    // goes through, so it happens exactly once regardless of which caller
    // asked.
    {
        // Resolved to a real disk path first - `file` itself is routinely
        // just logical (relative to whichever search path already finds it),
        // and addSearchPath()/isDirectory() both need a real one the same
        // way every write path already established elsewhere this file
        // needed FileSystem::resolve() first (see the .rskel/.ranim caching
        // in InspectorPanel's Add Animator flow).
        FileSystem& files = FileSystem::getSingleton();
        const std::string resolved = files.resolve(file);
        const std::string base = resolved.empty() ? file : resolved;
        const usize slash = base.find_last_of("/\\");
        const std::string meshDir = slash == std::string::npos ? std::string() : base.substr(0, slash);
        if (!meshDir.empty())
        {
            files.addSearchPath(meshDir);
            const std::string texturesDir = meshDir + "/textures";
            if (files.isDirectory(texturesDir))
                files.addSearchPath(texturesDir);
        }
    }
}

bool AssetManager::importMeshGeometry(const std::string& file, MeshData& data)
{
    // Logged before the read, not after: on a large mesh this call is where
    // the editor sits unresponsive for seconds, and a line that only appears
    // once it finishes cannot say what it was waiting on.
    Log::info("AssetManager: loading mesh '%s'...", file.c_str());
    const auto meshLoadStarted = std::chrono::steady_clock::now();

    if (!importMesh(file, data))
    {
        Log::error("AssetManager: could not import mesh '%s'", file.c_str());
        return false;
    }

    // Done here rather than by each caller, so the same file always yields
    // the same mesh however it was asked for.
    recalculateTangents(data);
    computeBounds(data);
    computeSubMeshBounds(data);
    Log::info("AssetManager: loaded mesh '%s' in %.0f ms (%zu verts, %zu tris, %zu submeshes, "
              "%zu materials)",
              file.c_str(),
              std::chrono::duration<f64, std::milli>(std::chrono::steady_clock::now() -
                                                    meshLoadStarted)
                  .count(),
              data.positions.size(), data.indices.size() / 3, data.submeshes.size(),
              data.materials.size());
    return true;
}

void AssetManager::applyMeshFileMaterials(const std::string& file, MeshData& data)
{
    {
        FileSystem& files = FileSystem::getSingleton();
        retryUnderTexturesSubfolder(files, data.materialTextureFiles);
        retryUnderTexturesSubfolder(files, data.materialNormalFiles);
        retryUnderTexturesSubfolder(files, data.materialSurfaceFiles);
        retryUnderTexturesSubfolder(files, data.materialEmissiveFiles);
    }
    // A mesh file's own materials are commonly a placeholder (.rmesh
    // never embeds real ones; OBJ's are stand-ins) - the actual look
    // lives in a same-named .material file, the convention every demo
    // hand-rolls today. Applied here so it happens once, by file, and
    // MeshRenderer's own overrides (Save/Load already covers those)
    // still win by replacing whatever this loads.
    {
        const usize dot = file.find_last_of('.');
        const std::string stem = dot == std::string::npos ? file : file.substr(0, dot);
        // Two sidecar spellings are both in real use across the asset tree
        // today (.material for Sinbad/DamagedHelmet/the ninja, .mat for
        // Sponza/city/flocking's fish) - .material tried first since it is
        // the newer/more common one, .mat as a fallback rather than a
        // second silent-flat-fallback for anything that only has that one.
        std::string materialFile = stem + ".material";
        if (!FileSystem::getSingleton().exists(materialFile))
            materialFile = stem + ".mat";
        if (FileSystem::getSingleton().exists(materialFile))
            MaterialManager::getSingleton().load(materialFile, data.materials);
        else
            // No sidecar to say otherwise: every mesh importer (Fbx/Gltf/Ogre/
            // MS3D/B3D) builds its Material with flags defaulted to just cast+
            // receive shadow, never MaterialLit - imported straight in, the
            // mesh would render through the unlit path despite carrying an
            // albedo texture and looking like it should be shaded. Primitives
            // get this same flag from defaultPrimitiveMaterial() (HierarchyPanel)
            // for the same reason.
            for (Material& material : data.materials)
                material.flags |= MaterialLit;
    }
}

MeshHandle AssetManager::buildFromDesc(const MeshDesc& desc)
{
    MeshData data;
    if (!buildMeshData(desc, data))
        return MeshHandle();
    const MeshHandle handle = createMesh(data);
    if (handle.valid() && mRetainFileMeshData && desc.source == MeshSource::File)
        mRetainedMeshData[packHandle(handle)] = std::move(data);
    return handle;
}

bool AssetManager::buildMeshData(const MeshDesc& desc, MeshData& data)
{
    const f32* p = desc.params;
    switch (desc.source)
    {
    case MeshSource::File:
        return importMeshFileData(desc.file, data);
    case MeshSource::Box:
        buildBox(data, glm::vec3(p[0], p[1], p[2]));
        break;
    case MeshSource::Plane:
        buildPlane(data, p[0], p[1], static_cast<u32>(p[2]), static_cast<u32>(p[3]), p[4]);
        break;
    case MeshSource::Sphere:
        buildSphere(data, p[0], static_cast<u32>(p[1]), static_cast<u32>(p[2]));
        break;
    case MeshSource::Cylinder:
        buildTube(data, p[0], p[1], static_cast<u32>(p[2]), 1.0f);
        break;
    case MeshSource::Cone:
        buildTube(data, p[0], p[1], static_cast<u32>(p[2]), 0.0f);
        break;
    case MeshSource::Capsule:
        buildCapsule(data, p[0], p[1], static_cast<u32>(p[2]), static_cast<u32>(p[3]));
        break;
    case MeshSource::Torus:
        buildTorus(data, p[0], p[1], static_cast<u32>(p[2]), static_cast<u32>(p[3]));
        break;
    case MeshSource::HillsPlane:
    {
        Pixmap heightmap;
        if (!heightmap.load(desc.file.c_str()))
        {
            Log::error("AssetManager: could not load heightmap '%s'", desc.file.c_str());
            return false;
        }
        buildHillsPlane(data, p[0], p[1], static_cast<u32>(p[2]), static_cast<u32>(p[3]), heightmap,
                        p[4], p[5]);
        break;
    }
    case MeshSource::Heightfield:
    {
        Pixmap heightmap;
        if (!heightmap.load(desc.file.c_str()))
        {
            Log::error("AssetManager: could not load heightmap '%s'", desc.file.c_str());
            return false;
        }
        if (heightmap.width < 2 || heightmap.height < 2)
        {
            Log::error("AssetManager: heightmap '%s' is smaller than 2x2", desc.file.c_str());
            return false;
        }
        buildHeightfieldFromPixmap(data, heightmap, p[0], p[1], p[2]);
        break;
    }
    case MeshSource::None:
        return false;
    }
    computeBounds(data);
    return true;
}

// ------------------------------------------------------------------- meshes

std::vector<Material> AssetManager::materialsForSidecar(const MeshData& data) const
{
    std::vector<Material> materials = data.materials;
    const struct
    {
        const std::vector<std::string>* files;
        MaterialSlot slot;
    } sources[] = {{&data.materialTextureFiles, SlotAlbedo},
                   {&data.materialNormalFiles, SlotNormal},
                   {&data.materialSurfaceFiles, SlotSurface},
                   {&data.materialEmissiveFiles, SlotEmissive},
                   {&data.materialHeightFiles, SlotHeight}};
    for (const auto& source : sources)
        for (usize i = 0; i < materials.size() && i < source.files->size(); ++i)
        {
            MaterialTexture& slot = materials[i].textures[source.slot];
            if (!(*source.files)[i].empty() && slot.file.empty())
                slot.file = (*source.files)[i];
        }
    return materials;
}

void AssetManager::loadMeshMaterialTextures(Mesh& mesh, const MeshData& data)
{
    for (usize i = 0; i < mesh.materials.size() && i < data.materialTextureFiles.size(); ++i)
    {
        MaterialTexture& slot = mesh.materials[i].textures[SlotAlbedo];
        // A same-named .material/.mat sidecar (importMeshFileData(), run
        // before this) already replaced data.materials[i] wholesale,
        // including this slot's own .file, when it has one for this
        // material - the importer's own reference (an OBJ's map_Kd, joined
        // against the mesh's directory, which is routinely wrong for an
        // asset like Sponza that keeps its textures one level down in
        // textures/) must never override a sidecar that got it right.
        if (!data.materialTextureFiles[i].empty() && slot.file.empty())
        {
            // Async, the same choice SceneSerializer already made for the
            // textures a saved scene names: a mesh the size of the Bistro
            // brings hundreds of these, and decoding them one after another
            // on this thread is what froze the window for the whole load.
            // The handle comes back immediately as a grey placeholder and
            // AsyncTextureLoader rebuilds it in place a few frames later.
            slot.texture = loadTextureAsync(data.materialTextureFiles[i],
                                            Material::colorSpaceFor(SlotAlbedo), true);
            // The GPU handle alone renders fine, but MaterialTexture::file is
            // what InspectorPanel's texture slot UI actually reads to show a
            // name (drawMaterialFields()'s buttonLabel) - without it every
            // slot on an importer-built material (no .material sidecar, so
            // nothing else ever sets this) reads as empty even though the
            // texture is genuinely bound and on screen.
            slot.file = data.materialTextureFiles[i];
        }
    }

    // Normal maps are linear data that only looks like colour - decoding them
    // as sRGB would bend every normal. colorSpaceFor() is the one place that
    // rule lives, so it decides here too rather than this call site guessing.
    for (usize i = 0; i < mesh.materials.size() && i < data.materialNormalFiles.size(); ++i)
    {
        MaterialTexture& slot = mesh.materials[i].textures[SlotNormal];
        if (!data.materialNormalFiles[i].empty() && slot.file.empty())
        {
            slot.texture = loadTextureAsync(data.materialNormalFiles[i],
                                            Material::colorSpaceFor(SlotNormal), true);
            slot.file = data.materialNormalFiles[i];
        }
    }

    // Surface (roughness/metalness, linear data like the normal map above)
    // and Emissive (colour, sRGB like albedo) - a glTF's own
    // metallicRoughnessTexture/emissiveTexture, only GltfImporter fills
    // these two source arrays today.
    for (usize i = 0; i < mesh.materials.size() && i < data.materialSurfaceFiles.size(); ++i)
    {
        MaterialTexture& slot = mesh.materials[i].textures[SlotSurface];
        if (!data.materialSurfaceFiles[i].empty() && slot.file.empty())
        {
            slot.texture = loadTextureAsync(data.materialSurfaceFiles[i],
                                            Material::colorSpaceFor(SlotSurface), true);
            slot.file = data.materialSurfaceFiles[i];
        }
    }
    for (usize i = 0; i < mesh.materials.size() && i < data.materialEmissiveFiles.size(); ++i)
    {
        MaterialTexture& slot = mesh.materials[i].textures[SlotEmissive];
        if (!data.materialEmissiveFiles[i].empty() && slot.file.empty())
        {
            slot.texture = loadTextureAsync(data.materialEmissiveFiles[i],
                                            Material::colorSpaceFor(SlotEmissive), true);
            slot.file = data.materialEmissiveFiles[i];
        }
    }
}

void AssetManager::loadMeshDataMaterialTextures(MeshData& data)
{
    for (usize i = 0; i < data.materials.size() && i < data.materialTextureFiles.size(); ++i)
    {
        MaterialTexture& slot = data.materials[i].textures[SlotAlbedo];
        if (!data.materialTextureFiles[i].empty() && slot.file.empty())
        {
            slot.texture = loadTexture(data.materialTextureFiles[i], Material::colorSpaceFor(SlotAlbedo));
            slot.file = data.materialTextureFiles[i];
        }
    }
    for (usize i = 0; i < data.materials.size() && i < data.materialNormalFiles.size(); ++i)
    {
        MaterialTexture& slot = data.materials[i].textures[SlotNormal];
        if (!data.materialNormalFiles[i].empty() && slot.file.empty())
        {
            slot.texture = loadTexture(data.materialNormalFiles[i], Material::colorSpaceFor(SlotNormal));
            slot.file = data.materialNormalFiles[i];
        }
    }
    for (usize i = 0; i < data.materials.size() && i < data.materialSurfaceFiles.size(); ++i)
    {
        MaterialTexture& slot = data.materials[i].textures[SlotSurface];
        if (!data.materialSurfaceFiles[i].empty() && slot.file.empty())
        {
            slot.texture = loadTexture(data.materialSurfaceFiles[i], Material::colorSpaceFor(SlotSurface));
            slot.file = data.materialSurfaceFiles[i];
        }
    }
    for (usize i = 0; i < data.materials.size() && i < data.materialEmissiveFiles.size(); ++i)
    {
        MaterialTexture& slot = data.materials[i].textures[SlotEmissive];
        if (!data.materialEmissiveFiles[i].empty() && slot.file.empty())
        {
            slot.texture = loadTexture(data.materialEmissiveFiles[i], Material::colorSpaceFor(SlotEmissive));
            slot.file = data.materialEmissiveFiles[i];
        }
    }
}

MeshHandle AssetManager::createMesh(const MeshData& data)
{
    Mesh mesh;
    if (!upload(data, mesh, Residency::Static))
    {
        release(mesh);
        return MeshHandle();
    }
    loadMeshMaterialTextures(mesh, data);
    return mMeshes.add(mesh);
}

MeshHandle AssetManager::createMeshAsync(const MeshDesc& desc)
{
    if (desc.source != MeshSource::File)
        return createMesh(desc);

    const std::string key = desc.key();
    const auto cached = mMeshByKey.find(key);
    if (cached != mMeshByKey.end() && mMeshes.get(cached->second))
        return cached->second;

    // An empty Mesh is intentional: RenderList sees no submeshes and skips
    // it. The handle remains valid while the worker decodes the file.
    const MeshHandle handle = mMeshes.add(Mesh());
    mMeshDescs[packHandle(handle)] = desc;
    mMeshByKey[key] = handle;

    // Only queued here, never started: launching would put a worker inside
    // MeshLoader's FileSystem reads while this same thread is still adding
    // search paths for the next mesh. processAsyncMeshLoads() starts them
    // one at a time instead, with that preparation done while nothing runs.
    PendingMesh queued;
    queued.handle = handle;
    queued.file = desc.file;
    mQueuedMeshes.push_back(std::move(queued));
    return handle;
}

u32 AssetManager::processAsyncMeshLoads()
{
    u32 completed = 0;

    if (mMeshInFlight.handle.valid())
    {
        if (mMeshInFlight.result.wait_for(std::chrono::milliseconds(0)) !=
            std::future_status::ready)
            return 0;

        MeshData data = mMeshInFlight.result.get();
        if (data.positions.empty())
            Log::error("AssetManager: async mesh import returned no data for handle %u",
                       mMeshInFlight.handle.index);
        else
        {
            // Main thread, no worker running: the one moment the material
            // list and the search paths can be touched safely.
            applyMeshFileMaterials(mMeshInFlight.file, data);
            if (!replaceMesh(mMeshInFlight.handle, data))
                Log::error("AssetManager: async mesh upload failed for handle %u",
                           mMeshInFlight.handle.index);
            else if (mRetainFileMeshData)
                mRetainedMeshData[packHandle(mMeshInFlight.handle)] = std::move(data);
        }
        ++completed;
        mMeshInFlight = PendingMesh();
    }

    if (!mMeshInFlight.handle.valid() && !mQueuedMeshes.empty())
    {
        mMeshInFlight.handle = mQueuedMeshes.front().handle;
        mMeshInFlight.file = mQueuedMeshes.front().file;
        mQueuedMeshes.erase(mQueuedMeshes.begin());

        registerMeshSearchPaths(mMeshInFlight.file);
        mMeshInFlight.result = std::async(std::launch::async, [this, file = mMeshInFlight.file]()
        {
            MeshData data;
            importMeshGeometry(file, data);
            return data;
        });
    }

    return completed;
}

u32 AssetManager::pendingAsyncMeshLoads() const
{
    return static_cast<u32>(mQueuedMeshes.size()) + (mMeshInFlight.handle.valid() ? 1u : 0u);
}

void AssetManager::setRetainFileMeshData(bool retain)
{
    mRetainFileMeshData = retain;
}

const MeshData* AssetManager::meshData(MeshHandle handle) const
{
    const auto entry = mRetainedMeshData.find(packHandle(handle));
    return entry != mRetainedMeshData.end() ? &entry->second : nullptr;
}

void AssetManager::releaseMeshData(MeshHandle handle)
{
    mRetainedMeshData.erase(packHandle(handle));
}

bool AssetManager::replaceMesh(MeshHandle handle, const MeshData& data)
{
    Mesh* slot = mMeshes.get(handle);
    if (!slot)
        return false;

    Mesh replacement;
    if (!upload(data, replacement, Residency::Static))
    {
        release(replacement);
        return false;
    }
    loadMeshMaterialTextures(replacement, data);

    // Same handle (index+generation) throughout - every MeshRenderer already
    // holding it keeps pointing at this exact slot and draws the new
    // geometry next frame without anyone telling it the handle changed. The
    // in-place counterpart to GPU::replaceTexture() for the same reason:
    // a mesh-editing tool works on a MeshData already referenced by objects
    // in the scene, and handing back a *new* handle would leave every one of
    // them still pointing at the stale mesh.
    release(*slot);
    *slot = replacement;
    return true;
}

MeshHandle AssetManager::createDynamicMesh(const MeshData& data)
{
    Mesh mesh;
    if (!upload(data, mesh, Residency::Dynamic))
    {
        release(mesh);
        return MeshHandle();
    }
    return mMeshes.add(mesh);
}

bool AssetManager::updateMeshVertices(MeshHandle handle, u32 firstVertex, u32 vertexCount,
                                      const glm::vec3* positions, const MeshAttribs* attribs)
{
    Mesh* mesh = getMesh(handle);
    if (!mesh || !positions || !attribs || vertexCount == 0 ||
        static_cast<u64>(firstVertex) + vertexCount > mesh->vertexCount)
        return false;

    GPU& gpu = GPU::getSingleton();
    gpu.updateBuffer(mesh->positionBuffer, static_cast<u64>(firstVertex) * sizeof(glm::vec3),
                     static_cast<u64>(vertexCount) * sizeof(glm::vec3), positions);
    gpu.updateBuffer(mesh->attribBuffer, static_cast<u64>(firstVertex) * sizeof(MeshAttribs),
                     static_cast<u64>(vertexCount) * sizeof(MeshAttribs), attribs);
    return true;
}

bool AssetManager::updateMeshVertices(MeshHandle handle, const MeshData& data)
{
    const usize count = data.positions.size();
    if (count == 0)
        return false;

    std::vector<MeshAttribs> attribs(count);
    for (usize i = 0; i < count; ++i)
    {
        MeshAttribs& attrib = attribs[i];
        attrib.normal = i < data.normals.size() ? data.normals[i] : glm::vec3(0.0f, 1.0f, 0.0f);
        attrib.tangent =
            i < data.tangents.size() ? data.tangents[i] : glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
        attrib.uv = i < data.uvs.size() ? data.uvs[i] : glm::vec2(0.0f);
        attrib.uv2 = i < data.uvs2.size() ? data.uvs2[i] : glm::vec2(0.0f);
        attrib.color = i < data.colors.size() ? data.colors[i] : 0xFFFFFFFFu;
    }

    return updateMeshVertices(handle, 0, static_cast<u32>(count), data.positions.data(),
                              attribs.data());
}

bool AssetManager::updateSubMeshBounds(MeshHandle handle, u32 submeshIndex, const AABB& bounds)
{
    Mesh* mesh = getMesh(handle);
    if (!mesh || submeshIndex >= mesh->submeshes.size())
        return false;

    mesh->submeshes[submeshIndex].bounds = bounds;

    AABB total;
    for (const SubMesh& sub : mesh->submeshes)
        total.merge(sub.bounds);
    mesh->bounds = total;
    return true;
}

bool AssetManager::updateMeshIndices(MeshHandle handle, const u32* indices, u32 indexCount)
{
    Mesh* mesh = getMesh(handle);
    // This only knows how to write u32 indices at the given stride, and only
    // how to grow submesh 0's count - adoptMesh() can hand the pool a Mesh
    // built with U16 indices or no submeshes at all, and this used to write
    // indexCount * sizeof(u32) bytes regardless (up to twice the index
    // buffer's actual size for a U16 mesh) and index submeshes[0]
    // unconditionally.
    if (!mesh || mesh->indexType != IndexType::U32 || mesh->submeshes.empty() ||
        indexCount > mesh->indexCount || (indexCount != 0 && !indices))
        return false;

    if (indexCount != 0)
        GPU::getSingleton().updateBuffer(mesh->indexBuffer, 0,
                                         static_cast<u64>(indexCount) * sizeof(u32), indices);
    mesh->submeshes[0].indexCount = indexCount;
    return true;
}

MeshHandle AssetManager::adoptMesh(const Mesh& mesh)
{
    return mMeshes.add(mesh);
}

void AssetManager::destroyMesh(MeshHandle handle)
{
    Mesh mesh;
    if (!mMeshes.remove(handle, mesh))
        return;

    // The pool recycles the slot, so a description left behind would end up
    // naming whatever mesh lands there next.
    const auto entry = mMeshDescs.find(packHandle(handle));
    if (entry != mMeshDescs.end())
    {
        mMeshByKey.erase(entry->second.key());
        mMeshDescs.erase(entry);
    }
    release(mesh);
}

void AssetManager::destroyAllMeshes()
{
    mMeshes.forEach(
        [this](Mesh& mesh)
        {
            release(mesh);
        });
    mMeshes.clear();
    mMeshDescs.clear();
    mMeshByKey.clear();
}

Mesh* AssetManager::getMesh(MeshHandle handle)
{
    return mMeshes.get(handle);
}

const Mesh* AssetManager::getMesh(MeshHandle handle) const
{
    return mMeshes.get(handle);
}

usize AssetManager::meshCount() const
{
    return mMeshes.liveCount();
}

bool AssetManager::exportMesh(MeshHandle handle, const std::string& filename,
                              const std::string& skeletonFile) const
{
    const Mesh* mesh = getMesh(handle);
    if (!mesh || mesh->vertexCount == 0 || mesh->indexCount == 0)
        return false;

    GPU& gpu = GPU::getSingleton();
    const u32 vertexCount = mesh->vertexCount;

    MeshData data;
    data.positions.resize(vertexCount);
    if (!gpu.readBuffer(mesh->positionBuffer, 0, static_cast<u64>(vertexCount) * sizeof(glm::vec3),
                        data.positions.data()))
        return false;

    std::vector<MeshAttribs> attribs(vertexCount);
    if (!gpu.readBuffer(mesh->attribBuffer, 0, static_cast<u64>(vertexCount) * sizeof(MeshAttribs),
                        attribs.data()))
        return false;

    data.normals.resize(vertexCount);
    data.tangents.resize(vertexCount);
    data.uvs.resize(vertexCount);
    data.uvs2.resize(vertexCount);
    data.colors.resize(vertexCount);
    for (u32 i = 0; i < vertexCount; ++i)
    {
        const MeshAttribs& attrib = attribs[i];
        data.normals[i] = attrib.normal;
        data.tangents[i] = attrib.tangent;
        data.uvs[i] = attrib.uv;
        data.uvs2[i] = attrib.uv2;
        data.colors[i] = attrib.color;
    }

    if (mesh->isSkinned())
    {
        data.skin.resize(vertexCount);
        if (!gpu.readBuffer(mesh->skinBuffer, 0,
                            static_cast<u64>(vertexCount) * sizeof(MeshSkinVertex),
                            data.skin.data()))
            return false;
    }

    const u32 indexCount = mesh->indexCount;
    data.indices.resize(indexCount);
    if (mesh->indexType == IndexType::U32)
    {
        if (!gpu.readBuffer(mesh->indexBuffer, 0, static_cast<u64>(indexCount) * sizeof(u32),
                            data.indices.data()))
            return false;
    }
    else
    {
        // Landscape's adopted meshes use U16; saveRadionMesh only writes u32.
        std::vector<u16> narrow(indexCount);
        if (!gpu.readBuffer(mesh->indexBuffer, 0, static_cast<u64>(indexCount) * sizeof(u16),
                            narrow.data()))
            return false;
        for (u32 i = 0; i < indexCount; ++i)
            data.indices[i] = narrow[i];
    }

    data.submeshes = mesh->submeshes;
    data.materials = mesh->materials;
    data.bounds = mesh->bounds;

    return saveRadionMesh(filename, data, skeletonFile);
}

bool AssetManager::saveMesh(const MeshData& mesh, const std::string& filename,
                            const std::string& skeletonFile) const
{
    return saveRadionMesh(filename, mesh, skeletonFile);
}

void AssetManager::release(Mesh& mesh) const
{
    GPU& gpu = GPU::getSingleton();
    MaterialManager& materials = MaterialManager::getSingleton();
    for (usize i = 0; i < mesh.materials.size(); ++i)
        materials.release(mesh.materials[i]);

    if (mesh.positionBuffer.valid())
        gpu.destroy(mesh.positionBuffer);
    if (mesh.attribBuffer.valid())
        gpu.destroy(mesh.attribBuffer);
    if (mesh.skinBuffer.valid())
        gpu.destroy(mesh.skinBuffer);
    if (mesh.indexBuffer.valid() && mesh.ownsIndexBuffer)
        gpu.destroy(mesh.indexBuffer);

    mesh = Mesh();
}

// ------------------------------------------------------------------- bounds

void AssetManager::computeNormals(MeshData& mesh) const
{
    if (mesh.positions.empty() || mesh.indices.size() < 3)
        return;

    mesh.normals.assign(mesh.positions.size(), glm::vec3(0.0f));
    const usize triangles = mesh.indices.size() / 3;
    for (usize t = 0; t < triangles; ++t)
    {
        const u32 i0 = mesh.indices[t * 3];
        const u32 i1 = mesh.indices[t * 3 + 1];
        const u32 i2 = mesh.indices[t * 3 + 2];
        if (i0 >= mesh.positions.size() || i1 >= mesh.positions.size() ||
            i2 >= mesh.positions.size())
            continue;
        // Not normalised: the cross product's length is twice the triangle's
        // area, so accumulating it raw weights each face by its size. A long
        // thin triangle then counts for what it is instead of as much as the
        // big one beside it.
        const glm::vec3 face = glm::cross(mesh.positions[i1] - mesh.positions[i0],
                                          mesh.positions[i2] - mesh.positions[i0]);
        mesh.normals[i0] += face;
        mesh.normals[i1] += face;
        mesh.normals[i2] += face;
    }

    for (glm::vec3& normal : mesh.normals)
    {
        const f32 length = glm::length(normal);
        normal = length > 1.0e-8f ? normal / length : glm::vec3(0.0f, 1.0f, 0.0f);
    }
}

void AssetManager::computeTangents(MeshData& mesh) const
{
    const usize count = mesh.positions.size();
    if (count == 0 || mesh.indices.size() < 3 || mesh.normals.size() != count ||
        mesh.uvs.size() != count)
        return;

    std::vector<glm::vec3> tangent(count, glm::vec3(0.0f));
    std::vector<glm::vec3> bitangent(count, glm::vec3(0.0f));

    const usize triangles = mesh.indices.size() / 3;
    for (usize t = 0; t < triangles; ++t)
    {
        const u32 i0 = mesh.indices[t * 3];
        const u32 i1 = mesh.indices[t * 3 + 1];
        const u32 i2 = mesh.indices[t * 3 + 2];
        if (i0 >= count || i1 >= count || i2 >= count)
            continue;

        const glm::vec3 e1 = mesh.positions[i1] - mesh.positions[i0];
        const glm::vec3 e2 = mesh.positions[i2] - mesh.positions[i0];
        const glm::vec2 d1 = mesh.uvs[i1] - mesh.uvs[i0];
        const glm::vec2 d2 = mesh.uvs[i2] - mesh.uvs[i0];

        // Degenerate in UV space - two vertices share a texture coordinate,
        // which happens on seams and on untextured filler geometry. There is
        // no basis to derive; leaving it at zero lets the fallback below take
        // over instead of producing an infinity.
        const f32 determinant = d1.x * d2.y - d2.x * d1.y;
        if (std::abs(determinant) < 1.0e-12f)
            continue;
        const f32 inverse = 1.0f / determinant;

        const glm::vec3 faceTangent = (e1 * d2.y - e2 * d1.y) * inverse;
        const glm::vec3 faceBitangent = (e2 * d1.x - e1 * d2.x) * inverse;
        for (const u32 index : {i0, i1, i2})
        {
            tangent[index] += faceTangent;
            bitangent[index] += faceBitangent;
        }
    }

    mesh.tangents.assign(count, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
    for (usize i = 0; i < count; ++i)
    {
        const glm::vec3& normal = mesh.normals[i];
        glm::vec3 t = tangent[i] - normal * glm::dot(normal, tangent[i]);
        if (glm::dot(t, t) < 1.0e-16f)
        {
            // Any vector perpendicular to the normal will do where the UVs
            // gave nothing - the shader needs a basis, not a correct one.
            const glm::vec3 axis =
                std::abs(normal.x) < 0.9f ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
            t = glm::normalize(glm::cross(normal, axis));
        }
        else
        {
            t = glm::normalize(t);
        }
        // w carries the handedness, which is what tells the shader whether to
        // flip the bitangent - a mirrored UV island lights inside out without.
        const f32 handedness = glm::dot(glm::cross(normal, t), bitangent[i]) < 0.0f ? -1.0f : 1.0f;
        mesh.tangents[i] = glm::vec4(t, handedness);
    }
}

u32 AssetManager::fixWinding(MeshData& mesh) const
{
    const usize triangles = mesh.indices.size() / 3;
    if (triangles < 2)
        return 0;

    // Which triangles meet at each edge, keyed by the vertex pair in
    // ascending order so the two that share it land together whichever way
    // each traverses it.
    struct Neighbour
    {
        u64 key;
        u32 triangle;
    };
    std::vector<Neighbour> edges;
    edges.reserve(triangles * 3);
    for (usize t = 0; t < triangles; ++t)
    {
        for (u32 e = 0; e < 3; ++e)
        {
            const u32 from = mesh.indices[t * 3 + e];
            const u32 to = mesh.indices[t * 3 + (e + 1) % 3];
            const u64 low = from < to ? from : to;
            const u64 high = from < to ? to : from;
            edges.push_back({(low << 32) | high, static_cast<u32>(t)});
        }
    }
    std::sort(edges.begin(), edges.end(),
              [](const Neighbour& a, const Neighbour& b) { return a.key < b.key; });

    std::vector<std::vector<u32>> adjacency(triangles);
    for (usize i = 0; i + 1 < edges.size();)
    {
        usize end = i + 1;
        while (end < edges.size() && edges[end].key == edges[i].key)
            ++end;
        if (end - i == 2)
        {
            adjacency[edges[i].triangle].push_back(edges[i + 1].triangle);
            adjacency[edges[i + 1].triangle].push_back(edges[i].triangle);
        }
        i = end;
    }

    const auto sharesEdgeSameWay = [&mesh](u32 a, u32 b)
    {
        for (u32 i = 0; i < 3; ++i)
        {
            const u32 a0 = mesh.indices[a * 3 + i];
            const u32 a1 = mesh.indices[a * 3 + (i + 1) % 3];
            for (u32 j = 0; j < 3; ++j)
            {
                const u32 b0 = mesh.indices[b * 3 + j];
                const u32 b1 = mesh.indices[b * 3 + (j + 1) % 3];
                // Same direction across a shared edge means one of them is
                // wound backwards relative to the other.
                if (a0 == b0 && a1 == b1)
                    return true;
            }
        }
        return false;
    };

    u32 flipped = 0;
    std::vector<u8> visited(triangles, 0);
    std::vector<u32> stack;
    for (usize seed = 0; seed < triangles; ++seed)
    {
        if (visited[seed])
            continue;
        visited[seed] = 1;
        stack.push_back(static_cast<u32>(seed));
        while (!stack.empty())
        {
            const u32 current = stack.back();
            stack.pop_back();
            for (const u32 neighbour : adjacency[current])
            {
                if (visited[neighbour])
                    continue;
                visited[neighbour] = 1;
                if (sharesEdgeSameWay(current, neighbour))
                {
                    std::swap(mesh.indices[neighbour * 3 + 1], mesh.indices[neighbour * 3 + 2]);
                    ++flipped;
                }
                stack.push_back(neighbour);
            }
        }
    }
    return flipped;
}

void AssetManager::computeBounds(MeshData& mesh) const
{
    mesh.bounds = AABB();
    for (usize i = 0; i < mesh.positions.size(); ++i)
        mesh.bounds.expand(mesh.positions[i]);
}

void AssetManager::computeSubMeshBounds(MeshData& mesh) const
{
    for (usize s = 0; s < mesh.submeshes.size(); ++s)
    {
        SubMesh& submesh = mesh.submeshes[s];
        submesh.bounds = AABB();

        const u32 last = submesh.indexOffset + submesh.indexCount;
        for (u32 i = submesh.indexOffset; i < last && i < mesh.indices.size(); ++i)
        {
            const u32 index = mesh.indices[i];
            if (index < mesh.positions.size())
                submesh.bounds.expand(mesh.positions[index]);
        }
    }
}

void AssetManager::splitSubMeshes(MeshData& mesh, u32 targetTriangles) const
{
    // Bind-pose vertices - the runtime BVH already refuses to index a
    // skinned mesh's submeshes for the same reason, see SceneBVH::build().
    if (!mesh.skin.empty())
        return;

    const usize before = mesh.submeshes.size();

    std::vector<SubMesh> result;
    result.reserve(mesh.submeshes.size());

    for (usize s = 0; s < mesh.submeshes.size(); ++s)
    {
        const SubMesh submesh = mesh.submeshes[s];
        if (!splitSubMeshGrid(mesh, submesh, targetTriangles, result))
            result.push_back(submesh);
    }

    mesh.submeshes = std::move(result);
    computeSubMeshBounds(mesh);

    Log::info("AssetManager: splitSubMeshes %zu -> %zu submeshes", before, mesh.submeshes.size());
}

// ------------------------------------------------------------------ normals

void AssetManager::recalculateNormals(MeshData& mesh, bool smooth, bool angleWeighted) const
{
    if (mesh.positions.empty() || mesh.indices.size() < 3)
        return;

    mesh.normals.assign(mesh.positions.size(), glm::vec3(0.0f));

    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const u32 i0 = mesh.indices[i + 0];
        const u32 i1 = mesh.indices[i + 1];
        const u32 i2 = mesh.indices[i + 2];
        if (i0 >= mesh.positions.size() || i1 >= mesh.positions.size() ||
            i2 >= mesh.positions.size())
            continue;

        const glm::vec3& v0 = mesh.positions[i0];
        const glm::vec3& v1 = mesh.positions[i1];
        const glm::vec3& v2 = mesh.positions[i2];
        const glm::vec3 normal = faceNormal(v0, v1, v2);

        if (!smooth)
        {
            mesh.normals[i0] = normal;
            mesh.normals[i1] = normal;
            mesh.normals[i2] = normal;
            continue;
        }

        const glm::vec3 weight = angleWeighted ? angleWeights(v0, v1, v2) : glm::vec3(1.0f);
        mesh.normals[i0] += normal * weight.x;
        mesh.normals[i1] += normal * weight.y;
        mesh.normals[i2] += normal * weight.z;
    }

    if (!smooth)
        return;

    for (usize i = 0; i < mesh.normals.size(); ++i)
    {
        const float length = glm::length(mesh.normals[i]);
        mesh.normals[i] = length > 0.0f ? mesh.normals[i] / length : glm::vec3(0.0f, 1.0f, 0.0f);
    }
}

void AssetManager::recalculateTangents(MeshData& mesh) const
{
    if (mesh.positions.empty() || mesh.uvs.size() != mesh.positions.size())
    {
        Log::warning("AssetManager: tangents need uvs for every vertex");
        return;
    }
    if (mesh.normals.size() != mesh.positions.size())
        recalculateNormals(mesh, true);

    std::vector<glm::vec3> tangentAccum(mesh.positions.size(), glm::vec3(0.0f));
    std::vector<glm::vec3> bitangentAccum(mesh.positions.size(), glm::vec3(0.0f));

    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const u32 i0 = mesh.indices[i + 0];
        const u32 i1 = mesh.indices[i + 1];
        const u32 i2 = mesh.indices[i + 2];
        if (i0 >= mesh.positions.size() || i1 >= mesh.positions.size() ||
            i2 >= mesh.positions.size())
            continue;

        const glm::vec3 edge1 = mesh.positions[i1] - mesh.positions[i0];
        const glm::vec3 edge2 = mesh.positions[i2] - mesh.positions[i0];
        const glm::vec2 deltaUV1 = mesh.uvs[i1] - mesh.uvs[i0];
        const glm::vec2 deltaUV2 = mesh.uvs[i2] - mesh.uvs[i0];

        // Degenerate uvs give a zero determinant; skipping leaves the vertex to
        // whatever its other triangles say instead of poisoning it with NaN.
        const float determinant = deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y;
        if (std::abs(determinant) < 1e-12f)
            continue;

        const float r = 1.0f / determinant;
        const glm::vec3 tangent = (edge1 * deltaUV2.y - edge2 * deltaUV1.y) * r;
        const glm::vec3 bitangent = (edge2 * deltaUV1.x - edge1 * deltaUV2.x) * r;

        tangentAccum[i0] += tangent;
        tangentAccum[i1] += tangent;
        tangentAccum[i2] += tangent;
        bitangentAccum[i0] += bitangent;
        bitangentAccum[i1] += bitangent;
        bitangentAccum[i2] += bitangent;
    }

    mesh.tangents.resize(mesh.positions.size());
    for (usize i = 0; i < mesh.positions.size(); ++i)
    {
        const glm::vec3& normal = mesh.normals[i];
        glm::vec3 tangent = tangentAccum[i];

        // Gram-Schmidt: drop whatever part of the tangent leans along the
        // normal, so the frame stays on the surface.
        tangent -= normal * glm::dot(normal, tangent);
        const float length = glm::length(tangent);
        tangent = length > 0.0f ? tangent / length : glm::vec3(1.0f, 0.0f, 0.0f);

        // w tells the shader which way to cross for the bitangent, which is
        // what keeps mirrored uv islands from lighting inverted.
        const float handedness =
            glm::dot(glm::cross(normal, tangent), bitangentAccum[i]) < 0.0f ? -1.0f : 1.0f;

        mesh.tangents[i] = glm::vec4(tangent, handedness);
    }
}

// ---------------------------------------------------------------- planar uv

u32 AssetManager::duplicateMeshVertex(MeshData& mesh, u32 source) const
{
    const u32 vertexIndex = static_cast<u32>(mesh.positions.size());
    mesh.resizeVertices(mesh.positions.size() + 1);
    mesh.positions[vertexIndex] = mesh.positions[source];
    if (source < mesh.normals.size())
        mesh.normals[vertexIndex] = mesh.normals[source];
    if (source < mesh.colors.size())
        mesh.colors[vertexIndex] = mesh.colors[source];
    if (source < mesh.skin.size())
        mesh.skin[vertexIndex] = mesh.skin[source];
    return vertexIndex;
}

void AssetManager::makePlanarUV(MeshData& mesh, f32 resolution) const
{
    // A shared vertex can only hold one UV, but two triangles meeting at a
    // hard edge (a box's corner is the standard case) commonly pick
    // *different* dominant axes for their own projection - whichever
    // triangle got processed last used to silently overwrite whatever the
    // other one had just written into that shared slot, leaving the loser's
    // face with someone else's UVs. The fix a real box/planar mapper uses is
    // the same one flat shading already needs (recalculateNormals()'s own
    // "split the mesh first" note): duplicate the vertex per (original
    // vertex, chosen axis) pair instead of writing through the old shared
    // index, so two faces disagreeing about the axis just get two vertices.
    // Existing SubMesh index ranges stay valid - the same number of indices,
    // in the same order, only ever renumbered to point at the (possibly new)
    // per-axis duplicate.
    HashMap<u32, u8> originalAxis;  // original vertex index -> the one axis it still owns
    HashMap<u64, u32> duplicates;   // (original index << 2 | axis) -> duplicate vertex index
    std::vector<u32> remapped(mesh.indices.size());

    const usize originalVertexCount = mesh.positions.size();
    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const u32 index[3] = {mesh.indices[i + 0], mesh.indices[i + 1], mesh.indices[i + 2]};
        if (index[0] >= originalVertexCount || index[1] >= originalVertexCount ||
            index[2] >= originalVertexCount)
        {
            remapped[i + 0] = index[0];
            remapped[i + 1] = index[1];
            remapped[i + 2] = index[2];
            continue;
        }

        const glm::vec3 normal = glm::abs(faceNormal(
            mesh.positions[index[0]], mesh.positions[index[1]], mesh.positions[index[2]]));
        const u8 axis = (normal.x > normal.y && normal.x > normal.z) ? 0
                       : (normal.y > normal.x && normal.y > normal.z) ? 1
                                                                       : 2;

        for (u32 o = 0; o < 3; ++o)
        {
            const u32 original = index[o];
            u32 vertexIndex;
            const auto ownerIt = originalAxis.find(original);
            if (ownerIt == originalAxis.end())
            {
                // First triangle to ever touch this vertex claims the
                // original slot outright - the common case, no duplicate.
                originalAxis[original] = axis;
                vertexIndex = original;
            }
            else if (ownerIt->second == axis)
                vertexIndex = original; // same axis as whoever claimed it first
            else
            {
                // A different axis than the original slot's owner - reuse a
                // duplicate already made for this (original, axis) pair, or
                // make one.
                const u64 key = (static_cast<u64>(original) << 2) | axis;
                const auto dupIt = duplicates.find(key);
                if (dupIt != duplicates.end())
                    vertexIndex = dupIt->second;
                else
                {
                    vertexIndex = duplicateMeshVertex(mesh, original);
                    duplicates[key] = vertexIndex;
                }
            }

            const glm::vec3& position = mesh.positions[vertexIndex];
            mesh.uvs.resize(mesh.positions.size());
            mesh.uvs[vertexIndex] = axis == 0   ? glm::vec2(position.y, position.z) * resolution
                                    : axis == 1 ? glm::vec2(position.x, position.z) * resolution
                                                : glm::vec2(position.x, position.y) * resolution;
            remapped[i + o] = vertexIndex;
        }
    }

    mesh.indices = std::move(remapped);
    // Stale the instant vertex count changed - a tangent is built from the
    // old topology's UVs and no longer matches. Generate Tangents already
    // has to run again after any UV change; this just makes that honest
    // instead of leaving wrong data sitting there.
    mesh.tangents.clear();
}

void AssetManager::makePlanarUV(MeshData& mesh, f32 resolutionS, f32 resolutionT, u8 axis,
                                const glm::vec3& offset) const
{
    mesh.uvs.resize(mesh.positions.size());

    for (usize i = 0; i < mesh.positions.size(); ++i)
    {
        const glm::vec3 position = mesh.positions[i] + offset;

        if (axis == 0)
            mesh.uvs[i] =
                glm::vec2(0.5f + position.z * resolutionS, 0.5f - position.y * resolutionT);
        else if (axis == 1)
            mesh.uvs[i] =
                glm::vec2(0.5f + position.x * resolutionS, 1.0f - position.z * resolutionT);
        else
            mesh.uvs[i] =
                glm::vec2(0.5f + position.x * resolutionS, 0.5f - position.y * resolutionT);
    }
}

void AssetManager::makeCylindricalUV(MeshData& mesh, f32 resolutionU, f32 resolutionV) const
{
    if (mesh.positions.empty())
        return;

    f32 minY = mesh.positions[0].y;
    f32 maxY = mesh.positions[0].y;
    for (const glm::vec3& p : mesh.positions)
    {
        minY = glm::min(minY, p.y);
        maxY = glm::max(maxY, p.y);
    }
    const f32 heightRange = glm::max(maxY - minY, 1e-6f);

    const auto rawU = [&](const glm::vec3& p) -> f32
    {
        const f32 theta = std::atan2(p.x, p.z);
        return (theta / (2.0f * glm::pi<f32>()) + 0.5f) * resolutionU;
    };
    const auto rawV = [&](const glm::vec3& p) -> f32
    { return ((p.y - minY) / heightRange) * resolutionV; };

    // Same problem makePlanarUV() has at a hard edge, here at the seam where
    // u wraps from resolutionU back to 0 (theta crossing +-pi): a shared
    // vertex right on the seam is asked for two different u values by the
    // triangles on either side of it. Unwrap per-triangle (shift whichever
    // corner would otherwise tear more than half the wrap width away from
    // the triangle's own max) and duplicate only where that disagrees with
    // whichever triangle already claimed the vertex's original slot.
    HashMap<u32, u8> originalSide; // original vertex -> 0 (as computed) or 1 (+resolutionU)
    HashMap<u64, u32> duplicates;  // (original << 1 | side) -> duplicate vertex index
    std::vector<u32> remapped(mesh.indices.size());
    const usize originalVertexCount = mesh.positions.size();

    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const u32 index[3] = {mesh.indices[i + 0], mesh.indices[i + 1], mesh.indices[i + 2]};
        if (index[0] >= originalVertexCount || index[1] >= originalVertexCount ||
            index[2] >= originalVertexCount)
        {
            remapped[i + 0] = index[0];
            remapped[i + 1] = index[1];
            remapped[i + 2] = index[2];
            continue;
        }

        f32 u[3] = {rawU(mesh.positions[index[0]]), rawU(mesh.positions[index[1]]),
                   rawU(mesh.positions[index[2]])};
        const f32 uMax = glm::max(glm::max(u[0], u[1]), u[2]);
        u8 side[3] = {0, 0, 0};
        for (u32 o = 0; o < 3; ++o)
        {
            if (uMax - u[o] > resolutionU * 0.5f)
            {
                u[o] += resolutionU;
                side[o] = 1;
            }
        }

        for (u32 o = 0; o < 3; ++o)
        {
            const u32 original = index[o];
            u32 vertexIndex;
            const auto ownerIt = originalSide.find(original);
            if (ownerIt == originalSide.end())
            {
                originalSide[original] = side[o];
                vertexIndex = original;
            }
            else if (ownerIt->second == side[o])
                vertexIndex = original;
            else
            {
                const u64 key = (static_cast<u64>(original) << 1) | side[o];
                const auto dupIt = duplicates.find(key);
                if (dupIt != duplicates.end())
                    vertexIndex = dupIt->second;
                else
                {
                    vertexIndex = duplicateMeshVertex(mesh, original);
                    duplicates[key] = vertexIndex;
                }
            }

            mesh.uvs.resize(mesh.positions.size());
            mesh.uvs[vertexIndex] = glm::vec2(u[o], rawV(mesh.positions[original]));
            remapped[i + o] = vertexIndex;
        }
    }

    mesh.indices = std::move(remapped);
    mesh.tangents.clear();
}

void AssetManager::makeSphericalUV(MeshData& mesh, f32 resolutionU, f32 resolutionV) const
{
    if (mesh.positions.empty())
        return;

    glm::vec3 center(0.0f);
    for (const glm::vec3& p : mesh.positions)
        center += p;
    center /= static_cast<f32>(mesh.positions.size());

    f32 maxRadius = 1e-6f;
    for (const glm::vec3& p : mesh.positions)
        maxRadius = glm::max(maxRadius, glm::length(p - center));

    const auto rawU = [&](const glm::vec3& p) -> f32
    {
        const f32 theta = std::atan2(p.x - center.x, p.z - center.z);
        return (theta / (2.0f * glm::pi<f32>()) + 0.5f) * resolutionU;
    };
    const auto rawV = [&](const glm::vec3& p) -> f32
    {
        const f32 y = glm::clamp((p.y - center.y) / maxRadius, -1.0f, 1.0f);
        return (std::acos(y) / glm::pi<f32>()) * resolutionV;
    };
    // Close enough to the vertical axis that atan2(x,z) stops meaning
    // anything - every triangle fanning around a pole gets its own
    // duplicate below instead of trying to share one longitude that does
    // not exist for a point sitting exactly on it.
    const auto isPole = [&](const glm::vec3& p) -> bool
    {
        const f32 horizontal = glm::length(glm::vec2(p.x - center.x, p.z - center.z));
        return horizontal < maxRadius * 0.001f;
    };

    HashMap<u32, u8> originalSide;
    HashMap<u64, u32> duplicates;
    std::vector<u32> remapped(mesh.indices.size());
    const usize originalVertexCount = mesh.positions.size();

    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const u32 index[3] = {mesh.indices[i + 0], mesh.indices[i + 1], mesh.indices[i + 2]};
        if (index[0] >= originalVertexCount || index[1] >= originalVertexCount ||
            index[2] >= originalVertexCount)
        {
            remapped[i + 0] = index[0];
            remapped[i + 1] = index[1];
            remapped[i + 2] = index[2];
            continue;
        }

        bool pole[3];
        f32 u[3];
        for (u32 o = 0; o < 3; ++o)
        {
            pole[o] = isPole(mesh.positions[index[o]]);
            u[o] = pole[o] ? 0.0f : rawU(mesh.positions[index[o]]);
        }
        f32 uMaxNonPole = -1.0f;
        for (u32 o = 0; o < 3; ++o)
            if (!pole[o])
                uMaxNonPole = glm::max(uMaxNonPole, u[o]);
        u8 side[3] = {0, 0, 0};
        for (u32 o = 0; o < 3; ++o)
        {
            if (!pole[o] && uMaxNonPole - u[o] > resolutionU * 0.5f)
            {
                u[o] += resolutionU;
                side[o] = 1;
            }
        }
        // A pole corner has no longitude of its own - average whatever the
        // other two corners settled on so the triangle fan does not swirl.
        if (pole[0] || pole[1] || pole[2])
        {
            f32 sum = 0.0f;
            int count = 0;
            for (u32 o = 0; o < 3; ++o)
                if (!pole[o])
                {
                    sum += u[o];
                    ++count;
                }
            const f32 average = count > 0 ? sum / static_cast<f32>(count) : 0.0f;
            for (u32 o = 0; o < 3; ++o)
                if (pole[o])
                    u[o] = average;
        }

        for (u32 o = 0; o < 3; ++o)
        {
            const u32 original = index[o];
            u32 vertexIndex;
            if (pole[o])
            {
                // Never shared, unlike the ordinary seam case below - every
                // triangle touching a pole needs its own vertex, since no
                // single longitude would ever serve all of them.
                vertexIndex = duplicateMeshVertex(mesh, original);
            }
            else
            {
                const auto ownerIt = originalSide.find(original);
                if (ownerIt == originalSide.end())
                {
                    originalSide[original] = side[o];
                    vertexIndex = original;
                }
                else if (ownerIt->second == side[o])
                    vertexIndex = original;
                else
                {
                    const u64 key = (static_cast<u64>(original) << 1) | side[o];
                    const auto dupIt = duplicates.find(key);
                    if (dupIt != duplicates.end())
                        vertexIndex = dupIt->second;
                    else
                    {
                        vertexIndex = duplicateMeshVertex(mesh, original);
                        duplicates[key] = vertexIndex;
                    }
                }
            }

            mesh.uvs.resize(mesh.positions.size());
            mesh.uvs[vertexIndex] = glm::vec2(u[o], rawV(mesh.positions[original]));
            remapped[i + o] = vertexIndex;
        }
    }

    mesh.indices = std::move(remapped);
    mesh.tangents.clear();
}

// --------------------------------------------------------------- transforms

void AssetManager::translate(MeshData& mesh, const glm::vec3& delta) const
{
    for (usize i = 0; i < mesh.positions.size(); ++i)
        mesh.positions[i] += delta;

    computeBounds(mesh);
    computeSubMeshBounds(mesh);
}

void AssetManager::scale(MeshData& mesh, const glm::vec3& factor) const
{
    for (usize i = 0; i < mesh.positions.size(); ++i)
        mesh.positions[i] *= factor;

    // A negative scale on an odd number of axes turns the mesh inside out.
    if (factor.x * factor.y * factor.z < 0.0f)
        flipWinding(mesh);

    if (!mesh.normals.empty())
        recalculateNormals(mesh, true);

    computeBounds(mesh);
    computeSubMeshBounds(mesh);
}

void AssetManager::transform(MeshData& mesh, const glm::mat4& matrix) const
{
    const glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(matrix)));

    for (usize i = 0; i < mesh.positions.size(); ++i)
        mesh.positions[i] = glm::vec3(matrix * glm::vec4(mesh.positions[i], 1.0f));

    for (usize i = 0; i < mesh.normals.size(); ++i)
        mesh.normals[i] = glm::normalize(normalMatrix * mesh.normals[i]);

    for (usize i = 0; i < mesh.tangents.size(); ++i)
    {
        const glm::vec3 tangent = glm::normalize(normalMatrix * glm::vec3(mesh.tangents[i]));
        mesh.tangents[i] = glm::vec4(tangent, mesh.tangents[i].w);
    }

    if (glm::determinant(glm::mat3(matrix)) < 0.0f)
        flipWinding(mesh);

    computeBounds(mesh);
    computeSubMeshBounds(mesh);
}

void AssetManager::transformVertices(MeshData& mesh, const glm::mat4& matrix,
                                     const std::vector<u32>& vertexIndices) const
{
    const usize vertexCount = mesh.positions.size();
    if (vertexCount == 0)
        return;

    const bool wholeMesh = vertexIndices.empty();
    const usize affected = wholeMesh ? vertexCount : vertexIndices.size();

    glm::dvec3 sum(0.0);
    usize counted = 0;
    for (usize i = 0; i < affected; ++i)
    {
        const usize index = wholeMesh ? i : static_cast<usize>(vertexIndices[i]);
        if (index >= vertexCount)
            continue;
        sum += glm::dvec3(mesh.positions[index]);
        ++counted;
    }
    if (counted == 0)
        return;

    // Accumulated in double: a median over hundreds of thousands of vertices
    // far from the origin loses enough in float to visibly shift the pivot.
    transformVerticesAbout(mesh, matrix, glm::vec3(sum / static_cast<double>(counted)),
                           vertexIndices);
}

void AssetManager::transformVerticesAbout(MeshData& mesh, const glm::mat4& matrix,
                                          const glm::vec3& pivot,
                                          const std::vector<u32>& vertexIndices) const
{
    const usize vertexCount = mesh.positions.size();
    if (vertexCount == 0)
        return;

    const bool wholeMesh = vertexIndices.empty();
    const usize affected = wholeMesh ? vertexCount : vertexIndices.size();

    const glm::mat4 aboutPivot =
        glm::translate(glm::mat4(1.0f), pivot) * matrix * glm::translate(glm::mat4(1.0f), -pivot);
    const glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(aboutPivot)));

    const bool hasNormals = !mesh.normals.empty();
    const bool hasTangents = !mesh.tangents.empty();

    for (usize i = 0; i < affected; ++i)
    {
        const usize index = wholeMesh ? i : static_cast<usize>(vertexIndices[i]);
        if (index >= vertexCount)
            continue;

        mesh.positions[index] = glm::vec3(aboutPivot * glm::vec4(mesh.positions[index], 1.0f));

        if (hasNormals && index < mesh.normals.size())
            mesh.normals[index] = glm::normalize(normalMatrix * mesh.normals[index]);

        if (hasTangents && index < mesh.tangents.size())
        {
            const glm::vec3 tangent = glm::normalize(normalMatrix * glm::vec3(mesh.tangents[index]));
            mesh.tangents[index] = glm::vec4(tangent, mesh.tangents[index].w);
        }
    }

    // Winding is a property of a triangle, not of a vertex: flipping the
    // whole mesh because part of it was mirrored would turn the untouched
    // faces inside out too. Only the whole-mesh case can say anything.
    if (wholeMesh && glm::determinant(glm::mat3(matrix)) < 0.0f)
        flipWinding(mesh);

    computeBounds(mesh);
    computeSubMeshBounds(mesh);
}

void AssetManager::center(MeshData& mesh) const
{
    computeBounds(mesh);
    if (mesh.bounds.empty())
        return;

    translate(mesh, -mesh.bounds.center());
}

void AssetManager::centerOnGround(MeshData& mesh) const
{
    computeBounds(mesh);
    if (mesh.bounds.empty())
        return;

    const glm::vec3 center = mesh.bounds.center();
    translate(mesh, glm::vec3(-center.x, -mesh.bounds.min.y, -center.z));
}

void AssetManager::flipWinding(MeshData& mesh) const
{
    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const u32 swap = mesh.indices[i + 1];
        mesh.indices[i + 1] = mesh.indices[i + 2];
        mesh.indices[i + 2] = swap;
    }
}

void AssetManager::flipWinding(MeshData& mesh, u32 submeshIndex) const
{
    if (submeshIndex >= mesh.submeshes.size())
        return;

    const SubMesh& submesh = mesh.submeshes[submeshIndex];
    const u64 end = static_cast<u64>(submesh.indexOffset) + submesh.indexCount;
    if (end > mesh.indices.size())
        return;

    for (u32 i = submesh.indexOffset; i + 2 < submesh.indexOffset + submesh.indexCount; i += 3)
    {
        const u32 swap = mesh.indices[i + 1];
        mesh.indices[i + 1] = mesh.indices[i + 2];
        mesh.indices[i + 2] = swap;
    }
}

// ------------------------------------------------------------ decomposition

// Appends one input's vertices to `out`, transformed if asked, keeping every
// attribute array in step: an array that some input has and another has not
// is padded, or the arrays drift and vertex i stops meaning the same vertex
// across them.
namespace
{
void appendVertices(MeshData& out, const MeshData& source, const glm::mat4& transform,
                    bool applyTransform)
{
    const usize base = out.positions.size();
    const usize count = source.positions.size();
    // A normal is not transformed by the same matrix as a position: a
    // non-uniform scale would leave it off the surface.
    const glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(transform)));

    out.positions.reserve(base + count);
    for (usize i = 0; i < count; ++i)
        out.positions.push_back(applyTransform
                                    ? glm::vec3(transform * glm::vec4(source.positions[i], 1.0f))
                                    : source.positions[i]);

    if (!out.normals.empty() || !source.normals.empty())
    {
        out.normals.resize(base, glm::vec3(0.0f, 1.0f, 0.0f));
        for (usize i = 0; i < count; ++i)
        {
            const glm::vec3 normal =
                i < source.normals.size() ? source.normals[i] : glm::vec3(0.0f, 1.0f, 0.0f);
            out.normals.push_back(applyTransform ? glm::normalize(normalMatrix * normal) : normal);
        }
    }
    if (!out.tangents.empty() || !source.tangents.empty())
    {
        out.tangents.resize(base, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
        for (usize i = 0; i < count; ++i)
        {
            const glm::vec4 tangent =
                i < source.tangents.size() ? source.tangents[i] : glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
            // w carries handedness, not a coordinate - it never goes through
            // the matrix.
            out.tangents.push_back(applyTransform ? glm::vec4(glm::normalize(glm::mat3(transform) *
                                                                             glm::vec3(tangent)),
                                                              tangent.w)
                                                  : tangent);
        }
    }
    if (!out.uvs.empty() || !source.uvs.empty())
    {
        out.uvs.resize(base, glm::vec2(0.0f));
        for (usize i = 0; i < count; ++i)
            out.uvs.push_back(i < source.uvs.size() ? source.uvs[i] : glm::vec2(0.0f));
    }
    if (!out.uvs2.empty() || !source.uvs2.empty())
    {
        out.uvs2.resize(base, glm::vec2(0.0f));
        for (usize i = 0; i < count; ++i)
            out.uvs2.push_back(i < source.uvs2.size() ? source.uvs2[i] : glm::vec2(0.0f));
    }
    if (!out.colors.empty() || !source.colors.empty())
    {
        out.colors.resize(base, 0xFFFFFFFFu);
        for (usize i = 0; i < count; ++i)
            out.colors.push_back(i < source.colors.size() ? source.colors[i] : 0xFFFFFFFFu);
    }
    if (!out.skin.empty() || !source.skin.empty())
    {
        out.skin.resize(base, MeshSkinVertex());
        for (usize i = 0; i < count; ++i)
            out.skin.push_back(i < source.skin.size() ? source.skin[i] : MeshSkinVertex());
    }
}
} // namespace

bool AssetManager::mergeMeshes(const std::vector<MeshMergeInput>& inputs,
                               const MeshMergeOptions& options, MeshData& output,
                               std::string* error) const
{
    if (inputs.empty())
    {
        if (error)
            *error = "no inputs";
        return false;
    }

    u64 totalVertices = 0;
    for (const MeshMergeInput& input : inputs)
    {
        if (!input.mesh)
        {
            if (error)
                *error = "null mesh in inputs";
            return false;
        }
        totalVertices += input.mesh->positions.size();
    }
    // One shared index buffer addresses every vertex, so the merged mesh has
    // to stay inside what a u32 index can reach whatever the caller asked for.
    const u64 limit = options.maxVertices ? options.maxVertices : 0xFFFFFFFFull;
    if (totalVertices > limit)
    {
        if (error)
            *error = "merged mesh would exceed " + std::to_string(limit) + " vertices";
        return false;
    }

    output.clear();
    for (const MeshMergeInput& input : inputs)
    {
        const MeshData& source = *input.mesh;
        if (source.positions.empty() || source.indices.empty())
            continue;

        const u32 vertexBase = static_cast<u32>(output.positions.size());
        appendVertices(output, source, input.transform, options.applyTransforms);

        // Each source submesh becomes an output submesh - geometry groups are
        // never merged across materials, only their slots are remapped. With
        // preserveSubmeshBoundaries off, a source's submeshes that share one
        // material still come out as one group.
        const std::vector<SubMesh>& submeshes = source.submeshes;
        for (usize s = 0; s < submeshes.size(); ++s)
        {
            const SubMesh& submesh = submeshes[s];
            if (submesh.indexCount == 0 ||
                static_cast<u64>(submesh.indexOffset) + submesh.indexCount > source.indices.size())
                continue;

            // Deduplicate the material, not the geometry: the same material
            // arriving from two inputs gets one slot.
            u32 slot = 0;
            if (submesh.materialSlot < source.materials.size())
            {
                const Material& material = source.materials[submesh.materialSlot];
                const std::string& albedo =
                    submesh.materialSlot < source.materialTextureFiles.size()
                        ? source.materialTextureFiles[submesh.materialSlot]
                        : std::string();
                bool found = false;
                for (usize m = 0; m < output.materials.size() && !found; ++m)
                {
                    const std::string& existing = m < output.materialTextureFiles.size()
                                                      ? output.materialTextureFiles[m]
                                                      : std::string();
                    // Not memcmp(&output.materials[m], &material, sizeof(Material)) -
                    // Material holds a std::string (name) and struct padding
                    // that is not guaranteed equal between two independently
                    // constructed copies of the identical logical material,
                    // so a raw byte compare found two submeshes sharing the
                    // exact same source Material "different" and duplicated
                    // it once per submesh instead of reusing the one slot.
                    const Material& candidate = output.materials[m];
                    if (existing == albedo && candidate.name == material.name &&
                        candidate.flags == material.flags && candidate.blend == material.blend &&
                        candidate.cull == material.cull &&
                        std::memcmp(&candidate.params, &material.params, sizeof(MaterialParams)) ==
                            0)
                    {
                        slot = static_cast<u32>(m);
                        found = true;
                    }
                }
                if (!found)
                {
                    slot = static_cast<u32>(output.materials.size());
                    output.materials.push_back(material);
                    output.materialTextureFiles.push_back(albedo);
                    output.materialNormalFiles.push_back(
                        submesh.materialSlot < source.materialNormalFiles.size()
                            ? source.materialNormalFiles[submesh.materialSlot]
                            : std::string());
                }
            }
            else
            {
                if (output.materials.empty())
                {
                    output.materials.push_back(Material());
                    output.materialTextureFiles.push_back(std::string());
                    output.materialNormalFiles.push_back(std::string());
                }
            }

            SubMesh* target = nullptr;
            if (!options.preserveSubmeshBoundaries && !output.submeshes.empty() &&
                output.submeshes.back().materialSlot == slot &&
                output.submeshes.back().indexOffset + output.submeshes.back().indexCount ==
                    static_cast<u32>(output.indices.size()))
                target = &output.submeshes.back();

            const u32 indexBase = static_cast<u32>(output.indices.size());
            output.indices.reserve(output.indices.size() + submesh.indexCount);
            for (u32 i = 0; i < submesh.indexCount; ++i)
                output.indices.push_back(source.indices[submesh.indexOffset + i] + vertexBase);

            if (target)
            {
                target->indexCount += submesh.indexCount;
            }
            else
            {
                SubMesh merged;
                merged.indexOffset = indexBase;
                merged.indexCount = submesh.indexCount;
                merged.materialSlot = slot;
                merged.lightmapPage = submesh.lightmapPage;
                output.submeshes.push_back(merged);
            }
        }
    }

    if (output.positions.empty() || output.indices.empty())
    {
        if (error)
            *error = "inputs held no geometry";
        return false;
    }

    computeBounds(output);
    computeSubMeshBounds(output);
    return true;
}

bool AssetManager::mergeSubmeshes(MeshData& mesh, bool preserveSubmeshBoundaries) const
{
    if (mesh.submeshes.size() < 2)
        return false;

    MeshMergeInput input;
    input.mesh = &mesh;
    MeshMergeOptions options;
    options.applyTransforms = false;
    options.preserveSubmeshBoundaries = preserveSubmeshBoundaries;

    MeshData merged;
    if (!mergeMeshes({input}, options, merged, nullptr))
        return false;
    mesh = merged;
    return true;
}

bool AssetManager::mergeSubmeshesByMaterial(MeshData& mesh) const
{
    if (mesh.submeshes.size() < 2)
        return false;

    struct MaterialGroup
    {
        u32 materialSlot = 0;
        u32 lightmapPage = 0;
        std::vector<u32> indices;
    };

    std::vector<MaterialGroup> groups;
    usize dropped = 0;
    for (usize s = 0; s < mesh.submeshes.size(); ++s)
    {
        const SubMesh& submesh = mesh.submeshes[s];
        if (submesh.indexCount == 0 ||
            static_cast<u64>(submesh.indexOffset) + submesh.indexCount > mesh.indices.size())
        {
            ++dropped;
            continue;
        }

        MaterialGroup* target = nullptr;
        for (usize g = 0; g < groups.size(); ++g)
        {
            if (groups[g].materialSlot == submesh.materialSlot &&
                groups[g].lightmapPage == submesh.lightmapPage)
            {
                target = &groups[g];
                break;
            }
        }
        if (!target)
        {
            groups.push_back(MaterialGroup());
            target = &groups.back();
            target->materialSlot = submesh.materialSlot;
            target->lightmapPage = submesh.lightmapPage;
        }

        target->indices.reserve(target->indices.size() + submesh.indexCount);
        for (u32 i = 0; i < submesh.indexCount; ++i)
            target->indices.push_back(mesh.indices[submesh.indexOffset + i]);
    }

    if (dropped == 0 && groups.size() >= mesh.submeshes.size())
        return false;
    if (dropped > 0)
        Log::warning("AssetManager: mergeSubmeshesByMaterial dropped %zu empty or out-of-range "
                     "submeshes of %zu",
                     dropped, mesh.submeshes.size());

    std::vector<u32> newIndices;
    newIndices.reserve(mesh.indices.size());
    std::vector<SubMesh> newSubmeshes;
    newSubmeshes.reserve(groups.size());

    for (usize g = 0; g < groups.size(); ++g)
    {
        SubMesh submesh;
        submesh.indexOffset = static_cast<u32>(newIndices.size());
        submesh.indexCount = static_cast<u32>(groups[g].indices.size());
        submesh.materialSlot = groups[g].materialSlot;
        submesh.lightmapPage = groups[g].lightmapPage;
        newSubmeshes.push_back(submesh);
        newIndices.insert(newIndices.end(), groups[g].indices.begin(), groups[g].indices.end());
    }

    mesh.indices = std::move(newIndices);
    mesh.submeshes = std::move(newSubmeshes);
    computeSubMeshBounds(mesh);
    return true;
}

bool AssetManager::extractSubmesh(const MeshData& source, u32 submeshIndex, MeshData& out) const
{
    if (submeshIndex >= source.submeshes.size())
        return false;

    const SubMesh& submesh = source.submeshes[submeshIndex];
    if (submesh.indexCount == 0 ||
        static_cast<u64>(submesh.indexOffset) + submesh.indexCount > source.indices.size())
        return false;

    out.clear();

    const bool hasNormals = source.normals.size() == source.positions.size();
    const bool hasTangents = source.tangents.size() == source.positions.size();
    const bool hasUVs = source.uvs.size() == source.positions.size();
    const bool hasColors = source.colors.size() == source.positions.size();
    const bool hasSkin = source.skin.size() == source.positions.size();

    // Old index -> compact new index. The submesh's indices are a sparse
    // subset of source's shared buffers, so every vertex is copied at most
    // once no matter how many triangles reference it.
    HashMap<u32, u32> remap;
    out.indices.reserve(submesh.indexCount);

    for (u32 i = 0; i < submesh.indexCount; ++i)
    {
        const u32 oldIndex = source.indices[submesh.indexOffset + i];
        if (oldIndex >= source.positions.size())
            return false;

        u32 newIndex;
        const auto it = remap.find(oldIndex);
        if (it == remap.end())
        {
            newIndex = static_cast<u32>(out.positions.size());
            remap.emplace(oldIndex, newIndex);

            out.positions.push_back(source.positions[oldIndex]);
            if (hasNormals)
                out.normals.push_back(source.normals[oldIndex]);
            if (hasTangents)
                out.tangents.push_back(source.tangents[oldIndex]);
            if (hasUVs)
                out.uvs.push_back(source.uvs[oldIndex]);
            if (hasColors)
                out.colors.push_back(source.colors[oldIndex]);
            if (hasSkin)
                out.skin.push_back(source.skin[oldIndex]);
        }
        else
        {
            newIndex = it->second;
        }

        out.indices.push_back(newIndex);
    }

    SubMesh newSubmesh;
    newSubmesh.indexOffset = 0;
    newSubmesh.indexCount = static_cast<u32>(out.indices.size());
    newSubmesh.materialSlot = 0;
    out.submeshes.push_back(newSubmesh);

    if (submesh.materialSlot < source.materials.size())
        out.materials.push_back(source.materials[submesh.materialSlot]);
    if (submesh.materialSlot < source.materialTextureFiles.size())
        out.materialTextureFiles.push_back(source.materialTextureFiles[submesh.materialSlot]);
    if (submesh.materialSlot < source.materialNormalFiles.size())
        out.materialNormalFiles.push_back(source.materialNormalFiles[submesh.materialSlot]);

    computeBounds(out);
    computeSubMeshBounds(out);

    return true;
}

bool AssetManager::removeSubmesh(MeshData& mesh, u32 submeshIndex) const
{
    if (submeshIndex >= mesh.submeshes.size())
        return false;

    mesh.submeshes.erase(mesh.submeshes.begin() + submeshIndex);
    return true;
}

u32 AssetManager::compactGeometry(MeshData& mesh) const
{
    const usize originalVertexCount = mesh.positions.size();
    if (mesh.submeshes.empty() || mesh.indices.empty())
        return 0;

    // Which vertices any surviving submesh still names. Everything else is
    // dead weight the file has been carrying since the first deletion.
    std::vector<u32> remap(originalVertexCount, 0xFFFFFFFFu);
    std::vector<u32> newIndices;
    newIndices.reserve(mesh.indices.size());
    std::vector<u32> keptOrder;
    keptOrder.reserve(originalVertexCount);

    std::vector<SubMesh> newSubmeshes;
    newSubmeshes.reserve(mesh.submeshes.size());
    for (const SubMesh& submesh : mesh.submeshes)
    {
        SubMesh rebuilt = submesh;
        rebuilt.indexOffset = static_cast<u32>(newIndices.size());
        rebuilt.indexCount = 0;

        const usize end =
            glm::min<usize>(submesh.indexOffset + submesh.indexCount, mesh.indices.size());
        for (usize i = submesh.indexOffset; i < end; ++i)
        {
            const u32 vertex = mesh.indices[i];
            if (vertex >= originalVertexCount)
                continue;
            if (remap[vertex] == 0xFFFFFFFFu)
            {
                remap[vertex] = static_cast<u32>(keptOrder.size());
                keptOrder.push_back(vertex);
            }
            newIndices.push_back(remap[vertex]);
            ++rebuilt.indexCount;
        }
        newSubmeshes.push_back(rebuilt);
    }

    if (keptOrder.size() == originalVertexCount && newIndices.size() == mesh.indices.size())
        return 0;

    // Every per-vertex stream is parallel to positions and has to be rebuilt
    // in the same new order - a stream left behind would pair the wrong
    // normal or UV with each vertex from here on.
    const auto compactStream = [&keptOrder](auto& stream)
    {
        if (stream.empty())
            return;
        using Element = typename std::decay_t<decltype(stream)>::value_type;
        std::vector<Element> rebuilt;
        rebuilt.reserve(keptOrder.size());
        for (u32 vertex : keptOrder)
            rebuilt.push_back(vertex < stream.size() ? stream[vertex] : Element{});
        stream = std::move(rebuilt);
    };
    compactStream(mesh.positions);
    compactStream(mesh.normals);
    compactStream(mesh.tangents);
    compactStream(mesh.uvs);
    compactStream(mesh.uvs2);
    compactStream(mesh.colors);
    compactStream(mesh.skin);

    mesh.indices = std::move(newIndices);
    mesh.submeshes = std::move(newSubmeshes);
    computeBounds(mesh);
    computeSubMeshBounds(mesh);
    return static_cast<u32>(originalVertexCount - keptOrder.size());
}

u32 AssetManager::compactMaterials(MeshData& mesh, std::vector<u32>* outRemap) const
{
    std::vector<bool> used(mesh.materials.size(), false);
    for (const SubMesh& submesh : mesh.submeshes)
        if (submesh.materialSlot < used.size())
            used[submesh.materialSlot] = true;

    u32 removed = 0;
    for (bool slotUsed : used)
        if (!slotUsed)
            ++removed;
    if (removed == 0)
    {
        // Identity, so a caller can put its own per-slot state through the
        // same table unconditionally instead of special-casing "nothing
        // moved".
        if (outRemap)
        {
            outRemap->resize(used.size());
            for (usize slot = 0; slot < used.size(); ++slot)
                (*outRemap)[slot] = static_cast<u32>(slot);
        }
        return 0;
    }

    std::vector<u32> remap(used.size(), kInvalidMaterialSlot);
    std::vector<Material> keptMaterials;
    std::vector<std::string> keptTextureFiles;
    std::vector<std::string> keptNormalFiles;
    std::vector<std::string> keptSurfaceFiles;
    std::vector<std::string> keptEmissiveFiles;
    std::vector<std::string> keptHeightFiles;
    keptMaterials.reserve(used.size() - removed);

    // Every one of these arrays is indexed BY material slot, so a kept slot
    // has to produce exactly one entry in each - pushing only when the
    // source array happens to be long enough (importers fill them to
    // different lengths) shifts every later entry onto the wrong material.
    const auto keepFile = [](const std::vector<std::string>& source, usize slot,
                             std::vector<std::string>& out)
    {
        out.push_back(slot < source.size() ? source[slot] : std::string());
    };

    for (usize slot = 0; slot < used.size(); ++slot)
    {
        if (!used[slot])
            continue;
        remap[slot] = static_cast<u32>(keptMaterials.size());
        keptMaterials.push_back(mesh.materials[slot]);
        keepFile(mesh.materialTextureFiles, slot, keptTextureFiles);
        keepFile(mesh.materialNormalFiles, slot, keptNormalFiles);
        keepFile(mesh.materialSurfaceFiles, slot, keptSurfaceFiles);
        keepFile(mesh.materialEmissiveFiles, slot, keptEmissiveFiles);
        keepFile(mesh.materialHeightFiles, slot, keptHeightFiles);
    }

    for (SubMesh& submesh : mesh.submeshes)
        if (submesh.materialSlot < remap.size() &&
            remap[submesh.materialSlot] != kInvalidMaterialSlot)
            submesh.materialSlot = remap[submesh.materialSlot];

    mesh.materials = std::move(keptMaterials);
    mesh.materialTextureFiles = std::move(keptTextureFiles);
    mesh.materialNormalFiles = std::move(keptNormalFiles);
    mesh.materialSurfaceFiles = std::move(keptSurfaceFiles);
    mesh.materialEmissiveFiles = std::move(keptEmissiveFiles);
    mesh.materialHeightFiles = std::move(keptHeightFiles);
    if (outRemap)
        *outRemap = std::move(remap);
    return removed;
}

// ------------------------------------------------------------- optimization

namespace
{

template <typename T>
void remapVertexArray(std::vector<T>& array, const std::vector<u32>& remap, usize uniqueCount)
{
    if (array.empty())
        return;
    std::vector<T> result(uniqueCount);
    meshopt_remapVertexBuffer(result.data(), array.data(), array.size(), sizeof(T), remap.data());
    array = std::move(result);
}

usize gatherVertexStreams(const MeshData& mesh, meshopt_Stream* streams)
{
    const usize vertexCount = mesh.positions.size();
    usize count = 0;
    streams[count++] = {mesh.positions.data(), sizeof(glm::vec3), sizeof(glm::vec3)};
    if (mesh.normals.size() == vertexCount)
        streams[count++] = {mesh.normals.data(), sizeof(glm::vec3), sizeof(glm::vec3)};
    if (mesh.tangents.size() == vertexCount)
        streams[count++] = {mesh.tangents.data(), sizeof(glm::vec4), sizeof(glm::vec4)};
    if (mesh.uvs.size() == vertexCount)
        streams[count++] = {mesh.uvs.data(), sizeof(glm::vec2), sizeof(glm::vec2)};
    if (mesh.uvs2.size() == vertexCount)
        streams[count++] = {mesh.uvs2.data(), sizeof(glm::vec2), sizeof(glm::vec2)};
    if (mesh.colors.size() == vertexCount)
        streams[count++] = {mesh.colors.data(), sizeof(u32), sizeof(u32)};
    if (mesh.skin.size() == vertexCount)
        streams[count++] = {mesh.skin.data(), sizeof(MeshSkinVertex), sizeof(MeshSkinVertex)};
    return count;
}

void remapAllVertexArrays(MeshData& mesh, const std::vector<u32>& remap, usize uniqueCount)
{
    remapVertexArray(mesh.positions, remap, uniqueCount);
    remapVertexArray(mesh.normals, remap, uniqueCount);
    remapVertexArray(mesh.tangents, remap, uniqueCount);
    remapVertexArray(mesh.uvs, remap, uniqueCount);
    remapVertexArray(mesh.uvs2, remap, uniqueCount);
    remapVertexArray(mesh.colors, remap, uniqueCount);
    remapVertexArray(mesh.skin, remap, uniqueCount);
}

} // namespace

u32 AssetManager::weldVertices(MeshData& mesh) const
{
    const usize vertexCount = mesh.positions.size();
    if (vertexCount == 0 || mesh.indices.empty())
        return 0;

    meshopt_Stream streams[7];
    const usize streamCount = gatherVertexStreams(mesh, streams);

    std::vector<u32> remap(vertexCount);
    const usize uniqueCount =
        meshopt_generateVertexRemapMulti(remap.data(), mesh.indices.data(), mesh.indices.size(),
                                         vertexCount, streams, streamCount);
    if (uniqueCount >= vertexCount)
        return 0;

    meshopt_remapIndexBuffer(mesh.indices.data(), mesh.indices.data(), mesh.indices.size(),
                             remap.data());
    remapAllVertexArrays(mesh, remap, uniqueCount);
    return static_cast<u32>(vertexCount - uniqueCount);
}

namespace
{

u32 findWeldRoot(std::vector<u32>& parent, u32 x)
{
    while (parent[x] != x)
    {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    return x;
}

void unionWeldRoots(std::vector<u32>& parent, u32 a, u32 b)
{
    const u32 rootA = findWeldRoot(parent, a);
    const u32 rootB = findWeldRoot(parent, b);
    if (rootA != rootB)
        parent[rootA] = rootB;
}

// Packs a 3D grid cell into one key, 21 bits per axis - a generous range for
// an edit-time tolerance weld on one mesh's local coordinates.
u64 weldCellKey(const glm::ivec3& cell)
{
    const u64 x = static_cast<u64>(static_cast<u32>(cell.x)) & 0x1FFFFFu;
    const u64 y = static_cast<u64>(static_cast<u32>(cell.y)) & 0x1FFFFFu;
    const u64 z = static_cast<u64>(static_cast<u32>(cell.z)) & 0x1FFFFFu;
    return (x << 42) | (y << 21) | z;
}

glm::ivec3 weldCellOf(const glm::vec3& position, f32 cellSize)
{
    return glm::ivec3(glm::floor(position / cellSize));
}

} // namespace

u32 AssetManager::weldVertices(MeshData& mesh, f32 distance, const std::vector<u32>& vertexIndices) const
{
    const usize vertexCount = mesh.positions.size();
    if (vertexCount == 0 || mesh.indices.size() < 3 || distance <= 0.0f)
        return 0;

    std::vector<bool> eligible(vertexCount, vertexIndices.empty());
    if (!vertexIndices.empty())
        for (u32 v : vertexIndices)
            if (v < vertexCount)
                eligible[v] = true;

    HashMap<u64, std::vector<u32>> buckets;
    for (u32 i = 0; i < vertexCount; ++i)
        if (eligible[i])
            buckets[weldCellKey(weldCellOf(mesh.positions[i], distance))].push_back(i);

    std::vector<u32> parent(vertexCount);
    for (u32 i = 0; i < vertexCount; ++i)
        parent[i] = i;

    for (u32 i = 0; i < vertexCount; ++i)
    {
        if (!eligible[i])
            continue;
        const glm::ivec3 cell = weldCellOf(mesh.positions[i], distance);
        for (s32 dz = -1; dz <= 1; ++dz)
            for (s32 dy = -1; dy <= 1; ++dy)
                for (s32 dx = -1; dx <= 1; ++dx)
                {
                    const auto it = buckets.find(weldCellKey(cell + glm::ivec3(dx, dy, dz)));
                    if (it == buckets.end())
                        continue;
                    for (u32 j : it->second)
                    {
                        if (j <= i)
                            continue;
                        if (glm::distance(mesh.positions[i], mesh.positions[j]) <= distance)
                            unionWeldRoots(parent, i, j);
                    }
                }
    }

    std::vector<u32> groupSize(vertexCount, 0);
    std::vector<glm::vec3> groupSum(vertexCount, glm::vec3(0.0f));
    u32 mergedCount = 0;
    for (u32 i = 0; i < vertexCount; ++i)
    {
        const u32 root = findWeldRoot(parent, i);
        groupSize[root] += 1;
        groupSum[root] += mesh.positions[i];
        if (root != i)
            ++mergedCount;
    }
    if (mergedCount == 0)
        return 0;

    for (u32 i = 0; i < vertexCount; ++i)
        if (groupSize[i] > 1)
            mesh.positions[i] = groupSum[i] / static_cast<f32>(groupSize[i]);

    for (u32& index : mesh.indices)
        index = findWeldRoot(parent, index);

    const bool hadNoSubmeshes = mesh.submeshes.empty();
    std::vector<SubMesh> sourceSubmeshes = mesh.submeshes;
    if (sourceSubmeshes.empty())
    {
        SubMesh whole;
        whole.indexOffset = 0;
        whole.indexCount = static_cast<u32>(mesh.indices.size());
        sourceSubmeshes.push_back(whole);
    }

    std::vector<u32> newIndices;
    newIndices.reserve(mesh.indices.size());
    std::vector<SubMesh> newSubmeshes;
    newSubmeshes.reserve(sourceSubmeshes.size());
    for (const SubMesh& submesh : sourceSubmeshes)
    {
        SubMesh kept = submesh;
        kept.indexOffset = static_cast<u32>(newIndices.size());
        kept.indexCount = 0;
        const u64 end = static_cast<u64>(submesh.indexOffset) + submesh.indexCount;
        for (u32 i = submesh.indexOffset; i + 2 < end && i + 2 < mesh.indices.size(); i += 3)
        {
            const u32 i0 = mesh.indices[i + 0];
            const u32 i1 = mesh.indices[i + 1];
            const u32 i2 = mesh.indices[i + 2];
            if (i0 == i1 || i1 == i2 || i0 == i2)
                continue;
            newIndices.push_back(i0);
            newIndices.push_back(i1);
            newIndices.push_back(i2);
            kept.indexCount += 3;
        }
        if (kept.indexCount > 0)
            newSubmeshes.push_back(kept);
    }
    mesh.indices = std::move(newIndices);
    mesh.submeshes = hadNoSubmeshes ? std::vector<SubMesh>() : std::move(newSubmeshes);

    if (mesh.indices.empty())
    {
        const u32 removed = static_cast<u32>(vertexCount);
        mesh.positions.clear();
        mesh.normals.clear();
        mesh.tangents.clear();
        mesh.uvs.clear();
        mesh.uvs2.clear();
        mesh.colors.clear();
        mesh.skin.clear();
        mesh.submeshes.clear();
        return removed;
    }

    std::vector<u32> remap(vertexCount);
    const usize uniqueCount = meshopt_optimizeVertexFetchRemap(remap.data(), mesh.indices.data(),
                                                                mesh.indices.size(), vertexCount);
    meshopt_remapIndexBuffer(mesh.indices.data(), mesh.indices.data(), mesh.indices.size(),
                             remap.data());
    remapAllVertexArrays(mesh, remap, uniqueCount);
    computeSubMeshBounds(mesh);
    return static_cast<u32>(vertexCount - uniqueCount);
}

void AssetManager::smoothVertices(MeshData& mesh, f32 strength, u32 iterations,
                                  const std::vector<u32>& vertexIndices) const
{
    const usize vertexCount = mesh.positions.size();
    if (vertexCount == 0 || mesh.indices.size() < 3 || strength <= 0.0f || iterations == 0)
        return;

    std::vector<bool> eligible(vertexCount, vertexIndices.empty());
    if (!vertexIndices.empty())
        for (u32 v : vertexIndices)
            if (v < vertexCount)
                eligible[v] = true;

    // Adjacency built once from the (unchanging) topology: every vertex a
    // triangle edge touches becomes a neighbor of the vertex on the other end.
    std::vector<std::vector<u32>> neighbors(vertexCount);
    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const u32 i0 = mesh.indices[i + 0];
        const u32 i1 = mesh.indices[i + 1];
        const u32 i2 = mesh.indices[i + 2];
        if (i0 >= vertexCount || i1 >= vertexCount || i2 >= vertexCount)
            continue;
        neighbors[i0].push_back(i1);
        neighbors[i0].push_back(i2);
        neighbors[i1].push_back(i0);
        neighbors[i1].push_back(i2);
        neighbors[i2].push_back(i0);
        neighbors[i2].push_back(i1);
    }

    std::vector<glm::vec3> next(vertexCount);
    for (u32 pass = 0; pass < iterations; ++pass)
    {
        next = mesh.positions;
        for (u32 v = 0; v < vertexCount; ++v)
        {
            if (!eligible[v] || neighbors[v].empty())
                continue;
            glm::vec3 average(0.0f);
            for (u32 n : neighbors[v])
                average += mesh.positions[n];
            average /= static_cast<f32>(neighbors[v].size());
            next[v] = glm::mix(mesh.positions[v], average, strength);
        }
        mesh.positions = next;
    }

    computeBounds(mesh);
}

void AssetManager::optimizeVertexCache(MeshData& mesh) const
{
    const usize vertexCount = mesh.positions.size();
    if (vertexCount == 0 || mesh.indices.empty())
        return;

    std::vector<u32> scratch;
    if (mesh.submeshes.empty())
    {
        scratch = mesh.indices;
        meshopt_optimizeVertexCache(mesh.indices.data(), scratch.data(), scratch.size(),
                                    vertexCount);
        return;
    }
    for (const SubMesh& submesh : mesh.submeshes)
    {
        if (submesh.indexCount < 3)
            continue;
        scratch.assign(mesh.indices.begin() + submesh.indexOffset,
                       mesh.indices.begin() + submesh.indexOffset + submesh.indexCount);
        meshopt_optimizeVertexCache(mesh.indices.data() + submesh.indexOffset, scratch.data(),
                                    submesh.indexCount, vertexCount);
    }
}

void AssetManager::optimizeOverdraw(MeshData& mesh, f32 threshold) const
{
    const usize vertexCount = mesh.positions.size();
    if (vertexCount == 0 || mesh.indices.empty())
        return;

    const f32* positions = &mesh.positions[0].x;
    std::vector<u32> scratch;
    if (mesh.submeshes.empty())
    {
        scratch = mesh.indices;
        meshopt_optimizeOverdraw(mesh.indices.data(), scratch.data(), scratch.size(), positions,
                                 vertexCount, sizeof(glm::vec3), threshold);
        return;
    }
    for (const SubMesh& submesh : mesh.submeshes)
    {
        if (submesh.indexCount < 3)
            continue;
        scratch.assign(mesh.indices.begin() + submesh.indexOffset,
                       mesh.indices.begin() + submesh.indexOffset + submesh.indexCount);
        meshopt_optimizeOverdraw(mesh.indices.data() + submesh.indexOffset, scratch.data(),
                                 submesh.indexCount, positions, vertexCount, sizeof(glm::vec3),
                                 threshold);
    }
}

void AssetManager::optimizeVertexFetch(MeshData& mesh) const
{
    const usize vertexCount = mesh.positions.size();
    if (vertexCount == 0 || mesh.indices.empty())
        return;

    std::vector<u32> remap(vertexCount);
    const usize uniqueCount = meshopt_optimizeVertexFetchRemap(remap.data(), mesh.indices.data(),
                                                               mesh.indices.size(), vertexCount);
    meshopt_remapIndexBuffer(mesh.indices.data(), mesh.indices.data(), mesh.indices.size(),
                             remap.data());
    remapAllVertexArrays(mesh, remap, uniqueCount);
}

// -------------------------------------------------------------- edit ops

void AssetManager::deleteFaces(MeshData& mesh, const std::vector<u32>& faceIndices) const
{
    if (faceIndices.empty() || mesh.indices.size() < 3)
        return;

    const usize faceCount = mesh.indices.size() / 3;
    std::vector<bool> removed(faceCount, false);
    for (u32 face : faceIndices)
        if (face < faceCount)
            removed[face] = true;

    const bool hadNoSubmeshes = mesh.submeshes.empty();
    std::vector<SubMesh> sourceSubmeshes = mesh.submeshes;
    if (sourceSubmeshes.empty())
    {
        SubMesh whole;
        whole.indexOffset = 0;
        whole.indexCount = static_cast<u32>(mesh.indices.size());
        sourceSubmeshes.push_back(whole);
    }

    std::vector<u32> newIndices;
    newIndices.reserve(mesh.indices.size());
    std::vector<SubMesh> newSubmeshes;
    newSubmeshes.reserve(sourceSubmeshes.size());

    for (const SubMesh& submesh : sourceSubmeshes)
    {
        SubMesh kept = submesh;
        kept.indexOffset = static_cast<u32>(newIndices.size());
        kept.indexCount = 0;
        const u64 end = static_cast<u64>(submesh.indexOffset) + submesh.indexCount;
        for (u32 i = submesh.indexOffset; i + 2 < end && i + 2 < mesh.indices.size(); i += 3)
        {
            const usize face = i / 3;
            if (face < faceCount && removed[face])
                continue;
            newIndices.push_back(mesh.indices[i + 0]);
            newIndices.push_back(mesh.indices[i + 1]);
            newIndices.push_back(mesh.indices[i + 2]);
            kept.indexCount += 3;
        }
        if (kept.indexCount > 0)
            newSubmeshes.push_back(kept);
    }

    mesh.indices = std::move(newIndices);
    mesh.submeshes = hadNoSubmeshes ? std::vector<SubMesh>() : std::move(newSubmeshes);

    if (mesh.indices.empty())
    {
        mesh.positions.clear();
        mesh.normals.clear();
        mesh.tangents.clear();
        mesh.uvs.clear();
        mesh.uvs2.clear();
        mesh.colors.clear();
        mesh.skin.clear();
        mesh.submeshes.clear();
        return;
    }

    const usize vertexCount = mesh.positions.size();
    std::vector<u32> remap(vertexCount);
    const usize uniqueCount = meshopt_optimizeVertexFetchRemap(remap.data(), mesh.indices.data(),
                                                                mesh.indices.size(), vertexCount);
    meshopt_remapIndexBuffer(mesh.indices.data(), mesh.indices.data(), mesh.indices.size(),
                             remap.data());
    remapAllVertexArrays(mesh, remap, uniqueCount);

    computeSubMeshBounds(mesh);
}

void AssetManager::deleteVertices(MeshData& mesh, const std::vector<u32>& vertexIndices) const
{
    if (vertexIndices.empty() || mesh.indices.size() < 3)
        return;

    const usize vertexCount = mesh.positions.size();
    std::vector<bool> removedVertex(vertexCount, false);
    for (u32 v : vertexIndices)
        if (v < vertexCount)
            removedVertex[v] = true;

    const usize faceCount = mesh.indices.size() / 3;
    std::vector<u32> faces;
    faces.reserve(faceCount);
    for (usize face = 0; face < faceCount; ++face)
    {
        const u32 i0 = mesh.indices[face * 3 + 0];
        const u32 i1 = mesh.indices[face * 3 + 1];
        const u32 i2 = mesh.indices[face * 3 + 2];
        if ((i0 < vertexCount && removedVertex[i0]) || (i1 < vertexCount && removedVertex[i1]) ||
            (i2 < vertexCount && removedVertex[i2]))
            faces.push_back(static_cast<u32>(face));
    }

    deleteFaces(mesh, faces);
}

namespace
{
constexpr u32 kNoDuplicate = 0xffffffffu;

u64 edgeKey(u32 a, u32 b)
{
    const u32 low = a < b ? a : b;
    const u32 high = a < b ? b : a;
    return (static_cast<u64>(low) << 32) | static_cast<u64>(high);
}
} // namespace

bool AssetManager::extrudeFaces(MeshData& mesh, const std::vector<u32>& faceIndices, f32 distance,
                                std::vector<u32>* extrudedFaces) const
{
    if (extrudedFaces)
        extrudedFaces->clear();

    const usize faceCount = mesh.indices.size() / 3;
    const usize vertexCount = mesh.positions.size();
    if (faceIndices.empty() || faceCount == 0 || vertexCount == 0)
        return false;

    std::vector<bool> selected(faceCount, false);
    usize selectedCount = 0;
    for (u32 face : faceIndices)
    {
        if (face < faceCount && !selected[face])
        {
            selected[face] = true;
            ++selectedCount;
        }
    }
    if (selectedCount == 0)
        return false;

    std::vector<glm::vec3> offset(vertexCount, glm::vec3(0.0f));
    std::vector<bool> used(vertexCount, false);
    std::unordered_map<u64, u32> edgeUse;
    edgeUse.reserve(selectedCount * 3);

    for (usize face = 0; face < faceCount; ++face)
    {
        if (!selected[face])
            continue;

        const u32 i0 = mesh.indices[face * 3 + 0];
        const u32 i1 = mesh.indices[face * 3 + 1];
        const u32 i2 = mesh.indices[face * 3 + 2];
        if (i0 >= vertexCount || i1 >= vertexCount || i2 >= vertexCount)
        {
            selected[face] = false;
            --selectedCount;
            continue;
        }

        // Not normalized: the cross product's length is twice the triangle's
        // area, so a big face pulls a shared vertex more than a sliver does.
        const glm::vec3 faceNormal = glm::cross(mesh.positions[i1] - mesh.positions[i0],
                                                mesh.positions[i2] - mesh.positions[i0]);
        offset[i0] += faceNormal;
        offset[i1] += faceNormal;
        offset[i2] += faceNormal;
        used[i0] = true;
        used[i1] = true;
        used[i2] = true;

        ++edgeUse[edgeKey(i0, i1)];
        ++edgeUse[edgeKey(i1, i2)];
        ++edgeUse[edgeKey(i2, i0)];
    }
    if (selectedCount == 0)
        return false;

    const bool hasNormals = mesh.normals.size() == vertexCount;
    const bool hasTangents = mesh.tangents.size() == vertexCount;
    const bool hasUvs = mesh.uvs.size() == vertexCount;
    const bool hasUvs2 = mesh.uvs2.size() == vertexCount;
    const bool hasColors = mesh.colors.size() == vertexCount;
    const bool hasSkin = mesh.skin.size() == vertexCount;

    std::vector<u32> duplicate(vertexCount, kNoDuplicate);
    for (u32 v = 0; v < static_cast<u32>(vertexCount); ++v)
    {
        if (!used[v])
            continue;

        duplicate[v] = static_cast<u32>(mesh.positions.size());

        const f32 length = glm::length(offset[v]);
        const glm::vec3 direction = length > 1e-8f ? offset[v] / length : glm::vec3(0.0f);
        const glm::vec3 raised = mesh.positions[v] + direction * distance;

        mesh.positions.push_back(raised);
        if (hasNormals)
            mesh.normals.push_back(mesh.normals[v]);
        if (hasTangents)
            mesh.tangents.push_back(mesh.tangents[v]);
        if (hasUvs)
            mesh.uvs.push_back(mesh.uvs[v]);
        if (hasUvs2)
            mesh.uvs2.push_back(mesh.uvs2[v]);
        if (hasColors)
            mesh.colors.push_back(mesh.colors[v]);
        if (hasSkin)
            mesh.skin.push_back(mesh.skin[v]);
    }

    std::vector<SubMesh> sourceSubmeshes = mesh.submeshes;
    if (sourceSubmeshes.empty())
    {
        SubMesh whole;
        whole.indexOffset = 0;
        whole.indexCount = static_cast<u32>(mesh.indices.size());
        sourceSubmeshes.push_back(whole);
    }

    std::vector<u32> newIndices;
    newIndices.reserve(mesh.indices.size() + selectedCount * 18);
    std::vector<SubMesh> newSubmeshes;
    newSubmeshes.reserve(sourceSubmeshes.size());

    for (const SubMesh& submesh : sourceSubmeshes)
    {
        SubMesh rebuilt = submesh;
        rebuilt.indexOffset = static_cast<u32>(newIndices.size());

        const u64 end = static_cast<u64>(submesh.indexOffset) + submesh.indexCount;
        for (u32 i = submesh.indexOffset; i + 2 < end && i + 2 < mesh.indices.size(); i += 3)
        {
            const usize face = i / 3;
            const u32 i0 = mesh.indices[i];
            const u32 i1 = mesh.indices[i + 1];
            const u32 i2 = mesh.indices[i + 2];

            if (face >= faceCount || !selected[face])
            {
                newIndices.push_back(i0);
                newIndices.push_back(i1);
                newIndices.push_back(i2);
                continue;
            }

            if (extrudedFaces)
                extrudedFaces->push_back(static_cast<u32>(newIndices.size() / 3));
            newIndices.push_back(duplicate[i0]);
            newIndices.push_back(duplicate[i1]);
            newIndices.push_back(duplicate[i2]);

            const u32 edges[3][2] = {{i0, i1}, {i1, i2}, {i2, i0}};
            for (u32 e = 0; e < 3; ++e)
            {
                const u32 a = edges[e][0];
                const u32 b = edges[e][1];
                if (edgeUse[edgeKey(a, b)] != 1)
                    continue;

                // Walking the boundary in the face's own winding keeps the
                // region on the left, so this pair faces outward.
                newIndices.push_back(a);
                newIndices.push_back(b);
                newIndices.push_back(duplicate[b]);

                newIndices.push_back(a);
                newIndices.push_back(duplicate[b]);
                newIndices.push_back(duplicate[a]);
            }
        }

        rebuilt.indexCount = static_cast<u32>(newIndices.size()) - rebuilt.indexOffset;
        newSubmeshes.push_back(rebuilt);
    }

    mesh.indices = std::move(newIndices);
    mesh.submeshes = std::move(newSubmeshes);

    computeBounds(mesh);
    computeSubMeshBounds(mesh);
    return true;
}

namespace
{
// Marks the given indices in a bitvector sized to `count`, ignoring anything
// past the end - a selection can outlive the mesh it was made against.
void markSelected(const std::vector<u32>& indices, usize count, std::vector<bool>& marked)
{
    marked.assign(count, false);
    for (u32 index : indices)
        if (index < count)
            marked[index] = true;
}

void collectMarked(const std::vector<bool>& marked, std::vector<u32>& out)
{
    out.clear();
    for (u32 i = 0; i < static_cast<u32>(marked.size()); ++i)
        if (marked[i])
            out.push_back(i);
}
} // namespace

bool AssetManager::transformFaceUVs(MeshData& mesh, const std::vector<u32>& faceIndices,
                                    const glm::vec2& scale, f32 rotationDegrees,
                                    const glm::vec2& offset) const
{
    const usize vertexCount = mesh.positions.size();
    const usize faceCount = mesh.indices.size() / 3;
    if (vertexCount == 0 || faceCount == 0 || mesh.uvs.size() != vertexCount)
        return false;

    const bool wholeMesh = faceIndices.empty();
    std::vector<bool> selected(faceCount, wholeMesh);
    if (!wholeMesh)
    {
        usize selectedCount = 0;
        for (u32 face : faceIndices)
        {
            if (face < faceCount && !selected[face])
            {
                selected[face] = true;
                ++selectedCount;
            }
        }
        if (selectedCount == 0)
            return false;
    }

    std::vector<bool> usedBySelected(vertexCount, false);
    std::vector<bool> usedByRest(vertexCount, false);
    for (usize face = 0; face < faceCount; ++face)
    {
        std::vector<bool>& mark = selected[face] ? usedBySelected : usedByRest;
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 index = mesh.indices[face * 3 + corner];
            if (index < vertexCount)
                mark[index] = true;
        }
    }

    const bool hasNormals = mesh.normals.size() == vertexCount;
    const bool hasTangents = mesh.tangents.size() == vertexCount;
    const bool hasUvs2 = mesh.uvs2.size() == vertexCount;
    const bool hasColors = mesh.colors.size() == vertexCount;
    const bool hasSkin = mesh.skin.size() == vertexCount;

    std::vector<u32> duplicate(vertexCount, kNoDuplicate);
    for (u32 v = 0; v < static_cast<u32>(vertexCount); ++v)
    {
        // Only the ones straddling the edge of the selection: a vertex the
        // rest of the mesh never touches can simply be moved.
        if (!usedBySelected[v] || !usedByRest[v])
            continue;

        duplicate[v] = static_cast<u32>(mesh.positions.size());
        mesh.positions.push_back(mesh.positions[v]);
        mesh.uvs.push_back(mesh.uvs[v]);
        if (hasNormals)
            mesh.normals.push_back(mesh.normals[v]);
        if (hasTangents)
            mesh.tangents.push_back(mesh.tangents[v]);
        if (hasUvs2)
            mesh.uvs2.push_back(mesh.uvs2[v]);
        if (hasColors)
            mesh.colors.push_back(mesh.colors[v]);
        if (hasSkin)
            mesh.skin.push_back(mesh.skin[v]);
    }

    for (usize face = 0; face < faceCount; ++face)
    {
        if (!selected[face])
            continue;
        for (u32 corner = 0; corner < 3; ++corner)
        {
            u32& index = mesh.indices[face * 3 + corner];
            if (index < vertexCount && duplicate[index] != kNoDuplicate)
                index = duplicate[index];
        }
    }

    std::vector<bool> affected(mesh.positions.size(), false);
    for (usize face = 0; face < faceCount; ++face)
    {
        if (!selected[face])
            continue;
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 index = mesh.indices[face * 3 + corner];
            if (index < affected.size())
                affected[index] = true;
        }
    }

    glm::vec2 minUV(std::numeric_limits<f32>::max());
    glm::vec2 maxUV(-std::numeric_limits<f32>::max());
    bool any = false;
    for (usize v = 0; v < affected.size(); ++v)
    {
        if (!affected[v])
            continue;
        minUV = glm::min(minUV, mesh.uvs[v]);
        maxUV = glm::max(maxUV, mesh.uvs[v]);
        any = true;
    }
    if (!any)
        return false;

    const glm::vec2 center = (minUV + maxUV) * 0.5f;
    const f32 radians = glm::radians(rotationDegrees);
    const f32 cosine = glm::cos(radians);
    const f32 sine = glm::sin(radians);

    for (usize v = 0; v < affected.size(); ++v)
    {
        if (!affected[v])
            continue;

        const glm::vec2 local = (mesh.uvs[v] - center) * scale;
        mesh.uvs[v] = center + glm::vec2(local.x * cosine - local.y * sine,
                                         local.x * sine + local.y * cosine) +
                      offset;
    }

    // Tangents are built from the UVs, so leaving them alone after a rotation
    // or a mirror leaves every normal-mapped surface here lit for the old
    // layout. Only worth doing if the mesh had them to begin with.
    if (hasTangents)
        recalculateTangents(mesh);

    return true;
}

void AssetManager::analyzeMesh(const MeshData& mesh, Diagnostics& out) const
{
    out = Diagnostics();

    const usize vertexCount = mesh.positions.size();
    out.vertexCount = vertexCount;
    out.triangleCount = mesh.indices.size() / 3;
    out.submeshCount = mesh.submeshes.size();
    out.materialCount = mesh.materials.size();
    out.memoryBytes = mesh.memoryBytes();
    out.bounds = mesh.bounds;

    out.hasNormals = !mesh.normals.empty();
    out.hasTangents = !mesh.tangents.empty();
    out.hasUvs = !mesh.uvs.empty();
    out.hasUvs2 = !mesh.uvs2.empty();
    out.hasColors = !mesh.colors.empty();
    out.hasSkin = !mesh.skin.empty();

    out.streamsMismatched =
        (out.hasNormals && mesh.normals.size() != vertexCount) ||
        (out.hasTangents && mesh.tangents.size() != vertexCount) ||
        (out.hasUvs && mesh.uvs.size() != vertexCount) ||
        (out.hasUvs2 && mesh.uvs2.size() != vertexCount) ||
        (out.hasColors && mesh.colors.size() != vertexCount) ||
        (out.hasSkin && mesh.skin.size() != vertexCount);

    out.trianglesTruncated = (mesh.indices.size() % 3) != 0;

    for (usize i = 0; i < mesh.submeshes.size(); ++i)
    {
        const SubMesh& submesh = mesh.submeshes[i];
        const u64 end = static_cast<u64>(submesh.indexOffset) + submesh.indexCount;
        if (end > mesh.indices.size() || (submesh.indexCount % 3) != 0)
        {
            out.submeshRangesInvalid = true;
            break;
        }
    }

    if (vertexCount == 0)
        return;

    std::vector<bool> referenced(vertexCount, false);
    std::unordered_map<u64, u32> edgeUse;
    edgeUse.reserve(out.triangleCount * 3);

    for (usize face = 0; face < out.triangleCount; ++face)
    {
        const u32 i0 = mesh.indices[face * 3 + 0];
        const u32 i1 = mesh.indices[face * 3 + 1];
        const u32 i2 = mesh.indices[face * 3 + 2];

        if (i0 >= vertexCount || i1 >= vertexCount || i2 >= vertexCount)
        {
            out.outOfRangeIndices += (i0 >= vertexCount) + (i1 >= vertexCount) + (i2 >= vertexCount);
            continue;
        }

        referenced[i0] = true;
        referenced[i1] = true;
        referenced[i2] = true;

        if (i0 == i1 || i1 == i2 || i2 == i0)
        {
            ++out.degenerateTriangles;
            continue;
        }

        // Twice the area. Comparing that against a small epsilon rather than
        // exactly zero catches the slivers too - three points on a line have
        // no normal to give, and a zero normal averaged into its vertices
        // takes the shading of everything around it with it.
        const glm::vec3 cross = glm::cross(mesh.positions[i1] - mesh.positions[i0],
                                           mesh.positions[i2] - mesh.positions[i0]);
        if (glm::length(cross) <= 1e-12f)
        {
            ++out.degenerateTriangles;
            continue;
        }

        ++edgeUse[edgeKey(i0, i1)];
        ++edgeUse[edgeKey(i1, i2)];
        ++edgeUse[edgeKey(i2, i0)];
    }

    for (usize v = 0; v < vertexCount; ++v)
        if (!referenced[v])
            ++out.orphanVertices;

    for (std::unordered_map<u64, u32>::const_iterator it = edgeUse.begin(); it != edgeUse.end();
         ++it)
    {
        if (it->second == 1)
            ++out.boundaryEdges;
        else if (it->second > 2)
            ++out.nonManifoldEdges;
    }

    std::unordered_map<u64, u32> positionUse;
    positionUse.reserve(vertexCount);
    for (usize v = 0; v < vertexCount; ++v)
    {
        const glm::vec3& p = mesh.positions[v];
        u32 bits[3];
        std::memcpy(bits, &p, sizeof(bits));
        // FNV-1a over the three float bit patterns: exact matches only, which
        // is what "the same position" has to mean without a tolerance.
        u64 hash = 14695981039346656037ull;
        for (u32 i = 0; i < 3; ++i)
        {
            hash ^= bits[i];
            hash *= 1099511628211ull;
        }
        if (++positionUse[hash] > 1)
            ++out.exactDuplicatePositions;
    }
}

void AssetManager::growVertexSelection(const MeshData& mesh, const std::vector<u32>& vertexIndices,
                                       std::vector<u32>& out) const
{
    const usize vertexCount = mesh.positions.size();
    std::vector<bool> selected;
    markSelected(vertexIndices, vertexCount, selected);

    // Read from `selected` and write to `grown`, never both: growing in place
    // would let a vertex added by one triangle seed the next one in the same
    // pass, spreading further than the single ring asked for.
    std::vector<bool> grown = selected;
    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const u32 corner[3] = {mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2]};
        if (corner[0] >= vertexCount || corner[1] >= vertexCount || corner[2] >= vertexCount)
            continue;

        for (u32 e = 0; e < 3; ++e)
        {
            const u32 a = corner[e];
            const u32 b = corner[(e + 1) % 3];
            if (selected[a])
                grown[b] = true;
            if (selected[b])
                grown[a] = true;
        }
    }

    collectMarked(grown, out);
}

void AssetManager::shrinkVertexSelection(const MeshData& mesh,
                                         const std::vector<u32>& vertexIndices,
                                         std::vector<u32>& out) const
{
    const usize vertexCount = mesh.positions.size();
    std::vector<bool> selected;
    markSelected(vertexIndices, vertexCount, selected);

    std::vector<bool> kept = selected;
    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const u32 corner[3] = {mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2]};
        if (corner[0] >= vertexCount || corner[1] >= vertexCount || corner[2] >= vertexCount)
            continue;

        for (u32 e = 0; e < 3; ++e)
        {
            const u32 a = corner[e];
            const u32 b = corner[(e + 1) % 3];
            // A vertex with an unselected neighbour is on the border.
            if (!selected[b])
                kept[a] = false;
            if (!selected[a])
                kept[b] = false;
        }
    }

    collectMarked(kept, out);
}

void AssetManager::growFaceSelection(const MeshData& mesh, const std::vector<u32>& faceIndices,
                                     std::vector<u32>& out) const
{
    const usize vertexCount = mesh.positions.size();
    const usize faceCount = mesh.indices.size() / 3;
    std::vector<bool> selected;
    markSelected(faceIndices, faceCount, selected);

    std::vector<bool> touched(vertexCount, false);
    for (usize face = 0; face < faceCount; ++face)
    {
        if (!selected[face])
            continue;
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 index = mesh.indices[face * 3 + corner];
            if (index < vertexCount)
                touched[index] = true;
        }
    }

    std::vector<bool> grown = selected;
    for (usize face = 0; face < faceCount; ++face)
    {
        if (grown[face])
            continue;
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 index = mesh.indices[face * 3 + corner];
            if (index < vertexCount && touched[index])
            {
                grown[face] = true;
                break;
            }
        }
    }

    collectMarked(grown, out);
}

void AssetManager::selectLinkedVertices(const MeshData& mesh,
                                        const std::vector<u32>& seedVertices,
                                        std::vector<u32>& out) const
{
    const usize vertexCount = mesh.positions.size();
    const usize faceCount = mesh.indices.size() / 3;

    std::vector<bool> reached;
    markSelected(seedVertices, vertexCount, reached);

    // Sweeping the triangle list until a pass adds nothing: a component
    // spanning the mesh needs as many passes as it is long in triangles, but
    // no adjacency structure has to be built and thrown away for one query.
    bool changed = true;
    while (changed)
    {
        changed = false;
        for (usize face = 0; face < faceCount; ++face)
        {
            const u32 i0 = mesh.indices[face * 3 + 0];
            const u32 i1 = mesh.indices[face * 3 + 1];
            const u32 i2 = mesh.indices[face * 3 + 2];
            if (i0 >= vertexCount || i1 >= vertexCount || i2 >= vertexCount)
                continue;

            if (!reached[i0] && !reached[i1] && !reached[i2])
                continue;
            if (reached[i0] && reached[i1] && reached[i2])
                continue;

            reached[i0] = true;
            reached[i1] = true;
            reached[i2] = true;
            changed = true;
        }
    }

    collectMarked(reached, out);
}

void AssetManager::selectLinkedFaces(const MeshData& mesh, const std::vector<u32>& seedFaces,
                                     std::vector<u32>& out) const
{
    const usize vertexCount = mesh.positions.size();
    const usize faceCount = mesh.indices.size() / 3;

    std::vector<u32> seedVertices;
    seedVertices.reserve(seedFaces.size() * 3);
    for (u32 face : seedFaces)
    {
        if (face >= faceCount)
            continue;
        for (u32 corner = 0; corner < 3; ++corner)
            seedVertices.push_back(mesh.indices[face * 3 + corner]);
    }

    std::vector<u32> linked;
    selectLinkedVertices(mesh, seedVertices, linked);

    std::vector<bool> reached;
    markSelected(linked, vertexCount, reached);

    out.clear();
    for (usize face = 0; face < faceCount; ++face)
    {
        const u32 i0 = mesh.indices[face * 3 + 0];
        if (i0 < vertexCount && reached[i0])
            out.push_back(static_cast<u32>(face));
    }
}

void AssetManager::submeshFaces(const MeshData& mesh, u32 submeshIndex,
                                std::vector<u32>& out) const
{
    out.clear();
    if (submeshIndex >= mesh.submeshes.size())
        return;

    const SubMesh& submesh = mesh.submeshes[submeshIndex];
    const u64 end = static_cast<u64>(submesh.indexOffset) + submesh.indexCount;
    for (u64 i = submesh.indexOffset; i + 2 < end && i + 2 < mesh.indices.size(); i += 3)
        out.push_back(static_cast<u32>(i / 3));
}

bool AssetManager::groupFacesIntoSubmesh(MeshData& mesh, const std::vector<u32>& faceIndices) const
{
    if (faceIndices.empty() || mesh.indices.size() < 3)
        return false;

    const usize faceCount = mesh.indices.size() / 3;
    std::vector<bool> grouped(faceCount, false);
    usize groupedCount = 0;
    for (u32 face : faceIndices)
    {
        if (face < faceCount && !grouped[face])
        {
            grouped[face] = true;
            ++groupedCount;
        }
    }
    if (groupedCount == 0)
        return false;

    std::vector<SubMesh> sourceSubmeshes = mesh.submeshes;
    if (sourceSubmeshes.empty())
    {
        SubMesh whole;
        whole.indexOffset = 0;
        whole.indexCount = static_cast<u32>(mesh.indices.size());
        sourceSubmeshes.push_back(whole);
    }

    std::vector<u32> newIndices;
    newIndices.reserve(mesh.indices.size());
    std::vector<SubMesh> newSubmeshes;
    newSubmeshes.reserve(sourceSubmeshes.size() + 1);

    std::vector<u32> groupedIndices;
    groupedIndices.reserve(groupedCount * 3);
    u32 groupedMaterialSlot = 0;
    u32 groupedLightmapPage = 0;
    bool groupedMaterialSet = false;

    for (const SubMesh& submesh : sourceSubmeshes)
    {
        SubMesh kept = submesh;
        kept.indexOffset = static_cast<u32>(newIndices.size());
        kept.indexCount = 0;
        const u64 end = static_cast<u64>(submesh.indexOffset) + submesh.indexCount;
        for (u32 i = submesh.indexOffset; i + 2 < end && i + 2 < mesh.indices.size(); i += 3)
        {
            const usize face = i / 3;
            if (face < faceCount && grouped[face])
            {
                if (!groupedMaterialSet)
                {
                    groupedMaterialSlot = submesh.materialSlot;
                    groupedLightmapPage = submesh.lightmapPage;
                    groupedMaterialSet = true;
                }
                groupedIndices.push_back(mesh.indices[i + 0]);
                groupedIndices.push_back(mesh.indices[i + 1]);
                groupedIndices.push_back(mesh.indices[i + 2]);
                continue;
            }
            newIndices.push_back(mesh.indices[i + 0]);
            newIndices.push_back(mesh.indices[i + 1]);
            newIndices.push_back(mesh.indices[i + 2]);
            kept.indexCount += 3;
        }
        if (kept.indexCount > 0)
            newSubmeshes.push_back(kept);
    }

    SubMesh newGroup;
    newGroup.indexOffset = static_cast<u32>(newIndices.size());
    newGroup.indexCount = static_cast<u32>(groupedIndices.size());
    newGroup.materialSlot = groupedMaterialSlot;
    newGroup.lightmapPage = groupedLightmapPage;
    newIndices.insert(newIndices.end(), groupedIndices.begin(), groupedIndices.end());
    newSubmeshes.push_back(newGroup);

    mesh.indices = std::move(newIndices);
    mesh.submeshes = std::move(newSubmeshes);
    computeSubMeshBounds(mesh);
    return true;
}

bool AssetManager::simplifyMesh(MeshData& mesh, f32 targetRatio, f32 targetError,
                                f32* resultError) const
{
    if (mesh.positions.empty() || mesh.indices.empty())
        return false;

    targetRatio = glm::clamp(targetRatio, 0.0f, 1.0f);
    const f32* positions = &mesh.positions[0].x;
    const usize vertexCount = mesh.positions.size();

    // Borders between submeshes have to stay put when there is more than one
    // range, otherwise each range pulls its shared edge its own way and the
    // seams crack open.
    const u32 options = mesh.submeshes.size() > 1 ? meshopt_SimplifyLockBorder : 0;

    f32 worstError = 0.0f;
    if (mesh.submeshes.empty())
    {
        std::vector<u32> result(mesh.indices.size());
        const usize target = static_cast<usize>(mesh.indices.size() * targetRatio) / 3 * 3;
        f32 error = 0.0f;
        const usize newCount =
            meshopt_simplify(result.data(), mesh.indices.data(), mesh.indices.size(), positions,
                             vertexCount, sizeof(glm::vec3), target, targetError, options, &error);
        result.resize(newCount);
        mesh.indices = std::move(result);
        worstError = error;
    }
    else
    {
        std::vector<u32> newIndices;
        newIndices.reserve(mesh.indices.size());
        std::vector<u32> scratch;
        for (SubMesh& submesh : mesh.submeshes)
        {
            if (submesh.indexCount < 3)
            {
                submesh.indexOffset = static_cast<u32>(newIndices.size());
                submesh.indexCount = 0;
                continue;
            }
            scratch.resize(submesh.indexCount);
            const usize target =
                static_cast<usize>(submesh.indexCount * targetRatio) / 3 * 3;
            f32 error = 0.0f;
            const usize newCount = meshopt_simplify(scratch.data(),
                                                    mesh.indices.data() + submesh.indexOffset,
                                                    submesh.indexCount, positions, vertexCount,
                                                    sizeof(glm::vec3), target, targetError,
                                                    options, &error);
            submesh.indexOffset = static_cast<u32>(newIndices.size());
            submesh.indexCount = static_cast<u32>(newCount);
            newIndices.insert(newIndices.end(), scratch.begin(), scratch.begin() + newCount);
            worstError = glm::max(worstError, error);
        }
        mesh.indices = std::move(newIndices);
        computeSubMeshBounds(mesh);
    }

    if (resultError)
        *resultError = worstError;
    return true;
}

// ---------------------------------------------------------------- collision

void AssetManager::buildCollisionMesh(const MeshData& mesh, CollisionMesh& out) const
{
    out.positions = mesh.positions;
    out.indices = mesh.indices;

    out.bounds = AABB();
    for (usize i = 0; i < out.positions.size(); ++i)
        out.bounds.expand(out.positions[i]);
}

bool AssetManager::raycast(const CollisionMesh& mesh, const Ray& ray, f32& t, u32& triangle) const
{
    // The box rejects the whole mesh in one test before touching triangles.
    f32 boxHit = 0.0f;
    if (!ray.intersects(mesh.bounds, boxHit))
        return false;

    bool hit = false;
    f32 nearest = 3.402823466e+38F;

    for (usize i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const u32 i0 = mesh.indices[i + 0];
        const u32 i1 = mesh.indices[i + 1];
        const u32 i2 = mesh.indices[i + 2];
        if (i0 >= mesh.positions.size() || i1 >= mesh.positions.size() ||
            i2 >= mesh.positions.size())
            continue;

        f32 current = 0.0f;
        if (!ray.intersects(mesh.positions[i0], mesh.positions[i1], mesh.positions[i2], current))
            continue;

        if (current < nearest)
        {
            nearest = current;
            triangle = static_cast<u32>(i / 3);
            hit = true;
        }
    }

    if (hit)
        t = nearest;
    return hit;
}

// ------------------------------------------------------------ primitives

// Thin wrappers over createMesh(MeshDesc): the description carries the
// recipe, so a mesh built through any of these already knows what it is and
// nothing here has to record anything afterwards.

MeshHandle AssetManager::createBox(const glm::vec3& size)
{
    return createMesh(MeshDesc::box(size));
}

MeshHandle AssetManager::createPlane(f32 width, f32 depth, u32 segX, u32 segZ, f32 uvTiles)
{
    return createMesh(MeshDesc::plane(width, depth, segX, segZ, uvTiles));
}

MeshHandle AssetManager::createSphere(f32 radius, u32 rings, u32 slices)
{
    return createMesh(MeshDesc::sphere(radius, rings, slices));
}

MeshHandle AssetManager::createCylinder(f32 radius, f32 height, u32 slices)
{
    return createMesh(MeshDesc::cylinder(radius, height, slices));
}

MeshHandle AssetManager::createCone(f32 radius, f32 height, u32 slices)
{
    return createMesh(MeshDesc::cone(radius, height, slices));
}

MeshHandle AssetManager::createCapsule(f32 radius, f32 height, u32 rings, u32 slices)
{
    return createMesh(MeshDesc::capsule(radius, height, rings, slices));
}

MeshHandle AssetManager::createTorus(f32 majorRadius, f32 minorRadius, u32 majorSegments,
                                     u32 minorSegments)
{
    return createMesh(MeshDesc::torus(majorRadius, minorRadius, majorSegments, minorSegments));
}

MeshHandle AssetManager::createHillsPlane(f32 width, f32 depth, u32 segX, u32 segZ,
                                          const std::string& heightmapFile, f32 heightScale,
                                          f32 uvTiles)
{
    return createMesh(
        MeshDesc::hillsPlane(width, depth, segX, segZ, heightmapFile, heightScale, uvTiles));
}

MeshHandle AssetManager::createHeightfield(const std::string& heightmapFile, f32 cellSize,
                                           f32 heightScale, f32 uvTiles)
{
    return createMesh(MeshDesc::heightfield(heightmapFile, cellSize, heightScale, uvTiles));
}

// The two taking an already-loaded image cannot go through a description - a
// Pixmap in memory has no name to write down - so they build and upload
// directly, and no saved scene can refer to the result.

MeshHandle AssetManager::createHillsPlane(f32 width, f32 depth, u32 segX, u32 segZ,
                                          const Pixmap& heightmap, f32 heightScale, f32 uvTiles)
{
    if (!heightmap.is_valid() || heightmap.width < 1 || heightmap.height < 1)
    {
        Log::error("AssetManager: createHillsPlane() got an empty heightmap");
        return MeshHandle();
    }
    MeshData data;
    buildHillsPlane(data, width, depth, segX, segZ, heightmap, heightScale, uvTiles);
    computeBounds(data);
    return createMesh(data);
}

MeshHandle AssetManager::createHeightfield(const Pixmap& heightmap, f32 cellSize, f32 heightScale,
                                           f32 uvTiles)
{
    if (!heightmap.is_valid() || heightmap.width < 2 || heightmap.height < 2)
    {
        Log::error("AssetManager: createHeightfield() needs a heightmap of at least 2x2");
        return MeshHandle();
    }
    MeshData data;
    buildHeightfieldFromPixmap(data, heightmap, cellSize, heightScale, uvTiles);
    computeBounds(data);
    return createMesh(data);
}

// ------------------------------------------------------------------- upload

bool AssetManager::upload(const MeshData& data, Mesh& out, Residency residency) const
{
    GPU& gpu = GPU::getSingleton();

    if (data.positions.empty() || data.indices.empty())
    {
        Log::error("AssetManager: nothing to upload");
        return false;
    }

    std::string error;
    if (!validateMeshData(data, error))
    {
        Log::error("AssetManager: rejected mesh data - %s", error.c_str());
        return false;
    }

    const usize count = data.positions.size();

    BufferDesc positionDesc;
    positionDesc.size = count * sizeof(glm::vec3);
    positionDesc.usage = BufferVertex | BufferStorage;
    positionDesc.residency = residency;
    positionDesc.stride = sizeof(glm::vec3);
    positionDesc.data = data.positions.data();
    positionDesc.debugName = "mesh.positions";
    out.positionBuffer = gpu.createBuffer(positionDesc);

    std::vector<MeshAttribs> attribs(count);
    for (usize i = 0; i < count; ++i)
    {
        MeshAttribs& attrib = attribs[i];
        attrib.normal = i < data.normals.size() ? data.normals[i] : glm::vec3(0.0f, 1.0f, 0.0f);
        attrib.tangent =
            i < data.tangents.size() ? data.tangents[i] : glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
        attrib.uv = i < data.uvs.size() ? data.uvs[i] : glm::vec2(0.0f);
        attrib.uv2 = i < data.uvs2.size() ? data.uvs2[i] : glm::vec2(0.0f);
        attrib.color = i < data.colors.size() ? data.colors[i] : 0xFFFFFFFFu;
    }

    BufferDesc attribDesc;
    attribDesc.size = attribs.size() * sizeof(MeshAttribs);
    attribDesc.usage = BufferVertex | BufferStorage;
    attribDesc.residency = residency;
    attribDesc.stride = sizeof(MeshAttribs);
    attribDesc.data = attribs.data();
    attribDesc.debugName = "mesh.attribs";
    out.attribBuffer = gpu.createBuffer(attribDesc);

    const bool skinned = data.skin.size() == count;
    if (!data.skin.empty() && !skinned)
    {
        Log::error("AssetManager: skin vertex count does not match positions");
        release(out);
        return false;
    }
    if (skinned)
    {
        BufferDesc skinDesc;
        skinDesc.size = data.skin.size() * sizeof(MeshSkinVertex);
        skinDesc.usage = BufferVertex | BufferStorage;
        skinDesc.residency = residency;
        skinDesc.stride = sizeof(MeshSkinVertex);
        skinDesc.data = data.skin.data();
        skinDesc.debugName = "mesh.skin";
        out.skinBuffer = gpu.createBuffer(skinDesc);
    }

    BufferDesc indexDesc;
    indexDesc.size = data.indices.size() * sizeof(u32);
    indexDesc.usage = BufferIndex;
    indexDesc.residency = residency;
    indexDesc.data = data.indices.data();
    indexDesc.debugName = "mesh.indices";
    out.indexBuffer = gpu.createBuffer(indexDesc);

    out.indexType = IndexType::U32;
    out.vertexCount = static_cast<u32>(count);
    out.indexCount = static_cast<u32>(data.indices.size());
    out.submeshes = data.submeshes;
    out.materials = data.materials;
    out.bounds = data.bounds;

    out.depthLayout = VertexLayout();
    out.depthLayout.streamCount = 1;
    out.depthLayout.streams[StreamPosition].stride = sizeof(glm::vec3);
    out.depthLayout.attribCount = 1;
    out.depthLayout.attribs[0] = {0, StreamPosition, 0, AttribFormat::Float3};

    out.colorLayout = out.depthLayout;
    out.colorLayout.streamCount = 2;
    out.colorLayout.streams[StreamAttribs].stride = sizeof(MeshAttribs);
    out.colorLayout.attribCount = 6;
    out.colorLayout.attribs[1] = {1, StreamAttribs, offsetof(MeshAttribs, normal),
                                  AttribFormat::Float3};
    out.colorLayout.attribs[2] = {2, StreamAttribs, offsetof(MeshAttribs, tangent),
                                  AttribFormat::Float4};
    out.colorLayout.attribs[3] = {3, StreamAttribs, offsetof(MeshAttribs, uv),
                                  AttribFormat::Float2};
    out.colorLayout.attribs[4] = {4, StreamAttribs, offsetof(MeshAttribs, color),
                                  AttribFormat::UByte4N};
    // Location 7, not 5: skinning claims 5/6 for joints/weights below, and
    // uv2 has to sit at a location that does not move between a skinned and
    // an unskinned mesh, or lit.vert would need two different layouts to
    // read the same attribute.
    out.colorLayout.attribs[5] = {7, StreamAttribs, offsetof(MeshAttribs, uv2),
                                  AttribFormat::Float2};
    if (skinned)
    {
        out.colorLayout.streamCount = 3;
        out.colorLayout.streams[StreamSkin].stride = sizeof(MeshSkinVertex);
        out.colorLayout.attribCount = 8;
        out.colorLayout.attribs[6] = {5, StreamSkin, offsetof(MeshSkinVertex, joints),
                                      AttribFormat::UByte4};
        out.colorLayout.attribs[7] = {6, StreamSkin, offsetof(MeshSkinVertex, weights),
                                      AttribFormat::Float4};
    }

    if (out.submeshes.empty())
    {
        SubMesh submesh;
        submesh.indexOffset = 0;
        submesh.indexCount = out.indexCount;
        submesh.bounds = out.bounds;
        out.submeshes.push_back(submesh);
    }

    if (out.materials.empty())
    {
        Material material;
        material.flags |= MaterialLit;
        material.params.baseColor = glm::vec4(0.7f, 0.7f, 0.7f, 1.0f);
        material.params.surface.x = 0.7f;
        material.params.surface.y = 0.0f;
        material.paramsDirty = true;
        out.materials.push_back(material);
    }

    return out.positionBuffer.valid() && out.attribBuffer.valid() && out.indexBuffer.valid() &&
           (!skinned || out.skinBuffer.valid());
}

} // namespace Radion
