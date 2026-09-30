#include "PCH.h"
#include "mesh/MeshBoolean.h"

#include "VolumeCSG.h"
#include "VolumeMeshSource.h"
#include "VolumeMesher.h"

using namespace Radion;

namespace
{
bool fail(std::string* error, const std::string& message)
{
    if (error)
        *error = message;
    return false;
}
} // namespace

namespace
{
// Mesher winding and normals are unreliable: fix winding per shell against the field, then rebuild shading with creases.
void orientAndShade(MeshData& mesh, const Volume::Source& field, f32 voxel)
{
    const f32 probe = voxel * 0.75f;
    const usize faceCount = mesh.indices.size() / 3;
    auto key = [](u32 a, u32 b) { return a < b ? (static_cast<u64>(a) << 32) | b : (static_cast<u64>(b) << 32) | a; };

    std::unordered_map<u64, std::vector<u32>> byEdge;
    byEdge.reserve(faceCount * 2);
    for (u32 f = 0; f < faceCount; ++f)
        for (u32 c = 0; c < 3; ++c)
            byEdge[key(mesh.indices[f * 3 + c], mesh.indices[f * 3 + (c + 1) % 3])].push_back(f);

    auto walks = [&](u32 f, u32 from, u32 to)
    {
        for (u32 c = 0; c < 3; ++c)
            if (mesh.indices[f * 3 + c] == from && mesh.indices[f * 3 + (c + 1) % 3] == to)
                return true;
        return false;
    };

    std::vector<s32> shell(faceCount, -1);
    std::vector<std::vector<u32>> shells;
    for (u32 seed = 0; seed < faceCount; ++seed)
    {
        if (shell[seed] >= 0)
            continue;
        const s32 id = static_cast<s32>(shells.size());
        shells.emplace_back();
        std::vector<u32> stack = {seed};
        shell[seed] = id;
        while (!stack.empty())
        {
            const u32 f = stack.back();
            stack.pop_back();
            shells[static_cast<usize>(id)].push_back(f);
            for (u32 c = 0; c < 3; ++c)
            {
                const u32 from = mesh.indices[f * 3 + c];
                const u32 to = mesh.indices[f * 3 + (c + 1) % 3];
                const std::vector<u32>& sharing = byEdge[key(from, to)];
                if (sharing.size() != 2)
                    continue;
                const u32 other = sharing[0] == f ? sharing[1] : sharing[0];
                if (shell[other] >= 0)
                    continue;
                if (walks(other, from, to))
                    std::swap(mesh.indices[other * 3 + 1], mesh.indices[other * 3 + 2]);
                shell[other] = id;
                stack.push_back(other);
            }
        }
    }

    // A shell of a few cells is field noise (inside/outside test tipped at an input's edge); drop it.
    std::vector<u8> dropped(faceCount, 0);
    for (const std::vector<u32>& members : shells)
    {
        Math::vec3 low(1.0e30f);
        Math::vec3 high(-1.0e30f);
        for (const u32 f : members)
            for (u32 c = 0; c < 3; ++c)
            {
                low = Math::min(low, mesh.positions[mesh.indices[f * 3 + c]]);
                high = Math::max(high, mesh.positions[mesh.indices[f * 3 + c]]);
            }
        const Math::vec3 extent = high - low;
        if (std::max(extent.x, std::max(extent.y, extent.z)) < voxel * 4.0f)
            for (const u32 f : members)
                dropped[f] = 1;
    }

    for (const std::vector<u32>& members : shells)
    {
        if (dropped[members.front()])
            continue;
        u32 best = members.front();
        f32 bestArea = -1.0f;
        for (const u32 f : members)
        {
            const Math::vec3& a = mesh.positions[mesh.indices[f * 3]];
            const Math::vec3& b = mesh.positions[mesh.indices[f * 3 + 1]];
            const Math::vec3& c = mesh.positions[mesh.indices[f * 3 + 2]];
            const f32 area = Math::length(Math::cross(b - a, c - a));
            if (area > bestArea)
            {
                bestArea = area;
                best = f;
            }
        }
        const Math::vec3& a = mesh.positions[mesh.indices[best * 3]];
        const Math::vec3& b = mesh.positions[mesh.indices[best * 3 + 1]];
        const Math::vec3& c = mesh.positions[mesh.indices[best * 3 + 2]];
        const Math::vec3 normal = Math::normalize(Math::cross(b - a, c - a));
        // Positive density is inside the solid: outward normals lead out of it.
        if (field.sampleDensity((a + b + c) / 3.0f + normal * probe) > 0.0f)
            for (const u32 f : members)
                std::swap(mesh.indices[f * 3 + 1], mesh.indices[f * 3 + 2]);
    }

    {
        std::vector<u32> kept;
        kept.reserve(mesh.indices.size());
        for (u32 f = 0; f < faceCount; ++f)
            if (!dropped[f])
                kept.insert(kept.end(), mesh.indices.begin() + f * 3, mesh.indices.begin() + f * 3 + 3);
        mesh.indices = std::move(kept);
    }
    const usize shadedFaceCount = mesh.indices.size() / 3;
    std::vector<Math::vec3> faceNormal(shadedFaceCount);
    for (u32 f = 0; f < shadedFaceCount; ++f)
    {
        const Math::vec3& a = mesh.positions[mesh.indices[f * 3]];
        const Math::vec3& b = mesh.positions[mesh.indices[f * 3 + 1]];
        const Math::vec3& c = mesh.positions[mesh.indices[f * 3 + 2]];
        faceNormal[f] = Math::cross(b - a, c - a);
    }
    std::vector<std::vector<u32>> around(mesh.positions.size());
    for (u32 f = 0; f < shadedFaceCount; ++f)
        for (u32 c = 0; c < 3; ++c)
            around[mesh.indices[f * 3 + c]].push_back(f);

    constexpr f32 kCreaseCos = 0.8192f; // cos(35 degrees)
    MeshData shaded;
    shaded.positions.reserve(mesh.positions.size());
    shaded.normals.reserve(mesh.positions.size());
    shaded.indices.resize(mesh.indices.size());
    std::unordered_map<u64, u32> made; // (vertex, quantised normal) -> new vertex
    for (u32 f = 0; f < shadedFaceCount; ++f)
    {
        const f32 length = Math::length(faceNormal[f]);
        const Math::vec3 own = length > 1.0e-20f ? faceNormal[f] / length : Math::vec3(0.0f, 1.0f, 0.0f);
        for (u32 c = 0; c < 3; ++c)
        {
            const u32 v = mesh.indices[f * 3 + c];
            Math::vec3 sum(0.0f);
            for (const u32 other : around[v])
            {
                const f32 otherLength = Math::length(faceNormal[other]);
                if (otherLength > 1.0e-20f && Math::dot(faceNormal[other] / otherLength, own) >= kCreaseCos)
                    sum += faceNormal[other];
            }
            const f32 sumLength = Math::length(sum);
            const Math::vec3 normal = sumLength > 1.0e-20f ? sum / sumLength : own;

            const s64 qx = static_cast<s64>(std::lround(normal.x * 64.0f));
            const s64 qy = static_cast<s64>(std::lround(normal.y * 64.0f));
            const s64 qz = static_cast<s64>(std::lround(normal.z * 64.0f));
            const u64 cluster = (static_cast<u64>(v) << 24) ^ static_cast<u64>((qx + 64) | ((qy + 64) << 8) | ((qz + 64) << 16));
            auto found = made.find(cluster);
            if (found == made.end())
            {
                found = made.emplace(cluster, static_cast<u32>(shaded.positions.size())).first;
                shaded.positions.push_back(mesh.positions[v]);
                shaded.normals.push_back(normal);
            }
            shaded.indices[f * 3 + c] = found->second;
        }
    }
    mesh = std::move(shaded);
}
} // namespace

bool MeshEdit::booleanMeshes(const MeshData& a, const MeshData& b, BooleanOp op, u32 resolution, MeshData& out,
                             std::string* error)
{
    if (resolution < kMinBooleanResolution || resolution > kMaxBooleanResolution)
        return fail(error, "resolution must be between " + std::to_string(kMinBooleanResolution) + " and " +
                               std::to_string(kMaxBooleanResolution));

    Volume::MeshSource left;
    Volume::MeshSource right;
    if (!left.build(a))
        return fail(error, "the first shape has no triangles");
    if (!right.build(b))
        return fail(error, "the second shape has no triangles");

    AABB box = left.bounds();
    box.merge(right.bounds());
    const Math::vec3 size = box.max - box.min;
    const f32 longest = std::max(size.x, std::max(size.y, size.z));
    if (!(longest > 1.0e-6f))
        return fail(error, "the shapes have no extent");
    const f32 voxel = longest / static_cast<f32>(resolution);
    // Offset the grid by an odd fraction of a cell: a plane on round coordinates makes the inside test flip on the surface.
    box.min -= Math::vec3(voxel * 2.0f) + Math::vec3(voxel * 0.3711f, voxel * 0.2719f, voxel * 0.4831f);
    box.max += Math::vec3(voxel * 2.0f);

    const Volume::UnionSource combinedUnion(left, right);
    const Volume::IntersectionSource combinedIntersection(left, right);
    const Volume::DifferenceSource combinedDifference(left, right);
    const Volume::Source* source = &combinedUnion;
    if (op == BooleanOp::Intersection)
        source = &combinedIntersection;
    else if (op == BooleanOp::Difference)
        source = &combinedDifference;

    Volume::MeshingSettings settings;
    settings.bounds = box;
    settings.voxelSize = voxel;
    settings.isoLevel = 0.0f;
    settings.generateUVs = false;

    MeshData meshed;
    if (!Volume::buildMesh(*source, settings, meshed))
        return fail(error, "the shapes could not be meshed");
    if (meshed.indices.size() < 3)
        return fail(error, op == BooleanOp::Intersection ? "the shapes do not overlap" : "the result is empty");

    orientAndShade(meshed, *source, voxel);

    out = std::move(meshed);
    return true;
}
