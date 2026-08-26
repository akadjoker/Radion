#include "PCH.h"

#include "MeshClipper.h"

#include "Color.h"

namespace Radion
{
namespace
{
constexpr f32 kClipEpsilon = 1e-6f;
constexpr u32 kInvalidVertex = static_cast<u32>(-1);

struct ClipVertex
{
    Math::Vec3 position{0.0f};
    Math::Vec3 normal{0.0f, 1.0f, 0.0f};
    Math::Vec4 tangent{1.0f, 0.0f, 0.0f, 1.0f};
    Math::Vec2 uv{0.0f};
    Math::Vec2 uv2{0.0f};
    u32 color = Color::White;
    f32 distance = 0.0f;
    u32 sourceIndex = kInvalidVertex;
};

template <class T>
T vertexAttribute(const std::vector<T>& values, u32 index, const T& fallback)
{
    return index < values.size() ? values[index] : fallback;
}

ClipVertex readVertex(const MeshData& mesh, u32 index, const Math::Vec3& normal, f32 offset)
{
    ClipVertex vertex;
    vertex.position = mesh.positions[index];
    vertex.normal = vertexAttribute(mesh.normals, index, Math::Vec3(0.0f, 1.0f, 0.0f));
    vertex.tangent =
        vertexAttribute(mesh.tangents, index, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    vertex.uv = vertexAttribute(mesh.uvs, index, Math::Vec2(0.0f));
    vertex.uv2 = vertexAttribute(mesh.uvs2, index, Math::Vec2(0.0f));
    vertex.color = vertexAttribute(mesh.colors, index, Color::White);
    vertex.distance = glm::dot(normal, vertex.position) + offset;
    vertex.sourceIndex = index;
    return vertex;
}

ClipVertex interpolate(const ClipVertex& a, const ClipVertex& b)
{
    const f32 denominator = a.distance - b.distance;
    const f32 t = std::abs(denominator) > kClipEpsilon
                      ? glm::clamp(a.distance / denominator, 0.0f, 1.0f)
                      : 0.5f;
    ClipVertex vertex;
    vertex.position = glm::mix(a.position, b.position, t);

    const Math::Vec3 normal = glm::mix(a.normal, b.normal, t);
    vertex.normal = glm::dot(normal, normal) > kClipEpsilon * kClipEpsilon
                        ? glm::normalize(normal)
                        : a.normal;

    const Math::Vec3 tangent = glm::mix(Math::Vec3(a.tangent), Math::Vec3(b.tangent), t);
    const Math::Vec3 tangentDirection = glm::dot(tangent, tangent) > kClipEpsilon * kClipEpsilon
                                           ? glm::normalize(tangent)
                                           : Math::Vec3(a.tangent);
    vertex.tangent = Math::Vec4(tangentDirection, t < 0.5f ? a.tangent.w : b.tangent.w);
    vertex.uv = glm::mix(a.uv, b.uv, t);
    vertex.uv2 = glm::mix(a.uv2, b.uv2, t);
    vertex.color = Color::lerp(Color(a.color), Color(b.color), t).value();
    return vertex;
}

bool kept(const ClipVertex& vertex, bool keepPositive)
{
    return keepPositive ? vertex.distance >= -kClipEpsilon : vertex.distance <= kClipEpsilon;
}

u32 appendVertex(MeshData& output, const MeshData& input, const ClipVertex& vertex)
{
    const u32 index = static_cast<u32>(output.positions.size());
    output.positions.push_back(vertex.position);
    if (!input.normals.empty())
        output.normals.push_back(vertex.normal);
    if (!input.tangents.empty())
        output.tangents.push_back(vertex.tangent);
    if (!input.uvs.empty())
        output.uvs.push_back(vertex.uv);
    if (!input.uvs2.empty())
        output.uvs2.push_back(vertex.uv2);
    if (!input.colors.empty())
        output.colors.push_back(vertex.color);
    output.bounds.expand(vertex.position);
    return index;
}

void clipTriangle(const MeshData& input, const Math::Vec3& normal, f32 offset,
                  bool keepPositive, u32 first, u32 second, u32 third, MeshData& output,
                  std::vector<u32>& remap)
{
    ClipVertex polygon[3] = {readVertex(input, first, normal, offset),
                             readVertex(input, second, normal, offset),
                             readVertex(input, third, normal, offset)};
    ClipVertex clipped[4];
    u32 clippedCount = 0;
    for (u32 i = 0; i < 3; ++i)
    {
        const ClipVertex& current = polygon[i];
        const ClipVertex& previous = polygon[(i + 2) % 3];
        const bool currentKept = kept(current, keepPositive);
        const bool previousKept = kept(previous, keepPositive);
        if (currentKept != previousKept)
            clipped[clippedCount++] = interpolate(previous, current);
        if (currentKept)
            clipped[clippedCount++] = current;
    }

    if (clippedCount < 3)
        return;

    u32 clippedIndices[4];
    for (u32 i = 0; i < clippedCount; ++i)
    {
        const u32 sourceIndex = clipped[i].sourceIndex;
        if (sourceIndex != kInvalidVertex && remap[sourceIndex] != kInvalidVertex)
            clippedIndices[i] = remap[sourceIndex];
        else
        {
            clippedIndices[i] = appendVertex(output, input, clipped[i]);
            if (sourceIndex != kInvalidVertex)
                remap[sourceIndex] = clippedIndices[i];
        }
    }
    for (u32 i = 1; i + 1 < clippedCount; ++i)
    {
        output.indices.push_back(clippedIndices[0]);
        output.indices.push_back(clippedIndices[i]);
        output.indices.push_back(clippedIndices[i + 1]);
    }
}

bool validSubmesh(const MeshData& mesh, const SubMesh& submesh)
{
    const u64 end = static_cast<u64>(submesh.indexOffset) + submesh.indexCount;
    return submesh.indexCount % 3 == 0 && end <= mesh.indices.size();
}
}

bool clipMeshByPlane(const MeshData& input, const Math::Vec3& planeNormal, f32 planeOffset,
                     bool keepPositive, MeshData& output)
{
    const f32 normalLength = glm::length(planeNormal);
    if (input.positions.empty() || input.indices.empty() || input.indices.size() % 3 != 0 ||
        !input.skin.empty() || !std::isfinite(normalLength) || normalLength <= kClipEpsilon ||
        !std::isfinite(planeOffset))
        return false;
    for (u32 index : input.indices)
        if (index >= input.positions.size())
            return false;
    for (const SubMesh& submesh : input.submeshes)
        if (!validSubmesh(input, submesh))
            return false;

    const Math::Vec3 normal = planeNormal / normalLength;
    const f32 offset = planeOffset / normalLength;
    MeshData result;
    result.materials = input.materials;
    result.materialTextureFiles = input.materialTextureFiles;
    result.materialNormalFiles = input.materialNormalFiles;
    result.materialSurfaceFiles = input.materialSurfaceFiles;
    result.materialEmissiveFiles = input.materialEmissiveFiles;
    result.materialHeightFiles = input.materialHeightFiles;
    result.positions.reserve(input.positions.size());
    result.indices.reserve(input.indices.size());
    if (!input.normals.empty())
        result.normals.reserve(input.normals.size());
    if (!input.tangents.empty())
        result.tangents.reserve(input.tangents.size());
    if (!input.uvs.empty())
        result.uvs.reserve(input.uvs.size());
    if (!input.uvs2.empty())
        result.uvs2.reserve(input.uvs2.size());
    if (!input.colors.empty())
        result.colors.reserve(input.colors.size());
    std::vector<u32> remap(input.positions.size(), kInvalidVertex);

    const auto clipRange = [&](u32 first, u32 count, const SubMesh& sourceSubmesh) {
        SubMesh submesh = sourceSubmesh;
        submesh.indexOffset = static_cast<u32>(result.indices.size());
        submesh.indexCount = 0;
        submesh.bounds = AABB();
        const u32 end = first + count;
        for (u32 i = first; i < end; i += 3)
            clipTriangle(input, normal, offset, keepPositive, input.indices[i],
                         input.indices[i + 1], input.indices[i + 2], result, remap);
        submesh.indexCount = static_cast<u32>(result.indices.size()) - submesh.indexOffset;
        if (submesh.indexCount == 0)
            return;
        for (u32 i = submesh.indexOffset; i < submesh.indexOffset + submesh.indexCount; ++i)
            submesh.bounds.expand(result.positions[result.indices[i]]);
        result.submeshes.push_back(submesh);
    };

    if (input.submeshes.empty())
    {
        SubMesh wholeMesh;
        clipRange(0, static_cast<u32>(input.indices.size()), wholeMesh);
    }
    else
    {
        for (const SubMesh& submesh : input.submeshes)
            clipRange(submesh.indexOffset, submesh.indexCount, submesh);
    }

    output = std::move(result);
    return true;
}

}
