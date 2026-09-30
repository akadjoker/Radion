#include "PCH.h"
#include "mesh/MeshUv.h"

#include "mesh/MeshEdit.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>

namespace Radion::MeshUv
{

namespace
{
bool triangleValid(const MeshData& mesh, u32 triangle)
{
    return static_cast<usize>(triangle) * 3 + 2 < mesh.indices.size();
}

u32 findRoot(std::vector<u32>& parent, u32 item)
{
    while (parent[item] != item)
    {
        parent[item] = parent[parent[item]];
        item = parent[item];
    }
    return item;
}

bool isPinned(const std::vector<u8>* pinned, u32 vertex)
{
    return pinned && vertex < pinned->size() && (*pinned)[vertex] != 0;
}
} // namespace

void ensureUvs(MeshData& mesh)
{
    if (mesh.uvs.size() != mesh.positions.size())
        mesh.uvs.resize(mesh.positions.size(), glm::vec2(0.0f));
}

std::vector<u32> verticesOfTriangles(const MeshData& mesh, const std::vector<u32>& triangles)
{
    std::vector<u32> vertices;
    for (const u32 triangle : triangles)
    {
        if (!triangleValid(mesh, triangle))
            continue;
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 vertex = mesh.indices[static_cast<usize>(triangle) * 3 + corner];
            if (vertex < mesh.positions.size())
                vertices.push_back(vertex);
        }
    }
    std::sort(vertices.begin(), vertices.end());
    vertices.erase(std::unique(vertices.begin(), vertices.end()), vertices.end());
    return vertices;
}

std::vector<u32> islands(const MeshData& mesh, u32* count)
{
    const u32 triangleCount = static_cast<u32>(mesh.indices.size() / 3);
    std::vector<u32> parent(triangleCount);
    std::iota(parent.begin(), parent.end(), 0u);

    // The first triangle seen at each vertex; every later one joins it.
    std::vector<s64> firstAt(mesh.positions.size(), -1);
    for (u32 triangle = 0; triangle < triangleCount; ++triangle)
    {
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 vertex = mesh.indices[static_cast<usize>(triangle) * 3 + corner];
            if (vertex >= firstAt.size())
                continue;
            if (firstAt[vertex] < 0)
                firstAt[vertex] = triangle;
            else
                parent[findRoot(parent, triangle)] = findRoot(parent, static_cast<u32>(firstAt[vertex]));
        }
    }

    std::vector<u32> result(triangleCount, 0);
    std::map<u32, u32> number;
    for (u32 triangle = 0; triangle < triangleCount; ++triangle)
    {
        const u32 root = findRoot(parent, triangle);
        const auto known = number.find(root);
        if (known == number.end())
            result[triangle] = number[root] = static_cast<u32>(number.size());
        else
            result[triangle] = known->second;
    }
    if (count)
        *count = static_cast<u32>(number.size());
    return result;
}

std::vector<u32> islandVertices(const MeshData& mesh, const std::vector<u32>& triangles)
{
    const std::vector<u32> island = islands(mesh);
    std::vector<bool> wanted;
    for (const u32 triangle : triangles)
    {
        if (triangle >= island.size())
            continue;
        if (island[triangle] >= wanted.size())
            wanted.resize(island[triangle] + 1, false);
        wanted[island[triangle]] = true;
    }
    std::vector<u32> members;
    for (u32 triangle = 0; triangle < static_cast<u32>(island.size()); ++triangle)
    {
        if (island[triangle] < wanted.size() && wanted[island[triangle]])
            members.push_back(triangle);
    }
    return verticesOfTriangles(mesh, members);
}

Rect bounds(const MeshData& mesh, const std::vector<u32>& vertices)
{
    Rect rect;
    for (const u32 vertex : vertices)
    {
        if (vertex >= mesh.uvs.size())
            continue;
        const glm::vec2& uv = mesh.uvs[vertex];
        if (!rect.valid)
        {
            rect.min = rect.max = uv;
            rect.valid = true;
        }
        else
        {
            rect.min = glm::min(rect.min, uv);
            rect.max = glm::max(rect.max, uv);
        }
    }
    return rect;
}

u32 transform(MeshData& mesh, const std::vector<u32>& vertices, const std::vector<u8>* pinned, const glm::vec2& pivot,
              const Transform& change)
{
    ensureUvs(mesh);
    // v runs DOWN the image (the engine, like glTF, puts UV (0,0) at the top left
    // of a texture), so a turn that looks counter-clockwise on screen is a
    // clockwise one in (u, v) coordinates.
    const f32 radians = -glm::radians(change.rotateDegrees);
    const f32 c = std::cos(radians);
    const f32 s = std::sin(radians);
    u32 moved = 0;
    for (const u32 vertex : vertices)
    {
        if (vertex >= mesh.uvs.size() || isPinned(pinned, vertex))
            continue;
        glm::vec2 offset = (mesh.uvs[vertex] - pivot) * change.scale;
        offset = glm::vec2(offset.x * c - offset.y * s, offset.x * s + offset.y * c);
        mesh.uvs[vertex] = pivot + offset + change.translate;
        ++moved;
    }
    return moved;
}

u32 fit(MeshData& mesh, const std::vector<u32>& vertices, const std::vector<u8>* pinned, bool keepAspect, f32 margin)
{
    ensureUvs(mesh);
    const Rect rect = bounds(mesh, vertices);
    if (!rect.valid)
        return 0;
    margin = glm::clamp(margin, 0.0f, 0.45f);
    const f32 room = 1.0f - 2.0f * margin;
    const glm::vec2 size = glm::max(rect.size(), glm::vec2(1e-8f));
    glm::vec2 scale = glm::vec2(room) / size;
    if (keepAspect)
        scale = glm::vec2(std::min(scale.x, scale.y));

    // A rectangle of zero width or height has nothing to stretch.
    if (rect.size().x < 1e-8f)
        scale.x = keepAspect ? scale.x : 1.0f;
    if (rect.size().y < 1e-8f)
        scale.y = keepAspect ? scale.y : 1.0f;

    const glm::vec2 centre = rect.center();
    u32 moved = 0;
    for (const u32 vertex : vertices)
    {
        if (vertex >= mesh.uvs.size() || isPinned(pinned, vertex))
            continue;
        mesh.uvs[vertex] = (mesh.uvs[vertex] - centre) * scale + glm::vec2(0.5f);
        ++moved;
    }
    return moved;
}

u32 boxMap(MeshData& mesh, const std::vector<u32>& triangles, f32 tile, const glm::vec2& offset,
           std::vector<u32>* touched)
{
    ensureUvs(mesh);
    const u32 triangleCount = static_cast<u32>(mesh.indices.size() / 3);

    std::vector<bool> inList(triangleCount, false);
    for (const u32 triangle : triangles)
    {
        if (triangle < triangleCount)
            inList[triangle] = true;
    }
    // Vertices the list does not own must keep their UV.
    std::vector<bool> usedOutside(mesh.positions.size(), false);
    for (u32 triangle = 0; triangle < triangleCount; ++triangle)
    {
        if (inList[triangle])
            continue;
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 vertex = mesh.indices[static_cast<usize>(triangle) * 3 + corner];
            if (vertex < usedOutside.size())
                usedOutside[vertex] = true;
        }
    }

    // (vertex, plane) -> the vertex that carries that plane's UV for it.
    std::map<std::pair<u32, u32>, u32> carrier;
    std::vector<bool> claimed(mesh.positions.size(), false);
    const u32 originalVertices = static_cast<u32>(mesh.positions.size());
    u32 added = 0;

    for (u32 triangle = 0; triangle < triangleCount; ++triangle)
    {
        if (!inList[triangle])
            continue;
        u32 corner3[3];
        for (u32 corner = 0; corner < 3; ++corner)
            corner3[corner] = mesh.indices[static_cast<usize>(triangle) * 3 + corner];
        if (corner3[0] >= originalVertices || corner3[1] >= originalVertices || corner3[2] >= originalVertices)
            continue;

        const glm::vec3& p0 = mesh.positions[corner3[0]];
        const glm::vec3 normal = glm::cross(mesh.positions[corner3[1]] - p0, mesh.positions[corner3[2]] - p0);
        const glm::vec3 magnitude = glm::abs(normal);
        const u32 axis = magnitude.x >= magnitude.y && magnitude.x >= magnitude.z ? 0u : magnitude.y >= magnitude.z ? 1u : 2u;
        const bool positive = normal[static_cast<glm::length_t>(axis)] > 0.0f;
        const u32 plane = axis * 2 + (positive ? 1u : 0u);

        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 original = corner3[corner];
            u32 vertex;
            const auto known = carrier.find({original, plane});
            if (known != carrier.end())
                vertex = known->second;
            else
            {
                if (!claimed[original] && !usedOutside[original])
                    vertex = original;
                else
                {
                    vertex = MeshEdit::duplicateVertex(mesh, original);
                    ++added;
                }
                claimed[original] = true;
                carrier[{original, plane}] = vertex;
            }
            mesh.indices[static_cast<usize>(triangle) * 3 + corner] = vertex;

            const glm::vec3& p = mesh.positions[original];
            glm::vec2 uv;
            if (axis == 0)
                uv = glm::vec2(positive ? -p.z : p.z, p.y);
            else if (axis == 1)
                uv = glm::vec2(p.x, positive ? -p.z : p.z);
            else
                uv = glm::vec2(positive ? p.x : -p.x, p.y);
            mesh.uvs[vertex] = uv * tile + offset;
        }
    }

    if (touched)
    {
        std::vector<u32> list;
        for (u32 triangle = 0; triangle < triangleCount; ++triangle)
        {
            if (inList[triangle])
                list.push_back(triangle);
        }
        *touched = verticesOfTriangles(mesh, list);
    }
    return added;
}

std::vector<u8> renderLayout(const MeshData& mesh, const std::vector<u32>& triangles, u32 size,
                             const std::vector<u8>& background, u32 backgroundSize)
{
    size = std::max(size, 16u);
    std::vector<u8> image(static_cast<usize>(size) * size * 4);

    // A dark checker, or the texture stretched over the square.
    const bool haveBackground = backgroundSize > 0 && background.size() >= static_cast<usize>(backgroundSize) * backgroundSize * 4;
    for (u32 y = 0; y < size; ++y)
    {
        for (u32 x = 0; x < size; ++x)
        {
            u8* pixel = &image[(static_cast<usize>(y) * size + x) * 4];
            if (haveBackground)
            {
                const u32 sx = std::min(backgroundSize - 1, x * backgroundSize / size);
                const u32 sy = std::min(backgroundSize - 1, y * backgroundSize / size);
                const u8* source = &background[(static_cast<usize>(sy) * backgroundSize + sx) * 4];
                // Dimmed so the wire stays readable.
                for (int c = 0; c < 3; ++c)
                    pixel[c] = static_cast<u8>(source[c] * 0.6f);
            }
            else
            {
                const u8 shade = ((x / 16 + y / 16) % 2 == 0) ? 40 : 52;
                pixel[0] = pixel[1] = pixel[2] = shade;
            }
            pixel[3] = 255;
        }
    }

    auto plot = [&](s32 x, s32 y, u8 r, u8 g, u8 b)
    {
        if (x < 0 || y < 0 || x >= static_cast<s32>(size) || y >= static_cast<s32>(size))
            return;
        u8* pixel = &image[(static_cast<usize>(y) * size + static_cast<usize>(x)) * 4];
        pixel[0] = r;
        pixel[1] = g;
        pixel[2] = b;
    };
    auto line = [&](glm::vec2 a, glm::vec2 b, u8 r, u8 g, u8 bl)
    {
        const f32 steps = std::max(std::abs(b.x - a.x), std::abs(b.y - a.y));
        const s32 count = static_cast<s32>(std::ceil(steps));
        for (s32 i = 0; i <= count; ++i)
        {
            const f32 t = count == 0 ? 0.0f : static_cast<f32>(i) / static_cast<f32>(count);
            const glm::vec2 p = glm::mix(a, b, t);
            plot(static_cast<s32>(std::lround(p.x)), static_cast<s32>(std::lround(p.y)), r, g, bl);
        }
    };
    // UV (0,0) is the top left of the square, as in a texture file: v runs down.
    auto toPixel = [&](const glm::vec2& uv)
    {
        return glm::vec2(uv.x * static_cast<f32>(size - 1), uv.y * static_cast<f32>(size - 1));
    };

    // The 0..1 frame.
    const glm::vec2 corner00 = toPixel(glm::vec2(0, 0)), corner10 = toPixel(glm::vec2(1, 0));
    const glm::vec2 corner11 = toPixel(glm::vec2(1, 1)), corner01 = toPixel(glm::vec2(0, 1));
    line(corner00, corner10, 120, 120, 120);
    line(corner10, corner11, 120, 120, 120);
    line(corner11, corner01, 120, 120, 120);
    line(corner01, corner00, 120, 120, 120);

    for (const u32 triangle : triangles)
    {
        if (!triangleValid(mesh, triangle))
            continue;
        glm::vec2 p[3];
        bool ok = true;
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 vertex = mesh.indices[static_cast<usize>(triangle) * 3 + corner];
            if (vertex >= mesh.uvs.size())
            {
                ok = false;
                break;
            }
            p[corner] = toPixel(mesh.uvs[vertex]);
        }
        if (!ok)
            continue;
        for (u32 corner = 0; corner < 3; ++corner)
            line(p[corner], p[(corner + 1) % 3], 255, 200, 60);
    }
    return image;
}

} // namespace Radion::MeshUv
