#include "PCH.h"

#include "HullMesh.h"

#include "Mesh.h"

namespace Radion::Geometry
{

bool buildHullMesh(const std::vector<glm::vec3>& vertices,
                   const std::vector<ConvexHullComputer::Edge>& edges,
                   const std::vector<int>& faces, MeshData& out)
{
    out.clear();
    if (faces.empty() || edges.empty() || vertices.empty())
        return false;

    const int vertexCount = static_cast<int>(vertices.size());
    const int edgeCount = static_cast<int>(edges.size());

    const int faceCount = static_cast<int>(faces.size());
    for (int f = 0; f < faceCount; f++)
    {
        if (faces[static_cast<usize>(f)] < 0 || faces[static_cast<usize>(f)] >= edgeCount)
            continue;

        const ConvexHullComputer::Edge* edge = &edges[static_cast<usize>(faces[static_cast<usize>(f)])];
        int v0 = edge->getSourceVertex();
        int v1 = edge->getTargetVertex();
        edge = edge->getNextEdgeOfFace();
        int v2 = edge->getTargetVertex();

        // Guard against a step count rather than trusting the loop to close:
        // a malformed face would otherwise walk its edges forever.
        int steps = 0;
        while (v2 != v0 && steps < edgeCount)
        {
            if (v0 < 0 || v1 < 0 || v2 < 0 || v0 >= vertexCount || v1 >= vertexCount ||
                v2 >= vertexCount)
                break;

            const glm::vec3& a = vertices[static_cast<usize>(v0)];
            const glm::vec3& b = vertices[static_cast<usize>(v1)];
            const glm::vec3& c = vertices[static_cast<usize>(v2)];

            const glm::vec3 cross = glm::cross(b - a, c - a);
            const f32 length = glm::length(cross);
            if (length > 1e-12f)
            {
                const glm::vec3 normal = cross / length;
                const u32 base = static_cast<u32>(out.positions.size());

                out.positions.push_back(a);
                out.positions.push_back(b);
                out.positions.push_back(c);
                out.normals.push_back(normal);
                out.normals.push_back(normal);
                out.normals.push_back(normal);
                out.uvs.push_back(glm::vec2(0.0f));
                out.uvs.push_back(glm::vec2(1.0f, 0.0f));
                out.uvs.push_back(glm::vec2(0.0f, 1.0f));

                out.indices.push_back(base);
                out.indices.push_back(base + 1);
                out.indices.push_back(base + 2);

                out.bounds.expand(a);
                out.bounds.expand(b);
                out.bounds.expand(c);
            }

            edge = edge->getNextEdgeOfFace();
            v1 = v2;
            v2 = edge->getTargetVertex();
            ++steps;
        }
    }

    if (out.indices.empty())
    {
        out.clear();
        return false;
    }

    SubMesh submesh;
    submesh.indexOffset = 0;
    submesh.indexCount = static_cast<u32>(out.indices.size());
    submesh.bounds = out.bounds;
    out.submeshes.push_back(submesh);
    return true;
}

bool buildConvexHullMesh(const std::vector<glm::vec3>& points, MeshData& out)
{
    out.clear();
    if (points.size() < 4)
        return false;

    ConvexHullComputer hull;
    hull.compute(&points[0].x, static_cast<int>(sizeof(glm::vec3)), static_cast<int>(points.size()),
                 0.0f, 0.0f);

    return buildHullMesh(hull.vertices, hull.edges, hull.faces, out);
}

} // namespace Radion::Geometry
