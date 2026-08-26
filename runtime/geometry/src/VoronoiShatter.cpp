#include "PCH.h"

#include "VoronoiShatter.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Radion::Geometry
{

namespace
{

constexpr f32 kInfinity = std::numeric_limits<f32>::infinity();
constexpr f32 kCrossEpsilon = 0.0001f;
constexpr f32 kQuotientEpsilon = 0.0001f;
constexpr f32 kInsideEpsilon = 0.000001f;

void insertSorted(std::vector<int>& values, int value)
{
    std::vector<int>::iterator it = std::lower_bound(values.begin(), values.end(), value);
    if (it == values.end() || *it != value)
    {
        values.insert(it, value);
    }
}

f32 triple(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
{
    return glm::dot(a, glm::cross(b, c));
}

struct DistanceFromPoint
{
    glm::vec3 center;

    bool operator()(const glm::vec3& a, const glm::vec3& b) const
    {
        f32 da = glm::dot(a - center, a - center);
        f32 db = glm::dot(b - center, b - center);
        return da < db;
    }
};

} // namespace

bool VoronoiShatter::getVerticesInsidePlanes(const std::vector<glm::vec4>& planes,
                                              std::vector<glm::vec3>& verticesOut,
                                              std::vector<int>& planeIndicesOut)
{
    verticesOut.clear();
    planeIndicesOut.clear();
    const int numPlanes = (int)planes.size();
    for (int i = 0; i < numPlanes; i++)
    {
        glm::vec3 n1(planes[i]);
        for (int j = i + 1; j < numPlanes; j++)
        {
            glm::vec3 n2(planes[j]);
            glm::vec3 n1n2 = glm::cross(n1, n2);
            if (glm::dot(n1n2, n1n2) > kCrossEpsilon)
            {
                for (int k = j + 1; k < numPlanes; k++)
                {
                    glm::vec3 n3(planes[k]);
                    glm::vec3 n2n3 = glm::cross(n2, n3);
                    glm::vec3 n3n1 = glm::cross(n3, n1);
                    if ((glm::dot(n2n3, n2n3) > kCrossEpsilon) && (glm::dot(n3n1, n3n1) > kCrossEpsilon))
                    {
                        f32 quotient = glm::dot(n1, n2n3);
                        if (std::fabs(quotient) > kQuotientEpsilon)
                        {
                            glm::vec3 potentialVertex = (n2n3 * planes[i].w + n3n1 * planes[j].w + n1n2 * planes[k].w) * (-1.0f / quotient);
                            int l = 0;
                            for (; l < numPlanes; l++)
                            {
                                const glm::vec4& np = planes[l];
                                if (glm::dot(glm::vec3(np), potentialVertex) + np.w > kInsideEpsilon)
                                {
                                    break;
                                }
                            }
                            if (l == numPlanes)
                            {
                                verticesOut.push_back(potentialVertex);
                                insertSorted(planeIndicesOut, i);
                                insertSorted(planeIndicesOut, j);
                                insertSorted(planeIndicesOut, k);
                            }
                        }
                    }
                }
            }
        }
    }
    return !verticesOut.empty();
}

void VoronoiShatter::shatter(const std::vector<glm::vec3>& sourceVertices,
                              const std::vector<glm::vec3>& voronoiPoints,
                              std::vector<Shard>& shardsOut)
{
    shardsOut.clear();

    ConvexHullComputer sourceHull;
    if (!sourceVertices.empty())
    {
        sourceHull.compute(&sourceVertices[0].x, sizeof(glm::vec3), (int)sourceVertices.size(), 0.0f, 0.0f);
    }

    std::vector<glm::vec4> convexPlanes;
    const int numSourceFaces = (int)sourceHull.faces.size();
    for (int i = 0; i < numSourceFaces; i++)
    {
        const ConvexHullComputer::Edge* edge = &sourceHull.edges[sourceHull.faces[i]];
        int v0 = edge->getSourceVertex();
        int v1 = edge->getTargetVertex();
        edge = edge->getNextEdgeOfFace();
        int v2 = edge->getTargetVertex();
        glm::vec3 normal = glm::normalize(glm::cross(sourceHull.vertices[v1] - sourceHull.vertices[v0],
                                                       sourceHull.vertices[v2] - sourceHull.vertices[v0]));
        f32 offset = -glm::dot(normal, sourceHull.vertices[v0]);
        convexPlanes.push_back(glm::vec4(normal, offset));
    }
    const int numConvexPlanes = (int)convexPlanes.size();

    std::vector<glm::vec3> sortedVoronoiPoints(voronoiPoints);
    const int numPoints = (int)voronoiPoints.size();

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec4> planes;
    std::vector<int> planeIndices;

    for (int i = 0; i < numPoints; i++)
    {
        const glm::vec3 curVoronoiPoint = voronoiPoints[i];

        planes = convexPlanes;
        for (int j = 0; j < numConvexPlanes; j++)
        {
            planes[j].w += glm::dot(glm::vec3(planes[j]), curVoronoiPoint);
        }

        f32 maxDistance = kInfinity;
        DistanceFromPoint distanceFromPoint;
        distanceFromPoint.center = curVoronoiPoint;
        std::sort(sortedVoronoiPoints.begin(), sortedVoronoiPoints.end(), distanceFromPoint);

        // Reference note: the ported loop below only (re-)populates "vertices" as a
        // side effect of adding a bisector plane against another Voronoi point, so
        // with a single site (no bisector ever added) it never runs and the cell
        // would be silently dropped. Seed "vertices" from the source shape's own
        // planes once so a lone Voronoi point still recovers the whole shape.
        getVerticesInsidePlanes(planes, vertices, planeIndices);
        for (int j = 1; j < numPoints; j++)
        {
            glm::vec3 normal = sortedVoronoiPoints[j] - curVoronoiPoint;
            f32 nlength = glm::length(normal);
            if (nlength > maxDistance)
            {
                break;
            }
            glm::vec4 plane(normal / nlength, -nlength * 0.5f);
            planes.push_back(plane);
            getVerticesInsidePlanes(planes, vertices, planeIndices);
            if (vertices.empty())
            {
                break;
            }
            int numPlaneIndices = (int)planeIndices.size();
            if (numPlaneIndices != (int)planes.size())
            {
                for (int k = 0; k < numPlaneIndices; k++)
                {
                    if (k != planeIndices[k])
                    {
                        planes[k] = planes[planeIndices[k]];
                    }
                }
                planes.resize(numPlaneIndices);
            }
            maxDistance = glm::length(vertices[0]);
            for (int k = 1; k < (int)vertices.size(); k++)
            {
                f32 distance = glm::length(vertices[k]);
                if (maxDistance < distance)
                {
                    maxDistance = distance;
                }
            }
            maxDistance *= 2.0f;
        }
        if (vertices.empty())
        {
            continue;
        }

        ConvexHullComputer cellHull;
        cellHull.compute(&vertices[0].x, sizeof(glm::vec3), (int)vertices.size(), 0.0f, 0.0f);

        const int numFaces = (int)cellHull.faces.size();
        f32 volume = 0.0f;
        glm::vec3 com(0.0f);
        for (int j = 0; j < numFaces; j++)
        {
            const ConvexHullComputer::Edge* edge = &cellHull.edges[cellHull.faces[j]];
            int v0 = edge->getSourceVertex();
            int v1 = edge->getTargetVertex();
            edge = edge->getNextEdgeOfFace();
            int v2 = edge->getTargetVertex();
            while (v2 != v0)
            {
                f32 vol = triple(cellHull.vertices[v0], cellHull.vertices[v1], cellHull.vertices[v2]);
                volume += vol;
                com += vol * (cellHull.vertices[v0] + cellHull.vertices[v1] + cellHull.vertices[v2]);
                edge = edge->getNextEdgeOfFace();
                v1 = v2;
                v2 = edge->getTargetVertex();
            }
        }
        com /= volume * 4.0f;
        volume /= 6.0f;

        const int numVerts = (int)cellHull.vertices.size();
        for (int j = 0; j < numVerts; j++)
        {
            cellHull.vertices[j] -= com;
        }

        Shard shard;
        shard.vertices = std::move(cellHull.vertices);
        shard.edges = std::move(cellHull.edges);
        shard.faces = std::move(cellHull.faces);
        shard.centroid = curVoronoiPoint + com;
        shard.volume = volume;
        shardsOut.push_back(std::move(shard));
    }
}

} // namespace Radion::Geometry
