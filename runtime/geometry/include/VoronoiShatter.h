#ifndef RADION_GEOMETRY_VORONOI_SHATTER_H
#define RADION_GEOMETRY_VORONOI_SHATTER_H

#include "ConvexHullComputer.h"
#include "Types.h"

#include <glm/glm.hpp>
#include <vector>

namespace Radion::Geometry
{

struct Shard
{
    std::vector<glm::vec3> vertices;
    std::vector<ConvexHullComputer::Edge> edges;
    std::vector<int> faces;
    glm::vec3 centroid = glm::vec3(0.0f);
    f32 volume = 0.0f;
};

// Voronoi cell decomposition of a convex point cloud (Stan Melax volume
// integration for mass properties, Preparata & Hong for the cell hulls).
class VoronoiShatter
{
public:
    // planes: half-space equations, xyz = normal, w = offset; a point p is inside
    // when dot(normal, p) + w <= 0 for every plane. Returns the vertices formed by
    // every triple-plane intersection that lies inside all planes, together with
    // the (sorted, unique) indices of the planes that actually produced one of
    // those vertices. Returns false when no such vertex exists.
    static bool getVerticesInsidePlanes(const std::vector<glm::vec4>& planes,
                                         std::vector<glm::vec3>& verticesOut,
                                         std::vector<int>& planeIndicesOut);

    // sourceVertices: convex point cloud of the shape being shattered, in the
    // shape's own local space. voronoiPoints: cell sites, in that same local
    // space. Produces one Shard per Voronoi point whose cell is non-empty; each
    // shard's vertices are relative to its own centroid (Shard::centroid holds
    // the position of that centroid back in the shared local space).
    static void shatter(const std::vector<glm::vec3>& sourceVertices,
                         const std::vector<glm::vec3>& voronoiPoints,
                         std::vector<Shard>& shardsOut);
};

} // namespace Radion::Geometry

#endif // RADION_GEOMETRY_VORONOI_SHATTER_H
