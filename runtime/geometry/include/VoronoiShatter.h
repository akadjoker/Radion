#ifndef RADION_GEOMETRY_VORONOI_SHATTER_H
#define RADION_GEOMETRY_VORONOI_SHATTER_H

#include "ConvexHullComputer.h"
#include "Types.h"

#include "Math.h"
#include <vector>

namespace Radion::Geometry
{

struct Shard
{
    std::vector<Math::vec3> vertices;
    std::vector<ConvexHullComputer::Edge> edges;
    std::vector<int> faces;
    Math::vec3 centroid = Math::vec3(0.0f);
    f32 volume = 0.0f;
};

// Voronoi cell decomposition of a convex point cloud (Stan Melax volume
// integration for mass properties, Preparata & Hong for the cell hulls).
class VoronoiShatter
{
public:
    // planes: xyz = normal, w = offset; p is inside when dot(normal, p) + w <= 0. Returns the vertices of triple-plane
    // intersections inside all planes, plus sorted unique indices of planes that produced one. False when none.
    static bool getVerticesInsidePlanes(const std::vector<Math::vec4>& planes,
                                         std::vector<Math::vec3>& verticesOut,
                                         std::vector<int>& planeIndicesOut);

    // sourceVertices and voronoiPoints are in the shape's local space. One Shard per non-empty cell; shard vertices are
    // relative to their centroid (Shard::centroid gives it in local space).
    static void shatter(const std::vector<Math::vec3>& sourceVertices,
                         const std::vector<Math::vec3>& voronoiPoints,
                         std::vector<Shard>& shardsOut);
};

} // namespace Radion::Geometry

#endif // RADION_GEOMETRY_VORONOI_SHATTER_H
