#ifndef RADION_GEOMETRY_HULL_MESH_H
#define RADION_GEOMETRY_HULL_MESH_H

#include "ConvexHullComputer.h"
#include "Types.h"

#include <vector>

namespace Radion
{
struct MeshData;
}

namespace Radion::Geometry
{

// Triangles out of the edge/face structure that ConvexHullComputer and Shard
// both carry. Neither of them produces geometry anything can draw or save on
// its own, so every caller that wanted a mesh had to walk the face loops
// itself - which is what examples/voronoi_shatter_demo/main.cpp:80 was doing
// inline to draw its debug lines.
//
// Every triangle gets three vertices of its own carrying the face's normal. A
// convex hull has no smooth edges: sharing vertices between faces would
// average the normals across corners that are meant to be hard.
//
// False when there are no faces to walk; `out` is cleared either way.
bool buildHullMesh(const std::vector<Math::vec3>& vertices,
                   const std::vector<ConvexHullComputer::Edge>& edges,
                   const std::vector<int>& faces, MeshData& out);

// The convex hull of a point cloud, as a mesh. The hull itself is computed
// with no shrink.
bool buildConvexHullMesh(const std::vector<Math::vec3>& points, MeshData& out);

} // namespace Radion::Geometry

#endif // RADION_GEOMETRY_HULL_MESH_H
