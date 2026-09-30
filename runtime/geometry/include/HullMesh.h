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

// Each triangle has its own vertices with the face normal: hull edges are hard, sharing would smooth them.
// False when there are no faces to walk; `out` is cleared either way.
bool buildHullMesh(const std::vector<Math::vec3>& vertices,
                   const std::vector<ConvexHullComputer::Edge>& edges,
                   const std::vector<int>& faces, MeshData& out);

// Hull of a point cloud as a mesh, computed with no shrink.
bool buildConvexHullMesh(const std::vector<Math::vec3>& points, MeshData& out);

} // namespace Radion::Geometry

#endif // RADION_GEOMETRY_HULL_MESH_H
