#ifndef RADION_MESH_CLIPPER_H
#define RADION_MESH_CLIPPER_H

#include "Mesh.h"

namespace Radion
{

bool clipMeshByPlane(const MeshData& input, const Math::Vec3& normal, f32 offset,
                     bool keepPositive, MeshData& output);

}

#endif
