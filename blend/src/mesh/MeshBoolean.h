#ifndef RADION_BLENDER_MESH_BOOLEAN_H
#define RADION_BLENDER_MESH_BOOLEAN_H

#include "Mesh.h"
#include "Types.h"

#include <string>

namespace Radion::MeshEdit
{

enum class BooleanOp : u8
{
    Union,
    Difference,
    Intersection
};

// Combines two closed meshes via density fields and re-meshes on a grid of `resolution` cells; a remesh, not an exact cut (UVs and small features are lost).
// Inputs must be closed solids. Returns false with `error` set for an empty input, bad resolution, or empty result.
bool booleanMeshes(const MeshData& a, const MeshData& b, BooleanOp op, u32 resolution, MeshData& out,
                   std::string* error = nullptr);

constexpr u32 kMinBooleanResolution = 8;
constexpr u32 kMaxBooleanResolution = 160;

} // namespace Radion::MeshEdit

#endif // RADION_BLENDER_MESH_BOOLEAN_H
