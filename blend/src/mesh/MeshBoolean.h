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
    Difference,   // a minus b
    Intersection
};

// A solid-geometry combination of two closed meshes, by way of the engine's
// density fields: each mesh becomes the distance to its surface (inside
// positive), the two are combined, and the result is meshed again on a grid of
// `resolution` cells along the longest side.
//
// It is a remesh, not an exact cut: the result is watertight and smooth-shaded
// but its triangles are the grid's, small features finer than a cell are lost,
// and UVs and the inputs' own triangulation do not survive. Raise `resolution`
// for finer detail (and a slower call).
//
// Both inputs must be closed solids - an open mesh has no inside, and the result
// is then meaningless. Returns false, with `error` set, for an empty input, a
// resolution out of range, or an empty result.
bool booleanMeshes(const MeshData& a, const MeshData& b, BooleanOp op, u32 resolution, MeshData& out,
                   std::string* error = nullptr);

constexpr u32 kMinBooleanResolution = 8;
constexpr u32 kMaxBooleanResolution = 160;

} // namespace Radion::MeshEdit

#endif // RADION_BLENDER_MESH_BOOLEAN_H
