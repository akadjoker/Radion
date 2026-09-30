#ifndef RADION_BLENDER_MESH_PAINT_H
#define RADION_BLENDER_MESH_PAINT_H

#include "Mesh.h"
#include "Types.h"

#include <vector>

namespace Radion::MeshPaint
{

// Vertex colours on a MeshData. Pure functions, no engine, no GL.
//
// MeshData::colors holds four bytes per vertex - r, g, b, a in memory, the same
// layout the GPU mesh and glTF's COLOR_0 (normalised unsigned bytes) use - and
// is either empty (every vertex white) or exactly as long as `positions`.
// Colours here are LINEAR, as glTF defines COLOR_0; callers that take sRGB from
// a user convert first (toLinear).

glm::vec3 toLinear(const glm::vec3& srgb);
glm::vec4 unpack(u32 packed);
u32 pack(const glm::vec4& color);

// Blends `color` into each listed vertex by `opacity` (0..1). Vertex ids out of
// range are ignored. Creates the colour array (white) when there is none.
// Returns how many vertices changed.
u32 paintVertices(MeshData& mesh, const std::vector<u32>& vertices, const glm::vec4& color, f32 opacity);

// Soft round brush: every vertex within `radius` of `center` (restricted to
// `subset` when it is given) is painted with weight opacity * falloff, where the
// weight is full out to `hardness` * radius and eases to zero at the radius.
u32 paintSphere(MeshData& mesh, const std::vector<u32>* subset, const glm::vec3& center, f32 radius, f32 hardness,
                const glm::vec4& color, f32 opacity);

// Back to "no vertex colours" for the listed vertices (white), or for the whole
// mesh when `vertices` is null - in which case the array is dropped entirely so
// the exported file carries no COLOR_0.
void clear(MeshData& mesh, const std::vector<u32>* vertices);

// True when any vertex is not plain white.
bool hasColors(const MeshData& mesh);

} // namespace Radion::MeshPaint

#endif // RADION_BLENDER_MESH_PAINT_H
