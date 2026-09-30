#ifndef RADION_BLENDER_MESH_PAINT_H
#define RADION_BLENDER_MESH_PAINT_H

#include "Mesh.h"
#include "Types.h"

#include <vector>

namespace Radion::MeshPaint
{

// Vertex colours on a MeshData: pure functions, no engine, no GL.
// MeshData::colors is 4 bytes/vertex (glTF COLOR_0 layout), empty (all white) or as long as `positions`; colours are LINEAR, so convert sRGB first (toLinear).

Math::vec3 toLinear(const Math::vec3& srgb);
Math::vec4 unpack(u32 packed);
u32 pack(const Math::vec4& color);

// Blends `color` into the listed vertices by `opacity`; out-of-range ids are ignored; creates the array (white) if missing. Returns how many changed.
u32 paintVertices(MeshData& mesh, const std::vector<u32>& vertices, const Math::vec4& color, f32 opacity);

// Soft round brush within `radius` of `center` (limited to `subset`), weight opacity * falloff: full to `hardness` * radius, easing to zero at the rim.
u32 paintSphere(MeshData& mesh, const std::vector<u32>* subset, const Math::vec3& center, f32 radius, f32 hardness,
                const Math::vec4& color, f32 opacity);

// Resets vertices to white; with null `vertices` the array is dropped so export has no COLOR_0.
void clear(MeshData& mesh, const std::vector<u32>* vertices);

bool hasColors(const MeshData& mesh);

} // namespace Radion::MeshPaint

#endif // RADION_BLENDER_MESH_PAINT_H
