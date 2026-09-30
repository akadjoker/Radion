#ifndef RADION_BLENDER_MESH_UV_H
#define RADION_BLENDER_MESH_UV_H

#include "Mesh.h"
#include "Types.h"

#include <vector>

namespace Radion::MeshUv
{

// Texture-coordinate editing on a triangle MeshData: pure functions, no engine, no GL. UVs belong to vertices, so edits name VERTEX indices (a seam's two vertices move independently).
// `pinned` (optional, one byte per vertex, non-zero = pinned) makes edits skip those vertices.

struct Rect
{
    Math::vec2 min = Math::vec2(0.0f);
    Math::vec2 max = Math::vec2(0.0f);
    bool valid = false;

    Math::vec2 center() const
    {
        return (min + max) * 0.5f;
    }
    Math::vec2 size() const
    {
        return max - min;
    }
};

void ensureUvs(MeshData& mesh);

std::vector<u32> verticesOfTriangles(const MeshData& mesh, const std::vector<u32>& triangles);

// Islands: triangles sharing a VERTEX (not just a point) form one, so a seam separates them. Returns each triangle's island from 0 in order of first appearance; `count` gets the total.
std::vector<u32> islands(const MeshData& mesh, u32* count = nullptr);

std::vector<u32> islandVertices(const MeshData& mesh, const std::vector<u32>& triangles);

Rect bounds(const MeshData& mesh, const std::vector<u32>& vertices);

struct Transform
{
    Math::vec2 translate = Math::vec2(0.0f);
    // Counter-clockwise as drawn in the UV layout (u right, v DOWN: UV (0,0) is the top
    // left of the texture, as in glTF and the engine's own loaders).
    f32 rotateDegrees = 0.0f;
    Math::vec2 scale = Math::vec2(1.0f); // negative flips
};

u32 transform(MeshData& mesh, const std::vector<u32>& vertices, const std::vector<u8>* pinned,
              const Math::vec2& pivot, const Transform& change);

// Fits the vertices' bounds into 0..1 leaving `margin`; `keepAspect` preserves shape and centres, else stretches.
u32 fit(MeshData& mesh, const std::vector<u32>& vertices, const std::vector<u8>* pinned, bool keepAspect,
        f32 margin);

// Box projection: each triangle takes the UV of the plane most facing it (`tile` repeats per world unit, plus `offset`). Vertices shared across planes or with outside triangles are duplicated and appended;
// `touched` gets every vertex used afterwards. Returns how many were added.
u32 boxMap(MeshData& mesh, const std::vector<u32>& triangles, f32 tile, const Math::vec2& offset,
           std::vector<u32>* touched = nullptr);

// Renders the UV layout as a `size` x `size` RGBA image over an optional square `background`. UV (0,0) is the top left so textures line up.
std::vector<u8> renderLayout(const MeshData& mesh, const std::vector<u32>& triangles, u32 size,
                             const std::vector<u8>& background, u32 backgroundSize);

} // namespace Radion::MeshUv

#endif // RADION_BLENDER_MESH_UV_H
