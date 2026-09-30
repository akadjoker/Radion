#ifndef RADION_BLENDER_MESH_UV_H
#define RADION_BLENDER_MESH_UV_H

#include "Mesh.h"
#include "Types.h"

#include <vector>

namespace Radion::MeshUv
{

// Texture-coordinate editing on a triangle MeshData: pure functions, no engine,
// no GL. UVs belong to vertices (MeshData::uvs), so an edit names VERTEX
// indices, not canonical points: two vertices standing at the same place can
// hold different UVs (a seam) and must be movable independently.
//
// `pinned` (optional, one byte per vertex, non-zero = pinned) makes the edit
// skip those vertices - the UV panel's "pin".

struct Rect
{
    glm::vec2 min = glm::vec2(0.0f);
    glm::vec2 max = glm::vec2(0.0f);
    bool valid = false;

    glm::vec2 center() const
    {
        return (min + max) * 0.5f;
    }
    glm::vec2 size() const
    {
        return max - min;
    }
};

// Gives the mesh a UV array in step with its positions (zeros) if it lacks one.
void ensureUvs(MeshData& mesh);

// Unique vertex indices the listed triangles use, ascending. Triangles out of
// range are ignored.
std::vector<u32> verticesOfTriangles(const MeshData& mesh, const std::vector<u32>& triangles);

// Connected pieces of UV space: triangles that share a VERTEX (not just a point)
// are one island, so a seam separates islands. Returns the island of every
// triangle, numbered from 0 in order of first appearance; `count` gets how many.
std::vector<u32> islands(const MeshData& mesh, u32* count = nullptr);

// Every vertex of every island that any of `triangles` belongs to.
std::vector<u32> islandVertices(const MeshData& mesh, const std::vector<u32>& triangles);

Rect bounds(const MeshData& mesh, const std::vector<u32>& vertices);

struct Transform
{
    glm::vec2 translate = glm::vec2(0.0f);
    // Counter-clockwise as drawn in the UV layout (u right, v DOWN: UV (0,0) is the top
    // left of the texture, as in glTF and the engine's own loaders).
    f32 rotateDegrees = 0.0f;
    glm::vec2 scale = glm::vec2(1.0f); // negative flips
};

// Scales and rotates about `pivot`, then translates. Returns how many vertices
// moved (pinned ones do not).
u32 transform(MeshData& mesh, const std::vector<u32>& vertices, const std::vector<u8>* pinned,
              const glm::vec2& pivot, const Transform& change);

// Moves and scales the vertices' bounds into the 0..1 square, leaving `margin`
// (a fraction of the square) free all round. With `keepAspect` the layout keeps
// its shape and is centred; otherwise it is stretched to fill.
u32 fit(MeshData& mesh, const std::vector<u32>& vertices, const std::vector<u8>* pinned, bool keepAspect,
        f32 margin);

// Box projection of the listed triangles: each takes the UV of the plane most
// facing it (three axes, both signs). `tile` is texture repeats per world unit
// and `offset` is added. A vertex used by triangles of different planes - or by
// triangles outside the list - is duplicated so each keeps its own UV. The new
// vertices are appended; `touched` gets every vertex the listed triangles use
// afterwards. Returns how many vertices were added.
u32 boxMap(MeshData& mesh, const std::vector<u32>& triangles, f32 tile, const glm::vec2& offset,
           std::vector<u32>* touched = nullptr);

// The UV layout of `triangles` as an RGBA image `size` x `size`: edges in a
// contrasting colour over a faint 0..1 frame, over `background` (a square image,
// RGBA, `backgroundSize` pixels wide; may be empty) when there is one. UV (0,0) is
// the top left, so a texture drawn upright lines up with the triangles.
std::vector<u8> renderLayout(const MeshData& mesh, const std::vector<u32>& triangles, u32 size,
                             const std::vector<u8>& background, u32 backgroundSize);

} // namespace Radion::MeshUv

#endif // RADION_BLENDER_MESH_UV_H
