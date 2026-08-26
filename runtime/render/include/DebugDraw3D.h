#ifndef RADION_DEBUG_DRAW_3D_H
#define RADION_DEBUG_DRAW_3D_H

#include "Color.h"
#include "Math.h"
#include "Mesh.h"

#include <glm/glm.hpp>
#include <vector>

namespace Radion
{

class RenderTechnique;

struct DebugLine3D
{
    glm::vec3 from;
    glm::vec3 to;
    Color color;
    bool depthTest = true;
};

struct DebugTriangle3D
{
    glm::vec3 a;
    glm::vec3 b;
    glm::vec3 c;
    Color color;
};

enum DebugMeshVectorFlags : u8
{
    DebugVectorsNormal = 1 << 0,
    DebugVectorsTangent = 1 << 1
};

struct DebugMeshVectors3D
{
    MeshHandle mesh;
    glm::mat4 transform = glm::mat4(1.0f);
    f32 length = 0.1f;
    u8 flags = 0;
};

struct DebugMeshOutline3D
{
    MeshHandle mesh;
    glm::mat4 transform = glm::mat4(1.0f);
    Color color = Color::Yellow;
    // Fraction the hull is scaled up by around the mesh's own local pivot for
    // the second (visible) pass - 0.05 means 5% bigger. Not a pixel size:
    // the outline's on-screen width scales with the mesh's own screen size.
    f32 thickness = 0.05f;

    // Which slice of the index buffer to outline. A count of 0 means the whole
    // mesh, which is what a single-object outline wants. Anything picked out of
    // a model that is one mesh with many submeshes needs the range instead -
    // outlining all of Sponza to show which arch was clicked says nothing.
    u32 indexOffset = 0;
    u32 indexCount = 0;
};

class DebugDraw3D
{
public:
    static DebugDraw3D& getSingleton();

    void clear();
    void line(const glm::vec3& from, const glm::vec3& to, Color color = Color::White,
              bool depthTest = true);
    void box(const AABB& bounds, Color color = Color::Green);
    void axis(const glm::mat4& transform, f32 size = 1.0f);
    void grid(f32 y, u32 slices, f32 spacing, bool axes = true);
    void triangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, Color color,
                  bool filled = false);
    void pickedTriangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                        const glm::vec3& hit, f32 normalLength = 0.25f);
    void meshVectors(MeshHandle mesh, const glm::mat4& transform, f32 length, u8 flags);
    void outline(MeshHandle mesh, const glm::mat4& transform, Color color = Color::Yellow,
                 f32 thickness = 0.05f);

    // One submesh of a larger mesh - see DebugMeshOutline3D::indexCount.
    void outlineRange(MeshHandle mesh, const glm::mat4& transform, u32 indexOffset, u32 indexCount,
                      Color color = Color::Yellow, f32 thickness = 0.05f);

    // Light gizmos, wireframe shapes built from line(). Each takes the
    // light's own colour so several lights stay tellable apart, and a
    // segment count for the curved parts rather than a fixed tessellation.
    void pointLightGizmo(const glm::vec3& position, f32 range, Color color, u32 segments = 24);
    void spotLightGizmo(const glm::vec3& position, const glm::vec3& direction, f32 range,
                        f32 innerAngleDegrees, f32 outerAngleDegrees, Color color,
                        u32 segments = 24);
    void rectangleLightGizmo(const glm::vec3& position, const glm::vec3& direction, f32 width,
                             f32 height, Color color, f32 normalLength = 0.5f);
    void directionalLightGizmo(const glm::vec3& position, const glm::vec3& direction, Color color,
                               f32 radius = 0.6f, f32 rayLength = 1.5f, u32 rayCount = 6);
    void circle(const glm::vec3& center, const glm::vec3& u, const glm::vec3& v, f32 radius,
                u32 segments, Color color);

    // Three axis-aligned segments through a point - marks a position without
    // implying a facing the way an axis gizmo does.
    void cross(const glm::vec3& position, f32 size, Color color);
    // A straight segment with an arrow head on either end (size 0 for none),
    // so a link reads as directed.
    void arrow(const glm::vec3& from, const glm::vec3& to, f32 headFrom, f32 headTo, Color color);
    // A segment bowed upward, `height` as a fraction of its own length. What
    // draws a connection between two points that is NOT a walk along the
    // ground - a jump, a ladder, a teleport - so it cannot be mistaken for
    // one that is.
    void arc(const glm::vec3& from, const glm::vec3& to, f32 height, f32 headFrom, f32 headTo,
             Color color);
    void arrowHead(const glm::vec3& from, const glm::vec3& to, f32 size, Color color);
    void cursor3D(const glm::vec3& position, const glm::vec3& cameraRight,
                  const glm::vec3& cameraUp, f32 radius);

    const std::vector<DebugLine3D>& lines() const;
    const std::vector<DebugTriangle3D>& triangles() const;
    const std::vector<DebugMeshVectors3D>& meshVectorCommands() const;
    const std::vector<DebugMeshOutline3D>& outlineCommands() const;
    bool empty() const;

private:
    DebugDraw3D();

    // One of spotLightGizmo()'s two cones: a ring at the end plus `ribs`
    // lines back to the apex.
    void spotCone(const glm::vec3& apex, const glm::vec3& endCenter, const glm::vec3& right,
                  const glm::vec3& up, f32 range, f32 angleDegrees, u32 segments, u32 ribs,
                  Color color);

    std::vector<DebugLine3D> mLines;
    std::vector<DebugTriangle3D> mTriangles;
    std::vector<DebugMeshVectors3D> mMeshVectors;
    std::vector<DebugMeshOutline3D> mOutlines;
};

inline DebugDraw3D& DebugDraw()
{
    return DebugDraw3D::getSingleton();
}

// The debug pass is renderer-owned implementation, not public engine state.
RenderTechnique* createDebugPass();

} // namespace Radion

#endif // RADION_DEBUG_DRAW_3D_H
