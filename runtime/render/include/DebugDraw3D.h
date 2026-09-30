#ifndef RADION_DEBUG_DRAW_3D_H
#define RADION_DEBUG_DRAW_3D_H

#include "Color.h"
#include "Math.h"
#include "Mesh.h"

#include <vector>

namespace Radion
{

class RenderTechnique;

struct DebugLine3D
{
    Math::vec3 from;
    Math::vec3 to;
    Color color;
    bool depthTest = true;
};

struct DebugTriangle3D
{
    Math::vec3 a;
    Math::vec3 b;
    Math::vec3 c;
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
    Math::mat4 transform = Math::mat4(1.0f);
    f32 length = 0.1f;
    u8 flags = 0;
};

struct DebugMeshOutline3D
{
    MeshHandle mesh;
    Math::mat4 transform = Math::mat4(1.0f);
    Color color = Color::Yellow;
    // Fraction the hull is scaled up around the mesh's local pivot (0.05 = 5%); not a pixel size.
    f32 thickness = 0.05f;

    // Index-buffer slice to outline; count 0 = whole mesh. Needed to pick one submesh out of a multi-submesh mesh.
    u32 indexOffset = 0;
    u32 indexCount = 0;
};

class DebugDraw3D
{
public:
    static DebugDraw3D& getSingleton();

    void clear();
    void line(const Math::vec3& from, const Math::vec3& to, Color color = Color::White,
              bool depthTest = true);
    void box(const AABB& bounds, Color color = Color::Green);
    void axis(const Math::mat4& transform, f32 size = 1.0f);
    void grid(f32 y, u32 slices, f32 spacing, bool axes = true);
    void triangle(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c, Color color,
                  bool filled = false);
    void pickedTriangle(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                        const Math::vec3& hit, f32 normalLength = 0.25f);
    void meshVectors(MeshHandle mesh, const Math::mat4& transform, f32 length, u8 flags);
    void outline(MeshHandle mesh, const Math::mat4& transform, Color color = Color::Yellow,
                 f32 thickness = 0.05f);

    void outlineRange(MeshHandle mesh, const Math::mat4& transform, u32 indexOffset, u32 indexCount,
                      Color color = Color::Yellow, f32 thickness = 0.05f);

    void pointLightGizmo(const Math::vec3& position, f32 range, Color color, u32 segments = 24);
    void spotLightGizmo(const Math::vec3& position, const Math::vec3& direction, f32 range,
                        f32 innerAngleDegrees, f32 outerAngleDegrees, Color color,
                        u32 segments = 24);
    void rectangleLightGizmo(const Math::vec3& position, const Math::vec3& direction, f32 width,
                             f32 height, Color color, f32 normalLength = 0.5f);
    void directionalLightGizmo(const Math::vec3& position, const Math::vec3& direction, Color color,
                               f32 radius = 0.6f, f32 rayLength = 1.5f, u32 rayCount = 6);
    void circle(const Math::vec3& center, const Math::vec3& u, const Math::vec3& v, f32 radius,
                u32 segments, Color color);

    // Three axis-aligned segments through a point; implies no facing.
    void cross(const Math::vec3& position, f32 size, Color color);
    void arrow(const Math::vec3& from, const Math::vec3& to, f32 headFrom, f32 headTo, Color color);
    // Segment bowed upward (`height` as a fraction of its length), for non-walk links such as jumps or teleports.
    void arc(const Math::vec3& from, const Math::vec3& to, f32 height, f32 headFrom, f32 headTo,
             Color color);
    void arrowHead(const Math::vec3& from, const Math::vec3& to, f32 size, Color color);
    void cursor3D(const Math::vec3& position, const Math::vec3& cameraRight,
                  const Math::vec3& cameraUp, f32 radius);

    const std::vector<DebugLine3D>& lines() const;
    const std::vector<DebugTriangle3D>& triangles() const;
    const std::vector<DebugMeshVectors3D>& meshVectorCommands() const;
    const std::vector<DebugMeshOutline3D>& outlineCommands() const;
    bool empty() const;

private:
    DebugDraw3D();

    void spotCone(const Math::vec3& apex, const Math::vec3& endCenter, const Math::vec3& right,
                  const Math::vec3& up, f32 range, f32 angleDegrees, u32 segments, u32 ribs,
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
