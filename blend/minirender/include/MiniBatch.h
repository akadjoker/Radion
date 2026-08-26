#ifndef RADION_MINI_BATCH_H
#define RADION_MINI_BATCH_H

#include "Types.h"
#include "Math.h"
#include "Math.h"
#include "Math.h"
#include <vector>

namespace Radion
{

 
class MiniBatch
{
public:
    MiniBatch();
    ~MiniBatch();

    MiniBatch(const MiniBatch&) = delete;
    MiniBatch& operator=(const MiniBatch&) = delete;

    bool initialize();
    void shutdown();

 
    void begin();

    void line(const Math::vec3& a, const Math::vec3& b, const Math::vec4& color);
    void point(const Math::vec3& p, const Math::vec4& color, f32 size = 6.0f);

    void triangle(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c, const Math::vec4& color);

    // Same shape as DebugDraw3D::grid() (runtime/render/src/DebugDraw3D.cpp:499-514):
    // `slices` lines each side of the origin, `spacing` apart, X line red and
    // Z line blue through the origin when axes is set. Ported as line() calls
    // since MiniBatch has no per-vertex-array grid primitive of its own.
    void grid(f32 y, u32 slices, f32 spacing, bool axes = true);
 
    void flush(const Math::mat4& viewProjection);

private:
    struct Vertex
    {
        Math::vec3 position;
        Math::vec4 color;
    };

    std::vector<Vertex> mTriangles;
    std::vector<Vertex> mLines;
    std::vector<Vertex> mPoints;
    f32 mPointSize = 6.0f;

    u32 mShaderProgram = 0;
    u32 mVAO = 0;
    u32 mVBO = 0;
    u32 mVBOCapacity = 0;

    bool compileShader();
};

} // namespace Radion

#endif // RADION_MINI_BATCH_H
