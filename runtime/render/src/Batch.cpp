#include "PCH.h"

#include "Batch.h"

#include "font_data.h"

#include <SDL2/SDL_timer.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace Radion
{

namespace
{

double getTimeMilliseconds()
{
    return static_cast<double>(SDL_GetPerformanceCounter()) * 1000.0 /
           static_cast<double>(SDL_GetPerformanceFrequency());
}

} // namespace

// Font atlas constants
static const int FONT_COLS = 16;
static const int FONT_ATLAS_W = 128; // 16 cols * 8px
static const int FONT_ATLAS_H = 48;  // 6 rows * 8px

static const float PI = 3.14159265359f;
static const float DEG2RAD = PI / 180.0f;

// ---------------------------------------------------------------------------
// Shader sources. Desktop GLSL 330 core; for GLES/WebGL the sources would need
// a different #version/precision preamble, but this matches the OpenGL 3.3 core
// context created by the Radion Window.
// ---------------------------------------------------------------------------
static const char* VERTEX_SHADER_SOURCE = R"(#version 450 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec2 a_texcoord;
layout(location = 2) in vec4 a_color;

layout(std140, binding = 0) uniform BatchUniforms
{
    mat4 u_mvp;
};

out vec2 v_texcoord;
out vec4 v_color;

void main()
{
    gl_Position = u_mvp * vec4(a_position, 1.0);
    v_texcoord = a_texcoord;
    v_color = a_color;
}
)";

static const char* FRAGMENT_SHADER_SOURCE = R"(#version 450 core
in vec2 v_texcoord;
in vec4 v_color;

layout(binding = 0) uniform sampler2D u_texture;

out vec4 FragColor;

void main()
{
    vec4 texColor = texture(u_texture, v_texcoord);
    FragColor = texColor * v_color;
}
)";

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

BatchRenderer::BatchRenderer()
    : mCurrentColor(0xFFFFFFFF), mCurrentMode(ModeTriangles), mInBeginEnd(false),
      mCurrentBlendMode(BlendMode::Alpha), mWindowWidth(800), mWindowHeight(600),
      mDepthTestEnabled(false), mDepthWriteEnabled(false), mBlendEnabled(true),
      mCullFaceEnabled(false), mClipEnabled(false), mGpu(nullptr)
{
    memset(&mStats, 0, sizeof(mStats));
    memset(mCurrentTexcoord, 0, sizeof(mCurrentTexcoord));
    mProjection = glm::mat4(1.0f);
    mCurrentMatrix = glm::mat4(1.0f);
}

BatchRenderer::~BatchRenderer()
{
    shutdown();
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

bool BatchRenderer::init(const Config& config)
{
    mConfig = config;

    // Reserve capacity for vertex data
    mVertices.reserve(mConfig.maxVertices);
    mIndices.reserve(mConfig.maxVertices * 2); // Rough estimate
    mDrawCalls.reserve(mConfig.maxDrawCalls);

    // Reserve matrix stack
    mMatrixStack.reserve(mConfig.stackDepth);

    mGpu = &GPU::getSingleton();

    setupBuffers();
    setupTexture();
    setupFontTexture();

    // Setup projection
    updateProjection();

    resetStats();

    printf("BatchRenderer initialized: maxVertices=%zu, maxDrawCalls=%zu\n", mConfig.maxVertices,
           mConfig.maxDrawCalls);

    return true;
}

void BatchRenderer::shutdown()
{
    if (!mGpu)
        return;

    // Never dereference the cached device after Engine::shutdown(). Normal
    // renderer teardown reaches here while it is alive; this guard also
    // makes a later/destructor cleanup harmless if ownership order changes.
    GPU* gpu = GPU::tryGet();
    if (gpu == mGpu)
    {
        for (usize i = 0; i < mPipelines.size(); ++i)
            gpu->destroy(mPipelines[i].pipeline);

        gpu->destroy(mVertexBuffer);
        gpu->destroy(mIndexBuffer);
        gpu->destroy(mUniformBuffer);
        gpu->destroy(mWhiteTexture);
        gpu->destroy(mFontTexture);
        gpu->destroy(mSampler);
    }
    mPipelines.clear();

    mVertexBuffer = BufferHandle();
    mIndexBuffer = BufferHandle();
    mUniformBuffer = BufferHandle();
    mWhiteTexture = TextureHandle();
    mFontTexture = TextureHandle();
    mSampler = SamplerHandle();
    mGpu = nullptr;

    mVertices.clear();
    mIndices.clear();
    mDrawCalls.clear();
    mMatrixStack.clear();
}

// ---------------------------------------------------------------------------
// Window management
// ---------------------------------------------------------------------------

bool BatchRenderer::resize(int width, int height)
{
    if (width <= 0 || height <= 0)
        return false;

    mWindowWidth = width;
    mWindowHeight = height;
    updateProjection();

    return true;
}

void BatchRenderer::getWindowSize(int& width, int& height) const
{
    width = mWindowWidth;
    height = mWindowHeight;
}

// ---------------------------------------------------------------------------
// Transform stack (like rlgl)
// ---------------------------------------------------------------------------

void BatchRenderer::pushMatrix()
{
    if (mMatrixStack.size() < mConfig.stackDepth)
    {
        mMatrixStack.push_back(mCurrentMatrix);
    }
}

void BatchRenderer::popMatrix()
{
    if (!mMatrixStack.empty())
    {
        mCurrentMatrix = mMatrixStack.back();
        mMatrixStack.pop_back();
    }
    else
    {
        mCurrentMatrix = glm::mat4(1.0f);
    }
}

void BatchRenderer::loadIdentity()
{
    mCurrentMatrix = glm::mat4(1.0f);
}

void BatchRenderer::translate(float x, float y, float z)
{
    mCurrentMatrix = glm::translate(mCurrentMatrix, glm::vec3(x, y, z));
}

void BatchRenderer::rotate(float angleDeg, float axisX, float axisY, float axisZ)
{
    glm::vec3 axis(axisX, axisY, axisZ);
    float angleRad = angleDeg * (Pi / 180.0f);
    mCurrentMatrix = glm::rotate(mCurrentMatrix, angleRad, axis);
}

void BatchRenderer::scale(float x, float y, float z)
{
    mCurrentMatrix = glm::scale(mCurrentMatrix, glm::vec3(x, y, z));
}

// ---------------------------------------------------------------------------
// State management
// ---------------------------------------------------------------------------

void BatchRenderer::setColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
    mCurrentColor = packColor(r, g, b, a);
}

void BatchRenderer::setColor(float r, float g, float b, float a)
{
    mCurrentColor = packColor(static_cast<unsigned char>(r * 255.0f + 0.5f),
                              static_cast<unsigned char>(g * 255.0f + 0.5f),
                              static_cast<unsigned char>(b * 255.0f + 0.5f),
                              static_cast<unsigned char>(a * 255.0f + 0.5f));
}

void BatchRenderer::setTexture(TextureHandle texture)
{
    if (mCurrentTexture != texture)
    {
        mCurrentTexture = texture;
        if (mConfig.enableProfiling)
        {
            mStats.textureSwitches++;
        }
    }
}

void BatchRenderer::setBlendMode(BlendMode mode)
{
    mCurrentBlendMode = mode;
}

void BatchRenderer::setTexcoord(float u, float v)
{
    mCurrentTexcoord[0] = u;
    mCurrentTexcoord[1] = v;
}

void BatchRenderer::setDefault3DState()
{
    mDepthTestEnabled = true;
    mDepthWriteEnabled = true;
    mBlendEnabled = false;
    mCullFaceEnabled = false;
}

// ---------------------------------------------------------------------------
// Soft clip
// ---------------------------------------------------------------------------

void BatchRenderer::setClipRect(float x, float y, float width, float height)
{
    mClipRect = FloatRect{x, y, width, height};
    mClipEnabled = true;
}

void BatchRenderer::setClipRect(const FloatRect& rect)
{
    mClipRect = rect;
    mClipEnabled = true;
}

void BatchRenderer::clearClipRect()
{
    mClipEnabled = false;
}

// Sutherland-Hodgman: clips a convex polygon (with UV) against the current
// clip rect, one half-plane at a time. `out` must hold at least inCount+4
// vertices (each pass can add at most one vertex per edge crossed). Returns
// the number of vertices written to `out` (0 if fully clipped away).
int BatchRenderer::clipPolygonToRect(const ClipVertex* in, int inCount, ClipVertex* out) const
{
    const float minX = mClipRect.x;
    const float minY = mClipRect.y;
    const float maxX = mClipRect.x + mClipRect.width;
    const float maxY = mClipRect.y + mClipRect.height;

    ClipVertex tmp[16];
    const ClipVertex* src = in;
    int srcCount = inCount;

    // Each plane clips `src` -> `dst`, then dst becomes the next src.
    for (int plane = 0; plane < 4; ++plane)
    {
        ClipVertex* dst = (plane % 2 == 0) ? tmp : out;
        int dstCount = 0;

        for (int i = 0; i < srcCount; ++i)
        {
            const ClipVertex& cur = src[i];
            const ClipVertex& prev = src[(i + srcCount - 1) % srcCount];

            bool curIn = false, prevIn = false;
            switch (plane)
            {
            case 0:
                curIn = cur.x >= minX;
                prevIn = prev.x >= minX;
                break; // left
            case 1:
                curIn = cur.x <= maxX;
                prevIn = prev.x <= maxX;
                break; // right
            case 2:
                curIn = cur.y >= minY;
                prevIn = prev.y >= minY;
                break; // top
            case 3:
                curIn = cur.y <= maxY;
                prevIn = prev.y <= maxY;
                break; // bottom
            }

            if (curIn != prevIn)
            {
                // Edge crosses the plane — interpolate the intersection point.
                float t = 0.f;
                switch (plane)
                {
                case 0:
                    t = (minX - prev.x) / (cur.x - prev.x);
                    break;
                case 1:
                    t = (maxX - prev.x) / (cur.x - prev.x);
                    break;
                case 2:
                    t = (minY - prev.y) / (cur.y - prev.y);
                    break;
                case 3:
                    t = (maxY - prev.y) / (cur.y - prev.y);
                    break;
                }
                ClipVertex isect;
                isect.x = prev.x + t * (cur.x - prev.x);
                isect.y = prev.y + t * (cur.y - prev.y);
                isect.u = prev.u + t * (cur.u - prev.u);
                isect.v = prev.v + t * (cur.v - prev.v);
                if (dstCount < 16)
                    dst[dstCount++] = isect;
            }
            if (curIn)
            {
                if (dstCount < 16)
                    dst[dstCount++] = cur;
            }
        }

        src = dst;
        srcCount = dstCount;
        if (srcCount == 0)
            return 0;
    }

    // If we ended on `tmp` (odd number of planes before the last), copy to out.
    if (src == tmp)
    {
        for (int i = 0; i < srcCount; ++i)
            out[i] = tmp[i];
    }
    return srcCount;
}

// Liang-Barsky line-clip against the current clip rect. Returns false if the
// segment lies entirely outside (nothing to draw).
bool BatchRenderer::clipSegmentToRect(float& x0, float& y0, float& x1, float& y1) const
{
    const float minX = mClipRect.x;
    const float minY = mClipRect.y;
    const float maxX = mClipRect.x + mClipRect.width;
    const float maxY = mClipRect.y + mClipRect.height;

    float dx = x1 - x0;
    float dy = y1 - y0;
    float tMin = 0.f, tMax = 1.f;

    float p[4] = {-dx, dx, -dy, dy};
    float q[4] = {x0 - minX, maxX - x0, y0 - minY, maxY - y0};

    for (int i = 0; i < 4; ++i)
    {
        if (p[i] == 0.f)
        {
            if (q[i] < 0.f)
                return false; // parallel and outside
        }
        else
        {
            float t = q[i] / p[i];
            if (p[i] < 0.f)
                tMin = Max(tMin, t);
            else
                tMax = Min(tMax, t);
        }
    }

    if (tMin > tMax)
        return false;

    float nx0 = x0 + tMin * dx, ny0 = y0 + tMin * dy;
    float nx1 = x0 + tMax * dx, ny1 = y0 + tMax * dy;
    x0 = nx0;
    y0 = ny0;
    x1 = nx1;
    y1 = ny1;
    return true;
}

void BatchRenderer::submitClippedQuad(float x0, float y0, float x1, float y1, float x2, float y2,
                                      float x3, float y3)
{
    ClipVertex in[4] = {
        {x0, y0, 0.f, 0.f},
        {x1, y1, 1.f, 0.f},
        {x2, y2, 1.f, 1.f},
        {x3, y3, 0.f, 1.f},
    };
    ClipVertex out[16];
    int n = clipPolygonToRect(in, 4, out);
    if (n < 3)
        return;

    // Fan-triangulate the resulting convex polygon.
    for (int i = 1; i + 1 < n; ++i)
    {
        submitVertex(out[0].x, out[0].y, 0.f, out[0].u, out[0].v);
        submitVertex(out[i].x, out[i].y, 0.f, out[i].u, out[i].v);
        submitVertex(out[i + 1].x, out[i + 1].y, 0.f, out[i + 1].u, out[i + 1].v);
    }
}

void BatchRenderer::submitClippedLine(float x0, float y0, float x1, float y1)
{
    if (!clipSegmentToRect(x0, y0, x1, y1))
        return;
    submitVertex(x0, y0, 0.f);
    submitVertex(x1, y1, 0.f);
}

void BatchRenderer::emitLine(float x0, float y0, float x1, float y1)
{
    if (mClipEnabled)
    {
        float z0 = 0.f, z1 = 0.f;
        applyTransform(x0, y0, z0);
        applyTransform(x1, y1, z1);
        submitClippedLine(x0, y0, x1, y1);
    }
    else
    {
        vertex2(x0, y0);
        vertex2(x1, y1);
    }
}

void BatchRenderer::emitTriangle(float x0, float y0, float x1, float y1, float x2, float y2)
{
    emitTexturedTriangle(x0, y0, 0.f, 0.f, x1, y1, 0.f, 0.f, x2, y2, 0.f, 0.f);
}

void BatchRenderer::emitTexturedTriangle(float x0, float y0, float u0, float v0, float x1, float y1,
                                         float u1, float v1, float x2, float y2, float u2, float v2)
{
    if (mClipEnabled)
    {
        float z0 = 0.f, z1 = 0.f, z2 = 0.f;
        applyTransform(x0, y0, z0);
        applyTransform(x1, y1, z1);
        applyTransform(x2, y2, z2);

        ClipVertex in[3] = {{x0, y0, u0, v0}, {x1, y1, u1, v1}, {x2, y2, u2, v2}};
        ClipVertex out[16];
        int n = clipPolygonToRect(in, 3, out);
        for (int i = 1; i + 1 < n; ++i)
        {
            submitVertex(out[0].x, out[0].y, 0.f, out[0].u, out[0].v);
            submitVertex(out[i].x, out[i].y, 0.f, out[i].u, out[i].v);
            submitVertex(out[i + 1].x, out[i + 1].y, 0.f, out[i + 1].u, out[i + 1].v);
        }
    }
    else
    {
        setTexcoord(u0, v0);
        vertex2(x0, y0);
        setTexcoord(u1, v1);
        vertex2(x1, y1);
        setTexcoord(u2, v2);
        vertex2(x2, y2);
    }
}

// ---------------------------------------------------------------------------
// Immediate mode vertex submission
// ---------------------------------------------------------------------------

void BatchRenderer::vertex2(float x, float y)
{
    vertex3(x, y, 0.0f);
}

void BatchRenderer::vertex3(float x, float y, float z)
{
    if (!mInBeginEnd)
        return;

    // Apply current transform
    applyTransform(x, y, z);

    // Add vertex
    Vertex v;
    v.x = x;
    v.y = y;
    v.z = z;
    v.u = mCurrentTexcoord[0];
    v.v = mCurrentTexcoord[1];
    unpackColor(mCurrentColor, v.r, v.g, v.b, v.a);

    mVertices.push_back(v);

    // Check capacity and flush if needed
    if (mVertices.size() >= mConfig.maxVertices)
    {
        flushBatch();
    }
}

// ---------------------------------------------------------------------------
// Drawing primitives (immediate mode style)
// ---------------------------------------------------------------------------

void BatchRenderer::begin(int mode)
{
    if (mInBeginEnd)
    {
        printf("Warning: BatchRenderer::begin() called without end()\n");
        end();
    }

    mInBeginEnd = true;
    mCurrentMode = mode;

    // Start new draw call if mode, texture, or render state changed since the
    // last one — this makes state flag changes (setDepthTest/setBlend/...)
    // take effect immediately per draw call, regardless of flush timing.
    if (mDrawCalls.empty() || mDrawCalls.back().mode != mode ||
        mDrawCalls.back().texture != mCurrentTexture ||
        mDrawCalls.back().blendMode != mCurrentBlendMode ||
        mDrawCalls.back().depthTest != mDepthTestEnabled ||
        mDrawCalls.back().depthWrite != mDepthWriteEnabled ||
        mDrawCalls.back().blend != mBlendEnabled || mDrawCalls.back().cullFace != mCullFaceEnabled)
    {
        DrawCall call;
        call.mode = mode;
        call.vertexCount = 0;
        call.vertexAlignment = mVertices.size(); // vertices before this draw call
        call.texture = mCurrentTexture;
        call.blendMode = mCurrentBlendMode;
        call.depthTest = mDepthTestEnabled;
        call.depthWrite = mDepthWriteEnabled;
        call.blend = mBlendEnabled;
        call.cullFace = mCullFaceEnabled;
        mDrawCalls.push_back(call);
    }
}

void BatchRenderer::end()
{
    if (!mInBeginEnd)
        return;

    mInBeginEnd = false;

    // update current draw call vertex count
    if (!mDrawCalls.empty())
    {
        mDrawCalls.back().vertexCount = mVertices.size() - mDrawCalls.back().vertexAlignment;
    }
}

void BatchRenderer::drawLine(float x0, float y0, float x1, float y1)
{
    setTexture(mWhiteTexture);
    begin(ModeLines);
    if (mClipEnabled)
    {
        float z = 0.f;
        applyTransform(x0, y0, z);
        z = 0.f;
        applyTransform(x1, y1, z);
        submitClippedLine(x0, y0, x1, y1);
    }
    else
    {
        vertex2(x0, y0);
        vertex2(x1, y1);
    }
    end();
}

void BatchRenderer::drawTriangle(float x1, float y1, float x2, float y2, float x3, float y3)
{
    setTexture(mWhiteTexture);
    begin(ModeTriangles);
    emitTriangle(x1, y1, x2, y2, x3, y3);
    end();
}

void BatchRenderer::drawRect(float x, float y, float width, float height, bool fill)
{
    setTexture(mWhiteTexture);
    if (fill)
    {
        begin(ModeTriangles);
        if (mClipEnabled)
        {
            float x0 = x, y0 = y, z0 = 0.f;
            float x1 = x + width, y1 = y, z1 = 0.f;
            float x2 = x + width, y2 = y + height, z2 = 0.f;
            float x3 = x, y3 = y + height, z3 = 0.f;
            applyTransform(x0, y0, z0);
            applyTransform(x1, y1, z1);
            applyTransform(x2, y2, z2);
            applyTransform(x3, y3, z3);
            submitClippedQuad(x0, y0, x1, y1, x2, y2, x3, y3);
        }
        else
        {
            vertex2(x, y);
            vertex2(x + width, y);
            vertex2(x, y + height);
            vertex2(x + width, y);
            vertex2(x + width, y + height);
            vertex2(x, y + height);
        }
        end();
    }
    else
    {
        begin(ModeLines);
        if (mClipEnabled)
        {
            float corners[4][2] = {
                {x, y}, {x + width, y}, {x + width, y + height}, {x, y + height}};
            for (int i = 0; i < 4; ++i)
            {
                float x0 = corners[i][0], y0 = corners[i][1], z0 = 0.f;
                float x1 = corners[(i + 1) % 4][0], y1 = corners[(i + 1) % 4][1], z1 = 0.f;
                applyTransform(x0, y0, z0);
                applyTransform(x1, y1, z1);
                submitClippedLine(x0, y0, x1, y1);
            }
        }
        else
        {
            vertex2(x, y);
            vertex2(x + width, y);
            vertex2(x + width, y);
            vertex2(x + width, y + height);
            vertex2(x + width, y + height);
            vertex2(x, y + height);
            vertex2(x, y + height);
            vertex2(x, y);
        }
        end();
    }
}

void BatchRenderer::drawQuad(float x, float y, float width, float height)
{
    setTexture(mWhiteTexture);
    begin(ModeQuads);
    vertex2(x, y);
    vertex2(x + width, y);
    vertex2(x + width, y + height);
    vertex2(x, y + height);
    end();
}

void BatchRenderer::drawCircle(float cx, float cy, float radius, int segments)
{
    setTexture(mWhiteTexture);
    begin(ModeLines);
    for (int i = 0; i < segments; ++i)
    {
        float angle1 = (float)i * 2.0f * PI / (float)segments;
        float angle2 = (float)(i + 1) * 2.0f * PI / (float)segments;

        float x1 = cx + cosf(angle1) * radius;
        float y1 = cy + sinf(angle1) * radius;
        float x2 = cx + cosf(angle2) * radius;
        float y2 = cy + sinf(angle2) * radius;

        emitLine(x1, y1, x2, y2);
    }
    end();
}

void BatchRenderer::drawPolyline(const float* xyPairs, int pointCount)
{
    if (!xyPairs || pointCount < 2)
        return;

    setTexture(mWhiteTexture);
    begin(ModeLines);
    for (int i = 0; i < pointCount - 1; ++i)
    {
        emitLine(xyPairs[i * 2], xyPairs[i * 2 + 1], xyPairs[(i + 1) * 2],
                 xyPairs[(i + 1) * 2 + 1]);
    }
    end();
}

void BatchRenderer::drawThickLine(float x0, float y0, float x1, float y1, float thickness)
{
    float dx = x1 - x0, dy = y1 - y0;
    float len = sqrtf(dx * dx + dy * dy);
    if (len <= 0.0f)
        return;
    float nx = -dy / len * thickness * 0.5f;
    float ny = dx / len * thickness * 0.5f;
    setTexture(mWhiteTexture);
    begin(ModeTriangles);
    emitTriangle(x0 + nx, y0 + ny, x0 - nx, y0 - ny, x1 - nx, y1 - ny);
    emitTriangle(x0 + nx, y0 + ny, x1 - nx, y1 - ny, x1 + nx, y1 + ny);
    end();
}

void BatchRenderer::drawEllipse(float cx, float cy, float rx, float ry, bool fill, int segments)
{
    if (segments < 3)
        segments = 3;
    float step = 2.0f * PI / (float)segments;
    setTexture(mWhiteTexture);
    if (fill)
    {
        begin(ModeTriangles);
        for (int i = 0; i < segments; ++i)
        {
            float a0 = i * step, a1 = (i + 1) * step;
            emitTriangle(cx, cy, cx + rx * cosf(a0), cy + ry * sinf(a0), cx + rx * cosf(a1),
                         cy + ry * sinf(a1));
        }
        end();
    }
    else
    {
        begin(ModeLines);
        for (int i = 0; i < segments; ++i)
        {
            float a0 = i * step, a1 = (i + 1) * step;
            emitLine(cx + rx * cosf(a0), cy + ry * sinf(a0), cx + rx * cosf(a1),
                     cy + ry * sinf(a1));
        }
        end();
    }
}

void BatchRenderer::drawArc(float cx, float cy, float radius, float startDeg, float endDeg,
                            int segments)
{
    if (segments < 1)
        segments = 1;
    float a0 = startDeg * DEG2RAD;
    float step = (endDeg - startDeg) * DEG2RAD / (float)segments;
    setTexture(mWhiteTexture);
    begin(ModeLines);
    for (int i = 0; i < segments; ++i)
    {
        float t0 = a0 + i * step, t1 = a0 + (i + 1) * step;
        emitLine(cx + radius * cosf(t0), cy + radius * sinf(t0), cx + radius * cosf(t1),
                 cy + radius * sinf(t1));
    }
    end();
}

void BatchRenderer::drawRing(float cx, float cy, float rInner, float rOuter, bool fill,
                             int segments)
{
    if (segments < 3)
        segments = 3;
    float step = 2.0f * PI / (float)segments;
    setTexture(mWhiteTexture);
    if (fill)
    {
        begin(ModeTriangles);
        for (int i = 0; i < segments; ++i)
        {
            float a0 = i * step, a1 = (i + 1) * step;
            float ix0 = cx + rInner * cosf(a0), iy0 = cy + rInner * sinf(a0);
            float ox0 = cx + rOuter * cosf(a0), oy0 = cy + rOuter * sinf(a0);
            float ix1 = cx + rInner * cosf(a1), iy1 = cy + rInner * sinf(a1);
            float ox1 = cx + rOuter * cosf(a1), oy1 = cy + rOuter * sinf(a1);
            emitTriangle(ix0, iy0, ox0, oy0, ox1, oy1);
            emitTriangle(ix0, iy0, ox1, oy1, ix1, iy1);
        }
        end();
    }
    else
    {
        drawCircle(cx, cy, rInner, segments);
        drawCircle(cx, cy, rOuter, segments);
    }
}

void BatchRenderer::drawPolygon(float cx, float cy, int sides, float radius, float rotDeg,
                                bool fill)
{
    if (sides < 3)
        sides = 3;
    float step = 2.0f * PI / (float)sides;
    float rot = rotDeg * DEG2RAD;
    setTexture(mWhiteTexture);
    if (fill)
    {
        begin(ModeTriangles);
        for (int i = 0; i < sides; ++i)
        {
            float a0 = rot + i * step, a1 = rot + (i + 1) * step;
            emitTriangle(cx, cy, cx + radius * cosf(a0), cy + radius * sinf(a0),
                         cx + radius * cosf(a1), cy + radius * sinf(a1));
        }
        end();
    }
    else
    {
        begin(ModeLines);
        for (int i = 0; i < sides; ++i)
        {
            float a0 = rot + i * step, a1 = rot + (i + 1) * step;
            emitLine(cx + radius * cosf(a0), cy + radius * sinf(a0), cx + radius * cosf(a1),
                     cy + radius * sinf(a1));
        }
        end();
    }
}

// ---------------------------------------------------------------------------
// 3D debug primitives
// ---------------------------------------------------------------------------

void BatchRenderer::drawLine3D(float x0, float y0, float z0, float x1, float y1, float z1)
{
    setTexture(mWhiteTexture);
    begin(ModeLines);
    vertex3(x0, y0, z0);
    vertex3(x1, y1, z1);
    end();
}

void BatchRenderer::drawTriangle3D(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
{
    begin(ModeTriangles);
    vertex3(a.x, a.y, a.z);
    vertex3(b.x, b.y, b.z);
    vertex3(c.x, c.y, c.z);
    end();
}

void BatchRenderer::drawTriangle3D(const glm::vec3& a, const glm::vec2& uvA, const glm::vec3& b,
                                   const glm::vec2& uvB, const glm::vec3& c, const glm::vec2& uvC)
{
    begin(ModeTriangles);
    setTexcoord(uvA.x, uvA.y);
    vertex3(a.x, a.y, a.z);
    setTexcoord(uvB.x, uvB.y);
    vertex3(b.x, b.y, b.z);
    setTexcoord(uvC.x, uvC.y);
    vertex3(c.x, c.y, c.z);
    end();
}

void BatchRenderer::drawTriangle3D(const glm::vec3& a, const glm::vec2& uvA, u32 colorA,
                                   const glm::vec3& b, const glm::vec2& uvB, u32 colorB,
                                   const glm::vec3& c, const glm::vec2& uvC, u32 colorC)
{
    begin(ModeTriangles);
    mCurrentColor = colorA;
    setTexcoord(uvA.x, uvA.y);
    vertex3(a.x, a.y, a.z);
    mCurrentColor = colorB;
    setTexcoord(uvB.x, uvB.y);
    vertex3(b.x, b.y, b.z);
    mCurrentColor = colorC;
    setTexcoord(uvC.x, uvC.y);
    vertex3(c.x, c.y, c.z);
    end();
}

void BatchRenderer::drawWireBox(float minX, float minY, float minZ, float maxX, float maxY,
                                float maxZ)
{
    setTexture(mWhiteTexture);
    begin(ModeLines);
    // Bottom face
    vertex3(minX, minY, minZ);
    vertex3(maxX, minY, minZ);
    vertex3(maxX, minY, minZ);
    vertex3(maxX, minY, maxZ);
    vertex3(maxX, minY, maxZ);
    vertex3(minX, minY, maxZ);
    vertex3(minX, minY, maxZ);
    vertex3(minX, minY, minZ);
    // Top face
    vertex3(minX, maxY, minZ);
    vertex3(maxX, maxY, minZ);
    vertex3(maxX, maxY, minZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(minX, maxY, maxZ);
    vertex3(minX, maxY, maxZ);
    vertex3(minX, maxY, minZ);
    // Verticals
    vertex3(minX, minY, minZ);
    vertex3(minX, maxY, minZ);
    vertex3(maxX, minY, minZ);
    vertex3(maxX, maxY, minZ);
    vertex3(maxX, minY, maxZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(minX, minY, maxZ);
    vertex3(minX, maxY, maxZ);
    end();
}

void BatchRenderer::drawWireSphere(float cx, float cy, float cz, float radius, int segments)
{
    float step = 2.0f * PI / (float)segments;
    setTexture(mWhiteTexture);
    begin(ModeLines);
    for (int i = 0; i < segments; ++i)
    {
        float a0 = i * step, a1 = (i + 1) * step;
        vertex3(cx + radius * cosf(a0), cy + radius * sinf(a0), cz);
        vertex3(cx + radius * cosf(a1), cy + radius * sinf(a1), cz);
        vertex3(cx + radius * cosf(a0), cy, cz + radius * sinf(a0));
        vertex3(cx + radius * cosf(a1), cy, cz + radius * sinf(a1));
        vertex3(cx, cy + radius * sinf(a0), cz + radius * cosf(a0));
        vertex3(cx, cy + radius * sinf(a1), cz + radius * cosf(a1));
    }
    end();
}

void BatchRenderer::drawWireCylinder(float cx, float cy, float cz, float radius, float height,
                                     int segments)
{
    float halfH = height * 0.5f;
    float step = 2.0f * PI / (float)segments;
    setTexture(mWhiteTexture);
    begin(ModeLines);
    for (int i = 0; i < segments; ++i)
    {
        float a0 = i * step, a1 = (i + 1) * step;
        float x0 = radius * cosf(a0), z0 = radius * sinf(a0);
        float x1 = radius * cosf(a1), z1 = radius * sinf(a1);
        // bottom ring
        vertex3(cx + x0, cy - halfH, cz + z0);
        vertex3(cx + x1, cy - halfH, cz + z1);
        // top ring
        vertex3(cx + x0, cy + halfH, cz + z0);
        vertex3(cx + x1, cy + halfH, cz + z1);
    }
    // 4 vertical lines
    for (int i = 0; i < 4; ++i)
    {
        float a = i * (PI * 0.5f);
        vertex3(cx + radius * cosf(a), cy - halfH, cz + radius * sinf(a));
        vertex3(cx + radius * cosf(a), cy + halfH, cz + radius * sinf(a));
    }
    end();
}

void BatchRenderer::drawWireCapsule(float cx, float cy, float cz, float radius, float height,
                                    int segments)
{
    float halfH = height * 0.5f;
    float step = 2.0f * PI / (float)segments;
    int hsegs = segments / 2;
    float hstep = PI / (float)hsegs; // full meridian (equator -> pole -> equator)

    setTexture(mWhiteTexture);
    begin(ModeLines);
    // Cylinder rings
    for (int i = 0; i < segments; ++i)
    {
        float a0 = i * step, a1 = (i + 1) * step;
        vertex3(cx + radius * cosf(a0), cy - halfH, cz + radius * sinf(a0));
        vertex3(cx + radius * cosf(a1), cy - halfH, cz + radius * sinf(a1));
        vertex3(cx + radius * cosf(a0), cy + halfH, cz + radius * sinf(a0));
        vertex3(cx + radius * cosf(a1), cy + halfH, cz + radius * sinf(a1));
    }
    // 4 vertical lines
    for (int i = 0; i < 4; ++i)
    {
        float a = i * (PI * 0.5f);
        vertex3(cx + radius * cosf(a), cy - halfH, cz + radius * sinf(a));
        vertex3(cx + radius * cosf(a), cy + halfH, cz + radius * sinf(a));
    }
    // Top hemisphere — full meridians through the pole (XZ front/back and left/right),
    // matching the equator points used by the vertical lines above.
    for (int i = 0; i < hsegs; ++i)
    {
        float t0 = i * hstep, t1 = (i + 1) * hstep;
        vertex3(cx + radius * cosf(t0), cy + halfH + radius * sinf(t0), cz);
        vertex3(cx + radius * cosf(t1), cy + halfH + radius * sinf(t1), cz);
        vertex3(cx, cy + halfH + radius * sinf(t0), cz + radius * cosf(t0));
        vertex3(cx, cy + halfH + radius * sinf(t1), cz + radius * cosf(t1));
    }
    // Bottom hemisphere — mirrored meridians, dome hanging below.
    for (int i = 0; i < hsegs; ++i)
    {
        float t0 = i * hstep, t1 = (i + 1) * hstep;
        vertex3(cx + radius * cosf(t0), cy - halfH - radius * sinf(t0), cz);
        vertex3(cx + radius * cosf(t1), cy - halfH - radius * sinf(t1), cz);
        vertex3(cx, cy - halfH - radius * sinf(t0), cz + radius * cosf(t0));
        vertex3(cx, cy - halfH - radius * sinf(t1), cz + radius * cosf(t1));
    }
    end();
}

void BatchRenderer::drawSolidBox(float minX, float minY, float minZ, float maxX, float maxY,
                                 float maxZ)
{
    setTexture(mWhiteTexture);
    begin(ModeTriangles);
    // Bottom (Y-)
    vertex3(minX, minY, minZ);
    vertex3(maxX, minY, maxZ);
    vertex3(maxX, minY, minZ);
    vertex3(minX, minY, minZ);
    vertex3(minX, minY, maxZ);
    vertex3(maxX, minY, maxZ);
    // Top (Y+)
    vertex3(minX, maxY, minZ);
    vertex3(maxX, maxY, minZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(minX, maxY, minZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(minX, maxY, maxZ);
    // Front (Z-)
    vertex3(minX, minY, minZ);
    vertex3(maxX, maxY, minZ);
    vertex3(minX, maxY, minZ);
    vertex3(minX, minY, minZ);
    vertex3(maxX, minY, minZ);
    vertex3(maxX, maxY, minZ);
    // Back (Z+)
    vertex3(minX, minY, maxZ);
    vertex3(minX, maxY, maxZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(minX, minY, maxZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(maxX, minY, maxZ);
    // Left (X-)
    vertex3(minX, minY, minZ);
    vertex3(minX, maxY, maxZ);
    vertex3(minX, minY, maxZ);
    vertex3(minX, minY, minZ);
    vertex3(minX, maxY, minZ);
    vertex3(minX, maxY, maxZ);
    // Right (X+)
    vertex3(maxX, minY, minZ);
    vertex3(maxX, minY, maxZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(maxX, minY, minZ);
    vertex3(maxX, maxY, maxZ);
    vertex3(maxX, maxY, minZ);
    end();
}

void BatchRenderer::drawSolidSphere(float cx, float cy, float cz, float radius, int rings,
                                    int segments)
{
    float rstep = PI / (float)rings;
    float sstep = 2.0f * PI / (float)segments;
    setTexture(mWhiteTexture);
    begin(ModeTriangles);
    for (int r = 0; r < rings; ++r)
    {
        float phi0 = r * rstep, phi1 = (r + 1) * rstep;
        for (int s = 0; s < segments; ++s)
        {
            float th0 = s * sstep, th1 = (s + 1) * sstep;
            float x00 = radius * sinf(phi0) * cosf(th0), y00 = radius * cosf(phi0),
                  z00 = radius * sinf(phi0) * sinf(th0);
            float x10 = radius * sinf(phi1) * cosf(th0), y10 = radius * cosf(phi1),
                  z10 = radius * sinf(phi1) * sinf(th0);
            float x01 = radius * sinf(phi0) * cosf(th1), y01 = radius * cosf(phi0),
                  z01 = radius * sinf(phi0) * sinf(th1);
            float x11 = radius * sinf(phi1) * cosf(th1), y11 = radius * cosf(phi1),
                  z11 = radius * sinf(phi1) * sinf(th1);
            vertex3(cx + x00, cy + y00, cz + z00);
            vertex3(cx + x10, cy + y10, cz + z10);
            vertex3(cx + x11, cy + y11, cz + z11);
            vertex3(cx + x00, cy + y00, cz + z00);
            vertex3(cx + x11, cy + y11, cz + z11);
            vertex3(cx + x01, cy + y01, cz + z01);
        }
    }
    end();
}

void BatchRenderer::drawSolidCylinder(float cx, float cy, float cz, float radius, float height,
                                      int segments)
{
    float halfH = height * 0.5f;
    float step = 2.0f * PI / (float)segments;
    setTexture(mWhiteTexture);
    begin(ModeTriangles);
    for (int i = 0; i < segments; ++i)
    {
        float a0 = i * step, a1 = (i + 1) * step;
        float x0 = radius * cosf(a0), z0 = radius * sinf(a0);
        float x1 = radius * cosf(a1), z1 = radius * sinf(a1);
        // side
        vertex3(cx + x0, cy - halfH, cz + z0);
        vertex3(cx + x1, cy - halfH, cz + z1);
        vertex3(cx + x1, cy + halfH, cz + z1);
        vertex3(cx + x0, cy - halfH, cz + z0);
        vertex3(cx + x1, cy + halfH, cz + z1);
        vertex3(cx + x0, cy + halfH, cz + z0);
        // bottom cap
        vertex3(cx, cy - halfH, cz);
        vertex3(cx + x1, cy - halfH, cz + z1);
        vertex3(cx + x0, cy - halfH, cz + z0);
        // top cap
        vertex3(cx, cy + halfH, cz);
        vertex3(cx + x0, cy + halfH, cz + z0);
        vertex3(cx + x1, cy + halfH, cz + z1);
    }
    end();
}

void BatchRenderer::drawSolidCapsule(float cx, float cy, float cz, float radius, float height,
                                     int rings, int segments)
{
    float halfH = height * 0.5f;
    float sstep = 2.0f * PI / (float)segments;
    float hstep = (PI * 0.5f) / (float)rings;

    setTexture(mWhiteTexture);
    begin(ModeTriangles);
    // Cylinder side
    for (int i = 0; i < segments; ++i)
    {
        float a0 = i * sstep, a1 = (i + 1) * sstep;
        float x0 = radius * cosf(a0), z0 = radius * sinf(a0);
        float x1 = radius * cosf(a1), z1 = radius * sinf(a1);
        vertex3(cx + x0, cy - halfH, cz + z0);
        vertex3(cx + x1, cy - halfH, cz + z1);
        vertex3(cx + x1, cy + halfH, cz + z1);
        vertex3(cx + x0, cy - halfH, cz + z0);
        vertex3(cx + x1, cy + halfH, cz + z1);
        vertex3(cx + x0, cy + halfH, cz + z0);
    }
    // Top hemisphere (theta 0..PI/2, offset +halfH)
    for (int r = 0; r < rings; ++r)
    {
        float t0 = r * hstep, t1 = (r + 1) * hstep;
        for (int s = 0; s < segments; ++s)
        {
            float a0 = s * sstep, a1 = (s + 1) * sstep;
            float x00 = radius * sinf(t0) * cosf(a0), y00 = radius * cosf(t0),
                  z00 = radius * sinf(t0) * sinf(a0);
            float x10 = radius * sinf(t1) * cosf(a0), y10 = radius * cosf(t1),
                  z10 = radius * sinf(t1) * sinf(a0);
            float x01 = radius * sinf(t0) * cosf(a1), y01 = radius * cosf(t0),
                  z01 = radius * sinf(t0) * sinf(a1);
            float x11 = radius * sinf(t1) * cosf(a1), y11 = radius * cosf(t1),
                  z11 = radius * sinf(t1) * sinf(a1);
            vertex3(cx + x00, cy + halfH + y00, cz + z00);
            vertex3(cx + x10, cy + halfH + y10, cz + z10);
            vertex3(cx + x11, cy + halfH + y11, cz + z11);
            vertex3(cx + x00, cy + halfH + y00, cz + z00);
            vertex3(cx + x11, cy + halfH + y11, cz + z11);
            vertex3(cx + x01, cy + halfH + y01, cz + z01);
        }
    }
    // Bottom hemisphere (theta PI/2..PI, offset -halfH)
    for (int r = 0; r < rings; ++r)
    {
        float t0 = PI * 0.5f + r * hstep, t1 = PI * 0.5f + (r + 1) * hstep;
        for (int s = 0; s < segments; ++s)
        {
            float a0 = s * sstep, a1 = (s + 1) * sstep;
            float x00 = radius * sinf(t0) * cosf(a0), y00 = radius * cosf(t0),
                  z00 = radius * sinf(t0) * sinf(a0);
            float x10 = radius * sinf(t1) * cosf(a0), y10 = radius * cosf(t1),
                  z10 = radius * sinf(t1) * sinf(a0);
            float x01 = radius * sinf(t0) * cosf(a1), y01 = radius * cosf(t0),
                  z01 = radius * sinf(t0) * sinf(a1);
            float x11 = radius * sinf(t1) * cosf(a1), y11 = radius * cosf(t1),
                  z11 = radius * sinf(t1) * sinf(a1);
            vertex3(cx + x00, cy - halfH + y00, cz + z00);
            vertex3(cx + x10, cy - halfH + y10, cz + z10);
            vertex3(cx + x11, cy - halfH + y11, cz + z11);
            vertex3(cx + x00, cy - halfH + y00, cz + z00);
            vertex3(cx + x11, cy - halfH + y11, cz + z11);
            vertex3(cx + x01, cy - halfH + y01, cz + z01);
        }
    }
    end();
}

void BatchRenderer::drawAxis(float x, float y, float z, float size)
{
    unsigned int savedColor = mCurrentColor;
    setTexture(mWhiteTexture);

    // X - red
    setColor((unsigned char)255, (unsigned char)0, (unsigned char)0, (unsigned char)255);
    begin(ModeLines);
    vertex3(x, y, z);
    vertex3(x + size, y, z);
    end();
    // Y - green
    setColor((unsigned char)0, (unsigned char)255, (unsigned char)0, (unsigned char)255);
    begin(ModeLines);
    vertex3(x, y, z);
    vertex3(x, y + size, z);
    end();
    // Z - blue
    setColor((unsigned char)0, (unsigned char)0, (unsigned char)255, (unsigned char)255);
    begin(ModeLines);
    vertex3(x, y, z);
    vertex3(x, y, z + size);
    end();

    mCurrentColor = savedColor;
}

void BatchRenderer::drawWireGrid(float y, int slices, float spacing, bool axes)
{
    slices = slices < 1 ? 1 : slices;
    const float half = (float)slices * spacing * 0.5f;

    setTexture(mWhiteTexture);
    begin(ModeLines);
    for (int i = 0; i <= slices; ++i)
    {
        const float offset = -half + (float)i * spacing;
        const bool center = axes && fabsf(offset) < 0.0001f;

        if (center)
            setColor(0.8f, 0.2f, 0.2f, 1.0f);
        else
            setColor(0.45f, 0.45f, 0.45f, 1.0f);
        vertex3(offset, y, -half);
        vertex3(offset, y, half);

        if (center)
            setColor(0.2f, 0.8f, 0.2f, 1.0f);
        else
            setColor(0.45f, 0.45f, 0.45f, 1.0f);
        vertex3(-half, y, offset);
        vertex3(half, y, offset);
    }
    end();
    if (axes)
        setColor(1.0f, 1.0f, 1.0f, 1.0f);
}

void BatchRenderer::drawTexture(TextureHandle texture, float dstX, float dstY, float dstW,
                                float dstH, float srcX, float srcY, float srcW, float srcH,
                                float pivotX, float pivotY, float rotationDeg)
{
    setTexture(texture);

    if (rotationDeg != 0.0f)
    {
        pushMatrix();
        translate(dstX + pivotX, dstY + pivotY);
        rotate(rotationDeg, 0.0f, 0.0f, 1.0f);
        translate(-pivotX, -pivotY);
        dstX = 0.0f;
        dstY = 0.0f;
    }

    begin(ModeTriangles);

    float u0 = srcX, v0 = srcY;
    float u1 = srcX + srcW, v1 = srcY + srcH;

    if (srcW == 0.0f || srcH == 0.0f)
    {
        u0 = 0.0f;
        v0 = 0.0f;
        u1 = 1.0f;
        v1 = 1.0f;
    }

    emitTexturedTriangle(dstX, dstY, u0, v0, dstX + dstW, dstY, u1, v0, dstX + dstW, dstY + dstH,
                         u1, v1);
    emitTexturedTriangle(dstX, dstY, u0, v0, dstX + dstW, dstY + dstH, u1, v1, dstX, dstY + dstH,
                         u0, v1);

    end();

    if (rotationDeg != 0.0f)
    {
        popMatrix();
    }
}

// ---------------------------------------------------------------------------
// Batch flushing and rendering
// ---------------------------------------------------------------------------

void BatchRenderer::drawRenderBatch()
{
    flushBatch();
}

void BatchRenderer::update()
{
    // Per-frame counters start over here; frameCount is a running total and
    // survives, which is why this is not a plain resetStats().
    mStats.drawCalls = 0;
    mStats.verticesDrawn = 0;
    mStats.indicesDrawn = 0;
    mStats.textureSwitches = 0;
    mStats.batchesFlushed = 0;

    if (mConfig.enableProfiling)
    {
        mFrameStartTime = getTimeMilliseconds();
    }
}

void BatchRenderer::flip()
{
    const double start = getTimeMilliseconds();

    // Flush any pending data
    if (mInBeginEnd)
    {
        end();
    }

    if (!mVertices.empty())
    {
        flushBatch();
    }

    const double end = getTimeMilliseconds();

    if (mConfig.enableProfiling)
    {
        mStats.renderTime = end - start;
        mStats.totalTime = end - mFrameStartTime;
        mStats.frameCount++;
        mStats.batchTime = mStats.totalTime - mStats.renderTime;
    }
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

void BatchRenderer::resetStats()
{
    memset(&mStats, 0, sizeof(mStats));
}

void BatchRenderer::printStats() const
{
    const Stats& s = mStats;

    printf("=== BatchRenderer Stats ===\n");
    printf("  Draw Calls:        %zu\n", s.drawCalls);
    printf("  Vertices Drawn:    %zu\n", s.verticesDrawn);
    printf("  Indices Drawn:     %zu\n", s.indicesDrawn);
    printf("  Texture Switches:  %zu\n", s.textureSwitches);
    printf("  Batches Flushed:   %zu\n", s.batchesFlushed);
    printf("  Frame Count:       %zu\n", s.frameCount);
    printf("  Batch Time:        %.2f ms\n", s.batchTime);
    printf("  Render Time:       %.2f ms\n", s.renderTime);
    printf("  Total Time:        %.2f ms\n", s.totalTime);
    printf("  FPS:               %.2f\n", s.totalTime > 0.0 ? 1000.0 / s.totalTime : 0.0);
    printf("===========================\n");
}

// ---------------------------------------------------------------------------
// Projection matrix
// ---------------------------------------------------------------------------

void BatchRenderer::setProjection(const glm::mat4& matrix)
{
    mProjection = matrix;
}

// ---------------------------------------------------------------------------
// Color utilities
// ---------------------------------------------------------------------------

unsigned int BatchRenderer::packColor(unsigned char r, unsigned char g, unsigned char b,
                                      unsigned char a)
{
    return (unsigned int)r | ((unsigned int)g << 8) | ((unsigned int)b << 16) |
           ((unsigned int)a << 24);
}

void BatchRenderer::unpackColor(unsigned int packed, unsigned char& r, unsigned char& g,
                                unsigned char& b, unsigned char& a)
{
    r = packed & 0xFF;
    g = (packed >> 8) & 0xFF;
    b = (packed >> 16) & 0xFF;
    a = (packed >> 24) & 0xFF;
}

// ---------------------------------------------------------------------------
// Internal methods
// ---------------------------------------------------------------------------

void BatchRenderer::updateProjection()
{
    mProjection = glm::ortho(0.0f, static_cast<float>(mWindowWidth),
                             static_cast<float>(mWindowHeight), 0.0f, -1.0f, 1.0f);
}

void BatchRenderer::flushBatch()
{
    if (mVertices.empty() || mDrawCalls.empty())
        return;

    usize vertexBufferSize = mVertices.size() * sizeof(Vertex);
    mGpu->updateBuffer(mVertexBuffer, 0, vertexBufferSize, mVertices.data());

    // Generate indices for quads
    mIndices.clear();

    for (usize i = 0; i < mDrawCalls.size(); ++i)
    {
        const DrawCall& call = mDrawCalls[i];

        if (call.mode == ModeQuads)
        {
            usize quadCount = call.vertexCount / 4;
            for (usize q = 0; q < quadCount; ++q)
            {
                usize baseVertex = call.vertexAlignment + q * 4;
                mIndices.push_back(static_cast<unsigned short>(baseVertex + 0));
                mIndices.push_back(static_cast<unsigned short>(baseVertex + 1));
                mIndices.push_back(static_cast<unsigned short>(baseVertex + 2));
                mIndices.push_back(static_cast<unsigned short>(baseVertex + 0));
                mIndices.push_back(static_cast<unsigned short>(baseVertex + 2));
                mIndices.push_back(static_cast<unsigned short>(baseVertex + 3));
            }
        }
    }

    if (!mIndices.empty())
    {
        usize indexBufferSize = mIndices.size() * sizeof(unsigned short);
        mGpu->updateBuffer(mIndexBuffer, 0, indexBufferSize, mIndices.data());
    }

    // Draw all draw calls
    applyDrawCalls();

    if (mConfig.enableProfiling)
    {
        mStats.drawCalls += mDrawCalls.size();
        mStats.verticesDrawn += mVertices.size();
        mStats.indicesDrawn += mIndices.size();
        mStats.batchesFlushed++;
    }

    // Clear for next frame
    mVertices.clear();
    mIndices.clear();
    mDrawCalls.clear();
}

namespace
{

Radion::BlendMode blendModeToGPU(BatchRenderer::BlendMode mode)
{
    switch (mode)
    {
    case BatchRenderer::BlendMode::Alpha:
        return Radion::BlendMode::Alpha;
    case BatchRenderer::BlendMode::Additive:
        return Radion::BlendMode::Additive;
    case BatchRenderer::BlendMode::Multiplied:
        return Radion::BlendMode::Multiply;
    case BatchRenderer::BlendMode::AddColors:
        return Radion::BlendMode::AddColors;
    case BatchRenderer::BlendMode::SubtractColors:
        return Radion::BlendMode::SubtractColors;
    }
    return Radion::BlendMode::Alpha;
}

} // namespace

PipelineHandle BatchRenderer::pipelineFor(const DrawCall& call)
{
    const u32 key = static_cast<u32>(call.mode) | (call.depthTest ? 1u << 8 : 0u) |
                    (call.depthWrite ? 1u << 9 : 0u) | (call.blend ? 1u << 10 : 0u) |
                    (call.cullFace ? 1u << 11 : 0u) | (static_cast<u32>(call.blendMode) << 12);

    for (usize i = 0; i < mPipelines.size(); ++i)
    {
        if (mPipelines[i].key == key)
            return mPipelines[i].pipeline;
    }

    PipelineDesc desc;
    desc.vs.code = VERTEX_SHADER_SOURCE;
    desc.vs.name = "batch.vs";
    desc.fs.code = FRAGMENT_SHADER_SOURCE;
    desc.fs.name = "batch.fs";
    desc.debugName = "batch";

    desc.layout.streamCount = 1;
    desc.layout.streams[0].stride = sizeof(Vertex);
    desc.layout.attribCount = 3;
    desc.layout.attribs[0] = {0, 0, 0, AttribFormat::Float3};
    desc.layout.attribs[1] = {1, 0, offsetof(Vertex, u), AttribFormat::Float2};
    desc.layout.attribs[2] = {2, 0, offsetof(Vertex, r), AttribFormat::UByte4N};

    desc.topology = call.mode == ModeLines ? Topology::Lines : Topology::Triangles;
    desc.blend.mode = call.blend ? blendModeToGPU(call.blendMode) : Radion::BlendMode::Opaque;
    desc.depth.test = call.depthTest;
    desc.depth.write = call.depthWrite;
    desc.raster.cull = call.cullFace ? CullMode::Back : CullMode::None;

    CachedPipeline cached;
    cached.key = key;
    cached.pipeline = mGpu->createPipeline(desc);
    mPipelines.push_back(cached);
    return cached.pipeline;
}

void BatchRenderer::applyDrawCalls()
{
    mGpu->updateBuffer(mUniformBuffer, 0, sizeof(glm::mat4), &mProjection[0][0]);
    mGpu->bindUniform(0, mUniformBuffer, 0, sizeof(glm::mat4));

    usize vertexOffset = 0;
    u32 indexOffset = 0;

    for (usize i = 0; i < mDrawCalls.size(); ++i)
    {
        const DrawCall& call = mDrawCalls[i];

        PipelineHandle pipeline = pipelineFor(call);
        if (!pipeline.valid())
        {
            vertexOffset += call.vertexCount;
            continue;
        }
        mGpu->setPipeline(pipeline);
        mGpu->bindTexture(0, call.texture.valid() ? call.texture : mWhiteTexture, mSampler);

        DrawDesc draw;
        draw.vertexBuffers[0] = mVertexBuffer;
        draw.vertexBufferCount = 1;

        if (call.mode == ModeQuads)
        {
            const u32 indexCount = static_cast<u32>(call.vertexCount / 4) * 6;
            draw.indexBuffer = mIndexBuffer;
            draw.indexType = IndexType::U16;
            draw.count = indexCount;
            draw.first = indexOffset;
            indexOffset += indexCount;
        }
        else
        {
            draw.count = static_cast<u32>(call.vertexCount);
            draw.first = static_cast<u32>(vertexOffset);
        }

        mGpu->draw(draw);
        vertexOffset += call.vertexCount;
    }
}

void BatchRenderer::setupBuffers()
{
    BufferDesc vertexDesc;
    vertexDesc.size = mConfig.maxVertices * sizeof(Vertex);
    vertexDesc.usage = BufferVertex;
    vertexDesc.residency = Residency::Stream;
    vertexDesc.debugName = "batch.vertices";
    mVertexBuffer = mGpu->createBuffer(vertexDesc);

    BufferDesc indexDesc;
    indexDesc.size = mConfig.maxVertices * 3 * sizeof(unsigned short);
    indexDesc.usage = BufferIndex;
    indexDesc.residency = Residency::Stream;
    indexDesc.debugName = "batch.indices";
    mIndexBuffer = mGpu->createBuffer(indexDesc);

    BufferDesc uniformDesc;
    uniformDesc.size = sizeof(glm::mat4);
    uniformDesc.usage = BufferUniform;
    uniformDesc.residency = Residency::Stream;
    uniformDesc.debugName = "batch.uniforms";
    mUniformBuffer = mGpu->createBuffer(uniformDesc);

    SamplerDesc samplerDesc;
    samplerDesc.filter = Filter::Point;
    samplerDesc.wrapU = Wrap::Clamp;
    samplerDesc.wrapV = Wrap::Clamp;
    mSampler = mGpu->createSampler(samplerDesc);
}

void BatchRenderer::setupTexture()
{
    unsigned char whitePixel[4] = {255, 255, 255, 255};

    TextureDesc desc;
    desc.width = 1;
    desc.height = 1;
    desc.mips = 1;
    desc.format = Format::RGBA8;
    desc.data = whitePixel;
    desc.debugName = "batch.white";
    mWhiteTexture = mGpu->createTexture(desc);
}

void BatchRenderer::setupFontTexture()
{
    // Generate white RGBA font atlas from embedded 8x8 bitmap font
    u8* atlas = new u8[FONT_ATLAS_W * FONT_ATLAS_H * 4];
    memset(atlas, 0, FONT_ATLAS_W * FONT_ATLAS_H * 4);

    for (int g = 0; g < 96; ++g)
    {
        int cellX = (g % FONT_COLS) * 8;
        int cellY = (g / FONT_COLS) * 8;
        for (int row = 0; row < 8; ++row)
        {
            u8 bits = kFont8x8[g][row];
            for (int col = 0; col < 8; ++col)
            {
                if (!((bits >> col) & 1))
                    continue; // LSB = leftmost pixel
                u8* p = &atlas[((cellY + row) * FONT_ATLAS_W + cellX + col) * 4];
                p[0] = p[1] = p[2] = p[3] = 255;
            }
        }
    }

    TextureDesc desc;
    desc.width = FONT_ATLAS_W;
    desc.height = FONT_ATLAS_H;
    desc.mips = 1;
    desc.format = Format::RGBA8;
    desc.data = atlas;
    desc.debugName = "batch.font";
    mFontTexture = mGpu->createTexture(desc);

    delete[] atlas;
}

void BatchRenderer::submitVertex(float x, float y, float z)
{
    submitVertex(x, y, z, mCurrentTexcoord[0], mCurrentTexcoord[1]);
}

void BatchRenderer::submitVertex(float x, float y, float z, float u, float v)
{
    Vertex vert;
    vert.x = x;
    vert.y = y;
    vert.z = z;
    vert.u = u;
    vert.v = v;
    unpackColor(mCurrentColor, vert.r, vert.g, vert.b, vert.a);
    mVertices.push_back(vert);
}

void BatchRenderer::applyTransform(float& x, float& y, float& z)
{
    const glm::vec4 transformed = mCurrentMatrix * glm::vec4(x, y, z, 1.0f);
    x = transformed.x;
    y = transformed.y;
    z = transformed.z;
}

// ---------------------------------------------------------------------------
// Text rendering with embedded 8x8 font
// ---------------------------------------------------------------------------

glm::vec4 fontGlyphUVRect(unsigned char code)
{
    if (code < 32 || code > 127 || code == ' ')
        return glm::vec4(0.0f);
    const f32 cw = 8.f / (f32)FONT_ATLAS_W;
    const f32 ch = 8.f / (f32)FONT_ATLAS_H;
    const int g = code - 32;
    const f32 u0 = (f32)(g % FONT_COLS) * cw;
    const f32 v0 = (f32)(g / FONT_COLS) * ch;
    return glm::vec4(u0, v0, cw, ch);
}

void BatchRenderer::drawText(f32 x, f32 y, f32 size, const char* text)
{
    if (!text || size <= 0.f)
        return;

    setTexture(mFontTexture);
    begin(ModeTriangles);

    f32 penX = x, penY = y;

    for (const char* c = text; *c; ++c)
    {
        if (*c == '\n')
        {
            penX = x;
            penY += size;
            continue;
        }
        u8 code = (u8)*c;
        if (code < 32 || code > 127)
            code = '?';
        const glm::vec4 rect = fontGlyphUVRect(code);
        if (rect.z > 0.0f)
        {
            const f32 u0 = rect.x, v0 = rect.y, u1 = rect.x + rect.z, v1 = rect.y + rect.w;
            emitTexturedTriangle(penX, penY, u0, v0, penX + size, penY, u1, v0, penX, penY + size,
                                 u0, v1);
            emitTexturedTriangle(penX + size, penY, u1, v0, penX + size, penY + size, u1, v1, penX,
                                 penY + size, u0, v1);
        }
        penX += size;
    }

    end();
}

f32 BatchRenderer::textWidth(f32 size, const char* text) const
{
    if (!text)
        return 0.f;
    u32 longest = 0, line = 0;
    for (const char* c = text; *c; ++c)
    {
        if (*c == '\n')
        {
            if (line > longest)
                longest = line;
            line = 0;
            continue;
        }
        ++line;
    }
    if (line > longest)
        longest = line;
    return (f32)longest * size;
}

} // namespace Radion
