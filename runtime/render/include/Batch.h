#ifndef RADION_BATCH_H
#define RADION_BATCH_H

#include "GPU.h"
#include "Math.h"
#include "Types.h"

#include <vector>

namespace Radion
{

// Normalized (u0, v0, width, height) of one glyph's cell in the embedded
// 8x8 font atlas (BatchRenderer::fontTexture()), ASCII 32..127. Space and
// out-of-range codes return a zero-area rect - callers skip drawing on
// width == 0 rather than sampling a garbage cell.
glm::vec4 fontGlyphUVRect(unsigned char code);

class BatchRenderer
{
public:
    struct Config
    {
        usize maxVertices;
        usize maxDrawCalls;
        usize stackDepth;
        bool enableProfiling;

        Config() : maxVertices(32768), maxDrawCalls(256), stackDepth(32), enableProfiling(true)
        {
        }
    };

    struct Stats
    {
        usize drawCalls;
        usize verticesDrawn;
        usize indicesDrawn;
        usize textureSwitches;
        double batchTime;
        double renderTime;
        double totalTime;
        usize batchesFlushed;
        usize frameCount;
    };

    enum class BlendMode
    {
        Alpha,
        Additive,
        Multiplied,
        AddColors,
        SubtractColors
    };

    BatchRenderer();
    ~BatchRenderer();

    bool init(const Config& config = Config());
    void shutdown();

    bool resize(int width, int height);
    void getWindowSize(int& width, int& height) const;

    // Transform stack
    void pushMatrix();
    void popMatrix();
    void loadIdentity();
    void translate(float x, float y, float z = 0.0f);
    void rotate(float angleDeg, float axisX, float axisY, float axisZ);
    void scale(float x, float y, float z = 1.0f);

    // State
    void setColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255);
    void setColor(float r, float g, float b, float a = 1.0f);
    void setTexture(TextureHandle texture);
    void setBlendMode(BlendMode mode);
    void setTexcoord(float u, float v);
    void setDepthTest(bool enabled)
    {
        mDepthTestEnabled = enabled;
    }
    void setDepthWrite(bool enabled)
    {
        mDepthWriteEnabled = enabled;
    }
    void setBlend(bool enabled)
    {
        mBlendEnabled = enabled;
    }
    void setCullFace(bool enabled)
    {
        mCullFaceEnabled = enabled;
    }
    void setDefault3DState();

    void setClipRect(float x, float y, float width, float height);
    void setClipRect(const FloatRect& rect);
    void clearClipRect();
    bool isClipEnabled() const
    {
        return mClipEnabled;
    }
    const FloatRect& getClipRect() const
    {
        return mClipRect;
    }

    // Immediate mode
    void begin(int mode);
    void end();
    void vertex2(float x, float y);
    void vertex3(float x, float y, float z);

    // Primitives
    void drawLine(float x0, float y0, float x1, float y1);
    void drawThickLine(float x0, float y0, float x1, float y1, float thickness);
    void drawTriangle(float x1, float y1, float x2, float y2, float x3, float y3);
    void drawRect(float x, float y, float width, float height, bool fill = true);
    void drawQuad(float x, float y, float width, float height);
    void drawCircle(float cx, float cy, float radius, int segments = 32);
    void drawEllipse(float cx, float cy, float rx, float ry, bool fill = true, int segments = 32);
    void drawArc(float cx, float cy, float radius, float startDeg, float endDeg, int segments = 32);
    void drawRing(float cx, float cy, float rInner, float rOuter, bool fill = true,
                  int segments = 32);
    void drawPolygon(float cx, float cy, int sides, float radius, float rotDeg = 0.0f,
                     bool fill = true);
    void drawPolyline(const float* xyPairs, int pointCount);

    // 3D debug primitives
    void drawLine3D(float x0, float y0, float z0, float x1, float y1, float z1);
    void drawTriangle3D(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);
    void drawTriangle3D(const glm::vec3& a, const glm::vec2& uvA, const glm::vec3& b,
                        const glm::vec2& uvB, const glm::vec3& c, const glm::vec2& uvC);
    void drawTriangle3D(const glm::vec3& a, const glm::vec2& uvA, u32 colorA, const glm::vec3& b,
                        const glm::vec2& uvB, u32 colorB, const glm::vec3& c, const glm::vec2& uvC,
                        u32 colorC);

    // Wireframe
    void drawWireBox(float minX, float minY, float minZ, float maxX, float maxY, float maxZ);
    void drawWireSphere(float cx, float cy, float cz, float radius, int segments = 24);
    void drawWireCylinder(float cx, float cy, float cz, float radius, float height,
                          int segments = 24);
    void drawWireCapsule(float cx, float cy, float cz, float radius, float height,
                         int segments = 24);

    // Solid
    void drawSolidBox(float minX, float minY, float minZ, float maxX, float maxY, float maxZ);
    void drawSolidSphere(float cx, float cy, float cz, float radius, int rings = 12,
                         int segments = 24);
    void drawSolidCylinder(float cx, float cy, float cz, float radius, float height,
                           int segments = 24);
    void drawSolidCapsule(float cx, float cy, float cz, float radius, float height, int rings = 8,
                          int segments = 24);

    void drawAxis(float x, float y, float z, float size = 1.0f);
    void drawWireGrid(float y, int slices, float spacing, bool axes = true);

    // Textured draw
    void drawTexture(TextureHandle texture, float dstX, float dstY, float dstW, float dstH,
                     float srcX = 0.0f, float srcY = 0.0f, float srcW = 0.0f, float srcH = 0.0f,
                     float pivotX = 0.0f, float pivotY = 0.0f, float rotationDeg = 0.0f);

    // Batch control
    void drawRenderBatch();
    void update();
    void flip();

    // Text rendering (embedded 8x8 font)
    void drawText(float x, float y, float size, const char* text);
    float textWidth(float size, const char* text) const;
    // The embedded font's atlas texture, valid once init() has run. Anything
    // that draws the same 8x8 font outside this batch's own drawText() (a
    // world-space text component, say) samples this atlas with
    // fontGlyphUVRect() rather than baking its own copy.
    TextureHandle fontTexture() const
    {
        return mFontTexture;
    }

    // Projection
    void setProjection(const glm::mat4& matrix);
    const glm::mat4& getProjection() const
    {
        return mProjection;
    }

    // Stats
    void resetStats();
    void printStats() const;
    const Stats& getStats() const
    {
        return mStats;
    }

    // Color utilities
    static unsigned int packColor(unsigned char r, unsigned char g, unsigned char b,
                                  unsigned char a);
    static void unpackColor(unsigned int packed, unsigned char& r, unsigned char& g,
                            unsigned char& b, unsigned char& a);

private:
#pragma pack(push, 1)
    struct Vertex
    {
        float x, y, z;
        float u, v;
        unsigned char r, g, b, a;
    };
#pragma pack(pop)

    struct DrawCall
    {
        int mode;
        int vertexCount;
        usize vertexAlignment;
        TextureHandle texture;
        BlendMode blendMode;
        bool depthTest;
        bool depthWrite;
        bool blend;
        bool cullFace;
    };

    void updateProjection();
    void flushBatch();
    void applyDrawCalls();
    void setupBuffers();
    void setupTexture();
    void setupFontTexture();

    // One PSO per combination of primitive and render state actually used.
    // Built on demand, so a program that only draws wire boxes ends up with a
    // single pipeline instead of the full matrix of them.
    PipelineHandle pipelineFor(const DrawCall& call);
    void submitVertex(float x, float y, float z);
    void submitVertex(float x, float y, float z, float u, float v);
    void applyTransform(float& x, float& y, float& z);

    struct ClipVertex
    {
        float x, y, u, v;
    };
    int clipPolygonToRect(const ClipVertex* in, int inCount, ClipVertex* out) const;
    bool clipSegmentToRect(float& x0, float& y0, float& x1, float& y1) const;
    void submitClippedQuad(float x0, float y0, float x1, float y1, float x2, float y2, float x3,
                           float y3);
    void submitClippedLine(float x0, float y0, float x1, float y1);
    void emitLine(float x0, float y0, float x1, float y1);
    void emitTriangle(float x0, float y0, float x1, float y1, float x2, float y2);
    void emitTexturedTriangle(float x0, float y0, float u0, float v0, float x1, float y1, float u1,
                              float v1, float x2, float y2, float u2, float v2);

    static constexpr int ModeLines = 0x0001;
    static constexpr int ModeTriangles = 0x0004;
    static constexpr int ModeQuads = 0x0007;

    Config mConfig;
    Stats mStats;

    std::vector<Vertex> mVertices;
    std::vector<unsigned short> mIndices;
    std::vector<DrawCall> mDrawCalls;

    std::vector<glm::mat4> mMatrixStack;
    glm::mat4 mCurrentMatrix;

    unsigned int mCurrentColor;
    TextureHandle mCurrentTexture;
    float mCurrentTexcoord[2];
    int mCurrentMode;
    bool mInBeginEnd;
    BlendMode mCurrentBlendMode;

    int mWindowWidth;
    int mWindowHeight;
    glm::mat4 mProjection;
    bool mDepthTestEnabled;
    bool mDepthWriteEnabled;
    bool mBlendEnabled;
    bool mCullFaceEnabled;

    bool mClipEnabled;
    FloatRect mClipRect;

    struct CachedPipeline
    {
        u32 key;
        PipelineHandle pipeline;
    };

    GPU* mGpu;
    BufferHandle mVertexBuffer;
    BufferHandle mIndexBuffer;
    BufferHandle mUniformBuffer;
    TextureHandle mWhiteTexture;
    TextureHandle mFontTexture;
    SamplerHandle mSampler;
    std::vector<CachedPipeline> mPipelines;

    double mFrameStartTime;
};

} // namespace Radion

#endif // RADION_BATCH_H
