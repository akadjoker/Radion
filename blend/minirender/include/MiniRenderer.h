#ifndef RADION_MINI_RENDERER_H
#define RADION_MINI_RENDERER_H

#include "Types.h"
#include "Math.h"
#include <vector>

namespace Radion
{
class Engine;
struct MeshData;

// Upper bound on uBonePalette[]; a preview shows one rigged character at a time.
constexpr u32 kMiniRendererMaxBones = 128;

struct MiniRendererConfig
{
    f32 lightIntensity = 1.0f;
    // Direction the light travels (the shader lights by the dot with the opposite).
    Math::vec3 lightDirection = Math::normalize(Math::vec3(-0.5f, -1.0f, -0.5f));
    Math::vec3 ambientColor = Math::vec3(0.3f, 0.3f, 0.3f);
    f32 ambientIntensity = 0.3f;
};

enum class MiniRenderMode : u8
{
    Wireframe,
    Solid,
    Textured,
};

enum class MiniDebugView : u8
{
    None,
    Normals,
    Tangents,
    UVs,
};

struct MiniDrawParams
{
    MiniRenderMode mode = MiniRenderMode::Textured;
    f32 alpha = 1.0f; // < 1 draws blended, depth write off (onion-skin ghosts)
    Math::vec3 tint = Math::vec3(1.0f);

    // Depth test off for the whole draw; alpha defaults to 0.35 when left at 1.0 (opaque X-ray shows nothing).
    bool xray = false;

    MiniDebugView debugView = MiniDebugView::None;
    bool facetedShading = false;
    bool unlit = false;
    // Linear vertex colours, as glTF COLOR_0.
    bool vertexColors = false;
    // GL_POINTS pass over the static vertex buffer; selected points come from setVertexSelection().
    bool showVertexPoints = false;
    Math::vec3 vertexColor = Math::vec3(1.0f, 0.8f, 0.1f);
    Math::vec3 selectedVertexColor = Math::vec3(1.0f, 0.4f, 0.0f);
    f32 vertexPointSize = 4.0f;
    bool showWireframeOverlay = false;
    bool colorBySubmesh = false;

    // Index-parallel to MeshData::submeshes, nonzero = visible; entries past submeshVisibleCount draw as visible.
    const u8* submeshVisible = nullptr;
    u32 submeshVisibleCount = 0;

    // World/model-space palette per bone; empty draws every vertex with joint 0 at identity.
    const Math::mat4* bonePalette = nullptr;
    u32 boneCount = 0;
};

class MiniRenderer
{
public:
    explicit MiniRenderer(Engine& engine);
    ~MiniRenderer();

    MiniRenderer(const MiniRenderer&) = delete;
    MiniRenderer& operator=(const MiniRenderer&) = delete;

    bool initialize();
    void shutdown();

    void setLightDirection(const Math::vec3& direction)
    {
        mConfig.lightDirection = Math::normalize(direction);
    }
    void setLightIntensity(f32 intensity)
    {
        mConfig.lightIntensity = intensity;
    }
    void setAmbientColor(const Math::vec3& color)
    {
        mConfig.ambientColor = color;
    }
    void setAmbientIntensity(f32 intensity)
    {
        mConfig.ambientIntensity = intensity;
    }

    const MiniRendererConfig& config() const
    {
        return mConfig;
    }

    void invalidate();

    // One byte per vertex, nonzero = selected, in its own buffer; call only when the selection changed.
    void setVertexSelection(const u8* selected, u32 count);

    // One byte per triangle, nonzero = hidden; ignored unless it has exactly one entry per triangle.
    void setHiddenFaces(const u8* faceHidden, u32 faceCount);

    // Bumped on every mesh upload, which also zeroes the selection buffer; callers caching what they sent must watch it.
    u64 meshUploadRevision() const
    {
        return mUploadRevision;
    }

    void renderViewport(const MeshData* mesh,
                        const Math::mat4& viewMatrix,
                        const Math::mat4& projectionMatrix,
                        const Math::vec3& cameraPos,
                        const MiniDrawParams& params = {});

private:
    Engine& mEngine;
    MiniRendererConfig mConfig;
    u32 mShaderProgram = 0;

    u32 mVAO = 0;
    u32 mVBO = 0;
    u32 mEBO = 0;
    u32 mSelectionVBO = 0;
    u32 mSelectionCapacity = 0;
    u64 mUploadRevision = 0;
    u32 mIndexCount = 0;
    u32 mVertexCount = 0;
    const MeshData* mUploadedMesh = nullptr;
    std::vector<u8> mHiddenFaces;
    bool mHasHiddenFaces = false;

    u32 mWhiteTexture = 0;
    u32 mFlatNormalTexture = 0;

    bool compileShaders();
    bool createDefaultTextures();
    void destroyBuffers();
    void uploadMesh(const MeshData& mesh);
    // Skips hidden triangles inside the range.
    void drawTriangleRange(u32 indexOffset, u32 indexCount);
};

} // namespace Radion

#endif // RADION_MINI_RENDERER_H
