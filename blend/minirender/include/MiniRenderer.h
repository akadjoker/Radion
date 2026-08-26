#ifndef RADION_MINI_RENDERER_H
#define RADION_MINI_RENDERER_H

#include "Types.h"
#include "Math.h"
#include "Math.h"
#include <vector>

namespace Radion
{
class Engine;
struct MeshData;

// Upper bound on the shader's uBonePalette[] array. A preview never needs
// the engine's own MatrixPalette budget - this only has to cover the one
// rigged character on screen at a time.
constexpr u32 kMiniRendererMaxBones = 128;

struct MiniRendererConfig
{
    f32 lightIntensity = 1.0f;
    Math::vec3 lightDirection = Math::normalize(Math::vec3(0.5f, 1.0f, 0.5f));
    Math::vec3 ambientColor = Math::vec3(0.3f, 0.3f, 0.3f);
    f32 ambientIntensity = 0.3f;
};

enum class MiniRenderMode : u8
{
    Wireframe,
    Solid,
    Textured,
};

// What the solid/textured pass colours the surface with, instead of lit
// shading - a GPU-side debug view, not a separate technique to isolate in a
// panel toggle. None runs the ordinary lighting/BRDF path.
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

    // Blender's X-ray: depth test off for the whole draw (main pass and any
    // overlay), so nothing behind the mesh is occluded by it - stacked
    // translucent layers, not a sorted blend. Defaults alpha to 0.35 when the
    // caller left it at the opaque 1.0, since an opaque X-ray shows nothing.
    bool xray = false;

    MiniDebugView debugView = MiniDebugView::None;
    // Flat (per-face, provoking-vertex) normal instead of the smooth one -
    // both in shading and in the Normals debug view. Costs nothing extra:
    // the flat varying is always written, this only picks which one reads.
    bool facetedShading = false;
    // Skips the light/BRDF term entirely - just uTint (or the per-submesh
    // tint colorBySubmesh already multiplies it by), the "flat color, no
    // shadow" solid look.
    bool unlit = false;
    // Extra GL_POINTS pass over the mesh, drawn straight from the static
    // vertex buffer - one draw call, nothing uploaded per frame. Which points
    // come out selected is whatever setVertexSelection() last stored.
    bool showVertexPoints = false;
    Math::vec3 vertexColor = Math::vec3(1.0f, 0.8f, 0.1f);
    Math::vec3 selectedVertexColor = Math::vec3(1.0f, 0.4f, 0.0f);
    f32 vertexPointSize = 4.0f;
    bool showWireframeOverlay = false; // extra wireframe pass over the solid one
    bool colorBySubmesh = false;

    // Per-submesh viewport visibility, index-parallel to MeshData::submeshes,
    // nonzero meaning visible - a byte array rather than bool* since
    // std::vector<bool> has no real storage to point into. A submesh at or
    // past submeshVisibleCount draws as usual (missing means visible, not
    // hidden), so a caller only needs to size this to however many entries
    // it actually tracked. Forces the same per-submesh draw loop
    // colorBySubmesh uses, even with colorBySubmesh off.
    const u8* submeshVisible = nullptr;
    u32 submeshVisibleCount = 0;

    // Skinning palette, world/model space per bone (Skeleton::evaluate()'s
    // own palette output). Empty draws every vertex with joint 0 at identity
    // - the same "zero when unused" convention MeshPreview's GPUInstance uses,
    // so an unrigged mesh needs no separate vertex format or shader branch.
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

    // Configuration
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

    // One byte per vertex, nonzero where selected, kept in its own buffer so
    // a selection change costs a byte per vertex and never touches the
    // geometry. Call it only when the selection actually changed - the
    // viewport has BlenderSelection::revision() to tell.
    void setVertexSelection(const u8* selected, u32 count);

    // Bumped every time the mesh is uploaded, which is also every time the
    // selection buffer is recreated and zeroed. A caller that caches what it
    // last sent has to watch this as well as its own state: editing a mesh
    // in place leaves the pointer and the selection unchanged while the
    // buffer behind them is new and empty.
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

    u32 mWhiteTexture = 0;
    u32 mFlatNormalTexture = 0;

    bool compileShaders();
    bool createDefaultTextures();
    void destroyBuffers();
    void uploadMesh(const MeshData& mesh);
};

} // namespace Radion

#endif // RADION_MINI_RENDERER_H
