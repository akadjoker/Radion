#ifndef RADION_LIGHTING_H
#define RADION_LIGHTING_H

#include "FrameContext.h"
#include "RenderList.h"
#include "Shadows.h"

#include <vector>

namespace Radion
{

class DepthPass;
class DecalSystem;

// Matches the Lighting UBO lit.frag reads at BindingLighting; frame-wide, so not carried in a material's params.
struct alignas(16) LightingBlock
{
    Math::vec4 counts = Math::vec4(0.0f);   // x=entities, y=tiled on, z=tile debug, w=shading debug
    Math::vec4 tileGrid = Math::vec4(0.0f); // x=tiles across, y=tiles down
};

// Matches light_culling.comp's own Culling UBO.
struct alignas(16) CullingBlock
{
    Math::mat4 inverseProjection = Math::mat4(1.0f);
    Math::mat4 view = Math::mat4(1.0f);
    Math::vec4 screenSize = Math::vec4(0.0f);       // xy used
    Math::vec4 tileCountEtc = Math::vec4(0.0f);     // x,y=tiles, z=entity count, w=2.5D on
};

// What uDebugMode selects in lit.frag; the numbers must stay in step with the shader.
enum class LightingDebugMode : u8
{
    Lit = 0,
    CascadeIndex = 1,
    ShadowFactor = 2,
    Normals = 3,
    AmbientAndShadow = 4,
    Roughness = 5,
    Albedo = 6,
    AmbientOcclusion = 7,

    // Only the image-based reflection term; tells a bad probe from a bad material.
    EnvironmentReflection = 8,

    // Flat shading only: no textures, shadows, local lights or reflections; returns before doing that work.
    FastShaded = 9
};

// Owns everything a Lit shader reads beyond the sun: the entity buffer (sun at index 0), the shared shadow atlas and the tiled-cull results.
// No shadow cache: every tile a light holds is redrawn every frame.
class Lighting final
{
public:
    bool setup();
    void shutdown();

    // Rebuilds the entity list (sun first; lit.frag reads it through Environment) and redraws the atlas tiles ShadowAtlasLayout assigned this frame.
    void prepare(ShadowCasterSource& casters, FrameContext& frame, DepthPass& depthPass,
                 bool renderShadows = true);

    // Appends decals after the lights, sharing their entity and matrix budget; a decal past RenderList::MaxLights is dropped.
    // Call between prepare() and cull(); prepare() clears the entity list.
    void submitDecals(const DecalSystem& decals);

    // Dispatches the tiled cull against the prepass depth; separate from prepare() because that depth is ready only after Renderer::executeDepth().
    void cull(FrameContext& frame, TextureHandle sceneDepth, u32 screenWidth, u32 screenHeight);

    ShadowAtlasSettings& atlasSettings()
    {
        return mAtlas.settings;
    }

    // Read-only atlas texture, for the debug overlay.
    TextureHandle atlasTexture() const
    {
        return mAtlasTexture;
    }

    bool tiled = true;
    bool use25D = true;
    bool debugTiles = false;
    bool decalsEnabled = true;
    LightingDebugMode debugMode = LightingDebugMode::Lit;

    u32 droppedDecalCount() const
    {
        return mDroppedDecals;
    }

    u32 tilesUsed() const
    {
        return static_cast<u32>(mAtlas.tiles().size());
    }
    // 1.0 = every tile got its requested size; less means the atlas overflowed and all tiles shrank.
    f32 atlasScale() const
    {
        return mAtlas.scale();
    }
    u32 lightsWithoutTile() const
    {
        return mLightsWithoutTile;
    }
    u32 entityCount() const
    {
        return static_cast<u32>(mEntities.size());
    }

    // Entities from buildEntities() with matrixIndex/shadowAtlasMulAdd/shadowFade filled in; VolumetricPass sets these as per-draw uniforms for point and rect lights.
    const std::vector<RenderLight>& entities() const
    {
        return mEntities;
    }

    // Shadow view-projections indexed by matrixIndex; point lights use six consecutive entries, spot and rect one.
    const std::vector<Math::mat4>& matrices() const
    {
        return mMatrices;
    }

private:
    bool createAtlasTexture();
    void destroyAtlasTexture();
    bool ensureCullPipeline();
    void buildEntities(ShadowCasterSource& casters, FrameContext& frame, DepthPass& depthPass,
                       bool renderShadows);

    ShadowAtlasLayout mAtlas;
    std::vector<RenderLight> mEntities;
    std::vector<Math::mat4> mMatrices;
    u32 mLightsWithoutTile = 0;
    u32 mDroppedDecals = 0;
    bool mExtraSunWarned = false;

    // Rebuilt per atlas tile pass; see ShadowPass::mShadowList for why one reused list is enough.
    RenderList mTileList;

    BufferHandle mEntityBuffer;
    BufferHandle mMatrixBuffer;
    BufferHandle mTileBuffer;
    BufferHandle mLightingBlock;
    BufferHandle mCullingBlock;
    u32 mTileBufferCapacity = 0;

    TextureHandle mAtlasTexture;
    SamplerHandle mAtlasSampler;
    TargetHandle mAtlasTarget;
    u32 mAtlasSize = 0;

    PipelineHandle mCullPipeline;

    u32 mTilesX = 0;
    u32 mTilesY = 0;
};

} // namespace Radion

#endif // RADION_LIGHTING_H
