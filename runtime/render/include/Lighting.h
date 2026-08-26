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

// Matches the Lighting UBO lit.frag reads at BindingLighting. Not carried in
// a material's own params: a technique draws through a `const Material*`, and
// this state is the same for every lit material in the frame rather than
// authored per-material.
struct alignas(16) LightingBlock
{
    Math::Vec4 counts = Math::Vec4(0.0f);   // x=entities, y=tiled on, z=tile debug, w=shading debug
    Math::Vec4 tileGrid = Math::Vec4(0.0f); // x=tiles across, y=tiles down
};

// Matches light_culling.comp's own Culling UBO.
struct alignas(16) CullingBlock
{
    Math::Mat4 inverseProjection = Math::Mat4(1.0f);
    Math::Mat4 view = Math::Mat4(1.0f);
    Math::Vec4 screenSize = Math::Vec4(0.0f);       // xy used
    Math::Vec4 tileCountEtc = Math::Vec4(0.0f);     // x,y=tiles, z=entity count, w=2.5D on
};

// What uDebugMode selects in lit.frag.
// The numbers are the ones lit.frag switches on, so the two have to stay in
// step. Roughness/Albedo/AmbientOcclusion existed in the shader long before
// they were named here, which meant reaching them needed a raw cast.
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

    // Only the image-based reflection term, with nothing else added - the
    // fastest way to tell "the probe is wrong" apart from "the material is
    // wrong".
    EnvironmentReflection = 8,

    // Flat shading and nothing else: no textures, no shadows, no local
    // lights, no reflections. Returns before any of that work is done, so a
    // scene too heavy to navigate in the editor still shows its shapes.
    FastShaded = 9
};

// Owns everything a Lit material's shader reads beyond the sun: the entity
// buffer (local lights, sun at index 0 by convention), the shadow atlas
// they share, and the tiled-cull results. The Renderer holds one instance,
// the way it already holds a ShadowPass.
//
// No shadow cache: every tile a light holds this frame is redrawn every
// frame. The reference this was ported from keeps a cache keyed by light
// state and skips redrawing a tile nothing moved - that is a real
// optimisation this class does not have yet, not a simplification of the
// atlas layout or the shading math, both of which are unchanged.
class Lighting final
{
public:
    bool setup();
    void shutdown();

    // Rebuilds the entity list (the sun first, by convention - lit.frag
    // skips it in the per-entity loop and reads it through Environment
    // instead) and redraws the atlas tiles ShadowAtlasLayout assigned this
    // frame. Independent of the scene's own depth texture.
    void prepare(ShadowCasterSource& casters, FrameContext& frame, DepthPass& depthPass,
                 bool renderShadows = true);

    // Appends decals.count() entries after the lights buildEntities() already
    // collected, sharing their entity and matrix budget - a decal that does
    // not fit past RenderList::MaxLights is dropped rather than pushed past
    // it. Call between prepare() and cull(), or before prepare() and it will
    // be overwritten: prepare() clears the entity list from scratch.
    void submitDecals(const DecalSystem& decals);

    // Dispatches the tiled cull against the depth the prepass just wrote.
    // Separate from prepare(): that texture is not ready until after
    // Renderer::executeDepth() has run.
    void cull(FrameContext& frame, TextureHandle sceneDepth, u32 screenWidth, u32 screenHeight);

    ShadowAtlasSettings& atlasSettings()
    {
        return mAtlas.settings;
    }

    // The atlas texture, read-only. For the debug overlay - shading reaches
    // it through FrameContext::shadowAtlas instead.
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
    // 1.0 = every tile got the size its importance asked for; less than that
    // means the atlas overflowed and every tile shrank together to fit.
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

    // Read-only access to the entities buildEntities() produced, with
    // matrixIndex/shadowAtlasMulAdd/shadowFade already filled in by whatever
    // tile the atlas gave each one. VolumetricPass needs the same annotated
    // data the lit shader reads through the SSBO, but for point and rect
    // lights it sets those values as per-draw uniforms on the CPU side rather
    // than reading them in a shader, so it needs the array itself.
    const std::vector<RenderLight>& entities() const
    {
        return mEntities;
    }

    // The shadow view-projections entities() indexes into by matrixIndex.
    // Point lights use six consecutive entries (one per cube face); spot and
    // rect use one.
    const std::vector<Math::Mat4>& matrices() const
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
    std::vector<Math::Mat4> mMatrices;
    u32 mLightsWithoutTile = 0;
    u32 mDroppedDecals = 0;
    bool mExtraSunWarned = false;

    // Rebuilt for each atlas tile pass in turn - see ShadowPass::mShadowList
    // for why one reused list is enough.
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
