#ifndef RADION_MATERIAL_H
#define RADION_MATERIAL_H

#include "GPU.h"
#include "Types.h"

#include "Math.h"

#include <string>

namespace Radion
{

enum MaterialFlags : u32
{
    MaterialCastShadow = 1 << 0,
    MaterialReceiveShadow = 1 << 1,
    MaterialTwoSided = 1 << 2,
    MaterialAlphaTest = 1 << 3,
    MaterialRefraction = 1 << 4,
    MaterialReflection = 1 << 5,
    MaterialSkinned = 1 << 6,
    MaterialNoDepthWrite = 1 << 7,
    MaterialAnimated = 1 << 8,

    // Shaded by lit.vert/lit.frag (sun cascades, local lights, shadow atlas); otherwise the unlit path.
    MaterialLit = 1 << 9,

    // Compiles lit.frag with LANDSCAPE_REGIONS: the vertex's four weights pick among four textures. Only Landscape sets this (other meshes have no weight attribute).
    MaterialLandscape = 1 << 10,

    // A flat mirror, otherwise ordinary Lit geometry. Compiles lit.frag with HAS_MIRROR, sampling the planar reflection Renderer::executeReflection() renders for water.
    // One plane per frame: the first MaterialMirror packet in the opaque list wins (see Renderer::executeReflection).
    MaterialMirror = 1 << 11,

    // Offsets vUV by view direction via SlotHeight (lit.frag HAS_PARALLAX path); with no SlotHeight bound the UV is untouched.
    MaterialParallax = 1 << 12,

    // SlotSurface is a glTF metallic-roughness texture (G = roughness, B = metalness, times uRoughness/uMetallic) instead of the legacy specular map (R, roughness = 1 - specular).
    MaterialMetallicRoughnessMap = 1 << 13,

    // SlotSurface is a glTF KHR_materials_pbrSpecularGlossiness map (RGB = specular, A = glossiness); exclusive with the two readings above.
    // params.custom0 carries (specularFactor.rgb, glossinessFactor), so no detail map.
    MaterialSpecularGlossinessMap = 1 << 14,

    // Finite heightmap terrain. Slots 0-3 are four albedo layers; SlotColorMap is an optional RGBA splat map, SlotHeight an optional large-scale colour map. Selects TerrainSurface() in lit.frag.
    MaterialTerrain = 1 << 15,

    // Terrain variant: SlotAlbedo is one image stretched over the terrain through uv2, times SlotDetail tiling; params.custom0 = (detail tiles, detail strength).
    // Only meaningful with MaterialTerrain.
    MaterialTerrainClassic = 1 << 16,

    // Voxel meshes store repeated face coordinates in UV0 and the atlas tile origin in UV1; Lit rebuilds the sample UV within the tile so greedy quads repeat the texture.
    MaterialVoxelAtlas = 1 << 17,
};

struct MaterialFlagName
{
    u32 bit;
    const char* name;
};

enum class RenderCategory : u8
{
    Opaque,
    AlphaTest,
    Transparent,
    Refraction
};

enum MaterialPipelinePass : u8
{
    MaterialPipelineForward = 0,
    MaterialPipelineNoTemporal = 1 << 0
};

// The first four slots are fixed and the shader relies on the order. Slots 4-7 are fixed too but bound only under the variant that reads them (Detail under Lit, ColorMap under Landscape, Lightmap with uv2, Height under Parallax).
enum MaterialSlot : u8
{
    SlotAlbedo = 0,
    SlotNormal = 1,
    SlotSurface = 2, // roughness / metalness / occlusion
    SlotEmissive = 3,
    SlotDetail = 4,   // close-up tiling detail map (BindingDetail)
    SlotColorMap = 5, // a landscape's authored colour map (BindingColorMap)
    SlotLightmap = 6, // baked lightmap, sampled through uv2 (BindingLightmap)
    SlotHeight = 7,   // parallax offset source (BindingHeight, MaterialParallax)

    MaterialSlotCount = 8
};

// How texture bytes are read: sRGB colour (albedo, emissive) must be decoded; data textures (normals, roughness, masks, height) are already linear.
enum class ColorSpace : u8
{
    Linear,
    sRGB
};

enum class TextureSource : u8
{
    None = 0,
    Static,       // a single file
    Sequence,     // N files in a Tex2DArray, animated over time
    RenderTarget, // named target, resolved at frame start
};

struct MaterialTexture
{
    TextureHandle texture;
    SamplerHandle sampler;
    // Source path retained so runtime-generated materials can be serialized again.
    std::string file;
    TextureSource source = TextureSource::None;
    u16 layers = 0;     // Sequence only
    u32 targetName = 0; // name hash, RenderTarget only
};

// std140 block of 128 bytes uploaded verbatim; fields up to 'custom' have fixed meaning.
struct MaterialParams
{
    Math::vec4 baseColor = Math::vec4(1.0f);
    Math::vec4 emissive = Math::vec4(0.0f);
    Math::vec4 surface = Math::vec4(1.0f, 0.0f, 0.5f, 1.0f); // rough, metal, alphaCut, normalScale
    Math::vec4 uvTransform = Math::vec4(1.0f, 1.0f, 0.0f, 0.0f); // tileU, tileV, offU, offV
    Math::vec4 uvAnim = Math::vec4(0.0f);                        // scrollU, scrollV, rotSpeed, _
    Math::vec4 sequence = Math::vec4(0.0f);                      // frames, fps, loop, interpolate
    Math::vec4 custom0 = Math::vec4(0.0f);
    Math::vec4 custom1 = Math::vec4(0.0f);
};

static_assert(sizeof(MaterialParams) == 128, "block must match the std140 layout in the shader");

enum class Curve : u8
{
    Linear,
    SineWave,
    PingPong,
    Noise
};

// Points at one MaterialParams field: the vec4 index plus which components the curve writes.
struct MaterialAnim
{
    u8 field = 0;
    u8 mask = 0x7;
    Curve curve = Curve::SineWave;
    f32 speed = 1.0f;
    f32 phase = 0.0f;
    Math::vec4 min = Math::vec4(0.0f);
    Math::vec4 max = Math::vec4(1.0f);
};

struct Material
{
    static constexpr u32 MaxAnims = 4;

    // The one place the slot-to-colour-space rule lives; a wrong choice reads as washed-out colour with no visible cause.
    static ColorSpace colorSpaceFor(MaterialSlot slot);
    static ColorSpace colorSpaceFor(MaterialSlot slot, u32 flags);

    u32 flags = MaterialCastShadow | MaterialReceiveShadow;

    BlendMode blend = BlendMode::Opaque;
    CullMode cull = CullMode::Back;

    MaterialTexture textures[MaterialSlotCount];

    MaterialParams params;
    BufferHandle paramsBuffer;
    bool paramsDirty = true;

    MaterialAnim anims[MaxAnims];
    u8 animCount = 0;

    PipelineHandle pipeline;

    std::string name;
    u32 nameHash = 0;
};

// Bindings every forward shader agrees on; BindingMaterial is filled by MaterialManager::sync(), the rest by the drawing technique.
enum UniformBinding : u32
{
    BindingCamera = 0,
    BindingMaterial = 1,

    // Reflection camera view-projection; only water.vert reads it. Set by WaterPass from ReflectionPass's matrix.
    BindingReflectionCamera = 2,
    BindingDirectionalShadow = 3,

    BindingEnvironment = 4,

    BindingLighting = 5,

    // Time and camera near/far for water: near/far linearize the refraction depth, time scrolls noise. See WaterBlock.
    BindingWater = 6,
    BindingTemporal = 7,
};

// Storage buffer bindings, a separate namespace in GL from the uniform ones.
enum StorageBinding : u32
{
    BindingInstances = 0,
    BindingPalettes = 1,

    // Local lights/decals and their shadow matrices; bound only while drawing Lit materials.
    BindingEntities = 2,
    BindingEntityMatrices = 3,
    BindingLightTiles = 4,
};

// Texture unit bindings (a third namespace). Reflection/Refraction are published each frame by their techniques (resolved via AssetManager::resolveRenderTarget()), not stored on the Material.
enum TextureBinding : u32
{
    BindingAlbedo = 0,
    BindingReflection = 1,
    BindingRefraction = 2,
    BindingDetail = 3,
    BindingAmbientOcclusion = 4,
    BindingDirectionalShadowMap = 5,
    // Same atlas bound again without depth comparison, for the penumbra blocker search.
    BindingDirectionalShadowRaw = 14,

    // Lit pipeline units alias Reflection/Refraction: safe because Water's and Lit's programs are never bound together.
    BindingNormal = BindingReflection,
    BindingSurface = BindingRefraction,

    // Depth half of the scene copy, for water refraction depth.
    BindingRefractionDepth = BindingDetail,
    BindingShadowAtlas = 6,

    // Landscape colour map (SlotColorMap) draped over the terrain; only meaningful under MaterialLandscape's pipeline.
    BindingColorMap = 7,

    // Probe cubemap; frame state. Declared unconditionally by the lit shader, so ForwardPass binds a neutral cube when no probe exists (an unbound samplerCube fails every later draw).
    BindingEnvironmentCube = 8,

    // Emissive map (SlotEmissive): which PART of a surface glows.
    BindingEmissive = 12,

    // Decal arrays: a sampler with no explicit binding lands on unit 0 and collides with albedo, failing every draw, so they get their own units and a neutral array.
    BindingDecalAlbedo = 9,
    BindingDecalNormal = 10,
    BindingDecalSurface = 11,

    // Baked lightmap (SlotLightmap), sampled through uv2 (MeshAttribs::uv2).
    BindingLightmap = 13,

    // MaterialMirror's planar reflection (same texture as kReflectionTargetName), read by lit.frag's HAS_MIRROR path. Own unit because a mirror is a Lit material and needs Normal (unit 1) at the same time.
    BindingMirrorReflection = 14,

    // Parallax offset source (SlotHeight); only meaningful under MaterialParallax.
    BindingHeight = 15,
};

// Names published/resolved through AssetManager's render-target registry.
constexpr const char* kReflectionTargetName = "Reflection";
constexpr const char* kRefractionTargetName = "Refraction";

constexpr const char* kRefractionDepthTargetName = "RefractionDepth";

// Optional shader code a pipeline was compiled with, derived from what the material has attached; ORed into PipelineKey::flags above MaterialFlags (bits 0-8).
enum VariantFlags : u32
{
    VariantHasAlbedo = 1u << 16,
    VariantHasDetail = 1u << 17,
    VariantHasNormal = 1u << 18,
    VariantHasSurface = 1u << 19,
    VariantHasColorMap = 1u << 20,
    VariantHasEmissive = 1u << 21,
    VariantHasLightmap = 1u << 22,

    // SlotDetail bound to a Sequence texture (sampler2DArray, frame blended by time); set alongside VariantHasDetail.
    VariantHasDetailSequence = 1u << 23,

    VariantHasHeight = 1u << 24,
};

// Materials with the same key share a pipeline.
struct PipelineKey
{
    RenderCategory category;
    BlendMode blend;
    CullMode cull;
    u8 pass;
    u32 flags;

    bool operator==(const PipelineKey& other) const;
};

} // namespace Radion

#endif // RADION_MATERIAL_H
