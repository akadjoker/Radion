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

    // Shaded by lit.vert/lit.frag - the sun with cascades, the local
    // lights out of the entity buffer, and the shadow atlas. Without it a
    // material takes the unlit path.
    MaterialLit = 1 << 9,

    // Compiles lit.frag with LANDSCAPE_REGIONS: the vertex's four weights pick
    // among four textures instead of sampling one through uAlbedoTex. Only
    // Landscape sets this - an ordinary mesh has no weight attribute for the
    // shader to read.
    MaterialLandscape = 1 << 10,

    // A flat mirror, ordinary Lit geometry otherwise - not Water/Ocean's own
    // pipeline. Compiles lit.frag with HAS_MIRROR, which samples the same
    // planar reflection Renderer::executeReflection() renders for a water
    // surface, screen-space projected the same way (ocean.frag's own
    // technique). Renderer picks the first MaterialMirror packet it finds
    // in the frame's opaque list as ITS plane too, alongside water - one
    // plane per frame, same "first one wins" the water path already
    // documents (Renderer::executeReflection).
    MaterialMirror = 1 << 11,

    // Offsets vUV by the view direction before Albedo/Normal/Surface sample
    // it, using SlotHeight - lit.frag's HAS_PARALLAX path. Needs SlotHeight
    // bound to do anything; the flag alone with no texture leaves the UV
    // untouched, same as MaterialReflection with no probe in range.
    MaterialParallax = 1 << 12,

    // SlotSurface is read as a glTF-style metallic-roughness texture
    // (G channel = roughness, B channel = metalness, both still multiplied
    // by the material's own uRoughness/uMetallic scalars) instead of the
    // legacy specular map lit.frag otherwise reads there (R channel,
    // roughness = 1 - specular). A material never means both at once from
    // the same texture.
    MaterialMetallicRoughnessMap = 1 << 13,

    // SlotSurface is read as a glTF KHR_materials_pbrSpecularGlossiness map:
    // RGB = specular colour, A = glossiness. Mutually exclusive with the two
    // readings above - the three are different packings of the same slot.
    // params.custom0 carries (specularFactor.rgb, glossinessFactor); a
    // material using this never also uses a detail map, which is the other
    // claimant on custom0.
    MaterialSpecularGlossinessMap = 1 << 14,

    // Finite heightmap terrain. Slots 0-3 are four albedo layers rather than
    // the ordinary albedo/normal/surface/detail meanings; SlotColorMap is an
    // optional RGBA splat map and SlotHeight an optional large-scale colour
    // map. It keeps the shared Lit lighting path, but selects TerrainSurface()
    // in lit.frag.
    MaterialTerrain = 1 << 15,

    // The terrain's other surface state. Instead of blending four layers by
    // height and slope, SlotAlbedo is one authored image stretched over the
    // whole terrain through uv2, multiplied by SlotDetail tiled across it -
    // params.custom0 carries (detail tiles, detail strength), the same pair
    // the ordinary Lit detail path reads. Only meaningful with
    // MaterialTerrain: alone it does nothing.
    MaterialTerrainClassic = 1 << 16,

    // A voxel mesh stores repeated face coordinates in UV0 and the atlas tile
    // origin in UV1. Lit reconstructs the sample UV within that tile so a
    // greedy quad repeats its block texture instead of stretching it.
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

// ------------------------------------------------------------ slots

// The first four positions in Material::textures are fixed; the rest are
// free per kind. The shader relies on this order. Slots 4-7 have a fixed
// meaning too, just not one every material uses - ForwardPass only binds
// them under the pipeline variant that reads them (Detail under Lit,
// ColorMap under MaterialLandscape, Lightmap when uv2 carries one, Height
// under MaterialParallax).
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

// How the bytes in a texture file are to be read. Colour authored for a
// display (albedo, emissive) is sRGB-encoded and has to be decoded before it
// can be shaded with; data that only looks like colour (normals, roughness,
// masks, height) is already linear and decoding it would corrupt it.
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
    // Source path retained so runtime-generated materials can be serialized
    // again (for example a baked lightmap written by the Bistro demo).
    std::string file;
    TextureSource source = TextureSource::None;
    u16 layers = 0;     // Sequence only
    u32 targetName = 0; // name hash, RenderTarget only
};

// ------------------------------------------------------------ parameters

// std140 block of 128 bytes uploaded verbatim. Every field up to 'custom' has
// a fixed meaning; the render technique decides how custom fields are used.
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

// ------------------------------------------------------------ animation

enum class Curve : u8
{
    Linear,
    SineWave,
    PingPong,
    Noise
};

// Points at one field of MaterialParams: the vec4 index plus which of the four
// components the curve writes.
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

// ------------------------------------------------------------ material

struct Material
{
    static constexpr u32 MaxAnims = 4;

    // The one place the slot-to-colour-space rule lives. Callers never decide
    // this themselves: forgetting once leaves a texture undecoded but still
    // encoded on the way out, which reads as washed-out colour and nothing
    // else in the frame points at the cause.
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

// Bindings every forward shader agrees on. BindingMaterial is filled by
// MaterialManager::sync(); the rest belong to the technique doing the draw.
enum UniformBinding : u32
{
    BindingCamera = 0,
    BindingMaterial = 1,

    // The reflection camera's own view-projection - only water.vert reads
    // this, to project a point through a different camera than the one
    // drawing it. Set by WaterPass from the same matrix ReflectionPass used.
    BindingReflectionCamera = 2,
    BindingDirectionalShadow = 3,

    // The frame's sun and ambient, from the sky. See EnvironmentBlock.h.
    BindingEnvironment = 4,

    // Per-frame entity/tile counts the lit pipeline reads alongside the
    // entity SSBOs below. See Lighting.h.
    BindingLighting = 5,

    // Time and the camera's near/far, which the water surface needs and no
    // other block carries: near/far to linearize the refraction depth it
    // samples, time to scroll its noise. See WaterBlock.
    BindingWater = 6,
    BindingTemporal = 7,
};

// Storage buffer bindings, a separate namespace in GL from the uniform ones.
enum StorageBinding : u32
{
    BindingInstances = 0,
    BindingPalettes = 1,

    // Local lights and decals, and the shadow matrices they index into. Only
    // bound while drawing Lit materials - see Lighting.h and ForwardPass.
    BindingEntities = 2,
    BindingEntityMatrices = 3,
    BindingLightTiles = 4,
};

// Texture unit bindings, a third namespace of its own. Reflection/Refraction
// are not per-material assets like Albedo - they are published every frame by
// whichever technique renders them (ReflectionPass, the scene colour copy),
// resolved by name through AssetManager::resolveRenderTarget() and bound by
// the technique that draws Water/Refraction-category geometry, not by
// anything stored on the Material itself.
enum TextureBinding : u32
{
    BindingAlbedo = 0,
    BindingReflection = 1,
    BindingRefraction = 2,
    BindingDetail = 3,
    BindingAmbientOcclusion = 4,
    BindingDirectionalShadowMap = 5,
    // The same atlas texture bound a second time without depth comparison,
    // so the penumbra blocker search can read raw depth.
    BindingDirectionalShadowRaw = 14,

    // The lit pipeline's own units. Reusing Reflection/Refraction's numbers is
    // safe: those only mean something while WaterPass's program is active, and
    // Normal/Surface only mean something while a Lit material's program is -
    // the two pipelines are never bound at the same time.
    BindingNormal = BindingReflection,
    BindingSurface = BindingRefraction,

    // The depth half of the scene copy the water samples for refraction, so
    // the surface can tell how deep the column under each pixel is.
    BindingRefractionDepth = BindingDetail,
    BindingShadowAtlas = 6,

    // A landscape chunk's colour map (SlotColorMap) - a large authored texture
    // draped over the whole terrain (roads, clearings, an island's painted
    // shoreline) that no slope/altitude rule could invent. Only meaningful
    // while MaterialLandscape's pipeline is bound, same as Normal/Surface
    // above are only meaningful while Lit's is.
    BindingColorMap = 7,

    // The environment probe's cubemap, for image-based reflections. Frame
    // state, not per-material: every Lit surface reflects the same
    // surroundings. Declared unconditionally by the lit shader, so ForwardPass
    // binds a neutral cube when no probe exists - a declared samplerCube left
    // unbound fails every draw after it, the same trap the decal arrays below
    // already carry a warning about.
    BindingEnvironmentCube = 8,

    // The emissive map (SlotEmissive) - which PART of a surface is lit up.
    // Without it an emissive material glows over its whole area, which is
    // wrong for a tail light on a body panel.
    BindingEmissive = 12,

    // Decal arrays. Declared by the lit shader whether or not any decal
    // exists, and a sampler with no explicit binding lands on unit 0 - where
    // it collides with the albedo sampler and fails every draw. They get
    // units of their own, and a neutral array until decals are real.
    BindingDecalAlbedo = 9,
    BindingDecalNormal = 10,
    BindingDecalSurface = 11,

    // A baked lightmap (SlotLightmap), sampled through the mesh's second UV
    // set (MeshAttribs::uv2) instead of the tiling UV0 the base texture uses.
    BindingLightmap = 13,

    // MaterialMirror's planar reflection - the same texture kReflectionTargetName
    // publishes for water, read by lit.frag's own HAS_MIRROR path instead of
    // Water's pipeline. A unit of its own rather than reusing Reflection/
    // Refraction's aliasing trick above: those alias because Water and Lit
    // never bind at once, but a mirror IS a Lit material and wants its own
    // Normal (unit 1) at the same time as this.
    BindingMirrorReflection = 14,

    // Parallax offset source (SlotHeight), sampled before Albedo/Normal/
    // Surface to displace vUV. Only meaningful under MaterialParallax.
    BindingHeight = 15,
};

// Names published/resolved through AssetManager's render-target registry.
constexpr const char* kReflectionTargetName = "Reflection";
constexpr const char* kRefractionTargetName = "Refraction";

// The refraction pass' own depth, for a surface that needs to know how deep
// the column under each of its pixels is.
constexpr const char* kRefractionDepthTargetName = "RefractionDepth";

// Which optional shader code a pipeline was compiled with, on top of `kind`.
// Kept out of MaterialFlags (bits 0-8): those are authored intent, these are
// derived from what the material actually has attached (a texture, e.g.) and
// only exist to pick a compile-time #define. ORed into PipelineKey::flags,
// high enough not to collide.
enum VariantFlags : u32
{
    VariantHasAlbedo = 1u << 16,
    VariantHasDetail = 1u << 17,
    VariantHasNormal = 1u << 18,
    VariantHasSurface = 1u << 19,
    VariantHasColorMap = 1u << 20,
    VariantHasEmissive = 1u << 21,
    VariantHasLightmap = 1u << 22,

    // SlotDetail bound to a Sequence texture, not Static - samples a
    // sampler2DArray and blends a frame by time. Set alongside
    // VariantHasDetail, not instead of it.
    VariantHasDetailSequence = 1u << 23,

    VariantHasHeight = 1u << 24,
};

// What the pipeline resolver consumes; two materials with the same key share
// a pipeline.
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
