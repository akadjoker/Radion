#ifndef RADION_LIGHTMAP_BAKE_PASS_H
#define RADION_LIGHTMAP_BAKE_PASS_H

#include "GPU.h"
#include "Math.h"
#include "Mesh.h"

#include <string>

namespace Radion
{

struct LightmapBakeSettings
{
    // Depth bias against the sun's shadow map, IN WORLD UNITS, or ZERO to let
    // bake() derive it from `biasTexels` below - which is the default, and
    // what every scene should want. Too low and flat surfaces self-shadow in
    // a moire pattern; too high and the shadow slides off the wall casting it.
    f32 bias = 0.0f;

    // What the automatic bias is: a number of shadow-map texels. Scale-free
    // by construction - bake() knows the world size of a texel once it has
    // fitted the sun's frustum (ortho width / shadowResolution), so the same
    // setting lands at centimetres in a room and metres across a city.
    //
    // The alternative, a bias expressed as a fraction of the depth range,
    // silently means whatever the scene's size makes it mean: the same 0.01
    // was well under a unit in a courtyard and 178 units across a city map,
    // where it slid every shadow clean off its caster. Acne is a function of
    // how far apart two shadow texels sample, so texels are the unit the
    // problem is actually in.
    f32 biasTexels = 3.0f;

    // Sky light, for everything the sun does not reach. This pass traces one
    // direct bounce and nothing else, so without it a surface facing away
    // from the sun is pure black - and a flat term everywhere makes a floor,
    // a wall and the underside of a balcony all equally lit, which reads as
    // paint rather than light.
    //
    // Weighted by how much sky the surface actually faces (n.y): a floor gets
    // all of `ambient`, a vertical wall the midpoint between it and
    // `ambientGround`, an overhang's underside all of `ambientGround`. That
    // is a uniform sky dome with a bounce off the ground - the cheapest model
    // that still separates the three, and enough to stop shadowed geometry
    // reading as holes.
    Math::Vec3 ambient = Math::Vec3(0.12f, 0.16f, 0.24f);
    // The ground bounce, as a fraction of `ambient`. Never zero unless a
    // black underside is what you want.
    f32 ambientGround = 0.35f;
    // PCF neighbourhood radius, in shadow map texels. 0 disables the
    // filter and falls back to a single hard compare.
    f32 filterRadius = 1.0f;

    // The sun is not a point - it subtends a small angle in the sky, which
    // is what actually gives real shadows a soft penumbra that widens with
    // distance from the occluder. A bake has no per-frame budget, so unlike
    // the real-time cascades this can afford to re-render the shadow map
    // several times from slightly different angles within that disk and
    // average the result, instead of faking softness with a screen-space
    // blur. 0 disables it (a single sample, same as before).
    f32 sunAngularRadius = 0.0f; // degrees
    u32 sampleCount = 1;

    // Size of the depth map the sun casts into, which is a different question
    // from how many texels the atlas has and used to be answered with the
    // same number. The atlas resolution decides how finely a shadow can be
    // STORED; this decides how finely it was ever COMPUTED - one ortho map
    // stretched over the whole scene, so on a street-sized model its texels
    // are centimetres across and no atlas density recovers an edge lost here.
    // Costs only its own depth texture, and nothing on the readback, so it is
    // usually the cheapest quality left to buy. 0 keeps the old behaviour of
    // matching the lightmap resolution.
    u32 shadowResolution = 0;
};

class LightmapBakePass
{
public:
    ~LightmapBakePass();

    bool bake(MeshHandle mesh, const Math::Mat4& model, const AABB& bounds,
              const Math::Vec3& lightDirection, const Math::Vec3& lightColor,
              u32 resolution = 1024, const LightmapBakeSettings& settings = LightmapBakeSettings());
    bool save(const std::string& filename) const;

    // Switches every material to the lightmap-lit shader: strips the flags
    // that route it through the real-time lit path, binds the baked texture
    // into SlotLightmap, and resolves a pipeline for the new state. Callers
    // still own applying the result to their own renderer/override and
    // persisting mesh/materials - that part is scene-specific, this part
    // (what a baked material looks like) is not, so it stays out of every
    // demo that bakes a lightmap.
    static void applyToMaterials(std::vector<Material>& materials, const VertexLayout& colorLayout,
                                 TextureHandle lightmapTexture, const std::string& lightmapFile);

    TextureHandle texture() const { return mLightmap; }
    void shutdown();

private:
    bool setup(const VertexLayout& layout);
    void destroyResources();

    PipelineHandle mShadowPipeline;
    PipelineHandle mBakePipeline;
    BufferHandle mBlock;
    TextureHandle mShadowMap;
    SamplerHandle mShadowSampler;
    TargetHandle mShadowTarget;
    TextureHandle mLightmap;
    TargetHandle mLightmapTarget;
    VertexLayout mLayout{};
    u32 mResolution = 0;
    u32 mShadowResolution = 0;
};

} // namespace Radion

#endif // RADION_LIGHTMAP_BAKE_PASS_H
