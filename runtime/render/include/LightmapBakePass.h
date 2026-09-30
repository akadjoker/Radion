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
    // Depth bias against the sun's shadow map in WORLD UNITS, or ZERO to derive it from `biasTexels` (the default).
    // Too low gives moire self-shadowing; too high detaches shadows from their casters.
    f32 bias = 0.0f;

    // Automatic bias in shadow-map texels; scale-free because bake() knows a texel's world size after fitting the sun's frustum.
    // A depth-fraction bias would mean different things at different scene sizes.
    f32 biasTexels = 3.0f;

    // Sky light for surfaces the sun misses (the pass traces one direct bounce only).
    // Weighted by sky facing (n.y): floor gets `ambient`, a vertical wall the midpoint with `ambientGround`, an overhang underside `ambientGround`.
    Math::vec3 ambient = Math::vec3(0.12f, 0.16f, 0.24f);
    // Ground bounce as a fraction of `ambient`.
    f32 ambientGround = 0.35f;
    // PCF radius in shadow-map texels; 0 = single hard compare.
    f32 filterRadius = 1.0f;

    // Sun angular radius: re-renders the shadow map from several angles within that disk and averages, giving a penumbra. 0 = single sample.
    f32 sunAngularRadius = 0.0f; // degrees
    u32 sampleCount = 1;

    // Size of the sun's depth map, separate from the atlas resolution (how finely a shadow is stored); this sets how finely it is computed.
    // 0 matches the lightmap resolution.
    u32 shadowResolution = 0;
};

class LightmapBakePass
{
public:
    ~LightmapBakePass();

    bool bake(MeshHandle mesh, const Math::mat4& model, const AABB& bounds,
              const Math::vec3& lightDirection, const Math::vec3& lightColor,
              u32 resolution = 1024, const LightmapBakeSettings& settings = LightmapBakeSettings());
    bool save(const std::string& filename) const;

    // Switches materials to the lightmap-lit shader: strips real-time lit flags, binds the baked texture into SlotLightmap, resolves a new pipeline.
    // Callers still apply and persist the result themselves.
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
