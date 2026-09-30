#ifndef RADION_SHADOWS_H
#define RADION_SHADOWS_H

#include "RenderList.h"

#include <vector>

namespace Radion
{

static constexpr u32 MaxShadowCascades = 4;

struct DirectionalShadowRegion
{
    u32 x = 0;
    u32 y = 0;
    u32 width = 1;
    u32 height = 1;
};

// Godot's layout for one directional light: one split uses the whole atlas,
// two splits use horizontal halves, and four splits use quadrants.
DirectionalShadowRegion directionalShadowRegion(u32 atlasSize, u32 cascadeCount, u32 cascade);

struct ShadowCamera
{
    Math::mat4 view = Math::mat4(1.0f);
    f32 fieldOfView = 60.0f;
    f32 aspect = 16.0f / 9.0f;
    f32 nearPlane = 0.1f;
};

struct CascadeShadowSettings
{
    bool enabled = true;
    u32 count = 4;
    // Resolution of one cascade region; four 2048 cascades give Godot's 4096x4096 atlas.
    u32 resolution = 1024;
    f32 distance = 150.0f;
    bool blend = true;

    u32 quality = 2;
    f32 splitOffset[3] = {0.1f, 0.2f, 0.5f};
    f32 bias = 0.1f;
    f32 normalBias = 2.0f;
    f32 pancakeSize = 20.0f;
    f32 blur = 1.0f;
    f32 fadeStart = 0.8f;
    f32 opacity = 1.0f;
    // Angular diameter of the sun in degrees; above zero switches to the penumbra path (width from blocker distance).
    f32 angularDiameter = 0.0f;

    // Settings scaled to the scene; the defaults above suit a ~120 unit demo radius and look nearly off on larger scenes. Does not fix far-cascade texel density. See Shadows.cpp for the reasoning per value.
    static CascadeShadowSettings sizedForScene(f32 sceneRadius);
    // Solves only `distance` (and its extrusion) so cascade 0 reaches `targetTexelsPerUnit`, keeping the other fields of `base` since they all feed the density.
    static CascadeShadowSettings sizedForCamera(const CascadeShadowSettings& base, f32 sceneRadius,
                                                const ShadowCamera& camera,
                                                const Math::vec3& lightDirection,
                                                f32 targetTexelsPerUnit = 20.0f);
};

struct CascadeShadowData
{
    Math::mat4 viewProjection[MaxShadowCascades];
    // Same volume with the near plane pushed toward the sun, so culling keeps geometry only the pancake clamp can flatten in.
    Math::mat4 cullViewProjection[MaxShadowCascades];
    // World to atlas UV, split rect and NDC-to-UV bias already folded in.
    Math::mat4 shadowMatrix[MaxShadowCascades];
    f32 splits[MaxShadowCascades]{};
    f32 halfExtents[MaxShadowCascades]{};
    f32 shadowBias[MaxShadowCascades]{};
    f32 shadowNormalBias[MaxShadowCascades]{};
    f32 rangeBegin[MaxShadowCascades]{};
    Math::vec2 uvScale[MaxShadowCascades]{};
    f32 texelSize[MaxShadowCascades]{};
    f32 fadeFrom = 0.0f;
    f32 fadeTo = 0.0f;
    f32 softShadowScale = 1.0f;
    // Convex hull of the camera slice and its extrusion toward the sun; casters outside it cannot shadow a visible receiver.
    std::vector<Plane> casterPlanes[MaxShadowCascades];
    u32 count = 0;
};

class CascadeShadowCalculator final
{
public:
    CascadeShadowSettings settings;
    bool update(const ShadowCamera& camera, const Math::vec3& lightDirection,
                CascadeShadowData& output) const;
};

struct ShadowAtlasSettings
{
    u32 size = 4096;
    u32 maximumTileSize = 512;
    u32 minimumTileSize = 64;
    f32 volumetricPriority = 4.0f;
    f32 pointPriority = 2.0f;
    bool point = true;
    bool spot = true;

    // Depth bias for point-light tiles, written into the linear distance depth_point.frag outputs (glPolygonOffset does not affect gl_FragDepth writes).
    f32 pointBias = 0.003f;

    // Depth bias for spot/rect tiles via glPolygonOffset (they draw through depth.frag); point lights do not use these.
    f32 biasSlope = 2.5f;
    f32 biasConstant = 8.0f;
};

struct ShadowTile
{
    u32 lightIndex = 0;
    u32 x = 0;
    u32 y = 0;
    u32 size = 0;
    u32 faceCount = 1;
    f32 importance = 0.0f;
};

class ShadowAtlasLayout final
{
public:
    ShadowAtlasSettings settings;
    void update(const Math::vec3& cameraPosition, const std::vector<RenderLight>& lights);

    const std::vector<ShadowTile>& tiles() const
    {
        return mTiles;
    }
    f32 scale() const
    {
        return mScale;
    }

private:
    std::vector<ShadowTile> mTiles;
    f32 mScale = 1.0f;
};

} // namespace Radion

#endif // RADION_SHADOWS_H
