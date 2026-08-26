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
    // Resolution of one cascade region. Four 2048 cascades therefore create
    // the same 4096x4096 directional atlas used by Godot on desktop.
    u32 resolution = 1024;
    f32 lambda = 0.70f;
    f32 distance = 150.0f;
    f32 casterExtrusion = 250.0f;
    f32 depthBiasSlope = 2.5f;
    f32 depthBiasConstant = 6.0f;
    // PCF blur width in world units, constant across cascades: the spread in
    // UV is radius / (2 * halfExtent) per cascade, plus a two-texel floor.
    f32 filterRadiusWorld = 0.5f;
    u32 filterTaps = 8;
    bool stabilize = true;
    bool blend = true;
    bool cullFront = false;
    bool alternateFarCascades = true;

    u32 quality = 2;
    f32 splitOffset[3] = {0.1f, 0.2f, 0.5f};
    f32 bias = 0.1f;
    f32 normalBias = 2.0f;
    f32 pancakeSize = 20.0f;
    f32 blur = 1.0f;
    f32 fadeStart = 0.8f;
    f32 opacity = 1.0f;
    // Angular diameter of the sun in degrees; above zero switches the shader
    // to the penumbra path, whose width comes from the blocker distance.
    f32 angularDiameter = 0.0f;

    // A starting point scaled to the scene instead of these defaults above,
    // which are sized for a courtyard-scale demo (playground's own ~120
    // unit radius) and read as almost switched off on anything bigger.
    // Not a fix for the far cascade's texel density - that is FOV and
    // distance squared, no combination of settings makes it go away, only
    // moves where it starts - just numbers that match the scene instead of
    // fighting it. See Shadows.cpp for the reasoning behind each one.
    static CascadeShadowSettings sizedForScene(f32 sceneRadius);
    // Solves only `distance` (and the extrusion that follows it) so cascade 0
    // reaches `targetTexelsPerUnit`, keeping every other field of `base` -
    // count, resolution, lambda, taps and bias all feed that density, so they
    // have to be the ones already in use rather than this factory's defaults.
    static CascadeShadowSettings sizedForCamera(const CascadeShadowSettings& base, f32 sceneRadius,
                                                const ShadowCamera& camera,
                                                const Math::vec3& lightDirection,
                                                f32 targetTexelsPerUnit = 20.0f);
};

struct CascadeShadowData
{
    Math::mat4 viewProjection[MaxShadowCascades];
    // Same volume with the near plane pushed back towards the sun, so caster
    // culling keeps geometry that only the pancake clamp can flatten in.
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
    // Convex hull of the camera slice and that slice extruded towards the
    // sun. A caster outside it cannot possibly shadow a visible receiver.
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

    // Depth bias for a point light's atlas tiles. Written straight into the
    // linear distance depth_point.frag outputs, since glPolygonOffset has no
    // effect on a fragment shader's own gl_FragDepth write.
    f32 pointBias = 0.003f;

    // Depth bias for spot/rect atlas tiles, applied through glPolygonOffset -
    // unlike pointBias above, these draw through the ordinary depth.frag and
    // do get one. Point lights never use these two.
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
