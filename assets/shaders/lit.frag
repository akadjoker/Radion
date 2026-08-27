#version 450 core
precision highp float;
precision highp sampler2DShadow;

#define MAX_CASCADES 4

#define ENTITY_TYPE_DIRECTIONALLIGHT 0u
#define ENTITY_TYPE_POINTLIGHT       1u
#define ENTITY_TYPE_SPOTLIGHT        2u
#define ENTITY_TYPE_RECTLIGHT        3u
#define ENTITY_TYPE_DECAL            4u
#define ENTITY_FLAG_LIGHT_CASTING_SHADOW 2u
#define ENTITY_FLAG_DECAL_BASECOLOR_ONLY_ALPHA 8u

struct ShaderEntity
{
    vec3 position;
    float range;

    vec3 direction;
    float coneAngleCos;

    vec3 color;
    float coneAngleScale;

    uint type;
    uint flags;
    int matrixIndex;
    float shadowFade;

    vec4 shadowAtlasMulAdd;
    vec3 rectRight;
    float rectWidth;
    vec3 rectUp;
    float rectHeight;
};

layout(std430, binding = 2) readonly buffer EntityBuffer
{
    ShaderEntity entities[];
};

layout(std430, binding = 3) readonly buffer MatrixBuffer
{
    mat4 entityMatrices[];
};

#define TILE_SIZE 32
#define BUCKETS 8

layout(std430, binding = 4) readonly buffer TileBuffer
{
    uint tilesLuz[];
};

layout(std140, binding = 0) uniform Camera
{
    mat4 uViewProj;
    vec4 uClipPlane;
    vec4 uCameraPos;
    mat4 uView;
};

layout(std140, binding = 1) uniform MaterialBlock
{
    vec4 baseColor;
    vec4 emissive;
    vec4 surface;
    vec4 uvTransform;
    vec4 uvAnim;
    vec4 sequence;
    vec4 custom0;
    vec4 custom1;
};

#define MAX_SHADOW_KERNEL 32

layout(std140, binding = 3) uniform DirectionalShadow
{
    mat4 uShadowMatrix[MAX_CASCADES];
    vec4 uCascadeSplit;
    vec4 uShadowBias4;
    vec4 uShadowNormalBias4;
    vec4 uShadowRangeBegin4;
    vec4 uShadowUvScale[MAX_CASCADES];
    vec4 uSunDirectionAndCount;
    vec4 uShadowSampling;
    vec4 uShadowSampling2;
    vec4 uShadowSampling3;
    vec4 uShadowSoftKernel[MAX_SHADOW_KERNEL];
    vec4 uShadowPenumbraKernel[MAX_SHADOW_KERNEL];
};

layout(std140, binding = 4) uniform Environment
{
    vec4 uSun;
    vec4 uSunColor;
    vec4 uAmbient4;
    vec4 uProbePositionAndMips;
    vec4 uProbeExtentsAndIntensity;
    vec4 uTime; // x = frame time, seconds - see EnvironmentBlock.h
};

// Per-frame entity/tile counts. Not carried in MaterialBlock's custom0/1: a
// technique draws through a `const Material*`, and this state is the same
// for every lit material in the frame rather than authored per-material, so
// it gets its own small block instead of one material speaking for all.
layout(std140, binding = 5) uniform Lighting
{
    vec4 uLightingCounts;   // x = entity count, y = tiled cull on, z = tile debug, w = debug mode
    vec4 uLightingTileGrid; // x = tiles across, y = tiles down, z = decals on
};

#define uLightDir           uSun.xyz
#define uLightColor         uSunColor.rgb
#define uAmbient            uAmbient4.rgb
#define uLightmapIntensity  uAmbient4.w
#define uLightmapShadowLift uTime.y
#define uViewPos            uCameraPos.xyz
#define uCascadeCount       int(uSunDirectionAndCount.w)
#define uShadowPixelSize    uShadowSampling.x
#define uSoftShadowScale    uShadowSampling.y
#define uSoftShadowAngle    uShadowSampling.z
#define uBlendCascades      int(uShadowSampling.w)
#define uSoftSamples        int(uShadowSampling2.x)
#define uPenumbraSamples    int(uShadowSampling2.y)
#define uShadowOpacity      uShadowSampling2.z
#define uTaaFrameCount      uShadowSampling2.w
#define uShadowFadeFrom     uShadowSampling3.x
#define uShadowFadeTo       uShadowSampling3.y
#define uAlbedo             baseColor.rgb
#define uRoughness          surface.x
#define uMetallic           surface.y
#define uAlphaCut           surface.z
#define uNormalStrength     surface.w
#define uParallaxScale      custom1.y
#define uProbePosition      uProbePositionAndMips.xyz
#define uProbeMips          uProbePositionAndMips.w
#define uProbeExtents       uProbeExtentsAndIntensity.xyz
#define uProbeIntensity     uProbeExtentsAndIntensity.w

#define uEntityCount        int(uLightingCounts.x)
#define uTiled              int(uLightingCounts.y)
#define uDebugTiles         int(uLightingCounts.z)
#define uDebugMode          int(uLightingCounts.w)
#define uTileCount          ivec2(uLightingTileGrid.xy)
#define uDecalsOn           int(uLightingTileGrid.z)

layout(binding = 0) uniform sampler2D uAlbedoTex;
layout(binding = 1) uniform sampler2D uNormalTex;
layout(binding = 2) uniform sampler2D uSpecTex;
#if (defined(HAS_PARALLAX) || defined(TERRAIN_SURFACE)) && defined(HAS_HEIGHT)
layout(binding = 15) uniform sampler2D uHeightTex;
#endif
#ifdef HAS_DETAIL
#ifdef DETAIL_SEQUENCE
layout(binding = 3) uniform sampler2DArray uDetailTex;
#else
layout(binding = 3) uniform sampler2D uDetailTex;
#endif
#endif
#ifdef HAS_COLORMAP
layout(binding = 7) uniform sampler2D uColorMapTex;
#endif

#ifdef HAS_ALBEDO
#define uHasTexture 1
#else
#define uHasTexture 0
#endif
#ifdef HAS_NORMAL
#define uHasNormalMap 1
#else
#define uHasNormalMap 0
#endif
#ifdef HAS_SURFACE
#define uHasSpecMap 1
#else
#define uHasSpecMap 0
#endif

#ifdef HAS_EMISSIVE
layout(binding = 12) uniform sampler2D uEmissiveTex;
#endif
#ifdef HAS_LIGHTMAP
layout(binding = 13) uniform sampler2D uLightmapTex;
#endif
layout(binding = 4) uniform sampler2D uAOTex;
// Always bound, even with no probe in the frame: ForwardPass puts a 1x1 black
// cube here instead, because a declared samplerCube left unbound fails every
// draw after it.
layout(binding = 8) uniform samplerCube uEnvironmentCube;
layout(binding = 5) uniform sampler2DShadow uShadowMap;
layout(binding = 14) uniform sampler2D uShadowMapRaw;
layout(binding = 6) uniform sampler2DShadow uShadowAtlas;

#ifdef HAS_MIRROR
// Same binding WaterPass's own uReflectionVP uses (water.vert) - the two
// pipelines never bind at once, so ForwardPass rewrites this slot with its
// own small buffer before drawing a Mirror batch, same data either way:
// Renderer::executeReflection()'s mirrored view-projection for this frame's
// one plane.
layout(std140, binding = 2) uniform ReflectionCamera
{
    mat4 uMirrorViewProj;
};
layout(binding = 14) uniform sampler2D uMirrorReflectionTex;
#endif

// Neither is a real uniform: this engine has no per-location glUniform call,
// only UBO/SSBO/texture-unit binding (see AssetTexture.cpp's own texture-only
// convention), so both are derived from the textures that already carry the
// answer - the same way uAmbientOcclusion's size stands in for the screen
// size in unlit.frag.
#define uScreenSize          (vec2(textureSize(uAOTex, 0)) * 2.0)
#define uShadowAtlasTexelSize (1.0 / vec2(textureSize(uShadowAtlas, 0)))

#ifdef DETAIL_SEQUENCE
// sequence = (frameCount, fps, loop, interpolate).
vec3 sampleDetailSequence(vec2 uv)
{
    float frameCount = max(sequence.x, 1.0);
    float looping = sequence.z;
    float f = uTime.x * sequence.y;
    f = looping > 0.5 ? mod(f, frameCount) : min(f, frameCount - 1.0);
    float f0 = floor(f);
    vec3 a = texture(uDetailTex, vec3(uv, f0)).rgb;
    if (sequence.w < 0.5)
        return a;
    float f1 = looping > 0.5 ? mod(f0 + 1.0, frameCount) : min(f0 + 1.0, frameCount - 1.0);
    vec3 b = texture(uDetailTex, vec3(uv, f1)).rgb;
    return mix(a, b, fract(f));
}
#endif

in vec3  vWorldPos;
in vec3  vNormal;
in vec2  vUV;
in vec2  vUV2;
#ifndef LANDSCAPE_REGIONS
in vec4  vColor;
#endif
in float vViewDepth;
in vec4  vTangent;
#ifndef NO_TEMPORAL
in vec2  vMotionNDC;
#endif

#ifdef LANDSCAPE_REGIONS
// The four-region blend, as opposed to a landscape with one base texture:
// only with the slope or low-altitude region bound is there a region set to
// blend at all. It also settles what two shared registers mean here -
// SlotDetail is the high-altitude REGION rather than a close-up detail map,
// and custom0 is the draped colour map's (centre, size, strength) rather
// than the detail map's (tiles, strength). Named once because three
// different places downstream have to agree on it.
#if defined(HAS_NORMAL) || defined(HAS_SURFACE)
#define LANDSCAPE_MULTI_REGION 1
#endif

// No new texture units: this reuses the same slots (albedo/normal/surface)
// ForwardPass already binds for every material, valid or not - a landscape's
// region textures go into SlotAlbedo/SlotNormal/SlotSurface instead of what
// those slots normally hold (colour, normal map, roughness map). The
// HAS_NORMAL/HAS_SURFACE guards below are the same ones the ordinary Lit path
// uses, and they end up true here for the same reason: variantFlagsOf() turns
// them on whenever the matching slot carries a valid texture. Bind only
// SlotAlbedo for a plain single-texture chunk - optionally with SlotDetail as
// an ordinary close-up detail map, and SlotColorMap as a second tiled texture
// for the ground past that single texture's own footprint (see
// LandscapeAlbedo() below). Bind SlotNormal and/or SlotSurface too for the
// full four-region blend instead, where SlotDetail/SlotColorMap then mean the
// high-altitude region and the large-scale colour map respectively.
in vec4 vWeights;

// Planar (top-down) UV rather than the chunk's own, so texel density stays the
// same everywhere and does not stretch with chunk size. It stretches on
// vertical rock faces instead - the known defect of planar projection;
// triplanar would cost 3x the samples.
vec3 LandscapeAlbedo()
{
#ifndef LANDSCAPE_MULTI_REGION
    // Neither the slope nor the low-altitude region texture is bound, so
    // there is no region set to blend - one base texture (SlotAlbedo), no
    // per-region weighting. Weighting it by regionBase would still dim it
    // wherever slope or altitude climbs (those weights come from the
    // geometry alone, with or without textures to carry them), so this path
    // skips the blend outright. SlotDetail/HAS_DETAIL, if bound, is NOT the
    // high-altitude region here - it is an ordinary close-up detail map,
    // handled by the shared HAS_DETAIL block below like any other Lit
    // material's.
    //
    // A single base is usually one large authored image (a colour map)
    // meant to cover the whole generated patch once, not repeat every few
    // units like a tiled ground texture - so it is centred/sized (custom1.xyz)
    // rather than tiled by world position like the multi-region case below.
    const vec2 baseUV = (vWorldPos.xz - custom1.xy) / max(0.0001, custom1.z) + 0.5;
    vec3 color;
#ifdef HAS_COLORMAP
    // Past the authored patch's own edge there is nothing more of it to
    // show, so SlotColorMap (bound only when this is used) takes over as an
    // ordinary tiled ground texture instead of stretching the last edge
    // pixel of the authored one - custom1.w is its tile scale.
    if (baseUV.x < 0.0 || baseUV.x > 1.0 || baseUV.y < 0.0 || baseUV.y > 1.0)
        color = texture(uColorMapTex, vWorldPos.xz * custom1.w).rgb;
    else
#endif
        color = texture(uAlbedoTex, baseUV).rgb;
#else
    // surface.z is uAlphaCut on every other Lit material; a landscape chunk
    // never alpha-tests, so this reuses the same field as the region UV's
    // tiling scale instead - one more MaterialParams meaning per material
    // kind, the way uRoughness/uMetallic already are.
    const vec2 duv = vWorldPos.xz * uAlphaCut;

    // Normalise: the weights arrive UNNORMALISED from the CPU, on purpose -
    // Wicked's own comment is "blending shader wants unnormalized!". Building
    // them already-normalised on the CPU would need floating point where the
    // generator naturally produces stacked thresholds instead.
    vec4 w = vWeights / max(0.0001, vWeights.x + vWeights.y + vWeights.z + vWeights.w);

    vec3 color = texture(uAlbedoTex, duv).rgb * w.x;
#ifdef HAS_NORMAL
    color += texture(uNormalTex, duv).rgb * w.y;
#endif
#ifdef HAS_SURFACE
    color += texture(uSpecTex, duv).rgb * w.z;
#endif
#ifdef HAS_DETAIL
#ifdef DETAIL_SEQUENCE
    color += sampleDetailSequence(duv) * w.w;
#else
    color += texture(uDetailTex, duv).rgb * w.w;
#endif
#endif
#endif

#if defined(HAS_COLORMAP) && defined(LANDSCAPE_MULTI_REGION)
    // Only in the multi-region branch: HAS_COLORMAP/SlotColorMap there is a
    // large authored texture draped over the whole terrain, not the
    // exterior-ground texture the single-base branch above uses it as.
    // custom0 is (centerX, centerZ, size, strength), custom1.x is the mode
    // (0 = modulate the region blend, 1 = replace it, keeping some of its
    // own detail as "grain") and custom1.y is how much of that grain shows.
    const vec2 mapCenter = custom0.xy;
    const float mapSize = custom0.z;
    const float mapStrength = custom0.w;
    if (mapStrength > 0.0)
    {
        const vec2 cuv = (vWorldPos.xz - mapCenter) / mapSize + 0.5;
        if (cuv.x >= 0.0 && cuv.x <= 1.0 && cuv.y >= 0.0 && cuv.y <= 1.0)
        {
            const vec3 macro = texture(uColorMapTex, cuv).rgb;
            if (custom1.x >= 0.5)
            {
                const vec3 grain = mix(vec3(1.0), color * 2.0, custom1.y);
                color = mix(color, macro * grain, mapStrength);
            }
            else
            {
                color = mix(color, color * macro * 2.0, mapStrength);
            }
        }
    }
#endif
    return color;
}
#endif

#if defined(TERRAIN_SURFACE) && defined(TERRAIN_CLASSIC)
// The terrain's other surface state: one authored image stretched over the
// whole terrain through vUV2, which is 0..1 corner to corner whatever the
// terrain's size, times a detail map tiled custom0.x times across it.
// custom0.y is how much of the detail shows through - the same (tiles,
// strength) pair the ordinary Lit detail path below reads, so a terrain and
// a mesh answer to the same two numbers.
vec3 TerrainSurface(vec3 N)
{
    vec3 color = vec3(1.0);
#ifdef HAS_ALBEDO
    color = texture(uAlbedoTex, vUV2).rgb;
#endif
#ifdef HAS_DETAIL
    const vec2 detailUV = vUV * max(custom0.x, 1.0);
#ifdef DETAIL_SEQUENCE
    const vec3 detail = sampleDetailSequence(detailUV);
#else
    const vec3 detail = texture(uDetailTex, detailUV).rgb;
#endif
    color = mix(color, color * detail * 2.0, clamp(custom0.y, 0.0, 1.0));
#endif
    return color;
}
#elif defined(TERRAIN_SURFACE)
// Finite heightmap terrain. vColor is generated per vertex as
// (normalised height, slope, stable variation, 1), so the rules remain in
// terrain-local space even when the component is moved or rotated.
vec3 TerrainSurface(vec3 N)
{
    const float height01 = vColor.r;
    const float slope = vColor.g;
    const float lowEnd = clamp(custom0.x, 0.001, 0.999);
    const float highStart = clamp(custom0.y, 0.001, 0.999);
    const float slopeStart = clamp(custom0.z, 0.0, 0.999);
    const float slopeEnd = max(slopeStart + 0.001, custom0.w);

    float rock = smoothstep(slopeStart, slopeEnd, slope);
    float low = (1.0 - smoothstep(0.0, lowEnd, height01)) * (1.0 - rock);
    float high = smoothstep(highStart, 1.0, height01) * (1.0 - rock);
    float base = max(0.0, 1.0 - rock - low - high);
    vec4 weights = vec4(base, rock, low, high);

#ifdef HAS_COLORMAP
    vec4 painted = texture(uColorMapTex, vUV2);
    painted /= max(dot(painted, vec4(1.0)), 0.0001);
    weights = mix(weights, painted, clamp(custom1.z, 0.0, 1.0));
#endif
    weights /= max(dot(weights, vec4(1.0)), 0.0001);

    const vec2 layerUV = vUV * uvTransform.xy;
    vec3 layer0 = vec3(1.0);
#ifdef HAS_ALBEDO
    layer0 = texture(uAlbedoTex, layerUV).rgb;
#endif

    vec3 layer1 = layer0;
#ifdef HAS_NORMAL
    // Only cliffs pay for triplanar projection. Flat layers keep one cheap
    // top-down sample while rock avoids the familiar vertical stretching.
    const float rockScale = max(custom1.w, 0.0001);
    vec3 blend = pow(abs(N), vec3(4.0));
    blend /= max(blend.x + blend.y + blend.z, 0.0001);
    layer1 = texture(uNormalTex, vWorldPos.yz * rockScale).rgb * blend.x +
             texture(uNormalTex, vWorldPos.xz * rockScale).rgb * blend.y +
             texture(uNormalTex, vWorldPos.xy * rockScale).rgb * blend.z;
#endif

    vec3 layer2 = layer0;
#ifdef HAS_SURFACE
    layer2 = texture(uSpecTex, layerUV).rgb;
#endif
    vec3 layer3 = layer0;
#ifdef HAS_DETAIL
#ifdef DETAIL_SEQUENCE
    layer3 = sampleDetailSequence(layerUV);
#else
    layer3 = texture(uDetailTex, layerUV).rgb;
#endif
#endif

    vec3 color = layer0 * weights.x + layer1 * weights.y +
                 layer2 * weights.z + layer3 * weights.w;
#ifdef HAS_HEIGHT
    const float macroTiles = max(custom1.x, 0.0001);
    const vec3 macro = texture(uHeightTex, vUV2 * macroTiles).rgb;
    color *= mix(vec3(1.0), macro * 2.0, clamp(custom1.y, 0.0, 1.0));
#endif
    color *= mix(0.92, 1.08, vColor.b);
    return color;
}
#endif

layout(location = 0) out vec4 FragColor;
#ifndef NO_TEMPORAL
layout(location = 1) out vec2 FragVelocity;
layout(location = 2) out float FragReactive;
#endif

const vec3 CASCADE_TINT[MAX_CASCADES] = vec3[MAX_CASCADES](
    vec3(1.0, 0.35, 0.35),
    vec3(0.35, 1.0, 0.35),
    vec3(0.35, 0.55, 1.0),
    vec3(1.0, 1.0, 0.35)
);

vec3 SunAxis()
{
    return -normalize(uSunDirectionAndCount.xyz);
}

float QuickHash(vec2 pos)
{
    const vec3 magic = vec3(0.06711056, 0.00583715, 52.9829189);
    return fract(magic.z * fract(dot(pos, magic.xy)));
}

mat2 DiskRotation()
{
    float r = QuickHash(gl_FragCoord.xy) * 6.28318530718;
    float sr = sin(r);
    float cr = cos(r);
    return mat2(vec2(cr, -sr), vec2(sr, cr));
}

float SamplePcfShadow(float pixelSize, vec3 coord)
{
    if (uSoftSamples == 0)
        return texture(uShadowMap, coord);

    mat2 rot = DiskRotation();
    float avg = 0.0;
    for (int i = 0; i < MAX_SHADOW_KERNEL; ++i)
    {
        if (i >= uSoftSamples) break;
        avg += texture(uShadowMap,
                       vec3(coord.xy + pixelSize * (rot * uShadowSoftKernel[i].xy), coord.z));
    }
    return avg / float(uSoftSamples);
}

float SamplePenumbraShadow(vec3 coord, vec2 texScale)
{
    mat2 rot = DiskRotation();

    float blockerCount = 0.0;
    float blockerAverage = 0.0;
    for (int i = 0; i < MAX_SHADOW_KERNEL; ++i)
    {
        if (i >= uPenumbraSamples) break;
        vec2 suv = coord.xy + (rot * uShadowPenumbraKernel[i].xy) * texScale;
        float d = textureLod(uShadowMapRaw, suv, 0.0).r;
        if (d < coord.z)
        {
            blockerAverage += d;
            blockerCount += 1.0;
        }
    }

    if (blockerCount <= 0.0)
        return 1.0;

    blockerAverage /= blockerCount;
    float penumbra = (coord.z - blockerAverage) / max(blockerAverage, 0.000001);
    texScale *= penumbra;

    float s = 0.0;
    for (int i = 0; i < MAX_SHADOW_KERNEL; ++i)
    {
        if (i >= uPenumbraSamples) break;
        vec2 suv = coord.xy + (rot * uShadowPenumbraKernel[i].xy) * texScale;
        s += texture(uShadowMap, vec3(suv, coord.z));
    }
    return s / float(uPenumbraSamples);
}

vec3 ApplyShadowBias(vec3 baseNormalBias, int idx)
{
    vec3 axis = SunAxis();
    vec3 v = vWorldPos + axis * uShadowBias4[idx];
    vec3 normalBias = baseNormalBias * uShadowNormalBias4[idx];
    normalBias -= axis * dot(axis, normalBias);
    return v + normalBias;
}

float ShadowForSplit(vec3 baseNormalBias, int idx, float blurFactor)
{
    vec3 v = ApplyShadowBias(baseNormalBias, idx);
    vec4 c = uShadowMatrix[idx] * vec4(v, 1.0);
    vec3 coord = c.xyz / c.w;

    if (uSoftShadowAngle > 0.0)
    {
        float rangePos = dot(SunAxis(), v - uViewPos);
        float testRadius = (rangePos - uShadowRangeBegin4[idx]) * uSoftShadowAngle;
        vec2 texScale = uShadowUvScale[idx].xy * testRadius;
        return SamplePenumbraShadow(coord, texScale * uSoftShadowScale);
    }

    float f = blurFactor + (1.0 - blurFactor) * float(uBlendCascades);
    return SamplePcfShadow(uShadowPixelSize * uSoftShadowScale * f, coord);
}

float ShadowFactor(out int outLayer)
{
    if (uCascadeCount <= 0)
    {
        outLayer = 0;
        return 1.0;
    }

    float depthZ = vViewDepth;
    vec3 geoNormal = normalize(vNormal);
    vec3 baseNormalBias = geoNormal * (1.0 - max(0.0, dot(SunAxis(), -geoNormal)));

    int idx;
    if (depthZ < uCascadeSplit.x)      idx = 0;
    else if (depthZ < uCascadeSplit.y) idx = 1;
    else if (depthZ < uCascadeSplit.z) idx = 2;
    else                               idx = 3;
    idx = min(idx, uCascadeCount - 1);
    outLayer = idx;
    float blurFactor =
        idx == 0 ? 1.0 : uCascadeSplit.x / max(uCascadeSplit[idx], 0.000001);

    float shadow = ShadowForSplit(baseNormalBias, idx, blurFactor);

    if (uBlendCascades == 1 && idx < uCascadeCount - 1)
    {
        float edge = uCascadeSplit[idx];
        float pssmBlend = smoothstep(edge - edge * 0.1, edge, depthZ);
        if (pssmBlend > 0.0)
        {
            float blur2 = uCascadeSplit.x / uCascadeSplit[idx + 1];
            float shadow2 = ShadowForSplit(baseNormalBias, idx + 1, blur2);
            shadow = mix(shadow, shadow2, pssmBlend);
        }
    }

    shadow = mix(shadow, 1.0, smoothstep(uShadowFadeFrom, uShadowFadeTo, depthZ));
    return mix(1.0, shadow, uShadowOpacity);
}

void CubeFaceUV(vec3 dir, out int face, out vec2 uv)
{
    vec3 a = abs(dir);
    if (a.x >= a.y && a.x >= a.z)
    {
        if (dir.x > 0.0) { face = 0; uv = vec2(-dir.z, -dir.y) / a.x; }
        else             { face = 1; uv = vec2( dir.z, -dir.y) / a.x; }
    }
    else if (a.y >= a.x && a.y >= a.z)
    {
        if (dir.y > 0.0) { face = 2; uv = vec2( dir.x,  dir.z) / a.y; }
        else             { face = 3; uv = vec2( dir.x, -dir.z) / a.y; }
    }
    else
    {
        if (dir.z > 0.0) { face = 4; uv = vec2( dir.x, -dir.y) / a.z; }
        else             { face = 5; uv = vec2(-dir.x, -dir.y) / a.z; }
    }
    uv = uv * 0.5 + 0.5;
}

vec2 AtlasUV(vec4 mulAdd, vec2 uv, float tileOffset)
{
    return vec2(uv.x + tileOffset, uv.y) * mulAdd.xy + mulAdd.zw;
}

float SamplePointShadowAtlas(ShaderEntity e, vec3 fragPos)
{
    if (e.matrixIndex < 0 || e.shadowFade <= 0.0) return 1.0;

    vec3 toFrag = fragPos - e.position;
    float dist = length(toFrag);
    float cmp = clamp(dist / max(0.0001, e.range), 0.0, 1.0);

    int face; vec2 uv;
    CubeFaceUV(toFrag, face, uv);

    vec2 border = uShadowAtlasTexelSize / max(e.shadowAtlasMulAdd.xy, vec2(0.0001)) * 0.5;
    uv = clamp(uv, border, vec2(1.0) - border);

    vec2 atlasUV = AtlasUV(e.shadowAtlasMulAdd, uv, float(face));
    float shadow = texture(uShadowAtlas, vec3(atlasUV, cmp));
    return mix(1.0, shadow, e.shadowFade);
}

float SampleSpotShadowAtlas(ShaderEntity e, vec3 fragPos)
{
    if (e.matrixIndex < 0 || e.shadowFade <= 0.0) return 1.0;

    vec4 lp = entityMatrices[e.matrixIndex] * vec4(fragPos, 1.0);
    if (lp.w <= 0.0) return 1.0;
    vec3 uv = (lp.xyz / lp.w) * 0.5 + 0.5;
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0 || uv.z < 0.0 || uv.z > 1.0)
        return 1.0;

    vec2 border = uShadowAtlasTexelSize / max(e.shadowAtlasMulAdd.xy, vec2(0.0001)) * 0.5;
    uv.xy = clamp(uv.xy, border, vec2(1.0) - border);

    vec2 atlasUV = AtlasUV(e.shadowAtlasMulAdd, uv.xy, 0.0);
    float shadow = texture(uShadowAtlas, vec3(atlasUV, uv.z));
    return mix(1.0, shadow, e.shadowFade);
}

const float PI = 3.14159265359;

float DistributionGGX(float NdotH, float roughness)
{
    float a = roughness * roughness;
    float a2 = a * a;
    float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / max(1e-7, PI * d * d);
}

float GeometrySmith(float NdotV, float NdotL, float roughness)
{
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    float ggxV = NdotV / (NdotV * (1.0 - k) + k);
    float ggxL = NdotL / (NdotL * (1.0 - k) + k);
    return ggxV * ggxL;
}

vec3 FresnelSchlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

vec3 BRDF(vec3 N, vec3 V, vec3 L, vec3 albedo, float roughness, float metallic)
{
    float NdotL = max(dot(N, L), 0.0);
    if (NdotL <= 0.0) return vec3(0.0);

    float NdotV = max(dot(N, V), 1e-4);
    vec3 H = normalize(V + L);
    float NdotH = max(dot(N, H), 0.0);
    float VdotH = max(dot(V, H), 0.0);

    vec3 F0 = mix(vec3(0.04), albedo, metallic);

    float D = DistributionGGX(NdotH, roughness);
    float G = GeometrySmith(NdotV, NdotL, roughness);
    vec3  F = FresnelSchlick(VdotH, F0);

    vec3 specular = (D * G * F) / max(1e-7, 4.0 * NdotV * NdotL);

    vec3 kD = (vec3(1.0) - F) * (1.0 - metallic);

    return (kD * albedo / PI + specular) * NdotL;
}

// Karis' analytic fit for the second half of the split-sum approximation -
// the reference uses this same polynomial (surfaceHF.hlsli:28, from the UE4
// "PBR on Mobile" post) rather than the 2D LUT version of the same idea, so
// there is no lookup texture to generate.
vec3 EnvBRDFApprox(vec3 specularColor, float roughness, float NoV)
{
    const vec4 c0 = vec4(-1.0, -0.0275, -0.572, 0.022);
    const vec4 c1 = vec4(1.0, 0.0425, 1.04, -0.04);
    vec4 r = roughness * c0 + c1;
    float a004 = min(r.x * r.x, exp2(-9.28 * NoV)) * r.x + r.y;
    vec2 AB = vec2(-1.04, 1.04) * a004 + r.zw;
    return specularColor * AB.x + AB.y;
}

// Image-based specular: what the surroundings put back into the eye. This is
// what makes metal read as metal - a car body has almost no diffuse response,
// so without it a metallic material is just dark with a sun highlight.
//
// The mip chain IS the roughness axis (see EnvironmentProbe): mip 0 mirrors,
// the last mip is nearly diffuse.
vec3 EnvironmentReflection(vec3 N, vec3 V, vec3 albedo, float roughness, float metallic)
{
    if (uProbeIntensity <= 0.0)
        return vec3(0.0);

    vec3 R = reflect(-V, N);

    // Parallax correction, the reference's EnvironmentReflection_Local
    // (lightingHF.hlsli:581-587): intersect the reflection ray with the
    // probe's box and re-aim it at the hit point, so a wall reflects from
    // where it actually is instead of from infinitely far away. Extents all
    // zero means no volume was given - that is the reference's _Global path,
    // which samples R straight.
    if (any(greaterThan(uProbeExtents, vec3(0.0))))
    {
        // The box as the unit cube: dividing by the half-extents puts the
        // shading point in [-1,1] and scales the ray to match.
        vec3 clipSpacePos = (vWorldPos - uProbePosition) / uProbeExtents;
        vec3 rayLS = R / uProbeExtents;
        vec3 firstPlane = (vec3(1.0) - clipSpacePos) / rayLS;
        vec3 secondPlane = (vec3(-1.0) - clipSpacePos) / rayLS;
        vec3 furthestPlane = max(firstPlane, secondPlane);
        float distanceToWall = min(furthestPlane.x, min(furthestPlane.y, furthestPlane.z));
        R = (vWorldPos + R * distanceToWall) - uProbePosition;
    }

    float mip = roughness * max(uProbeMips - 1.0, 0.0);
    vec3 environment = textureLod(uEnvironmentCube, R, mip).rgb;

    vec3 F0 = mix(vec3(0.04), albedo, metallic);
    float NoV = max(dot(N, V), 1e-4);
    return environment * EnvBRDFApprox(F0, roughness, NoV) * uProbeIntensity;
}

#if defined(HAS_PARALLAX) && defined(HAS_HEIGHT)
// Offset-limiting parallax (single sample, no occlusion/self-shadowing) -
// the height at the unshifted UV stands in for the height at the real one,
// which the map's own slope error near silhouette-grazing angles.
// Good enough close to a surface at a shallow angle; a POM pass (marching
// several samples for that self-occlusion) would be a separate, heavier
// path on top of this one.
vec2 ParallaxOffset(vec3 N, vec4 tangent, vec2 uv, vec3 viewWorld, float scale)
{
    vec3 T = normalize(tangent.xyz - N * dot(N, tangent.xyz));
    vec3 B = cross(N, T) * tangent.w;
    vec3 viewTS = normalize(vec3(dot(viewWorld, T), dot(viewWorld, B), dot(viewWorld, N)));
    float height = texture(uHeightTex, uv).r;
    return uv - (viewTS.xy / max(viewTS.z, 0.1)) * (height * scale);
}
#endif

vec3 ApplyNormalMap(vec3 N, vec4 tangent, vec2 uv, float strength)
{
    vec3 Traw = tangent.xyz - N * dot(N, tangent.xyz);
    if (dot(Traw, Traw) < 0.00000001)
        return N;
    vec3 T = normalize(Traw);
    vec3 B = cross(N, T) * tangent.w;
    vec2 rg = texture(uNormalTex, uv).rg * 2.0 - 1.0;
    rg *= strength;
    vec3 n = vec3(rg, sqrt(max(0.0, 1.0 - dot(rg, rg))));
    return normalize(mat3(T, B, N) * n);
}

float RangeAttenuation(float dist2, float range)
{
    float distRatio2 = dist2 / max(0.0001, range * range);
    float falloff = clamp(1.0 - distRatio2, 0.0, 1.0);
    return falloff * falloff;
}

float RectSolidAngle(vec3 P, vec3 p0, vec3 p1, vec3 p2, vec3 p3, out float facing, vec3 L)
{
    vec3 v0 = normalize(p0 - P);
    vec3 v1 = normalize(p1 - P);
    vec3 v2 = normalize(p2 - P);
    vec3 v3 = normalize(p3 - P);
    vec3 n0 = normalize(cross(v0, v1));
    vec3 n1 = normalize(cross(v1, v2));
    vec3 n2 = normalize(cross(v2, v3));
    vec3 n3 = normalize(cross(v3, v0));
    float g0 = acos(clamp(dot(-n0, n1), -1.0, 1.0));
    float g1 = acos(clamp(dot(-n1, n2), -1.0, 1.0));
    float g2 = acos(clamp(dot(-n2, n3), -1.0, 1.0));
    float g3 = acos(clamp(dot(-n3, n0), -1.0, 1.0));
    facing = 0.25 * (max(dot(v0, L), 0.0) + max(dot(v1, L), 0.0) +
                     max(dot(v2, L), 0.0) + max(dot(v3, L), 0.0));
    return clamp(g0 + g1 + g2 + g3 - 2.0 * 3.14159265, 0.0, 6.2831853);
}

layout(binding = 9)  uniform sampler2DArray uDecalAlbedo;
layout(binding = 10) uniform sampler2DArray uDecalNormal;
layout(binding = 11) uniform sampler2DArray uDecalSurface;

void ApplyDecals(vec3 P, inout vec3 N, inout vec3 albedo,
                 inout float roughness, inout float metallic)
{
    if (uDecalsOn == 0) return;

    vec3 P_dx = dFdx(P);
    vec3 P_dy = dFdy(P);

    vec4  accCor   = vec4(0.0);
    vec3  accBump  = vec3(0.0);
    float accBumpA = 0.0;
    vec3  accSurf  = vec3(0.0);
    float accSurfA = 0.0;

    ivec2 tile = ivec2(gl_FragCoord.xy) / TILE_SIZE;
    int tileBase = (tile.y * uTileCount.x + tile.x) * BUCKETS;

    for (int b = 0; b < BUCKETS; ++b)
    {
        uint bucket = (uTiled == 1) ? tilesLuz[tileBase + b] : 0xFFFFFFFFu;
        if (uTiled == 0 && b * 32 >= uEntityCount) break;

        while (bucket != 0u)
        {
            int bit = findLSB(bucket);
            bucket &= ~(1u << uint(bit));
            int i = b * 32 + bit;
            if (i >= uEntityCount) break;

            ShaderEntity e = entities[i];
            if (e.type != ENTITY_TYPE_DECAL) continue;
            if (e.matrixIndex < 0) continue;

            if (accCor.a >= 1.0 && accBumpA >= 1.0 && accSurfA >= 1.0) break;

            mat4 proj = entityMatrices[e.matrixIndex];

            vec3 box = (proj * vec4(P, 1.0)).xyz;
            if (any(greaterThan(abs(box), vec3(1.0)))) continue;

            vec2 uv = box.xy * vec2(0.5, -0.5) + 0.5;

            vec2 dX = (mat3(proj) * P_dx).xy * vec2(0.5, -0.5);
            vec2 dY = (mat3(proj) * P_dy).xy * vec2(0.5, -0.5);

            float camada = float(floatBitsToInt(proj[0][3]));

            float edge = 1.0 - pow(clamp(abs(box.z), 0.0, 1.0), 8.0);

            float slope = (e.coneAngleCos > 0.0)
                        ? pow(clamp(dot(N, e.direction), 0.0, 1.0), e.coneAngleCos)
                        : 1.0;

            vec4 amostra = textureGrad(uDecalAlbedo, vec3(uv, camada), dX, dY);
            float a = e.coneAngleScale * edge * slope * amostra.a;
            if (a <= 0.0) continue;

            vec3 cor = e.color;
            if ((e.flags & ENTITY_FLAG_DECAL_BASECOLOR_ONLY_ALPHA) == 0u)
                cor *= amostra.rgb;

            accCor.rgb += (1.0 - accCor.a) * a * cor;
            accCor.a    = a + (1.0 - a) * accCor.a;

            if (e.rectWidth > 0.0)
            {
                vec2 rg = textureGrad(uDecalNormal, vec3(uv, camada), dX, dY).rg * 2.0 - 1.0;
                vec3 dirRight = normalize(vec3(proj[0][0], proj[1][0], proj[2][0]));
                vec3 dirUp    = -normalize(vec3(proj[0][1], proj[1][1], proj[2][1]));
                vec3 bump = (dirRight * rg.x + dirUp * rg.y) * e.rectWidth;

                accBump  += (1.0 - accBumpA) * a * bump;
                accBumpA  = a + (1.0 - a) * accBumpA;
            }

            vec3 surf = textureGrad(uDecalSurface, vec3(uv, camada), dX, dY).rgb;
            accSurf  += (1.0 - accSurfA) * a * surf;
            accSurfA  = a + (1.0 - a) * accSurfA;
        }
    }

    if (accCor.a <= 0.0 && accSurfA <= 0.0) return;

    albedo    = albedo * (1.0 - accCor.a) + accCor.rgb;
    roughness = roughness * (1.0 - accSurfA) + accSurf.r;
    metallic  = metallic * (1.0 - accSurfA) + accSurf.g;

    if (accBumpA > 0.0)
        N = normalize(N + accBump);
}

vec3 EvaluateEntityLights(vec3 P, vec3 N, vec3 V, vec3 albedo, float roughness, float metallic)
{
    vec3 result = vec3(0.0);

    ivec2 tile = ivec2(gl_FragCoord.xy) / TILE_SIZE;
    int tileBase = (tile.y * uTileCount.x + tile.x) * BUCKETS;

    for (int b = 0; b < BUCKETS; ++b)
    {
        uint bucket = (uTiled == 1) ? tilesLuz[tileBase + b] : 0xFFFFFFFFu;
        if (uTiled == 0 && b * 32 >= uEntityCount) break;

        while (bucket != 0u)
        {
            int bit = findLSB(bucket);
            bucket &= ~(1u << uint(bit));
            int i = b * 32 + bit;
            if (i >= uEntityCount) break;

        ShaderEntity e = entities[i];
        if (e.type == ENTITY_TYPE_DECAL) continue;
        if (e.type == ENTITY_TYPE_DIRECTIONALLIGHT)
        {
            if (i == 0) continue;
            vec3 Ldir = -normalize(e.direction);
            if (dot(N, Ldir) <= 0.0) continue;
            result += e.color * BRDF(N, V, Ldir, albedo, roughness, metallic);
            continue;
        }

        vec3 Lvec = e.position - P;
        float dist2 = dot(Lvec, Lvec);
        if (dist2 > e.range * e.range) continue;

        float dist = sqrt(dist2);
        vec3 L = Lvec / max(0.0001, dist);
        float NdotL = max(dot(N, L), 0.0);
        if (NdotL <= 0.0) continue;

        float atten = RangeAttenuation(dist2, e.range);
        float shadow = 1.0;

        if (e.type == ENTITY_TYPE_RECTLIGHT)
        {
            vec3 R = normalize(e.rectRight) * (e.rectWidth * 0.5);
            vec3 U = normalize(e.rectUp) * (e.rectHeight * 0.5);
            vec3 fwd = normalize(e.direction);
            if (dot(P - e.position, fwd) <= 0.0) continue;

            float facing;
            float sa = RectSolidAngle(P,
                e.position - R + U, e.position + R + U,
                e.position + R - U, e.position - R - U, facing, L);
            atten *= sa * facing / 6.2831853;
            shadow = SampleSpotShadowAtlas(e, P);
        }
        else if (e.type == ENTITY_TYPE_SPOTLIGHT)
        {
            float spotCos = dot(-L, normalize(e.direction));
            float coneAtten = clamp((spotCos - e.coneAngleCos) * e.coneAngleScale, 0.0, 1.0);
            if (coneAtten <= 0.0) continue;
            atten *= coneAtten * coneAtten;
            shadow = SampleSpotShadowAtlas(e, P);
        }
        else
        {
            shadow = SamplePointShadowAtlas(e, P);
        }

            result += e.color * atten * shadow * BRDF(N, V, L, albedo, roughness, metallic);
        }
    }

    return result;
}

void main()
{
#ifndef NO_TEMPORAL
    FragVelocity = vMotionNDC;
    FragReactive = 0.0;
#endif
    vec3 N = normalize(vNormal);
    vec3 V = normalize(uViewPos - vWorldPos);

    // Fast shading: out before every texture fetch, shadow lookup and light
    // loop below. A fixed key light so the forms read the same wherever the
    // sun happens to be, which is the whole point while placing geometry.
    if (uDebugMode == 9)
    {
        float keyLight = max(dot(N, normalize(vec3(0.35, 0.85, 0.4))), 0.0);
        FragColor = vec4(vec3(0.62, 0.64, 0.68) * (0.28 + 0.72 * keyLight), 1.0);
        return;
    }

    // The same authored transform used by the alpha-tested depth variants;
    // keeping colour and shadow silhouettes on identical UVs is essential
    // for tiled/cutout materials, and also makes the Inspector's tiling and
    // offset controls affect ordinary Lit materials as intended.
    vec2 uv = vUV * uvTransform.xy + uvTransform.zw;
#ifdef VOXEL_ATLAS
    vec2 voxelUv = fract(vUV);
    uv = vUV2 + custom0.zw + voxelUv * (custom0.xy - custom0.zw * 2.0);
#endif
#if defined(HAS_PARALLAX) && defined(HAS_HEIGHT)
    uv = ParallaxOffset(N, vTangent, vUV, V, uParallaxScale);
#endif

#ifdef TERRAIN_SURFACE
    vec3 albedo = uAlbedo * TerrainSurface(N);
#elif defined(LANDSCAPE_REGIONS)
    vec3 albedo = uAlbedo * LandscapeAlbedo();
#else
    vec3 albedo = uAlbedo;
    if (uHasTexture == 1)
    {
        vec4 albedoSample = texture(uAlbedoTex, uv);
        albedo *= albedoSample.rgb;
        if (albedoSample.a < uAlphaCut)
            discard;
    }
#endif

#ifndef TERRAIN_SURFACE
    if (uHasNormalMap == 1)
        N = ApplyNormalMap(N, vTangent, uv, uNormalStrength);
#endif

    vec3 L = normalize(-uLightDir);

    int layer = -1;
#ifdef HAS_LIGHTMAP
    // The bake already resolved the sun (direct + its own shadow + a flat
    // ambient term - see LightmapBakePass) into uLightmapTex. Sampling the
    // cascades here on top would be relighting the same sun twice, so this
    // never even looks at them - RECEIVES_SHADOW is not defined for a baked
    // material in the first place (see applyToMaterials()).
    float shadow = 1.0;
#elif defined(RECEIVES_SHADOW)
    float shadow = ShadowFactor(layer);
#else
    // The material says it takes no sun shadow. Skipping the lookup rather
    // than multiplying by one: the cascade sample is the expensive part, and
    // this is also what lets a surface opt out of the acne its own geometry
    // causes without touching the bias for everything else.
    float shadow = 1.0;
#endif

    if (uDebugMode == 1)
    {
        vec3 tint = layer < 0 ? vec3(0.5) : CASCADE_TINT[layer];
        FragColor = vec4(tint * (0.35 + 0.65 * shadow), 1.0);
        return;
    }
    if (uDebugMode == 2)
    {
        FragColor = vec4(vec3(shadow), 1.0);
        return;
    }
    if (uDebugMode == 3)
    {
        FragColor = vec4(N * 0.5 + 0.5, 1.0);
        return;
    }

#if defined(HAS_DETAIL) && !defined(TERRAIN_SURFACE) && !defined(LANDSCAPE_MULTI_REGION)
    // Same close-up blend as unlit.frag's terrain path: custom0.x tiles the
    // detail map tighter than the base UVs, custom0.y is how much of it
    // shows through.
    //
    // Excluded for the multi-region landscape: SlotDetail is the altitude
    // region there, LandscapeAlbedo() has already blended it, and custom0
    // carries the draped colour map's centre/size - reading it here as
    // (tiles, strength) would fold the region in a second time at a
    // strength that is really a world coordinate.
    vec2 detailUV = uv * max(custom0.x, 1.0);
#ifdef DETAIL_SEQUENCE
    vec3 detail = sampleDetailSequence(detailUV);
#else
    vec3 detail = texture(uDetailTex, detailUV).rgb;
#endif
    albedo = mix(albedo, albedo * detail * 2.0, clamp(custom0.y, 0.0, 1.0));
#endif

    float roughness = uRoughness;
    float metallic = uMetallic;
#ifndef TERRAIN_SURFACE
    if (uHasSpecMap == 1)
    {
#ifdef HAS_METALLIC_ROUGHNESS_MAP
        // glTF's own packing (pbrMetallicRoughness.metallicRoughnessTexture):
        // G = roughness, B = metalness. Both still scale the material's own
        // uRoughness/uMetallic - factor 1.0 (the glTF default when a texture
        // is bound) means "use the texture value as-is", same convention the
        // spec itself uses.
        vec3 metallicRoughness = texture(uSpecTex, uv).rgb;
        roughness = clamp(metallicRoughness.g * uRoughness, 0.04, 1.0);
        metallic = clamp(metallicRoughness.b * uMetallic, 0.0, 1.0);
#elif defined(HAS_SPECULAR_GLOSSINESS_MAP)
        // glTF KHR_materials_pbrSpecularGlossiness: RGB = specular colour,
        // A = glossiness, both scaled by custom0's own factors.
        //
        // Re-expressed here in the (albedo, metallic) pair the rest of this
        // shader works in - the image-based lighting below and ApplyDecals()
        // read the same two, so producing one consistent triple keeps every
        // path agreeing, which changing only the direct BRDF would not.
        // A dielectric's F0 is 0.04; how far the specular colour sits above
        // that is what metalness means, and at metallic 1 the F0 that BRDF()
        // builds is the specular colour itself. What this cannot express is
        // a strongly coloured specular on a non-metal, which the parameter
        // pair has no room for and which authored assets do not use.
        // The spec puts the specular colour in sRGB and the glossiness in
        // linear, in the one texture. SlotSurface is sampled linear because
        // a metallic-roughness map has to be, so the RGB half is decoded
        // here instead - exactly, not through a 2.2 power.
        vec4 specularGloss = texture(uSpecTex, uv);
        vec3 encoded = specularGloss.rgb;
        vec3 decoded = mix(encoded / 12.92, pow((encoded + 0.055) / 1.055, vec3(2.4)),
                           step(vec3(0.04045), encoded));
        vec3 specularColor = decoded * custom0.rgb;
        roughness = clamp(1.0 - specularGloss.a * custom0.a, 0.04, 1.0);
        metallic = clamp((max(max(specularColor.r, specularColor.g), specularColor.b) - 0.04) /
                             0.96,
                         0.0, 1.0);
        albedo = mix(albedo, specularColor, metallic);
#else
        float spec = texture(uSpecTex, uv).r;
        roughness = clamp(1.0 - spec, 0.04, 1.0);
#endif
    }
#endif

    ApplyDecals(vWorldPos, N, albedo, roughness, metallic);

    // Geometric specular AA: fold the pixel's normal variance into the
    // roughness so thin-edge highlights stop flickering.
    vec3 normalDx = dFdx(N);
    vec3 normalDy = dFdy(N);
    float normalVariance = 0.25 * (dot(normalDx, normalDx) + dot(normalDy, normalDy));
    roughness = clamp(sqrt(roughness * roughness + min(2.0 * normalVariance, 0.18)), 0.04, 1.0);

    if (uDebugMode == 5) { FragColor = vec4(vec3(roughness), 1.0); return; }
    if (uDebugMode == 6) { FragColor = vec4(albedo, 1.0); return; }
    if (uDebugMode == 7)
    {
        FragColor = vec4(vec3(texture(uAOTex, gl_FragCoord.xy / uScreenSize).r), 1.0);
        return;
    }

#ifdef HAS_LIGHTMAP
    // Substitutes for the sun term below, not an extra multiply on top of
    // it - vUV2 is the bake's own atlas UV, independent of vUV's tiling.
    // uLightmapIntensity scales the bake, uLightmapShadowLift raises its dark
    // end with the scene's own ambient before either hits albedo - both are
    // 0/1 defaults that leave a straight lightmap sample untouched.
    vec3 lm = texture(uLightmapTex, vUV2).rgb * uLightmapIntensity;
    lm += uAmbient * uLightmapShadowLift;
    vec3 color = lm * albedo;
#else
    vec3 color = uLightColor * shadow * BRDF(N, V, L, albedo, roughness, metallic);
#endif
    // Point/spot/etc are never baked (LightmapBakePass only ever traces the
    // one directional sun) - they still light and self-shadow in real time
    // on a baked surface exactly as they would on any other.
    color += EvaluateEntityLights(vWorldPos, N, V, albedo, roughness, metallic);

    float ao = texture(uAOTex, gl_FragCoord.xy / uScreenSize).r;
#ifdef VOXEL_ATLAS
    // Ambient occlusion the voxel mesher baked per vertex, from the blocks
    // around each corner. It belongs with the ambient rather than the sun: the
    // corner is closed to light arriving from the surroundings, while the sun
    // it can still see is already answered by the shadow map.
    ao *= vColor.r;
#endif
    // Occluded by AO like the ambient it belongs with: both are light arriving
    // from the surroundings rather than from a light the frame knows about.
    // Opt-in per material, through the Reflection flag. A probe is captured
    // from ONE point, so away from it the box projection only approximates -
    // on a wide flat floor that shows up as the sky pasted on in a hard-edged
    // patch. Surfaces that should mirror their surroundings ask for it; the
    // rest are not made to pay for an approximation they never wanted.
#ifdef HAS_REFLECTION
    color += EnvironmentReflection(N, V, albedo, roughness, metallic) * ao;
#endif
    // A flat mirror: screen-space projection of the frame's one planar
    // capture (Renderer::executeReflection(), the same texture a water
    // surface reads) instead of the environment cube - a cubemap was
    // captured from one fixed point and reflects this plane's surroundings
    // only approximately, the way HAS_REFLECTION already does; a mirror is
    // the one surface in the scene that gets to be exact instead. custom0.x
    // is how strongly it replaces the shaded colour (0 = none, 1 = full).
    // The Fresnel term only brightens further at a grazing angle - it does
    // NOT dim a face-on view the way water's own (ocean.frag) does: water is
    // meant to show what is under it head-on and only turn mirror-like at a
    // shallow angle, but glass/polished metal reflects strongly at every
    // angle, and a low floor here is exactly what read as a washed-out,
    // barely-there reflection looking straight at the mirror.
#ifdef HAS_MIRROR
    // How much of `color` the mix below actually replaced - the flat
    // ambient term further down has to know, or it adds itself on top of
    // an already-finished mirror colour unconditionally and washes it back
    // out to pale/grey, undoing the mix entirely at mirrorWeight close to 1.
    float mirrorWeight = 0.0;
    {
        // Bump: the same Normal map slot every other Lit material already
        // has, read again here rather than reusing the lighting normal N -
        // N is world-space and already folded into the surface's shading,
        // while this wants the map's own raw XY as a screen-space nudge on
        // the reflection lookup, the same technique ocean.frag's own ripple
        // distortion uses. custom0.z is the Inspector's Bump Strength: 0
        // leaves a perfectly flat mirror untouched.
        vec2 bumpOffset = vec2(0.0);
        if (uHasNormalMap == 1 && custom0.z > 0.0)
            bumpOffset = (texture(uNormalTex, vUV).rg * 2.0 - 1.0) * custom0.z;

        vec4 mirrorClip = uMirrorViewProj * vec4(vWorldPos, 1.0);
        if (mirrorClip.w > 0.0)
        {
            vec2 mirrorUV = clamp(mirrorClip.xy / mirrorClip.w * 0.5 + 0.5 + bumpOffset,
                                  vec2(0.002), vec2(0.998));
            // A polished mirror (roughness 0) samples level 0, sharp. A
            // frosted one blurs toward the coarsest mip Renderer::
            // executeReflection() generated - textureQueryLevels() asks the
            // sampler how many exist rather than this shader assuming a
            // fixed reflection-target resolution.
            float mirrorLod = roughness * float(textureQueryLevels(uMirrorReflectionTex) - 1);
            vec4 mirrorSample = textureLod(uMirrorReflectionTex, mirrorUV, mirrorLod);
            float facing = clamp(dot(N, V), 0.0, 1.0);
            float fresnel = mix(0.9, 1.0, pow(1.0 - facing, 3.0));
            mirrorWeight = mirrorSample.a * fresnel * custom0.x;
            color = mix(color, mirrorSample.rgb, mirrorWeight);
        }
    }
#endif
    // Scaled by (1 - metallic) for the same reason kD is inside BRDF(): a
    // metal has no diffuse reflectance at all, its whole response is the
    // specular term above. Adding flat ambient to it turned a mirror into a
    // white ball with a reflection painted on top. Skipped when HAS_LIGHTMAP:
    // the bake's own flat ambient term already covers this, adding this one
    // too would double it.
    //
    // That factor only holds while the reflection it stands for was actually
    // added. Two ways it is not: the material never asked for it (no
    // HAS_REFLECTION), or no probe is bound (uProbeIntensity 0 - what every
    // environment capture sets, reflections being single bounce). In either
    // case the specular never arrives and the same factor leaves metal at
    // pure black - a metallic material with the flag off rendered black on
    // screen, and every metal rendered black inside every other object's
    // reflection. The flat ambient is the crudest stand-in for exactly the
    // light the probe would have carried, so whenever none arrives the metal
    // keeps all of it. Gating on both is also what keeps one object shaded
    // the same way on screen and inside a capture.
#ifndef HAS_LIGHTMAP
#ifdef HAS_REFLECTION
    float ambientWeight = uProbeIntensity > 0.0 ? (1.0 - metallic) : 1.0;
#else
    float ambientWeight = 1.0;
#endif
#ifdef HAS_MIRROR
    // The mix above already replaced mirrorWeight's worth of `color` with
    // the reflection itself, which already carries whatever ambient light
    // that part of the scene has - adding this on top of it a second time,
    // full-strength, is the flat wash that made a fresh mirror look pale
    // and grey instead of like a mirror.
    ambientWeight *= (1.0 - mirrorWeight);
#endif
    color += uAmbient * albedo * ao * ambientWeight;
#endif

    if (uDebugMode == 8)
    {
        // Black on a material without the flag, which is the answer this view
        // is being asked for: nothing of what is on screen came from a probe.
#ifdef HAS_REFLECTION
        FragColor = vec4(EnvironmentReflection(N, V, albedo, roughness, metallic), 1.0);
#else
        FragColor = vec4(0.0, 0.0, 0.0, 1.0);
#endif
        return;
    }

    // Emissive: light the surface gives off, added after everything that
    // lights it FROM somewhere - it takes no shadow, no ambient occlusion and
    // no normal, because none of those apply to a surface that is the source.
    //
    // Values above 1 are the point, not an accident: the scene target is HDR
    // and the bloom pass only picks up what is brighter than its threshold, so
    // a tail light authored at (4, 0.2, 0.2) gets a halo where (1, 0, 0) just
    // reads as flat red.
    //
    // What this does NOT do is light anything else - the ground under a glowing
    // tail light stays unlit. A real point light next to it is what does that.
    vec3 emissiveTerm = emissive.rgb * emissive.w;
#ifdef HAS_EMISSIVE
    // The map says WHERE, so a body panel can carry the light without the
    // whole panel glowing.
    emissiveTerm *= texture(uEmissiveTex, uv).rgb;
#endif
    color += emissiveTerm;

    if (uDebugMode == 4)
        color = vec3(uAmbient + uLightColor * shadow * max(dot(N, L), 0.0));

    if (uDebugTiles == 1)
    {
        ivec2 t = ivec2(gl_FragCoord.xy) / TILE_SIZE;
        int base = (t.y * uTileCount.x + t.x) * BUCKETS;
        int n = 0;
        for (int b = 0; b < BUCKETS; ++b)
        {
            uint bk = (uTiled == 1) ? tilesLuz[base + b] : 0xFFFFFFFFu;
            n += bitCount(bk);
        }
        n = min(n, uEntityCount);
        float f = float(n) / max(1.0, float(uEntityCount));
        vec3 c = mix(vec3(0.0, 0.6, 0.1), vec3(0.9, 0.1, 0.0), sqrt(f));
        ivec2 dentro = ivec2(gl_FragCoord.xy) % TILE_SIZE;
        if (dentro.x == 0 || dentro.y == 0) c *= 0.4;
        FragColor = vec4(c, 1.0);
        return;
    }

    FragColor = vec4(color, 1.0);
}
