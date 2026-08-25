#version 450 core

// Water deliberately kept close to the RealisticWaterSceneNode model:
// animated bump -> projected refraction/reflection -> Fresnel -> water tint.
// The Gerstner vertex shader supplies only the large, slow surface shape.

in vec3 vWorldPos;
in vec3 vNormal;
in vec2 vUV;
in float vCrest;

out vec4 FragColor;

#include "ocean_uniforms.glsl"

layout(binding = 0) uniform sampler2D uNormalMap1;
layout(binding = 1) uniform sampler2D uFoamTex;
#ifdef OCEAN_HAS_REFLECTION
layout(binding = 2) uniform sampler2D uReflection;
#endif
#ifdef OCEAN_HAS_DEPTH
layout(binding = 3) uniform sampler2D uSceneDepth;
layout(binding = 5) uniform sampler2D uSceneColor;
#endif
layout(binding = 4) uniform samplerCube uSkyCube;

vec3 SkyGradient(vec3 direction)
{
    vec3 sunDirection = normalize(-uLightDir);
    float day = smoothstep(-0.08, 0.25, sunDirection.y);
    vec3 horizon = mix(uAmbient * 0.5, vec3(0.58, 0.72, 0.82), day);
    vec3 zenith = mix(uAmbient * 0.25, vec3(0.12, 0.32, 0.58), day);
    float height = clamp(direction.y, 0.0, 1.0);
    return mix(horizon, zenith, pow(height, 0.65)) * uSkyIntensity;
}

#ifdef OCEAN_HAS_DEPTH
vec3 WorldPosFromDepth(vec2 uv, float depth)
{
    vec4 clip = vec4(uv * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
    vec4 world = uInvViewProj * clip;
    return world.xyz / max(abs(world.w), 1e-5);
}
#endif

void main()
{
    vec3 V = normalize(uViewPos - vWorldPos);
    vec3 macroNormal = normalize(vNormal);
    if (dot(macroNormal, V) < 0.0)
        macroNormal = -macroNormal;

    // The Irrlicht shader scrolls one bump texture. Two differently scaled
    // samples retain that calm flow but hide the obvious repeating tile.
    vec2 flow = normalize(vec2(0.87, 0.49));
    vec2 uv1 = vWorldPos.xz * uNormalScale1 + flow * uTime * uNormalSpeed1 * 0.025;
    vec2 uv2 = vWorldPos.xz * (uNormalScale1 * 0.47)
             - flow.yx * uTime * uNormalSpeed1 * 0.014;

    vec2 bump = vec2(0.0);
    if (uHasNormalMap == 1)
    {
        bump = (texture(uNormalMap1, uv1).rg * 2.0 - 1.0) * 0.70;
        bump += (texture(uNormalMap1, uv2).rg * 2.0 - 1.0) * 0.30;
        bump *= uNormalStrength;
    }

    vec3 detailNormal = normalize(vec3(macroNormal.x + bump.x,
                                       max(0.25, macroNormal.y),
                                       macroNormal.z + bump.y));
    if (dot(detailNormal, V) < 0.0)
        detailNormal = -detailNormal;

    vec2 screenUV = gl_FragCoord.xy / max(uScreenSize, vec2(1.0));
    vec2 distortion = bump * uReflectionDistortion;

    // Refraction is the already rendered scene. Without the depth tier there
    // is no scene colour copy, so the authored deep colour is the fallback.
    vec3 refractionColor = uDeepColor;
    vec3 sceneBackground = vec3(0.0);
    float depthFactor = 1.0;
    float bottomDistance = 1e6;
#ifdef OCEAN_HAS_DEPTH
    // Keep the undistorted pixel too.  This tier is emitted opaque to avoid
    // a read/write feedback loop, so it has to perform the same alpha
    // composition as the reflection-only tier by hand at the end.
    sceneBackground = texture(uSceneColor, screenUV).rgb;
    float bottomDepth = texture(uSceneDepth, screenUV).r;
    bool hasBottom = bottomDepth < 0.999999;
    if (hasBottom)
    {
        vec3 bottomPosition = WorldPosFromDepth(screenUV, bottomDepth);
        bottomDistance = max(0.0, vWorldPos.y - bottomPosition.y);
        depthFactor = clamp(bottomDistance / max(uAbsorptionDistance, 0.001), 0.0, 1.0);

        vec2 refractUV = clamp(screenUV + distortion, vec2(0.002), vec2(0.998));
        if (texture(uSceneDepth, refractUV).r < bottomDepth)
            refractUV = screenUV;

        vec3 sceneColor = texture(uSceneColor, refractUV).rgb;
        vec3 absorptionColor = mix(uShallowColor, uDeepColor, depthFactor);

        // Beer-Lambert-inspired visibility: the bottom is clear at the shore
        // and contributes nothing once the column is deep.  The previous
        // fixed 20% leak kept sampling the HDR sky/scene even at maximum
        // depth, which became a white veil after tone mapping.
        float bottomVisibility = pow(1.0 - depthFactor, 1.6);
        refractionColor = mix(absorptionColor, sceneColor, bottomVisibility * 0.82);
    }
    else
    {
        // Far-plane depth means open water, not a bright object under it.
        // Sampling uSceneColor here reads the sky and incorrectly places it
        // in the refraction as well as in the reflection.
        refractionColor = uDeepColor;
        depthFactor = 1.0;
    }
#endif

    // Start with the real sky so reflection never depends on an empty planar
    // target. Detail normal bends the sky exactly like the bump perturbation
    // bends the two projected maps in the reference implementation.
    vec3 reflectedDirection = reflect(-V, detailNormal);
    vec3 reflectionColor = uHasSkyCube == 1
        ? texture(uSkyCube, reflectedDirection).rgb * uSkyIntensity
        : SkyGradient(reflectedDirection);

#ifdef OCEAN_HAS_REFLECTION
    vec4 reflectedPosition = uReflectionVP * vec4(vWorldPos, 1.0);
    if (reflectedPosition.w > 0.0)
    {
        vec2 reflectionUV = reflectedPosition.xy / reflectedPosition.w * 0.5 + 0.5;
        reflectionUV = clamp(reflectionUV + distortion, vec2(0.002), vec2(0.998));
        vec4 planar = texture(uReflection, reflectionUV);
        // SkyPass writes alpha 1 as well as geometry, so planar.a alone is
        // not a useful strength control. uNormalScale2 is unused by this
        // simplified two-sample bump shader and is exposed by OceanDemo as
        // "Forca reflexo planar" while retaining the existing UBO ABI.
        float planarWeight = planar.a * clamp(uNormalScale2, 0.0, 1.0);
        reflectionColor = mix(reflectionColor, planar.rgb, planarWeight);
    }
#endif

    // Water reflects little when viewed from above and increasingly toward
    // the horizon. A softer curve than Schlick keeps the old demo's readable
    // blue body without producing a white grazing-angle sheet.
    float facing = clamp(dot(macroNormal, V), 0.0, 1.0);
    float fresnel = 0.05 + 0.72 * pow(1.0 - facing, 3.0);
    fresnel = min(fresnel, min(uFresnelMax, 0.82));

    // Reflected HDR highlights are compressed locally. Final tone mapping
    // still happens later, but it no longer receives an almost solid white
    // ocean from this material.
    reflectionColor = reflectionColor / (vec3(1.0) + reflectionColor * 0.35);

    vec3 color = mix(refractionColor, reflectionColor, fresnel);
    color = mix(color, uDeepColor, 0.08);

    // Direct sun.  A cubemap is static, so moving the scene sun cannot be
    // visible through the reflected environment alone; this term is the part
    // that must track uLightDir.  Water has a tight core plus a wider sheen
    // produced by its small ripples.
    vec3 L = normalize(-uLightDir);
    vec3 H = normalize(L + V);
    float roughness = clamp(uRoughness, 0.08, 0.6);
    float coreExponent = min(256.0, 2.0 / (roughness * roughness) - 2.0);
    float NoH = max(dot(detailNormal, H), 0.0);
    float specularCore = pow(NoH, coreExponent);
    float specularSheen = pow(NoH, mix(38.0, 8.0, roughness));
    float horizonBoost = 0.35 + 0.65 * (1.0 - facing);
    float specular = (specularCore * 0.75 + specularSheen * 0.35) * horizonBoost;
    color += uLightColor * specular * uSpecularStrength * 0.65;

    // Very small diffuse fill: not a Lambert-painted plastic surface, just
    // sunlight scattered back out of the top water layer.  It prevents the
    // body from becoming black when the reflected cubemap is dark.
    float NoL = max(dot(macroNormal, L), 0.0);
    color += uLightColor * uShallowColor * NoL * 0.055;

    float foam = 0.0;
    if (uHasFoam == 1 && uUnderwaterCamera == 0)
    {
        float shore = 1.0 - smoothstep(0.0, max(uFoamDepth, 0.01), bottomDistance);
        float pattern = texture(uFoamTex, vWorldPos.xz * uFoamScale
                                + flow * uTime * 0.012).r;
        float amount = clamp((shore + vCrest * uFoamCrest) * uFoamStrength, 0.0, 1.0);
        foam = smoothstep(0.58, 0.82, pattern) * amount;
        color = mix(color, vec3(0.72, 0.82, 0.84), foam * 0.65);
    }

    if (uUnderwaterCamera == 1)
        color = mix(color, uUnderwaterColor, 0.65);

#ifdef OCEAN_HAS_DEPTH
    if (uUnderwaterCamera == 0)
    {
        // Reflection-only lets the fixed-function blend darken the water by
        // this coverage.  The refraction tier must be opaque because it has
        // already sampled/composited the scene, but previously forgot the
        // equivalent mix and therefore appeared much brighter.  Applying it
        // explicitly makes both quality modes agree while retaining the
        // distorted bottom in refractionColor.
        float coverage = max(uMinOpacity, fresnel);
        color = mix(sceneBackground, color, coverage);

        // Artist control for the full refraction path.  Exposure and sky HDR
        // vary substantially between scenes, so a single physically inspired
        // constant cannot make both a noon sky and a storm sky readable.
        // uFresnelDetail is retained in the shared ABI but, in this simpler
        // Irrlicht-style shader, is exposed as "Brilho da refracao".
        color *= mix(0.20, 1.0, uFresnelDetail);
    }
#endif

    if (uDebugMode != 0)
    {
        if (uDebugMode == 1) FragColor = vec4(vec3(foam), 1.0);
        else if (uDebugMode == 2) FragColor = vec4(vec3(specular), 1.0);
        else if (uDebugMode == 3) FragColor = vec4(vec3(fresnel), 1.0);
        else if (uDebugMode == 4) FragColor = vec4(reflectionColor, 1.0);
        else if (uDebugMode == 5) FragColor = vec4(refractionColor, 1.0);
        else FragColor = vec4(detailNormal * 0.5 + 0.5, 1.0);
        return;
    }

#ifdef OCEAN_HAS_DEPTH
    float alpha = 1.0;
#else
    float alpha = max(uMinOpacity, fresnel);
#endif
    FragColor = vec4(max(color, vec3(0.0)), alpha);
}
