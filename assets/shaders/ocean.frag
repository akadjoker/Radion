#version 450 core

// Water surface, same technique as the fresnel demo's water.frag: a tunable
// Fresnel curve (bias/scale/power) instead of a fixed physical one, direct
// sun glint off the bump normal, Beer-Lambert-style depth absorption. The
// difference is only where the inputs come from - vWorldPos/vNormal/vCrest
// off ocean.vert's Gerstner waves instead of a flat plane's varyings.

in vec3 vWorldPos;
in vec3 vNormal;
in vec2 vUV;
in float vCrest;

out vec4 FragColor;

#include "ocean_uniforms.glsl"

layout(binding = 0) uniform sampler2D uNormalMap1; // large-scale ripple detail
layout(binding = 1) uniform sampler2D uFoamTex;
#ifdef OCEAN_HAS_REFLECTION
layout(binding = 2) uniform sampler2D uReflection; // planar reflection (alpha 0 = sky only)
#endif
layout(binding = 4) uniform samplerCube uSkyCube;
#ifdef OCEAN_HAS_DEPTH
layout(binding = 3) uniform sampler2D uSceneDepth; // depth of what the water sits over
layout(binding = 5) uniform sampler2D uSceneColor; // opaque scene copied before the ocean
#endif

// A copy of the engine's own sky gradient - no #include across the two, so
// they have to agree by hand, or the water reflects a sky that does not exist.
vec3 SkyGradient(vec3 dir)
{
    vec3 sunDir = normalize(-uLightDir);
    float day = smoothstep(-0.08, 0.25, sunDir.y);
    vec3 horizon = mix(uAmbient * 0.5, vec3(0.72, 0.82, 0.92), day);
    vec3 zenith = mix(uAmbient * 0.25, vec3(0.22, 0.45, 0.85), day);
    float h = clamp(dir.y, 0.0, 1.0);
    vec3 col = mix(horizon, zenith, pow(h, 0.6));
    if (dir.y < 0.0)
        col = mix(horizon, horizon * 0.55, clamp(-dir.y * 3.0, 0.0, 1.0));
    return col * uSkyIntensity;
}

#ifdef OCEAN_HAS_DEPTH
vec3 WorldPosFromDepth(vec2 uv, float depth)
{
    vec4 clip = vec4(uv * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
    vec4 w = uInvViewProj * clip;
    return w.xyz / w.w;
}
#endif

void main()
{
    vec3 V = normalize(uViewPos - vWorldPos);
    vec3 N = normalize(vNormal);
    if (dot(N, V) < 0.0)
        N = -N;

    // Octaves of the same texture, gradients summed rather than normals
    // blended - see the old ocean.frag for why (slopes add linearly, normals
    // do not). Kept only as a 2D bump: nothing here needs the separate
    // macro/detail normal split the previous version used to protect Fresnel.
    vec2 bump = vec2(0.0);
    if (uHasNormalMap == 1)
    {
        vec2 base = vWorldPos.xz - vec2(0.87, 0.49) * uNormalSpeed1 * uTime;
        vec2 uvBase = base * uNormalScale1;

        vec2 grad = vec2(0.0);
        float amp = 1.0;
        float total = 0.0;
        for (int i = 0; i < uNormalOctaves; ++i)
        {
            float scale = pow(2.0, float(i));
            vec2 uv = uvBase * scale + vec2(0.013, -0.021) * uTime * float(i);
            grad += (texture(uNormalMap1, uv).xy * 2.0 - 1.0) * amp;
            total += amp;
            amp *= 0.5;
        }
        grad /= max(0.001, total);
        bump = grad * uNormalStrength;

        vec3 T = normalize(cross(vec3(0.0, 0.0, 1.0), N));
        vec3 B = cross(N, T);
        vec3 nd = normalize(vec3(bump, 1.0));
        N = normalize(mat3(T, B, N) * nd);
        if (dot(N, V) < 0.0)
            N = -N;
    }

    vec2 screenUV = gl_FragCoord.xy / uScreenSize;
    vec2 distortion = bump * uReflectionDistortion;

    // ---- Refraction / depth absorption ----
    vec3 refractionColor = uDeepColor * uOpticalStrengths.z;
    float depthFactor = 1.0;
    float bottomDistance = 1e6;
#ifdef OCEAN_HAS_DEPTH
    float bottomDepth = texture(uSceneDepth, screenUV).r;
    if (bottomDepth < 0.999999)
    {
        vec3 bottomPos = WorldPosFromDepth(screenUV, bottomDepth);
        bottomDistance = max(0.0, vWorldPos.y - bottomPos.y);
        depthFactor = clamp(bottomDistance / max(uAbsorptionDistance, 0.001), 0.0, 1.0);

        vec2 refractUV = clamp(screenUV + distortion, vec2(0.002), vec2(0.998));
        if (texture(uSceneDepth, refractUV).r < bottomDepth)
            refractUV = screenUV;

        vec3 sceneColor = texture(uSceneColor, refractUV).rgb;
        vec3 absorptionColor = mix(uShallowColor, uDeepColor, depthFactor);
        // Beer-Lambert-ish: the bottom is clear at the shore and contributes
        // nothing once the column is deep.
        float bottomVisibility = pow(1.0 - depthFactor, 1.6);
        refractionColor = mix(absorptionColor, sceneColor, bottomVisibility * 0.82) *
                          uOpticalStrengths.z;
    }
#endif

    // ---- Reflection ----
    vec3 R = reflect(-V, N);
    vec3 reflectionColor = uHasSkyCube == 1
        ? texture(uSkyCube, R).rgb * uSkyIntensity
        : SkyGradient(R);

#ifdef OCEAN_HAS_REFLECTION
    vec4 refPos = uReflectionVP * vec4(vWorldPos, 1.0);
    if (refPos.w > 0.0)
    {
        vec2 reflectionUV = clamp(refPos.xy / refPos.w * 0.5 + 0.5 + distortion,
                                  vec2(0.002), vec2(0.998));
        vec4 r = texture(uReflection, reflectionUV);
        reflectionColor = mix(reflectionColor, r.rgb, r.a * clamp(uOpticalStrengths.x, 0.0, 1.0));
    }
#endif

    // ---- Fresnel ----
    // Same shape the fresnel demo's water material uses - an artist-tunable
    // curve instead of a fixed F0/exponent, easier to push toward "looks
    // right" than a physically-derived one.
    float facing = clamp(dot(N, V), 0.0, 1.0);
    float rawFresnel = uFresnelBias + uFresnelScale * pow(1.0 - facing, uFresnelPower);
    float F = clamp(min(rawFresnel, uFresnelMax), 0.0, 1.0);

    vec3 color;
    if (uUnderwaterCamera == 1)
    {
        // cos(48.6 deg) = 0.661: water's critical angle. Inside it the world
        // above is visible through Snell's window; beyond it, total internal
        // reflection.
        float cosV = clamp(abs(dot(N, V)), 0.0, 1.0);
        float window = smoothstep(0.60, 0.74, cosV);
        vec3 outside = reflectionColor * 0.7;
        vec3 mirror = uUnderwaterColor * 1.15;
        color = mix(mirror, outside, window);
    }
    else
    {
        color = mix(refractionColor, reflectionColor, F);
    }

    // ---- Sun glint ----
    // Direct term, not the mirrored reflection: a cubemap is static and an
    // enclosed reflection can miss the sun entirely, but this lights up
    // whenever the surface normal bisects eye and sun regardless.
    vec3 L = normalize(-uLightDir);
    if (uUnderwaterCamera == 0)
    {
        vec3 glintNormal = normalize(vec3(distortion.x * 8.0, 1.0, distortion.y * 8.0));
        vec3 halfVec = normalize(V + L);
        float roughness = max(0.02, uRoughness);
        float power = 2.0 / (roughness * roughness);
        float glint = pow(max(dot(glintNormal, halfVec), 0.0), power) * depthFactor;
        color += uLightColor * glint * uSpecularStrength;
    }

    // ---- Shoreline foam ----
    // Mask from depth (where the water is shallow), shape from a texture -
    // two samples rolling in opposite directions so it looks agitated
    // instead of sliding as one block.
    float debugFoam = 0.0;
    float band = 0.0;
#ifdef OCEAN_HAS_DEPTH
    band = 1.0 - smoothstep(0.0, max(0.01, uFoamDepth), bottomDistance);
#endif
    if (uUnderwaterCamera == 1)
        band = 0.0;

    if (uHasFoam == 1 && uUnderwaterCamera == 0)
    {
        float f1 = texture(uFoamTex, vWorldPos.xz * uFoamScale + vec2( 0.02, 0.013) * uTime).r;
        float f2 = texture(uFoamTex, vWorldPos.xz * uFoamScale * 1.7 + vec2(-0.011, 0.021) * uTime).r;
        float f = max(f1, f2);
        f = smoothstep(0.10, 0.55, f);

        float amount = clamp((band + vCrest * uFoamCrest) * uFoamStrength, 0.0, 1.0);
        float threshold = mix(1.02, -0.02, amount);
        float foam = smoothstep(threshold, threshold + 0.25, f);
        debugFoam = foam;

        color = mix(color, vec3(0.90, 0.94, 0.96) * (uAmbient + uLightColor * 0.75), foam);
        F *= (1.0 - foam * 0.9);
    }
    else if (uUnderwaterCamera == 0)
    {
        color = mix(color, vec3(0.9), band * 0.35);
    }

#ifdef OCEAN_HAS_DEPTH
    float alpha = 1.0;
#else
    float alpha = mix(1.0, max(uMinOpacity, rawFresnel), clamp(uOpticalStrengths.y, 0.0, 1.0));
#endif
    if (uUnderwaterCamera == 1)
        alpha = 1.0;

    if (uDebugMode != 0)
    {
        if (uDebugMode == 1) FragColor = vec4(vec3(debugFoam), 1.0);
        if (uDebugMode == 2) FragColor = vec4(vec3(F), 1.0);
        if (uDebugMode == 3) FragColor = vec4(vec3(F), 1.0);
        if (uDebugMode == 4) FragColor = vec4(reflectionColor, 1.0);
        if (uDebugMode == 5) FragColor = vec4(refractionColor, 1.0);
        if (uDebugMode == 6) FragColor = vec4(N * 0.5 + 0.5, 1.0);
        return;
    }

    FragColor = vec4(max(color, vec3(0.0)), alpha);
}
