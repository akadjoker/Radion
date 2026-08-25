#version 450 core

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

// The frame's sun and ambient (EnvironmentBlock.h). uSun points the way the
// light travels, into the scene. Only the leading three vec4s are declared:
// the block is longer, and a shader may read a prefix of it.
layout(std140, binding = 4) uniform Environment
{
    vec4 uSun;
    vec4 uSunColor;
    vec4 uAmbient;
};

// x = time, y = near, z = far. See WaterBlock.
layout(std140, binding = 6) uniform Water { vec4 uTimeNearFar; };

layout(binding = 0) uniform sampler2D uNoiseMap;
layout(binding = 1) uniform sampler2D uReflection;
layout(binding = 2) uniform sampler2D uRefraction;
layout(binding = 3) uniform sampler2D uRefractionDepth;

// baseColor is the water at the surface, emissive the colour absorption pulls
// toward with depth - not emission, it is simply the only spare vec4 left.
#define uShallowColor  baseColor.rgb
#define uDeepColor     emissive.rgb
#define uAbsorption    surface.x
#define uGlintStrength surface.y
#define uGlintPower    surface.z

#define uFresnelBias  custom0.x
#define uFresnelScale custom0.y
#define uFresnelPower custom0.z
#define uNoiseScale   custom0.w
#define uShoreFade    custom1.w

// Per-technique switches, so the demo panel can isolate which one is wrong.
#define uDistortionOn sequence.x
#define uAbsorptionOn sequence.y
#define uDepthRejectOn sequence.z
#define uGlintOn      sequence.w
#define uShoreFadeOn  uvAnim.x

#define uNear uTimeNearFar.y
#define uFar  uTimeNearFar.z

in vec3 vWorldPos;
in vec3 vNormal;
in vec2 vNoiseCoord;
in vec4 vClipPos;
in vec3 vEyeDir;
in vec4 vReflectionClip;

out vec4 FragColor;

float linearDepth(float d)
{
    float z = d * 2.0 - 1.0;
    return 2.0 * uNear * uFar / (uFar + uNear - z * (uFar - uNear));
}

void main()
{
    // The projection is done by hand so the distortion can be applied to the
    // result; a projected texture lookup would bake it in too early.
    vec2 ndc = (vClipPos.xy / vClipPos.w) * 0.5 + 0.5;

    // Per-pixel water depth: from this surface point down to whatever the
    // refraction copy saw at the same pixel. Sampled at the UNdistorted UV so
    // the depth read stays stable where the distortion is large.
    float bottomDepth = texture(uRefractionDepth, ndc).r;
    // Nothing behind this pixel but the far plane: the surface reaches past
    // whatever basin holds it and there is no water column here at all.
    // Without this the depth difference comes out enormous, the edge fade
    // saturates, and the plane draws as a solid sheet over open background.
    if (bottomDepth >= 0.999999)
        discard;
    float floorDist = linearDepth(bottomDepth);
    float waterDist = linearDepth(gl_FragCoord.z);
    // abs(): seen from underneath, the refraction view is the world ABOVE the
    // surface and this difference comes out negative. The fade only cares how
    // far the surface is from whatever is behind it.
    float waterDepth = abs(floorDist - waterDist);
    // 0 right at the waterline, 1 in open water.
    float soft = clamp(waterDepth / max(uShoreFade, 0.001), 0.0, 1.0);

    // Two samples of the same noise at different scales and drift: one alone
    // shows its own tile at this texture size.
    vec2 n1 = texture(uNoiseMap, vNoiseCoord * 0.2).rg - 0.5;
    vec2 n2 = texture(uNoiseMap, vNoiseCoord * 0.094 + 0.37).rg - 0.5;
    vec2 noise = (n1 * 0.70 + n2 * 0.30) * uNoiseScale;

    // Calmed as the water gets shallow, so the distorted lookup cannot drag
    // submerged pixels up over the dry geometry at the edge.
    vec2 distortion = noise * soft * step(0.5, uDistortionOn);

    vec2 refrUV = clamp(ndc + distortion, 0.001, 0.999);
    // A distorted sample that lands nearer than the undistorted one belongs to
    // geometry in FRONT of the water, not under it. Pulling that into the
    // refraction smears whatever stands at the surface across the pool.
    if (uDepthRejectOn > 0.5 && texture(uRefractionDepth, refrUV).r < bottomDepth)
        refrUV = ndc;

    vec2 reflUV = clamp((vReflectionClip.xy / vReflectionClip.w) * 0.5 + 0.5 + distortion,
                        0.001, 0.999);

    // Re-oriented toward the viewer so the term stays correct from underneath,
    // where the geometric normal points away from the camera.
    vec3 N = normalize(vNormal);
    if (dot(vEyeDir, N) > 0.0)
        N = -N;

    // vEyeDir points eye -> surface, so dot() is negative on a surface facing
    // the camera: 1 + dot is near zero looking straight down (refraction wins)
    // and near one at grazing angles (reflection wins).
    float fresnel = uFresnelBias + uFresnelScale * pow(1.0 + dot(vEyeDir, N), uFresnelPower);
    fresnel = clamp(fresnel, 0.0, 1.0);

    vec3 reflectionColor = texture(uReflection, reflUV).rgb;
    vec3 sceneColor = texture(uRefraction, refrUV).rgb;

    vec3 refractionColor = sceneColor;
    if (uAbsorptionOn > 0.5)
    {
        float depthFactor = clamp(waterDepth / max(uAbsorption, 0.001), 0.0, 1.0);
        // The bottom is clear at the shore and contributes nothing once the
        // column is deep; a fixed leak keeps the scene visible at every depth
        // and reads as a veil over the water instead of water over the scene.
        float bottomVisibility = pow(1.0 - depthFactor, 1.6);
        refractionColor = mix(mix(uShallowColor, uDeepColor, depthFactor), sceneColor,
                              bottomVisibility * 0.82);
    }

    vec3 color = mix(refractionColor, reflectionColor, fresnel);

    // A direct specular term is what actually puts the sun on the water. The
    // mirrored reflection can only show it when it sits low enough for the
    // Fresnel term to favour reflection, and in an enclosed courtyard that
    // near-horizontal reflected ray hits a wall rather than the sky. This
    // term cares about neither: it lights up whenever the surface normal
    // bisects eye and sun.
    if (uGlintOn > 0.5)
    {
        vec3 glintNormal = normalize(vec3(distortion.x * 8.0, 1.0, distortion.y * 8.0));
        vec3 halfVec = normalize(-vEyeDir - uSun.xyz);
        float glint = pow(max(dot(glintNormal, halfVec), 0.0), max(uGlintPower, 1.0)) * soft;
        color += uSunColor.rgb * glint * uGlintStrength;
    }

    // Optional: fades the surface out as the column under it goes to nothing.
    // Right for water that runs out onto a shore, wrong for a pool whose built
    // sides already end it - there the pale ramp against the stone reads as
    // scum. Off leaves the surface opaque to its own edge. Separate from the
    // uShoreFade damping above, which solves a different problem.
    float edge = 1.0;
    if (uShoreFadeOn > 0.5)
        edge = clamp(waterDepth / max(uShoreFade * 0.6, 0.001), 0.0, 1.0);
    FragColor = vec4(max(color, vec3(0.0)), edge);
}
