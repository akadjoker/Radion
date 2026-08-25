// Shared between ocean.vert and ocean.frag - one block, so the two never
// disagree about a binding. std140 pairs each vec3 with the scalar that
// follows it, the same convention grass_uniforms.glsl uses.
layout(std140, binding = 0) uniform OceanUniforms
{
    mat4 uModel;
    mat4 uViewProj;
    mat4 uReflectionVP;
    mat4 uInvViewProj;

    vec3 uViewPos;      float uTime;
    vec3 uLightDir;     float uTimeScale;
    vec3 uLightColor;   float uSteepness;
    vec3 uAmbient;      float uSkyIntensity;

    vec4 uWaves[6];      // xy = direction, z = wavelength, w = amplitude
    int  uWaveCount;      float uPad0, uPad1, uPad2;

    vec3 uShallowColor; float uAbsorptionDistance;
    vec3 uDeepColor;    float uRoughness;

    float uFresnelDetail;
    float uFresnelMax;
    float uMinOpacity;
    float uSpecularStrength;

    float uNormalScale1;
    float uNormalScale2;
    float uNormalStrength;
    float uNormalSpeed1;

    float uNormalSpeed2;
    int   uNormalOctaves;
    int   uHasNormalMap;
    int   uHasFoam;

    float uFoamScale;
    float uFoamStrength;
    float uFoamDepth;
    float uFoamCrest;

    float uReflectionDistortion;
    int   uDebugMode;
    vec2  uScreenSize;

    vec3 uUnderwaterColor; int uUnderwaterCamera;

    int  uHasSkyCube;   float uFresnelBias, uFresnelScale, uFresnelPower;

    vec4 uOpticalStrengths; // x reflection, y refraction, z water colour, w unused
};
