 

layout(std140, binding = 0) uniform GrassUniforms
{
    mat4 uViewProj;
    // Jitter-free pair, for the motion vector only. uViewProj still carries
    // the jitter, because that is what rasterisation has to use.
    mat4 uViewProjNoJitter;
    mat4 uPrevViewProjNoJitter;

    vec3 uCameraPos;
    float uTime;

    vec3 uCameraUp;
    float uAltura;

    vec3 uLightDir;
    float uLargura;

    vec3 uLightColor;
    float uVento;

    vec3 uAmbient;
    float uDistMax;

    vec3 uViewPos;
    float uAlphaCut;

    float uBendCamera;
    float uStiffness;
    float uDrag;
    float uDt;

    int uRectCount;
    int uClumpCount;
    int uMaxVisiveis;
    int uModo;

    int uReset;
    int uInfluCount;
    int uGrassPad0;
    int uGrassPad1;

    vec4 uInflu[8];        // xyz = centro, w = raio
    float uInfluForca[8];  // std140 gives each its own 16 bytes
};
