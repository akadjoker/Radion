#version 450 core
precision highp float;

in vec3 vWorldPos;
#ifdef MATERIAL_ALPHA_TEST
in vec2 vUV;
layout(binding = 0) uniform sampler2D uAlbedoTex;
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
#endif

layout(std140, binding = 2) uniform PointDepth
{
    vec4 uLightPositionAndRange; // xyz = position, w = range
    vec4 uBias;                  // x = bias
};

void main()
{
#ifdef MATERIAL_ALPHA_TEST
    vec2 uv = vUV * uvTransform.xy + uvTransform.zw;
    if (texture(uAlbedoTex, uv).a < surface.z)
        discard;
#endif
    float d = length(vWorldPos - uLightPositionAndRange.xyz) /
              max(0.0001, uLightPositionAndRange.w);
    gl_FragDepth = clamp(d + uBias.x, 0.0, 1.0);
}
