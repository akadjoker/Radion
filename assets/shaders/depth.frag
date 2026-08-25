#version 450 core
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

void main()
{
#ifdef MATERIAL_ALPHA_TEST
    vec2 uv = vUV * uvTransform.xy + uvTransform.zw;
    if (texture(uAlbedoTex, uv).a < surface.z)
        discard;
#endif
}
