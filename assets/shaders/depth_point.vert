#version 450 core
#extension GL_ARB_shader_draw_parameters : require

layout(location = 0) in vec3 aPosition;
#ifdef MATERIAL_ALPHA_TEST
layout(location = 3) in vec2 aUV;
out vec2 vUV;
#endif
#ifdef MATERIAL_SKINNED
layout(location = 5) in uvec4 aJoints;
layout(location = 6) in vec4 aWeights;
#endif

layout(std140, binding = 0) uniform Camera { mat4 uViewProj; vec4 uClipPlane; };
struct InstanceData { mat4 model; uint paletteOffset; uint pad0; uint pad1; uint pad2; };
layout(std430, binding = 0) readonly buffer Instances { InstanceData uInstances[]; };
#ifdef MATERIAL_SKINNED
layout(std430, binding = 1) readonly buffer Palettes { mat4 uPalettes[]; };
#endif

out vec3 vWorldPos;

void main()
{
    InstanceData instance = uInstances[gl_BaseInstanceARB + gl_InstanceID];
#ifdef MATERIAL_SKINNED
    uint base = instance.paletteOffset;
    mat4 skin = uPalettes[base + aJoints.x] * aWeights.x +
                uPalettes[base + aJoints.y] * aWeights.y +
                uPalettes[base + aJoints.z] * aWeights.z +
                uPalettes[base + aJoints.w] * aWeights.w;
    vec4 localPosition = skin * vec4(aPosition, 1.0);
#else
    vec4 localPosition = vec4(aPosition, 1.0);
#endif
    vec4 worldPos = instance.model * localPosition;
    vWorldPos = worldPos.xyz;
#ifdef MATERIAL_ALPHA_TEST
    vUV = aUV;
#endif
    gl_Position = uViewProj * worldPos;
}
