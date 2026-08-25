#version 450 core
#extension GL_ARB_shader_draw_parameters : require

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aTangent;
layout(location = 3) in vec2 aUV;
layout(location = 4) in vec4 aColor;
#ifdef MATERIAL_SKINNED
layout(location = 5) in uvec4 aJoints;
layout(location = 6) in vec4 aWeights;
#endif
// Location 7, not 5/6: those are claimed by skinning above, and this
// attribute has to sit at the same location whether or not the mesh is
// skinned - see AssetMesh.cpp's colorLayout comment.
layout(location = 7) in vec2 aUV2;

layout(std140, binding = 0) uniform Camera
{
    mat4 uViewProj;
    vec4 uClipPlane;
    vec4 uCameraPos;
    mat4 uView;
};

#ifndef NO_TEMPORAL
layout(std140, binding = 7) uniform TemporalCamera
{
    mat4 uViewProjectionNoJitter;
    mat4 uPrevViewProjectionNoJitter;
};
#endif

struct InstanceData
{
    mat4 model;
    mat4 prevModel;
    uint paletteOffset;
    uint prevPaletteOffset;
    uint pad0;
    uint pad1;
};
layout(std430, binding = 0) readonly buffer Instances { InstanceData uInstances[]; };
#ifdef MATERIAL_SKINNED
layout(std430, binding = 1) readonly buffer Palettes { mat4 uPalettes[]; };
#endif

out float gl_ClipDistance[1];

out vec3 vWorldPos;
out vec3 vNormal;
out vec2 vUV;
out vec2 vUV2;
out vec4 vColor;
out vec4 vTangent;

out float vViewDepth;
#ifndef NO_TEMPORAL
out vec2 vMotionNDC;
#endif

void main()
{
    InstanceData instance = uInstances[gl_BaseInstanceARB + gl_InstanceID];
    mat4 model = instance.model;
#ifdef MATERIAL_SKINNED
    uint base = instance.paletteOffset;
    mat4 skin = uPalettes[base + aJoints.x] * aWeights.x +
                uPalettes[base + aJoints.y] * aWeights.y +
                uPalettes[base + aJoints.z] * aWeights.z +
                uPalettes[base + aJoints.w] * aWeights.w;
    vec4 localPosition = skin * vec4(aPosition, 1.0);
    uint prevBase = instance.prevPaletteOffset;
    mat4 prevSkin = uPalettes[prevBase + aJoints.x] * aWeights.x +
                    uPalettes[prevBase + aJoints.y] * aWeights.y +
                    uPalettes[prevBase + aJoints.z] * aWeights.z +
                    uPalettes[prevBase + aJoints.w] * aWeights.w;
    vec4 prevLocalPosition = prevSkin * vec4(aPosition, 1.0);
    vec3 localNormal = mat3(skin) * aNormal;
    vec3 localTangent = mat3(skin) * aTangent.xyz;
#else
    vec4 localPosition = vec4(aPosition, 1.0);
    vec4 prevLocalPosition = localPosition;
    vec3 localNormal = aNormal;
    vec3 localTangent = aTangent.xyz;
#endif
    vec4 worldPos = model * localPosition;
#ifndef NO_TEMPORAL
    vec4 prevWorldPos = instance.prevModel * prevLocalPosition;
#endif

    mat3 normalMatrix = mat3(model);

    vWorldPos = worldPos.xyz;
    vNormal = normalMatrix * localNormal;
    vTangent = vec4(normalMatrix * localTangent, aTangent.w);
    vUV = aUV;
    vUV2 = aUV2;
    vColor = aColor;
    vViewDepth = -(uView * worldPos).z;

    gl_Position = uViewProj * worldPos;
#ifndef NO_TEMPORAL
    vec4 currentNoJitter = uViewProjectionNoJitter * worldPos;
    vec4 previousNoJitter = uPrevViewProjectionNoJitter * prevWorldPos;
    vMotionNDC = previousNoJitter.xy / previousNoJitter.w -
                 currentNoJitter.xy / currentNoJitter.w;
#endif
    gl_ClipDistance[0] = dot(worldPos, uClipPlane);
}
