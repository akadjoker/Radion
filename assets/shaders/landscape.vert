#version 450 core
#extension GL_ARB_shader_draw_parameters : require

// Landscape chunks. Draws through the exact same instancing path lit.vert
// uses - one model matrix per instance out of the Instances SSBO - because a
// chunk is submitted through the ordinary RenderList/ForwardPass path like
// any other mesh: shadows, local lights, decals and fog all come for free.
// Paired with lit.frag compiled with LANDSCAPE_REGIONS, so the fragment side
// is shared too; only the vertex layout differs, because a chunk carries
// region weights instead of a tangent and a vertex colour.

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV;
layout(location = 3) in vec4 aWeights; // base, slope, low altitude, high altitude

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

// Same layout as ForwardPass::GPUInstance and lit.vert - a chunk never skins,
// but the stride is shared with every other material that pass draws.
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

out float gl_ClipDistance[1];

out vec3 vWorldPos;
out vec3 vNormal;
out vec2 vUV;
out float vViewDepth;
out vec4 vTangent;
out vec4 vWeights;
#ifndef NO_TEMPORAL
out vec2 vMotionNDC;
#endif

void main()
{
    InstanceData instance = uInstances[gl_BaseInstanceARB + gl_InstanceID];
    vec4 worldPos = instance.model * vec4(aPosition, 1.0);

    vWorldPos = worldPos.xyz;
    vNormal = mat3(instance.model) * aNormal;
    vUV = aUV;
    vWeights = aWeights;

    // lit.frag declares this unconditionally for the normal-map path; a
    // chunk has no tangent (it has no normal map), so this is a fixed value
    // rather than an unmatched varying.
    vTangent = vec4(1.0, 0.0, 0.0, 1.0);

    vViewDepth = -(uView * worldPos).z;

    gl_Position = uViewProj * worldPos;
#ifndef NO_TEMPORAL
    vec4 prevWorldPos = instance.prevModel * vec4(aPosition, 1.0);
    vec4 currentNoJitter = uViewProjectionNoJitter * worldPos;
    vec4 previousNoJitter = uPrevViewProjectionNoJitter * prevWorldPos;
    vMotionNDC = previousNoJitter.xy / previousNoJitter.w -
                 currentNoJitter.xy / currentNoJitter.w;
#endif
    gl_ClipDistance[0] = dot(worldPos, uClipPlane);
}
