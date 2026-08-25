#version 450 core
#extension GL_ARB_shader_draw_parameters : require

// Attributes match Mesh::colorLayout (see AssetManager::upload): position on
// stream 0, the rest on stream 1.
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aTangent;
layout(location = 3) in vec2 aUV;
layout(location = 4) in vec4 aColor;
#ifdef MATERIAL_SKINNED
layout(location = 5) in uvec4 aJoints;
layout(location = 6) in vec4 aWeights;
#endif
layout(location = 7) in vec2 aUV2;

// uClipPlane (0,0,0,0) by default: dot(pos, 0) == 0 for every point, so
// nothing is ever cut unless the camera that filled this buffer set a real
// plane (see FrameContext::clipPlane) - a planar reflection's mirrored
// camera, e.g. There is no separate on/off switch: the plane itself is the
// state, so nothing can leak into the next pass by forgetting to disable it.
layout(std140, binding = 0) uniform Camera { mat4 uViewProj; vec4 uClipPlane; };

#ifndef NO_TEMPORAL
layout(std140, binding = 7) uniform TemporalCamera
{
    mat4 uViewProjectionNoJitter;
    mat4 uPrevViewProjectionNoJitter;
};
#endif

// Every model matrix for the frame, in draw order, uploaded once. A run of
// packets sharing a mesh draws as one instanced call: baseInstance is where
// the run starts, gl_InstanceID walks it. The layout is shared with
// ForwardPass::GPUInstance and with every other vertex shader that pass can
// bind: a field added here has to appear in all of them or the stride stops
// matching and each instance reads the previous one's tail.
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
    vec3 localNormal = mat3(skin) * aNormal;
    uint prevBase = instance.prevPaletteOffset;
    mat4 prevSkin = uPalettes[prevBase + aJoints.x] * aWeights.x +
                    uPalettes[prevBase + aJoints.y] * aWeights.y +
                    uPalettes[prevBase + aJoints.z] * aWeights.z +
                    uPalettes[prevBase + aJoints.w] * aWeights.w;
    vec4 prevLocalPosition = prevSkin * vec4(aPosition, 1.0);
#else
    vec4 localPosition = vec4(aPosition, 1.0);
    vec3 localNormal = aNormal;
    vec4 prevLocalPosition = localPosition;
#endif
    vec4 worldPos = model * localPosition;
#ifndef NO_TEMPORAL
    vec4 prevWorldPos = instance.prevModel * prevLocalPosition;
#endif

    vWorldPos = worldPos.xyz;
    vNormal = mat3(model) * localNormal;
    vUV = aUV;
    vUV2 = aUV2;
    vColor = aColor;
    gl_Position = uViewProj * worldPos;
#ifndef NO_TEMPORAL
    vec4 currentNoJitter = uViewProjectionNoJitter * worldPos;
    vec4 previousNoJitter = uPrevViewProjectionNoJitter * prevWorldPos;
    vMotionNDC = previousNoJitter.xy / previousNoJitter.w -
                 currentNoJitter.xy / currentNoJitter.w;
#endif
    gl_ClipDistance[0] = dot(worldPos, uClipPlane);
}
