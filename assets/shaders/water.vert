#version 450 core
#extension GL_ARB_shader_draw_parameters : require

// Attributes match Mesh::colorLayout (see AssetManager::upload): position on
// stream 0, the rest on stream 1.
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aTangent;
layout(location = 3) in vec2 aUV;
layout(location = 4) in vec4 aColor;

// Same Camera block every forward shader binds at 0 (see CameraBlock.h).
// uClipPlane keeps the water's own draw from clipping itself (it is 0 here,
// only the reflection/opaque passes that render *around* the water set it).
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

// The reflection camera's own view-projection - a point's reflection UV comes
// from projecting through THIS matrix, never the main camera's. Written by
// WaterPass from the same matrix the reflection was rendered with.
layout(std140, binding = 2) uniform ReflectionCamera { mat4 uReflectionVP; };

// x = time, y = near, z = far. See WaterBlock.
layout(std140, binding = 6) uniform Water { vec4 uTimeNearFar; };

layout(std430, binding = 0) readonly buffer Instances { mat4 uModels[]; };

#define uTimeScale custom1.x
#define uScroll    custom1.y
#define uTexScale  custom1.z

// GL_CLIP_DISTANCE0 stays enabled for the reflection pass, where ordinary
// geometry uses it. Leaving this built-in unwritten is undefined and cuts
// arbitrary pieces out of the surface, so it has to be written here too -
// uClipPlane is (0,0,0,0) outside a clipped pass, which keeps every point.
out float gl_ClipDistance[1];

out vec3 vWorldPos;
out vec3 vNormal;
out vec2 vNoiseCoord;
out vec4 vClipPos;
out vec3 vEyeDir;
out vec4 vReflectionClip;

void main()
{
    mat4 model = uModels[gl_BaseInstanceARB + gl_InstanceID];
    vec4 worldPos = model * vec4(aPosition, 1.0);

    // Scrolled here rather than in the fragment shader: one multiply-add per
    // vertex instead of per pixel, and the surface is a handful of quads.
    vNoiseCoord = (aUV + uTimeNearFar.x * uTimeScale * uScroll) * uTexScale;

    vWorldPos = worldPos.xyz;
    vNormal = mat3(model) * aNormal;
    vEyeDir = normalize(worldPos.xyz - uCameraPos.xyz);
    vReflectionClip = uReflectionVP * worldPos;

    // Kept as an output so the fragment shader can do the perspective divide
    // itself and distort the result afterwards.
    vClipPos = uViewProj * worldPos;
    gl_Position = vClipPos;
    gl_ClipDistance[0] = dot(worldPos, uClipPlane);
}
