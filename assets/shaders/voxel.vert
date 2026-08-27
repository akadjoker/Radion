#version 450 core
#extension GL_ARB_shader_draw_parameters : require

// A voxel chunk's vertex: the position stream every mesh has, plus one packed
// word in place of the normal/tangent/uv/colour/uv2 record. The outputs are
// exactly lit.vert's, so lit.frag shades a chunk with the same sun, cascades,
// local lights and fog as anything else in the scene.
//
// Bit layout, shared with VoxelMesher::packVertex - neither side moves a
// field alone:
//   0-2   face index, into the six normals below
//   3-4   ambient occlusion, 0 darkest
//   5-9   atlas column
//   10-14 atlas row
//   15-20 u, in tiles across the merged quad
//   21-26 v
layout(location = 0) in vec3 aPosition;
layout(location = 1) in uint aVoxel;

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
out vec2 vUV2;
out vec4 vColor;
out vec4 vTangent;

out float vViewDepth;
#ifndef NO_TEMPORAL
out vec2 vMotionNDC;
#endif

const vec3 kFaceNormals[6] = vec3[6](vec3(-1.0, 0.0, 0.0), vec3(1.0, 0.0, 0.0),
                                     vec3(0.0, -1.0, 0.0), vec3(0.0, 1.0, 0.0),
                                     vec3(0.0, 0.0, -1.0), vec3(0.0, 0.0, 1.0));

// The mesher's own curve: the darkest corner still has to read as shaded
// ground rather than a hole.
const float kOcclusion[4] = float[4](0.45, 0.65, 0.82, 1.0);

void main()
{
    InstanceData instance = uInstances[gl_BaseInstanceARB + gl_InstanceID];
    mat4 model = instance.model;

    vec4 localPosition = vec4(aPosition, 1.0);
    vec4 worldPos = model * localPosition;
#ifndef NO_TEMPORAL
    vec4 prevWorldPos = instance.prevModel * localPosition;
#endif

    uint face = aVoxel & 7u;
    uint occlusion = (aVoxel >> 3) & 3u;
    vec2 tile = vec2(float((aVoxel >> 5) & 31u), float((aVoxel >> 10) & 31u));
    vec2 extent = vec2(float((aVoxel >> 15) & 63u), float((aVoxel >> 21) & 63u));

    mat3 normalMatrix = mat3(model);

    vWorldPos = worldPos.xyz;
    vNormal = normalMatrix * kFaceNormals[face];
    // No normal maps on blocks, so this only has to be a finite basis vector
    // lit.frag can carry through untouched.
    vTangent = vec4(normalMatrix * vec3(1.0, 0.0, 0.0), 1.0);
    vUV = extent;
    // custom0.xy is one tile's share of the atlas, which is what turns a
    // column and row into the origin lit.frag's VOXEL_ATLAS branch samples in.
    vUV2 = tile * custom0.xy;
    vColor = vec4(vec3(kOcclusion[occlusion]), 1.0);
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
