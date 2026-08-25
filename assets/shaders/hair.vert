#version 450 core

struct HairRoot
{
    vec4 positionLength;
    vec4 normalWidth;
    uvec4 joints;
    vec4 weights;
    vec4 params;
};
struct HairState { vec4 current; vec4 previous; };
struct HairPose { vec4 positionLength; vec4 normalWidth; vec4 previousPosition; };
layout(std430, binding = 4) readonly buffer RootBuffer { HairRoot roots[]; };
layout(std430, binding = 5) readonly buffer VisibleBuffer { uint visibleRoots[]; };
layout(std430, binding = 7) readonly buffer StateBuffer { HairState states[]; };
layout(std430, binding = 8) readonly buffer PoseBuffer { HairPose poses[]; };

#include "hair_uniforms.glsl"

out vec2 vUV;
out vec3 vWorldPos;
out vec3 vNormal;
out vec3 vTangent;
out vec3 vTint;
out float vRootFade;
out vec2 vMotionNDC;

void main()
{
    uint rootID = visibleRoots[gl_InstanceID];
    HairRoot root = roots[rootID];
    HairPose pose = poses[rootID];

    int verticesPerFollower = uSegmentCount * 6;
    int follower = gl_VertexID / verticesPerFollower;
    int localVertex = gl_VertexID - follower * verticesPerFollower;
    int segment = localVertex / 6;
    int triVertex = localVertex % 6;
    const int corners[6] = int[6](0, 1, 2, 2, 1, 3);
    int corner = corners[triVertex];
    float sideSign = (corner & 1) == 0 ? -1.0 : 1.0;
    float along = float(corner >> 1);

    uint stateID = rootID * uint(uSegmentCount) + uint(segment);
    vec3 p0 = segment == 0 ? pose.positionLength.xyz : states[stateID - 1u].current.xyz;
    vec3 p1 = states[stateID].current.xyz;
    vec3 old0 = segment == 0 ? pose.previousPosition.xyz : states[stateID - 1u].previous.xyz;
    vec3 old1 = states[stateID].previous.xyz;
    vec3 tangent = normalize(p1 - p0);
    vec3 centre = mix(p0, p1, along);
    vec3 oldCentre = mix(old0, old1, along);

    vec3 viewDirection = normalize(uCameraPos - centre);
    vec3 side = cross(viewDirection, tangent);
    if (dot(side, side) < 1e-6) side = cross(uCameraUp.xyz, tangent);
    side = normalize(side);
    float angle = root.params.x + float(follower) * 3.14159265 / float(max(uFollowerCount, 1));
    vec3 around = normalize(cross(tangent, side));
    side = normalize(side * cos(angle) + around * sin(angle));

    float t = (float(segment) + along) / float(uSegmentCount);
    float taper = mix(1.0, 0.18, t);
    float followerScale = follower == 0 ? 1.0 : 0.72;
    float width = pose.normalWidth.w * taper * followerScale;
    vec3 position = centre + side * sideSign * width;
    vec3 previousPosition = oldCentre + side * sideSign * width;

    vUV = vec2(sideSign * 0.5 + 0.5, t);
    vWorldPos = position;
    vTangent = tangent;
    vNormal = normalize(cross(side, tangent));
    vTint = mix(vec3(0.72, 0.55, 0.42), vec3(1.10, 0.92, 0.72), root.params.y);
    vRootFade = smoothstep(0.0, 0.16, t);

    gl_Position = uViewProj * vec4(position, 1.0);
    vec4 actual = uViewProjNoJitter * vec4(position, 1.0);
    vec4 previous = uPrevViewProjNoJitter * vec4(previousPosition, 1.0);
    vMotionNDC = previous.xy / previous.w - actual.xy / actual.w;
}
