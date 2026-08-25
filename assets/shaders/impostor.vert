#version 450 core

// Impostor: a quad that stands in for the tree at distance.
//
// No vertex buffer - the four corners come from gl_VertexID, the instance from
// the SSBO.
//
// The billboard turns about Y ONLY, not toward the camera outright. That is
// deliberate: the photographs were taken from N angles around the Y axis, so
// the quad has to stay in the plane they were captured in. A spherical
// billboard would tilt the tree when seen from above, and the photograph -
// taken level - would give itself away immediately.

struct TreeInstance
{
    vec3 position;
    float scale;
    vec3 normal;
    float rotation;
};
layout(std430, binding = 6) readonly buffer TreeInstanceBuffer
{
    TreeInstance instances[];
};

layout(std140, binding = 0) uniform Camera
{
    mat4 uViewProj;
    vec4 uClipPlane;
    vec4 uCameraPos;
    mat4 uView;
    mat4 uViewProjNoJitter;
    mat4 uPrevViewProjNoJitter;
};

layout(std140, binding = 7) uniform ImpostorBlock
{
    // x = tree height in world units, y = quad width over height,
    // z = angle count, w = first array layer of this species.
    vec4 uImpostorShape;
    // x = swap distance, y = swap band half-width, z = alpha cut.
    vec4 uImpostorSwap;
};

#define uHeight     uImpostorShape.x
#define uWidth      uImpostorShape.y
#define uAngles     int(uImpostorShape.z)
#define uBaseLayer  int(uImpostorShape.w)
#define uSwapDist   uImpostorSwap.x
#define uSwapBand   uImpostorSwap.y

out vec2 vUV;
out float vLayer;
out float vRotation; // the instance's own spin, to turn the normal in the frag
out vec3 vWorldPos;
out float vHeight01;
out float vFade;
out vec2 vMotionNDC;

void main()
{
    TreeInstance it = instances[gl_InstanceID];

    float height = uHeight * it.scale;
    float width = height * uWidth;

    // Tree to camera, in the horizontal plane only.
    vec3 toCamera = uCameraPos.xyz - it.position;
    toCamera.y = 0.0;
    float cameraDistance = length(toCamera);
    toCamera = (cameraDistance > 1e-4) ? toCamera / cameraDistance : vec3(0.0, 0.0, 1.0);

    vec3 right = normalize(cross(vec3(0.0, 1.0, 0.0), toCamera));

    // ---- Choosing the photograph ----
    // The viewing angle MINUS the instance's own rotation. Without subtracting
    // it, every tree showed the same face and the variety the rotation gave the
    // mesh vanished the moment it became an impostor.
    float angle = atan(toCamera.x, toCamera.z) - it.rotation;
    const float TAU = 6.28318530718;
    angle = angle - TAU * floor(angle / TAU); // into [0, 2pi)
    int index = int(floor(angle / TAU * float(uAngles) + 0.5)) % uAngles;
    vLayer = float(uBaseLayer + index);
    vRotation = it.rotation;

    // The four corners: 0=(-1,0) 1=(1,0) 2=(-1,1) 3=(1,1)
    vec2 corner = vec2(float(gl_VertexID & 1) * 2.0 - 1.0, float(gl_VertexID >> 1));
    vUV = vec2(corner.x * 0.5 + 0.5, corner.y);
    vHeight01 = corner.y;

    // The impostor does the opposite of the mesh: it fades IN as the tree gets
    // further away.
    vFade = smoothstep(uSwapDist - uSwapBand, uSwapDist + uSwapBand, cameraDistance);

    vec3 position = it.position + right * (corner.x * width * 0.5) +
                    vec3(0.0, corner.y * height, 0.0);
    vWorldPos = position;

    gl_Position = uViewProj * vec4(position, 1.0);
    // The tree itself does not move, so this is camera motion only. The quad
    // also re-aims at the camera every frame, and that part is deliberately
    // not modelled here: the corner shift is a fraction of a pixel at the
    // distance an impostor takes over, and reconstructing the previous
    // billboard needs a previous camera position this block does not carry.
    vec4 current = uViewProjNoJitter * vec4(position, 1.0);
    vec4 previous = uPrevViewProjNoJitter * vec4(position, 1.0);
    vMotionNDC = previous.xy / previous.w - current.xy / current.w;
}
