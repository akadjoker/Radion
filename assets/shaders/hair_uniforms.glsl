struct HairColliderGPU
{
    vec4 a;
    vec4 b;
    uint type;
    uint _pad0, _pad1, _pad2;
};

layout(std140, binding = 0) uniform HairBlock
{
    mat4 uViewProj;
    mat4 uViewProjNoJitter;
    mat4 uPrevViewProjNoJitter;
    mat4 uModel;
    mat4 uPrevModel;

    vec4 uColorRoughness;
    vec4 uCameraTime;
    vec4 uCameraUp;
    vec4 uLightDirWind;
    vec4 uLightColorGravity;
    vec4 uAmbientDt;
    vec4 uPhysics;
    vec4 uAppearance;
    ivec4 uCounts;
    ivec4 uState;
    HairColliderGPU uColliders[8];
};

#define uCameraPos      uCameraTime.xyz
#define uTime           uCameraTime.w
#define uLightDir       uLightDirWind.xyz
#define uWind           uLightDirWind.w
#define uLightColor     uLightColorGravity.xyz
#define uGravity        uLightColorGravity.w
#define uAmbient        uAmbientDt.xyz
#define uDt             uAmbientDt.w
#define uStiffness      uPhysics.x
#define uDrag           uPhysics.y
#define uDrawDistance   uPhysics.z
#define uAlphaCut       uPhysics.w
#define uSpecularStrength uAppearance.x
#define uSpecularTint   uAppearance.y
#define uTransmission   uAppearance.z
#define uRootCount      uCounts.x
#define uSegmentCount   uCounts.y
#define uFollowerCount  uCounts.z
#define uColliderCount  uCounts.w
#define uReset          uState.x
#define uPaletteCount   uState.y
#define uPrevPaletteCount uState.z
#define uMode           uState.w
