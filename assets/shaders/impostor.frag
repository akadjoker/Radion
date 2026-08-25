#version 450 core

// The impostor stores TWO maps per angle: colour and NORMALS.
//
// The normals are the whole point of the work. Capturing albedo alone leaves
// distant trees frozen in the light of the instant they were photographed: the
// sun turns, the near forest follows, and the far forest does not - and it is
// the far forest that holds most of the trees.
//
// With the normals stored, the impostor is lit by the same light as everything
// else. Wicked keeps three slices (colour, normal, surface); here there are
// two, and the surface is left out - roughness is assumed constant, which on a
// distant tree does not show.

in vec2 vUV;
in float vLayer;
in float vRotation;
in vec3 vWorldPos;
in float vHeight01;
in float vFade;
in vec2 vMotionNDC;

layout(std140, binding = 0) uniform Camera
{
    mat4 uViewProj;
    vec4 uClipPlane;
    vec4 uCameraPos;
    mat4 uView;
    // Matches impostor.vert - one program, one layout for the block.
    mat4 uViewProjNoJitter;
    mat4 uPrevViewProjNoJitter;
};

layout(std140, binding = 4) uniform Environment
{
    vec4 uSun;
    vec4 uSunColor;
    vec4 uAmbient;
};

layout(std140, binding = 7) uniform ImpostorBlock
{
    vec4 uImpostorShape;
    vec4 uImpostorSwap;
};

#define uAlphaCut uImpostorSwap.z

layout(binding = 0) uniform sampler2DArray uImpostorAlbedo;
layout(binding = 1) uniform sampler2DArray uImpostorNormal;

layout(location = 0) out vec4 FragColor;
layout(location = 1) out vec2 FragVelocity;
layout(location = 2) out float FragReactive;

void main()
{
    vec4 albedo = texture(uImpostorAlbedo, vec3(vUV, vLayer));
    if (albedo.a < uAlphaCut)
        discard;

    // The normal was stored in WORLD space at capture time, with the tree at
    // the origin and unrotated. Each instance is spun about Y, so the same spin
    // has to be applied here - otherwise every tree's lighting would agree with
    // the lighting of exactly one of them.
    vec3 stored = texture(uImpostorNormal, vec3(vUV, vLayer)).xyz * 2.0 - 1.0;
    float s = sin(vRotation), c = cos(vRotation);
    vec3 N = normalize(vec3(stored.x * c - stored.z * s, stored.y,
                            stored.x * s + stored.z * c));

    vec3 L = normalize(-uSun.xyz);
    vec3 V = normalize(uCameraPos.xyz - vWorldPos);

    // The same two things the near foliage uses, so the mesh -> impostor
    // transition does not change appearance: half-lambert and translucency.
    float ndl = dot(N, L) * 0.5 + 0.5;
    ndl *= ndl;
    float translucency = pow(max(dot(V, -L), 0.0), 4.0) * 0.5;

    float ao = mix(0.6, 1.0, vHeight01);

    vec3 color = albedo.rgb * (uSunColor.rgb * (ndl + translucency) + uAmbient.rgb) * ao;

    // A real cross-fade, not a dither.
    //
    // The dither that was here first was a binary screen-space pattern: with no
    // TAA to integrate it, it shimmered on every camera move and read as a mesh
    // laid over the leaves.
    //
    // This way the mesh stays opaque and the impostor comes in over it with
    // alpha. The two cover nearly the same screen area, so the blend reads as a
    // smooth handover - and there is no pattern left to shimmer.
    FragColor = vec4(color, vFade);
    FragVelocity = vMotionNDC;
    // Only the handover itself is reactive. This draw blends with vFade as its
    // alpha, and that blend applies to this attachment too, so the value that
    // lands is vFade times what is written here: zero once the impostor has
    // fully taken over, and peaking while mesh and impostor are both on screen
    // - which is the moment a pixel's history describes the other one.
    FragReactive = 1.0 - abs(2.0 * vFade - 1.0);
}
