#version 450 core

// Trees: trunk and leaves in the SAME shader, told apart by uIsLeaves.
//
// One shader because the two draws share nearly everything - instance, wind,
// lighting - and what differs is two short branches. Two shaders would mean
// duplicating the whole vertex shader.

in vec3 vNormal;
in vec4 vTangent;
in vec2 vUV;
in vec3 vWorldPos;
in float vHeight01;
in vec2 vMotionNDC;

layout(std140, binding = 6) uniform TreeBlock
{
    vec4 uTreeWind;
    vec4 uTreeSurface;
    vec4 uTreeTemporal;
};

#define uIsLeaves   int(uTreeWind.w)
#define uAlphaCut   uTreeSurface.y
#define uBumpForce  uTreeSurface.z
#define uCapture    int(uTreeSurface.w)

// The frame's sun and ambient (EnvironmentBlock.h). uSun points the way the
// light travels, into the scene - so the shading direction is the same one the
// shadow cascades were built from.
layout(std140, binding = 4) uniform Environment
{
    vec4 uSun;
    vec4 uSunColor;
    vec4 uAmbient;
};

layout(std140, binding = 0) uniform Camera
{
    mat4 uViewProj;
    vec4 uClipPlane;
    vec4 uCameraPos;
    mat4 uView;
    // Declared to match tree.vert: the same program cannot hold two different
    // layouts for one block, even where only the vertex stage reads them.
    mat4 uViewProjNoJitter;
    mat4 uPrevViewProjNoJitter;
};

layout(binding = 0) uniform sampler2D uBark;
layout(binding = 1) uniform sampler2D uBarkNormal;
// A plain 2D texture, where the reference used a sampler2DArray with a layer
// index per species. It needed the array because it drew every species in one
// batch; here there is one draw per species already, so the array would buy
// nothing and cost a resample of every leaf card to a common size.
layout(binding = 2) uniform sampler2D uTwigTex;

layout(location = 0) out vec4 FragColor;
layout(location = 1) out vec2 FragVelocity;
layout(location = 2) out float FragReactive;

void main()
{
    // First, not last: the capture modes below return early, and an attachment
    // left unwritten on those paths keeps whatever the clear put there.
    FragVelocity = vMotionNDC;
    FragReactive = 0.0;
    vec3 N = normalize(vNormal);
    vec3 L = normalize(-uSun.xyz);
    vec3 V = normalize(uCameraPos.xyz - vWorldPos);

    vec3 albedo;
    float ndl;
    float translucency = 0.0;

    if (uIsLeaves == 1)
    {
        vec4 card = texture(uTwigTex, vUV);
        if (card.a < uAlphaCut)
            discard; // without this every leaf is a rectangular card
        albedo = card.rgb;

        // Half-lambert: a leaf lit from the side has no black face, because
        // light passes through it. With max(dot,0) half the crown goes fully
        // dark and reads as plastic.
        ndl = dot(N, L) * 0.5 + 0.5;
        ndl *= ndl;

        // Backlit translucency: the most characteristic thing foliage does
        // against the sun, and what is most missed when it is absent.
        translucency = pow(max(dot(V, -L), 0.0), 4.0) * 0.7;
    }
    else
    {
        albedo = texture(uBark, vUV).rgb;

        // Bark normal map with a real TBN. The tangents come from the UV
        // derivatives in the generator (AssetManager::recalculateTangents) -
        // proctree.js does not produce them, because its own demo has no
        // normal map.
        //
        // An improvised basis built from the normal alone put the relief at
        // random angles: the trunk's grooves pointed different ways per face
        // instead of running along the branch the way the UV says.
        vec3 nm = texture(uBarkNormal, vUV).xyz * 2.0 - 1.0;
        nm.xy *= uBumpForce;

        // Re-orthogonalised per fragment: interpolating between vertices does
        // not preserve the basis's orthogonality.
        vec3 T = normalize(vTangent.xyz - N * dot(N, vTangent.xyz));
        vec3 B = cross(N, T) * vTangent.w;
        N = normalize(mat3(T, B, N) * nm);

        ndl = max(dot(N, L), 0.0);
    }

    // The base takes less light: the crown occluding the trunk, and the
    // foliage occluding itself. Without it the tree looks like it floats.
    float ao = mix(0.55, 1.0, vHeight01);

    // ---- Impostor capture modes ----
    // Alpha is 1 wherever there is geometry and the target is cleared to
    // (0,0,0,0), so the tree's silhouette comes out of the alpha channel free.
    if (uCapture == 1)
    {
        FragColor = vec4(albedo, 1.0); // unlit: the impostor applies the light
        return;
    }
    if (uCapture == 2)
    {
        FragColor = vec4(N * 0.5 + 0.5, 1.0); // WORLD normal, encoded
        return;
    }

    vec3 color = albedo * (uSunColor.rgb * (ndl + translucency) + uAmbient.rgb) * ao;
    FragColor = vec4(color, 1.0);
}
