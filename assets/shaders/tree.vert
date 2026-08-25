#version 450 core

// Instanced trees. The geometry comes from the mesh, the instance from the
// SSBO. Port of the reference demo's tree.vert - the wind below is what the
// parameter values mean, so changing it changes every tree tuned against it.
//
// The reference used loose uniforms; this engine has no per-location uniform
// call, only UBO/SSBO/texture bindings, so they live in TreeBlock instead.

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aTangent; // xyz = tangent, w = handedness
layout(location = 3) in vec2 aUV;

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

layout(std140, binding = 6) uniform TreeBlock
{
    // x = model height in metres, y = time, z = wind strength,
    // w = 1 when this draw is the leaf cards.
    vec4 uTreeWind;
    // x = twig array layer, y = alpha cut, z = bark bump strength,
    // w = capture mode (0 normal, 1 raw albedo, 2 world normals).
    vec4 uTreeSurface;
    // x = the time one frame ago. The sway is a pure function of time, so
    // evaluating it twice is what gives the leaves a real previous position
    // instead of one that only follows the trunk.
    vec4 uTreeTemporal;
};

// The reference kept the raw generator output and scaled it here, through a
// `uNormalizacao` factor. This engine scales the mesh once at build time
// instead (Forest::buildSpecies), so the vertices arrive already in metres -
// which is why the height below divides by the model's real height and the
// wind's axis distance needs no factor at all.
#define uModelHeight   uTreeWind.x
#define uTime          uTreeWind.y
#define uWind          uTreeWind.z
#define uIsLeaves      int(uTreeWind.w)
#define uPrevTime      uTreeTemporal.x

out vec3 vNormal;
out vec4 vTangent;
out vec2 vUV;
out vec3 vWorldPos;
out float vHeight01;
out vec2 vMotionNDC;

// The sway, as a pure function of time. Called once for this frame and once
// for the previous one, which is what makes the leaves' motion vector follow
// the foliage instead of only the trunk.
vec3 WindOffset(vec3 rawPosition, vec3 instancePosition, float axisDistance,
                float height01, float time, float wind)
{
    float phase = dot(rawPosition, vec3(0.71, 0.33, 0.57))
                + instancePosition.x * 0.12 + instancePosition.z * 0.09;
    float wave = sin(time * 1.1 + phase) + 0.5 * sin(time * 1.9 + phase * 1.7);
    vec3 direction = normalize(vec3(0.82, 0.0, 0.57));
    vec3 offset = direction * wave + vec3(0.0, 0.25 * wave * wave, 0.0);
    return offset * (wind * height01 * axisDistance * 0.06);
}

void main()
{
    TreeInstance it = instances[gl_InstanceID];

    vec3 p = aPosition * it.scale;
    vec3 n = aNormal;

    // Spun about its own axis: without this, a hundred copies of one mesh read
    // immediately as a hundred copies of one mesh.
    float s = sin(it.rotation), c = cos(it.rotation);
    p = vec3(p.x * c - p.z * s, p.y, p.x * s + p.z * c);
    n = vec3(n.x * c - n.z * s, n.y, n.x * s + n.z * c);
    // The tangent turns with the instance, the same as the normal. Without it
    // the bark's relief pointed the wrong way on every rotated tree.
    vec3 t = vec3(aTangent.x * c - aTangent.z * s, aTangent.y,
                  aTangent.x * s + aTangent.z * c);

    // Normalised height, for the wind and for the AO. From the RAW vertex, so
    // an instance's own scale does not change where along the trunk a given
    // vertex counts as being.
    vHeight01 = clamp(aPosition.y / max(0.001, uModelHeight), 0.0, 1.0);

    vec3 world = it.position + p;

    // ---- Wind ----
    // Only the LEAVES sway: the trunk is rigid and its base is in the ground.
    // Applying wind to the trunk gives a rubber tree.
    //
    // The phase comes from the vertex's LOCAL position, not just the
    // instance's. Without that, every leaf on one tree moves in lockstep -
    // which reads as something sliding rather than foliage swaying. With a
    // phase per twig each goes at its own time and the mass undulates.
    vec3 worldPrev = world;
    if (uIsLeaves == 1)
    {
        // The phase is taken from the vertex position normalised back to the
        // generator's own scale (its raw model stands about two units tall),
        // NOT from the metres this engine pre-scaled it to.
        //
        // Getting this wrong tears the cards apart: a twig card is half a metre
        // across, so in metres its four corners differ by ~0.4 radians of phase
        // and each corner rides its own wave. The card stops being a card and
        // smears into a streak.
        vec3 rawPosition = aPosition / max(0.001, uModelHeight) * 2.0;

        // Weighted by distance from the trunk's axis: branch tips move a lot,
        // foliage near the trunk almost none. Uniform, the whole crown slides
        // sideways as one. Already in metres - see uModelHeight above.
        float axisDistance = length(aPosition.xz);

        world += WindOffset(rawPosition, it.position, axisDistance, vHeight01, uTime, uWind);
        worldPrev += WindOffset(rawPosition, it.position, axisDistance, vHeight01, uPrevTime,
                                uWind);
    }

    vWorldPos = world;
    vNormal = normalize(n);
    vTangent = vec4(normalize(t), aTangent.w);
    vUV = aUV;

    gl_Position = uViewProj * vec4(world, 1.0);
    vec4 current = uViewProjNoJitter * vec4(world, 1.0);
    vec4 previous = uPrevViewProjNoJitter * vec4(worldPrev, 1.0);
    vMotionNDC = previous.xy / previous.w - current.xy / current.w;
}
