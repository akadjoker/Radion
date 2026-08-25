#version 450 core

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

// The frame's sun and ambient, from the sky (EnvironmentBlock.h). uSun points
// the way the light travels, into the scene.
layout(std140, binding = 4) uniform Environment
{
    vec4 uSun;
    vec4 uSunColor;
    vec4 uAmbient;
    // Unused here, but this is the same buffer lit.frag reads - the probe
    // fields must stay declared or uTime lands at the wrong offset.
    vec4 uProbePositionAndMips;
    vec4 uProbeExtentsAndIntensity;
    vec4 uTime; // x = frame time, seconds
};

#ifdef HAS_ALBEDO
layout(binding = 0) uniform sampler2D uAlbedo;
#endif
layout(binding = 4) uniform sampler2D uAmbientOcclusion;
#ifdef HAS_EMISSIVE
layout(binding = 12) uniform sampler2D uEmissiveTex;
#endif
#ifdef HAS_DETAIL
#ifdef DETAIL_SEQUENCE
layout(binding = 3) uniform sampler2DArray uDetail;
#else
layout(binding = 3) uniform sampler2D uDetail;
#endif
#endif
#ifdef HAS_LIGHTMAP
layout(binding = 13) uniform sampler2D uLightmapTex;
#endif

#ifdef DETAIL_SEQUENCE
// Same as lit.frag's sampleDetailSequence() - see the comment there.
vec3 sampleDetailSequence(vec2 uv)
{
    float frameCount = max(sequence.x, 1.0);
    float looping = sequence.z;
    float f = uTime.x * sequence.y;
    f = looping > 0.5 ? mod(f, frameCount) : min(f, frameCount - 1.0);
    float f0 = floor(f);
    vec3 a = texture(uDetail, vec3(uv, f0)).rgb;
    if (sequence.w < 0.5)
        return a;
    float f1 = looping > 0.5 ? mod(f0 + 1.0, frameCount) : min(f0 + 1.0, frameCount - 1.0);
    vec3 b = texture(uDetail, vec3(uv, f1)).rgb;
    return mix(a, b, fract(f));
}
#endif

// Decals - see ApplyDecals() in lit.frag for the full explanation. Ported
// here too because terrain draws through this shader, not that one, and a
// decal painted on the ground has to land on whatever the ground actually
// uses.
#define ENTITY_TYPE_DECAL 4u
#define ENTITY_FLAG_DECAL_BASECOLOR_ONLY_ALPHA 8u

struct ShaderEntity
{
    vec3 position;
    float range;

    vec3 direction;
    float coneAngleCos;

    vec3 color;
    float coneAngleScale;

    uint type;
    uint flags;
    int matrixIndex;
    float shadowFade;

    vec4 shadowAtlasMulAdd;
    vec3 rectRight;
    float rectWidth;
    vec3 rectUp;
    float rectHeight;
};

layout(std430, binding = 2) readonly buffer EntityBuffer
{
    ShaderEntity entities[];
};

layout(std430, binding = 3) readonly buffer MatrixBuffer
{
    mat4 entityMatrices[];
};

#define TILE_SIZE 32
#define BUCKETS 8

layout(std430, binding = 4) readonly buffer TileBuffer
{
    uint tilesLuz[];
};

layout(std140, binding = 5) uniform Lighting
{
    vec4 uLightingCounts;   // x = entity count, y = tiled cull on
    vec4 uLightingTileGrid; // x = tiles across, y = tiles down, z = decals on
};

#define uEntityCount int(uLightingCounts.x)
#define uTiled       int(uLightingCounts.y)
#define uTileCount   ivec2(uLightingTileGrid.xy)
#define uDecalsOn    int(uLightingTileGrid.z)

layout(binding = 9)  uniform sampler2DArray uDecalAlbedo;
layout(binding = 10) uniform sampler2DArray uDecalNormal;
layout(binding = 11) uniform sampler2DArray uDecalSurface;

void ApplyDecals(vec3 P, inout vec3 albedo)
{
    if (uDecalsOn == 0)
        return;

    vec4 accCor = vec4(0.0);

    ivec2 tile = ivec2(gl_FragCoord.xy) / TILE_SIZE;
    int tileBase = (tile.y * uTileCount.x + tile.x) * BUCKETS;

    for (int b = 0; b < BUCKETS; ++b)
    {
        uint bucket = (uTiled == 1) ? tilesLuz[tileBase + b] : 0xFFFFFFFFu;
        if (uTiled == 0 && b * 32 >= uEntityCount)
            break;

        while (bucket != 0u)
        {
            int bit = findLSB(bucket);
            bucket &= ~(1u << uint(bit));
            int i = b * 32 + bit;
            if (i >= uEntityCount)
                break;

            ShaderEntity e = entities[i];
            if (e.type != ENTITY_TYPE_DECAL)
                continue;
            if (e.matrixIndex < 0)
                continue;
            if (accCor.a >= 1.0)
                break;

            mat4 proj = entityMatrices[e.matrixIndex];
            vec3 box = (proj * vec4(P, 1.0)).xyz;
            if (any(greaterThan(abs(box), vec3(1.0))))
                continue;

            vec2 uv = box.xy * vec2(0.5, -0.5) + 0.5;
            float camada = float(floatBitsToInt(proj[0][3]));
            float edge = 1.0 - pow(clamp(abs(box.z), 0.0, 1.0), 8.0);

            vec4 amostra = texture(uDecalAlbedo, vec3(uv, camada));
            float a = e.coneAngleScale * edge * amostra.a;
            if (a <= 0.0)
                continue;

            vec3 cor = e.color;
            if ((e.flags & ENTITY_FLAG_DECAL_BASECOLOR_ONLY_ALPHA) == 0u)
                cor *= amostra.rgb;

            accCor.rgb += (1.0 - accCor.a) * a * cor;
            accCor.a = a + (1.0 - a) * accCor.a;
        }
    }

    if (accCor.a > 0.0)
        albedo = albedo * (1.0 - accCor.a) + accCor.rgb;
}

in vec3 vWorldPos;
in vec3 vNormal;
in vec2 vUV;
in vec2 vUV2;
in vec4 vColor;
#ifndef NO_TEMPORAL
in vec2 vMotionNDC;
#endif
layout(location = 0) out vec4 FragColor;
#ifndef NO_TEMPORAL
layout(location = 1) out vec2 FragVelocity;
layout(location = 2) out float FragReactive;
#endif

void main()
{
    vec3 normal = normalize(vNormal);
    float diffuse = max(dot(normal, normalize(-uSun.xyz)), 0.0);
    float sky = clamp(normal.y * 0.5 + 0.5, 0.0, 1.0);
    float ambientStrength = custom1.x > 0.0 ? custom1.x : 0.55;
    float diffuseStrength = custom1.y > 0.0 ? custom1.y : 0.45;
    vec2 aoSize = vec2(textureSize(uAmbientOcclusion, 0));
    float ao = texture(uAmbientOcclusion, gl_FragCoord.xy / (aoSize * 2.0)).r;
    float lighting = (ambientStrength + sky * 0.10) * ao + diffuse * diffuseStrength;

    vec4 albedo = vec4(1.0);
#ifdef HAS_ALBEDO
    albedo = texture(uAlbedo, vUV * uvTransform.xy + uvTransform.zw);
#endif
#ifdef HAS_DETAIL
    vec2 detailUV = vUV * max(custom0.x, 1.0);
#ifdef DETAIL_SEQUENCE
    vec3 detail = sampleDetailSequence(detailUV);
#else
    vec3 detail = texture(uDetail, detailUV).rgb;
#endif
    albedo.rgb = mix(albedo.rgb, albedo.rgb * detail * 2.0, clamp(custom0.y, 0.0, 1.0));
#endif
    ApplyDecals(vWorldPos, albedo.rgb);

#ifdef HAS_LIGHTMAP
    albedo.rgb *= texture(uLightmapTex, vUV2).rgb;
#endif

    vec3 color = baseColor.rgb * albedo.rgb * vColor.rgb;

    // The sky decides the colour, the material keeps deciding the strength.
    // Both tints are normalised to a maximum of one so this changes what the
    // light looks like and not how much of it there is - every scene was
    // tuned against the old flat white, and shifting the brightness here
    // would send us chasing all of them at once.
    vec3 ambientTint = uAmbient.rgb / max(max(uAmbient.r, max(uAmbient.g, uAmbient.b)), 1e-4);
    if (custom1.z < 0.5)
        color *= ambientTint * ((ambientStrength + sky * 0.10) * ao) +
                 uSunColor.rgb * (diffuse * diffuseStrength);

    // AFTER the lighting multiply, never inside it: emissive is light the
    // surface gives off, so nothing that decides how much light REACHES it
    // applies. Above 1 is deliberate - that is what the bloom threshold reads.
    vec3 emissiveTerm = emissive.rgb * emissive.w;
#ifdef HAS_EMISSIVE
    emissiveTerm *= texture(uEmissiveTex, vUV).rgb;
#endif
    color += emissiveTerm;

    FragColor = vec4(color, baseColor.a * albedo.a);
#ifndef NO_TEMPORAL
    FragVelocity = vMotionNDC;
    FragReactive = 0.0;
#endif
}
