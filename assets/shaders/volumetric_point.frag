#version 450 core

out vec4 FragColor;

layout(binding = 0) uniform sampler2D uDepthTex;
layout(binding = 2) uniform sampler2DShadow uShadowAtlas;

layout(std140, binding = 1) uniform VolumetricSettings
{
    mat4 uInvViewProj;
    vec4 uCameraPosAndMaxDistance;
    vec4 uDestSizeAndSamples;
};

layout(std140, binding = 2) uniform VolumetricPoint
{
    vec4 uLightPosAndRange;    // xyz position, w range
    vec4 uLightColorAndDensity; // rgb color, w density
    vec4 uShadowAtlasMulAdd;
    vec4 uHasShadow; // x = 0/1
};

#define uCameraPos       uCameraPosAndMaxDistance.xyz
#define uDestSize        uDestSizeAndSamples.xy
#define uSampleCount     int(uDestSizeAndSamples.z)
#define uScattering      uDestSizeAndSamples.w
#define uLightPos        uLightPosAndRange.xyz
#define uLightRange      uLightPosAndRange.w
#define uLightColor      uLightColorAndDensity.rgb
#define uDensity         uLightColorAndDensity.w

float HenyeyGreenstein(float cosTheta, float g)
{
    float g2 = g * g;
    float denom = 1.0 + g2 - 2.0 * g * cosTheta;
    return (1.0 - g2) / (4.0 * 3.14159265 * pow(max(denom, 1e-4), 1.5));
}

void CubeFaceUV(vec3 dir, out int face, out vec2 uv)
{
    vec3 a = abs(dir);
    if (a.x >= a.y && a.x >= a.z)
    {
        if (dir.x > 0.0) { face = 0; uv = vec2(-dir.z, -dir.y) / a.x; }
        else             { face = 1; uv = vec2( dir.z, -dir.y) / a.x; }
    }
    else if (a.y >= a.x && a.y >= a.z)
    {
        if (dir.y > 0.0) { face = 2; uv = vec2( dir.x,  dir.z) / a.y; }
        else             { face = 3; uv = vec2( dir.x, -dir.z) / a.y; }
    }
    else
    {
        if (dir.z > 0.0) { face = 4; uv = vec2( dir.x, -dir.y) / a.z; }
        else             { face = 5; uv = vec2(-dir.x, -dir.y) / a.z; }
    }
    uv = uv * 0.5 + 0.5;
}

float PointShadowAt(vec3 P, vec3 lightPos, float range)
{
    if (uHasShadow.x < 0.5) return 1.0;

    vec3 toFrag = P - lightPos;
    float cmp = clamp(length(toFrag) / max(0.0001, range), 0.0, 1.0);

    int face; vec2 uv;
    CubeFaceUV(toFrag, face, uv);

    vec2 tileTexel = uShadowAtlasMulAdd.xy;
    vec2 border = vec2(0.5) / max(vec2(1.0), tileTexel * 4096.0);
    uv = clamp(uv, border, vec2(1.0) - border);

    vec2 atlasUV = vec2(uv.x + float(face), uv.y) * uShadowAtlasMulAdd.xy
                 + uShadowAtlasMulAdd.zw;
    return texture(uShadowAtlas, vec3(atlasUV, cmp));
}

float Dither(vec2 pos)
{
    return fract(52.9829189 * fract(dot(pos, vec2(0.06711056, 0.00583715))));
}

vec3 WorldPosFromDepth(vec2 uv, float depth)
{
    vec4 clip = vec4(uv * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
    vec4 world = uInvViewProj * clip;
    return world.xyz / world.w;
}

bool RaySphere(vec3 ro, vec3 rd, vec3 center, float radius, out float t0, out float t1)
{
    vec3 oc = ro - center;
    float b = dot(oc, rd);
    float c = dot(oc, oc) - radius * radius;
    float disc = b * b - c;
    if (disc < 0.0) return false;
    float sq = sqrt(disc);
    t0 = -b - sq;
    t1 = -b + sq;
    return true;
}

void main()
{
    vec2 uv = gl_FragCoord.xy / uDestSize;

    float sceneDepth = texture(uDepthTex, uv).r;
    vec3 scenePos = WorldPosFromDepth(uv, min(sceneDepth, 0.9999));

    vec3 toScene = scenePos - uCameraPos;
    float sceneDist = length(toScene);
    vec3 rayDir = toScene / max(sceneDist, 1e-5);

    float t0, t1;
    if (!RaySphere(uCameraPos, rayDir, uLightPos, uLightRange, t0, t1))
        discard;

    float tStart = max(t0, 0.0);
    float tEnd   = min(t1, sceneDist);
    if (tEnd <= tStart) discard;

    float segLen = tEnd - tStart;
    float stepSize = segLen / float(uSampleCount);
    float offset = Dither(gl_FragCoord.xy);

    vec3 accum = vec3(0.0);
    for (int i = 0; i < uSampleCount; ++i)
    {
        float t = tStart + (float(i) + offset) * stepSize;
        vec3 P = uCameraPos + rayDir * t;

        vec3 toLight = uLightPos - P;
        float dist2 = dot(toLight, toLight);
        float dist = sqrt(dist2);
        vec3 L = toLight / max(dist, 1e-5);

        float ratio2 = dist2 / max(1e-4, uLightRange * uLightRange);
        float atten = clamp(1.0 - ratio2, 0.0, 1.0);
        atten *= atten;

        float phase = HenyeyGreenstein(dot(rayDir, -L), uScattering);
        float shadow = PointShadowAt(P, uLightPos, uLightRange);

        accum += uLightColor * atten * phase * shadow;
    }

    accum *= uDensity * stepSize;
    FragColor = vec4(accum, 1.0);
}
