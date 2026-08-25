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

layout(std140, binding = 2) uniform VolumetricRect
{
    vec4 uLightPosAndRange;     // xyz position, w range
    vec4 uLightColorAndDensity; // rgb color, w density
    vec4 uLightRightAndWidth;   // xyz right, w width
    vec4 uLightUpAndHeight;     // xyz up, w height
    mat4 uLightVP;
    vec4 uShadowAtlasMulAdd;
    vec4 uHasShadowAndTwoSided; // x hasShadow, y twoSided
};

#define uCameraPos       uCameraPosAndMaxDistance.xyz
#define uDestSize        uDestSizeAndSamples.xy
#define uSampleCount     int(uDestSizeAndSamples.z)
#define uScattering      uDestSizeAndSamples.w
#define uLightPos        uLightPosAndRange.xyz
#define uLightRange      uLightPosAndRange.w
#define uLightColor      uLightColorAndDensity.rgb
#define uDensity         uLightColorAndDensity.w
#define uLightRight      uLightRightAndWidth.xyz
#define uLightWidth      uLightRightAndWidth.w
#define uLightUp         uLightUpAndHeight.xyz
#define uLightHeight     uLightUpAndHeight.w
#define uHasShadow       uHasShadowAndTwoSided.x
#define uTwoSided        uHasShadowAndTwoSided.y

float HenyeyGreenstein(float cosTheta, float g)
{
    float g2 = g * g;
    float denom = 1.0 + g2 - 2.0 * g * cosTheta;
    return (1.0 - g2) / (4.0 * 3.14159265 * pow(max(denom, 1e-4), 1.5));
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

float RectShadowAt(vec3 P)
{
    if (uHasShadow < 0.5) return 1.0;
    vec4 lp = uLightVP * vec4(P, 1.0);
    if (lp.w <= 0.0) return 1.0;
    vec3 uv = (lp.xyz / lp.w) * 0.5 + 0.5;
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0 || uv.z < 0.0 || uv.z > 1.0)
        return 1.0;
    vec2 atlasUV = uv.xy * uShadowAtlasMulAdd.xy + uShadowAtlasMulAdd.zw;
    return texture(uShadowAtlas, vec3(atlasUV, uv.z));
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

    vec3 R = normalize(uLightRight) * (uLightWidth * 0.5);
    vec3 U = normalize(uLightUp) * (uLightHeight * 0.5);
    vec3 forward = normalize(cross(normalize(uLightUp), normalize(uLightRight)));
    vec3 p0 = uLightPos - R + U;
    vec3 p1 = uLightPos + R + U;
    vec3 p2 = uLightPos + R - U;
    vec3 p3 = uLightPos - R - U;

    float segLen = tEnd - tStart;
    float stepSize = segLen / float(uSampleCount);
    float offset = Dither(gl_FragCoord.xy);

    vec3 accum = vec3(0.0);
    for (int i = 0; i < uSampleCount; ++i)
    {
        float t = tStart + (float(i) + offset) * stepSize;
        vec3 P = uCameraPos + rayDir * t;

        vec3 Lv = uLightPos - P;
        float dist2 = dot(Lv, Lv);
        vec3 L = Lv * inversesqrt(max(dist2, 1e-8));

        if (uTwoSided < 0.5 && dot(P - uLightPos, forward) <= 0.0)
            continue;

        vec3 v0 = normalize(p0 - P);
        vec3 v1 = normalize(p1 - P);
        vec3 v2 = normalize(p2 - P);
        vec3 v3 = normalize(p3 - P);
        vec3 n0 = normalize(cross(v0, v1));
        vec3 n1 = normalize(cross(v1, v2));
        vec3 n2 = normalize(cross(v2, v3));
        vec3 n3 = normalize(cross(v3, v0));
        float g0 = acos(clamp(dot(-n0, n1), -1.0, 1.0));
        float g1 = acos(clamp(dot(-n1, n2), -1.0, 1.0));
        float g2 = acos(clamp(dot(-n2, n3), -1.0, 1.0));
        float g3 = acos(clamp(dot(-n3, n0), -1.0, 1.0));
        float solidAngle = clamp(g0 + g1 + g2 + g3 - 2.0 * 3.14159265, 0.0, 6.2831853);

        float facing = 0.25 * (max(dot(v0, L), 0.0) + max(dot(v1, L), 0.0) +
                               max(dot(v2, L), 0.0) + max(dot(v3, L), 0.0));

        float ratio2 = dist2 / max(1e-4, uLightRange * uLightRange);
        float rangeAtten = clamp(1.0 - ratio2, 0.0, 1.0);
        rangeAtten *= rangeAtten;

        float shadow = RectShadowAt(P);
        float phase = HenyeyGreenstein(dot(rayDir, -L), uScattering);

        accum += uLightColor * solidAngle * facing * rangeAtten * shadow * phase;
    }

    accum *= uDensity * stepSize;
    FragColor = vec4(accum, 1.0);
}
