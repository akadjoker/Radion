#version 450 core
layout(binding = 1) uniform sampler2D uShadowMap;
layout(std140, binding = 0) uniform BakeBlock {
    mat4 uShadowVP;
    mat4 uModel;
    vec4 uLightDirection;
    vec4 uLightColor;
    vec4 uParams; // x = bias, y = ground bounce fraction, z = PCF radius (shadow texels), w = sample weight
    vec4 uAmbientSky; // rgb = sky light for everything the sun does not reach
    vec4 uJitter;
};
#define uBias uParams.x
#define uAmbientGround uParams.y
#define uFilterRadius uParams.z
#define uSampleWeight uParams.w
centroid in vec3 vWorldPosition;
centroid in vec3 vNormal;
centroid in vec4 vShadowPosition;
layout(location = 0) out vec4 FragColor;
void main()
{
    vec3 n = normalize(vNormal);
    vec3 lightToSurface = normalize(uLightDirection.xyz);
    float direct = max(dot(n, -lightToSurface), 0.0);
    vec3 shadowCoord = vShadowPosition.xyz / vShadowPosition.w;
    shadowCoord = shadowCoord * 0.5 + 0.5;
    float shadow = 1.0;
    if (shadowCoord.x >= 0.0 && shadowCoord.x <= 1.0 && shadowCoord.y >= 0.0 &&
        shadowCoord.y <= 1.0 && shadowCoord.z >= 0.0 && shadowCoord.z <= 1.0)
    {
        // A single hard depth compare bakes a jagged, staircased edge
        // permanently into the texture - there is no next frame to hide it
        // behind temporal jitter like the real-time cascades get. Averaging
        // a small neighbourhood turns that step into a soft gradient a few
        // texels wide instead. uFilterRadius == 0 keeps the old single tap.
        vec2 texelSize = uFilterRadius / vec2(textureSize(uShadowMap, 0));
        shadow = 0.0;
        float taps = 0.0;
        for (int y = -1; y <= 1; ++y)
            for (int x = -1; x <= 1; ++x)
            {
                vec2 uv = shadowCoord.xy + vec2(x, y) * texelSize;
                float depth = texture(uShadowMap, uv).r;
                shadow += shadowCoord.z - uBias <= depth ? 1.0 : 0.0;
                taps += 1.0;
            }
        shadow /= taps;
    }
    // Additive blend accumulates several sun-disk samples into the same
    // texel (see bake()'s sample loop) - each one contributes its share of
    // the average. Alpha is left unweighted and grows with sample count,
    // but save()'s dilate pass only tests it against zero, so a covered
    // texel reading N instead of 1 makes no difference to that mask.
    // Sky above, ground bounce below, the surface's own normal deciding how
    // much of each it sees. A flat term instead gives a floor, a wall and the
    // underside of a balcony the same value, and with only the sun traced on
    // top of it that leaves the whole scene at two brightnesses - lit, or the
    // one ambient constant. This is what puts a gradient back between them.
    vec3 ambient = mix(uAmbientSky.rgb * uAmbientGround, uAmbientSky.rgb, 0.5 + 0.5 * n.y);
    FragColor = vec4((ambient + direct * shadow * uLightColor.rgb) * uSampleWeight, 1.0);
}
