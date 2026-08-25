#version 450 core
 

layout(location = 0) out vec2 vUV;
layout(location = 1) out float vOpacity;
layout(location = 2) out float vDebug;

layout(binding = 0) uniform sampler2D uDepthTex;

layout(std140, binding = 0) uniform LensFlareBlock
{
    vec4 uLightScreenPosAndOcclusion; // xyz = light screen pos (z = depth [0,1]), w = occlusion sample radius in texels
    vec4 uOffsetAndSize;              // x = offset (the ghost's position multiplier), yz = sprite size in pixels
    vec4 uScreenSizeRcp;              // xy = 1 / render resolution, z = debug (nonzero paints vOpacity raw)
};

const vec2 BILLBOARD[4] = vec2[4](
    vec2(-1, -1), vec2(1, -1), vec2(-1, 1), vec2(1, 1)
);

void main()
{
    vUV = BILLBOARD[gl_VertexID] * 0.5 + 0.5;

    const vec3 lightScreenPos = uLightScreenPosAndOcclusion.xyz;
    const float occlusionRange = uLightScreenPosAndOcclusion.w;

    // ---- Occlusion ----
    // A flare that does not disappear when its source goes behind a pillar
    // reads as fake immediately. Sample the depth buffer in a GRID around the
    // light's position and count the fraction of samples with nothing closer
    // in front of them.
    //
    // Sampling a grid instead of a single point is what gives a CONTINUOUS
    // visibility between 0 and 1: the flare fades out as the pillar covers
    // it, instead of popping off all at once.
    //
    // This runs in the VERTEX shader, only 4 vertices per element - the cost
    // is negligible even with hundreds of samples.
    //
    // The sun sits very far away and projects to z > 1 (past the far plane).
    // Left unclamped, refDepth would land above 1 and the test would fail on
    // EVERY pixel, since the depth buffer's own maximum is 1.0 - the flare
    // would appear only while the sun happened to fall inside range and
    // disappear the moment it passed it.
    //
    // Clamped, the test becomes "is this pixel sky?", which is exactly the
    // right criterion for a source at infinity.
    const float refDepth = min(lightScreenPos.z, 0.999999);
    const vec2 step = uScreenSizeRcp.xy;
    float samples = 0.0;
    float visible = 0.0;

    for (float y = -occlusionRange; y <= occlusionRange; y += 1.0)
    {
        for (float x = -occlusionRange; x <= occlusionRange; x += 1.0)
        {
            vec2 uv = lightScreenPos.xy + vec2(x, y) * step;
            samples += 1.0;
            // Off-screen counts as visible: the source can be just barely
            // out of frame and the flare still makes sense.
            if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0)
            {
                visible += 1.0;
                continue;
            }
            // Nothing in front of the light at this sample -> it sees the light.
            visible += (texture(uDepthTex, uv).r >= refDepth) ? 1.0 : 0.0;
        }
    }
    float opacity = visible / max(1.0, samples);

    // ---- Element position ----
    // pos: the light, relative to screen centre, in NDC.
    const vec2 pos = (lightScreenPos.xy - 0.5) * vec2(2.0, 2.0);
    const vec2 modPos = pos * uOffsetAndSize.x;

    // Elements far from the source fade out. Without this the ghosts on the
    // opposite side keep the same strength as the light itself and give the
    // trick away.
    opacity *= clamp(1.0 - length(pos - modPos), 0.0, 1.0);

    vOpacity = opacity;
    vDebug = uScreenSizeRcp.z;

    gl_Position = vec4(modPos + BILLBOARD[gl_VertexID] * uOffsetAndSize.yz * uScreenSizeRcp.xy,
                       0.0, 1.0);
}
