#version 450 core

layout(location = 0) in vec2 vUV;
layout(location = 1) in float vOpacity;

out vec4 FragColor;

layout(binding = 1) uniform sampler2D uFlareTex;

layout(location = 2) in float vDebug;

void main()
{
    if (vDebug != 0.0)
    {
        // Flat grey = the vertex shader's occlusion result, nothing else -
        // white where the sun's sample grid is unobstructed, black where a
        // depth read in front of it occluded every sample. If this does not
        // go black behind solid geometry, the depth read is the broken part,
        // not the sprite/blend.
        FragColor = vec4(vec3(vOpacity), 1.0);
        return;
    }

    vec4 c = texture(uFlareTex, vUV);

    // Written to an HDR target with additive blending: the texture's alpha
    // enters as a weight, not as transparency. That is what makes the flare
    // add light instead of covering the scene - and what lets bloom pick it
    // up afterwards. Pre-multiplying here and blending ONE/ONE gives the same
    // result as alpha-blending with the weight applied in the blend stage.
    FragColor = vec4(c.rgb * c.a * vOpacity, 1.0);
}
