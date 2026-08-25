#version 450 core

in vec2 vUV;
in vec3 vWorldPos;
in vec3 vNormal;
in vec3 vTangent;
in vec3 vTint;
in float vRootFade;
in vec2 vMotionNDC;

#include "hair_uniforms.glsl"

layout(binding = 0) uniform sampler2D uHairTex;
layout(location = 0) out vec4 FragColor;
layout(location = 1) out vec2 FragVelocity;
layout(location = 2) out float FragReactive;

void main()
{
    vec4 tex = texture(uHairTex, vUV);
    // Even a plain white fallback texture produces a rounded fibre instead
    // of a stack of opaque rectangular cards. Authored textures can still
    // add breakup and fly-away detail through their alpha channel.
    float fibre = smoothstep(0.0, 0.18, vUV.x) * smoothstep(1.0, 0.82, vUV.x);
    float alpha = tex.a * fibre;
    if (uMode == 0)
    {
        if (alpha < uAlphaCut) discard;
    }
    else
    {
        if (alpha >= uAlphaCut || alpha < 0.025) discard;
    }

    vec3 T = normalize(vTangent);
    vec3 N = normalize(vNormal);
    if (!gl_FrontFacing) N = -N;
    vec3 L = normalize(-uLightDir);
    vec3 V = normalize(uCameraPos - vWorldPos);

    float diffuse = dot(N, L) * 0.5 + 0.5;
    float tl = dot(T, L);
    float tv = dot(T, V);
    float sinL = sqrt(max(0.0, 1.0 - tl * tl));
    float sinV = sqrt(max(0.0, 1.0 - tv * tv));
    float longitudinal = max(0.0, sinL * sinV + tl * tv);
    float exponent = mix(96.0, 12.0, uColorRoughness.w);
    float primary = pow(longitudinal, exponent);
    float secondary = pow(max(0.0, sinL * sinV + tl * (tv * 0.82)), exponent * 0.35) * 0.35;
    float transmission = pow(max(dot(V, -L), 0.0), 4.0) * uTransmission;

    // Strand maps provide opacity/detail; colour remains artist controlled
    // and does not inherit a checker/fallback texture's RGB.
    vec3 albedo = uColorRoughness.rgb * vTint;
    vec3 color = albedo * (uAmbient + uLightColor * (diffuse + transmission));
    // Hair highlights are strong but not unbounded white paint. Strength is
    // deliberately artist controlled and tint lets dark hair retain colour
    // instead of thousands of similarly aligned ribbons saturating together.
    vec3 specularColour = mix(vec3(1.0), normalize(albedo + vec3(0.001)), uSpecularTint);
    color += uLightColor * (primary + secondary) * uSpecularStrength * specularColour;
    color *= mix(0.48, 1.0, vRootFade);

    float outputAlpha = uMode == 0 ? 1.0 : smoothstep(0.025, uAlphaCut, alpha);
    FragColor = vec4(color, outputAlpha);
    FragVelocity = vMotionNDC;
    FragReactive = uMode == 0 ? 0.0 : 1.0;
}
