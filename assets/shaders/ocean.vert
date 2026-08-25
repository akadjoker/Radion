#version 450 core

// Gerstner (trochoidal) waves.
//
// A plain sine wave only moves vertices UP and DOWN, and the result reads as
// jelly: round crests, round troughs. Real water does not do that - the
// particles trace circles, which sharpens the crests and widens the troughs.
//
// Gerstner does exactly that: on top of the vertical displacement, it pushes
// each vertex HORIZONTALLY along the propagation direction, more at the crest
// than in the trough.
//
// For each wave with direction D, wavenumber k = 2*pi/wavelength, angular
// speed w = sqrt(g*k) (deep-water dispersion) and amplitude A:
//
//     phase = k * dot(D, xz) - w * t
//     y    += A * sin(phase)
//     xz   += D * (Q * A * cos(phase))
//
// Q controls how sharp the crest gets. Past Q*k*A > 1 the wave folds over
// itself and loops appear on the surface - which is why Q is normalised by
// the wave count below.

layout (location = 0) in vec3 aPosition;

out vec3 vWorldPos;
out vec3 vNormal;
out vec2 vUV;
out float vCrest; // 0..1: how high this point sits within the wave

#include "ocean_uniforms.glsl"

const float G = 9.81;

// A handful of pure sine terms line up in a perfectly regular lattice - real
// water never has just a few frequencies. Shifting each wave's phase by a
// fixed, unrelated offset breaks that alignment without touching direction,
// wavelength or amplitude. Authored, not random-per-frame, so the surface
// stays a fixed shape that only scrolls with time. Must match
// Ocean::heightAt()/normalAt() exactly, or the buoyancy query disagrees with
// what is drawn.
const vec2 kPhaseOffset[6] = vec2[](
    vec2(0.0, 0.0),
    vec2(311.7, 172.3),
    vec2(-198.4, 402.1),
    vec2(87.6, -266.9),
    vec2(-355.2, -114.8),
    vec2(224.1, 333.6)
);

void main()
{
    vec3 P = (uModel * vec4(aPosition, 1.0)).xyz;
    vec3 base = P;
    float t = uTime * uTimeScale;

    // Partial-derivative accumulators, so the normal comes out analytic
    // instead of from finite differences - exact, and effectively free.
    vec3 tangent  = vec3(1.0, 0.0, 0.0);
    vec3 binormal = vec3(0.0, 0.0, 1.0);

    for (int i = 0; i < uWaveCount && i < 6; ++i)
    {
        // The CPU side already only ever uploads a normalised, non-zero
        // direction - see Ocean::safeDirection() - but normalize() on a zero
        // vector is undefined, not zero, so this stays defended on its own.
        vec2  rawDirection = uWaves[i].xy;
        vec2  D = dot(rawDirection, rawDirection) < 1e-8 ? vec2(1.0, 0.0) : normalize(rawDirection);
        float wavelength = max(0.001, uWaves[i].z);
        float A = uWaves[i].w;

        float k = 6.28318530718 / wavelength;
        float w = sqrt(G * k);
        // Q divided by the wave count: several waves each at a high Q fold
        // over and loop when summed together.
        float Q = uSteepness / (k * A * float(uWaveCount) + 1e-5);

        float phase = k * dot(D, base.xz + kPhaseOffset[i]) - w * t;
        float s = sin(phase);
        float c = cos(phase);

        P.y  += A * s;
        P.xz += D * (Q * A * c);

        float QA = Q * A;
        tangent += vec3(-D.x * D.x * QA * k * s,
                          D.x * A * k * c,
                         -D.x * D.y * QA * k * s);
        binormal += vec3(-D.x * D.y * QA * k * s,
                          D.y * A * k * c,
                         -D.y * D.y * QA * k * s);
    }

    // Relative height inside the wave: 1 at the crest, 0 in the trough. A
    // cheap proxy for where the surface is about to fold, which is where real
    // water foams.
    float totalAmplitude = 0.0;
    for (int i = 0; i < uWaveCount && i < 6; ++i)
        totalAmplitude += uWaves[i].w;
    vCrest = clamp((P.y - base.y) / max(0.001, totalAmplitude * 0.7), 0.0, 1.0);

    vWorldPos = P;
    vNormal   = normalize(cross(binormal, tangent));
    vUV       = base.xz * 0.02;

    gl_Position = uViewProj * vec4(P, 1.0);
}
