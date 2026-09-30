#ifndef RADION_ENVIRONMENT_BLOCK_H
#define RADION_ENVIRONMENT_BLOCK_H

#include "Math.h"

namespace Radion
{

struct SkySettings;
struct FrameContext;

// The frame's outdoor lighting, filled from the sky and bound by ForwardPass at BindingEnvironment.
// sunDirection points the way light travels (the sky's direction-to-sun, negated once).
struct EnvironmentBlock
{
    Math::vec4 sunDirection = Math::vec4(0.0f, -1.0f, 0.0f, 0.0f);

    // Sun colour after atmospheric attenuation; not multiplied by sunIntensity (HDR; the pipeline writes to the back buffer, see ENGINE.md 7.1).
    Math::vec4 sunColor = Math::vec4(1.0f);

    // w is uLightmapIntensity (lit.frag), a multiplier on the lightmap sample.
    Math::vec4 ambient = Math::vec4(0.12f, 0.16f, 0.24f, 1.0f);

    // Environment probe for reflections; w is the mip count, which maps roughness to a level.
    Math::vec4 probePositionAndMips = Math::vec4(0.0f, 0.0f, 0.0f, 1.0f);

    // Probe box half-extents; ALL ZERO means infinitely distant (no parallax correction, EnvironmentReflection_Global path). w multiplies the reflection.
    Math::vec4 probeExtentsAndIntensity = Math::vec4(0.0f, 0.0f, 0.0f, 1.0f);

    // x = frame time in seconds. y = uLightmapShadowLift (lit.frag): flat lift scaled by uAmbient added to the lightmap sample. zw unused (std140 alignment).
    Math::vec4 timeAndUnused = Math::vec4(0.0f);
};

// Reads the sky, or leaves the defaults when there is none or it is off.
EnvironmentBlock environmentFromSky(const SkySettings* sky);

// The only environment a pass should use: sun direction/colour come from the scene's first directional light if any, else the sky; ambient from the sky.
// Must match the shadow cascades' light (see ShadowPass), or shading and shadow tests disagree and acne appears.
EnvironmentBlock environmentForFrame(const FrameContext& frame);

} // namespace Radion

#endif // RADION_ENVIRONMENT_BLOCK_H
