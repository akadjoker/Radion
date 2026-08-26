#ifndef RADION_ENVIRONMENT_BLOCK_H
#define RADION_ENVIRONMENT_BLOCK_H

#include <glm/glm.hpp>

namespace Radion
{

struct SkySettings;
struct FrameContext;

// The frame's outdoor lighting, filled from the sky and bound by ForwardPass
// at BindingEnvironment. Before this every shader carried its own hardcoded
// sun - unlit.frag had one direction, the grass another - so moving the time
// of day changed the sky and nothing under it.
//
// The sun here points the way light travels, into the scene: the sky stores
// the direction towards the sun, and this is its negation, done once rather
// than in each shader.
struct EnvironmentBlock
{
    Math::Vec4 sunDirection = Math::Vec4(0.0f, -1.0f, 0.0f, 0.0f);

    // Sun colour after the atmosphere has taken its cut - so it reddens at
    // dusk and goes to nothing at night, which is the sky's whole point. Not
    // multiplied by sunIntensity: that is an HDR value and this pipeline still
    // writes straight to the back buffer (see ENGINE.md 7.1).
    Math::Vec4 sunColor = Math::Vec4(1.0f);

    // w carries uLightmapIntensity (lit.frag): a multiplier on the lightmap
    // sample, default 1.0 to match the alpha this vec4 already defaulted to
    // before that use existed.
    Math::Vec4 ambient = Math::Vec4(0.12f, 0.16f, 0.24f, 1.0f);

    // The environment probe, for image-based reflections. w carries the mip
    // count because the shader picks a level from roughness and the chain
    // length is what maps one onto the other.
    Math::Vec4 probePositionAndMips = Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f);

    // Half-extents of the probe's box. ALL ZERO means an infinitely distant
    // probe: the shader then samples the reflection direction straight,
    // skipping the parallax correction - the reference keeps these as two
    // separate shader paths (EnvironmentReflection_Global and _Local), and
    // this is the one flag that chooses between them. w is a plain multiplier
    // on the reflection, for turning it down without moving anything.
    Math::Vec4 probeExtentsAndIntensity = Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f);

    // x = frame time in seconds, for an animated detail sequence to pick its
    // frame from. y carries uLightmapShadowLift (lit.frag): scales uAmbient
    // as a flat lift added to the lightmap sample before it hits albedo, so
    // a dark bake can be raised without touching the bake itself. zw unused,
    // kept for std140 alignment.
    Math::Vec4 timeAndUnused = Math::Vec4(0.0f);
};

// Reads the sky, or leaves the defaults when there is none or it is off.
EnvironmentBlock environmentFromSky(const SkySettings* sky);

// The frame's lighting environment, and the only one a pass should use.
//
// The sun direction and colour come from the scene's first directional light
// when there is one, and from the sky otherwise; the ambient always comes from
// the sky. That order matters: the shadow cascades are built from that same
// directional light (see ShadowPass), so taking the shading direction from
// anywhere else lets the two disagree - and shading a surface against one
// direction while testing its shadow against another is acne everywhere, on
// every surface at once.
EnvironmentBlock environmentForFrame(const FrameContext& frame);

} // namespace Radion

#endif // RADION_ENVIRONMENT_BLOCK_H
