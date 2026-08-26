#ifndef RADION_SKY_H
#define RADION_SKY_H

#include "RenderTechnique.h"

#include <string>

namespace Radion
{

enum class SkyMode : u8
{
    Gradient,
    Atmosphere,
    Cubemap
};

struct SkySettings
{
    bool enabled = true;
    SkyMode mode = SkyMode::Gradient;
    bool automaticSun = true;
    bool sunFromSky = true;
    f32 timeOfDay = 12.0f;
    f32 sunAzimuth = 180.0f;
    f32 sunElevation = 65.0f;
    f32 northOffset = 0.0f;
    f32 maximumElevation = 65.0f;
    Math::Vec3 sunDirection = Math::Vec3(0.0f, 1.0f, 0.0f);
    Math::Vec3 sunTransmittance = Math::Vec3(1.0f);
    Math::Vec3 ambient = Math::Vec3(0.12f, 0.16f, 0.24f);
    f32 ambientStrength = 1.0f;
    f32 intensity = 1.0f;

    // Runtime controls for a baked (HAS_LIGHTMAP) surface, applied in
    // lit.frag on top of the lightmap sample. The bake already cooks in its
    // own ambient, so these are the only knobs a lightmapped surface has at
    // runtime - the defaults leave the sampled lightmap untouched.
    f32 lightmapIntensity = 1.0f;
    f32 lightmapShadowLift = 0.0f;
    f32 sunIntensity = 22.0f;
    f32 rayleigh = 1.0f;
    f32 mie = 1.0f;
    f32 mieG = 0.76f;
    f32 atmosphereExposure = 1.0f;
    u32 viewSteps = 16;
    u32 lightSteps = 8;

    // Non-owning: AssetManager owns the texture, this only points at it. The
    // name is what gets serialised - Engine::setSkyCubemap() re-resolves the
    // handle from it on load.
    TextureHandle cubemap;
    std::string cubemapName;

    // Where the sky panel looks for selectable cubemaps. A setting rather
    // than a constant because the engine has no business knowing how a
    // particular project lays its assets out.
    std::string cubemapDirectory = "skys";

    // Cloud layer, composited on top of whichever background mode is active
    // above (gradient, atmosphere or cubemap), not a mode of its own.
    bool cloudsEnabled = false;
    f32 cloudHeight = 2000.0f;
    f32 cloudScale = 0.0008f;
    f32 cloudCoverage = 0.5f;
    f32 cloudDensity = 0.5f;
    f32 cloudSpeed = 1.0f;
    Math::Vec2 cloudDirection = Math::Vec2(1.0f, 0.0f);
    Math::Vec3 cloudColor = Math::Vec3(1.0f, 1.0f, 1.0f);

    void updateSun();
};

// Resolves the six faces through AssetManager and points `sky` at the
// result, leaving `sky.mode` alone: picking which cubemap and deciding to
// display it are separate choices, and restoring settings needs the first
// without the second. False (and `sky` untouched) when the faces are
// missing or fail to load.
bool loadSkyCubemap(SkySettings& sky, const std::string& baseName);

RenderTechnique* createSkyPass();

} // namespace Radion

#endif // RADION_SKY_H
