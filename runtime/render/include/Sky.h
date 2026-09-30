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
    Math::vec3 sunDirection = Math::vec3(0.0f, 1.0f, 0.0f);
    Math::vec3 sunTransmittance = Math::vec3(1.0f);
    Math::vec3 ambient = Math::vec3(0.12f, 0.16f, 0.24f);
    f32 ambientStrength = 1.0f;
    f32 intensity = 1.0f;

    // Runtime controls for a baked (HAS_LIGHTMAP) surface, applied in lit.frag on top of the lightmap sample; defaults leave it untouched.
    f32 lightmapIntensity = 1.0f;
    f32 lightmapShadowLift = 0.0f;
    f32 sunIntensity = 22.0f;
    f32 rayleigh = 1.0f;
    f32 mie = 1.0f;
    f32 mieG = 0.76f;
    f32 atmosphereExposure = 1.0f;
    u32 viewSteps = 16;
    u32 lightSteps = 8;

    // Non-owning: AssetManager owns the texture. The name is serialised; Engine::setSkyCubemap() re-resolves the handle on load.
    TextureHandle cubemap;
    std::string cubemapName;

    // Where the sky panel looks for cubemaps; a setting because the engine does not know a project's asset layout.
    std::string cubemapDirectory = "skys";

    // Cloud layer composited over the active background mode, not a mode itself.
    bool cloudsEnabled = false;
    f32 cloudHeight = 2000.0f;
    f32 cloudScale = 0.0008f;
    f32 cloudCoverage = 0.5f;
    f32 cloudDensity = 0.5f;
    f32 cloudSpeed = 1.0f;
    Math::vec2 cloudDirection = Math::vec2(1.0f, 0.0f);
    Math::vec3 cloudColor = Math::vec3(1.0f, 1.0f, 1.0f);

    void updateSun();
};

// Resolves the six faces through AssetManager and points `sky` at them, leaving `sky.mode` alone. False (sky untouched) when faces are missing or fail to load.
bool loadSkyCubemap(SkySettings& sky, const std::string& baseName);

RenderTechnique* createSkyPass();

} // namespace Radion

#endif // RADION_SKY_H
