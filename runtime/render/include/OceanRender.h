#ifndef RADION_OCEAN_RENDER_H
#define RADION_OCEAN_RENDER_H

#include "GPU.h"
#include "RenderTechnique.h"

#include "Math.h"

#include <vector>

namespace Radion
{

// As many Gerstner waves as the shader sums; the CPU side agrees on this
// number so heightAt() never reads past what the GPU was given.
constexpr u32 kOceanMaxWaves = 6;

// Published only for the OceanDemo/debug UI after the refraction scene copy.
constexpr const char* kOceanSceneColorDebugTargetName = "ocean.debug.scene_color";
constexpr const char* kOceanSceneDepthDebugTargetName = "ocean.debug.scene_depth";

// One Gerstner wave: xy = propagation direction (normalised), z = wavelength,
// w = amplitude. Matches uWaves[i] in ocean.vert/ocean.frag exactly.
struct OceanWave
{
    Math::vec2 direction = Math::vec2(1.0f, 0.0f);
    f32 wavelength = 20.0f;
    f32 amplitude = 0.35f;
};

// What the surface samples, from cheapest to most expensive. Waves, foam and
// normal-map detail stay in every tier - only the extra render targets a
// higher tier needs to look correct come and go.
enum class OceanQuality : u8
{
    SkyOnly,             // sky/environment reflection only, no render target
    Reflection,          // + planar reflection of the scene
    ReflectionRefraction // + depth-based shallow/deep absorption and shoreline foam
};

// What an Ocean component hands the pass for the frame. The grid mesh is
// built once by the component; this only carries what changes.
struct OceanDrawCommand
{
    MeshHandle mesh;
    Math::mat4 model = Math::mat4(1.0f);

    OceanQuality quality = OceanQuality::Reflection;

    OceanWave waves[kOceanMaxWaves];
    u32 waveCount = 0;
    f32 steepness = 0.55f;
    f32 timeScale = 1.0f;

    Math::vec3 shallowColor = Math::vec3(0.28f, 0.55f, 0.55f);
    Math::vec3 deepColor = Math::vec3(0.02f, 0.11f, 0.20f);
    f32 absorptionDistance = 28.0f;
    f32 roughness = 0.06f;
    f32 specularStrength = 0.6f;

    bool hasNormalMap = true;
    TextureHandle normalMap;
    u32 normalOctaves = 4;
    f32 normalScale1 = 0.038f;
    f32 normalScale2 = 0.0067f;
    f32 normalStrength = 0.1f;
    f32 normalSpeed1 = 0.35f;
    f32 normalSpeed2 = 0.55f;

    bool hasFoam = true;
    TextureHandle foamMap;
    f32 foamScale = 0.035f;
    f32 foamStrength = 1.0f;
    f32 foamDepth = 6.0f;
    f32 foamCrest = 0.0f;

    f32 fresnelDetail = 0.25f;
    f32 fresnelMax = 1.0f;
    // fresnel = bias + scale * pow(1 - dot(N, V), power) - same shape as the
    // fresnel demo's water material, tunable the same way.
    f32 fresnelBias = 0.10f;
    f32 fresnelScale = 0.90f;
    f32 fresnelPower = 4.0f;
    f32 minOpacity = 0.45f;
    f32 reflectionDistortion = 0.035f;
    f32 reflectionStrength = 1.0f;
    f32 refractionStrength = 1.0f;
    f32 colorStrength = 1.0f;

    Math::vec3 underwaterColor = Math::vec3(0.06f, 0.22f, 0.30f);

    s32 debugMode = 0;

    // Defaults match Grass/ForwardPass: a fake sun until the sky/scene light
    // takes over.
    Math::vec3 lightDirection = -Math::normalize(Math::vec3(0.4f, 0.8f, 0.3f));
    Math::vec3 lightColor = Math::vec3(1.0f);
    Math::vec3 ambient = Math::vec3(0.2f);
    f32 skyIntensity = 1.0f;
};

class OceanRenderQueue
{
public:
    static OceanRenderQueue& getSingleton();

    void clear();
    void submit(const OceanDrawCommand& command);
    const std::vector<OceanDrawCommand>& commands() const;

private:
    std::vector<OceanDrawCommand> mCommands;
};

OceanRenderQueue& OceanDraws();

// Compiles one pipeline variant per OceanQuality tier (SkyOnly/Reflection/
// ReflectionRefraction), the same #define-driven variant scheme
// MaterialManager uses for materials.
RenderTechnique* createOceanPass();

} // namespace Radion

#endif // RADION_OCEAN_RENDER_H
