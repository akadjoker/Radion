#ifndef RADION_OCEAN_H
#define RADION_OCEAN_H

#include "Component.h"
#include "OceanRender.h"

#include <string>

namespace Radion
{

// A Gerstner-wave sea surface: a tessellated grid plus the CPU side of the
// same wave sum the vertex shader runs, so gameplay (buoyancy, a boat's hull,
// a splash at the shoreline) can ask where the surface sits without reading
// the GPU back.
//
// The component owns the wave set and the grid; drawing belongs to the ocean
// pass, the same split Grass/GrassRender use. Nothing here talks to the GPU
// beyond building the grid mesh once.
class Ocean final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Ocean;

    // Builds the tessellated grid the surface displaces, centred on the
    // owner's origin. `segments` is per side, so the vertex count is
    // (segments+1)^2 - keep this in mind before doubling it, a plain forward
    // draw with no LOD pays for every one of them every frame.
    bool build(f32 size, u32 segments);
    f32 size() const;
    u32 segments() const;

    void setQuality(OceanQuality quality);
    OceanQuality quality() const;

    // Up to kOceanMaxWaves; index beyond that is ignored. Direction is
    // normalised on read, not on write, so a panel can drag it through zero
    // without the call failing.
    void setWave(u32 index, const Math::Vec2& direction, f32 wavelength, f32 amplitude);
    const OceanWave& wave(u32 index) const;
    void setWaveCount(u32 count);
    u32 waveCount() const;

    // One multiplier over every wave's amplitude. The wave set is authored at
    // some scale - a 1200-unit lagoon, say - and reusing it on a 3200-unit
    // ocean otherwise means retuning six amplitudes by hand. It scales the
    // horizontal displacement along with the height - a Gerstner particle
    // traces a circle, and scaling one axis without the other flattens it into
    // a sine wave.
    void setWaveScale(f32 scale);
    f32 waveScale() const;

    void setSteepness(f32 steepness);
    f32 steepness() const;
    void setTimeScale(f32 scale);
    f32 timeScale() const;

    void setLevel(f32 level);
    f32 level() const;

    void setShallowColor(const Math::Vec3& color);
    const Math::Vec3& shallowColor() const;
    void setDeepColor(const Math::Vec3& color);
    const Math::Vec3& deepColor() const;
    void setAbsorptionDistance(f32 distance);
    f32 absorptionDistance() const;
    void setRoughness(f32 roughness);
    f32 roughness() const;
    void setSpecularStrength(f32 strength);
    f32 specularStrength() const;

    void setNormalMap(TextureHandle texture);
    void setNormalMapFile(const std::string& file);
    const std::string& normalMapFile() const;
    void setNormalMapEnabled(bool enabled);
    bool normalMapEnabled() const;
    void setNormalOctaves(u32 octaves);
    u32 normalOctaves() const;
    void setNormalScale(f32 scale1, f32 scale2);
    f32 normalScale1() const;
    f32 normalScale2() const;
    void setNormalStrength(f32 strength);
    f32 normalStrength() const;
    void setNormalSpeed(f32 speed1, f32 speed2);
    f32 normalSpeed1() const;
    f32 normalSpeed2() const;

    void setFoamTexture(TextureHandle texture);
    void setFoamTextureFile(const std::string& file);
    const std::string& foamTextureFile() const;
    void setFoamEnabled(bool enabled);
    bool foamEnabled() const;
    void setFoamScale(f32 scale);
    f32 foamScale() const;
    void setFoamStrength(f32 strength);
    f32 foamStrength() const;
    void setFoamDepth(f32 depth);
    f32 foamDepth() const;
    void setFoamCrest(f32 crest);
    f32 foamCrest() const;

    void setFresnelDetail(f32 amount);
    f32 fresnelDetail() const;
    void setFresnelMax(f32 amount);
    f32 fresnelMax() const;
    void setFresnelBias(f32 amount);
    f32 fresnelBias() const;
    void setFresnelScale(f32 amount);
    f32 fresnelScale() const;
    void setFresnelPower(f32 amount);
    f32 fresnelPower() const;
    void setMinOpacity(f32 amount);
    f32 minOpacity() const;
    void setReflectionDistortion(f32 amount);
    f32 reflectionDistortion() const;
    void setReflectionStrength(f32 amount);
    f32 reflectionStrength() const;
    void setRefractionStrength(f32 amount);
    f32 refractionStrength() const;
    void setColorStrength(f32 amount);
    f32 colorStrength() const;
    void setUnderwaterColor(const Math::Vec3& color);
    const Math::Vec3& underwaterColor() const;
    void setDebugMode(s32 mode);
    s32 debugMode() const;

    // Same Gerstner sum the vertex shader runs, evaluated on the CPU. Takes
    // `time` explicitly rather than an internal clock: the caller passes the
    // exact value it hands the frame (FrameContext::time), which is the only
    // way the two are guaranteed to agree - a clock ticking independently on
    // the CPU would drift a frame or a stutter away from what the GPU drew.
    f32 heightAt(f32 x, f32 z, f32 time) const;
    Math::Vec3 normalAt(f32 x, f32 z, f32 time) const;

private:
    friend class GameObject;
    friend class Scene;

    Ocean();
    void onDestroy() override;

    void submit(const Math::Mat4& transform);

    MeshHandle mMesh;
    // World units between two vertices, from build(). The Gerstner sum runs
    // per vertex, so this is the sampling rate the wave set has to fit in.
    f32 mSpacing = 1.0f;
    f32 mSize = 0.0f;
    u32 mSegments = 0;
    OceanWave mWaves[kOceanMaxWaves];
    u32 mWaveCount = 4;
    f32 mWaveScale = 1.0f;
    f32 mSteepness = 0.55f;
    f32 mTimeScale = 1.0f;
    f32 mLevel = 0.0f;

    OceanQuality mQuality = OceanQuality::Reflection;

    Math::Vec3 mShallowColor = Math::Vec3(0.28f, 0.55f, 0.55f);
    Math::Vec3 mDeepColor = Math::Vec3(0.02f, 0.11f, 0.20f);
    f32 mAbsorptionDistance = 28.0f;
    f32 mRoughness = 0.06f;
    f32 mSpecularStrength = 0.6f;

    TextureHandle mNormalMap;
    std::string mNormalMapFile;
    bool mHasNormalMap = true;
    u32 mNormalOctaves = 4;
    f32 mNormalScale1 = 0.038f;
    f32 mNormalScale2 = 0.0067f;
    f32 mNormalStrength = 0.1f;
    f32 mNormalSpeed1 = 0.35f;
    f32 mNormalSpeed2 = 0.55f;

    TextureHandle mFoamMap;
    std::string mFoamMapFile;
    bool mHasFoam = true;
    f32 mFoamScale = 0.035f;
    f32 mFoamStrength = 1.0f;
    f32 mFoamDepth = 6.0f;
    f32 mFoamCrest = 0.0f;

    f32 mFresnelDetail = 0.25f;
    f32 mFresnelMax = 1.0f;
    f32 mFresnelBias = 0.10f;
    f32 mFresnelScale = 0.90f;
    f32 mFresnelPower = 4.0f;
    f32 mMinOpacity = 0.45f;
    f32 mReflectionDistortion = 0.035f;
    f32 mReflectionStrength = 1.0f;
    f32 mRefractionStrength = 1.0f;
    f32 mColorStrength = 1.0f;
    Math::Vec3 mUnderwaterColor = Math::Vec3(0.06f, 0.22f, 0.30f);
    s32 mDebugMode = 0;
};

} // namespace Radion

#endif // RADION_OCEAN_H
