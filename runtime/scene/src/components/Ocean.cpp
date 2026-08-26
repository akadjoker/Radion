#include "PCH.h"

#include "Ocean.h"

#include "AssetManager.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cmath>

namespace Radion
{

namespace
{
constexpr f32 kGravity = 9.81f;

// glm::normalize() on a zero (or NaN/infinite) vector is undefined - not the
// harmless zero a caller might assume. A single bad wave direction propagates
// through heightAt()/normalAt() into position, normal, clip-space and depth
// for the whole surface. Falls back to (1,0) rather than the caller's
// previous direction: this is a pure function, with nothing to remember.
Math::Vec2 safeDirection(const Math::Vec2& direction)
{
    const f32 lengthSq = glm::dot(direction, direction);
    if (!std::isfinite(lengthSq) || lengthSq < 1e-8f)
        return Math::Vec2(1.0f, 0.0f);
    return direction * (1.0f / std::sqrt(lengthSq));
}

// Must match kPhaseOffset in ocean.vert exactly - see the comment there.
const Math::Vec2 kPhaseOffset[kOceanMaxWaves] = {
    {0.0f, 0.0f}, {311.7f, 172.3f}, {-198.4f, 402.1f},
    {87.6f, -266.9f}, {-355.2f, -114.8f}, {224.1f, 333.6f},
};
}

Ocean::Ocean() : Component(Type)
{
    // Wavelengths coprime on purpose: multiples of one another realign
    // periodically and the repeat becomes visible on the surface.
    mWaves[0] = {Math::Vec2(1.00f, 0.15f), 61.0f, 1.35f};
    mWaves[1] = {Math::Vec2(0.70f, -0.70f), 37.0f, 0.75f};
    mWaves[2] = {Math::Vec2(-0.35f, 0.94f), 23.0f, 0.40f};
    mWaves[3] = {Math::Vec2(0.90f, 0.44f), 13.0f, 0.18f};
    mWaves[4] = {Math::Vec2(-0.80f, -0.60f), 7.0f, 0.09f};
    mWaves[5] = {Math::Vec2(0.20f, 0.98f), 4.0f, 0.05f};
}

void Ocean::onDestroy()
{
    // The component owns the mesh build() created - invalidating the handle
    // without destroying it left the buffers, materials and pool entry alive
    // in AssetManager until engine shutdown, one leak per Ocean removed.
    if (mMesh.valid())
        Assets().destroyMesh(mMesh);
    mMesh = MeshHandle();
}

bool Ocean::build(f32 size, u32 segments)
{
    if (size <= 0.0f || segments == 0)
        return false;

    // A single flat grid, no LOD: the vertex shader displaces every vertex
    // by the full Gerstner sum regardless of distance, so segments is a
    // direct multiplier on vertex shader cost. (segments+1)^2 vertices,
    // 2*segments^2 triangles.
    //
    // Built into a temporary first: mMesh must keep pointing at a valid mesh
    // (or stay MeshHandle()) for as long as this component might still be
    // drawn, and only the old one - never the new, still-being-built one -
    // is safe to drop on a rebuild.
    const MeshHandle mesh = Assets().createPlane(size, size, segments, segments, 1.0f);
    if (!mesh.valid())
        return false;

    if (mMesh.valid())
        Assets().destroyMesh(mMesh);
    mMesh = mesh;
    mSpacing = size / static_cast<f32>(segments);
    mSize = size;
    mSegments = segments;
    return true;
}

f32 Ocean::size() const { return mSize; }
u32 Ocean::segments() const { return mSegments; }

void Ocean::setQuality(OceanQuality quality)
{
    mQuality = quality;
}

OceanQuality Ocean::quality() const
{
    return mQuality;
}

void Ocean::setWave(u32 index, const Math::Vec2& direction, f32 wavelength, f32 amplitude)
{
    if (index >= kOceanMaxWaves)
        return;
    mWaves[index].direction = safeDirection(direction);
    mWaves[index].wavelength = glm::max(0.001f, wavelength);
    mWaves[index].amplitude = std::isfinite(amplitude) ? glm::max(0.0f, amplitude) : 0.0f;
}

const OceanWave& Ocean::wave(u32 index) const
{
    static const OceanWave fallback{};
    return index < kOceanMaxWaves ? mWaves[index] : fallback;
}

void Ocean::setWaveCount(u32 count)
{
    mWaveCount = glm::min(count, kOceanMaxWaves);
}

u32 Ocean::waveCount() const
{
    return mWaveCount;
}

void Ocean::setWaveScale(f32 scale)
{
    mWaveScale = glm::max(0.0f, scale);
}

f32 Ocean::waveScale() const
{
    return mWaveScale;
}

void Ocean::setSteepness(f32 steepness)
{
    mSteepness = glm::clamp(steepness, 0.0f, 1.0f);
}

f32 Ocean::steepness() const
{
    return mSteepness;
}

void Ocean::setTimeScale(f32 scale)
{
    mTimeScale = scale;
}

f32 Ocean::timeScale() const
{
    return mTimeScale;
}

void Ocean::setLevel(f32 level)
{
    mLevel = level;
}

f32 Ocean::level() const
{
    return mLevel;
}

void Ocean::setShallowColor(const Math::Vec3& color)
{
    mShallowColor = color;
}

const Math::Vec3& Ocean::shallowColor() const { return mShallowColor; }

void Ocean::setDeepColor(const Math::Vec3& color)
{
    mDeepColor = color;
}

const Math::Vec3& Ocean::deepColor() const { return mDeepColor; }

void Ocean::setAbsorptionDistance(f32 distance)
{
    mAbsorptionDistance = glm::max(0.001f, distance);
}

f32 Ocean::absorptionDistance() const { return mAbsorptionDistance; }

void Ocean::setRoughness(f32 roughness)
{
    mRoughness = glm::clamp(roughness, 0.001f, 1.0f);
}

f32 Ocean::roughness() const { return mRoughness; }

void Ocean::setSpecularStrength(f32 strength)
{
    mSpecularStrength = glm::max(0.0f, strength);
}

f32 Ocean::specularStrength() const { return mSpecularStrength; }

void Ocean::setNormalMap(TextureHandle texture)
{
    mNormalMap = texture;
}

void Ocean::setNormalMapFile(const std::string& file)
{
    mNormalMapFile = file;
    mNormalMap = file.empty() ? TextureHandle() : Assets().loadTexture(file, ColorSpace::Linear);
}

const std::string& Ocean::normalMapFile() const { return mNormalMapFile; }

void Ocean::setNormalMapEnabled(bool enabled)
{
    mHasNormalMap = enabled;
}

bool Ocean::normalMapEnabled() const { return mHasNormalMap; }

void Ocean::setNormalOctaves(u32 octaves)
{
    mNormalOctaves = glm::clamp(octaves, 1u, 6u);
}

u32 Ocean::normalOctaves() const { return mNormalOctaves; }

void Ocean::setNormalScale(f32 scale1, f32 scale2)
{
    mNormalScale1 = scale1;
    mNormalScale2 = scale2;
}

f32 Ocean::normalScale1() const { return mNormalScale1; }
f32 Ocean::normalScale2() const { return mNormalScale2; }

void Ocean::setNormalStrength(f32 strength)
{
    mNormalStrength = strength;
}

f32 Ocean::normalStrength() const { return mNormalStrength; }

void Ocean::setNormalSpeed(f32 speed1, f32 speed2)
{
    mNormalSpeed1 = speed1;
    mNormalSpeed2 = speed2;
}

f32 Ocean::normalSpeed1() const { return mNormalSpeed1; }
f32 Ocean::normalSpeed2() const { return mNormalSpeed2; }

void Ocean::setFoamTexture(TextureHandle texture)
{
    mFoamMap = texture;
}

void Ocean::setFoamTextureFile(const std::string& file)
{
    mFoamMapFile = file;
    mFoamMap = file.empty() ? TextureHandle() : Assets().loadTexture(file, ColorSpace::Linear);
}

const std::string& Ocean::foamTextureFile() const { return mFoamMapFile; }

void Ocean::setFoamEnabled(bool enabled)
{
    mHasFoam = enabled;
}

bool Ocean::foamEnabled() const { return mHasFoam; }

void Ocean::setFoamScale(f32 scale)
{
    mFoamScale = scale;
}

f32 Ocean::foamScale() const { return mFoamScale; }

void Ocean::setFoamStrength(f32 strength)
{
    mFoamStrength = strength;
}

f32 Ocean::foamStrength() const { return mFoamStrength; }

void Ocean::setFoamDepth(f32 depth)
{
    mFoamDepth = glm::max(0.001f, depth);
}

f32 Ocean::foamDepth() const { return mFoamDepth; }

void Ocean::setFoamCrest(f32 crest)
{
    mFoamCrest = crest;
}

f32 Ocean::foamCrest() const { return mFoamCrest; }

void Ocean::setFresnelDetail(f32 amount)
{
    mFresnelDetail = glm::clamp(amount, 0.0f, 1.0f);
}

f32 Ocean::fresnelDetail() const { return mFresnelDetail; }

void Ocean::setFresnelMax(f32 amount)
{
    mFresnelMax = glm::clamp(amount, 0.0f, 1.0f);
}

f32 Ocean::fresnelMax() const { return mFresnelMax; }

void Ocean::setFresnelBias(f32 amount)
{
    mFresnelBias = glm::clamp(amount, 0.0f, 1.0f);
}

f32 Ocean::fresnelBias() const { return mFresnelBias; }

void Ocean::setFresnelScale(f32 amount)
{
    mFresnelScale = glm::max(amount, 0.0f);
}

f32 Ocean::fresnelScale() const { return mFresnelScale; }

void Ocean::setFresnelPower(f32 amount)
{
    mFresnelPower = glm::max(amount, 0.1f);
}

f32 Ocean::fresnelPower() const { return mFresnelPower; }

void Ocean::setMinOpacity(f32 amount)
{
    mMinOpacity = glm::clamp(amount, 0.0f, 1.0f);
}

f32 Ocean::minOpacity() const { return mMinOpacity; }

void Ocean::setReflectionDistortion(f32 amount)
{
    mReflectionDistortion = amount;
}

f32 Ocean::reflectionDistortion() const { return mReflectionDistortion; }

void Ocean::setReflectionStrength(f32 amount)
{
    mReflectionStrength = glm::clamp(amount, 0.0f, 2.0f);
}

f32 Ocean::reflectionStrength() const { return mReflectionStrength; }

void Ocean::setRefractionStrength(f32 amount)
{
    mRefractionStrength = glm::clamp(amount, 0.0f, 1.0f);
}

f32 Ocean::refractionStrength() const { return mRefractionStrength; }

void Ocean::setColorStrength(f32 amount)
{
    mColorStrength = glm::clamp(amount, 0.0f, 2.0f);
}

f32 Ocean::colorStrength() const { return mColorStrength; }

void Ocean::setUnderwaterColor(const Math::Vec3& color)
{
    mUnderwaterColor = color;
}

const Math::Vec3& Ocean::underwaterColor() const { return mUnderwaterColor; }

void Ocean::setDebugMode(s32 mode)
{
    mDebugMode = mode;
}

s32 Ocean::debugMode() const { return mDebugMode; }

// Same sum ocean.vert runs, in world x/z - see the header comment on why
// `time` is a parameter rather than a clock this object keeps itself.
f32 Ocean::heightAt(f32 x, f32 z, f32 time) const
{
    const f32 t = time * mTimeScale;
    f32 y = mLevel;

    for (u32 i = 0; i < mWaveCount && i < kOceanMaxWaves; ++i)
    {
        const OceanWave& wave = mWaves[i];
        const Math::Vec2 D = safeDirection(wave.direction);
        const f32 wavelength = glm::max(0.001f, wave.wavelength);
        const f32 A = wave.amplitude;

        const f32 k = 6.28318530718f / wavelength;
        const f32 w = std::sqrt(kGravity * k);
        const Math::Vec2& offset = kPhaseOffset[i];
        const f32 phase = k * (D.x * (x + offset.x) + D.y * (z + offset.y)) - w * t;
        y += A * mWaveScale * std::sin(phase);
    }
    return y;
}

// Same tangent/binormal accumulation as the vertex shader, evaluated at one
// point instead of a whole grid.
Math::Vec3 Ocean::normalAt(f32 x, f32 z, f32 time) const
{
    const f32 t = time * mTimeScale;
    Math::Vec3 tangent(1.0f, 0.0f, 0.0f);
    Math::Vec3 binormal(0.0f, 0.0f, 1.0f);

    for (u32 i = 0; i < mWaveCount && i < kOceanMaxWaves; ++i)
    {
        const OceanWave& wave = mWaves[i];
        const Math::Vec2 D = safeDirection(wave.direction);
        const f32 wavelength = glm::max(0.001f, wave.wavelength);
        const f32 A = wave.amplitude;

        const f32 k = 6.28318530718f / wavelength;
        const f32 w = std::sqrt(kGravity * k);
        const f32 Q = mSteepness / (k * A * static_cast<f32>(mWaveCount) + 1e-5f);

        const Math::Vec2& offset = kPhaseOffset[i];
        const f32 phase = k * (D.x * (x + offset.x) + D.y * (z + offset.y)) - w * t;
        const f32 s = std::sin(phase);
        const f32 c = std::cos(phase);
        const f32 QA = Q * A;

        tangent += Math::Vec3(-D.x * D.x * QA * k * s, D.x * A * k * c, -D.x * D.y * QA * k * s);
        binormal += Math::Vec3(-D.x * D.y * QA * k * s, D.y * A * k * c, -D.y * D.y * QA * k * s);
    }
    return glm::normalize(glm::cross(binormal, tangent));
}

void Ocean::submit(const Math::Mat4& transform)
{
    if (!mMesh.valid() || mWaveCount == 0)
        return;

    OceanDrawCommand command;
    command.mesh = mMesh;
    // The level rides in the transform rather than in the shader: it is the
    // one place both sides can agree on it. heightAt() adds it on the CPU, the
    // vertex shader gets it through uModel, and the reflection reads the plane
    // straight off model[3].y - three readers, one number.
    command.model = glm::translate(transform, Math::Vec3(0.0f, mLevel, 0.0f));
    command.quality = mQuality;

    // Only the waves the grid can actually carry. The displacement happens per
    // vertex, so a wave shorter than two quads has no samples to be drawn with
    // and turns into a lattice beating against the mesh - the fine detail it
    // was meant to add is the normal map's job, not the geometry's.
    const f32 shortest = 2.0f * mSpacing;
    u32 count = 0;
    for (u32 i = 0; i < mWaveCount && i < kOceanMaxWaves; ++i)
    {
        if (mWaves[i].wavelength < shortest)
            continue;
        command.waves[count] = mWaves[i];
        command.waves[count].amplitude *= mWaveScale;
        ++count;
    }
    command.waveCount = count;
    if (count == 0)
        return;
    // The scale has to reach BOTH axes or it is not a trochoid any more. Q is
    // derived as steepness/(k*A*n), so scaling the amplitude alone leaves the
    // horizontal push Q*A = steepness/(k*n) untouched: the crests stop
    // pinching and the sum degenerates into plain sines - round crests, round
    // troughs, the jelly Gerstner exists to avoid. Scaling the steepness by
    // the same factor cancels out of Q and carries through to Q*A, so the
    // particle keeps tracing a circle and only its radius changes.
    //
    // Clamped at 1, and that number is not arbitrary: each wave contributes
    // Q*k*A to the sum and Q is derived as steepness/(k*A*n), so every term is
    // steepness/n and the n of them add back up to steepness exactly. At 1 the
    // crest is a cusp - the sharpest a real wave gets before it breaks. Past
    // it the trochoid folds through itself and the surface turns into spikes,
    // which is what scaling straight through this clamp produced.
    //
    // So the scale keeps raising the height above that point, but the crests
    // stop sharpening: taller waves at the same wavelength is exactly the
    // thing real water answers by breaking.
    command.steepness = glm::min(mSteepness * mWaveScale, 1.0f);
    command.timeScale = mTimeScale;

    command.shallowColor = mShallowColor;
    command.deepColor = mDeepColor;
    command.absorptionDistance = mAbsorptionDistance;
    command.roughness = mRoughness;
    command.specularStrength = mSpecularStrength;

    command.hasNormalMap = mHasNormalMap;
    command.normalMap = mNormalMap;
    command.normalOctaves = mNormalOctaves;
    command.normalScale1 = mNormalScale1;
    command.normalScale2 = mNormalScale2;
    command.normalStrength = mNormalStrength;
    command.normalSpeed1 = mNormalSpeed1;
    command.normalSpeed2 = mNormalSpeed2;

    command.hasFoam = mHasFoam;
    command.foamMap = mFoamMap;
    command.foamScale = mFoamScale;
    command.foamStrength = mFoamStrength;
    command.foamDepth = mFoamDepth;
    command.foamCrest = mFoamCrest;

    command.fresnelDetail = mFresnelDetail;
    command.fresnelMax = mFresnelMax;
    command.fresnelBias = mFresnelBias;
    command.fresnelScale = mFresnelScale;
    command.fresnelPower = mFresnelPower;
    command.minOpacity = mMinOpacity;
    command.reflectionDistortion = mReflectionDistortion;
    command.reflectionStrength = mReflectionStrength;
    command.refractionStrength = mRefractionStrength;
    command.colorStrength = mColorStrength;
    command.underwaterColor = mUnderwaterColor;
    command.debugMode = mDebugMode;

    OceanDraws().submit(command);
}

} // namespace Radion
