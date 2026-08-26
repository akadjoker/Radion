#ifndef RADION_PARTICLE_EMITTER_H
#define RADION_PARTICLE_EMITTER_H

#include "Color.h"
#include "Component.h"
#include "GPU.h"
#include "TrailRender.h" // BillboardMode

#include "Math.h"
#include <string>
#include <random>
#include <vector>

namespace Radion
{

struct Particle
{
    Math::vec3 position = Math::vec3(0.0f);
    Math::vec3 velocity = Math::vec3(0.0f);
    Math::vec3 acceleration = Math::vec3(0.0f);
    Color color;
    Color colorStart;
    Color colorEnd;
    Math::vec2 size = Math::vec2(1.0f);
    Math::vec2 sizeStart = Math::vec2(1.0f);
    Math::vec2 sizeEnd = Math::vec2(1.0f);
    f32 rotation = 0.0f;
    f32 rotationSpeed = 0.0f;
    Math::vec4 texRect = Math::vec4(0.0f, 0.0f, 1.0f, 1.0f);
    f32 lifetime = 1.0f;
    f32 timeAlive = 0.0f;
    bool active = false;
};

// Forces applied to every live particle each frame, on top of the emitter's
// own base gravity/drag. A list of small strategy objects the emitter owns,
// each free to touch acceleration or velocity directly - add as many as an
// effect needs, in any order.
class ParticleAffector
{
public:
    bool enabled = true;
    virtual ~ParticleAffector() = default;
    virtual void apply(Particle& particle, f32 deltaTime) = 0;
};

class GravityAffector final : public ParticleAffector
{
public:
    explicit GravityAffector(const Math::vec3& gravity) : gravity(gravity)
    {
    }
    void apply(Particle& particle, f32 deltaTime) override;
    Math::vec3 gravity;
};

class DragAffector final : public ParticleAffector
{
public:
    explicit DragAffector(f32 drag) : drag(drag)
    {
    }
    void apply(Particle& particle, f32 deltaTime) override;
    f32 drag;
};

// Rotational pull around `center`, in the XZ plane - a smoke ring or a
// cyclone effect. Falls off linearly to zero at `radius`.
class VortexAffector final : public ParticleAffector
{
public:
    VortexAffector(const Math::vec3& center, f32 strength, f32 radius)
        : center(center), strength(strength), radius(radius)
    {
    }
    void apply(Particle& particle, f32 deltaTime) override;
    Math::vec3 center;
    f32 strength;
    f32 radius;
};

// Pulls (or, with repulse, pushes) particles toward `position`, falling off
// linearly to zero at `radius` - a black hole / explosion shockwave.
class AttractorAffector final : public ParticleAffector
{
public:
    AttractorAffector(const Math::vec3& position, f32 strength, f32 radius, bool repulse = false)
        : position(position), strength(strength), radius(radius), repulse(repulse)
    {
    }
    void apply(Particle& particle, f32 deltaTime) override;
    Math::vec3 position;
    f32 strength;
    f32 radius;
    bool repulse;
};

// A per-particle sine-noise wobble, sampled from each particle's own
// position so nearby particles drift in a correlated way instead of
// independently jittering.
class TurbulenceAffector final : public ParticleAffector
{
public:
    TurbulenceAffector(f32 strength, f32 frequency) : strength(strength), frequency(frequency)
    {
    }
    void apply(Particle& particle, f32 deltaTime) override;
    f32 strength;
    f32 frequency;
    f32 time = 0.0f;
};

// Overrides the emitter's own colorStart/colorEnd fade for particles this is
// attached to - lets one emitter mix particles that fade differently.
class ColorOverLifetimeAffector final : public ParticleAffector
{
public:
    ColorOverLifetimeAffector(Color start, Color end) : startColor(start), endColor(end)
    {
    }
    void apply(Particle& particle, f32 deltaTime) override;
    Color startColor;
    Color endColor;
};

// Same idea as ColorOverLifetimeAffector, for size.
class SizeOverLifetimeAffector final : public ParticleAffector
{
public:
    SizeOverLifetimeAffector(const Math::vec2& start, const Math::vec2& end)
        : startSize(start), endSize(end)
    {
    }
    void apply(Particle& particle, f32 deltaTime) override;
    Math::vec2 startSize;
    Math::vec2 endSize;
};

enum class ParticleEmitterShape : u8
{
    Point,
    Sphere,
    Box,
    Cone,
    Circle,
    Ring
};

enum class ParticleEmissionMode : u8
{
    Continuous,
    Burst,
    OneShot,
    Pulse
};

// CPU-simulated particle emitter: a fixed pool (the oldest particle is
// recycled once it fills up, never just refusing new ones), 6 spawn shapes,
// 4 emission modes, and a list of composable affectors on top of the
// emitter's own gravity/drag. Renders through TrailDraws(), the same
// world-space billboard queue Billboard/Text3D use.
//
// ParticleEffect (one compute-shader pool shared by the whole scene) stays
// the right tool for thousand-particle volumes like smoke or fire; this one
// is for effects where each particle's own behavior matters and the count
// is in the tens or hundreds - bullet impacts, debris, magic effects.
class ParticleEmitter final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::ParticleEmitter;

    // Resizing clears every particle in the pool.
    void setMaxParticles(u32 count);
    u32 maxParticles() const;
    u32 aliveCount() const;

    void setEmissionMode(ParticleEmissionMode mode);
    ParticleEmissionMode emissionMode() const;
    void setContinuous(f32 particlesPerSecond);
    f32 emissionRate() const;
    void setBurst(u32 count, f32 interval = 1.0f);
    u32 burstCount() const;
    f32 burstInterval() const;
    void setOneShot(u32 count);
    u32 oneShotCount() const;
    void setPulse(f32 particlesPerSecond, u32 particlesPerPulse);
    f32 pulseRate() const;
    u32 particlesPerPulse() const;

    void setShapePoint();
    void setShapeSphere(f32 radius);
    void setShapeBox(const Math::vec3& size);
    // coneAngleDegrees is stored and returned by shapeConeAngle() for the
    // Inspector, same as the system this was ported from, but nothing reads
    // it back yet - a Cone spawns on the same flat disk a Circle does; the
    // spray angle that actually makes it conical is setSpreadAngle() below.
    void setShapeCone(f32 coneAngleDegrees, f32 baseRadius = 0.0f);
    void setShapeCircle(f32 radius);
    void setShapeRing(f32 outerRadius, f32 innerRadius);
    ParticleEmitterShape shape() const;
    f32 shapeRadius() const;
    f32 shapeInnerRadius() const;
    f32 shapeConeAngle() const; // degrees
    const Math::vec3& shapeBoxSize() const;

    void setEmissionOffset(const Math::vec3& offset);
    const Math::vec3& emissionOffset() const;
    // Local-space base spray direction, rotated by the owner's own
    // orientation at spawn time, same as everything else here.
    void setEmissionDirection(const Math::vec3& direction);
    const Math::vec3& emissionDirection() const;
    void setSpreadAngle(f32 degrees);
    f32 spreadAngle() const;

    void setLifetime(f32 lifeMin, f32 lifeMax);
    f32 lifetimeMin() const;
    f32 lifetimeMax() const;
    void setSpeed(f32 speedMin, f32 speedMax);
    f32 speedMin() const;
    f32 speedMax() const;
    void setSize(const Math::vec2& sizeStart, const Math::vec2& sizeEnd);
    const Math::vec2& sizeStart() const;
    const Math::vec2& sizeEnd() const;
    void setColor(Color colorStart, Color colorEnd);
    Color colorStart() const;
    Color colorEnd() const;
    void setRotationSpeed(f32 speedMin, f32 speedMax); // radians/second
    f32 rotationSpeedMin() const;
    f32 rotationSpeedMax() const;

    void setGravity(const Math::vec3& gravity);
    const Math::vec3& gravity() const;
    void setDrag(f32 drag);
    f32 drag() const;
    // <= 0 runs forever. Otherwise the emitter restarts (loop) or stops
    // (!loop) once `seconds` have passed since play() or the last restart.
    void setDuration(f32 seconds);
    f32 duration() const;
    void setLoop(bool loop);
    bool loop() const;

    // An NxM atlas; every particle spawned samples a random cell and keeps
    // it for its whole life.
    void setAtlasGrid(u32 cols, u32 rows);
    void clearAtlas();
    bool usesAtlas() const;
    u32 atlasCols() const;
    u32 atlasRows() const;

    void setTexture(TextureHandle texture);
    TextureHandle texture() const;
    void setTextureFile(const std::string& file);
    const std::string& textureFile() const;
    void setAdditive(bool additive);
    bool additive() const;
    void setDepthTest(bool enabled);
    bool depthTest() const;
    void setBillboardMode(BillboardMode mode);
    BillboardMode billboardMode() const;

    void play();
    void stop();
    void pause();
    bool isPlaying() const;

    // Takes ownership; freed by clearAffectors() or on destruction.
    void addAffector(ParticleAffector* affector);
    bool removeAffector(usize index);
    void clearAffectors();
    const std::vector<ParticleAffector*>& affectors() const;

    // Spawns `count` particles right now, independent of the emission mode.
    void emitBurst(u32 count);

    static void presetBulletImpact(ParticleEmitter& emitter);
    static void presetDebris(ParticleEmitter& emitter);
    // Ambient motes drifting in still air (a sunbeam, a dusty room) - zero
    // gravity, unlike the other two presets. ParticleEffect has a preset by
    // the same name that looks the part but can't do this: its gravity is
    // ParticleSystem's, one setting shared by every GPU-driven effect in the
    // scene, so it falls right along with any bullet impact or explosion
    // sharing that pool. This one is its own emitter with its own gravity.
    static void presetDust(ParticleEmitter& emitter);
    // Same reasoning as presetDust(): a rising plume needs its own near-zero
    // gravity to keep rising instead of eventually being overtaken by
    // whatever ParticleSystem's shared gravity is doing for every other GPU
    // effect in the scene.
    static void presetSmoke(ParticleEmitter& emitter);

private:
    friend class GameObject;

    ParticleEmitter();
    ~ParticleEmitter() override;

    void onLateUpdate(f32 deltaTime) override;
    void onDestroy() override;

    void simulate(f32 deltaTime);
    void emitContinuous(f32 deltaTime);
    void emitBurstMode(f32 deltaTime);
    void emitPulseMode(f32 deltaTime);
    Particle* freeParticle();
    void initParticle(Particle& particle);
    Math::vec3 calcEmissionPosition() const;
    Math::vec3 calcEmissionVelocity() const;
    void updateParticles(f32 deltaTime);
    void applyAffectors(f32 deltaTime);
    void submit();
    f32 rnd(f32 a, f32 b) const;
    Math::vec3 randomUnitVector() const;

    std::vector<Particle> mParticles;
    std::vector<ParticleAffector*> mAffectors;

    ParticleEmissionMode mEmissionMode = ParticleEmissionMode::Continuous;
    ParticleEmitterShape mShape = ParticleEmitterShape::Point;
    bool mPlaying = false;
    bool mPaused = false;
    Math::vec3 mEmissionOffset = Math::vec3(0.0f);
    Math::vec3 mEmissionDirection = Math::vec3(0.0f, 1.0f, 0.0f);
    f32 mEmissionRate = 10.0f;
    f32 mBurstInterval = 1.0f;
    u32 mBurstCount = 10;
    u32 mOneShotCount = 10;
    f32 mPulseRate = 1.0f;
    u32 mParticlesPerPulse = 5;

    std::vector<Math::vec4> mAtlasFrames;
    u32 mAtlasCols = 1, mAtlasRows = 1;
    bool mUseAtlas = false;

    f32 mRadius = 1.0f, mInnerRadius = 0.5f, mConeAngle = 0.785f; // radians
    Math::vec3 mBoxSize = Math::vec3(1.0f);
    f32 mLifetimeMin = 1.0f, mLifetimeMax = 3.0f;
    f32 mSpeedMin = 1.0f, mSpeedMax = 5.0f;
    Math::vec2 mSizeStart = Math::vec2(0.5f), mSizeEnd = Math::vec2(0.1f);
    Color mColorStart;
    Color mColorEnd;
    f32 mRotationSpeedMin = 0.0f, mRotationSpeedMax = 0.0f, mSpreadAngle = 0.0f; // radians
    Math::vec3 mGravity = Math::vec3(0.0f);
    f32 mDrag = 0.0f;
    f32 mDuration = -1.0f;
    bool mLoop = true;
    f32 mEmissionTimer = 0.0f, mTimeAlive = 0.0f, mBurstTimer = 0.0f, mPulseTimer = 0.0f;
    bool mHasEmittedOneShot = false;
    // Where the owner was the last time emitContinuous() ran, so a batch
    // spawned this frame can be smeared back along the distance traveled
    // since then instead of every particle in it landing on the exact same
    // point - see emitContinuous()'s comment.
    Math::vec3 mLastEmitPosition = Math::vec3(0.0f);
    bool mHasLastEmitPosition = false;

    TextureHandle mTexture;
    std::string mTextureFile;
    bool mAdditive = true;
    bool mDepthTest = true;
    BillboardMode mBillboardMode = BillboardMode::Free;

    mutable std::mt19937 mRng;
    mutable std::uniform_real_distribution<f32> mDist01{0.0f, 1.0f};
};

} // namespace Radion

#endif // RADION_PARTICLE_EMITTER_H
