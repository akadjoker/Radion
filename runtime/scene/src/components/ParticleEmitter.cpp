#include "PCH.h"

#include "ParticleEmitter.h"

#include "GameObject.h"
#include "AssetManager.h"

#include <cmath>

namespace Radion
{

namespace
{
constexpr f32 kTau = 6.28318530f;
constexpr f32 kDegToRad = 3.14159265f / 180.0f;
constexpr f32 kRadToDeg = 180.0f / 3.14159265f;
constexpr u32 kDefaultMaxParticles = 200;
}

// ── Affectors ──

void GravityAffector::apply(Particle& particle, f32)
{
    if (enabled)
        particle.acceleration += gravity;
}
void DragAffector::apply(Particle& particle, f32 deltaTime)
{
    if (enabled)
        particle.velocity *= (1.0f - drag * deltaTime);
}
void VortexAffector::apply(Particle& particle, f32)
{
    if (!enabled)
        return;
    const Math::vec3 toCenter = center - particle.position;
    const f32 d = Math::length(toCenter);
    if (d < radius && d > 0.001f)
    {
        const Math::vec3 dir = toCenter * (1.0f / d);
        const Math::vec3 tangent(-dir.z, 0.0f, dir.x);
        particle.acceleration += tangent * (strength * (1.0f - d / radius));
    }
}
void AttractorAffector::apply(Particle& particle, f32)
{
    if (!enabled)
        return;
    const Math::vec3 to = position - particle.position;
    const f32 d = Math::length(to);
    if (d < radius && d > 0.001f)
        particle.acceleration +=
            (to * (1.0f / d)) * (strength * (1.0f - d / radius) * (repulse ? -1.0f : 1.0f));
}
void TurbulenceAffector::apply(Particle& particle, f32 deltaTime)
{
    if (!enabled)
        return;
    time += deltaTime;
    particle.acceleration +=
        Math::vec3(Math::sin(particle.position.x * frequency + time) * strength,
                 Math::sin(particle.position.y * frequency + time * 1.3f) * strength,
                 Math::cos(particle.position.z * frequency + time * 0.7f) * strength);
}
void ColorOverLifetimeAffector::apply(Particle& particle, f32)
{
    if (enabled)
        particle.color = Color::lerp(startColor, endColor, particle.timeAlive / particle.lifetime);
}
void SizeOverLifetimeAffector::apply(Particle& particle, f32)
{
    if (enabled)
        particle.size = Math::mix(startSize, endSize, particle.timeAlive / particle.lifetime);
}

// ── ParticleEmitter ──

ParticleEmitter::ParticleEmitter() : Component(Type, ComponentEventLateUpdate)
{
    mParticles.assign(kDefaultMaxParticles, Particle());
    mColorEnd = Color(255, 255, 255, 0);
}

ParticleEmitter::~ParticleEmitter()
{
    clearAffectors();
}

void ParticleEmitter::setMaxParticles(u32 count)
{
    mParticles.assign(Math::max(1u, count), Particle());
}
u32 ParticleEmitter::maxParticles() const
{
    return (u32)mParticles.size();
}
u32 ParticleEmitter::aliveCount() const
{
    u32 count = 0;
    for (const Particle& p : mParticles)
        if (p.active)
            ++count;
    return count;
}

void ParticleEmitter::setEmissionMode(ParticleEmissionMode mode)
{
    mEmissionMode = mode;
}
ParticleEmissionMode ParticleEmitter::emissionMode() const
{
    return mEmissionMode;
}
void ParticleEmitter::setContinuous(f32 particlesPerSecond)
{
    mEmissionMode = ParticleEmissionMode::Continuous;
    mEmissionRate = particlesPerSecond;
}
f32 ParticleEmitter::emissionRate() const
{
    return mEmissionRate;
}
void ParticleEmitter::setBurst(u32 count, f32 interval)
{
    mEmissionMode = ParticleEmissionMode::Burst;
    mBurstCount = count;
    mBurstInterval = interval;
}
u32 ParticleEmitter::burstCount() const
{
    return mBurstCount;
}
f32 ParticleEmitter::burstInterval() const
{
    return mBurstInterval;
}
void ParticleEmitter::setOneShot(u32 count)
{
    mEmissionMode = ParticleEmissionMode::OneShot;
    mOneShotCount = count;
}
u32 ParticleEmitter::oneShotCount() const
{
    return mOneShotCount;
}
void ParticleEmitter::setPulse(f32 particlesPerSecond, u32 particlesPerPulse)
{
    mEmissionMode = ParticleEmissionMode::Pulse;
    mPulseRate = particlesPerSecond;
    mParticlesPerPulse = particlesPerPulse;
}
f32 ParticleEmitter::pulseRate() const
{
    return mPulseRate;
}
u32 ParticleEmitter::particlesPerPulse() const
{
    return mParticlesPerPulse;
}

void ParticleEmitter::setShapePoint()
{
    mShape = ParticleEmitterShape::Point;
}
void ParticleEmitter::setShapeSphere(f32 radius)
{
    mShape = ParticleEmitterShape::Sphere;
    mRadius = radius;
}
void ParticleEmitter::setShapeBox(const Math::vec3& size)
{
    mShape = ParticleEmitterShape::Box;
    mBoxSize = size;
}
void ParticleEmitter::setShapeCone(f32 coneAngleDegrees, f32 baseRadius)
{
    mShape = ParticleEmitterShape::Cone;
    mConeAngle = coneAngleDegrees * kDegToRad;
    mRadius = baseRadius;
}
void ParticleEmitter::setShapeCircle(f32 radius)
{
    mShape = ParticleEmitterShape::Circle;
    mRadius = radius;
}
void ParticleEmitter::setShapeRing(f32 outerRadius, f32 innerRadius)
{
    mShape = ParticleEmitterShape::Ring;
    mRadius = outerRadius;
    mInnerRadius = innerRadius;
}
ParticleEmitterShape ParticleEmitter::shape() const
{
    return mShape;
}
f32 ParticleEmitter::shapeRadius() const
{
    return mRadius;
}
f32 ParticleEmitter::shapeInnerRadius() const
{
    return mInnerRadius;
}
f32 ParticleEmitter::shapeConeAngle() const
{
    return mConeAngle * kRadToDeg;
}
const Math::vec3& ParticleEmitter::shapeBoxSize() const
{
    return mBoxSize;
}

void ParticleEmitter::setEmissionOffset(const Math::vec3& offset)
{
    mEmissionOffset = offset;
}
const Math::vec3& ParticleEmitter::emissionOffset() const
{
    return mEmissionOffset;
}
void ParticleEmitter::setEmissionDirection(const Math::vec3& direction)
{
    mEmissionDirection = direction;
}
const Math::vec3& ParticleEmitter::emissionDirection() const
{
    return mEmissionDirection;
}
void ParticleEmitter::setSpreadAngle(f32 degrees)
{
    mSpreadAngle = degrees * kDegToRad;
}
f32 ParticleEmitter::spreadAngle() const
{
    return mSpreadAngle * kRadToDeg;
}

void ParticleEmitter::setLifetime(f32 lifeMin, f32 lifeMax)
{
    mLifetimeMin = lifeMin;
    mLifetimeMax = lifeMax;
}
f32 ParticleEmitter::lifetimeMin() const
{
    return mLifetimeMin;
}
f32 ParticleEmitter::lifetimeMax() const
{
    return mLifetimeMax;
}
void ParticleEmitter::setSpeed(f32 speedMin, f32 speedMax)
{
    mSpeedMin = speedMin;
    mSpeedMax = speedMax;
}
f32 ParticleEmitter::speedMin() const
{
    return mSpeedMin;
}
f32 ParticleEmitter::speedMax() const
{
    return mSpeedMax;
}
void ParticleEmitter::setSize(const Math::vec2& sizeStart, const Math::vec2& sizeEnd)
{
    mSizeStart = sizeStart;
    mSizeEnd = sizeEnd;
}
const Math::vec2& ParticleEmitter::sizeStart() const
{
    return mSizeStart;
}
const Math::vec2& ParticleEmitter::sizeEnd() const
{
    return mSizeEnd;
}
void ParticleEmitter::setColor(Color colorStart, Color colorEnd)
{
    mColorStart = colorStart;
    mColorEnd = colorEnd;
}
Color ParticleEmitter::colorStart() const
{
    return mColorStart;
}
Color ParticleEmitter::colorEnd() const
{
    return mColorEnd;
}
void ParticleEmitter::setRotationSpeed(f32 speedMin, f32 speedMax)
{
    mRotationSpeedMin = speedMin;
    mRotationSpeedMax = speedMax;
}
f32 ParticleEmitter::rotationSpeedMin() const
{
    return mRotationSpeedMin;
}
f32 ParticleEmitter::rotationSpeedMax() const
{
    return mRotationSpeedMax;
}

void ParticleEmitter::setGravity(const Math::vec3& gravity)
{
    mGravity = gravity;
}
const Math::vec3& ParticleEmitter::gravity() const
{
    return mGravity;
}
void ParticleEmitter::setDrag(f32 drag)
{
    mDrag = drag;
}
f32 ParticleEmitter::drag() const
{
    return mDrag;
}
void ParticleEmitter::setDuration(f32 seconds)
{
    mDuration = seconds;
}
f32 ParticleEmitter::duration() const
{
    return mDuration;
}
void ParticleEmitter::setLoop(bool loop)
{
    mLoop = loop;
}
bool ParticleEmitter::loop() const
{
    return mLoop;
}

void ParticleEmitter::setAtlasGrid(u32 cols, u32 rows)
{
    mAtlasFrames.clear();
    cols = cols > 0 ? cols : 1;
    rows = rows > 0 ? rows : 1;
    const f32 cw = 1.0f / (f32)cols, ch = 1.0f / (f32)rows;
    for (u32 row = 0; row < rows; ++row)
        for (u32 col = 0; col < cols; ++col)
            mAtlasFrames.push_back(Math::vec4((f32)col * cw, (f32)row * ch, cw, ch));
    mAtlasCols = cols;
    mAtlasRows = rows;
    mUseAtlas = true;
}
void ParticleEmitter::clearAtlas()
{
    mAtlasFrames.clear();
    mUseAtlas = false;
}
bool ParticleEmitter::usesAtlas() const
{
    return mUseAtlas;
}
u32 ParticleEmitter::atlasCols() const
{
    return mAtlasCols;
}
u32 ParticleEmitter::atlasRows() const
{
    return mAtlasRows;
}

void ParticleEmitter::setTexture(TextureHandle texture)
{
    mTexture = texture;
}

void ParticleEmitter::setTextureFile(const std::string& file)
{
    mTextureFile = file;
    mTexture = file.empty() ? TextureHandle() : Assets().loadTexture(file, ColorSpace::sRGB);
}

const std::string& ParticleEmitter::textureFile() const
{
    return mTextureFile;
}
TextureHandle ParticleEmitter::texture() const
{
    return mTexture;
}
void ParticleEmitter::setAdditive(bool additive)
{
    mAdditive = additive;
}
bool ParticleEmitter::additive() const
{
    return mAdditive;
}
void ParticleEmitter::setDepthTest(bool enabled)
{
    mDepthTest = enabled;
}
bool ParticleEmitter::depthTest() const
{
    return mDepthTest;
}
void ParticleEmitter::setBillboardMode(BillboardMode mode)
{
    mBillboardMode = mode;
}
BillboardMode ParticleEmitter::billboardMode() const
{
    return mBillboardMode;
}

void ParticleEmitter::play()
{
    mPlaying = true;
    mPaused = false;
    // Forget where continuous emission last measured the owner from - if
    // the owner was moved (or this is the first play() ever), the next
    // emitContinuous() must not treat that gap as travel to smear a batch
    // across.
    mHasLastEmitPosition = false;
    if (mEmissionMode == ParticleEmissionMode::OneShot && !mHasEmittedOneShot)
    {
        emitBurst(mOneShotCount);
        mHasEmittedOneShot = true;
    }
}
void ParticleEmitter::stop()
{
    mPlaying = false;
    mPaused = false;
    for (Particle& p : mParticles)
        p.active = false;
}
void ParticleEmitter::pause()
{
    mPaused = true;
}
bool ParticleEmitter::isPlaying() const
{
    return mPlaying && !mPaused;
}

void ParticleEmitter::addAffector(ParticleAffector* affector)
{
    if (affector)
        mAffectors.push_back(affector);
}

bool ParticleEmitter::removeAffector(usize index)
{
    if (index >= mAffectors.size())
        return false;
    delete mAffectors[index];
    mAffectors.erase(mAffectors.begin() + index);
    return true;
}
void ParticleEmitter::clearAffectors()
{
    for (ParticleAffector* affector : mAffectors)
        delete affector;
    mAffectors.clear();
}

const std::vector<ParticleAffector*>& ParticleEmitter::affectors() const
{
    return mAffectors;
}

void ParticleEmitter::emitBurst(u32 count)
{
    for (u32 i = 0; i < count; ++i)
    {
        Particle* p = freeParticle();
        if (p)
            initParticle(*p);
    }
}

void ParticleEmitter::onLateUpdate(f32 deltaTime)
{
    simulate(deltaTime);
    submit();
}

void ParticleEmitter::onDestroy()
{
    stop();
}

void ParticleEmitter::simulate(f32 deltaTime)
{
    if (mPaused)
        return;
    mTimeAlive += deltaTime;
    if (mPlaying)
    {
        switch (mEmissionMode)
        {
        case ParticleEmissionMode::Continuous:
            emitContinuous(deltaTime);
            break;
        case ParticleEmissionMode::Burst:
            emitBurstMode(deltaTime);
            break;
        case ParticleEmissionMode::Pulse:
            emitPulseMode(deltaTime);
            break;
        case ParticleEmissionMode::OneShot:
            break;
        }
        if (mDuration > 0.0f && mTimeAlive >= mDuration)
        {
            if (mLoop)
            {
                mTimeAlive = 0.0f;
                mHasEmittedOneShot = false;
            }
            else
            {
                stop();
            }
        }
    }
    updateParticles(deltaTime);
    applyAffectors(deltaTime);
}

void ParticleEmitter::emitContinuous(f32 deltaTime)
{
    mEmissionTimer += deltaTime;
    const f32 interval = 1.0f / mEmissionRate;
    u32 count = 0;
    while (mEmissionTimer >= interval)
    {
        ++count;
        mEmissionTimer -= interval;
        if (count > 100)
        {
            mEmissionTimer = 0.0f;
            break;
        }
    }
    if (count == 0)
        return;

    const Math::vec3 currentPosition = owner()->globalPosition();
    const Math::vec3 travel =
        mHasLastEmitPosition ? currentPosition - mLastEmitPosition : Math::vec3(0.0f);
    mLastEmitPosition = currentPosition;
    mHasLastEmitPosition = true;

    // Smears this frame's whole batch back along the distance the owner
    // traveled since the last continuous emit, oldest-first. Without this,
    // every particle in the batch spawns at today's single instantaneous
    // position - fine for a still or slow emitter, but a fast-moving one
    // (a climbing firework rocket, e.g.) spawning dozens of particles a
    // frame stacks all of them on one point, and the next frame's batch on
    // the next point, beading the trail into visible gaps instead of a
    // smooth one.
    for (u32 i = 0; i < count; ++i)
    {
        Particle* p = freeParticle();
        if (!p)
            continue;
        initParticle(*p);
        const f32 t = count > 1 ? (f32)i / (f32)(count - 1) : 1.0f;
        p->position -= travel * (1.0f - t);
    }
}
void ParticleEmitter::emitBurstMode(f32 deltaTime)
{
    mBurstTimer += deltaTime;
    if (mBurstTimer >= mBurstInterval)
    {
        emitBurst(mBurstCount);
        mBurstTimer = 0.0f;
        if (!mLoop)
            stop();
    }
}
void ParticleEmitter::emitPulseMode(f32 deltaTime)
{
    mPulseTimer += deltaTime;
    const f32 interval = 1.0f / mPulseRate;
    if (mPulseTimer >= interval)
    {
        emitBurst(mParticlesPerPulse);
        mPulseTimer = 0.0f;
    }
}

// Pool never grows: an inactive slot is reused if there is one; otherwise
// the particle furthest through its life (past 90%) is recycled early
// rather than refusing the new one.
Particle* ParticleEmitter::freeParticle()
{
    for (Particle& p : mParticles)
        if (!p.active)
            return &p;
    Particle* oldest = nullptr;
    f32 maxProgress = 0.9f;
    for (Particle& p : mParticles)
    {
        const f32 progress = p.timeAlive / p.lifetime;
        if (progress > maxProgress)
        {
            maxProgress = progress;
            oldest = &p;
        }
    }
    return oldest;
}

void ParticleEmitter::initParticle(Particle& particle)
{
    particle.position = calcEmissionPosition();
    particle.velocity = calcEmissionVelocity();
    particle.acceleration = mGravity;
    particle.lifetime = rnd(mLifetimeMin, mLifetimeMax);
    particle.timeAlive = 0.0f;
    particle.sizeStart = mSizeStart;
    particle.sizeEnd = mSizeEnd;
    particle.size = mSizeStart;
    particle.colorStart = mColorStart;
    particle.colorEnd = mColorEnd;
    particle.color = mColorStart;
    particle.rotation = rnd(0.0f, kTau);
    particle.rotationSpeed = rnd(mRotationSpeedMin, mRotationSpeedMax);
    // A random cell per particle, not a shared current-frame index: the
    // system this was ported from tracked a "current frame" that nothing
    // ever advanced, so every particle always drew atlas cell 0. Picking at
    // random here is a deliberate fix, not a faithfully-ported behavior -
    // an atlas grid that always shows its first cell has no reason to exist.
    if (mUseAtlas && !mAtlasFrames.empty())
    {
        const u32 index = Math::min((u32)(mDist01(mRng) * (f32)mAtlasFrames.size()),
                                   (u32)mAtlasFrames.size() - 1);
        particle.texRect = mAtlasFrames[index];
    }
    else
    {
        particle.texRect = Math::vec4(0.0f, 0.0f, 1.0f, 1.0f);
    }
    particle.active = true;
}

// Point/Sphere/Box offsets are computed in the owner's LOCAL space and
// rotated into world space through its full transform, same as the system
// this was ported from. Cone/Circle/Ring build their offset directly from
// the owner's own WORLD-space right/up axes - ported fixing a bug in the
// original, where that already-world-space offset was rotated by the
// owner's transform a second time, spinning the spawn disk further than
// intended whenever the emitter itself wasn't axis-aligned.
Math::vec3 ParticleEmitter::calcEmissionPosition() const
{
    GameObject* object = owner();
    const Math::mat4& world = object->globalTransform();
    const Math::vec3 basePosition = Math::vec3(world * Math::vec4(mEmissionOffset, 1.0f));

    switch (mShape)
    {
    case ParticleEmitterShape::Point:
        return basePosition;

    case ParticleEmitterShape::Sphere:
    {
        const Math::vec3 dir = randomUnitVector();
        const f32 r = std::cbrt(mDist01(mRng)) * mRadius; // cbrt: uniform over the volume
        const Math::vec3 localOffset = dir * r;
        return basePosition + Math::vec3(world * Math::vec4(localOffset, 0.0f));
    }
    case ParticleEmitterShape::Box:
    {
        const Math::vec3 localOffset(rnd(-mBoxSize.x * 0.5f, mBoxSize.x * 0.5f),
                                    rnd(-mBoxSize.y * 0.5f, mBoxSize.y * 0.5f),
                                    rnd(-mBoxSize.z * 0.5f, mBoxSize.z * 0.5f));
        return basePosition + Math::vec3(world * Math::vec4(localOffset, 0.0f));
    }
    case ParticleEmitterShape::Cone:
    case ParticleEmitterShape::Circle:
    {
        if (mRadius <= 0.0f)
            return basePosition;
        const f32 a = rnd(0.0f, kTau);
        const f32 r = Math::sqrt(mDist01(mRng)) * mRadius; // sqrt: uniform over the disk's area
        return basePosition + object->right() * (Math::cos(a) * r) + object->up() * (Math::sin(a) * r);
    }
    case ParticleEmitterShape::Ring:
    {
        const f32 a = rnd(0.0f, kTau);
        const f32 r = rnd(mInnerRadius, mRadius);
        return basePosition + object->right() * (Math::cos(a) * r) + object->up() * (Math::sin(a) * r);
    }
    }
    return basePosition;
}

Math::vec3 ParticleEmitter::calcEmissionVelocity() const
{
    GameObject* object = owner();
    const f32 speed = rnd(mSpeedMin, mSpeedMax);
    Math::vec3 dir = Math::normalize(
        Math::vec3(object->globalTransform() * Math::vec4(mEmissionDirection, 0.0f)));

    if (mSpreadAngle > 0.0f)
    {
        const f32 theta = rnd(0.0f, kTau);
        const f32 phi = rnd(0.0f, mSpreadAngle);
        Math::vec3 u(0.0f, 1.0f, 0.0f);
        if (Math::abs(Math::dot(dir, u)) > 0.99f)
            u = Math::vec3(1.0f, 0.0f, 0.0f);
        const Math::vec3 r2 = Math::normalize(Math::cross(dir, u));
        u = Math::normalize(Math::cross(r2, dir));
        dir = Math::normalize(dir * Math::cos(phi) +
                             (r2 * Math::cos(theta) + u * Math::sin(theta)) * Math::sin(phi));
    }
    return dir * speed;
}

void ParticleEmitter::updateParticles(f32 deltaTime)
{
    for (Particle& p : mParticles)
    {
        if (!p.active)
            continue;
        p.timeAlive += deltaTime;
        if (p.timeAlive >= p.lifetime)
        {
            p.active = false;
            continue;
        }
        p.velocity += p.acceleration * deltaTime;
        if (mDrag > 0.0f)
            p.velocity *= (1.0f - mDrag * deltaTime);
        p.position += p.velocity * deltaTime;
        p.rotation += p.rotationSpeed * deltaTime;
        const f32 t = p.timeAlive / p.lifetime;
        p.color = Color::lerp(p.colorStart, p.colorEnd, t);
        p.size = Math::mix(p.sizeStart, p.sizeEnd, t);
        // Reset for this frame's affectors to build back up; consumed as
        // last frame's acceleration on the NEXT call, same one-frame-delayed
        // shape the system this was ported from used.
        p.acceleration = mGravity;
    }
}

void ParticleEmitter::applyAffectors(f32 deltaTime)
{
    if (mAffectors.empty())
        return;
    for (Particle& p : mParticles)
    {
        if (!p.active)
            continue;
        for (ParticleAffector* affector : mAffectors)
            if (affector && affector->enabled)
                affector->apply(p, deltaTime);
    }
}

void ParticleEmitter::submit()
{
    if (mParticles.empty())
        return;
    GameObject* object = owner();
    const Math::vec3 fixedRight = object->right();
    const Math::vec3 fixedUp = object->up();

    for (const Particle& p : mParticles)
    {
        if (!p.active)
            continue;
        BillboardInstance instance;
        instance.position = p.position;
        instance.size = p.size;
        instance.rotation = p.rotation;
        instance.uvRect = p.texRect;
        instance.color = p.color;
        instance.mode = mBillboardMode;
        instance.fixedRight = fixedRight;
        instance.fixedUp = fixedUp;
        instance.texture = mTexture;
        instance.blend =
            mAdditive ? BatchRenderer::BlendMode::Additive : BatchRenderer::BlendMode::Alpha;
        instance.depthTest = mDepthTest;
        TrailDraws().submit(instance);
    }
}

f32 ParticleEmitter::rnd(f32 a, f32 b) const
{
    return a + mDist01(mRng) * (b - a);
}
Math::vec3 ParticleEmitter::randomUnitVector() const
{
    const f32 z = rnd(-1.0f, 1.0f);
    const f32 a = rnd(0.0f, kTau);
    const f32 r = Math::sqrt(Math::max(0.0f, 1.0f - z * z));
    return Math::vec3(r * Math::cos(a), r * Math::sin(a), z);
}

void ParticleEmitter::presetBulletImpact(ParticleEmitter& emitter)
{
    emitter.setOneShot(20);
    emitter.setShapePoint();
    emitter.setEmissionDirection(Math::vec3(0.0f, 1.0f, 0.0f));
    emitter.setSpreadAngle(70.0f);
    emitter.setSpeed(2.0f, 6.0f);
    emitter.setLifetime(0.15f, 0.35f);
    emitter.setSize(Math::vec2(0.06f), Math::vec2(0.015f));
    emitter.setColor(Color(255, 220, 120, 255), Color(255, 60, 20, 0));
    emitter.setRotationSpeed(-6.0f, 6.0f);
    emitter.setGravity(Math::vec3(0.0f, -9.8f, 0.0f));
    emitter.setDrag(2.0f);
}

void ParticleEmitter::presetDust(ParticleEmitter& emitter)
{
    emitter.setContinuous(3.0f);
    emitter.setShapeSphere(2.0f); // call setShapeSphere() again to change the volume's radius
    emitter.setEmissionDirection(Math::vec3(0.0f, 1.0f, 0.0f));
    emitter.setSpreadAngle(180.0f); // full sphere: drifts every which way, not just "up"
    emitter.setSpeed(0.03f, 0.15f);
    emitter.setLifetime(15.0f, 25.0f);
    // Small enough to read as a fine speck, not so small it falls under a
    // pixel at normal viewing distance and vanishes. Near-constant size -
    // a mote drifting in a sunbeam doesn't visibly grow the way smoke does.
    emitter.setSize(Math::vec2(0.02f), Math::vec2(0.025f));
    emitter.setColor(Color(255, 255, 242, 230), Color(255, 255, 242, 0));
    emitter.setRotationSpeed(-0.2f, 0.2f);
    // The whole point of this being its own CPU emitter: gravity/drag are
    // this emitter's own, not ParticleSystem's shared setting, so dust can
    // float while a bullet impact or explosion sharing the GPU pool still
    // falls normally.
    emitter.setGravity(Math::vec3(0.0f));
    emitter.setDrag(0.1f);
}

void ParticleEmitter::presetSmoke(ParticleEmitter& emitter)
{
    emitter.setContinuous(40.0f);
    emitter.setShapeSphere(0.05f);
    emitter.setEmissionDirection(Math::vec3(0.0f, 1.0f, 0.0f));
    emitter.setSpreadAngle(23.0f); // ~0.4 radians, a narrow rising column
    emitter.setSpeed(0.5f, 1.5f);
    emitter.setLifetime(2.0f, 4.0f);
    emitter.setSize(Math::vec2(0.3f), Math::vec2(1.0f));
    emitter.setColor(Color(153, 153, 153, 102), Color(128, 128, 128, 0));
    emitter.setGravity(Math::vec3(0.0f));
    emitter.setDrag(0.3f);
}

void ParticleEmitter::presetDebris(ParticleEmitter& emitter)
{
    emitter.setOneShot(14);
    emitter.setShapeSphere(0.05f);
    emitter.setEmissionDirection(Math::vec3(0.0f, 1.0f, 0.0f));
    emitter.setSpreadAngle(120.0f);
    emitter.setSpeed(1.5f, 5.0f);
    emitter.setLifetime(0.6f, 1.4f);
    emitter.setSize(Math::vec2(0.04f), Math::vec2(0.04f));
    emitter.setColor(Color(170, 150, 130, 255), Color(120, 105, 90, 255));
    emitter.setRotationSpeed(-8.0f, 8.0f);
    emitter.setGravity(Math::vec3(0.0f, -9.8f, 0.0f));
    emitter.setDrag(0.4f);
}

} // namespace Radion
