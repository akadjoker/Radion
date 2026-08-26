#include "PCH.h"

#include "ParticleEffect.h"


#include "GameObject.h"
#include "ParticlePass.h"

namespace Radion
{

ParticleEffect::ParticleEffect() : Component(Type, ComponentEventUpdate)
{
}

void ParticleEffect::setMode(ParticleEffectMode mode)
{
    mMode = mode;
}

ParticleEffectMode ParticleEffect::mode() const
{
    return mMode;
}

void ParticleEffect::setEmitter(const ParticleSystem::Emitter& emitter)
{
    mEmitter = emitter;
}

const ParticleSystem::Emitter& ParticleEffect::emitter() const
{
    return mEmitter;
}

void ParticleEffect::setBurstCount(u32 count)
{
    mBurstCount = count;
}

u32 ParticleEffect::burstCount() const
{
    return mBurstCount;
}

void ParticleEffect::setAutoDestroy(bool autoDestroy)
{
    mAutoDestroy = autoDestroy;
}

bool ParticleEffect::autoDestroy() const
{
    return mAutoDestroy;
}

void ParticleEffect::setUseOwnerDirection(bool useOwnerDirection)
{
    mUseOwnerDirection = useOwnerDirection;
}

bool ParticleEffect::useOwnerDirection() const
{
    return mUseOwnerDirection;
}

void ParticleEffect::play()
{
    mPlaying = true;
    mBurstSubmitted = false;
    mAliveTimer = 0.0f;
}

void ParticleEffect::stop()
{
    mPlaying = false;
}

bool ParticleEffect::isPlaying() const
{
    return mPlaying;
}

bool ParticleEffect::isFinished() const
{
    if (mMode != ParticleEffectMode::OneShot)
        return false;
    if (!mBurstSubmitted)
        return false;
    return mAliveTimer >= mEmitter.lifeMax;
}

void ParticleEffect::onAwake()
{
}

void ParticleEffect::onUpdate(f32 deltaTime)
{
    if (!mPlaying)
        return;

    GameObject* object = owner();
    if (!object)
        return;

    mEmitter.position = object->globalPosition();
    if (mUseOwnerDirection)
        mEmitter.direction = object->forward();

    if (mMode == ParticleEffectMode::OneShot)
    {
        if (!mBurstSubmitted)
        {
            submitBurst();
            mBurstSubmitted = true;
            mAliveTimer = 0.0f;
        }

        mAliveTimer += deltaTime;

        if (mAutoDestroy && isFinished())
            object->dispose();
    }
    else
    {
        ParticleDraws().submitContinuous(mEmitter, deltaTime);
    }
}

void ParticleEffect::onDestroy()
{
    stop();
}

void ParticleEffect::submitBurst()
{
    ParticleDraws().submitBurst(mEmitter, mBurstCount);
}

ParticleSystem::Emitter ParticleEffect::presetBulletImpact()
{
    ParticleSystem::Emitter e;
    e.rate = 0.0f;
    e.direction = Math::vec3(0.0f, 1.0f, 0.0f);
    e.spread = 0.6f;
    e.speedMin = 3.0f;
    e.speedMax = 8.0f;
    e.lifeMin = 0.15f;
    e.lifeMax = 0.35f;
    e.sizeBegin = 0.08f;
    e.sizeEnd = 0.02f;
    e.colorBegin = Math::vec4(1.0f, 0.9f, 0.4f, 1.0f);
    e.colorEnd = Math::vec4(1.0f, 0.2f, 0.05f, 0.0f);
    e.mass = 1.0f;
    e.rotationVelocity = 2.0f;
    e.startRadius = 0.02f;
    return e;
}

ParticleSystem::Emitter ParticleEffect::presetBlood()
{
    // Slower and shorter-lived than a spark (presetBulletImpact): a droplet
    // arcs and falls, it does not ricochet - the caller pairing this with a
    // ballistic drop of its own (a demo tracking a few points against the
    // level, one gravity integration per frame) wants the visible burst to
    // finish at roughly the same time those land.
    ParticleSystem::Emitter e;
    e.rate = 0.0f;
    e.direction = Math::vec3(0.0f, 1.0f, 0.0f);
    e.spread = 0.55f;
    e.speedMin = 1.5f;
    e.speedMax = 4.5f;
    e.lifeMin = 0.3f;
    e.lifeMax = 0.6f;
    e.sizeBegin = 0.06f;
    e.sizeEnd = 0.02f;
    e.colorBegin = Math::vec4(0.55f, 0.02f, 0.02f, 1.0f);
    e.colorEnd = Math::vec4(0.20f, 0.0f, 0.0f, 0.0f);
    e.mass = 1.0f;
    e.rotationVelocity = 1.0f;
    e.startRadius = 0.03f;
    return e;
}

ParticleSystem::Emitter ParticleEffect::presetExplosion()
{
    ParticleSystem::Emitter e;
    e.rate = 0.0f;
    e.direction = Math::vec3(0.0f, 1.0f, 0.0f);
    e.spread = 3.14159265f; // full sphere
    e.speedMin = 2.0f;
    e.speedMax = 9.0f;
    e.lifeMin = 0.6f;
    e.lifeMax = 1.2f;
    e.sizeBegin = 0.6f;
    e.sizeEnd = 1.2f;
    e.colorBegin = Math::vec4(1.0f, 0.6f, 0.15f, 1.0f);
    e.colorEnd = Math::vec4(0.2f, 0.05f, 0.02f, 0.0f);
    e.mass = 1.0f;
    e.rotationVelocity = 1.5f;
    e.startRadius = 0.1f;
    return e;
}

ParticleSystem::Emitter ParticleEffect::presetFirework()
{
    ParticleSystem::Emitter e;
    e.rate = 0.0f;
    e.direction = Math::vec3(0.0f, 1.0f, 0.0f);
    e.spread = 3.14159265f;
    // Speed kept in a narrow band on purpose - fireworks read as one
    // expanding shell when every particle is roughly the same distance out
    // at any given moment; a wide speedMin/speedMax spread thins some of
    // the shell out ahead of the rest and the burst looks patchy instead of
    // one coherent wave.
    e.speedMin = 8.0f;
    e.speedMax = 10.0f;
    e.lifeMin = 0.9f;
    e.lifeMax = 1.3f;
    e.sizeBegin = 0.22f;
    e.sizeEnd = 0.08f;
    e.colorBegin = Math::vec4(0.6f, 0.85f, 1.0f, 1.0f);
    e.colorEnd = Math::vec4(0.9f, 0.3f, 0.6f, 0.0f);
    e.mass = 0.5f;
    e.rotationVelocity = 2.5f;
    e.startRadius = 0.05f;
    return e;
}

// Ambient motes drifting in still air (a sunbeam, a dusty room) - a large
// startRadius spawns them through a whole volume rather than from a point,
// spread = PI (full sphere) sends each one drifting in its own random
// direction instead of all rising together, and the long life keeps the
// volume populated at a low, steady rate. gravity/drag are global on the
// ParticleSystem though (ParticleDraws().system()), not per-emitter - a
// scene using this preset alongside a falling effect (sparks, debris) has
// to keep gravity near zero, or the dust falls too.
ParticleSystem::Emitter ParticleEffect::presetDust()
{
    ParticleSystem::Emitter e;
    e.rate = 3.0f;
    e.direction = Math::vec3(0.0f, 1.0f, 0.0f);
    e.spread = 3.14159265f; // full sphere
    e.speedMin = 0.03f;
    e.speedMax = 0.15f;
    e.lifeMin = 4.0f;
    e.lifeMax = 8.0f;
    // Small enough to read as a mote, not so small it falls under a pixel at
    // normal viewing distance and vanishes - the fragment shader's falloff
    // (particle.frag) already shrinks the visible core well inside the quad.
    e.sizeBegin = 0.05f;
    e.sizeEnd = 0.08f;
    e.colorBegin = Math::vec4(1.0f, 1.0f, 0.95f, 0.85f);
    e.colorEnd = Math::vec4(1.0f, 1.0f, 0.95f, 0.0f);
    e.mass = 1.0f;
    e.rotationVelocity = 0.2f;
    e.startRadius = 2.0f;
    return e;
}

ParticleSystem::Emitter ParticleEffect::presetSmoke()
{
    ParticleSystem::Emitter e;
    e.rate = 40.0f;
    e.direction = Math::vec3(0.0f, 100.0f, 0.0f);
    e.spread = 0.4f;
    e.speedMin = 0.5f;
    e.speedMax = 1.5f;
    e.lifeMin = 2.0f;
    e.lifeMax = 4.0f;
    e.sizeBegin = 0.3f;
    e.sizeEnd = 1.0f;
    e.colorBegin = Math::vec4(0.6f, 0.6f, 0.6f, 0.4f);
    e.colorEnd = Math::vec4(0.5f, 0.5f, 0.5f, 0.0f);
    e.mass = 2.0f;
    e.rotationVelocity = 0.8f;
    e.startRadius = 0.05f;
    return e;
}

} // namespace Radion
