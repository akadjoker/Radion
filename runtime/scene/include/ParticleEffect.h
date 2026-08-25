#ifndef RADION_PARTICLE_EFFECT_H
#define RADION_PARTICLE_EFFECT_H

#include "Component.h"
#include "ParticleRender.h"


namespace Radion
{

enum class ParticleEffectMode : u8
{
    OneShot,    // emits once and can auto-destroy when all particles die
    Continuous  // emits every frame while playing
};

// Scene component that drives the global GPU-driven particle system.
// Attach it to a GameObject, configure the emitter, and call play().
class ParticleEffect final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::ParticleEffect;

    void setMode(ParticleEffectMode mode);
    ParticleEffectMode mode() const;

    void setEmitter(const ParticleSystem::Emitter& emitter);
    const ParticleSystem::Emitter& emitter() const;

    // One-shot: how many particles to spawn on play().
    // Continuous: ignored; rate on the emitter controls emission.
    void setBurstCount(u32 count);
    u32 burstCount() const;

    void setAutoDestroy(bool autoDestroy);
    bool autoDestroy() const;

    void setUseOwnerDirection(bool useOwnerDirection);
    bool useOwnerDirection() const;

    void play();
    void stop();
    bool isPlaying() const;

    // True only for one-shot effects that already fired and have no alive particles.
    bool isFinished() const;

    // Presets for common effects. They only fill emitter parameters; position
    // and direction are still taken from the owner GameObject when playing.
    static ParticleSystem::Emitter presetBulletImpact();
    static ParticleSystem::Emitter presetExplosion();
    static ParticleSystem::Emitter presetFirework();
    static ParticleSystem::Emitter presetSmoke();
    static ParticleSystem::Emitter presetDust();
    static ParticleSystem::Emitter presetBlood();

private:
    friend class GameObject;

    ParticleEffect();

    void onAwake() override;
    void onUpdate(f32 deltaTime) override;
    void onDestroy() override;

    void submitBurst();

    ParticleSystem::Emitter mEmitter;
    ParticleEffectMode mMode = ParticleEffectMode::OneShot;
    u32 mBurstCount = 32;
    bool mPlaying = false;
    bool mBurstSubmitted = false;
    bool mAutoDestroy = true;
    bool mUseOwnerDirection = true;
    f32 mAliveTimer = 0.0f;
};

} // namespace Radion

#endif // RADION_PARTICLE_EFFECT_H
