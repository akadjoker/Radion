#ifndef RADION_PARTICLE_EFFECT_POOL_H
#define RADION_PARTICLE_EFFECT_POOL_H

#include "ParticleEffect.h"
#include "Types.h"

#include "Math.h"
#include <vector>

namespace Radion
{

class Scene;

class ParticleEffectPool
{
public:
    static ParticleEffectPool& getSingleton();

    void initialize(Scene& scene);
    void shutdown();

    ParticleEffect* spawn(const ParticleSystem::Emitter& emitter, u32 burstCount,
                          const Math::vec3& position,
                          const Math::vec3& direction = Math::vec3(0.0f));

    // Call once per frame after Scene::update().
    void reclaim();

    usize activeCount() const;
    usize availableCount() const;

private:
    ParticleEffectPool() = default;

    Scene* mScene = nullptr;
    std::vector<GameObject*> mAvailable;
    std::vector<GameObject*> mActive;
};

} // namespace Radion

#endif // RADION_PARTICLE_EFFECT_POOL_H
