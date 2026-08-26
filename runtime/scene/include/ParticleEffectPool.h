#ifndef RADION_PARTICLE_EFFECT_POOL_H
#define RADION_PARTICLE_EFFECT_POOL_H

#include "ParticleEffect.h"
#include "Types.h"

#include <glm/glm.hpp>
#include <vector>

namespace Radion
{

class Scene;

// Object pool for one-shot particle effects. Reuses GameObjects with a
// ParticleEffect component instead of creating/destroying them every time a
// bullet hits or an explosion fires.
class ParticleEffectPool
{
public:
    static ParticleEffectPool& getSingleton();

    void initialize(Scene& scene);
    void shutdown();

    // Spawns a one-shot effect at the given world position. If direction is
    // non-zero the spawned GameObject is rotated to face it.
    ParticleEffect* spawn(const ParticleSystem::Emitter& emitter, u32 burstCount,
                          const Math::Vec3& position,
                          const Math::Vec3& direction = Math::Vec3(0.0f));

    // Moves finished one-shots back to the available list and disables them.
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
