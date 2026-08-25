#ifndef RADION_PARTICLE_PASS_H
#define RADION_PARTICLE_PASS_H

#include "ParticleRender.h"
#include "RenderTechnique.h"

#include <string>

namespace Radion
{

// Global submission queue for particle effects. Mirrors the pattern used by
// TrailDraws(): components submit emit requests during the frame, and the
// ParticlePass executes them in one GPU-driven draw call.
class ParticleRenderQueue
{
public:
    static ParticleRenderQueue& getSingleton();

    bool create(u32 maxParticles = 262144);
    void shutdown();

    ParticleSystem& system();
    void setTexture(TextureHandle texture);
    void setTextureFile(const std::string& file);
    TextureHandle texture() const;
    const std::string& textureFile() const;

    // Called once per frame before any effect submits.
    void clear();

    void submitBurst(const ParticleSystem::Emitter& emitter, u32 count);
    void submitContinuous(const ParticleSystem::Emitter& emitter, f32 deltaTime);

private:
    ParticleRenderQueue() = default;

    ParticleSystem mSystem;
    std::string mTextureFile;
    bool mCreated = false;
};

ParticleRenderQueue& ParticleDraws();
RenderTechnique* createParticlePass();

} // namespace Radion

#endif // RADION_PARTICLE_PASS_H
