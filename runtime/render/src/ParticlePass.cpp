#include "PCH.h"

#include "ParticlePass.h"

#include "AssetManager.h"
#include "GPU.h"

namespace Radion
{

namespace
{

class ParticlePass final : public RenderTechnique
{
public:
    const char* name() const override
    {
        return "Particles";
    }

    bool setup() override
    {
        mReady = ParticleDraws().create();
        return mReady;
    }

    void execute(const FrameContext& frame) override
    {
        if (!mReady)
            return;

        ParticleSystem& system = ParticleDraws().system();

        GPU& gpu = GPU::getSingleton();
        gpu.setTarget(frame.target);
        gpu.setViewport(frame.viewport);

        system.update(frame.deltaTime);

        const Math::mat3 viewRotation(frame.view);
        const Math::vec3 cameraRight = Math::normalize(Math::vec3(viewRotation[0][0],
                                                               viewRotation[1][0],
                                                               viewRotation[2][0]));
        const Math::vec3 cameraUp = Math::normalize(Math::vec3(viewRotation[0][1],
                                                            viewRotation[1][1],
                                                            viewRotation[2][1]));

        // TODO: separate alpha/additive draws if effects track blend mode. All particles draw additive: ParticleSystem::texture is a black-background glow map.
        system.render(frame.viewProjection, cameraRight, cameraUp, true);
    }

    void shutdown() override
    {
        ParticleDraws().shutdown();
        mReady = false;
    }

private:
    bool mReady = false;
};

} // namespace

ParticleRenderQueue& ParticleRenderQueue::getSingleton()
{
    static ParticleRenderQueue queue;
    return queue;
}

bool ParticleRenderQueue::create(u32 maxParticles)
{
    if (mCreated)
        return true;
    mCreated = mSystem.create(maxParticles);
    return mCreated;
}

void ParticleRenderQueue::shutdown()
{
    if (!mCreated)
        return;
    mSystem.shutdown();
    mCreated = false;
}

ParticleSystem& ParticleRenderQueue::system()
{
    return mSystem;
}

void ParticleRenderQueue::setTexture(TextureHandle texture)
{
    mTextureFile.clear();
    mSystem.texture = texture;
}

void ParticleRenderQueue::setTextureFile(const std::string& file)
{
    mTextureFile = file;
    mSystem.texture = file.empty() ? TextureHandle() : Assets().loadTexture(file, ColorSpace::sRGB);
}

TextureHandle ParticleRenderQueue::texture() const
{
    return mSystem.texture;
}

const std::string& ParticleRenderQueue::textureFile() const
{
    return mTextureFile;
}

void ParticleRenderQueue::clear()
{
    // update() already drains mPending; ParticleSystem has no explicit clear, so rely on one update() per frame.
}

void ParticleRenderQueue::submitBurst(const ParticleSystem::Emitter& emitter, u32 count)
{
    if (mCreated)
        mSystem.burst(emitter, count);
}

void ParticleRenderQueue::submitContinuous(const ParticleSystem::Emitter& emitter, f32 deltaTime)
{
    if (mCreated)
        mSystem.emitContinuous(emitter, deltaTime);
}

ParticleRenderQueue& ParticleDraws()
{
    return ParticleRenderQueue::getSingleton();
}

RenderTechnique* createParticlePass()
{
    return new ParticlePass();
}

} // namespace Radion
