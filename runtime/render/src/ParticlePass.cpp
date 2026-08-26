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

        // Extract camera basis vectors from the view matrix. The columns of an
        // orthonormal view matrix are the camera axes in world space.
        const Math::Mat3 viewRotation(frame.view);
        const Math::Vec3 cameraRight = glm::normalize(Math::Vec3(viewRotation[0][0],
                                                               viewRotation[1][0],
                                                               viewRotation[2][0]));
        const Math::Vec3 cameraUp = glm::normalize(Math::Vec3(viewRotation[0][1],
                                                            viewRotation[1][1],
                                                            viewRotation[2][1]));

        // TODO: separate alpha and additive draws if the effect system starts
        // tracking blend mode per effect. For now all particles draw with the
        // same pipeline selected by the single global flag below - additive,
        // since ParticleSystem::texture is a black-background glow map (the
        // whole Particles/ asset pack is), and alpha blend would paint that
        // black background as an opaque square.
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
    // The underlying ParticleSystem accumulates pending bursts in mPending and
    // tracks continuous emission state. Between frames we only need to discard
    // requests that were not consumed; update() already drains mPending, so
    // this is mainly defensive against submit calls after the pass has run.
    //
    // No explicit clear exists on ParticleSystem today; its update() drains the
    // pending vector. We leave it alone and rely on update() being called once
    // per frame.
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
