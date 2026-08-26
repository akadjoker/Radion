#include "PCH.h"

#include "ParticleEffectPool.h"

#include "GameObject.h"
#include "Scene.h"

namespace Radion
{

namespace
{

GameObject* activatePooled(GameObject* object, const ParticleSystem::Emitter& emitter,
                         u32 burstCount, const Math::Vec3& position,
                         const Math::Vec3& direction)
{
    object->setActive(true);
    object->setName("ParticleEffect");
    object->setGlobalPosition(position);
    if (glm::dot(direction, direction) > 0.0001f)
        object->lookAt(position + direction);

    ParticleEffect* effect = object->getComponent<ParticleEffect>();
    if (!effect)
    {
        effect = object->addComponent<ParticleEffect>();
        if (!effect)
            return nullptr;
    }

    effect->setMode(ParticleEffectMode::OneShot);
    effect->setEmitter(emitter);
    effect->setBurstCount(burstCount);
    effect->setAutoDestroy(false); // pool owns lifetime
    effect->setUseOwnerDirection(direction != Math::Vec3(0.0f));
    effect->play();
    return object;
}

} // namespace

ParticleEffectPool& ParticleEffectPool::getSingleton()
{
    static ParticleEffectPool pool;
    return pool;
}

void ParticleEffectPool::initialize(Scene& scene)
{
    mScene = &scene;
}

void ParticleEffectPool::shutdown()
{
    // Objects owned by the scene are destroyed with it. We only clear our book-keeping.
    mAvailable.clear();
    mActive.clear();
    mScene = nullptr;
}

ParticleEffect* ParticleEffectPool::spawn(const ParticleSystem::Emitter& emitter, u32 burstCount,
                                          const Math::Vec3& position,
                                          const Math::Vec3& direction)
{
    if (!mScene)
        return nullptr;

    GameObject* object = nullptr;
    while (!mAvailable.empty() && !object)
    {
        GameObject* candidate = mAvailable.back();
        mAvailable.pop_back();
        if (!candidate || candidate->disposed())
            continue;
        object = candidate;
    }

    if (!object)
    {
        object = mScene->createGameObject("ParticleEffect");
        if (!object)
            return nullptr;
        ParticleEffect* effect = object->addComponent<ParticleEffect>();
        if (!effect)
        {
            mScene->destroy(object);
            return nullptr;
        }
    }

    if (!activatePooled(object, emitter, burstCount, position, direction))
        return nullptr;

    mActive.push_back(object);
    return object->getComponent<ParticleEffect>();
}

void ParticleEffectPool::reclaim()
{
    if (!mScene)
        return;

    for (usize i = 0; i < mActive.size();)
    {
        GameObject* object = mActive[i];
        bool remove = false;

        if (!object || object->disposed())
        {
            remove = true;
        }
        else
        {
            ParticleEffect* effect = object->findComponent<ParticleEffect>();
            if (!effect || effect->isFinished())
            {
                object->setActive(false);
                if (object->parent() && object->parent() != &mScene->root())
                    object->setGlobalPosition(mScene->root().globalPosition());
                mAvailable.push_back(object);
                remove = true;
            }
        }

        if (remove)
        {
            mActive[i] = mActive.back();
            mActive.pop_back();
        }
        else
        {
            ++i;
        }
    }
}

usize ParticleEffectPool::activeCount() const
{
    return mActive.size();
}

usize ParticleEffectPool::availableCount() const
{
    return mAvailable.size();
}

} // namespace Radion
