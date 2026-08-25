#include "PCH.h"

#include "SceneManager.h"

#include "Scene.h"

namespace Radion
{
SceneManager::~SceneManager()
{
    unload();
}

Scene* SceneManager::create()
{
    Scene* scene = new Scene();
    activate(scene);
    return scene;
}

bool SceneManager::activate(Scene* scene)
{
    if (!scene)
        return false;
    if (scene != mActive)
    {
        delete mActive;
        mActive = scene;
    }
    return true;
}

bool SceneManager::load(const std::string& path, SceneLoadResult& result)
{
    const SceneRenderSettings settings = renderSettings();
    Scene* scene = new Scene();
    if (!mSerializer.load(path, *scene, result, &settings))
    {
        delete scene;
        return false;
    }
    activate(scene);
    return true;
}

bool SceneManager::save(const std::string& path) const
{
    const SceneRenderSettings settings = renderSettings();
    return mActive && mSerializer.save(*mActive, path, &settings);
}

SceneRenderSettings SceneManager::renderSettings() const
{
    return {mShadowSettings,     mShadowAtlasSettings,      mPostProcessSettings,
            mLensFlareSettings,  mEnvironmentProbeSettings, mLightingSettings,
            mVolumetricSettings, mSkySettings,              mRenderResolutionSettings,
            mParticleSettings};
}

void SceneManager::unload()
{
    delete mActive;
    mActive = nullptr;
}

void SceneManager::bindRenderSettings(CascadeShadowSettings* shadows,
                                      ShadowAtlasSettings* shadowAtlas,
                                      PostProcessStack* postProcess, LensFlarePass* lensFlare,
                                      EnvironmentProbe* environmentProbe, Lighting* lighting,
                                      VolumetricPass* volumetric, SkySettings* sky,
                                      RenderResolution* resolution, ParticleRenderQueue* particles)
{
    mShadowSettings = shadows;
    mShadowAtlasSettings = shadowAtlas;
    mPostProcessSettings = postProcess;
    mLensFlareSettings = lensFlare;
    mEnvironmentProbeSettings = environmentProbe;
    mLightingSettings = lighting;
    mVolumetricSettings = volumetric;
    mSkySettings = sky;
    mRenderResolutionSettings = resolution;
    mParticleSettings = particles;
}
} // namespace Radion
