#ifndef RADION_SCENE_MANAGER_H
#define RADION_SCENE_MANAGER_H

#include "SceneSerializer.h"

namespace Radion
{
class Scene;
struct CascadeShadowSettings;
struct ShadowAtlasSettings;
class PostProcessStack;
class LensFlarePass;
class EnvironmentProbe;
class Lighting;
class VolumetricPass;
struct SkySettings;
struct RenderResolution;
class ParticleRenderQueue;

class SceneManager final
{
public:
    ~SceneManager();

    Scene* create();
    bool activate(Scene* scene);
    bool load(const std::string& path, SceneLoadResult& result);
    bool save(const std::string& path) const;
    void unload();

    // Set once by Engine; a null argument is simply not read/written for that scene section.
    void bindRenderSettings(CascadeShadowSettings* shadows, ShadowAtlasSettings* shadowAtlas,
                            PostProcessStack* postProcess, LensFlarePass* lensFlare = nullptr,
                            EnvironmentProbe* environmentProbe = nullptr,
                            Lighting* lighting = nullptr, VolumetricPass* volumetric = nullptr,
                            SkySettings* sky = nullptr, RenderResolution* resolution = nullptr,
                            ParticleRenderQueue* particles = nullptr);

    // The bundle save()/load() use, for callers with their own SceneSerializer.
    SceneRenderSettings renderSettings() const;

    Scene* active()
    {
        return mActive;
    }
    const Scene* active() const
    {
        return mActive;
    }

private:
    SceneSerializer mSerializer;
    Scene* mActive = nullptr;
    CascadeShadowSettings* mShadowSettings = nullptr;
    ShadowAtlasSettings* mShadowAtlasSettings = nullptr;
    PostProcessStack* mPostProcessSettings = nullptr;
    LensFlarePass* mLensFlareSettings = nullptr;
    EnvironmentProbe* mEnvironmentProbeSettings = nullptr;
    Lighting* mLightingSettings = nullptr;
    VolumetricPass* mVolumetricSettings = nullptr;
    SkySettings* mSkySettings = nullptr;
    RenderResolution* mRenderResolutionSettings = nullptr;
    ParticleRenderQueue* mParticleSettings = nullptr;
};
} // namespace Radion

#endif // RADION_SCENE_MANAGER_H
