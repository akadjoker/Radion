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

    // Set once by Engine, right after Renderer/Lighting/PostProcessStack
    // exist - what save()/load() pass through to SceneSerializer so a scene
    // file also carries shadow/post-process settings instead of just the
    // object graph. Any argument left null is simply not read/written for
    // that section; nothing here requires all three.
    void bindRenderSettings(CascadeShadowSettings* shadows, ShadowAtlasSettings* shadowAtlas,
                            PostProcessStack* postProcess, LensFlarePass* lensFlare = nullptr,
                            EnvironmentProbe* environmentProbe = nullptr,
                            Lighting* lighting = nullptr, VolumetricPass* volumetric = nullptr,
                            SkySettings* sky = nullptr, RenderResolution* resolution = nullptr,
                            ParticleRenderQueue* particles = nullptr);

    // The same bundle save()/load() build for themselves, handed out so a
    // caller with its own Scene/SceneSerializer (a standalone demo, not
    // going through this manager's own load()/save()) can still pass it to
    // SceneSerializer::load()/save() and have the scene file's post-process/
    // shadow/sky settings actually take effect instead of being silently
    // read and discarded.
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
