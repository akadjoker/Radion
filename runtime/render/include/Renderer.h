#ifndef RADION_RENDERER_H
#define RADION_RENDERER_H

#include "Decals.h"
#include "EnvironmentProbe.h"
#include "LensFlarePass.h"
#include "Lighting.h"
#include "OffscreenTarget.h"
#include "RenderTechnique.h"
#include "Shadows.h"
#include "VolumetricPass.h"

#include <vector>

namespace Radion
{

class DepthPass;
class ShadowPass;
class PostProcessStack;
class ShadowDebugView;

// The frame as data: techniques executed in the order added.
class Renderer
{
public:
    // The engine's standard frame; these techniques are created and owned by the Renderer.
    bool addDefaultPasses();

    // A technique the caller owns and must keep alive. Rejected (and not
    // kept) if setup() fails, so a broken one never reaches execute().
    bool add(RenderTechnique* technique);

    void execute(const FrameContext& frame);
    void executeShadows(ShadowCasterSource& casters, FrameContext& frame);
    void executeDepth(const FrameContext& frame);

    // Entity list + shadow atlas; independent of the scene depth, so it can run alongside executeShadows(). Cull needs that depth and runs after executeDepth().
    void executeLightingPrepare(ShadowCasterSource& casters, FrameContext& frame,
                                bool renderShadows = true);
    void executeLightingCull(FrameContext& frame, TextureHandle sceneDepth, u32 screenWidth,
                             u32 screenHeight);

    // Feeds the decal list into Lighting's entity buffer and points the frame at the decal arrays. Call between executeLightingPrepare() and executeLightingCull().
    void submitDecals(FrameContext& frame);

    // God rays: after the forward draw, composited into the scene's HDR colour (see VolumetricPass.h).
    void executeVolumetric(FrameContext& frame, PostProcessStack& post);

    void executeLensFlare(const FrameContext& frame, PostProcessStack& post);

    // Planar reflection for water, rendered from the camera mirrored through the water plane and published under kReflectionTargetName; the plane comes from the ocean queue.
    // Runs before the scene's draw and fills frame.reflectionViewProj. Does nothing, cheaply, when no surface wants a reflection.
    bool executeReflection(ShadowCasterSource& casters, FrameContext& frame);
    const char* reflectionSource() const;

    // Refraction, published under kRefractionTargetName: the scene clipped to the water's far side so only submerged geometry is in it. A pass rather than a frame copy, since a copy already contains surfaces above the water.
    bool executeRefraction(ShadowCasterSource& casters, FrameContext& frame);

    // Fills the probe's cubemap with six renders, one per face. Reuses the Forward and Sky techniques so reflections match the scene.
    // `frame` is the main camera's, borrowed for face-independent state (entity buffer, shadow atlas, sky); only matrices and target change per face.
    void captureEnvironment(EnvironmentProbe& probe, ShadowCasterSource& casters,
                            const FrameContext& frame);

    // Calls shutdown() on every technique, in reverse order of addition.
    void shutdown();

    RenderTechnique* find(const char* name) const;
    bool setEnabled(const char* name, bool enabled);

    usize count() const
    {
        return mTechniques.size();
    }
    RenderTechnique* at(usize index) const
    {
        return index < mTechniques.size() ? mTechniques[index] : nullptr;
    }

public:
    CascadeShadowSettings* cascadeSettings();
    f32 cascadeHalfExtent(u32 cascade) const;
    f32 cascadeSplit(u32 cascade) const;
    const RenderListStats* shadowListStats() const;

    // Why the last frame had no directional shadow, for a panel; "rendering" when it did.
    const char* sunShadowStatus() const;

    // Blits the cascade array/atlas into the backbuffer corners (see ShadowDebugView). Call after the frame colour is resolved there, or this draws under it.
    void debugDrawShadows(bool showCascades, bool showAtlas, u32 windowWidth, u32 windowHeight);
    void debugDrawTexture(TextureHandle texture, bool isArray, u32 layer, TargetHandle target,
                          u32 width, u32 height,
                          const Math::vec4& sourceRect = Math::vec4(1.0f, 1.0f, 0.0f, 0.0f));
    void debugDrawCubemap(TextureHandle texture, u32 face, u32 mip, TargetHandle target,
                          u32 width, u32 height);
    TextureHandle directionalShadowTexture() const;
    Lighting* lighting()
    {
        return &mLighting;
    }
    VolumetricPass* volumetric()
    {
        return &mVolumetric;
    }
    LensFlarePass* lensFlare()
    {
        return &mLensFlare;
    }
    DecalSystem* decals()
    {
        return &mDecals;
    }

private:
    std::vector<RenderTechnique*> mTechniques;
    std::vector<RenderTechnique*> mOwned;
    DepthPass* mDepth = nullptr;
    ShadowPass* mShadows = nullptr;
    ShadowDebugView* mShadowDebugView = nullptr;
    Lighting mLighting;
    VolumetricPass mVolumetric;
    LensFlarePass mLensFlare;
    DecalSystem mDecals;

    // Built six times per capture from a cube-face camera; nothing else may point at it meanwhile.
    RenderList* mCaptureList = nullptr;

    // Reflection's culled list, rebuilt every frame against the mirrored view-projection; kept across frames only for storage.
    RenderList* mReflectionList = nullptr;
    const char* mReflectionSource = "none";
    const char* mLoggedReflectionSource = nullptr;
    std::string mReflectionMaterialName;
    u32 mReflectionMaterialFlags = 0;
    RenderList* mRefractionList = nullptr;

    // Half the frame resolution, rebuilt when that changes.
    OffscreenTarget mReflection;

    // Full resolution: water samples this at its own screen position, so less shows as a soft patch.
    OffscreenTarget mRefraction;
};

} // namespace Radion

#endif // RADION_RENDERER_H
