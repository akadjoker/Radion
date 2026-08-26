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

// The frame as data: an ordered list of techniques, executed in the order they
// were added. Adding a stage is adding an entry.
class Renderer
{
public:
    // The engine's standard frame. A demo calls this and gets the pipeline
    // without assembling it; new stages land here, not in every demo. These
    // techniques are created and owned by the Renderer.
    bool addDefaultPasses();

    // A technique the caller owns and must keep alive. Rejected (and not
    // kept) if setup() fails, so a broken one never reaches execute().
    bool add(RenderTechnique* technique);

    void execute(const FrameContext& frame);
    void executeShadows(ShadowCasterSource& casters, FrameContext& frame);
    void executeDepth(const FrameContext& frame);

    // Entity list + shadow atlas: independent of the scene's own depth, so it
    // can run alongside executeShadows(). Cull needs that depth, so it runs
    // separately once executeDepth() has produced it.
    void executeLightingPrepare(ShadowCasterSource& casters, FrameContext& frame,
                                bool renderShadows = true);
    void executeLightingCull(FrameContext& frame, TextureHandle sceneDepth, u32 screenWidth,
                             u32 screenHeight);

    // Feeds the decal list into Lighting's entity buffer and points the
    // frame at the decal texture arrays. Call after executeLightingPrepare()
    // and before executeLightingCull(), or the decals miss this frame's tile
    // cull.
    void submitDecals(FrameContext& frame);

    // God rays. Runs after the forward draw and composites straight into the
    // scene's HDR colour - see VolumetricPass.h for why it is not folded into
    // the post-process chain.
    void executeVolumetric(FrameContext& frame, PostProcessStack& post);

    // Sun lens flare. Same slot as Volumetric - after Forward, straight into
    // the HDR scene colour.
    void executeLensFlare(const FrameContext& frame, PostProcessStack& post);

    // The planar reflection every water surface reads, rendered from the
    // camera mirrored through the water plane and published under
    // kReflectionTargetName. Which plane comes from the ocean queue itself, so
    // a scene with an Ocean in it gets a reflection without asking: this used
    // to be every demo's own job - its own target, its own mirrored camera,
    // its own resize handling - which is how two demos ended up with two
    // slightly different versions of it.
    //
    // Runs before the scene's own draw, and fills in frame.reflectionViewProj
    // for the surfaces that project through it. Does nothing, cheaply, when
    // no surface this frame wants a reflection.
    bool executeReflection(ShadowCasterSource& casters, FrameContext& frame);
    const char* reflectionSource() const;

    // The refraction half of the same idea, published under
    // kRefractionTargetName: the scene from the ordinary camera but clipped to
    // the water's far side, so only what is actually submerged ends up in it.
    // A pass rather than a copy of the finished frame on purpose - a copy
    // carries every surface above the water too, and no amount of sampling
    // care removes something that is already in the texture.
    bool executeRefraction(ShadowCasterSource& casters, FrameContext& frame);

    // Fills the probe's cubemap by rendering the scene six times, once per
    // cube face - the reference's RefreshEnvProbes. Reuses the Forward and Sky
    // techniques rather than owning shaders of its own: a reflection that was
    // drawn by different code than the scene would not match it.
    //
    // `frame` is the main camera's frame, borrowed for the state that is the
    // same whichever way the camera points - the entity buffer, the shadow
    // atlas, the sky. Only the matrices and the target change per face.
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

    // Why the last frame carried no directional shadow, as text for a panel.
    // "rendering" when it did.
    const char* sunShadowStatus() const;

    // Blits the cascade array and/or the atlas straight into the backbuffer,
    // in the corners - see ShadowDebugView. Call after the frame's own colour
    // has been resolved there, or this draws under it instead of over it.
    void debugDrawShadows(bool showCascades, bool showAtlas, u32 windowWidth, u32 windowHeight);
    void debugDrawTexture(TextureHandle texture, bool isArray, u32 layer, TargetHandle target,
                          u32 width, u32 height,
                          const Math::Vec4& sourceRect = Math::Vec4(1.0f, 1.0f, 0.0f, 0.0f));
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

    // The capture's own list: built six times per capture from a cube-face
    // camera, and nothing else may be pointing at it while that happens.
    RenderList* mCaptureList = nullptr;

    // The reflection's own culled list, rebuilt every frame against the
    // mirrored view-projection - see executeReflection(). Kept across frames
    // only for its storage, the same way mCaptureList is.
    RenderList* mReflectionList = nullptr;
    const char* mReflectionSource = "none";
    const char* mLoggedReflectionSource = nullptr;
    std::string mReflectionMaterialName;
    u32 mReflectionMaterialFlags = 0;
    RenderList* mRefractionList = nullptr;

    // Half the frame's resolution, rebuilt whenever that changes - which is
    // what makes a window resize follow without anyone outside handling it.
    OffscreenTarget mReflection;

    // Full resolution, unlike the reflection: the surface samples this at its
    // own screen position, so anything less shows as a soft patch exactly
    // where the water is meant to be clearest.
    OffscreenTarget mRefraction;
};

} // namespace Radion

#endif // RADION_RENDERER_H
