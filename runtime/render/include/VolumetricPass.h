#ifndef RADION_VOLUMETRIC_PASS_H
#define RADION_VOLUMETRIC_PASS_H

#include "FrameContext.h"
#include "OffscreenTarget.h"

namespace Radion
{

class PostProcessStack;
class Lighting;

// God rays, ported from docs/reference/particles/shaders/post_volumetric*.comp
// and vol_point/vol_rect - see VolumetricPass.cpp for which file each part
// came from. Runs after Forward and composites straight into the HDR scene
// colour, not through PostProcessStack's own chain: it reads the entity
// buffer, the shadow atlas and the cascade block the scene draw itself reads,
// which a plain image transform has no business knowing about.
class VolumetricPass final
{
public:
    bool setup();

    // `lighting` is where the point/rect proxies get the atlas-annotated
    // shadow data (matrixIndex/shadowAtlasMulAdd) for each light - the same
    // Lighting instance the Renderer already owns, not a second copy.
    void execute(FrameContext& frame, PostProcessStack& post, Lighting& lighting);
    void shutdown();

    // The sun and the spots use different shadow sources (cascades vs atlas)
    // and cost differently, so each source gets its own switch - matching the
    // reference, which keeps volumetricSunEnabled and volumetricSpotEnabled
    // independent for the same reason.
    bool sunEnabled = true;
    bool spotEnabled = true;
    bool pointEnabled = true;
    bool rectEnabled = true;
    bool blurEnabled = true;

    u32 samples = 16;
    f32 scattering = 0.7f; // Henyey-Greenstein g: 0 isotropic, ->1 forward
    f32 maxDistance = 2000.0f;
    f32 strength = 1.0f; // applied once, at the very end, to everything
    bool debugFallback = false;

    // Density shapes the raymarch itself (how thick the air reads); strength
    // is a plain multiplier applied when each source is combined - separate
    // controls because doubling density is not the same curve as doubling
    // strength once the samples are already summed.
    f32 sunDensity = 0.02f;
    f32 spotDensity = 0.04f;
    f32 spotStrength = 1.0f;
    f32 pointDensity = 0.05f;
    f32 pointStrength = 1.0f;
    f32 rectDensity = 0.05f;
    f32 rectStrength = 1.0f;

    // The proxy only decides which pixels pay for the raymarch - the maths
    // always uses the analytic range sphere, so this changes cost, not the
    // image. A cube is 12 triangles but covers ~1.9x the sphere's area.
    bool pointProxyIsCube = false;

private:
    bool ensurePipelines();
    bool resize(u32 halfWidth, u32 halfHeight);
    void runSun(const FrameContext& frame, PostProcessStack& post);
    void runSpot(const FrameContext& frame, PostProcessStack& post, Lighting& lighting);
    void runPoints(const FrameContext& frame, PostProcessStack& post, Lighting& lighting);
    void runRects(const FrameContext& frame, PostProcessStack& post, Lighting& lighting);
    void resolve(const FrameContext& frame, PostProcessStack& post);

    // Both half the frame's resolution, matching the reference's m_volA/
    // m_volB: the ray march is the expensive part and the result gets blurred
    // straight after, so full resolution would only cost more for no benefit.
    OffscreenTarget mAccumA;
    OffscreenTarget mAccumB;

    PipelineHandle mSunPipeline;
    PipelineHandle mSpotPipeline;
    PipelineHandle mAddPipeline;
    PipelineHandle mPointPipeline;
    PipelineHandle mRectPipeline;
    bool mPipelinesReady = false;
    bool mPipelinesFailed = false;

    BufferHandle mSettingsBlock;
    BufferHandle mSunBlock;
    BufferHandle mSpotBlock;
    BufferHandle mAddBlock;
    BufferHandle mProxyBlock;
    BufferHandle mPointBlock;
    BufferHandle mRectBlock;

    SamplerHandle mSampler;

    // Unit sphere (radius 1) and unit cube (-1..+1), the same proxy shapes
    // the reference builds once with MeshManager - scaled per light in
    // volumetric_proxy.vert rather than rebuilt per light.
    MeshHandle mSphereProxy;
    MeshHandle mCubeProxy;
};

} // namespace Radion

#endif // RADION_VOLUMETRIC_PASS_H
