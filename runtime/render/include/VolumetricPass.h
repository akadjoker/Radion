#ifndef RADION_VOLUMETRIC_PASS_H
#define RADION_VOLUMETRIC_PASS_H

#include "FrameContext.h"
#include "OffscreenTarget.h"

namespace Radion
{

class PostProcessStack;
class Lighting;

// God rays, after Forward, composited straight into the HDR scene colour rather than through PostProcessStack: it reads the entity buffer, shadow atlas and cascade block the scene draw reads.
// See VolumetricPass.cpp for the shader sources each part derives from.
class VolumetricPass final
{
public:
    bool setup();

    // `lighting` supplies the atlas-annotated shadow data (matrixIndex/shadowAtlasMulAdd) for point/rect proxies; it is the Renderer's own instance.
    void execute(FrameContext& frame, PostProcessStack& post, Lighting& lighting);
    void shutdown();

    // Sun and spots use different shadow sources (cascades vs atlas) and cost differently, so each source has its own switch.
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

    // Density shapes the raymarch (how thick the air reads); strength multiplies when sources are combined; not interchangeable once samples are summed.
    f32 sunDensity = 0.02f;
    f32 spotDensity = 0.04f;
    f32 spotStrength = 1.0f;
    f32 pointDensity = 0.05f;
    f32 pointStrength = 1.0f;
    f32 rectDensity = 0.05f;
    f32 rectStrength = 1.0f;

    // The proxy only decides which pixels pay for the raymarch (the maths uses the analytic range sphere).
    // A cube is 12 triangles but covers ~1.9x the sphere's area.
    bool pointProxyIsCube = false;

private:
    bool ensurePipelines();
    bool resize(u32 halfWidth, u32 halfHeight);
    void runSun(const FrameContext& frame, PostProcessStack& post);
    void runSpot(const FrameContext& frame, PostProcessStack& post, Lighting& lighting);
    void runPoints(const FrameContext& frame, PostProcessStack& post, Lighting& lighting);
    void runRects(const FrameContext& frame, PostProcessStack& post, Lighting& lighting);
    void resolve(const FrameContext& frame, PostProcessStack& post);

    // Half the frame resolution: the ray march is expensive and the result is blurred anyway.
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

    // Unit sphere (radius 1) and unit cube (-1..+1), scaled per light in volumetric_proxy.vert.
    MeshHandle mSphereProxy;
    MeshHandle mCubeProxy;
};

} // namespace Radion

#endif // RADION_VOLUMETRIC_PASS_H
