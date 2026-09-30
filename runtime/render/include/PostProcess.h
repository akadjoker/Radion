#ifndef RADION_POST_PROCESS_H
#define RADION_POST_PROCESS_H

#include "OffscreenTarget.h"
#include "FrameContext.h"

#include <vector>

namespace Radion
{

enum class PostEffect : u8
{
    ToneMap,
    Bloom,
    FXAA
};

enum class ToneMapMode : u8
{
    None,
    Reinhard,
    ACES
};

struct PostLayer
{
    PostEffect effect = PostEffect::ToneMap;
    bool enabled = true;
};

// Ordered image-effect stack; scene-dependent work (SSAO, shadows) stays a render technique, this only transforms the completed image.
class PostProcessStack
{
public:
    bool initialize();
    void shutdown();

    PostLayer& add(PostEffect effect);
    bool remove(PostEffect effect);
    bool setEnabled(PostEffect effect, bool enabled);
    bool isEnabled(PostEffect effect) const;
    bool move(usize from, usize to);
    void clear();

    bool begin(u32 width, u32 height, FrameContext& frame, u32 temporalIndex = 0,
               bool resetTemporalHistory = false);
    TextureHandle computeSSAO(const Math::mat4& projection);
    void resolve(const Rect& destination, u32 windowWidth, u32 windowHeight);

    // Same chain as resolve(), but the final tone-map/gamma draw lands in an internal LDR texture (borrowed; valid until resize or shutdown()). For embedded viewports.
    // Never sceneColor(): that is HDR linear.
    TextureHandle resolveToTexture(u32 outputIndex = 0, bool applyPostProcess = true);

    // This frame's prepass depth (same texture SSAO reads); the tiled light cull needs it.
    TextureHandle sceneDepth() const
    {
        return mScene.depth;
    }
    TextureHandle ssaoTexture() const
    {
        return mAO[0].color;
    }

    // The HDR colour Forward just wrote; VolumetricPass composites into it before tone mapping so light in the air goes through the same exposure curve.
    TextureHandle sceneColor() const
    {
        return mScene.color;
    }
    TextureHandle sceneVelocity() const
    {
        return mScene.velocity;
    }
    TextureHandle sceneReactive() const
    {
        return mScene.reactive;
    }
    u32 sceneWidth() const
    {
        return mScene.width;
    }
    u32 sceneHeight() const
    {
        return mScene.height;
    }

    // Fullscreen-triangle draw, public so techniques can reuse the bloom blur (modes 4/5). Mode numbers and `options` are defined only in PostProcess.cpp's fragment source.
    void draw(TextureHandle source, TextureHandle secondary, TargetHandle destination,
             const Viewport& viewport, u32 mode, const Math::vec4& options);

    f32 exposure = 1.0f;
    ToneMapMode toneMap = ToneMapMode::ACES;
    f32 bloomThreshold = 1.0f;
    f32 bloomSoftKnee = 0.5f;
    f32 bloomStrength = 0.35f;
    f32 fxaaSubpixel = 0.75f;
    f32 fxaaEdgeThreshold = 0.125f;
    f32 fxaaEdgeThresholdMin = 0.0312f;
    bool ssaoEnabled = false;
    f32 ssaoRadius = 2.0f;
    f32 ssaoBias = 0.05f;
    f32 ssaoIntensity = 1.0f;
    u32 ssaoSamples = 16;
    f32 ssaoDepthSigma = 80.0f;
    // The raw pass is pure noise without the blur; off is for seeing that.
    bool ssaoBlur = true;
    bool ssaoDebug = false;
    // HDR temporal resolve runs before Bloom/ToneMap; separate from the effect list because a history buffer has ordering and lifetime needs a stateless layer lacks.
    bool taaEnabled = false;
    f32 taaFeedback = 0.95f;
    f32 taaMotionFeedback = 0.85f;
    // Standard deviations of the 3x3 neighbourhood the history may sit outside; lower = less ghosting, more unresolved jitter.
    f32 taaClipWidth = 4.0f;
    // Unsharp amount on resolved luma (accumulation is softer); zero disables, past ~0.5 rings on high-contrast edges.
    f32 taaSharpness = 0.0f;
    bool enabled = false;

    const std::vector<PostLayer>& layers() const
    {
        return mLayers;
    }

private:
    bool resize(u32 width, u32 height);
    bool ensureTAAPipeline();
    TextureHandle resolveTAA(TextureHandle source);
    // Effect chain both resolve paths share, up to the final presentation draw. `displayEncoded` reports whether a tone-map layer already gamma-encoded the result.
    TextureHandle runLayers(bool& displayEncoded);

    OffscreenTarget mScene;
    OffscreenTarget mPing[2];
    OffscreenTarget mBloom[2];
    OffscreenTarget mAO[2];
    // Only created by resolveToTexture(); builds that never render to a texture pay nothing.
    OffscreenTarget mResolved[2];
    OffscreenTarget mTAAHistory[3][2];
    PipelineHandle mPipeline;
    PipelineHandle mTAAPipeline;
    BufferHandle mUniform;
    BufferHandle mTAAUniform;
    SamplerHandle mSampler;
    std::vector<PostLayer> mLayers;
    Math::mat4 mProjection = Math::mat4(1.0f);
    Math::mat4 mInverseProjection = Math::mat4(1.0f);
    u32 mTemporalIndex = 0;
    bool mTemporalAAAllowed = true;
    u32 mTAARead[3] = {};
    bool mTAAHistoryValid[3] = {};
    bool mTAAPipelineFailed = false;
};

} // namespace Radion

#endif // RADION_POST_PROCESS_H
