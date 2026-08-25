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

// Ordered image-effect stack. Scene-dependent work such as SSAO and shadows
// remains a render technique; this class only transforms the completed image.
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
    TextureHandle computeSSAO(const glm::mat4& projection);
    void resolve(const Rect& destination, u32 windowWidth, u32 windowHeight);

    // Same chain as resolve(), but the final tone-map/gamma draw lands in an
    // internal LDR texture instead of the backbuffer, and that texture is
    // returned. For an embedded viewport (the editor) - the handle is
    // borrowed, owned here, and stays valid until the next resize or
    // shutdown(). Never sceneColor(): that is HDR linear and is not what
    // reaches the window.
    TextureHandle resolveToTexture(u32 outputIndex = 0, bool applyPostProcess = true);

    // The depth the prepass wrote this frame, same texture SSAO reads. What
    // the tiled light cull needs and has no other way to reach, since the
    // scene's offscreen target is otherwise private to this class.
    TextureHandle sceneDepth() const
    {
        return mScene.depth;
    }
    TextureHandle ssaoTexture() const
    {
        return mAO[0].color;
    }

    // The HDR colour Forward just wrote. VolumetricPass composites into this
    // directly, before the tone-mapped chain below ever sees it - light in
    // the air has to go through the same exposure curve as everything else,
    // not be added on top of an already-graded image.
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

    // The stack's own fullscreen-triangle draw, public so a technique that
    // wants the exact same separable blur bloom uses (modes 4/5) is not left
    // copying it. Mode numbers and what `options` means for each are only
    // defined in PostProcess.cpp's fragment source - see the comments there
    // before passing a new one.
    void draw(TextureHandle source, TextureHandle secondary, TargetHandle destination,
             const Viewport& viewport, u32 mode, const glm::vec4& options);

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
    // The blur is what makes SSAO usable: the raw pass takes a handful of
    // random samples per pixel and is pure noise without it. Off is for
    // seeing that, not for shipping.
    bool ssaoBlur = true;
    bool ssaoDebug = false;
    // HDR temporal resolve happens before Bloom/ToneMap.  It is intentionally
    // independent from the generic effect list: a history buffer has ordering
    // and lifetime requirements that a stateless post layer does not have.
    bool taaEnabled = false;
    f32 taaFeedback = 0.95f;
    f32 taaMotionFeedback = 0.85f;
    // How many standard deviations of the 3x3 neighbourhood the history is
    // allowed to sit outside. Lower rejects more history: less ghosting, more
    // of the jitter left unresolved.
    f32 taaClipWidth = 4.0f;
    // Unsharp amount applied to the resolved luma. Temporal accumulation is
    // softer than the raw image by construction; zero disables the correction
    // and anything past ~0.5 starts to ring on high-contrast edges.
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
    // The effect chain both resolve paths share, up to but not including the
    // final presentation draw. Returns the texture holding the result and,
    // through `displayEncoded`, whether a tone-map layer already gamma-encoded
    // it - the final draw needs to know which of its two modes to use.
    TextureHandle runLayers(bool& displayEncoded);

    OffscreenTarget mScene;
    OffscreenTarget mPing[2];
    OffscreenTarget mBloom[2];
    OffscreenTarget mAO[2];
    // Only ever created by resolveToTexture() - a build that never renders
    // to a texture (every demo) pays nothing for this.
    OffscreenTarget mResolved[2];
    OffscreenTarget mTAAHistory[3][2];
    PipelineHandle mPipeline;
    PipelineHandle mTAAPipeline;
    BufferHandle mUniform;
    BufferHandle mTAAUniform;
    SamplerHandle mSampler;
    std::vector<PostLayer> mLayers;
    glm::mat4 mProjection = glm::mat4(1.0f);
    glm::mat4 mInverseProjection = glm::mat4(1.0f);
    u32 mTemporalIndex = 0;
    bool mTemporalAAAllowed = true;
    u32 mTAARead[3] = {};
    bool mTAAHistoryValid[3] = {};
    bool mTAAPipelineFailed = false;
};

} // namespace Radion

#endif // RADION_POST_PROCESS_H
