#ifndef RADION_FORWARD_PASS_H
#define RADION_FORWARD_PASS_H

#include "EnvironmentBlock.h"
#include "RenderTechnique.h"

#include <vector>

namespace Radion
{

// The opaque colour pass. Draws one instanced call per run of packets that
// share a mesh, submesh and material - which the RenderList sort is what makes
// possible.
class ForwardPass final : public RenderTechnique
{
public:
    const char* name() const override
    {
        return "Forward";
    }

    bool setup() override;

    // Full-scene draw: Opaque, AlphaTest and Transparent in one call. Used
    // where the pass stands in for the whole scene against one target - the
    // planar reflection and the environment probe capture - and where the
    // colour behind a transparent surface does not matter because nothing
    // reads it back.
    void execute(const FrameContext& frame) override;

    // The main frame instead calls these two directly, with Sky, the scene
    // colour copy and Water/Ocean run in between - see Renderer::execute().
    // Transparent geometry composited before the sky is what finding 38 in
    // docs/review.md is about.
    void executeOpaque(const FrameContext& frame);
    void executeTransparent(const FrameContext& frame);

    void shutdown() override;

private:
    void bindFrameState(const FrameContext& frame);

    // Grows in steps and never shrinks: a buffer cannot be resized in place,
    // so growing means destroy and recreate, and doing that every frame as the
    // count wobbles would cost more than holding the peak.
    bool ensureInstanceCapacity(u32 instances);
    bool ensurePaletteCapacity(u32 matrices);
    void drawCategory(const FrameContext& frame, RenderCategory category);

    BufferHandle mCameraBuffer;
    BufferHandle mTemporalBuffer;
    BufferHandle mEnvironmentBuffer;
    // The frame-wide environment (sun/ambient/fog/time plus the single
    // default probe), computed once in bindFrameState() and reused as the
    // base every per-batch local-probe override in drawCategory() patches
    // instead of re-deriving from frame.list->lights() every batch.
    EnvironmentBlock mFrameEnvironment;
    BufferHandle mInstanceBuffer;
    BufferHandle mPaletteBuffer;
    TextureHandle mWhiteAO;
    TextureHandle mNeutral;
    TextureHandle mNeutralArray;
    TextureHandle mNeutralCube;
    // MaterialMirror's own uMirrorReflectionTex/ReflectionCamera - a Lit
    // material reading WaterPass's own binding slots (see Material.h's
    // BindingMirrorReflection doc), so this pass needs its own copies rather
    // than reaching into WaterPass's private ones.
    BufferHandle mMirrorCameraBuffer;
    TextureHandle mMirrorFallback; // alpha 0: no capture yet, mix() picks none of it
    u32 mInstanceCapacity = 0;
    u32 mPaletteCapacity = 0;
    struct GPUInstance
    {
        glm::mat4 model;
        glm::mat4 prevModel;
        u32 paletteOffset;
        u32 prevPaletteOffset;
        u32 padding[2];
    };
    std::vector<GPUInstance> mGPUInstances;
    std::vector<glm::mat4> mPalettes;
};

} // namespace Radion

#endif // RADION_FORWARD_PASS_H
