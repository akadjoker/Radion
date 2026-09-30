#ifndef RADION_FORWARD_PASS_H
#define RADION_FORWARD_PASS_H

#include "EnvironmentBlock.h"
#include "RenderTechnique.h"

#include <vector>

namespace Radion
{

// The opaque colour pass: one instanced call per run of packets sharing mesh, submesh and material.
class ForwardPass final : public RenderTechnique
{
public:
    const char* name() const override
    {
        return "Forward";
    }

    bool setup() override;

    // Full-scene draw (Opaque, AlphaTest, Transparent) for passes standing in for the whole scene against one target: planar reflection, probe capture.
    void execute(const FrameContext& frame) override;

    // The main frame calls these two directly, with Sky, scene colour copy and Water/Ocean in between; see Renderer::execute().
    void executeOpaque(const FrameContext& frame);
    void executeTransparent(const FrameContext& frame);

    void shutdown() override;

private:
    void bindFrameState(const FrameContext& frame);

    // Grows in steps, never shrinks: buffers cannot resize in place, so regrowing per frame costs more than holding the peak.
    bool ensureInstanceCapacity(u32 instances);
    bool ensurePaletteCapacity(u32 matrices);
    void drawCategory(const FrameContext& frame, RenderCategory category);

    BufferHandle mCameraBuffer;
    BufferHandle mTemporalBuffer;
    BufferHandle mEnvironmentBuffer;
    // Frame-wide environment computed once in bindFrameState(); per-batch local-probe overrides patch this base.
    EnvironmentBlock mFrameEnvironment;
    BufferHandle mInstanceBuffer;
    BufferHandle mPaletteBuffer;
    TextureHandle mWhiteAO;
    TextureHandle mNeutral;
    TextureHandle mNeutralArray;
    TextureHandle mNeutralCube;
    // MaterialMirror's reflection texture/camera (see Material.h BindingMirrorReflection); kept here rather than reaching into WaterPass.
    BufferHandle mMirrorCameraBuffer;
    TextureHandle mMirrorFallback; // alpha 0: no capture yet, mix() picks none of it
    u32 mInstanceCapacity = 0;
    u32 mPaletteCapacity = 0;
    struct GPUInstance
    {
        Math::mat4 model;
        Math::mat4 prevModel;
        u32 paletteOffset;
        u32 prevPaletteOffset;
        u32 padding[2];
    };
    std::vector<GPUInstance> mGPUInstances;
    std::vector<Math::mat4> mPalettes;
};

} // namespace Radion

#endif // RADION_FORWARD_PASS_H
