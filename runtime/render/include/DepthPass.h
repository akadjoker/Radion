#ifndef RADION_DEPTH_PASS_H
#define RADION_DEPTH_PASS_H

#include "RenderTechnique.h"

#include <vector>

namespace Radion
{

class DepthPass final : public RenderTechnique
{
public:
    const char* name() const override
    {
        return "Depth prepass";
    }

    bool setup() override;
    void execute(const FrameContext& frame) override;
    void shutdown() override;

    // A shadow atlas tile for a point light: writes distance to `lightPosition`
    // over `range` (see depth_point.frag) instead of ordinary perspective
    // depth, because six of these tiles share one straight 2D texture and the
    // shader that samples them (SamplePointShadowAtlas in lit.frag) needs a
    // value it can compare without knowing which face produced it.
    void executePoint(const FrameContext& frame, const glm::vec3& lightPosition, f32 range,
                      f32 bias);

    // Same draw as execute(), plus a depth bias applied through
    // GPU::setDepthBias after each pipeline switch. A separate entry point
    // rather than a defaulted parameter on execute(): the pipeline cache is
    // shared with the ordinary (bias-free) depth prepass, and the bias has to
    // be a per-call override on top of it, not baked into the pipeline.
    void executeBiased(const FrameContext& frame, f32 biasSlope, f32 biasConstant,
                       bool cullFront = false);

    // Directional shadow variant: casters draw both faces (or front faces
    // only when asked) and the vertex shader flattens geometry behind the
    // far plane instead of clipping it away.
    void executeShadow(const FrameContext& frame, f32 biasSlope, f32 biasConstant,
                       bool cullFront = false);

private:
    struct GPUInstance
    {
        glm::mat4 model;
        u32 paletteOffset;
        u32 padding[3];
    };

    struct PipelineEntry
    {
        VertexLayout layout;
        bool skinned = false;
        bool alphaTest = false;
        bool pancake = false;
        CullMode cull = CullMode::Back;
        PipelineHandle pipeline;
    };

    struct IndirectCommand
    {
        u32 count;
        u32 instanceCount;
        u32 firstIndex;
        s32 baseVertex;
        u32 baseInstance;
    };

    // One glMultiDrawElementsIndirect batch: every submesh sharing a mesh and
    // a depth pipeline. Kept alive between calls, commands and all - a shadow
    // frame runs this once per cascade per category.
    struct DrawGroup
    {
        MeshHandle mesh;
        PipelineHandle pipeline;
        std::vector<IndirectCommand> commands;
    };

    // Rebuilds mInstances/mPalettes from the list's opaque packets. Shared by
    // execute() and executePoint(): both draw the same geometry, only the
    // pipeline and the extra per-pass uniforms differ.
    bool collectInstances(const FrameContext& frame, RenderCategory category);
    void drawCategory(const FrameContext& frame, RenderCategory category, f32 biasSlope,
                      f32 biasConstant, bool cullFront, bool pancake = false,
                      bool forceTwoSided = false);
    void drawPointCategory(const FrameContext& frame, RenderCategory category);
    // Slot for this mesh/pipeline pair among the groups built so far, opening
    // a new one when there is none. mGroupKeys is scanned rather than mGroups
    // itself: the packets arrive sorted by pipeline then texture then mesh,
    // so the last group answers nearly every call, and a miss walks packed
    // keys instead of striding over the groups' own command vectors.
    DrawGroup& groupFor(MeshHandle mesh, PipelineHandle pipeline);
    bool ensureInstanceCapacity(u32 count);
    bool ensurePaletteCapacity(u32 count);
    bool ensureIndirectCapacity(u32 count);
    PipelineHandle pipelineFor(const VertexLayout& layout, bool skinned, bool alphaTest,
                               CullMode cull, bool pancake = false);
    PipelineHandle pointPipelineFor(const VertexLayout& layout, bool skinned, bool alphaTest,
                                    CullMode cull);

    BufferHandle mCameraBuffer;
    BufferHandle mPointDepthBuffer;
    BufferHandle mInstanceBuffer;
    BufferHandle mPaletteBuffer;
    BufferHandle mIndirectBuffer;
    u32 mInstanceCapacity = 0;
    u32 mPaletteCapacity = 0;
    u32 mIndirectCapacity = 0;
    std::vector<GPUInstance> mInstances;
    std::vector<glm::mat4> mPalettes;
    std::vector<PipelineEntry> mPipelines;
    std::vector<PipelineEntry> mPointPipelines;
    // Grown, never shrunk: mGroupCount says how many of mGroups the call in
    // progress owns, so the unused tail keeps its commands' capacity for the
    // next cascade instead of being freed with it.
    std::vector<DrawGroup> mGroups;
    std::vector<u64> mGroupKeys;
    usize mGroupCount = 0;
    std::vector<IndirectCommand> mCommands;
};

} // namespace Radion

#endif // RADION_DEPTH_PASS_H
