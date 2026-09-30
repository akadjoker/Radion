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

    // Point-light shadow atlas tile: writes distance to `lightPosition` over `range` (see depth_point.frag) so SamplePointShadowAtlas can compare without knowing the face.
    void executePoint(const FrameContext& frame, const Math::vec3& lightPosition, f32 range,
                      f32 bias);

    // Same as execute() plus a depth bias via GPU::setDepthBias after each pipeline switch; per-call because the pipeline cache is shared with the bias-free prepass.
    void executeBiased(const FrameContext& frame, f32 biasSlope, f32 biasConstant,
                       bool cullFront = false);

    // Directional shadow variant: casters draw both faces and the vertex shader flattens geometry behind the far plane instead of clipping it.
    void executeShadow(const FrameContext& frame);

private:
    struct GPUInstance
    {
        Math::mat4 model;
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

    // One glMultiDrawElementsIndirect batch: submeshes sharing a mesh and depth pipeline. Kept between calls (once per cascade per category).
    struct DrawGroup
    {
        MeshHandle mesh;
        PipelineHandle pipeline;
        std::vector<IndirectCommand> commands;
    };

    // Rebuilds mInstances/mPalettes from the opaque packets; shared by execute() and executePoint().
    bool collectInstances(const FrameContext& frame, RenderCategory category);
    void drawCategory(const FrameContext& frame, RenderCategory category, f32 biasSlope,
                      f32 biasConstant, bool cullFront, bool pancake = false,
                      bool forceTwoSided = false);
    void drawPointCategory(const FrameContext& frame, RenderCategory category);
    // Finds or opens the group for this mesh/pipeline. Scans mGroupKeys because packets arrive sorted, so the last group answers nearly every call.
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
    std::vector<Math::mat4> mPalettes;
    std::vector<PipelineEntry> mPipelines;
    std::vector<PipelineEntry> mPointPipelines;
    // Grown, never shrunk: mGroupCount is how many the current call owns; the tail keeps its command capacity.
    std::vector<DrawGroup> mGroups;
    std::vector<u64> mGroupKeys;
    usize mGroupCount = 0;
    std::vector<IndirectCommand> mCommands;
};

} // namespace Radion

#endif // RADION_DEPTH_PASS_H
