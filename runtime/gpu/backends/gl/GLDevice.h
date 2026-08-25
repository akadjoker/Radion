#ifndef RADION_GL_DEVICE_H
#define RADION_GL_DEVICE_H

#include "GPU.h"

#include <set>
#include "GPUCaps.h"

#include <glad.h>

namespace Radion
{

struct GLBuffer
{
    GLuint id = 0;
    u64 size = 0;
    u32 usage = 0;
    Residency residency = Residency::Static;
    bool mapped = false;
    // Non-null only for BufferReadback: the buffer is mapped once at
    // creation and stays mapped for its whole life, so reading it costs a
    // pointer dereference instead of a driver round trip.
    void* persistent = nullptr;
};

struct GLTexture
{
    GLuint id = 0;
    GLenum target = 0;
    Format format = Format::Unknown;
    u32 width = 0;
    u32 height = 0;
    u32 depth = 0;
    u32 mips = 0;
};

struct GLSampler
{
    GLuint id = 0;
};

struct GLPipeline
{
    GLuint program = 0;
    GLuint vao = 0;
    VertexLayout layout;
    BlendState blend;
    DepthState depth;
    StencilState stencil;
    RasterState raster;
    Topology topology = Topology::Triangles;
    u32 patchVertices = 3;
    bool compute = false;
};

struct GLTarget
{
    GLuint fbo = 0;
    u32 width = 0;
    u32 height = 0;
    u8 colorCount = 0;
};

struct GLQuery
{
    GLuint id = 0;
};

class GLDevice final : public GPU
{
public:
    static constexpr u32 MaxTextureSlots = 32;
    static constexpr u32 MaxBufferSlots = 16;

    explicit GLDevice(Platform::Window& window);
    ~GLDevice() override;

    bool initialize();
    void shutdown() override;

    BufferHandle createBuffer(const BufferDesc& desc) override;
    TextureHandle createTexture(const TextureDesc& desc) override;
    bool replaceTexture(TextureHandle handle, const TextureDesc& desc) override;
    SamplerHandle createSampler(const SamplerDesc& desc) override;
    PipelineHandle createPipeline(const PipelineDesc& desc) override;
    TargetHandle createTarget(const TargetDesc& desc) override;
    QueryHandle createQuery() override;

    bool textureInfo(TextureHandle handle, TextureDesc& out) const override;
    u32 nativeTextureId(TextureHandle handle) const override;
    bool readDepthPixels(TextureHandle handle, u32 x, u32 y, u32 width, u32 height, f32* out,
                         u32 count) const override;
    bool readColorPixels(TextureHandle handle, u32 x, u32 y, u32 width, u32 height, f32* out,
                         u32 floatCount) const override;
    bool readBuffer(BufferHandle handle, u64 offset, u64 size, void* out) const override;
    const void* mappedData(BufferHandle handle) const override;

    void destroy(BufferHandle handle) override;
    void destroy(TextureHandle handle) override;
    void destroy(SamplerHandle handle) override;
    void destroy(PipelineHandle handle) override;
    void destroy(TargetHandle handle) override;
    void destroy(QueryHandle handle) override;

    void beginOcclusionQuery(QueryHandle handle) override;
    void endOcclusionQuery() override;
    void resolveQuery(QueryHandle handle, BufferHandle target, u64 offsetBytes) override;
    bool queryResultAvailable(QueryHandle handle) const override;
    u32 queryResult(QueryHandle handle) const override;

    void updateBuffer(BufferHandle handle, u64 offset, u64 size, const void* data) override;
    void updateTexture(TextureHandle handle, u32 mip, u32 slice, u32 x, u32 y, u32 width,
                       u32 height, const void* data) override;
    void generateMips(TextureHandle handle) override;

    void* mapWrite(BufferHandle handle, u64 offset, u64 size) override;
    void unmap(BufferHandle handle) override;

    void beginFrame() override;
    void endFrame() override;
    void present() override;

    void setTarget(TargetHandle target, const ClearValue& clear) override;
    void clearRegion(TargetHandle target, const Rect& rect, const ClearValue& clear) override;
    void clearColorAttachment(TargetHandle target, u32 attachment, const f32 color[4]) override;
    void setPipeline(PipelineHandle handle) override;
    void setViewport(const Viewport& viewport) override;
    void setScissor(const Rect& rect) override;
    void setScissorEnabled(bool enabled) override;
    void setClipDistanceEnabled(bool enabled) override;
    void setStencilRef(u32 value) override;
    void setBlendFactor(f32 r, f32 g, f32 b, f32 a) override;
    void setDepthBias(f32 slope, f32 constant) override;

    void bindTexture(u32 slot, TextureHandle texture, SamplerHandle sampler) override;
    void bindImage(u32 slot, TextureHandle texture, u32 mip, bool write) override;
    void bindUniform(u32 slot, BufferHandle buffer, u64 offset, u64 size) override;
    void bindStorage(u32 slot, BufferHandle buffer, u64 offset, u64 size) override;

    void draw(const DrawDesc& desc) override;
    void drawIndirect(const DrawDesc& base, BufferHandle args, u64 offset, u32 drawCount) override;
    void drawIndirectCount(const DrawDesc& base, BufferHandle args, u64 argsOffset,
                           BufferHandle count, u64 countOffset, u32 maxDraws) override;

    void dispatch(u32 x, u32 y, u32 z) override;
    void dispatchIndirect(BufferHandle args, u64 offset) override;
    void barrier(u32 bits) override;

    void copyBuffer(BufferHandle dst, u64 dstOffset, BufferHandle src, u64 srcOffset,
                    u64 size) override;
    void blitTarget(TargetHandle dst, TargetHandle src, const Rect& dstRect, const Rect& srcRect,
                    bool depth) override;

    void resetForExternal() override;
    void invalidateState() override;

    const GPUStats& stats() const override;
    const GPUCaps& caps() const override;
    void pushMarker(const char* name) override;
    void popMarker() override;

private:
    static constexpr u32 TimerQueryCount = 4;
    GLuint compileShader(GLenum stage, const ShaderSource& source);
    bool linkProgram(GLuint program, const char* debugName);
    GLuint buildVertexArray(const VertexLayout& layout);

    // The whole body of createTexture() minus the pool insertion - shared
    // with replaceTexture(), which needs the same GL object built but
    // written into an existing pool slot instead of a new one.
    bool buildTexture(const TextureDesc& desc, GLTexture& outTexture);

    void applyBlend(const BlendState& state);
    void applyDepth(const DepthState& state);
    void applyStencil(const StencilState& state);
    void applyRaster(const RasterState& state);

    // A destroyed GL name can be handed straight back to the next create, so
    // anything still cached against it must be forgotten first.
    void forgetTexture(GLuint id);
    void forgetBuffer(GLuint id);

    bool bindDrawState(const DrawDesc& desc, GLPipeline*& pipeline);

    Platform::Window& mWindow;
    GPUCaps mCaps;
    GPUStats mStats;

    Pool<GLBuffer, BufferHandle> mBuffers;
    Pool<GLTexture, TextureHandle> mTextures;
    Pool<GLSampler, SamplerHandle> mSamplers;
    Pool<GLPipeline, PipelineHandle> mPipelines;
    Pool<GLTarget, TargetHandle> mTargets;
    Pool<GLQuery, QueryHandle> mQueries;

    PipelineHandle mCurrentPipeline;
#ifdef RADION_DEBUG
    std::set<u32> mValidatedPrograms;
#endif
    TargetHandle mCurrentTarget;
    GLuint mBoundProgram = 0;
    GLuint mBoundVao = 0;
    GLuint mBoundFbo = 0;

    GLuint mBoundTextures[MaxTextureSlots] = {};
    GLuint mBoundSamplers[MaxTextureSlots] = {};
    GLuint mBoundUniforms[MaxBufferSlots] = {};
    GLuint mBoundStorages[MaxBufferSlots] = {};

    BlendState mBlend;
    DepthState mDepth;
    StencilState mStencil;
    RasterState mRaster;
    u32 mStencilRef = 0;
    bool mStateKnown = false;
    bool mScissorEnabled = false;
    bool mClipDistanceEnabled = false;
    Rect mScissorRect;
    bool mInitialized = false;
    GLuint mTimerQueries[TimerQueryCount] = {};
    bool mTimerPending[TimerQueryCount] = {};
    u32 mTimerCursor = 0;
    s32 mActiveTimer = -1;
    f32 mLastGpuMilliseconds = 0.0f;
};

} // namespace Radion

#endif // RADION_GL_DEVICE_H
