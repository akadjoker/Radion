#ifndef RADION_GPU_H
#define RADION_GPU_H

#include "Containers.h"
#include "Types.h"

#include "Math.h"

namespace Radion
{

namespace Platform
{
class Window;
}

struct GPUCaps;

using BufferHandle = Handle<struct BufferTag>;
using TextureHandle = Handle<struct TextureTag>;
using SamplerHandle = Handle<struct SamplerTag>;
using ShaderHandle = Handle<struct ShaderTag>;
using PipelineHandle = Handle<struct PipelineTag>;
using TargetHandle = Handle<struct TargetTag>;
using QueryHandle = Handle<struct QueryTag>;

enum class Format : u8
{
    Unknown = 0,

    R8,
    RG8,
    RGBA8,
    RGBA8_sRGB,
    R16F,
    RG16F,
    RGBA16F,
    R32F,
    RG32F,
    RGB32F,
    RGBA32F,
    R11G11B10F,
    RGB10A2,
    R32U,
    RG32U,
    RGBA32U,

    // Block-compressed, uploaded pre-compressed from a DDS mip chain; never produced by the GPU.
    BC1_RGBA,      // DXT1, 4bpp, 1-bit alpha
    BC1_RGBA_sRGB,
    BC3_RGBA,      // DXT5, 8bpp, full alpha
    BC3_RGBA_sRGB,
    BC5_RG,        // ATI2/3Dc, 8bpp, two independent channels (normal maps)
    BC7_RGBA,      // BPTC, 8bpp, what a DX10-header DDS almost always carries
    BC7_RGBA_sRGB,

    Depth16,
    Depth24,
    Depth32F,
    Depth24Stencil8,
};

enum class AttribFormat : u8
{
    Float1,
    Float2,
    Float3,
    Float4,
    Byte4N,  // normalized to [-1,1]
    UByte4N, // normalized to [0,1]
    UByte4,
    Short2N,
    Short4N,
    UInt1,
    UInt4,
};

enum BufferUsage : u32
{
    BufferVertex = 1 << 0,
    BufferIndex = 1 << 1,
    BufferUniform = 1 << 2,
    BufferStorage = 1 << 3,
    BufferIndirect = 1 << 4,
    BufferStaging = 1 << 5,
    // Mapped once and left mapped: read through mappedData() with no driver call. For per-frame readback; readBuffer() synchronises.
    BufferReadback = 1 << 6,
};

// Static: written once.  Dynamic: rewritten now and then.
// Stream: rewritten every frame.
enum class Residency : u8
{
    Static,
    Dynamic,
    Stream
};

struct BufferDesc
{
    u64 size = 0;
    u32 usage = 0;
    Residency residency = Residency::Static;
    u32 stride = 0; // required for structured storage
    const void* data = nullptr;
    const char* debugName = nullptr;
};

enum class IndexType : u8
{
    U16,
    U32
};

enum class TextureType : u8
{
    Tex2D,
    Tex2DArray,
    Tex3D,
    TexCube
};

enum TextureUsage : u32
{
    TextureSampled = 1 << 0,
    TextureStorage = 1 << 1, // written by compute
    TextureTarget = 1 << 2,  // attachable to a TargetHandle
};

// One compressed mip level as it sits in a DDS file: already-encoded block
// data, uploaded as-is with no decoding or GPU-side mip generation.
struct CompressedMip
{
    const void* data = nullptr;
    u32 size = 0;
};

struct TextureDesc
{
    TextureType type = TextureType::Tex2D;
    Format format = Format::RGBA8;
    u32 width = 0;
    u32 height = 0;
    u32 depth = 1; // slices for Tex2DArray/Tex3D
    u32 mips = 1;  // 0 requests the full chain
    u32 samples = 1;
    u32 usage = TextureSampled;
    const void* data = nullptr;
    const char* debugName = nullptr;

    // Block-compressed formats: one pre-encoded entry per mip, uploaded verbatim; there is no source to generate mips from.
    const CompressedMip* compressedMips = nullptr;
    u32 compressedMipCount = 0;
};

enum class Filter : u8
{
    Point,
    Linear,
    Trilinear,
    Anisotropic
};
enum class Wrap : u8
{
    Repeat,
    Mirror,
    Clamp,
    Border
};

struct SamplerDesc
{
    Filter filter = Filter::Linear;
    Wrap wrapU = Wrap::Repeat;
    Wrap wrapV = Wrap::Repeat;
    Wrap wrapW = Wrap::Repeat;
    f32 anisotropy = 1.0f;
    f32 mipBias = 0.0f;
    f32 border[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool compare = false; // shadow comparison sampling
};

enum class ShaderStage : u8
{
    Vertex,
    Fragment,
    Geometry,
    Compute
};

struct ShaderSource
{
    const char* code = nullptr;
    u32 size = 0; // 0 = null terminated
    const char* name = nullptr;
};

enum class BlendMode : u8
{
    Opaque,
    Alpha,
    Additive,
    Multiply,
    PremultipliedAlpha,
    AddColors,
    SubtractColors
};
enum class CullMode : u8
{
    None,
    Back,
    Front
};
enum class Compare : u8
{
    Never,
    Less,
    Equal,
    LessEqual,
    Greater,
    NotEqual,
    GreaterEqual,
    Always
};
enum class Topology : u8
{
    Triangles,
    TriangleStrip,
    Lines,
    LineStrip,
    Points,
    Patches
};

struct BlendState
{
    BlendMode mode = BlendMode::Opaque;
    bool writeRGB = true;
    bool writeA = true;
};

struct DepthState
{
    bool test = true;
    bool write = true;
    Compare func = Compare::LessEqual;
};

enum class StencilOp : u8
{
    Keep,
    Zero,
    Replace,
    IncrementClamp,
    DecrementClamp,
    Invert,
    IncrementWrap,
    DecrementWrap
};

struct StencilState
{
    bool enabled = false;
    Compare compare = Compare::Always;
    StencilOp fail = StencilOp::Keep;
    StencilOp depthFail = StencilOp::Keep;
    StencilOp pass = StencilOp::Keep;
    u8 readMask = 0xFF;
    u8 writeMask = 0xFF;
};

struct RasterState
{
    CullMode cull = CullMode::Back;
    bool frontCCW = true;
    bool wireframe = false;
    f32 depthBias = 0.0f;
    f32 depthBiasSlope = 0.0f;
};

struct VertexAttrib
{
    u8 location = 0;
    u8 stream = 0;
    u16 offset = 0;
    AttribFormat format = AttribFormat::Float3;
};

struct VertexStream
{
    u16 stride = 0;
    bool perInstance = false;
};

struct VertexLayout
{
    static constexpr u32 MaxAttribs = 16;
    static constexpr u32 MaxStreams = 4;

    VertexAttrib attribs[MaxAttribs];
    VertexStream streams[MaxStreams];
    u8 attribCount = 0;
    u8 streamCount = 0;
};

struct PipelineDesc
{
    ShaderSource vs;
    ShaderSource fs;
    ShaderSource gs;
    ShaderSource cs; // set on its own this becomes a compute pipeline

    VertexLayout layout;
    BlendState blend;
    DepthState depth;
    StencilState stencil;
    RasterState raster;
    Topology topology = Topology::Triangles;
    u32 patchVertices = 3;

    const char* debugName = nullptr;
};

struct TargetAttachment
{
    TextureHandle texture;
    u32 mip = 0;
    u32 slice = 0;
};

struct TargetDesc
{
    static constexpr u32 MaxColors = 8;

    TargetAttachment colors[MaxColors];
    TargetAttachment depth;
    u8 colorCount = 0;
    const char* debugName = nullptr;
};

enum ClearBits : u32
{
    ClearColor = 1 << 0,
    ClearDepth = 1 << 1,
    ClearStencil = 1 << 2,
};

struct ClearValue
{
    u32 bits = 0;
    f32 color[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    f32 depth = 1.0f;
    u32 stencil = 0;
};

struct Viewport
{
    f32 x = 0.0f, y = 0.0f;
    f32 width = 0.0f, height = 0.0f;
    f32 minDepth = 0.0f, maxDepth = 1.0f;
};

struct Rect
{
    s32 x = 0, y = 0;
    s32 width = 0, height = 0;
};

struct DrawDesc
{
    BufferHandle vertexBuffers[VertexLayout::MaxStreams];
    u64 vertexOffsets[VertexLayout::MaxStreams] = {0, 0, 0, 0};
    u8 vertexBufferCount = 0;

    BufferHandle indexBuffer; // invalid = non-indexed draw
    IndexType indexType = IndexType::U32;
    u64 indexOffset = 0;

    u32 count = 0; // indices, or vertices when non-indexed
    u32 first = 0;
    s32 baseVertex = 0;
    u32 instanceCount = 1;
    u32 firstInstance = 0;
};

enum BarrierBits : u32
{
    BarrierStorage = 1 << 0,
    BarrierIndirect = 1 << 1,
    BarrierVertex = 1 << 2,
    BarrierIndex = 1 << 3,
    BarrierUniform = 1 << 4,
    BarrierTexture = 1 << 5,
    BarrierImageWrite = 1 << 6,
    BarrierAll = 0xFFFFFFFFu,
};

struct GPUStats
{
    u32 drawCalls = 0;
    u32 dispatches = 0;
    u32 pipelineSwitches = 0;
    u32 targetSwitches = 0;
    u32 textureBinds = 0; // actual glBindTextureUnit calls, redundant ones already skipped
    u32 triangles = 0;
    f32 gpuMilliseconds = 0.0f;
};

class GPU
{
public:
    virtual ~GPU() = default;

    static GPU* createOpenGL(Platform::Window& window);
    static void destroyDevice(GPU* gpu);

    // Deletes every live resource, dependents first, and marks the context gone. Not the destructor's job: it may run
    // after the context is gone, where deleting GL names is undefined. Call while the context lives. Idempotent.
    virtual void shutdown() = 0;

    // One device per process. Only valid between createOpenGL and destroyDevice; outside that it logs and aborts.
    // Cleanup and teardown paths must use tryGet().
    static GPU& getSingleton();

    // For cleanup/fallback paths that may run before createOpenGL or after destroyDevice. Returns null instead of asserting.
    static GPU* tryGet();
    static bool ready();

    virtual BufferHandle createBuffer(const BufferDesc& desc) = 0;
    virtual TextureHandle createTexture(const TextureDesc& desc) = 0;

    // Rebuilds the GPU object behind an issued handle in place: the handle (index and generation) is unchanged, so
    // copies held elsewhere see the new content. Lets an async loader swap a placeholder for the decoded image;
    // unlike updateTexture(), size/format/mip count may differ.
    virtual bool replaceTexture(TextureHandle handle, const TextureDesc& desc) = 0;

    virtual SamplerHandle createSampler(const SamplerDesc& desc) = 0;
    virtual PipelineHandle createPipeline(const PipelineDesc& desc) = 0;
    virtual TargetHandle createTarget(const TargetDesc& desc) = 0;

    // One GL_SAMPLES_PASSED query lives as long as its object. Only read the previous frame's result; never stall on the latest.
    virtual QueryHandle createQuery() = 0;
    virtual void destroy(QueryHandle) = 0;
    virtual void beginOcclusionQuery(QueryHandle) = 0;
    virtual void endOcclusionQuery() = 0;
    virtual bool queryResultAvailable(QueryHandle) const = 0;
    // Samples that passed the depth test; callers only compare against zero.
    virtual u32 queryResult(QueryHandle) const = 0;

    // Size, format and mip count of a live texture. False leaves `out` untouched.
    virtual bool textureInfo(TextureHandle, TextureDesc& out) const = 0;

    // The backend's own id for a texture, for outside libraries that bind it themselves (ImGui::Image). 0 for a dead handle.
    // Nothing inside the engine should need this; use bindTexture().
    virtual u32 nativeTextureId(TextureHandle) const = 0;

    // Reads a rectangle of depth back as floats, for picking only. SYNCHRONISES with the GPU: fine per click, not per frame.
    // False leaves `out` untouched.
    virtual bool readDepthPixels(TextureHandle, u32 x, u32 y, u32 width, u32 height, f32* out,
                                 u32 count) const = 0;

    // Reads an RGBA rectangle back as floats (four per pixel); backend converts RGBA8/half/HDR. Synchronous: one-off use, not per frame.
    virtual bool readColorPixels(TextureHandle, u32 x, u32 y, u32 width, u32 height, f32* out,
                                 u32 floatCount) const = 0;

    // Reads a byte range of a buffer back. SYNCHRONISES with the GPU like readDepthPixels; occasional diagnostics only.
    virtual bool readBuffer(BufferHandle, u64 offset, u64 size, void* out) const = 0;

    // CPU-visible pointer of a BufferReadback buffer, else null. Contents are whatever the GPU finished; stay a frame or two behind.
    virtual const void* mappedData(BufferHandle) const = 0;

    // Copies a query's result into `target` at `offsetBytes` ON THE GPU TIMELINE, with no synchronisation,
    // unlike queryResult(). Paired with a BufferReadback read a frame later, occlusion results cost no sync.
    virtual void resolveQuery(QueryHandle, BufferHandle target, u64 offsetBytes) = 0;

    virtual void destroy(BufferHandle) = 0;
    virtual void destroy(TextureHandle) = 0;
    virtual void destroy(SamplerHandle) = 0;
    virtual void destroy(PipelineHandle) = 0;
    virtual void destroy(TargetHandle) = 0;

    virtual void updateBuffer(BufferHandle, u64 offset, u64 size, const void* data) = 0;
    virtual void updateTexture(TextureHandle, u32 mip, u32 slice, u32 x, u32 y, u32 width,
                               u32 height, const void* data) = 0;
    virtual void generateMips(TextureHandle) = 0;

    // Direct write into a buffer range, valid only until the frame ends.
    // Returns nullptr when the buffer is neither Dynamic nor Stream.
    virtual void* mapWrite(BufferHandle, u64 offset, u64 size) = 0;
    virtual void unmap(BufferHandle) = 0;

    virtual void beginFrame() = 0;
    virtual void endFrame() = 0;
    virtual void present() = 0;

    virtual void setTarget(TargetHandle target, const ClearValue& clear = {}) = 0;

    // Clears only `rect` of `target` (setTarget's clear covers the whole target), for tiled targets like the shadow atlas.
    // Restores the prior scissor state.
    virtual void clearRegion(TargetHandle target, const Rect& rect, const ClearValue& clear) = 0;
    virtual void clearColorAttachment(TargetHandle target, u32 attachment, const f32 color[4]) = 0;

    virtual void setPipeline(PipelineHandle) = 0;
    virtual void setViewport(const Viewport&) = 0;
    virtual void setScissor(const Rect&) = 0;
    virtual void setScissorEnabled(bool) = 0;

    // Enables GL_CLIP_DISTANCE0; off by default. Only shaders drawn while on may write gl_ClipDistance[0]
    // (unwritten is undefined, not zero), so turn it off after the clipped pass.
    virtual void setClipDistanceEnabled(bool) = 0;
    virtual void setStencilRef(u32) = 0;
    virtual void setBlendFactor(f32 r, f32 g, f32 b, f32 a) = 0;

    // Overrides the bound pipeline's depth bias until the next setPipeline, letting one bias-free cached pipeline
    // serve both biased shadow and ordinary depth draws.
    virtual void setDepthBias(f32 slope, f32 constant) = 0;

    virtual void bindTexture(u32 slot, TextureHandle, SamplerHandle = {}) = 0;
    virtual void bindImage(u32 slot, TextureHandle, u32 mip, bool write) = 0;
    virtual void bindUniform(u32 slot, BufferHandle, u64 offset = 0, u64 size = 0) = 0;
    virtual void bindStorage(u32 slot, BufferHandle, u64 offset = 0, u64 size = 0) = 0;

    virtual void draw(const DrawDesc&) = 0;
    virtual void drawIndirect(const DrawDesc& base, BufferHandle args, u64 offset,
                              u32 drawCount = 1) = 0;
    virtual void drawIndirectCount(const DrawDesc& base, BufferHandle args, u64 argsOffset,
                                   BufferHandle count, u64 countOffset, u32 maxDraws) = 0;

    virtual void dispatch(u32 x, u32 y, u32 z) = 0;
    virtual void dispatchIndirect(BufferHandle args, u64 offset) = 0;
    virtual void barrier(u32 bits) = 0;

    virtual void copyBuffer(BufferHandle dst, u64 dstOffset, BufferHandle src, u64 srcOffset,
                            u64 size) = 0;
    virtual void blitTarget(TargetHandle dst, TargetHandle src, const Rect& dstRect,
                            const Rect& srcRect, bool depth = false) = 0;

    // Puts GL back to what code outside gpu/ expects: no program, no vertex
    // array, no sampler objects, fill mode, texture unit 0. The render target
    // is left alone, since that is what the caller wants drawn into.
    virtual void resetForExternal() = 0;

    // Forget every cached binding. Call it after code outside gpu/ has touched
    // GL directly, otherwise the next bind is skipped against stale state.
    virtual void invalidateState() = 0;

    virtual const GPUStats& stats() const = 0;
    virtual const GPUCaps& caps() const = 0;
    virtual void pushMarker(const char* name) = 0;
    virtual void popMarker() = 0;

protected:
    static void setSingleton(GPU* gpu);
};

// Wrap any block that hands GL to code outside gpu/ -- the ImGui backend, the
// legacy batch renderer, a profiler overlay.
class ExternalGLScope
{
public:
    explicit ExternalGLScope(GPU& gpu);
    ~ExternalGLScope();

    ExternalGLScope(const ExternalGLScope&) = delete;
    ExternalGLScope& operator=(const ExternalGLScope&) = delete;

private:
    GPU& mGpu;
};

} // namespace Radion

#endif // RADION_GPU_H
