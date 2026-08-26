#include "PCH.h"

#include "GLDevice.h"

#include "GPUContext.h"
#include "Log.h"
#include "Window.h"

#include <cstring>

namespace Radion
{

namespace
{

struct FormatInfo
{
    GLenum internalFormat;
    GLenum layout;
    GLenum type;
    bool compressed = false;
};

#ifdef RADION_DEBUG
const char* shaderStageName(GLenum stage)
{
    switch (stage)
    {
    case GL_VERTEX_SHADER:
        return "vertex";
    case GL_FRAGMENT_SHADER:
        return "fragment";
    case GL_GEOMETRY_SHADER:
        return "geometry";
    case GL_COMPUTE_SHADER:
        return "compute";
    default:
        return "shader";
    }
}

const char* formatName(Format format)
{
    switch (format)
    {
    case Format::R8:
        return "R8";
    case Format::RG8:
        return "RG8";
    case Format::RGBA8:
        return "RGBA8";
    case Format::RGBA8_sRGB:
        return "RGBA8_sRGB";
    case Format::R16F:
        return "R16F";
    case Format::RG16F:
        return "RG16F";
    case Format::RGBA16F:
        return "RGBA16F";
    case Format::R32F:
        return "R32F";
    case Format::RG32F:
        return "RG32F";
    case Format::RGB32F:
        return "RGB32F";
    case Format::RGBA32F:
        return "RGBA32F";
    case Format::R11G11B10F:
        return "R11G11B10F";
    case Format::RGB10A2:
        return "RGB10A2";
    case Format::R32U:
        return "R32U";
    case Format::RG32U:
        return "RG32U";
    case Format::RGBA32U:
        return "RGBA32U";
    case Format::BC1_RGBA:
        return "BC1_RGBA";
    case Format::BC1_RGBA_sRGB:
        return "BC1_RGBA_sRGB";
    case Format::BC3_RGBA:
        return "BC3_RGBA";
    case Format::BC3_RGBA_sRGB:
        return "BC3_RGBA_sRGB";
    case Format::BC5_RG:
        return "BC5_RG";
    case Format::BC7_RGBA:
        return "BC7_RGBA";
    case Format::BC7_RGBA_sRGB:
        return "BC7_RGBA_sRGB";
    case Format::Depth16:
        return "Depth16";
    case Format::Depth24:
        return "Depth24";
    case Format::Depth32F:
        return "Depth32F";
    case Format::Depth24Stencil8:
        return "Depth24Stencil8";
    default:
        return "Unknown";
    }
}
#endif

FormatInfo formatInfo(Format format)
{
    switch (format)
    {
    case Format::R8:
        return {GL_R8, GL_RED, GL_UNSIGNED_BYTE};
    case Format::RG8:
        return {GL_RG8, GL_RG, GL_UNSIGNED_BYTE};
    case Format::RGBA8:
        return {GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE};
    case Format::RGBA8_sRGB:
        return {GL_SRGB8_ALPHA8, GL_RGBA, GL_UNSIGNED_BYTE};
    case Format::R16F:
        return {GL_R16F, GL_RED, GL_HALF_FLOAT};
    case Format::RG16F:
        return {GL_RG16F, GL_RG, GL_HALF_FLOAT};
    case Format::RGBA16F:
        return {GL_RGBA16F, GL_RGBA, GL_HALF_FLOAT};
    case Format::R32F:
        return {GL_R32F, GL_RED, GL_FLOAT};
    case Format::RG32F:
        return {GL_RG32F, GL_RG, GL_FLOAT};
    case Format::RGB32F:
        return {GL_RGB32F, GL_RGB, GL_FLOAT};
    case Format::RGBA32F:
        return {GL_RGBA32F, GL_RGBA, GL_FLOAT};
    case Format::R11G11B10F:
        return {GL_R11F_G11F_B10F, GL_RGB, GL_UNSIGNED_INT_10F_11F_11F_REV};
    case Format::RGB10A2:
        return {GL_RGB10_A2, GL_RGBA, GL_UNSIGNED_INT_2_10_10_10_REV};
    case Format::R32U:
        return {GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT};
    case Format::RG32U:
        return {GL_RG32UI, GL_RG_INTEGER, GL_UNSIGNED_INT};
    case Format::RGBA32U:
        return {GL_RGBA32UI, GL_RGBA_INTEGER, GL_UNSIGNED_INT};
    case Format::BC1_RGBA:
        return {GL_COMPRESSED_RGBA_S3TC_DXT1_EXT, 0, 0, true};
    case Format::BC1_RGBA_sRGB:
        return {GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT, 0, 0, true};
    case Format::BC3_RGBA:
        return {GL_COMPRESSED_RGBA_S3TC_DXT5_EXT, 0, 0, true};
    case Format::BC3_RGBA_sRGB:
        return {GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT, 0, 0, true};
    case Format::BC5_RG:
        return {GL_COMPRESSED_RG_RGTC2, 0, 0, true};
    case Format::BC7_RGBA:
        return {GL_COMPRESSED_RGBA_BPTC_UNORM, 0, 0, true};
    case Format::BC7_RGBA_sRGB:
        return {GL_COMPRESSED_SRGB_ALPHA_BPTC_UNORM, 0, 0, true};
    case Format::Depth16:
        return {GL_DEPTH_COMPONENT16, GL_DEPTH_COMPONENT, GL_UNSIGNED_SHORT};
    case Format::Depth24:
        return {GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT};
    case Format::Depth32F:
        return {GL_DEPTH_COMPONENT32F, GL_DEPTH_COMPONENT, GL_FLOAT};
    case Format::Depth24Stencil8:
        return {GL_DEPTH24_STENCIL8, GL_DEPTH_STENCIL, GL_UNSIGNED_INT_24_8};
    case Format::Unknown:
        break;
    }
    return {0, 0, 0};
}

struct AttribInfo
{
    GLint components;
    GLenum type;
    GLboolean normalized;
    bool integer;
};

AttribInfo attribInfo(AttribFormat format)
{
    switch (format)
    {
    case AttribFormat::Float1:
        return {1, GL_FLOAT, GL_FALSE, false};
    case AttribFormat::Float2:
        return {2, GL_FLOAT, GL_FALSE, false};
    case AttribFormat::Float3:
        return {3, GL_FLOAT, GL_FALSE, false};
    case AttribFormat::Float4:
        return {4, GL_FLOAT, GL_FALSE, false};
    case AttribFormat::Byte4N:
        return {4, GL_BYTE, GL_TRUE, false};
    case AttribFormat::UByte4N:
        return {4, GL_UNSIGNED_BYTE, GL_TRUE, false};
    case AttribFormat::UByte4:
        return {4, GL_UNSIGNED_BYTE, GL_FALSE, true};
    case AttribFormat::Short2N:
        return {2, GL_SHORT, GL_TRUE, false};
    case AttribFormat::Short4N:
        return {4, GL_SHORT, GL_TRUE, false};
    case AttribFormat::UInt1:
        return {1, GL_UNSIGNED_INT, GL_FALSE, true};
    case AttribFormat::UInt4:
        return {4, GL_UNSIGNED_INT, GL_FALSE, true};
    }
    return {4, GL_FLOAT, GL_FALSE, false};
}

GLenum compareEnum(Compare compare)
{
    switch (compare)
    {
    case Compare::Never:
        return GL_NEVER;
    case Compare::Less:
        return GL_LESS;
    case Compare::Equal:
        return GL_EQUAL;
    case Compare::LessEqual:
        return GL_LEQUAL;
    case Compare::Greater:
        return GL_GREATER;
    case Compare::NotEqual:
        return GL_NOTEQUAL;
    case Compare::GreaterEqual:
        return GL_GEQUAL;
    case Compare::Always:
        return GL_ALWAYS;
    }
    return GL_LEQUAL;
}

GLenum stencilOpEnum(StencilOp operation)
{
    switch (operation)
    {
    case StencilOp::Zero:
        return GL_ZERO;
    case StencilOp::Replace:
        return GL_REPLACE;
    case StencilOp::IncrementClamp:
        return GL_INCR;
    case StencilOp::DecrementClamp:
        return GL_DECR;
    case StencilOp::Invert:
        return GL_INVERT;
    case StencilOp::IncrementWrap:
        return GL_INCR_WRAP;
    case StencilOp::DecrementWrap:
        return GL_DECR_WRAP;
    case StencilOp::Keep:
    default:
        return GL_KEEP;
    }
}

GLenum topologyEnum(Topology topology)
{
    switch (topology)
    {
    case Topology::Triangles:
        return GL_TRIANGLES;
    case Topology::TriangleStrip:
        return GL_TRIANGLE_STRIP;
    case Topology::Lines:
        return GL_LINES;
    case Topology::LineStrip:
        return GL_LINE_STRIP;
    case Topology::Points:
        return GL_POINTS;
    case Topology::Patches:
        return GL_PATCHES;
    }
    return GL_TRIANGLES;
}

GLenum wrapEnum(Wrap wrap)
{
    switch (wrap)
    {
    case Wrap::Repeat:
        return GL_REPEAT;
    case Wrap::Mirror:
        return GL_MIRRORED_REPEAT;
    case Wrap::Clamp:
        return GL_CLAMP_TO_EDGE;
    case Wrap::Border:
        return GL_CLAMP_TO_BORDER;
    }
    return GL_REPEAT;
}

GLenum textureTargetEnum(TextureType type)
{
    switch (type)
    {
    case TextureType::Tex2D:
        return GL_TEXTURE_2D;
    case TextureType::Tex2DArray:
        return GL_TEXTURE_2D_ARRAY;
    case TextureType::Tex3D:
        return GL_TEXTURE_3D;
    case TextureType::TexCube:
        return GL_TEXTURE_CUBE_MAP;
    }
    return GL_TEXTURE_2D;
}

u32 mipCountFor(u32 width, u32 height)
{
    u32 size = width > height ? width : height;
    u32 levels = 1;
    while (size > 1)
    {
        size >>= 1;
        ++levels;
    }
    return levels;
}

// Rejects a descriptor before a single GL call is made from it. None of
// these used to be checked: a zero dimension, a mip count past what the
// dimensions could ever produce, a sample count on a type that is not
// Tex2D (createTexture forced GL_TEXTURE_2D_MULTISAMPLE onto it regardless
// of what desc.type asked for), or a compressed mip count past the storage
// that was actually allocated - each one reached a GL call with a
// parameter GL was never going to accept, or an upload that wrote past the
// mips the texture has.
bool validateTextureDesc(const TextureDesc& desc, std::string& error)
{
    if (desc.width == 0 || desc.height == 0)
    {
        error = "zero width/height";
        return false;
    }
    if ((desc.type == TextureType::Tex2DArray || desc.type == TextureType::Tex3D) &&
        desc.depth == 0)
    {
        error = "zero depth for an array/3D texture";
        return false;
    }
    if (desc.samples > 1 && desc.type != TextureType::Tex2D)
    {
        error = "multisampling is only supported for Tex2D";
        return false;
    }
    const u32 maxMips = mipCountFor(desc.width, desc.height);
    if (desc.mips > maxMips)
    {
        error = "mips (" + std::to_string(desc.mips) + ") exceeds the chain " +
                std::to_string(desc.width) + "x" + std::to_string(desc.height) + " can have (" +
                std::to_string(maxMips) + ")";
        return false;
    }
    if (desc.compressedMips && desc.compressedMipCount > 0)
    {
        const u32 storageMips = desc.mips ? desc.mips : maxMips;
        if (desc.compressedMipCount > storageMips)
        {
            error = "compressedMipCount (" + std::to_string(desc.compressedMipCount) +
                    ") exceeds the texture's own mip count (" + std::to_string(storageMips) + ")";
            return false;
        }
    }
    return true;
}

GLbitfield barrierBitsToGL(u32 bits)
{
    if (bits == BarrierAll)
        return GL_ALL_BARRIER_BITS;

    GLbitfield result = 0;
    if (bits & BarrierStorage)
        result |= GL_SHADER_STORAGE_BARRIER_BIT;
    if (bits & BarrierIndirect)
        result |= GL_COMMAND_BARRIER_BIT;
    if (bits & BarrierVertex)
        result |= GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT;
    if (bits & BarrierIndex)
        result |= GL_ELEMENT_ARRAY_BARRIER_BIT;
    if (bits & BarrierUniform)
        result |= GL_UNIFORM_BARRIER_BIT;
    if (bits & BarrierTexture)
        result |= GL_TEXTURE_FETCH_BARRIER_BIT;
    if (bits & BarrierImageWrite)
        result |= GL_SHADER_IMAGE_ACCESS_BARRIER_BIT;
    return result;
}

usize indexSize(IndexType type)
{
    return type == IndexType::U16 ? 2u : 4u;
}

GLenum indexTypeEnum(IndexType type)
{
    return type == IndexType::U16 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT;
}

} // namespace

GLDevice::GLDevice(Platform::Window& window) : mWindow(window)
{
}

GLDevice::~GLDevice()
{
    // No GL here on purpose - see GPU::shutdown(). Reaching this with the
    // device still up means nobody called it, so the leak is reported rather
    // than papered over by deleting names at an uncontrolled moment.
    if (mInitialized)
    {
        Log::warning("GPU: device destroyed without shutdown(); %zu buffers, %zu textures, "
                     "%zu samplers, %zu pipelines, %zu targets still live",
                     mBuffers.liveCount(), mTextures.liveCount(), mSamplers.liveCount(),
                     mPipelines.liveCount(), mTargets.liveCount());
    }
}

void GLDevice::shutdown()
{
    if (!mInitialized)
        return;

    if (mCaps.timerQuery)
        glDeleteQueries(TimerQueryCount, mTimerQueries);

    // Dependency order: a target references textures and a pipeline owns a
    // VAO that references buffer bindings, so both go before the resources
    // underneath them.
    mTargets.forEach(
        [](GLTarget& target)
        {
            glDeleteFramebuffers(1, &target.fbo);
        });
    mPipelines.forEach(
        [](GLPipeline& pipeline)
        {
            if (pipeline.vao)
                glDeleteVertexArrays(1, &pipeline.vao);
            glDeleteProgram(pipeline.program);
        });
    mSamplers.forEach(
        [](GLSampler& sampler)
        {
            glDeleteSamplers(1, &sampler.id);
        });
    mTextures.forEach(
        [](GLTexture& texture)
        {
            glDeleteTextures(1, &texture.id);
        });
    mBuffers.forEach(
        [](GLBuffer& buffer)
        {
            glDeleteBuffers(1, &buffer.id);
        });
    mQueries.forEach(
        [](GLQuery& query)
        {
            glDeleteQueries(1, &query.id);
        });

    mTargets.clear();
    mPipelines.clear();
    mSamplers.clear();
    mTextures.clear();
    mBuffers.clear();
    mQueries.clear();

    invalidateState();
    mCurrentPipeline = PipelineHandle();
    mCurrentTarget = TargetHandle();

    shutdownGPUContext();
    mInitialized = false;
}

bool GLDevice::initialize()
{
    if (mInitialized)
        return true;

    if (!initializeGPUContext(mWindow, mCaps))
        return false;

    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);

    // Off by default: GL_CLIP_DISTANCE0 active with a vertex shader that
    // never writes gl_ClipDistance[0] is undefined, not the harmless zero it
    // used to be assumed to be. The reflection capture - the only pass that
    // clips - turns this on around itself and off again; see
    // Renderer::executeReflection().
    mClipDistanceEnabled = false;

    if (mCaps.timerQuery)
        glGenQueries(TimerQueryCount, mTimerQueries);

    mInitialized = true;
    return true;
}

// ------------------------------------------------------------------ buffers

BufferHandle GLDevice::createBuffer(const BufferDesc& desc)
{
    if (desc.size == 0)
    {
        Log::error("GPU: createBuffer with zero size");
        return BufferHandle();
    }
    GLBuffer buffer;
    glCreateBuffers(1, &buffer.id);

    if (desc.usage & BufferReadback)
    {
        // Coherent as well as persistent: without it every read would need an
        // explicit glMemoryBarrier and a fence to be well defined, and the
        // point of this buffer is that reading it costs nothing.
        constexpr GLbitfield flags =
            GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT;
        glNamedBufferStorage(buffer.id, static_cast<GLsizeiptr>(desc.size), desc.data, flags);
        buffer.persistent = glMapNamedBufferRange(buffer.id, 0,
                                                  static_cast<GLsizeiptr>(desc.size), flags);
        if (!buffer.persistent)
            Log::error("GPU: readback buffer could not be mapped");
    }
    else if (desc.residency == Residency::Stream)
    {
        glNamedBufferData(buffer.id, static_cast<GLsizeiptr>(desc.size), desc.data,
                          GL_STREAM_DRAW);
    }
    else
    {
        GLbitfield flags = 0;
        if (desc.residency != Residency::Static)
            flags |= GL_DYNAMIC_STORAGE_BIT | GL_MAP_WRITE_BIT;
        glNamedBufferStorage(buffer.id, static_cast<GLsizeiptr>(desc.size), desc.data, flags);
    }

    buffer.size = desc.size;
    buffer.usage = desc.usage;
    buffer.residency = desc.residency;

    if (desc.debugName && mCaps.debugOutput)
        glObjectLabel(GL_BUFFER, buffer.id, -1, desc.debugName);

    return mBuffers.add(buffer);
}

bool GLDevice::textureInfo(TextureHandle handle, TextureDesc& out) const
{
    const GLTexture* texture = mTextures.get(handle);
    if (!texture)
        return false;

    out.format = texture->format;
    out.width = texture->width;
    out.height = texture->height;
    out.depth = texture->depth;
    out.mips = texture->mips;
    return true;
}

u32 GLDevice::nativeTextureId(TextureHandle handle) const
{
    const GLTexture* texture = mTextures.get(handle);
    return texture ? static_cast<u32>(texture->id) : 0u;
}

bool GLDevice::readDepthPixels(TextureHandle handle, u32 x, u32 y, u32 width, u32 height,
                               f32* out, u32 count) const
{
    const GLTexture* texture = mTextures.get(handle);
    if (!texture || !out || width == 0 || height == 0)
        return false;
    if (texture->format != Format::Depth24 && texture->format != Format::Depth24Stencil8 &&
        texture->format != Format::Depth32F)
        return false;
    // Readback coordinates routinely come from a caller doing its own
    // window/render-target math (a mouse pick, e.g.) - u64 here, not the u32
    // the parameters arrive as, because `width * height` and `x + width` can
    // each overflow before ever reaching the comparison that was supposed to
    // reject them.
    if (static_cast<u64>(count) < static_cast<u64>(width) * height)
        return false;
    if (x > texture->width || width > texture->width - x || y > texture->height ||
        height > texture->height - y)
        return false;

    // glGetTextureSubImage is GL 4.5: it reads straight from the texture,
    // without binding it to an FBO or touching any of the state a frame in
    // flight is relying on.
    glGetTextureSubImage(texture->id, 0, static_cast<GLint>(x), static_cast<GLint>(y), 0,
                         static_cast<GLsizei>(width), static_cast<GLsizei>(height), 1,
                         GL_DEPTH_COMPONENT, GL_FLOAT,
                         static_cast<GLsizei>(count * sizeof(f32)), out);
    return true;
}

bool GLDevice::readColorPixels(TextureHandle handle, u32 x, u32 y, u32 width, u32 height,
                               f32* out, u32 floatCount) const
{
    const GLTexture* texture = mTextures.get(handle);
    if (!texture || !out || width == 0 || height == 0)
        return false;
    if (texture->format != Format::RGBA8 && texture->format != Format::RGBA8_sRGB &&
        texture->format != Format::RGBA16F && texture->format != Format::RGBA32F)
        return false;
    const u64 required = static_cast<u64>(width) * height * 4;
    if (static_cast<u64>(floatCount) < required)
        return false;
    if (x > texture->width || width > texture->width - x || y > texture->height ||
        height > texture->height - y)
        return false;

    // Requesting float output makes the readback representation independent
    // of the target's storage format (including RGBA8 and RGBA16F).
    glGetTextureSubImage(texture->id, 0, static_cast<GLint>(x), static_cast<GLint>(y), 0,
                         static_cast<GLsizei>(width), static_cast<GLsizei>(height), 1,
                         GL_RGBA, GL_FLOAT,
                         static_cast<GLsizei>(required * sizeof(f32)), out);
    return true;
}

bool GLDevice::readBuffer(BufferHandle handle, u64 offset, u64 size, void* out) const
{
    const GLBuffer* buffer = mBuffers.get(handle);
    if (!buffer || !out || offset > buffer->size || size > buffer->size - offset)
        return false;
    glGetNamedBufferSubData(buffer->id, static_cast<GLintptr>(offset),
                            static_cast<GLsizeiptr>(size), out);
    return true;
}

void GLDevice::destroy(BufferHandle handle)
{
    GLBuffer buffer;
    if (!mBuffers.remove(handle, buffer))
        return;
    if (!isGPUContextAlive())
        return;

    // Unmapped before deletion. Deleting a mapped buffer is legal in GL, but
    // leaving the mapping to be torn down implicitly is how a stale pointer
    // outlives the thing it pointed into.
    if (buffer.persistent)
        glUnmapNamedBuffer(buffer.id);
    forgetBuffer(buffer.id);
    glDeleteBuffers(1, &buffer.id);
}

void GLDevice::updateBuffer(BufferHandle handle, u64 offset, u64 size, const void* data)
{
    GLBuffer* buffer = mBuffers.get(handle);
    if (!buffer)
        return;

    if (buffer->residency == Residency::Static)
    {
        Log::error("GPU: updateBuffer on a Static buffer");
        return;
    }
    if (!data || offset > buffer->size || size > buffer->size - offset)
    {
        Log::error("GPU: updateBuffer out of range");
        return;
    }

    if (buffer->residency == Residency::Stream && offset == 0)
    {
        glNamedBufferData(buffer->id, static_cast<GLsizeiptr>(buffer->size), nullptr,
                          GL_STREAM_DRAW);
    }
    glNamedBufferSubData(buffer->id, static_cast<GLintptr>(offset), static_cast<GLsizeiptr>(size),
                         data);
}

void* GLDevice::mapWrite(BufferHandle handle, u64 offset, u64 size)
{
    GLBuffer* buffer = mBuffers.get(handle);
    if (!buffer || buffer->residency == Residency::Static || buffer->mapped)
        return nullptr;
    if (offset > buffer->size || size > buffer->size - offset)
        return nullptr;

    GLbitfield access = GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_RANGE_BIT;
    void* pointer = glMapNamedBufferRange(buffer->id, static_cast<GLintptr>(offset),
                                          static_cast<GLsizeiptr>(size), access);
    buffer->mapped = pointer != nullptr;
    return pointer;
}

void GLDevice::unmap(BufferHandle handle)
{
    GLBuffer* buffer = mBuffers.get(handle);
    if (!buffer || !buffer->mapped)
        return;

    glUnmapNamedBuffer(buffer->id);
    buffer->mapped = false;
}

void GLDevice::copyBuffer(BufferHandle dst, u64 dstOffset, BufferHandle src, u64 srcOffset,
                          u64 size)
{
    GLBuffer* target = mBuffers.get(dst);
    GLBuffer* source = mBuffers.get(src);
    if (!target || !source)
        return;

    if (srcOffset > source->size || size > source->size - srcOffset || dstOffset > target->size ||
        size > target->size - dstOffset)
    {
        Log::error("GPU: copyBuffer out of range");
        return;
    }

    glCopyNamedBufferSubData(source->id, target->id, static_cast<GLintptr>(srcOffset),
                             static_cast<GLintptr>(dstOffset), static_cast<GLsizeiptr>(size));
}

// ----------------------------------------------------------------- textures

TextureHandle GLDevice::createTexture(const TextureDesc& desc)
{
    GLTexture texture;
    if (!buildTexture(desc, texture))
        return TextureHandle();
    return mTextures.add(texture);
}

bool GLDevice::replaceTexture(TextureHandle handle, const TextureDesc& desc)
{
    GLTexture* slot = mTextures.get(handle);
    if (!slot)
        return false;

    GLTexture texture;
    if (!buildTexture(desc, texture))
        return false;

    // The old GL object is gone the instant the new one is built - a
    // placeholder is only ever 1x1, so there is no useful window where
    // keeping both alive would matter, and dropping it first would leave
    // `handle` briefly pointing at nothing if buildTexture() below failed
    // and this returned early.
    if (slot->id)
    {
        forgetTexture(slot->id);
        glDeleteTextures(1, &slot->id);
    }
    *slot = texture;
    return true;
}

bool GLDevice::buildTexture(const TextureDesc& desc, GLTexture& outTexture)
{
    const FormatInfo info = formatInfo(desc.format);
    if (info.internalFormat == 0)
    {
        Log::error("GPU: createTexture with an unknown format");
        return false;
    }

    std::string error;
    if (!validateTextureDesc(desc, error))
    {
        Log::error("GPU: createTexture('%s') rejected - %s",
                  desc.debugName ? desc.debugName : "?", error.c_str());
        return false;
    }

    GLTexture texture;
    texture.target = textureTargetEnum(desc.type);
    texture.format = desc.format;
    texture.width = desc.width;
    texture.height = desc.height;
    texture.depth = desc.depth;
    texture.mips = desc.mips ? desc.mips : mipCountFor(desc.width, desc.height);

    const bool multisample = desc.samples > 1;
    if (multisample)
    {
        texture.target = GL_TEXTURE_2D_MULTISAMPLE;
        texture.mips = 1;
    }

    glCreateTextures(texture.target, 1, &texture.id);

    if (multisample)
    {
        glTextureStorage2DMultisample(texture.id, static_cast<GLsizei>(desc.samples),
                                      info.internalFormat, static_cast<GLsizei>(desc.width),
                                      static_cast<GLsizei>(desc.height), GL_TRUE);
    }
    else if (desc.type == TextureType::Tex2D || desc.type == TextureType::TexCube)
    {
        glTextureStorage2D(texture.id, static_cast<GLsizei>(texture.mips), info.internalFormat,
                           static_cast<GLsizei>(desc.width), static_cast<GLsizei>(desc.height));
    }
    else
    {
        glTextureStorage3D(texture.id, static_cast<GLsizei>(texture.mips), info.internalFormat,
                           static_cast<GLsizei>(desc.width), static_cast<GLsizei>(desc.height),
                           static_cast<GLsizei>(desc.depth));
    }

    if (info.compressed && desc.compressedMips && desc.compressedMipCount > 0 && !multisample)
    {
        // Every level already comes encoded straight from the DDS mip chain -
        // glGenerateTextureMipmap cannot rebuild a compressed level, and the
        // file already carries the ones GPU sampling needs.
        u32 mipWidth = desc.width;
        u32 mipHeight = desc.height;
        for (u32 mip = 0; mip < desc.compressedMipCount; ++mip)
        {
            const CompressedMip& level = desc.compressedMips[mip];
            glCompressedTextureSubImage2D(texture.id, static_cast<GLint>(mip), 0, 0,
                                          static_cast<GLsizei>(mipWidth > 0 ? mipWidth : 1),
                                          static_cast<GLsizei>(mipHeight > 0 ? mipHeight : 1),
                                          info.internalFormat, static_cast<GLsizei>(level.size),
                                          level.data);
            mipWidth = mipWidth > 1 ? mipWidth / 2 : 1;
            mipHeight = mipHeight > 1 ? mipHeight / 2 : 1;
        }
    }
    else if (desc.data && !multisample)
    {
        if (desc.type == TextureType::Tex2D)
        {
            glTextureSubImage2D(texture.id, 0, 0, 0, static_cast<GLsizei>(desc.width),
                                static_cast<GLsizei>(desc.height), info.layout, info.type,
                                desc.data);
        }
        else
        {
            glTextureSubImage3D(texture.id, 0, 0, 0, 0, static_cast<GLsizei>(desc.width),
                                static_cast<GLsizei>(desc.height), static_cast<GLsizei>(desc.depth),
                                info.layout, info.type, desc.data);
        }

        if (texture.mips > 1)
            glGenerateTextureMipmap(texture.id);
    }

    // Filtering normally comes from a sampler object, but a texture bound
    // without one falls back to its own parameters - and GL's defaults are
    // GL_NEAREST_MIPMAP_LINEAR with a maximum level of 1000, which leaves any
    // texture without a full mip chain incomplete. Sampling an incomplete
    // texture fails the whole draw, so give every texture parameters that are
    // consistent with what it actually has.
    if (!multisample)
    {
        glTextureParameteri(texture.id, GL_TEXTURE_MIN_FILTER,
                            texture.mips > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
        glTextureParameteri(texture.id, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTextureParameteri(texture.id, GL_TEXTURE_MAX_LEVEL,
                            static_cast<GLint>(texture.mips - 1));
    }

    if (desc.debugName && mCaps.debugOutput)
        glObjectLabel(GL_TEXTURE, texture.id, -1, desc.debugName);

#ifdef RADION_DEBUG
    Log::info("GPU: texture '%s' %ux%ux%u %s, %u mip(s), %s", desc.debugName ? desc.debugName : "?",
              texture.width, texture.height, texture.depth ? texture.depth : 1,
              formatName(desc.format), texture.mips, desc.data ? "uploaded" : "empty");
#endif

    outTexture = texture;
    return true;
}

void GLDevice::destroy(TextureHandle handle)
{
    GLTexture texture;
    if (!mTextures.remove(handle, texture))
        return;
    if (!isGPUContextAlive())
        return;

    forgetTexture(texture.id);
    glDeleteTextures(1, &texture.id);
}

void GLDevice::updateTexture(TextureHandle handle, u32 mip, u32 slice, u32 x, u32 y, u32 width,
                             u32 height, const void* data)
{
    GLTexture* texture = mTextures.get(handle);
    if (!texture || !data || width == 0 || height == 0)
        return;

    // None of this used to be checked: a mip past what the texture was
    // created with, or a region that runs past that mip's own (halved,
    // floored to 1) dimensions, reached glTextureSubImage2D/3D as a plain
    // GL_INVALID_VALUE at best - silently dropped without the debug context
    // on - or wrote into memory the driver had not allocated at worst.
    if (texture->target == GL_TEXTURE_2D_MULTISAMPLE || mip >= texture->mips)
        return;
    const u32 mipWidth = texture->width >> mip ? texture->width >> mip : 1u;
    const u32 mipHeight = texture->height >> mip ? texture->height >> mip : 1u;
    if (x > mipWidth || width > mipWidth - x || y > mipHeight || height > mipHeight - y)
        return;
    if (texture->target != GL_TEXTURE_2D)
    {
        const u32 sliceCount = texture->target == GL_TEXTURE_CUBE_MAP
                                   ? 6u
                                   : (texture->depth ? texture->depth : 1u);
        if (slice >= sliceCount)
            return;
    }

    const FormatInfo info = formatInfo(texture->format);
    if (texture->target == GL_TEXTURE_2D)
    {
        glTextureSubImage2D(texture->id, static_cast<GLint>(mip), static_cast<GLint>(x),
                            static_cast<GLint>(y), static_cast<GLsizei>(width),
                            static_cast<GLsizei>(height), info.layout, info.type, data);
    }
    else
    {
        glTextureSubImage3D(texture->id, static_cast<GLint>(mip), static_cast<GLint>(x),
                            static_cast<GLint>(y), static_cast<GLint>(slice),
                            static_cast<GLsizei>(width), static_cast<GLsizei>(height), 1,
                            info.layout, info.type, data);
    }
}

void GLDevice::generateMips(TextureHandle handle)
{
    GLTexture* texture = mTextures.get(handle);
    if (texture && texture->mips > 1)
        glGenerateTextureMipmap(texture->id);
}

// ----------------------------------------------------------------- samplers

SamplerHandle GLDevice::createSampler(const SamplerDesc& desc)
{
    GLSampler sampler;
    glCreateSamplers(1, &sampler.id);

    GLenum minFilter = GL_LINEAR_MIPMAP_LINEAR;
    GLenum magFilter = GL_LINEAR;
    switch (desc.filter)
    {
    case Filter::Point:
        minFilter = GL_NEAREST;
        magFilter = GL_NEAREST;
        break;
    case Filter::Linear:
        minFilter = GL_LINEAR;
        magFilter = GL_LINEAR;
        break;
    case Filter::Trilinear:
    case Filter::Anisotropic:
        minFilter = GL_LINEAR_MIPMAP_LINEAR;
        magFilter = GL_LINEAR;
        break;
    }

    glSamplerParameteri(sampler.id, GL_TEXTURE_MIN_FILTER, static_cast<GLint>(minFilter));
    glSamplerParameteri(sampler.id, GL_TEXTURE_MAG_FILTER, static_cast<GLint>(magFilter));
    glSamplerParameteri(sampler.id, GL_TEXTURE_WRAP_S, static_cast<GLint>(wrapEnum(desc.wrapU)));
    glSamplerParameteri(sampler.id, GL_TEXTURE_WRAP_T, static_cast<GLint>(wrapEnum(desc.wrapV)));
    glSamplerParameteri(sampler.id, GL_TEXTURE_WRAP_R, static_cast<GLint>(wrapEnum(desc.wrapW)));
    glSamplerParameterf(sampler.id, GL_TEXTURE_LOD_BIAS, desc.mipBias);
    glSamplerParameterfv(sampler.id, GL_TEXTURE_BORDER_COLOR, desc.border);

    if (desc.filter == Filter::Anisotropic && mCaps.anisotropicFilter)
    {
        f32 amount = desc.anisotropy > mCaps.maxAnisotropy ? mCaps.maxAnisotropy : desc.anisotropy;
        glSamplerParameterf(sampler.id, GL_TEXTURE_MAX_ANISOTROPY, amount);
    }

    if (desc.compare)
    {
        glSamplerParameteri(sampler.id, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
        glSamplerParameteri(sampler.id, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    }

    return mSamplers.add(sampler);
}

void GLDevice::destroy(SamplerHandle handle)
{
    GLSampler sampler;
    if (!mSamplers.remove(handle, sampler))
        return;
    if (!isGPUContextAlive())
        return;

    for (u32 i = 0; i < MaxTextureSlots; ++i)
    {
        if (mBoundSamplers[i] == sampler.id)
            mBoundSamplers[i] = 0;
    }
    glDeleteSamplers(1, &sampler.id);
}

// ---------------------------------------------------------------- pipelines

GLuint GLDevice::compileShader(GLenum stage, const ShaderSource& source)
{
    if (!source.code)
        return 0;

    GLuint shader = glCreateShader(stage);
    const GLint length = source.size ? static_cast<GLint>(source.size) : -1;
    if (length > 0)
        glShaderSource(shader, 1, &source.code, &length);
    else
        glShaderSource(shader, 1, &source.code, nullptr);
    glCompileShader(shader);

    GLint status = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (status == GL_FALSE)
    {
        char log[2048];
        GLsizei written = 0;
        glGetShaderInfoLog(shader, sizeof(log), &written, log);
        Log::error("GPU: shader %s failed to compile\n%s", source.name ? source.name : "?", log);
        glDeleteShader(shader);
        return 0;
    }

#ifdef RADION_DEBUG
    Log::info("GPU: compiled %s '%s' (%d bytes)", shaderStageName(stage),
              source.name ? source.name : "?",
              length > 0 ? static_cast<int>(length) : static_cast<int>(std::strlen(source.code)));
#endif

    return shader;
}

bool GLDevice::linkProgram(GLuint program, const char* debugName)
{
    glLinkProgram(program);

    GLint status = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &status);
    if (status == GL_FALSE)
    {
        char log[2048];
        GLsizei written = 0;
        glGetProgramInfoLog(program, sizeof(log), &written, log);
        Log::error("GPU: program %s failed to link\n%s", debugName ? debugName : "?", log);
        return false;
    }

#ifdef RADION_DEBUG
    Log::info("GPU: linked program '%s'", debugName ? debugName : "?");
#endif

    return true;
}

// Neither attribCount nor streamCount is bounded by its own type (u8, so up
// to 255) against the fixed arrays they index - VertexLayout::MaxAttribs
// (16) and MaxStreams (4). A caller-built layout with either set too high
// used to read straight past those C arrays in buildVertexArray() below,
// before a single GL call - undefined behaviour on the CPU, not a GL error
// the debug callback would ever see. Each attrib's own stream is checked
// against streamCount for the same reason: glVertexArrayAttribBinding()
// only silently ignores an invalid one, but nothing stopped attrib.stream
// from indexing past MaxStreams somewhere else callers rely on the layout
// for (ForwardPass's per-stream stride lookup, e.g.).
bool validateVertexLayout(const VertexLayout& layout, std::string& error)
{
    if (layout.attribCount > VertexLayout::MaxAttribs)
    {
        error = "attribCount (" + std::to_string(layout.attribCount) + ") exceeds MaxAttribs (" +
                std::to_string(VertexLayout::MaxAttribs) + ")";
        return false;
    }
    if (layout.streamCount > VertexLayout::MaxStreams)
    {
        error = "streamCount (" + std::to_string(layout.streamCount) +
                ") exceeds MaxStreams (" + std::to_string(VertexLayout::MaxStreams) + ")";
        return false;
    }
    for (u8 i = 0; i < layout.attribCount; ++i)
    {
        if (layout.attribs[i].stream >= layout.streamCount)
        {
            error = "attrib " + std::to_string(i) + " references stream " +
                    std::to_string(layout.attribs[i].stream) + " but streamCount is " +
                    std::to_string(layout.streamCount);
            return false;
        }
    }
    return true;
}

GLuint GLDevice::buildVertexArray(const VertexLayout& layout)
{
    GLuint vao = 0;
    glCreateVertexArrays(1, &vao);

    for (u8 i = 0; i < layout.attribCount; ++i)
    {
        const VertexAttrib& attrib = layout.attribs[i];
        const AttribInfo info = attribInfo(attrib.format);

        glEnableVertexArrayAttrib(vao, attrib.location);
        if (info.integer)
        {
            glVertexArrayAttribIFormat(vao, attrib.location, info.components, info.type,
                                       attrib.offset);
        }
        else
        {
            glVertexArrayAttribFormat(vao, attrib.location, info.components, info.type,
                                      info.normalized, attrib.offset);
        }
        glVertexArrayAttribBinding(vao, attrib.location, attrib.stream);
    }

    for (u8 i = 0; i < layout.streamCount; ++i)
        glVertexArrayBindingDivisor(vao, i, layout.streams[i].perInstance ? 1u : 0u);

    return vao;
}

PipelineHandle GLDevice::createPipeline(const PipelineDesc& desc)
{
    GLPipeline pipeline;
    pipeline.compute = desc.cs.code != nullptr;

    if (!pipeline.compute)
    {
        std::string error;
        if (!validateVertexLayout(desc.layout, error))
        {
            Log::error("GPU: createPipeline('%s') rejected - %s",
                      desc.debugName ? desc.debugName : "?", error.c_str());
            return PipelineHandle();
        }
    }

    GLuint vs = 0;
    GLuint fs = 0;
    GLuint gs = 0;
    GLuint cs = 0;

    if (pipeline.compute)
    {
        cs = compileShader(GL_COMPUTE_SHADER, desc.cs);
        if (!cs)
            return PipelineHandle();
    }
    else
    {
        vs = compileShader(GL_VERTEX_SHADER, desc.vs);
        fs = compileShader(GL_FRAGMENT_SHADER, desc.fs);
        if (!vs || !fs)
        {
            if (vs)
                glDeleteShader(vs);
            if (fs)
                glDeleteShader(fs);
            return PipelineHandle();
        }
        if (desc.gs.code)
        {
            gs = compileShader(GL_GEOMETRY_SHADER, desc.gs);
            if (!gs)
            {
                glDeleteShader(vs);
                glDeleteShader(fs);
                return PipelineHandle();
            }
        }
    }

    pipeline.program = glCreateProgram();
    if (cs)
        glAttachShader(pipeline.program, cs);
    if (vs)
        glAttachShader(pipeline.program, vs);
    if (fs)
        glAttachShader(pipeline.program, fs);
    if (gs)
        glAttachShader(pipeline.program, gs);

    const bool linked = linkProgram(pipeline.program, desc.debugName);

    if (cs)
        glDeleteShader(cs);
    if (vs)
        glDeleteShader(vs);
    if (fs)
        glDeleteShader(fs);
    if (gs)
        glDeleteShader(gs);

    if (!linked)
    {
        glDeleteProgram(pipeline.program);
        return PipelineHandle();
    }

    if (!pipeline.compute)
    {
        pipeline.vao = buildVertexArray(desc.layout);
        pipeline.layout = desc.layout;
        pipeline.blend = desc.blend;
        pipeline.depth = desc.depth;
        pipeline.stencil = desc.stencil;
        pipeline.raster = desc.raster;
        pipeline.topology = desc.topology;
        pipeline.patchVertices = desc.patchVertices;
    }

    if (desc.debugName && mCaps.debugOutput)
        glObjectLabel(GL_PROGRAM, pipeline.program, -1, desc.debugName);

    return mPipelines.add(pipeline);
}

void GLDevice::destroy(PipelineHandle handle)
{
    GLPipeline pipeline;
    if (!mPipelines.remove(handle, pipeline))
        return;
    if (!isGPUContextAlive())
        return;

    if (mBoundProgram == pipeline.program)
        mBoundProgram = 0;
    if (mBoundVao == pipeline.vao)
        mBoundVao = 0;
    if (mCurrentPipeline == handle)
        mCurrentPipeline = PipelineHandle();

    if (pipeline.vao)
        glDeleteVertexArrays(1, &pipeline.vao);
    glDeleteProgram(pipeline.program);
}

// ------------------------------------------------------------------ targets

TargetHandle GLDevice::createTarget(const TargetDesc& desc)
{
    GLTarget target;
    glCreateFramebuffers(1, &target.fbo);
    target.colorCount = desc.colorCount;

    GLenum drawBuffers[TargetDesc::MaxColors];

    for (u8 i = 0; i < desc.colorCount; ++i)
    {
        const TargetAttachment& attachment = desc.colors[i];
        GLTexture* texture = mTextures.get(attachment.texture);
        if (!texture)
        {
            Log::error("GPU: createTarget with a stale colour attachment");
            glDeleteFramebuffers(1, &target.fbo);
            return TargetHandle();
        }

        if (texture->target == GL_TEXTURE_2D_ARRAY || texture->target == GL_TEXTURE_3D ||
            texture->target == GL_TEXTURE_CUBE_MAP)
        {
            glNamedFramebufferTextureLayer(target.fbo, GL_COLOR_ATTACHMENT0 + i, texture->id,
                                           static_cast<GLint>(attachment.mip),
                                           static_cast<GLint>(attachment.slice));
        }
        else
        {
            glNamedFramebufferTexture(target.fbo, GL_COLOR_ATTACHMENT0 + i, texture->id,
                                      static_cast<GLint>(attachment.mip));
        }

        drawBuffers[i] = GL_COLOR_ATTACHMENT0 + i;
        target.width = Math::max(1u, texture->width >> attachment.mip);
        target.height = Math::max(1u, texture->height >> attachment.mip);
    }

    if (desc.colorCount > 0)
        glNamedFramebufferDrawBuffers(target.fbo, desc.colorCount, drawBuffers);
    else
        glNamedFramebufferDrawBuffer(target.fbo, GL_NONE);

    if (desc.depth.texture.valid())
    {
        GLTexture* texture = mTextures.get(desc.depth.texture);
        if (!texture)
        {
            Log::error("GPU: createTarget with a stale depth attachment");
            glDeleteFramebuffers(1, &target.fbo);
            return TargetHandle();
        }
        else
        {
            const GLenum attachmentPoint = texture->format == Format::Depth24Stencil8
                                               ? GL_DEPTH_STENCIL_ATTACHMENT
                                               : GL_DEPTH_ATTACHMENT;
            if (texture->target == GL_TEXTURE_2D_ARRAY || texture->target == GL_TEXTURE_CUBE_MAP)
            {
                glNamedFramebufferTextureLayer(target.fbo, attachmentPoint, texture->id,
                                               static_cast<GLint>(desc.depth.mip),
                                               static_cast<GLint>(desc.depth.slice));
            }
            else
            {
                glNamedFramebufferTexture(target.fbo, attachmentPoint, texture->id,
                                          static_cast<GLint>(desc.depth.mip));
            }

            if (target.width == 0)
            {
                // Was the base texture dimension unconditionally - a
                // depth-only target (no color attachments, target.width
                // still 0 here) at mip > 0 reported the wrong size for every
                // mip but 0, and callers that size a viewport off it drew
                // into a fraction of the actual attachment.
                target.width = Math::max(1u, texture->width >> desc.depth.mip);
                target.height = Math::max(1u, texture->height >> desc.depth.mip);
            }
        }
    }

    const GLenum status = glCheckNamedFramebufferStatus(target.fbo, GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE)
    {
        Log::error("GPU: framebuffer %s incomplete (0x%04X)", desc.debugName ? desc.debugName : "?",
                   status);
        glDeleteFramebuffers(1, &target.fbo);
        return TargetHandle();
    }

    if (desc.debugName && mCaps.debugOutput)
        glObjectLabel(GL_FRAMEBUFFER, target.fbo, -1, desc.debugName);

    return mTargets.add(target);
}

void GLDevice::destroy(TargetHandle handle)
{
    GLTarget target;
    if (!mTargets.remove(handle, target))
        return;
    if (!isGPUContextAlive())
        return;

    if (mBoundFbo == target.fbo)
        mBoundFbo = 0;
    if (mCurrentTarget == handle)
        mCurrentTarget = TargetHandle();

    glDeleteFramebuffers(1, &target.fbo);
}

QueryHandle GLDevice::createQuery()
{
    GLQuery query;
    glGenQueries(1, &query.id);
    return mQueries.add(query);
}

void GLDevice::destroy(QueryHandle handle)
{
    GLQuery query;
    if (!mQueries.remove(handle, query))
        return;
    if (!isGPUContextAlive())
        return;
    glDeleteQueries(1, &query.id);
}

void GLDevice::beginOcclusionQuery(QueryHandle handle)
{
    GLQuery* query = mQueries.get(handle);
    if (!query)
        return;
    // Every caller of this only ever compares the result against zero (see
    // Scene::updateOcclusionQueries()) - never needs an exact sample count.
    // The conservative any-samples query lets the GPU stop as soon as one
    // fragment passes instead of rasterizing and depth-testing the whole
    // box, which is strictly cheaper for a question with a boolean answer.
    glBeginQuery(GL_ANY_SAMPLES_PASSED_CONSERVATIVE, query->id);
}

void GLDevice::endOcclusionQuery()
{
    glEndQuery(GL_ANY_SAMPLES_PASSED_CONSERVATIVE);
}

void GLDevice::resolveQuery(QueryHandle handle, BufferHandle target, u64 offsetBytes)
{
    const GLQuery* query = mQueries.get(handle);
    const GLBuffer* buffer = mBuffers.get(target);
    if (!query || !buffer)
        return;
    // GL_QUERY_RESULT, not GL_QUERY_RESULT_NO_WAIT: the wait happens on the
    // GPU, which is exactly the point - the copy is ordered after the query
    // in the command stream and the CPU never learns about it. Asking for
    // NO_WAIT here would write a stale value whenever the query had not
    // finished, and nothing downstream could tell that had happened.
    glGetQueryBufferObjectuiv(query->id, buffer->id, GL_QUERY_RESULT,
                              static_cast<GLintptr>(offsetBytes));
}

const void* GLDevice::mappedData(BufferHandle handle) const
{
    const GLBuffer* buffer = mBuffers.get(handle);
    return buffer ? buffer->persistent : nullptr;
}

bool GLDevice::queryResultAvailable(QueryHandle handle) const
{
    const GLQuery* query = mQueries.get(handle);
    if (!query)
        return false;
    GLint available = GL_FALSE;
    glGetQueryObjectiv(query->id, GL_QUERY_RESULT_AVAILABLE, &available);
    return available == GL_TRUE;
}

u32 GLDevice::queryResult(QueryHandle handle) const
{
    const GLQuery* query = mQueries.get(handle);
    if (!query)
        return 0;
    GLuint result = 0;
    glGetQueryObjectuiv(query->id, GL_QUERY_RESULT, &result);
    return static_cast<u32>(result);
}

void GLDevice::blitTarget(TargetHandle dst, TargetHandle src, const Rect& dstRect,
                          const Rect& srcRect, bool depth)
{
    GLTarget* target = mTargets.get(dst);
    GLTarget* source = mTargets.get(src);

    const GLuint dstFbo = target ? target->fbo : 0;
    const GLuint srcFbo = source ? source->fbo : 0;
    const GLbitfield mask = depth ? GL_DEPTH_BUFFER_BIT : GL_COLOR_BUFFER_BIT;

    glBlitNamedFramebuffer(srcFbo, dstFbo, srcRect.x, srcRect.y, srcRect.x + srcRect.width,
                           srcRect.y + srcRect.height, dstRect.x, dstRect.y,
                           dstRect.x + dstRect.width, dstRect.y + dstRect.height, mask,
                           depth ? GL_NEAREST : GL_LINEAR);
}

// -------------------------------------------------------------------- state

void GLDevice::applyBlend(const BlendState& state)
{
    if (mStateKnown && mBlend.mode == state.mode && mBlend.writeRGB == state.writeRGB &&
        mBlend.writeA == state.writeA)
        return;

    if (state.mode == BlendMode::Opaque)
    {
        glDisable(GL_BLEND);
    }
    else
    {
        glEnable(GL_BLEND);
        switch (state.mode)
        {
        case BlendMode::Alpha:
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            break;
        case BlendMode::Additive:
            glBlendFunc(GL_SRC_ALPHA, GL_ONE);
            break;
        case BlendMode::Multiply:
            glBlendFunc(GL_DST_COLOR, GL_ZERO);
            break;
        case BlendMode::PremultipliedAlpha:
            glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
            break;
        case BlendMode::AddColors:
            glBlendFunc(GL_SRC_COLOR, GL_DST_COLOR);
            break;
        case BlendMode::SubtractColors:
            glBlendFunc(GL_SRC_COLOR, GL_ONE_MINUS_DST_COLOR);
            break;
        case BlendMode::Opaque:
            break;
        }
    }

    const GLboolean rgb = state.writeRGB ? GL_TRUE : GL_FALSE;
    const GLboolean alpha = state.writeA ? GL_TRUE : GL_FALSE;
    glColorMask(rgb, rgb, rgb, alpha);

    mBlend = state;
}

void GLDevice::applyDepth(const DepthState& state)
{
    if (mStateKnown && mDepth.test == state.test && mDepth.write == state.write &&
        mDepth.func == state.func)
        return;

    if (state.test)
        glEnable(GL_DEPTH_TEST);
    else
        glDisable(GL_DEPTH_TEST);

    glDepthMask(state.write ? GL_TRUE : GL_FALSE);
    glDepthFunc(compareEnum(state.func));

    mDepth = state;
}

void GLDevice::applyStencil(const StencilState& state)
{
    if (mStateKnown && mStencil.enabled == state.enabled && mStencil.compare == state.compare &&
        mStencil.fail == state.fail && mStencil.depthFail == state.depthFail &&
        mStencil.pass == state.pass && mStencil.readMask == state.readMask &&
        mStencil.writeMask == state.writeMask)
        return;

    if (state.enabled)
        glEnable(GL_STENCIL_TEST);
    else
        glDisable(GL_STENCIL_TEST);

    glStencilFunc(compareEnum(state.compare), static_cast<GLint>(mStencilRef), state.readMask);
    glStencilOp(stencilOpEnum(state.fail), stencilOpEnum(state.depthFail),
                stencilOpEnum(state.pass));
    glStencilMask(state.writeMask);
    mStencil = state;
}

void GLDevice::applyRaster(const RasterState& state)
{
    if (mStateKnown && mRaster.cull == state.cull && mRaster.frontCCW == state.frontCCW &&
        mRaster.wireframe == state.wireframe && mRaster.depthBias == state.depthBias &&
        mRaster.depthBiasSlope == state.depthBiasSlope)
        return;

    if (state.cull == CullMode::None)
    {
        glDisable(GL_CULL_FACE);
    }
    else
    {
        glEnable(GL_CULL_FACE);
        glCullFace(state.cull == CullMode::Back ? GL_BACK : GL_FRONT);
    }

    glFrontFace(state.frontCCW ? GL_CCW : GL_CW);
    glPolygonMode(GL_FRONT_AND_BACK, state.wireframe ? GL_LINE : GL_FILL);

    if (state.depthBias != 0.0f || state.depthBiasSlope != 0.0f)
    {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(state.depthBiasSlope, state.depthBias);
    }
    else
    {
        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    mRaster = state;
}

void GLDevice::setTarget(TargetHandle handle, const ClearValue& clear)
{
    GLuint fbo = 0;
    u32 width = 0;
    u32 height = 0;

    if (handle.valid())
    {
        GLTarget* target = mTargets.get(handle);
        if (!target)
        {
            Log::error("GPU: setTarget with a stale handle");
            return;
        }
        fbo = target->fbo;
        width = target->width;
        height = target->height;
    }
    else
    {
        int drawableWidth = 0;
        int drawableHeight = 0;
        mWindow.getDrawableSize(drawableWidth, drawableHeight);
        width = static_cast<u32>(drawableWidth);
        height = static_cast<u32>(drawableHeight);
    }

    if (mBoundFbo != fbo)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        mBoundFbo = fbo;
        ++mStats.targetSwitches;
    }
    mCurrentTarget = handle;

    Viewport viewport;
    viewport.width = static_cast<f32>(width);
    viewport.height = static_cast<f32>(height);
    setViewport(viewport);

    if (clear.bits == 0)
        return;

    // ClearBuffer* obeys the scissor test whatever it is clearing, so a
    // scissor left on by an earlier pass would clip the clear to that
    // rectangle and leave the rest of the target holding the last frame.
    if (mScissorEnabled)
    {
        glDisable(GL_SCISSOR_TEST);
        mScissorEnabled = false;
    }

    if (clear.bits & ClearColor)
    {
        // And it obeys the colour write mask, exactly as the depth clear
        // below obeys the depth one. A depth-only pass leaves the mask off
        // (see DepthPass), and the shadow passes run before the scene's own
        // clear - so without this the clear is dropped and only the sky's
        // fullscreen triangle hides it. Turn the sky off and last frame
        // stays on screen, smearing as the camera moves.
        if (!mBlend.writeRGB || !mBlend.writeA)
        {
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            mBlend.writeRGB = true;
            mBlend.writeA = true;
        }

        const u8 count = handle.valid() ? mTargets.get(handle)->colorCount : 1;
        for (u8 i = 0; i < count; ++i)
            glClearNamedFramebufferfv(fbo, GL_COLOR, i, clear.color);
    }

    if (clear.bits & (ClearDepth | ClearStencil))
    {
        // Write masks must not swallow a depth or stencil clear.
        if (!mDepth.write)
        {
            glDepthMask(GL_TRUE);
            mDepth.write = true;
        }
        if ((clear.bits & ClearStencil) && mStencil.writeMask != 0xFF)
        {
            glStencilMask(0xFF);
            mStencil.writeMask = 0xFF;
        }

        if ((clear.bits & ClearDepth) && (clear.bits & ClearStencil))
            glClearNamedFramebufferfi(fbo, GL_DEPTH_STENCIL, 0, clear.depth,
                                      static_cast<GLint>(clear.stencil));
        else if (clear.bits & ClearDepth)
            glClearNamedFramebufferfv(fbo, GL_DEPTH, 0, &clear.depth);
        else
            glClearNamedFramebufferiv(fbo, GL_STENCIL, 0,
                                      reinterpret_cast<const GLint*>(&clear.stencil));
    }
}

void GLDevice::clearColorAttachment(TargetHandle handle, u32 attachment, const f32 color[4])
{
    GLTarget* target = mTargets.get(handle);
    if (!target || attachment >= target->colorCount)
    {
        Log::error("GPU: clearColorAttachment with an invalid target or attachment");
        return;
    }

    const bool restoreScissor = mScissorEnabled;
    if (restoreScissor)
        glDisable(GL_SCISSOR_TEST);
    glClearNamedFramebufferfv(target->fbo, GL_COLOR, static_cast<GLint>(attachment), color);
    if (restoreScissor)
        glEnable(GL_SCISSOR_TEST);
}

void GLDevice::clearRegion(TargetHandle handle, const Rect& rect, const ClearValue& clear)
{
    if (clear.bits == 0)
        return;

    GLuint fbo = 0;
    u8 colorCount = 1;
    if (handle.valid())
    {
        GLTarget* target = mTargets.get(handle);
        if (!target)
        {
            Log::error("GPU: clearRegion with a stale handle");
            return;
        }
        fbo = target->fbo;
        colorCount = target->colorCount;
    }

    const bool previousScissorEnabled = mScissorEnabled;
    const Rect previousScissorRect = mScissorRect;
    setScissor(rect);
    setScissorEnabled(true);

    if (clear.bits & ClearColor)
    {
        if (!mBlend.writeRGB || !mBlend.writeA)
        {
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            mBlend.writeRGB = true;
            mBlend.writeA = true;
        }
        for (u8 i = 0; i < colorCount; ++i)
            glClearNamedFramebufferfv(fbo, GL_COLOR, i, clear.color);
    }

    if (clear.bits & (ClearDepth | ClearStencil))
    {
        if (!mDepth.write)
        {
            glDepthMask(GL_TRUE);
            mDepth.write = true;
        }
        if ((clear.bits & ClearStencil) && mStencil.writeMask != 0xFF)
        {
            glStencilMask(0xFF);
            mStencil.writeMask = 0xFF;
        }

        if ((clear.bits & ClearDepth) && (clear.bits & ClearStencil))
            glClearNamedFramebufferfi(fbo, GL_DEPTH_STENCIL, 0, clear.depth,
                                      static_cast<GLint>(clear.stencil));
        else if (clear.bits & ClearDepth)
            glClearNamedFramebufferfv(fbo, GL_DEPTH, 0, &clear.depth);
        else
            glClearNamedFramebufferiv(fbo, GL_STENCIL, 0,
                                      reinterpret_cast<const GLint*>(&clear.stencil));
    }

    setScissor(previousScissorRect);
    setScissorEnabled(previousScissorEnabled);
}

void GLDevice::setPipeline(PipelineHandle handle)
{
    GLPipeline* pipeline = mPipelines.get(handle);
    if (!pipeline)
    {
        Log::error("GPU: setPipeline with a stale handle");
        return;
    }

    if (mBoundProgram != pipeline->program)
    {
        glUseProgram(pipeline->program);
        mBoundProgram = pipeline->program;
        ++mStats.pipelineSwitches;
    }

    if (!pipeline->compute)
    {
        applyBlend(pipeline->blend);
        applyDepth(pipeline->depth);
        applyStencil(pipeline->stencil);
        applyRaster(pipeline->raster);
        mStateKnown = true;

        if (pipeline->topology == Topology::Patches)
            glPatchParameteri(GL_PATCH_VERTICES, static_cast<GLint>(pipeline->patchVertices));
    }

    mCurrentPipeline = handle;
}

void GLDevice::setViewport(const Viewport& viewport)
{
    glViewport(static_cast<GLint>(viewport.x), static_cast<GLint>(viewport.y),
               static_cast<GLsizei>(viewport.width), static_cast<GLsizei>(viewport.height));
    glDepthRangef(viewport.minDepth, viewport.maxDepth);
}

void GLDevice::setScissor(const Rect& rect)
{
    mScissorRect = rect;
    glScissor(rect.x, rect.y, rect.width, rect.height);
}

void GLDevice::setScissorEnabled(bool enabled)
{
    if (mScissorEnabled == enabled)
        return;

    if (enabled)
        glEnable(GL_SCISSOR_TEST);
    else
        glDisable(GL_SCISSOR_TEST);

    mScissorEnabled = enabled;
}

void GLDevice::setClipDistanceEnabled(bool enabled)
{
    if (mClipDistanceEnabled == enabled)
        return;

    if (enabled)
        glEnable(GL_CLIP_DISTANCE0);
    else
        glDisable(GL_CLIP_DISTANCE0);

    mClipDistanceEnabled = enabled;
}

void GLDevice::setStencilRef(u32 value)
{
    if (mStencilRef == value)
        return;

    mStencilRef = value;
    glStencilFunc(compareEnum(mStencil.compare), static_cast<GLint>(value), mStencil.readMask);
}
void GLDevice::setBlendFactor(f32 r, f32 g, f32 b, f32 a)
{
    glBlendColor(r, g, b, a);
}

void GLDevice::setDepthBias(f32 slope, f32 constant)
{
    RasterState state = mRaster;
    state.depthBias = constant;
    state.depthBiasSlope = slope;
    applyRaster(state);
}

// ----------------------------------------------------------------- bindings

void GLDevice::bindTexture(u32 slot, TextureHandle handle, SamplerHandle samplerHandle)
{
    if (slot >= MaxTextureSlots)
        return;

    GLTexture* texture = mTextures.get(handle);
    const GLuint textureId = texture ? texture->id : 0;
    if (mBoundTextures[slot] != textureId)
    {
        glBindTextureUnit(slot, textureId);
        mBoundTextures[slot] = textureId;
        ++mStats.textureBinds;
    }

    GLSampler* sampler = mSamplers.get(samplerHandle);
    const GLuint samplerId = sampler ? sampler->id : 0;
    if (mBoundSamplers[slot] != samplerId)
    {
        glBindSampler(slot, samplerId);
        mBoundSamplers[slot] = samplerId;
    }
}

void GLDevice::bindImage(u32 slot, TextureHandle handle, u32 mip, bool write)
{
    GLTexture* texture = mTextures.get(handle);
    if (!texture)
        return;

    const FormatInfo info = formatInfo(texture->format);
    const GLboolean layered = texture->target == GL_TEXTURE_2D_ARRAY ||
                                      texture->target == GL_TEXTURE_3D ||
                                      texture->target == GL_TEXTURE_CUBE_MAP
                                  ? GL_TRUE
                                  : GL_FALSE;

    glBindImageTexture(slot, texture->id, static_cast<GLint>(mip), layered, 0,
                       write ? GL_READ_WRITE : GL_READ_ONLY, info.internalFormat);
}

void GLDevice::bindUniform(u32 slot, BufferHandle handle, u64 offset, u64 size)
{
    if (slot >= MaxBufferSlots)
        return;

    GLBuffer* buffer = mBuffers.get(handle);
    if (!buffer)
        return;

    if (offset >= buffer->size || (size && size > buffer->size - offset))
    {
        Log::error("GPU: bindUniform out of range");
        return;
    }
    // glBindBufferRange raises GL_INVALID_VALUE for an offset that is not a
    // multiple of GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT - queried once into
    // mCaps.uniformOffsetAlignment already, just never checked against here.
    if (mCaps.uniformOffsetAlignment && offset % mCaps.uniformOffsetAlignment != 0)
    {
        Log::error("GPU: bindUniform offset %llu is not a multiple of the required alignment %u",
                  static_cast<unsigned long long>(offset), mCaps.uniformOffsetAlignment);
        return;
    }

    const GLsizeiptr range =
        size ? static_cast<GLsizeiptr>(size) : static_cast<GLsizeiptr>(buffer->size - offset);
    glBindBufferRange(GL_UNIFORM_BUFFER, slot, buffer->id, static_cast<GLintptr>(offset), range);
    mBoundUniforms[slot] = buffer->id;
}

void GLDevice::bindStorage(u32 slot, BufferHandle handle, u64 offset, u64 size)
{
    if (slot >= MaxBufferSlots)
        return;

    GLBuffer* buffer = mBuffers.get(handle);
    if (!buffer)
        return;

    if (offset >= buffer->size || (size && size > buffer->size - offset))
    {
        Log::error("GPU: bindStorage out of range");
        return;
    }
    if (mCaps.storageOffsetAlignment && offset % mCaps.storageOffsetAlignment != 0)
    {
        Log::error("GPU: bindStorage offset %llu is not a multiple of the required alignment %u",
                  static_cast<unsigned long long>(offset), mCaps.storageOffsetAlignment);
        return;
    }

    const GLsizeiptr range =
        size ? static_cast<GLsizeiptr>(size) : static_cast<GLsizeiptr>(buffer->size - offset);
    glBindBufferRange(GL_SHADER_STORAGE_BUFFER, slot, buffer->id, static_cast<GLintptr>(offset),
                      range);
    mBoundStorages[slot] = buffer->id;
}

void GLDevice::forgetTexture(GLuint id)
{
    for (u32 i = 0; i < MaxTextureSlots; ++i)
    {
        if (mBoundTextures[i] == id)
            mBoundTextures[i] = 0;
    }
}

void GLDevice::forgetBuffer(GLuint id)
{
    for (u32 i = 0; i < MaxBufferSlots; ++i)
    {
        if (mBoundUniforms[i] == id)
            mBoundUniforms[i] = 0;
        if (mBoundStorages[i] == id)
            mBoundStorages[i] = 0;
    }
}

// -------------------------------------------------------------------- draws

bool GLDevice::bindDrawState(const DrawDesc& desc, GLPipeline*& pipeline)
{
    pipeline = mPipelines.get(mCurrentPipeline);
    if (!pipeline || pipeline->compute)
    {
        Log::error("GPU: draw without a graphics pipeline");
        return false;
    }

    if (desc.vertexBufferCount > VertexLayout::MaxStreams)
    {
        Log::error("GPU: draw with vertexBufferCount (%u) past MaxStreams (%u)",
                  desc.vertexBufferCount, VertexLayout::MaxStreams);
        return false;
    }

    if (mBoundVao != pipeline->vao)
    {
        glBindVertexArray(pipeline->vao);
        mBoundVao = pipeline->vao;
    }

    for (u8 i = 0; i < desc.vertexBufferCount; ++i)
    {
        GLBuffer* buffer = mBuffers.get(desc.vertexBuffers[i]);
        if (!buffer)
        {
            Log::error("GPU: draw with a stale vertex buffer");
            return false;
        }
        const GLsizei stride = i < pipeline->layout.streamCount
                                   ? static_cast<GLsizei>(pipeline->layout.streams[i].stride)
                                   : 0;
        glVertexArrayVertexBuffer(pipeline->vao, i, buffer->id,
                                  static_cast<GLintptr>(desc.vertexOffsets[i]), stride);
    }

    if (desc.indexBuffer.valid())
    {
        GLBuffer* buffer = mBuffers.get(desc.indexBuffer);
        if (!buffer)
        {
            Log::error("GPU: draw with a stale index buffer");
            return false;
        }
        glVertexArrayElementBuffer(pipeline->vao, buffer->id);
    }

    return true;
}

void GLDevice::draw(const DrawDesc& desc)
{
    GLPipeline* pipeline = nullptr;
    if (!bindDrawState(desc, pipeline))
        return;

#ifdef RADION_DEBUG
    // Asks the driver why a draw would fail, which the plain GL error never
    // says. Once per program: the answer is about how the program is wired to
    // the current state, and that does not change between frames.
    if (mValidatedPrograms.insert(pipeline->program).second)
    {
        glValidateProgram(pipeline->program);
        GLint status = GL_FALSE;
        glGetProgramiv(pipeline->program, GL_VALIDATE_STATUS, &status);
        if (status == GL_FALSE)
        {
            char log[1024];
            GLsizei written = 0;
            glGetProgramInfoLog(pipeline->program, sizeof(log), &written, log);
            Log::error("GPU: program %u fails validation: %s", pipeline->program, log);
        }
    }
#endif

    const GLenum mode = topologyEnum(pipeline->topology);

    if (desc.indexBuffer.valid())
    {
        const usize stride = indexSize(desc.indexType);
        const usize offset = desc.indexOffset + static_cast<usize>(desc.first) * stride;
        // Neither indexOffset nor first*stride was ever checked against the
        // buffer glVertexArrayElementBuffer() just bound: a caller's range
        // arithmetic bug reached the driver as a byte offset with nothing on
        // this side to catch it first.
        const GLBuffer* indexBuffer = mBuffers.get(desc.indexBuffer);
        const usize span = static_cast<usize>(desc.count) * stride;
        if (!indexBuffer || offset > indexBuffer->size || span > indexBuffer->size - offset)
        {
            Log::error("GPU: draw with an index range past the bound index buffer");
            return;
        }
        glDrawElementsInstancedBaseVertexBaseInstance(
            mode, static_cast<GLsizei>(desc.count), indexTypeEnum(desc.indexType),
            reinterpret_cast<const void*>(offset), static_cast<GLsizei>(desc.instanceCount),
            desc.baseVertex, desc.firstInstance);
    }
    else
    {
        glDrawArraysInstancedBaseInstance(
            mode, static_cast<GLint>(desc.first), static_cast<GLsizei>(desc.count),
            static_cast<GLsizei>(desc.instanceCount), desc.firstInstance);
    }

    ++mStats.drawCalls;
    if (pipeline->topology == Topology::Triangles)
        mStats.triangles += desc.count / 3 * desc.instanceCount;
    else if (pipeline->topology == Topology::TriangleStrip && desc.count >= 3)
        mStats.triangles += (desc.count - 2) * desc.instanceCount;
}

namespace
{
// GL_DrawArraysIndirectCommand is 4 u32s; GL_DrawElementsIndirectCommand is
// 5 - both packed with no padding, which is what stride 0 below asks the
// driver to assume. Nothing previously confirmed drawCount commands of that
// size, at that offset, actually fit the buffer bound to
// GL_DRAW_INDIRECT_BUFFER; a producer that under-sized or mis-offset it had
// the driver read whatever memory followed.
bool validateIndirectRange(const GLBuffer* buffer, u64 offset, u32 drawCount, bool indexed,
                           const char* who)
{
    if (!buffer)
        return false;
    // 4-byte aligned per the GL spec for both GL_DRAW_INDIRECT_BUFFER and
    // GL_PARAMETER_BUFFER offsets.
    if (offset % 4 != 0)
    {
        Log::error("GPU: %s offset %llu is not 4-byte aligned", who,
                  static_cast<unsigned long long>(offset));
        return false;
    }
    const u64 commandSize = indexed ? 20u : 16u;
    const u64 span = static_cast<u64>(drawCount) * commandSize;
    if (offset > buffer->size || span > buffer->size - offset)
    {
        Log::error("GPU: %s range past the bound buffer (drawCount=%u)", who, drawCount);
        return false;
    }
    return true;
}
} // namespace

void GLDevice::drawIndirect(const DrawDesc& base, BufferHandle args, u64 offset, u32 drawCount)
{
    GLPipeline* pipeline = nullptr;
    if (!bindDrawState(base, pipeline))
        return;

    GLBuffer* buffer = mBuffers.get(args);
    if (!validateIndirectRange(buffer, offset, drawCount, base.indexBuffer.valid(),
                               "drawIndirect"))
        return;

    glBindBuffer(GL_DRAW_INDIRECT_BUFFER, buffer->id);
    const GLenum mode = topologyEnum(pipeline->topology);

    if (base.indexBuffer.valid())
    {
        glMultiDrawElementsIndirect(mode, indexTypeEnum(base.indexType),
                                    reinterpret_cast<const void*>(offset),
                                    static_cast<GLsizei>(drawCount), 0);
    }
    else
    {
        glMultiDrawArraysIndirect(mode, reinterpret_cast<const void*>(offset),
                                  static_cast<GLsizei>(drawCount), 0);
    }

    ++mStats.drawCalls;
}

void GLDevice::drawIndirectCount(const DrawDesc& base, BufferHandle args, u64 argsOffset,
                                 BufferHandle count, u64 countOffset, u32 maxDraws)
{
    if (!mCaps.indirectParameters)
    {
        // Without the count buffer every slot is issued; entries the GPU left
        // at zero instances are no-ops, so the result matches as long as the
        // producer zeroes what it does not use.
        drawIndirect(base, args, argsOffset, maxDraws);
        return;
    }

    GLPipeline* pipeline = nullptr;
    if (!bindDrawState(base, pipeline))
        return;

    GLBuffer* argsBuffer = mBuffers.get(args);
    GLBuffer* countBuffer = mBuffers.get(count);
    if (!validateIndirectRange(argsBuffer, argsOffset, maxDraws, base.indexBuffer.valid(),
                               "drawIndirectCount"))
        return;
    // The count the driver reads back at runtime can be anywhere up to
    // maxDraws; only the four bytes it is stored in need to fit here.
    if (!countBuffer || countOffset % 4 != 0 || countOffset > countBuffer->size ||
        sizeof(u32) > countBuffer->size - countOffset)
    {
        Log::error("GPU: drawIndirectCount count offset past the bound count buffer");
        return;
    }

    glBindBuffer(GL_DRAW_INDIRECT_BUFFER, argsBuffer->id);
    glBindBuffer(GL_PARAMETER_BUFFER, countBuffer->id);

    const GLenum mode = topologyEnum(pipeline->topology);
    if (base.indexBuffer.valid())
    {
        glMultiDrawElementsIndirectCount(
            mode, indexTypeEnum(base.indexType), reinterpret_cast<const void*>(argsOffset),
            static_cast<GLintptr>(countOffset), static_cast<GLsizei>(maxDraws), 0);
    }
    else
    {
        glMultiDrawArraysIndirectCount(mode, reinterpret_cast<const void*>(argsOffset),
                                       static_cast<GLintptr>(countOffset),
                                       static_cast<GLsizei>(maxDraws), 0);
    }

    ++mStats.drawCalls;
}

// ------------------------------------------------------------------ compute

void GLDevice::dispatch(u32 x, u32 y, u32 z)
{
    GLPipeline* pipeline = mPipelines.get(mCurrentPipeline);
    if (!pipeline || !pipeline->compute)
    {
        Log::error("GPU: dispatch without a compute pipeline");
        return;
    }
    glDispatchCompute(x, y, z);
    ++mStats.dispatches;
}

void GLDevice::dispatchIndirect(BufferHandle args, u64 offset)
{
    GLBuffer* buffer = mBuffers.get(args);
    if (!buffer)
        return;

    GLPipeline* pipeline = mPipelines.get(mCurrentPipeline);
    if (!pipeline || !pipeline->compute)
    {
        Log::error("GPU: dispatchIndirect without a compute pipeline");
        return;
    }

    glBindBuffer(GL_DISPATCH_INDIRECT_BUFFER, buffer->id);
    glDispatchComputeIndirect(static_cast<GLintptr>(offset));
    ++mStats.dispatches;
}

void GLDevice::barrier(u32 bits)
{
    glMemoryBarrier(barrierBitsToGL(bits));
}

// -------------------------------------------------------------------- frame

void GLDevice::beginFrame()
{
    mStats = GPUStats();
    mStats.gpuMilliseconds = mLastGpuMilliseconds;
    mActiveTimer = -1;
    if (!mCaps.timerQuery)
        return;

    const u32 slot = mTimerCursor;
    if (mTimerPending[slot])
    {
        GLint available = GL_FALSE;
        glGetQueryObjectiv(mTimerQueries[slot], GL_QUERY_RESULT_AVAILABLE, &available);
        if (!available)
            return;

        GLuint64 nanoseconds = 0;
        glGetQueryObjectui64v(mTimerQueries[slot], GL_QUERY_RESULT, &nanoseconds);
        mLastGpuMilliseconds = static_cast<f32>(nanoseconds) / 1000000.0f;
        mStats.gpuMilliseconds = mLastGpuMilliseconds;
        mTimerPending[slot] = false;
    }

    glBeginQuery(GL_TIME_ELAPSED, mTimerQueries[slot]);
    mActiveTimer = static_cast<s32>(slot);
}

void GLDevice::endFrame()
{
    if (mActiveTimer < 0)
        return;
    glEndQuery(GL_TIME_ELAPSED);
    mTimerPending[mActiveTimer] = true;
    mTimerCursor = (static_cast<u32>(mActiveTimer) + 1) % TimerQueryCount;
    mActiveTimer = -1;
}

void GLDevice::present()
{
    mWindow.flip();
}

void GLDevice::resetForExternal()
{
    glUseProgram(0);
    glBindVertexArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
    glBindBuffer(GL_DISPATCH_INDIRECT_BUFFER, 0);

    // A sampler object left on a unit overrides the texture's own parameters,
    // and code that binds plain textures has no idea it is there.
    for (u32 i = 0; i < MaxTextureSlots; ++i)
    {
        if (mBoundSamplers[i] != 0)
            glBindSampler(i, 0);
    }

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glActiveTexture(GL_TEXTURE0);

    invalidateState();
}

void GLDevice::invalidateState()
{
    mBoundProgram = 0;
    mBoundVao = 0;
    mBoundFbo = 0;
    std::memset(mBoundTextures, 0, sizeof(mBoundTextures));
    std::memset(mBoundSamplers, 0, sizeof(mBoundSamplers));
    std::memset(mBoundUniforms, 0, sizeof(mBoundUniforms));
    std::memset(mBoundStorages, 0, sizeof(mBoundStorages));

    mCurrentPipeline = PipelineHandle();
    mStateKnown = false;
    mScissorEnabled = false;
    mClipDistanceEnabled = false;
}

const GPUStats& GLDevice::stats() const
{
    return mStats;
}

const GPUCaps& GLDevice::caps() const
{
    return mCaps;
}

void GLDevice::pushMarker(const char* name)
{
    if (mCaps.debugOutput && name)
        glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, name);
}

void GLDevice::popMarker()
{
    if (mCaps.debugOutput)
        glPopDebugGroup();
}

// ------------------------------------------------------------------ factory

GPU* GPU::createOpenGL(Platform::Window& window)
{
    GLDevice* device = new GLDevice(window);
    if (!device->initialize())
    {
        delete device;
        return nullptr;
    }

    setSingleton(device);
    return device;
}

void GPU::destroyDevice(GPU* gpu)
{
    // getSingleton() aborts when no device is set - exactly the case a
    // cleanup path must tolerate, which is what tryGet() is for.
    if (gpu == tryGet())
        setSingleton(nullptr);
    delete gpu;
}

} // namespace Radion
