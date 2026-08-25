#include "PCH.h"

#include "GPUCaps.h"
#include "Log.h"

#include <cstring>
#include <glad.h>

namespace Radion
{

namespace
{

void copyString(char* dst, usize size, const GLubyte* src)
{
    if (!src)
    {
        dst[0] = '\0';
        return;
    }
    std::strncpy(dst, reinterpret_cast<const char*>(src), size - 1);
    dst[size - 1] = '\0';
}

u32 getU32(GLenum name)
{
    GLint value = 0;
    glGetIntegerv(name, &value);
    return value > 0 ? static_cast<u32>(value) : 0u;
}

u32 getIndexedU32(GLenum name, u32 index)
{
    GLint value = 0;
    glGetIntegeri_v(name, index, &value);
    return value > 0 ? static_cast<u32>(value) : 0u;
}

f32 getF32(GLenum name)
{
    GLfloat value = 0.0f;
    glGetFloatv(name, &value);
    return value;
}

bool require(bool present, const char* name, bool& ok)
{
    if (!present)
    {
        Log::error("GPU: missing required feature %s", name);
        ok = false;
    }
    return present;
}

} // namespace

bool hasGLExtension(const char* name)
{
    GLint count = 0;
    glGetIntegerv(GL_NUM_EXTENSIONS, &count);
    for (GLint i = 0; i < count; ++i)
    {
        const GLubyte* ext = glGetStringi(GL_EXTENSIONS, static_cast<GLuint>(i));
        if (ext && std::strcmp(reinterpret_cast<const char*>(ext), name) == 0)
            return true;
    }
    return false;
}

bool queryGPUCaps(GPUCaps& caps)
{
    caps = GPUCaps();

    copyString(caps.vendor, sizeof(caps.vendor), glGetString(GL_VENDOR));
    copyString(caps.renderer, sizeof(caps.renderer), glGetString(GL_RENDERER));
    copyString(caps.version, sizeof(caps.version), glGetString(GL_VERSION));
    copyString(caps.glsl, sizeof(caps.glsl), glGetString(GL_SHADING_LANGUAGE_VERSION));

    glGetIntegerv(GL_MAJOR_VERSION, &caps.versionMajor);
    glGetIntegerv(GL_MINOR_VERSION, &caps.versionMinor);

    caps.directStateAccess = GLAD_GL_VERSION_4_5 || GLAD_GL_ARB_direct_state_access;
    caps.computeShader = GLAD_GL_VERSION_4_3 || GLAD_GL_ARB_compute_shader;
    caps.storageBuffer = GLAD_GL_VERSION_4_3 || GLAD_GL_ARB_shader_storage_buffer_object;
    caps.bufferStorage = GLAD_GL_VERSION_4_4 || GLAD_GL_ARB_buffer_storage;
    caps.textureStorage = GLAD_GL_VERSION_4_2 || GLAD_GL_ARB_texture_storage;
    caps.drawIndirect = GLAD_GL_VERSION_4_0 || GLAD_GL_ARB_draw_indirect;
    caps.baseInstance = GLAD_GL_VERSION_4_2 || GLAD_GL_ARB_base_instance;
    caps.samplerObjects = GLAD_GL_VERSION_3_3 || GLAD_GL_ARB_sampler_objects;
    caps.debugOutput = GLAD_GL_VERSION_4_3 || GLAD_GL_KHR_debug;

    caps.indirectParameters = GLAD_GL_VERSION_4_6 || GLAD_GL_ARB_indirect_parameters;
    caps.multiDrawIndirect = GLAD_GL_VERSION_4_3 || GLAD_GL_ARB_multi_draw_indirect;
    caps.anisotropicFilter = GLAD_GL_VERSION_4_6 || GLAD_GL_ARB_texture_filter_anisotropic ||
                             GLAD_GL_EXT_texture_filter_anisotropic;
    caps.textureCompressionBC = GLAD_GL_VERSION_4_2 || GLAD_GL_ARB_texture_compression_bptc;
    caps.bindlessTexture = GLAD_GL_ARB_bindless_texture != 0;
    caps.clipControl = GLAD_GL_VERSION_4_5 || GLAD_GL_ARB_clip_control;
    caps.seamlessCubemap = GLAD_GL_VERSION_3_2 || GLAD_GL_ARB_seamless_cube_map;
    caps.shaderDrawParameters = GLAD_GL_VERSION_4_6 || GLAD_GL_ARB_shader_draw_parameters;
    caps.conservativeDepth = GLAD_GL_VERSION_4_2 || GLAD_GL_ARB_conservative_depth;
    caps.timerQuery = GLAD_GL_VERSION_3_3 || GLAD_GL_ARB_timer_query;

    caps.maxTextureSize = getU32(GL_MAX_TEXTURE_SIZE);
    caps.maxTextureLayers = getU32(GL_MAX_ARRAY_TEXTURE_LAYERS);
    caps.maxTexture3DSize = getU32(GL_MAX_3D_TEXTURE_SIZE);
    caps.maxCubeMapSize = getU32(GL_MAX_CUBE_MAP_TEXTURE_SIZE);
    caps.maxTextureUnits = getU32(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS);
    caps.maxColorAttachments = getU32(GL_MAX_COLOR_ATTACHMENTS);
    caps.maxDrawBuffers = getU32(GL_MAX_DRAW_BUFFERS);
    caps.maxSamples = getU32(GL_MAX_SAMPLES);
    caps.maxVertexAttribs = getU32(GL_MAX_VERTEX_ATTRIBS);
    caps.maxVaryings = getU32(GL_MAX_VARYING_COMPONENTS);

    if (caps.anisotropicFilter)
        caps.maxAnisotropy = getF32(GL_MAX_TEXTURE_MAX_ANISOTROPY);

    caps.maxUniformBindings = getU32(GL_MAX_UNIFORM_BUFFER_BINDINGS);
    caps.maxUniformBlockSize = getU32(GL_MAX_UNIFORM_BLOCK_SIZE);
    caps.uniformOffsetAlignment = getU32(GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT);

    if (caps.storageBuffer)
    {
        caps.maxStorageBindings = getU32(GL_MAX_SHADER_STORAGE_BUFFER_BINDINGS);
        caps.maxStorageBlockSize = getU32(GL_MAX_SHADER_STORAGE_BLOCK_SIZE);
        caps.storageOffsetAlignment = getU32(GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT);
    }

    if (caps.computeShader)
    {
        for (u32 i = 0; i < 3; ++i)
        {
            caps.maxComputeGroupCount[i] = getIndexedU32(GL_MAX_COMPUTE_WORK_GROUP_COUNT, i);
            caps.maxComputeGroupSize[i] = getIndexedU32(GL_MAX_COMPUTE_WORK_GROUP_SIZE, i);
        }
        caps.maxComputeInvocations = getU32(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS);
        caps.maxComputeSharedMemory = getU32(GL_MAX_COMPUTE_SHARED_MEMORY_SIZE);
    }

    bool ok = true;
    require(caps.directStateAccess, "direct state access (GL 4.5 / ARB_direct_state_access)", ok);
    require(caps.computeShader, "compute shaders (GL 4.3)", ok);
    require(caps.storageBuffer, "shader storage buffers (GL 4.3)", ok);
    require(caps.bufferStorage, "immutable buffer storage (GL 4.4)", ok);
    require(caps.textureStorage, "immutable texture storage (GL 4.2)", ok);
    require(caps.drawIndirect, "indirect draw (GL 4.0)", ok);
    require(caps.baseInstance, "base instance (GL 4.2)", ok);
    require(caps.samplerObjects, "sampler objects (GL 3.3)", ok);

    return ok;
}

void logGPUCaps(const GPUCaps& caps)
{
    Log::info("GPU: %s | %s", caps.vendor, caps.renderer);
    Log::info("GPU: OpenGL %s, GLSL %s", caps.version, caps.glsl);

    Log::info("GPU: optional -- indirectCount:%d multiDrawIndirect:%d aniso:%.0fx bc:%d "
              "bindless:%d clipControl:%d drawParams:%d",
              caps.indirectParameters ? 1 : 0, caps.multiDrawIndirect ? 1 : 0,
              static_cast<double>(caps.maxAnisotropy), caps.textureCompressionBC ? 1 : 0,
              caps.bindlessTexture ? 1 : 0, caps.clipControl ? 1 : 0,
              caps.shaderDrawParameters ? 1 : 0);

    Log::info("GPU: texture %u, layers %u, units %u, attachments %u, samples %u",
              caps.maxTextureSize, caps.maxTextureLayers, caps.maxTextureUnits,
              caps.maxColorAttachments, caps.maxSamples);

    Log::info(
        "GPU: ubo %u bindings / %u bytes / align %u -- ssbo %u bindings / %u bytes / align %u",
        caps.maxUniformBindings, caps.maxUniformBlockSize, caps.uniformOffsetAlignment,
        caps.maxStorageBindings, caps.maxStorageBlockSize, caps.storageOffsetAlignment);

    Log::info("GPU: compute groups %ux%ux%u, size %ux%ux%u, %u invocations, %u bytes shared",
              caps.maxComputeGroupCount[0], caps.maxComputeGroupCount[1],
              caps.maxComputeGroupCount[2], caps.maxComputeGroupSize[0],
              caps.maxComputeGroupSize[1], caps.maxComputeGroupSize[2], caps.maxComputeInvocations,
              caps.maxComputeSharedMemory);
}

} // namespace Radion
