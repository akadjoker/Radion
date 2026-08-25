#ifndef RADION_GPU_CAPS_H
#define RADION_GPU_CAPS_H

#include "Types.h"

namespace Radion
{

// What the driver on this machine actually offers. Filled once after the
// context is current; everything else in gpu/ reads it instead of asking GL.
struct GPUCaps
{
    char vendor[128] = {};
    char renderer[192] = {};
    char version[128] = {};
    char glsl[96] = {};

    s32 versionMajor = 0;
    s32 versionMinor = 0;

    // Without these the backend cannot run and createOpenGL fails.
    bool directStateAccess = false;
    bool computeShader = false;
    bool storageBuffer = false;
    bool bufferStorage = false;
    bool textureStorage = false;
    bool drawIndirect = false;
    bool baseInstance = false;
    bool samplerObjects = false;
    bool debugOutput = false;

    // Present or not, the engine keeps working; a path is taken or skipped.
    bool indirectParameters = false; // glMultiDrawElementsIndirectCount
    bool multiDrawIndirect = false;
    bool anisotropicFilter = false;
    bool textureCompressionBC = false;
    bool bindlessTexture = false;
    bool clipControl = false;
    bool seamlessCubemap = false;
    bool shaderDrawParameters = false;
    bool conservativeDepth = false;
    bool timerQuery = false;

    u32 maxTextureSize = 0;
    u32 maxTextureLayers = 0;
    u32 maxTexture3DSize = 0;
    u32 maxCubeMapSize = 0;
    u32 maxTextureUnits = 0;
    u32 maxColorAttachments = 0;
    u32 maxDrawBuffers = 0;
    u32 maxSamples = 0;
    f32 maxAnisotropy = 1.0f;

    u32 maxUniformBindings = 0;
    u32 maxUniformBlockSize = 0;
    u32 uniformOffsetAlignment = 0;
    u32 maxStorageBindings = 0;
    u32 maxStorageBlockSize = 0;
    u32 storageOffsetAlignment = 0;

    u32 maxComputeGroupCount[3] = {};
    u32 maxComputeGroupSize[3] = {};
    u32 maxComputeInvocations = 0;
    u32 maxComputeSharedMemory = 0;

    u32 maxVertexAttribs = 0;
    u32 maxVaryings = 0;
};

bool hasGLExtension(const char* name);

// Fills caps and returns false when a required feature is missing, naming the
// missing one in the log. Needs a current context.
bool queryGPUCaps(GPUCaps& caps);

void logGPUCaps(const GPUCaps& caps);

} // namespace Radion

#endif // RADION_GPU_CAPS_H
