#ifndef RADION_GPU_CONTEXT_H
#define RADION_GPU_CONTEXT_H

#include "GPUCaps.h"

namespace Radion
{

namespace Platform
{
class Window;
}

// Makes the window's context current, loads the entry points, installs the
// debug callback and fills caps. Fails when a required feature is missing.
bool initializeGPUContext(const Platform::Window& window, GPUCaps& caps);

void shutdownGPUContext();

// False once the context is gone. Resource destruction must check this before
// calling into GL, since deleting after the window closed is undefined.
bool isGPUContextAlive();

const GPUCaps& gpuCaps();

} // namespace Radion

#endif // RADION_GPU_CONTEXT_H
