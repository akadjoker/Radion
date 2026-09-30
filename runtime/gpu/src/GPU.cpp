#include "PCH.h"

#include "GPU.h"

#include "Log.h"

#include <cstdlib>

namespace Radion
{

namespace
{

GPU* gDevice = nullptr;

} // namespace

void GPU::setSingleton(GPU* gpu)
{
    gDevice = gpu;
}

GPU& GPU::getSingleton()
{
    if (!gDevice)
    {
        // A caller bug (before createOpenGL or after destroyDevice): fail loud instead of returning a null deref.
        Log::error("GPU: getSingleton() called with no device; this is a caller bug, "
                   "not a recoverable error - use GPU::tryGet() in cleanup paths instead");
        std::abort();
    }
    return *gDevice;
}

GPU* GPU::tryGet()
{
    return gDevice;
}

bool GPU::ready()
{
    return gDevice != nullptr;
}

ExternalGLScope::ExternalGLScope(GPU& gpu) : mGpu(gpu)
{
    mGpu.resetForExternal();
}

ExternalGLScope::~ExternalGLScope()
{
    mGpu.invalidateState();
}

} // namespace Radion
