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
        // Logging "no device" and then handing back *gDevice anyway used to
        // be a null dereference dressed up as a recoverable error - undefined
        // behaviour a caller could not tell from a real device. This path is
        // always a caller bug (calling before createOpenGL or after
        // destroyDevice), never a normal runtime condition, so it fails loud
        // and deterministic instead.
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
