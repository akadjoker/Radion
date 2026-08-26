#include "PCH.h"

#include "GPUContext.h"
#include "Log.h"
#include "Window.h"

#include <SDL2/SDL.h>
#include <glad.h>

namespace Radion
{

namespace
{

GPUCaps gCaps;
bool gAlive = false;

const char* debugSourceName(GLenum source)
{
    switch (source)
    {
    case GL_DEBUG_SOURCE_API:
        return "api";
    case GL_DEBUG_SOURCE_SHADER_COMPILER:
        return "shader";
    case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
        return "window";
    case GL_DEBUG_SOURCE_THIRD_PARTY:
        return "third-party";
    case GL_DEBUG_SOURCE_APPLICATION:
        return "app";
    default:
        return "other";
    }
}

const char* debugTypeName(GLenum type)
{
    switch (type)
    {
    case GL_DEBUG_TYPE_ERROR:
        return "error";
    case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
        return "deprecated";
    case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
        return "undefined";
    case GL_DEBUG_TYPE_PORTABILITY:
        return "portability";
    case GL_DEBUG_TYPE_PERFORMANCE:
        return "performance";
    case GL_DEBUG_TYPE_MARKER:
        return "marker";
    default:
        return "other";
    }
}

// Severity picks the log level rather than everything arriving as an error:
// a performance hint and undefined behaviour were indistinguishable before,
// so the ones that matter drowned in the ones that do not.
void GLAPIENTRY onDebugMessage(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei,
                               const GLchar* message, const void*)
{
    switch (severity)
    {
    case GL_DEBUG_SEVERITY_HIGH:
        Log::error("GL [%s/%s/%u] %s", debugSourceName(source), debugTypeName(type), id, message);
        break;
    case GL_DEBUG_SEVERITY_MEDIUM:
    case GL_DEBUG_SEVERITY_LOW:
        Log::warning("GL [%s/%s/%u] %s", debugSourceName(source), debugTypeName(type), id, message);
        break;
    default:
        Log::info("GL [%s/%s/%u] %s", debugSourceName(source), debugTypeName(type), id, message);
        break;
    }
}

} // namespace

bool initializeGPUContext(const Platform::Window& window, GPUCaps& caps)
{
    if (gAlive)
    {
        caps = gCaps;
        return true;
    }

    SDL_GL_MakeCurrent(window.getNativeWindow(), window.getGLContext());

    if (gladLoadGL(reinterpret_cast<GLADloadfunc>(SDL_GL_GetProcAddress)) == 0)
    {
        Log::error("GPU: glad could not load OpenGL");
        return false;
    }

    if (!queryGPUCaps(gCaps))
    {
        Log::error("GPU: this machine does not meet the minimum requirements");
        return false;
    }

    if (window.isDebugContext() && gCaps.debugOutput)
    {
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        // Keep a debug context useful without turning the terminal into a
        // frame-time cost: only API errors reach the callback.
        glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_FALSE);
        glDebugMessageControl(GL_DONT_CARE, GL_DEBUG_TYPE_ERROR, GL_DONT_CARE, 0, nullptr,
                      GL_TRUE);
        glDebugMessageCallback(onDebugMessage, nullptr);
    }

    logGPUCaps(gCaps);

    gAlive = true;
    caps = gCaps;
    return true;
}

void shutdownGPUContext()
{
    gAlive = false;
}

bool isGPUContextAlive()
{
    return gAlive;
}

const GPUCaps& gpuCaps()
{
    return gCaps;
}

} // namespace Radion
