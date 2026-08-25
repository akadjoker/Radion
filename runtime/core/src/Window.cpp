#include "PCH.h"

/** ****************************************************************************
  Radion Platform - Window implementation (SDL2 backend).
**************************************************************************** */
#include "Input.h"
#include "Log.h"
#include "Window.h"

#include <cmath>

extern "C" const char* __lsan_default_suppressions()
{
    return "leak:libSDL2\n"
           "leak:SDL_DBus\n";
}

namespace Radion
{
namespace Platform
{

Window::Window()
    : mWindow(nullptr), mContext(nullptr), mWidth(0), mHeight(0), mRunning(false), mResized(false),
      mMinimized(false), mFullscreen(false), mRelativeMouseMode(false), mCurrent(0.0),
      mPrevious(0.0), mUpdate(0.0), mDraw(0.0), mFrame(0.0), mTarget(0.0), mReady(false),
      mCloseKey(SDLK_ESCAPE), mMonitor(0), mDebugContext(false), mSdlInitialized(false),
      mFpsHistory(), mFpsHistoryIndex(0), mFpsAverage(0.0f), mFpsLastSampleTime(0.0)
{
}

Window::~Window()
{
    destroy();
}

bool Window::create(const std::string& title, int width, int height, int monitor, bool resizable,
                    bool fullscreen, bool visible)
{
#if defined(SDL_HINT_DBUS)
    SDL_SetHint(SDL_HINT_DBUS, "0");
#endif

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0)
    {
        Log::error("SDL_Init failed: %s", SDL_GetError());
        return false;
    }
    mSdlInitialized = true;

    mMonitor = monitor;

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 5);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    if (mDebugContext)
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);

    int monitorCount = SDL_GetNumVideoDisplays();
    if (monitorCount < 1)
    {
        Log::error("SDL_GetNumVideoDisplays failed: %s", SDL_GetError());
        monitorCount = 1;
    }
    if (mMonitor < 0 || mMonitor >= monitorCount)
    {
        Log::warning("Window::create: monitor %d out of range (0..%d), using 0", mMonitor,
                     monitorCount - 1);
        mMonitor = 0;
    }

    Uint32 windowFlags = SDL_WINDOW_OPENGL | (visible ? SDL_WINDOW_SHOWN : SDL_WINDOW_HIDDEN);
    if (resizable)
        windowFlags |= SDL_WINDOW_RESIZABLE;
    if (fullscreen)
        windowFlags |= SDL_WINDOW_FULLSCREEN_DESKTOP;

    mWindow =
        SDL_CreateWindow(title.c_str(), SDL_WINDOWPOS_CENTERED_DISPLAY(mMonitor),
                         SDL_WINDOWPOS_CENTERED_DISPLAY(mMonitor), width, height, windowFlags);
    if (!mWindow)
    {
        Log::error("SDL_CreateWindow failed: %s", SDL_GetError());
        destroy();
        return false;
    }
    mFullscreen = fullscreen;

    mContext = SDL_GL_CreateContext(static_cast<SDL_Window*>(mWindow));
    if (!mContext)
    {
        Log::error("SDL_GL_CreateContext failed: %s", SDL_GetError());
        destroy();
        return false;
    }

    SDL_GL_MakeCurrent(static_cast<SDL_Window*>(mWindow), mContext);
    SDL_GL_SetSwapInterval(1);

    mWidth = width;
    mHeight = height;
    mRunning = true;

    Input::init();
    mCurrent = getTime();
    mPrevious = mCurrent;
    mReady = true;

    return true;
}

void Window::destroy()
{
    if (!mWindow && !mContext && !mRunning && !mSdlInitialized)
        return;

    if (mContext)
    {
        SDL_GL_DeleteContext(mContext);
        mContext = nullptr;
    }
    if (mWindow)
    {
        SDL_DestroyWindow(static_cast<SDL_Window*>(mWindow));
        mWindow = nullptr;
    }
    if (mSdlInitialized)
    {
        SDL_Quit();
        mSdlInitialized = false;
    }
    mRunning = false;
    mReady = false;
}

bool Window::isOpen() const
{
    return mRunning;
}

bool Window::isMinimized() const
{
    return mMinimized;
}

bool Window::isMaximized() const
{
    if (!mWindow)
        return false;
    return (SDL_GetWindowFlags(static_cast<SDL_Window*>(mWindow)) & SDL_WINDOW_MAXIMIZED) != 0;
}

bool Window::hasFocus() const
{
    if (!mWindow)
        return false;
    return (SDL_GetWindowFlags(static_cast<SDL_Window*>(mWindow)) & SDL_WINDOW_INPUT_FOCUS) != 0;
}

void Window::update()
{
    if (!mReady)
        return;

    mCurrent = getTime();
    mUpdate = mCurrent - mPrevious;
    mPrevious = mCurrent;

    Input::update();

    SDL_Event e;
    while (SDL_PollEvent(&e))
    {
        if (e.type == SDL_QUIT)
            mRunning = false;
        if (e.type == SDL_KEYDOWN && e.key.keysym.sym == mCloseKey)
            mRunning = false;

        switch (e.type)
        {
        case SDL_KEYDOWN:
            Input::onKeyDown(e.key);
            break;
        case SDL_KEYUP:
            Input::onKeyUp(e.key);
            break;
        case SDL_MOUSEBUTTONDOWN:
            Input::onMouseDown(e.button);
            break;
        case SDL_MOUSEBUTTONUP:
            Input::onMouseUp(e.button);
            break;
        case SDL_MOUSEMOTION:
            Input::onMouseMove(e.motion);
            break;
        case SDL_MOUSEWHEEL:
            Input::onMouseWheel(e.wheel);
            break;
        case SDL_TEXTINPUT:
            Input::onTextInput(e.text);
            break;
        case SDL_FINGERDOWN:
            Input::onTouchDown(e.tfinger);
            break;
        case SDL_FINGERUP:
            Input::onTouchUp(e.tfinger);
            break;
        case SDL_FINGERMOTION:
            Input::onTouchMove(e.tfinger);
            break;
        default:
            break;
        }

        if (e.type == SDL_WINDOWEVENT)
        {
            switch (e.window.event)
            {
            case SDL_WINDOWEVENT_MINIMIZED:
                mMinimized = true;
                break;
            case SDL_WINDOWEVENT_RESTORED:
            case SDL_WINDOWEVENT_MAXIMIZED:
                mMinimized = false;
                break;
            case SDL_WINDOWEVENT_CLOSE:
                mRunning = false;
                break;
            case SDL_WINDOWEVENT_SIZE_CHANGED:
                if (e.window.data1 != mWidth || e.window.data2 != mHeight)
                {
                    mWidth = e.window.data1;
                    mHeight = e.window.data2;
                    mResized = true;
                }
                break;
            default:
                break;
            }
        }
    }

    // SDL can queue the window's creation resize event before a persisted
    // EngineSettings size is applied. Read the native window after all queued
    // events so that a stale event cannot put mWidth/mHeight back to the
    // launch defaults.
    if (mWindow)
    {
        int actualWidth = 0;
        int actualHeight = 0;
        SDL_GetWindowSize(static_cast<SDL_Window*>(mWindow), &actualWidth, &actualHeight);
        if (actualWidth > 0 && actualHeight > 0 &&
            (actualWidth != mWidth || actualHeight != mHeight))
        {
            mWidth = actualWidth;
            mHeight = actualHeight;
            mResized = true;
        }
    }
}

void Window::flip()
{
    if (!mReady)
        return;

    SDL_GL_SwapWindow(static_cast<SDL_Window*>(mWindow));

    mCurrent = getTime();
    mDraw = mCurrent - mPrevious;
    mPrevious = mCurrent;
    mFrame = mUpdate + mDraw;

    if (mTarget > 0.0 && mFrame < mTarget)
    {
        wait(static_cast<float>((mTarget - mFrame) * 1000.0));

        mCurrent = getTime();
        double waitTime = mCurrent - mPrevious;
        mPrevious = mCurrent;
        mFrame += waitTime;
    }

    const int kCaptureFrames = 30;
    const float kAverageTime = 0.5f;
    const float kStep = kAverageTime / kCaptureFrames;
    if (mFrame > 0.0 && (mCurrent - mFpsLastSampleTime) > kStep)
    {
        mFpsLastSampleTime = mCurrent;
        mFpsHistoryIndex = (mFpsHistoryIndex + 1) % kCaptureFrames;
        mFpsAverage -= mFpsHistory[mFpsHistoryIndex];
        mFpsHistory[mFpsHistoryIndex] = static_cast<float>(mFrame) / kCaptureFrames;
        mFpsAverage += mFpsHistory[mFpsHistoryIndex];
    }
}

void Window::setTargetFPS(int fps)
{
    mTarget = (fps < 1) ? 0.0 : 1.0 / fps;
}

int Window::getFPS()
{
    return mFpsAverage > 0.0f ? static_cast<int>(std::round(1.0f / mFpsAverage)) : 0;
}

double Window::getTime() const
{
    // Performance counter, not SDL_GetTicks(): every frame duration on this
    // clock - the delta the panel shows, the FPS average, the wait that
    // setTargetFPS() computes - came out quantised to whole milliseconds,
    // which at frame times in the twenties is a fifth of the value jumping
    // around for no reason the frame did anything about.
    return static_cast<double>(SDL_GetPerformanceCounter()) /
           static_cast<double>(SDL_GetPerformanceFrequency());
}

Uint32 Window::getTicks() const
{
    return SDL_GetTicks();
}

void Window::wait(float ms) const
{
    if (ms > 0.0f)
        SDL_Delay(static_cast<Uint32>(ms));
}

int Window::getWidth() const
{
    return mWidth;
}

int Window::getHeight() const
{
    return mHeight;
}

void Window::getDrawableSize(int& width, int& height) const
{
    width = 0;
    height = 0;
    if (mWindow)
        SDL_GL_GetDrawableSize(static_cast<SDL_Window*>(mWindow), &width, &height);
}

void Window::setSize(int width, int height)
{
    if (!mWindow || width <= 0 || height <= 0)
        return;
    SDL_SetWindowSize(static_cast<SDL_Window*>(mWindow), width, height);
    // Keep the engine-side logical size in sync with SDL. EngineSettings
    // reads these accessors when saving, and render/layout code uses them
    // before SDL sends the next resize event.
    mWidth = width;
    mHeight = height;
    mResized = true;
}

void Window::getPosition(int& x, int& y) const
{
    x = 0;
    y = 0;
    if (mWindow)
        SDL_GetWindowPosition(static_cast<SDL_Window*>(mWindow), &x, &y);
}

void Window::setPosition(int x, int y)
{
    if (mWindow)
        SDL_SetWindowPosition(static_cast<SDL_Window*>(mWindow), x, y);
}

bool Window::consumeResized()
{
    bool resized = mResized;
    mResized = false;
    return resized;
}

void Window::setTitle(const std::string& title)
{
    if (mWindow)
        SDL_SetWindowTitle(static_cast<SDL_Window*>(mWindow), title.c_str());
}

void Window::setFullscreen(bool fullscreen)
{
    if (!mWindow)
        return;
    if (SDL_SetWindowFullscreen(static_cast<SDL_Window*>(mWindow),
                                fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0) == 0)
    {
        mFullscreen = fullscreen;
    }
    else
    {
        Log::error("SDL_SetWindowFullscreen failed: %s", SDL_GetError());
    }
}

void Window::minimize()
{
    if (mWindow)
        SDL_MinimizeWindow(static_cast<SDL_Window*>(mWindow));
}

void Window::maximize()
{
    if (mWindow)
        SDL_MaximizeWindow(static_cast<SDL_Window*>(mWindow));
}

void Window::restore()
{
    if (mWindow)
        SDL_RestoreWindow(static_cast<SDL_Window*>(mWindow));
}

void Window::setVSync(bool enabled)
{
    if (SDL_GL_SetSwapInterval(enabled ? 1 : 0) != 0)
        Log::error("SDL_GL_SetSwapInterval failed: %s", SDL_GetError());
}

void Window::setRelativeMouseMode(bool enabled)
{
    if (SDL_SetRelativeMouseMode(enabled ? SDL_TRUE : SDL_FALSE) != 0)
        Log::error("SDL_SetRelativeMouseMode failed: %s", SDL_GetError());
    mRelativeMouseMode = enabled;
}

std::string Window::getClipboardText() const
{
    if (!SDL_HasClipboardText())
        return "";

    char* text = SDL_GetClipboardText();
    std::string result = text ? text : "";
    SDL_free(text);
    return result;
}

void Window::setClipboardText(const std::string& text)
{
    SDL_SetClipboardText(text.c_str());
}

void Window::setMonitor(int monitor)
{
    mMonitor = monitor;
    if (!mWindow)
        return;

    int monitorCount = SDL_GetNumVideoDisplays();
    if (monitor < 0 || monitor >= monitorCount)
        return;

    SDL_SetWindowPosition(static_cast<SDL_Window*>(mWindow),
                          SDL_WINDOWPOS_CENTERED_DISPLAY(monitor),
                          SDL_WINDOWPOS_CENTERED_DISPLAY(monitor));
}

int Window::getMonitorCount() const
{
    return SDL_GetNumVideoDisplays();
}

} // namespace Platform
} // namespace Radion
