#ifndef RADION_WINDOW_H
#define RADION_WINDOW_H

#include <SDL2/SDL.h>
#include <string>

namespace Radion
{
namespace Platform
{

class Window
{
public:
    Window();
    ~Window();

    bool create(const std::string& title, int width, int height, int monitor = 0,
                bool resizable = true, bool fullscreen = false, bool visible = true);

    // Requests a debug context; call before create().
    void setDebugContext(bool enable = true)
    {
        mDebugContext = enable;
    }
    bool isDebugContext() const
    {
        return mDebugContext;
    }

    void destroy();

    bool isOpen() const;
    void requestClose()
    {
        mRunning = false;
    }
    bool isMinimized() const;
    bool isMaximized() const;
    bool hasFocus() const;

    void update();
    void flip();

    SDL_Window* getNativeWindow() const
    {
        return static_cast<SDL_Window*>(mWindow);
    }
    SDL_GLContext getGLContext() const
    {
        return mContext;
    }

    float getDeltaTime() const
    {
        return (float)mFrame;
    }
    float getFrameTime() const
    {
        return (float)mFrame;
    }

    // fps < 1 means uncapped.
    void setTargetFPS(int fps);

    // 30-sample / 0.5s rolling average.
    int getFPS();

    // Seconds / milliseconds since SDL_Init (SDL_GetTicks()-based).
    double getTime() const;
    Uint32 getTicks() const;
    void wait(float ms) const;

    // SDL_Keycode that closes the window; defaults to SDLK_ESCAPE.
    void setExitKey(Sint32 key)
    {
        mCloseKey = key;
    }
    Sint32 getExitKey() const
    {
        return mCloseKey;
    }

    int getWidth() const;
    int getHeight() const;
    void getDrawableSize(int& width, int& height) const;
    void setSize(int width, int height);

    // Top-left in desktop coordinates, so a session can be restored where it was left.
    void getPosition(int& x, int& y) const;
    void setPosition(int x, int y);

    // True if resized since the last call; reading clears the flag.
    bool consumeResized();

    void setTitle(const std::string& title);

    void setFullscreen(bool fullscreen);
    bool isFullscreen() const
    {
        return mFullscreen;
    }

    void minimize();
    void maximize();
    void restore();

    void setVSync(bool enabled);

    // Hides and confines the cursor, delivering raw relative motion (mouse-look).
    void setRelativeMouseMode(bool enabled);
    bool isRelativeMouseMode() const
    {
        return mRelativeMouseMode;
    }

    std::string getClipboardText() const;
    void setClipboardText(const std::string& text);

    // setMonitor() works before create() (remembered) or after (moves the window).
    // getMonitorCount() needs SDL_INIT_VIDEO, so call it after create().
    void setMonitor(int monitor);
    int getMonitor() const
    {
        return mMonitor;
    }
    int getMonitorCount() const;

private:
    void* mWindow;  // SDL_Window*
    void* mContext; // SDL_GLContext

    int mWidth, mHeight;
    bool mRunning;
    bool mResized;
    bool mMinimized;
    bool mFullscreen;
    bool mRelativeMouseMode;

    double mCurrent;
    double mPrevious;
    double mUpdate;
    double mDraw;
    double mFrame;
    double mTarget;   // seconds per frame requested via setTargetFPS(), 0 = uncapped
    bool mReady;
    Sint32 mCloseKey;
    int mMonitor;
    bool mDebugContext;
    bool mSdlInitialized;

    float mFpsHistory[30];
    int mFpsHistoryIndex;
    float mFpsAverage;
    double mFpsLastSampleTime;
};

} // namespace Platform
} // namespace Radion

#endif // RADION_WINDOW_H
