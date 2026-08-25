/** ****************************************************************************
  Radion Platform - SDL desktop window and OpenGL context.
**************************************************************************** */
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

    // Requests a debug context. RenderDevice installs the OpenGL callback.
    // Call before create().
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

    // Updates input and pumps SDL events.
    void update();
    // Swaps the OpenGL buffers and applies the optional frame-rate cap.
    void flip();

    SDL_Window* getNativeWindow() const
    {
        return static_cast<SDL_Window*>(mWindow);
    }
    SDL_GLContext getGLContext() const
    {
        return mContext;
    }

    // Total time (update + draw + any Wait()) for the frame most recently
    // finished by flip(). Same value getFrameTime() would return.
    float getDeltaTime() const
    {
        return (float)mFrame;
    }
    float getFrameTime() const
    {
        return (float)mFrame;
    }

    // Caps the frame rate: flip() sleeps (SDL_Delay, via Wait()) at the
    // end of the frame if it finished early. fps < 1 means uncapped.
    void setTargetFPS(int fps);

    // Smoothed FPS - 30-sample / 0.5s rolling average of getFrameTime().
    int getFPS();

    // Seconds / milliseconds since SDL_Init (SDL_GetTicks()-based).
    double getTime() const;
    Uint32 getTicks() const;
    void wait(float ms) const;

    // SDL_Keycode (e.g. SDLK_ESCAPE) that closes the window when pressed,
    // checked by update(). Defaults to SDLK_ESCAPE.
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

    // Top-left of the window in desktop coordinates, so a session can be
    // restored where it was left rather than wherever the window manager
    // decides to put it next time.
    void getPosition(int& x, int& y) const;
    void setPosition(int x, int y);

    // True if the window size changed since the previous call to
    // consumeResized() (or since creation). Reading it clears the flag.
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

    // Hides the cursor and confines it to the window, delivering raw
    // relative motion (used for mouse-look while a look button is held).
    void setRelativeMouseMode(bool enabled);
    bool isRelativeMouseMode() const
    {
        return mRelativeMouseMode;
    }

    // SDL_GetClipboardText/SDL_SetClipboardText.
    std::string getClipboardText() const;
    void setClipboardText(const std::string& text);

    // Multi-monitor: which display (0, 1, ...) the window opens/moves to.
    // setMonitor() is callable before create() (remembered, used when the
    // window is first positioned) or after (moves the already-open window
    // there now). getMonitorCount() needs SDL_INIT_VIDEO, so only call it
    // after create().
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

    double mCurrent;  // GetTime() at the start of the frame most recently begun/finished
    double mPrevious; // GetTime() at the previous mark (rolls forward each Run()/Flip() step)
    double mUpdate;   // update()'s own cost: time spent polling/updating input
    double mDraw;     // time between update() returning and flip() being called
    double mFrame;    // total frame time: mUpdate + mDraw + any Wait()
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
