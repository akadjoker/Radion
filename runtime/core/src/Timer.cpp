#include "PCH.h"

/** ****************************************************************************
  Radion Platform - Timer implementation (SDL2 backend).
**************************************************************************** */
#include "Timer.h"

#include <SDL2/SDL.h>

namespace Radion
{

namespace
{

double getTime()
{
    return static_cast<double>(SDL_GetPerformanceCounter()) /
           static_cast<double>(SDL_GetPerformanceFrequency());
}

} // namespace

Timer::Timer()
{
    reset();
}

void Timer::reset()
{
    mLastTime = getTime();
    mElapsedTime = 0.0;
    mDeltaTime = 0.0f;
    mPaused = false;
}

void Timer::tick()
{
    const double now = getTime();

    if (mPaused)
    {
        // Keep mLastTime moving with real time so resume() doesn't see a
        // huge gap and report one giant dt for the whole paused interval.
        mLastTime = now;
        mDeltaTime = 0.0f;
        return;
    }

    const double dt = now - mLastTime;
    mLastTime = now;
    mDeltaTime = static_cast<float>(dt);
    mElapsedTime += dt;
}

void Timer::pause()
{
    mPaused = true;
    mDeltaTime = 0.0f;
}

void Timer::resume()
{
    mPaused = false;
    mLastTime = getTime();
}

} // namespace Radion
