#ifndef RADION_TIMER_H
#define RADION_TIMER_H

namespace Radion
{

class Timer
{
public:
    Timer();

    // Restarts elapsed time at 0 and unpauses. Call once before the loop
    // starts (the constructor already does this).
    void reset();

    // Advances the clock by real time elapsed since the last tick() (or
    // reset(), for the first call). Call exactly once per frame, before
    // reading getDeltaTime(). No-op while paused - getDeltaTime() reads 0.
    void tick();

    // Seconds since the previous tick(), as of the most recent tick() call.
    float getDeltaTime() const
    {
        return mDeltaTime;
    }

    // Seconds since reset(), excluding any time spent paused.
    double getElapsedTime() const
    {
        return mElapsedTime;
    }

    // While paused, tick() keeps mDeltaTime at 0 and elapsed time frozen,
    // rather than the caller having to skip calling tick() itself (which
    // would otherwise make the NEXT tick() report a large dt covering the
    // whole paused interval).
    void pause();
    void resume();
    bool isPaused() const
    {
        return mPaused;
    }

private:
    double mLastTime;
    double mElapsedTime;
    float mDeltaTime;
    bool mPaused;
};

} // namespace Radion

#endif // RADION_TIMER_H
