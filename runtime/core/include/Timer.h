#ifndef RADION_TIMER_H
#define RADION_TIMER_H

namespace Radion
{

class Timer
{
public:
    Timer();

    void reset();

    // Call exactly once per frame, before getDeltaTime(). No-op while paused.
    void tick();

    float getDeltaTime() const
    {
        return mDeltaTime;
    }

    // Seconds since reset(), excluding any time spent paused.
    double getElapsedTime() const
    {
        return mElapsedTime;
    }

    // While paused, tick() keeps dt at 0 and elapsed time frozen, so callers need not skip tick().
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
