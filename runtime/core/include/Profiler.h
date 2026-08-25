#ifndef RADION_PROFILER_H
#define RADION_PROFILER_H

#include "Types.h"

#include <string>

namespace Radion
{

struct ProfileSample
{
    static constexpr u32 HistorySize = 120;

    // Owned, not a borrowed const char*: a call site that passes a temporary
    // (name.c_str() on a local std::string, a formatted label) used to leave
    // the profiler holding an address that outlived nothing, read back with
    // strcmp() on the next frame.
    std::string name;
    f32 milliseconds = 0.0f;
    // What a panel should show instead of `milliseconds`: the same value,
    // held still and replaced only every RefreshSeconds. At sixty frames a
    // second a per-frame number is not slow reading, it is unreadable - it
    // changes several times before the eye has finished one line of the
    // table, and the digits that move most are the ones worth looking at.
    // The measurement itself is untouched; only how often it is handed over.
    f32 display = 0.0f;
    f32 average = 0.0f;
    f32 maximum = 0.0f;
    f32 history[HistorySize] = {};
    u32 historyCount = 0;
    u32 historyCursor = 0;
};

class Profiler
{
public:
    static constexpr u32 MaxSamples = 32;
    static constexpr u32 MaxDepth = 16;
    // How often the values a panel reads are replaced - see ProfileSample.
    static constexpr f64 RefreshSeconds = 0.25;

    static Profiler& getSingleton();

    void beginFrame();
    void endFrame();
    bool begin(const char* name);
    void end();
    // A duration measured outside any scope of this profiler's own, billed
    // to `name` as if a scope had produced it. For the one thing a scope
    // cannot reach: work that happens after endFrame() closed the frame it
    // belongs to, and so can only be reported into the next one.
    void addSample(const char* name, f32 milliseconds);

    const ProfileSample* samples() const;
    u32 sampleCount() const;
    // Held still on the same cadence as ProfileSample::display, and taken
    // from the same frame as those, so the total and the rows agree.
    f32 frameMilliseconds() const;

private:
    struct ActiveScope
    {
        u32 sample = 0;
        u64 counter = 0;
    };

    u32 findOrCreate(const char* name);

    ProfileSample mSamples[MaxSamples];
    ActiveScope mStack[MaxDepth];
    u64 mFrameStart = 0;
    u64 mFrequency = 1;
    u64 mLastRefresh = 0;
    f32 mFrameMilliseconds = 0.0f;
    f32 mDisplayFrameMilliseconds = 0.0f;
    u32 mSampleCount = 0;
    u32 mDepth = 0;
    bool mOverflowWarned = false;
    bool mDepthOverflowWarned = false;
};

class ProfileScope
{
public:
    explicit ProfileScope(const char* name);
    ~ProfileScope();

    ProfileScope(const ProfileScope&) = delete;
    ProfileScope& operator=(const ProfileScope&) = delete;

private:
    bool mActive = false;
};

} // namespace Radion

#define RADION_PROFILE_JOIN_IMPL(a, b) a##b
#define RADION_PROFILE_JOIN(a, b) RADION_PROFILE_JOIN_IMPL(a, b)
#define RADION_PROFILE_SCOPE(name)                                                                 \
    ::Radion::ProfileScope RADION_PROFILE_JOIN(radionProfileScope, __LINE__)(name)

#endif // RADION_PROFILER_H
