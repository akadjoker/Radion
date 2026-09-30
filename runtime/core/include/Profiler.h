#ifndef RADION_PROFILER_H
#define RADION_PROFILER_H

#include "Types.h"

#include <string>

namespace Radion
{

struct ProfileSample
{
    static constexpr u32 HistorySize = 120;

    // Owned, not const char*: a temporary name would leave a dangling pointer.
    std::string name;
    f32 milliseconds = 0.0f;
    // What a panel shows instead of `milliseconds`: held still, replaced every RefreshSeconds so it is readable.
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
    static constexpr f64 RefreshSeconds = 0.25;

    static Profiler& getSingleton();

    void beginFrame();
    void endFrame();
    bool begin(const char* name);
    void end();
    // Duration measured outside any scope, billed to `name`; for work finishing after endFrame().
    void addSample(const char* name, f32 milliseconds);

    const ProfileSample* samples() const;
    u32 sampleCount() const;
    // Held on the same cadence and frame as ProfileSample::display, so total and rows agree.
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
