#ifndef RADION_GPU_PROFILER_H
#define RADION_GPU_PROFILER_H

#include "Profiler.h"
#include "Types.h"

namespace Radion
{

// Per-pass GPU timing, to sit beside the CPU profiler's samples under the same
// names. The device already times the whole frame; this says where that time
// went, which is the only way a decision to turn an effect off is anything but
// a guess.
//
// Timestamps, not GL_TIME_ELAPSED: the device holds an elapsed-time query open
// across the entire frame, and those cannot nest. A pair of timestamps around
// a scope can, and costs the same.
//
// Results are read FrameDelay frames late, never in the frame that recorded
// them - asking the GPU for a result it has not reached yet stalls the CPU
// until it does, which would make the profiler the most expensive thing in the
// frame it is measuring.
class GPUProfiler
{
public:
    static constexpr u32 MaxSamples = 32;
    static constexpr u32 MaxDepth = 8;
    static constexpr u32 MaxScopesPerFrame = 64;
    static constexpr u32 FrameDelay = 3;

    static GPUProfiler& getSingleton();

    GPUProfiler(const GPUProfiler&) = delete;
    GPUProfiler& operator=(const GPUProfiler&) = delete;

    // Harvests whatever finished FrameDelay frames ago, then starts recording.
    void beginFrame();
    void endFrame();

    // Names must outlive the profiler - a literal, as with the CPU scopes.
    // Returns false when the frame is full or nesting is too deep, in which
    // case the matching end() must not run; ScopeGPU handles that.
    bool begin(const char* name);
    void end();

    // Releases the query objects. Must run while the GL context is alive.
    void shutdown();

    const ProfileSample* samples() const;
    u32 sampleCount() const;

    // Sum of the outermost scopes, which is what the panel totals.
    f32 frameMilliseconds() const;

    bool available() const;

private:
    GPUProfiler() = default;

    struct Scope
    {
        u32 sample = 0;
        u32 depth = 0;
        u32 beginQuery = 0;
        u32 endQuery = 0;
    };

    struct Frame
    {
        Scope scopes[MaxScopesPerFrame];
        u32 count = 0;
        bool pending = false;
    };

    u32 findOrCreate(const char* name);
    bool resultsReady(const Frame& frame) const;
    void harvest(Frame& frame);
    void pushHistory();

    ProfileSample mSamples[MaxSamples];
    Frame mFrames[FrameDelay];
    u32 mStack[MaxDepth];
    u64 mLastRefresh = 0;
    u32 mCursor = 0;
    u32 mDepth = 0;
    u32 mSampleCount = 0;
    f32 mFrameMilliseconds = 0.0f;
    f32 mDisplayFrameMilliseconds = 0.0f;
    bool mChecked = false;
    bool mAvailable = false;
};

class GPUProfileScope
{
public:
    explicit GPUProfileScope(const char* name);
    ~GPUProfileScope();

    GPUProfileScope(const GPUProfileScope&) = delete;
    GPUProfileScope& operator=(const GPUProfileScope&) = delete;

private:
    bool mActive;
};

#define RADION_GPU_PROFILE_JOIN_IMPL(a, b) a##b
#define RADION_GPU_PROFILE_JOIN(a, b) RADION_GPU_PROFILE_JOIN_IMPL(a, b)
#define RADION_GPU_PROFILE_SCOPE(name)                                                             \
    ::Radion::GPUProfileScope RADION_GPU_PROFILE_JOIN(radionGpuProfileScope, __LINE__)(name)

} // namespace Radion

#endif // RADION_GPU_PROFILER_H
