#include "PCH.h"

#include "Profiler.h"

#include "Log.h"

#include <SDL.h>

namespace Radion
{

Profiler& Profiler::getSingleton()
{
    static Profiler profiler;
    return profiler;
}

void Profiler::beginFrame()
{
    if (mFrequency == 1)
        mFrequency = SDL_GetPerformanceFrequency();
    for (u32 i = 0; i < mSampleCount; ++i)
        mSamples[i].milliseconds = 0.0f;
    mDepth = 0;
    mFrameStart = SDL_GetPerformanceCounter();
}

void Profiler::endFrame()
{
    const u64 now = SDL_GetPerformanceCounter();
    mFrameMilliseconds = static_cast<f32>((now - mFrameStart) * 1000.0 / mFrequency);
    const u32 frameSample = findOrCreate("Frame");
    if (frameSample < MaxSamples)
        mSamples[frameSample].milliseconds = mFrameMilliseconds;

    // One frame in four hundred milliseconds' worth is handed to the panel,
    // whole - every row from the same frame, so what they add up to is a
    // frame that actually happened rather than a mixture of several.
    const bool refresh = (now - mLastRefresh) > static_cast<u64>(RefreshSeconds * mFrequency);
    if (refresh)
    {
        mLastRefresh = now;
        mDisplayFrameMilliseconds = mFrameMilliseconds;
    }

    for (u32 i = 0; i < mSampleCount; ++i)
    {
        ProfileSample& sample = mSamples[i];
        if (refresh)
            sample.display = sample.milliseconds;
        sample.history[sample.historyCursor] = sample.milliseconds;
        sample.historyCursor = (sample.historyCursor + 1) % ProfileSample::HistorySize;
        if (sample.historyCount < ProfileSample::HistorySize)
            ++sample.historyCount;

        f32 total = 0.0f;
        sample.maximum = 0.0f;
        for (u32 j = 0; j < sample.historyCount; ++j)
        {
            total += sample.history[j];
            if (sample.history[j] > sample.maximum)
                sample.maximum = sample.history[j];
        }
        sample.average = sample.historyCount ? total / sample.historyCount : 0.0f;
    }
}

u32 Profiler::findOrCreate(const char* name)
{
    for (u32 i = 0; i < mSampleCount; ++i)
    {
        if (mSamples[i].name == name)
            return i;
    }
    if (mSampleCount >= MaxSamples)
    {
        if (!mOverflowWarned)
        {
            Log::warning("Profiler: MaxSamples (%u) exceeded; '%s' and further scopes will not "
                        "be tracked",
                        MaxSamples, name);
            mOverflowWarned = true;
        }
        return MaxSamples;
    }

    mSamples[mSampleCount].name = name;
    return mSampleCount++;
}

bool Profiler::begin(const char* name)
{
    if (!name)
        return false;
    if (mDepth >= MaxDepth)
    {
        if (!mDepthOverflowWarned)
        {
            Log::warning("Profiler: MaxDepth (%u) exceeded at '%s'; deeper scopes will not be "
                        "timed",
                        MaxDepth, name);
            mDepthOverflowWarned = true;
        }
        return false;
    }
    if (mFrequency == 1)
        mFrequency = SDL_GetPerformanceFrequency();
    const u32 sample = findOrCreate(name);
    if (sample >= MaxSamples)
        return false;
    mStack[mDepth++] = {sample, SDL_GetPerformanceCounter()};
    return true;
}

void Profiler::end()
{
    if (mDepth == 0)
        return;
    const u64 now = SDL_GetPerformanceCounter();
    const ActiveScope scope = mStack[--mDepth];
    mSamples[scope.sample].milliseconds +=
        static_cast<f32>((now - scope.counter) * 1000.0 / mFrequency);
}

void Profiler::addSample(const char* name, f32 milliseconds)
{
    if (!name)
        return;
    const u32 sample = findOrCreate(name);
    if (sample < MaxSamples)
        mSamples[sample].milliseconds += milliseconds;
}

const ProfileSample* Profiler::samples() const
{
    return mSamples;
}

u32 Profiler::sampleCount() const
{
    return mSampleCount;
}

f32 Profiler::frameMilliseconds() const
{
    return mDisplayFrameMilliseconds;
}

ProfileScope::ProfileScope(const char* name) : mActive(Profiler::getSingleton().begin(name))
{
}

ProfileScope::~ProfileScope()
{
    if (mActive)
        Profiler::getSingleton().end();
}

} // namespace Radion
