#include "PCH.h"

#include "GPUProfiler.h"

#include "GPUCaps.h"
#include "GPUContext.h"

#include <glad.h>

#include <SDL.h>

namespace Radion
{

GPUProfiler& GPUProfiler::getSingleton()
{
    static GPUProfiler profiler;
    return profiler;
}

bool GPUProfiler::available() const
{
    return mAvailable;
}

u32 GPUProfiler::findOrCreate(const char* name)
{
    for (u32 i = 0; i < mSampleCount; ++i)
    {
        if (mSamples[i].name == name)
            return i;
    }
    if (mSampleCount >= MaxSamples)
        return MaxSamples;

    mSamples[mSampleCount].name = name;
    return mSampleCount++;
}

// A scope's time is the difference between its two timestamps. Nested scopes
// keep their own full span rather than being subtracted out of the parent, so
// the numbers read the same way the CPU samples do.
//
// Replaces the visible values outright rather than adding to them: the panel
// shows one frame's worth, not a running total since startup.
void GPUProfiler::harvest(Frame& frame)
{
    for (u32 i = 0; i < mSampleCount; ++i)
        mSamples[i].milliseconds = 0.0f;
    mFrameMilliseconds = 0.0f;

    for (u32 i = 0; i < frame.count; ++i)
    {
        const Scope& scope = frame.scopes[i];
        GLuint64 begin = 0;
        GLuint64 end = 0;
        glGetQueryObjectui64v(scope.beginQuery, GL_QUERY_RESULT, &begin);
        glGetQueryObjectui64v(scope.endQuery, GL_QUERY_RESULT, &end);
        if (end < begin)
            continue;

        const f32 milliseconds = static_cast<f32>(end - begin) / 1000000.0f;
        mSamples[scope.sample].milliseconds += milliseconds;

        // Only the outermost scopes add up to the frame: counting a nested one
        // as well would bill the same microsecond twice.
        if (scope.depth == 0)
            mFrameMilliseconds += milliseconds;
    }

    pushHistory();
}

// One availability check on the last query written: the GPU retires them in
// order, so if the last has landed every earlier one has too.
bool GPUProfiler::resultsReady(const Frame& frame) const
{
    if (frame.count == 0)
        return true;
    GLint ready = GL_FALSE;
    glGetQueryObjectiv(frame.scopes[frame.count - 1].endQuery, GL_QUERY_RESULT_AVAILABLE, &ready);
    return ready == GL_TRUE;
}

void GPUProfiler::pushHistory()
{
    // Same cadence as the CPU profiler's, on its own clock - these frames
    // are harvested FrameDelay late and only when their results have landed,
    // so the two tables cannot be tied to the same tick without one of them
    // waiting on the other. See ProfileSample::display.
    const u64 now = SDL_GetPerformanceCounter();
    const bool refresh =
        (now - mLastRefresh) > static_cast<u64>(Profiler::RefreshSeconds *
                                                static_cast<f64>(SDL_GetPerformanceFrequency()));
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

void GPUProfiler::beginFrame()
{
    if (!mChecked)
    {
        mChecked = true;
        mAvailable = isGPUContextAlive() && gpuCaps().timerQuery;
    }
    if (!mAvailable)
        return;

    mDepth = 0;
    mCursor = (mCursor + 1) % FrameDelay;

    Frame& frame = mFrames[mCursor];

    // The slot coming back round holds what was recorded FrameDelay frames
    // ago. If it has not landed yet, leave it alone and record nothing this
    // frame - reusing a query object still in flight loses the result. The
    // panel keeps showing the last complete set rather than flickering.
    if (frame.pending)
    {
        if (!resultsReady(frame))
            return;
        harvest(frame);
        frame.pending = false;
    }

    frame.count = 0;
}

void GPUProfiler::endFrame()
{
    if (!mAvailable)
        return;

    Frame& frame = mFrames[mCursor];
    frame.pending = frame.count > 0;
}

bool GPUProfiler::begin(const char* name)
{
    if (!mAvailable || !name || mDepth >= MaxDepth)
        return false;

    Frame& frame = mFrames[mCursor];
    if (frame.pending || frame.count >= MaxScopesPerFrame)
        return false;

    const u32 sample = findOrCreate(name);
    if (sample >= MaxSamples)
        return false;

    Scope& scope = frame.scopes[frame.count];
    if (scope.beginQuery == 0)
    {
        GLuint queries[2] = {0, 0};
        glGenQueries(2, queries);
        scope.beginQuery = queries[0];
        scope.endQuery = queries[1];
    }
    scope.sample = sample;
    scope.depth = mDepth;

    glQueryCounter(scope.beginQuery, GL_TIMESTAMP);
    mStack[mDepth++] = frame.count++;
    return true;
}

void GPUProfiler::end()
{
    if (!mAvailable || mDepth == 0)
        return;

    Frame& frame = mFrames[mCursor];
    const u32 index = mStack[--mDepth];
    glQueryCounter(frame.scopes[index].endQuery, GL_TIMESTAMP);
}

void GPUProfiler::shutdown()
{
    if (isGPUContextAlive())
    {
        for (Frame& frame : mFrames)
        {
            for (Scope& scope : frame.scopes)
            {
                if (scope.beginQuery == 0)
                    continue;
                const GLuint queries[2] = {scope.beginQuery, scope.endQuery};
                glDeleteQueries(2, queries);
                scope.beginQuery = 0;
                scope.endQuery = 0;
            }
        }
    }

    for (Frame& frame : mFrames)
    {
        frame.count = 0;
        frame.pending = false;
    }
    mSampleCount = 0;
    mDepth = 0;
    mFrameMilliseconds = 0.0f;
    mChecked = false;
    mAvailable = false;
}

const ProfileSample* GPUProfiler::samples() const
{
    return mSamples;
}

u32 GPUProfiler::sampleCount() const
{
    return mSampleCount;
}

f32 GPUProfiler::frameMilliseconds() const
{
    return mDisplayFrameMilliseconds;
}

GPUProfileScope::GPUProfileScope(const char* name)
    : mActive(GPUProfiler::getSingleton().begin(name))
{
}

GPUProfileScope::~GPUProfileScope()
{
    if (mActive)
        GPUProfiler::getSingleton().end();
}

} // namespace Radion
