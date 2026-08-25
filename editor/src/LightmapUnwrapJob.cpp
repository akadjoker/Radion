#include "PCH.h"

#include "LightmapUnwrapJob.h"

#include "Log.h"

#include <SDL2/SDL.h>

namespace Radion
{

LightmapUnwrapJob::LightmapUnwrapJob()
{
    mMutex = SDL_CreateMutex();
}

LightmapUnwrapJob::~LightmapUnwrapJob()
{
    if (mThread)
    {
        cancel();
        SDL_WaitThread(mThread, nullptr);
        mThread = nullptr;
    }
    if (mMutex)
        SDL_DestroyMutex(mMutex);
}

bool LightmapUnwrapJob::start(const MeshData& input, const LightmapUnwrapSettings& settings)
{
    if (!mMutex || running())
        return false;

    // A previous run's thread is finished but never joined - collect() only
    // takes the result. Joining here keeps exactly one handle alive.
    if (mThread)
    {
        SDL_WaitThread(mThread, nullptr);
        mThread = nullptr;
    }

    mInput = input;
    mOutput.clear();
    mSettings = settings;
    mSettings.progress = &LightmapUnwrapJob::onProgress;
    mSettings.progressUserData = this;

    SDL_LockMutex(mMutex);
    mStage = "starting";
    mPercent = 0;
    mRunning = true;
    mFinished = false;
    mSucceeded = false;
    mCancelled = false;
    SDL_UnlockMutex(mMutex);

    mThread = SDL_CreateThread(&LightmapUnwrapJob::run, "RadionUnwrap", this);
    if (!mThread)
    {
        Log::error("LightmapUnwrapJob: could not start a thread (%s)", SDL_GetError());
        SDL_LockMutex(mMutex);
        mRunning = false;
        SDL_UnlockMutex(mMutex);
        return false;
    }
    return true;
}

int LightmapUnwrapJob::run(void* self)
{
    LightmapUnwrapJob& job = *static_cast<LightmapUnwrapJob*>(self);

    LightmapUnwrapper unwrapper;
    MeshData result;
    LightmapUnwrapResult atlas;
    const bool ok = unwrapper.unwrap(job.mInput, result, job.mSettings, &atlas);

    SDL_LockMutex(job.mMutex);
    job.mResult = atlas;
    job.mOutput = std::move(result);
    job.mSucceeded = ok;
    job.mFinished = true;
    job.mRunning = false;
    SDL_UnlockMutex(job.mMutex);
    return 0;
}

bool LightmapUnwrapJob::onProgress(const char* stage, u32 percent, void* userData)
{
    LightmapUnwrapJob& job = *static_cast<LightmapUnwrapJob*>(userData);
    SDL_LockMutex(job.mMutex);
    if (stage)
        job.mStage = stage;
    job.mPercent = percent;
    const bool cancelled = job.mCancelled;
    SDL_UnlockMutex(job.mMutex);
    return !cancelled;
}

bool LightmapUnwrapJob::running() const
{
    if (!mMutex)
        return false;
    SDL_LockMutex(mMutex);
    const bool value = mRunning;
    SDL_UnlockMutex(mMutex);
    return value;
}

void LightmapUnwrapJob::cancel()
{
    if (!mMutex)
        return;
    SDL_LockMutex(mMutex);
    mCancelled = true;
    SDL_UnlockMutex(mMutex);
}

u32 LightmapUnwrapJob::percent() const
{
    if (!mMutex)
        return 0;
    SDL_LockMutex(mMutex);
    const u32 value = mPercent;
    SDL_UnlockMutex(mMutex);
    return value;
}

std::string LightmapUnwrapJob::stage() const
{
    if (!mMutex)
        return std::string();
    SDL_LockMutex(mMutex);
    const std::string value = mStage;
    SDL_UnlockMutex(mMutex);
    return value;
}

bool LightmapUnwrapJob::collect(MeshData& output, bool& succeeded)
{
    if (!mMutex)
        return false;
    SDL_LockMutex(mMutex);
    const bool finished = mFinished;
    if (finished)
    {
        output = std::move(mOutput);
        mOutput.clear();
        succeeded = mSucceeded;
        mFinished = false;
    }
    SDL_UnlockMutex(mMutex);

    if (finished)
    {
        SDL_WaitThread(mThread, nullptr);
        mThread = nullptr;
        mInput.clear();
    }
    return finished;
}

} // namespace Radion
