#include "PCH.h"

#include "AsyncTextureLoader.h"

#include "Log.h"
#include "TextureDecode.h"

#include <SDL2/SDL.h>

namespace Radion
{

struct AsyncTextureLoader::Result
{
    TextureHandle placeholder;
    std::string filename;
    DecodedTexture decoded;
};

AsyncTextureLoader& AsyncTextureLoader::getSingleton()
{
    static AsyncTextureLoader instance;
    return instance;
}

void AsyncTextureLoader::start()
{
    if (mThread)
        return;

    mJobsMutex = SDL_CreateMutex();
    mResultsMutex = SDL_CreateMutex();
    mJobsCond = SDL_CreateCond();
    mResults = new std::vector<Result>();
    mRunning = true;
    mThread = SDL_CreateThread(&AsyncTextureLoader::threadMain, "radion.texture.loader", this);
}

void AsyncTextureLoader::shutdown()
{
    if (!mThread)
        return;

    SDL_LockMutex(mJobsMutex);
    mRunning = false;
    SDL_CondSignal(mJobsCond);
    SDL_UnlockMutex(mJobsMutex);

    SDL_WaitThread(mThread, nullptr);
    mThread = nullptr;

    // Whatever finished decoding but was never picked up by processCompleted()
    // still owns a Pixmap/DDSImage - free it now rather than leaking it.
    for (Result& result : *mResults)
        releaseDecodedTexture(result.decoded);

    delete mResults;
    mResults = nullptr;

    SDL_DestroyCond(mJobsCond);
    SDL_DestroyMutex(mResultsMutex);
    SDL_DestroyMutex(mJobsMutex);
    mJobsCond = nullptr;
    mResultsMutex = nullptr;
    mJobsMutex = nullptr;
    mJobs.clear();
}

void AsyncTextureLoader::enqueue(TextureHandle placeholder, const std::string& filename,
                                 ColorSpace space, bool generateMips, u32 mipLimit)
{
    if (!mThread)
        start();

    Job job;
    job.placeholder = placeholder;
    job.filename = filename;
    job.space = space;
    job.generateMips = generateMips;
    job.mipLimit = mipLimit;

    SDL_LockMutex(mJobsMutex);
    mJobs.push_back(job);
    SDL_CondSignal(mJobsCond);
    SDL_UnlockMutex(mJobsMutex);
}

u32 AsyncTextureLoader::pendingCount() const
{
    if (!mJobsMutex)
        return 0;
    SDL_LockMutex(mJobsMutex);
    const u32 count = static_cast<u32>(mJobs.size()) + (mInFlight ? 1u : 0u);
    SDL_UnlockMutex(mJobsMutex);
    return count;
}

u32 AsyncTextureLoader::completedCount() const
{
    if (!mResultsMutex || !mResults)
        return 0;
    SDL_LockMutex(mResultsMutex);
    const u32 count = static_cast<u32>(mResults->size());
    SDL_UnlockMutex(mResultsMutex);
    return count;
}

u32 AsyncTextureLoader::processCompleted()
{
    if (!mResults)
        return 0;

    std::vector<Result> finished;
    SDL_LockMutex(mResultsMutex);
    finished.swap(*mResults);
    SDL_UnlockMutex(mResultsMutex);

    for (Result& result : finished)
    {
        if (result.decoded.ok)
        {
            result.decoded.desc.debugName = result.filename.c_str();
            if (!GPU::getSingleton().replaceTexture(result.placeholder, result.decoded.desc))
                Log::error("AsyncTextureLoader: GPU upload failed for '%s'",
                          result.filename.c_str());
        }
        else
        {
            Log::error("AsyncTextureLoader: failed to load '%s'", result.filename.c_str());
        }
        releaseDecodedTexture(result.decoded);
    }
    return static_cast<u32>(finished.size());
}

int AsyncTextureLoader::threadMain(void* self)
{
    static_cast<AsyncTextureLoader*>(self)->workerLoop();
    return 0;
}

void AsyncTextureLoader::workerLoop()
{
    for (;;)
    {
        SDL_LockMutex(mJobsMutex);
        while (mRunning && mJobs.empty())
            SDL_CondWait(mJobsCond, mJobsMutex);
        if (!mRunning && mJobs.empty())
        {
            SDL_UnlockMutex(mJobsMutex);
            return;
        }
        Job job = mJobs.front();
        mJobs.erase(mJobs.begin());
        mInFlight = true;
        SDL_UnlockMutex(mJobsMutex);

        // Pure CPU work - file I/O and stb_image/DDS decoding, no GL calls -
        // is exactly what makes running this off the main thread safe. See
        // TextureDecode.h.
        Result result;
        result.placeholder = job.placeholder;
        result.filename = job.filename;
        result.decoded = decodeTextureFile(job.filename, job.space, job.generateMips, job.mipLimit);

        SDL_LockMutex(mResultsMutex);
        mResults->push_back(std::move(result));
        SDL_UnlockMutex(mResultsMutex);
        mInFlight = false;
    }
}

} // namespace Radion
