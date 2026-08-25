#ifndef RADION_ASYNC_TEXTURE_LOADER_H
#define RADION_ASYNC_TEXTURE_LOADER_H

#include "GPU.h"
#include "Material.h"

#include <string>
#include <vector>

struct SDL_Thread;
struct SDL_mutex;
struct SDL_cond;

namespace Radion
{


class AsyncTextureLoader
{
public:
    static AsyncTextureLoader& getSingleton();

    void start();
    void shutdown();

    void enqueue(TextureHandle placeholder, const std::string& filename, ColorSpace space,
                bool generateMips, u32 mipLimit);

    // Main thread only. Uploads whatever finished decoding since the last
    // call and returns how many textures it uploaded.
    u32 processCompleted();

    // Jobs handed to enqueue() that have not been picked up by the worker
    // yet, plus the one (if any) currently decoding - what a "Loading N
    // textures..." indicator reads.
    u32 pendingCount() const;

    // Decoded and waiting for the next processCompleted() to upload them.
    // Not part of pendingCount(), which counts what the worker still owes;
    // a caller that must not proceed until every texture is really on the
    // GPU - a loading screen deciding when to hand over to the game - needs
    // both to be zero, or it stops one frame short and shows placeholders.
    u32 completedCount() const;

private:
    AsyncTextureLoader() = default;
    ~AsyncTextureLoader() = default;
    AsyncTextureLoader(const AsyncTextureLoader&) = delete;
    AsyncTextureLoader& operator=(const AsyncTextureLoader&) = delete;

    struct Job
    {
        TextureHandle placeholder;
        std::string filename;
        ColorSpace space = ColorSpace::sRGB;
        bool generateMips = true;
        u32 mipLimit = 0;
    };

    struct Result;

    static int threadMain(void* self);
    void workerLoop();

    SDL_Thread* mThread = nullptr;
    SDL_mutex* mJobsMutex = nullptr;
    SDL_mutex* mResultsMutex = nullptr;
    SDL_cond* mJobsCond = nullptr;
    std::vector<Job> mJobs;
    std::vector<Result>* mResults = nullptr; // pimpl'd: Result owns a TextureDecode.h type
    bool mRunning = false;
    volatile bool mInFlight = false; // true while the worker holds a job outside mJobs
};

} // namespace Radion

#endif // RADION_ASYNC_TEXTURE_LOADER_H
