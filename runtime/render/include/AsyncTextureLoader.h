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

    // Main thread only. Uploads finished decodes; returns how many.
    u32 processCompleted();

    // Jobs not yet picked up by the worker, plus the one decoding.
    u32 pendingCount() const;

    // Decoded and awaiting upload; not counted in pendingCount(). A loading screen needs both at zero or it stops a frame short.
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
