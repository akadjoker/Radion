#ifndef RADION_LIGHTMAP_UNWRAP_JOB_H
#define RADION_LIGHTMAP_UNWRAP_JOB_H

#include "LightmapUnwrapper.h"
#include "Mesh.h"

#include <string>

struct SDL_Thread;
struct SDL_mutex;

namespace Radion
{

// Runs xatlas on its own thread so the editor keeps its frame; collect the result on the main thread.
class LightmapUnwrapJob
{
public:
    LightmapUnwrapJob();
    ~LightmapUnwrapJob();

    LightmapUnwrapJob(const LightmapUnwrapJob&) = delete;
    LightmapUnwrapJob& operator=(const LightmapUnwrapJob&) = delete;

    // Copies `input`: the caller's MeshData may be replaced or freed while this runs.
    bool start(const MeshData& input, const LightmapUnwrapSettings& settings);

    bool running() const;
    // Stops at the next progress report; running() stays true until the worker unwinds.
    void cancel();

    u32 percent() const;
    std::string stage() const;

    // True once when the worker finished and its result was moved into `output`; poll on the main thread.
    bool collect(MeshData& output, bool& succeeded);

    // Atlas of the last completed unwrap; survives collect().
    const LightmapUnwrapResult& result() const
    {
        return mResult;
    }

private:
    static int run(void* self);
    static bool onProgress(const char* stage, u32 percent, void* userData);

    SDL_Thread* mThread = nullptr;
    SDL_mutex* mMutex = nullptr;

    MeshData mInput;
    MeshData mOutput;
    LightmapUnwrapSettings mSettings;
    LightmapUnwrapResult mResult;

    std::string mStage;
    u32 mPercent = 0;
    bool mRunning = false;
    bool mFinished = false;
    bool mSucceeded = false;
    bool mCancelled = false;
};

} // namespace Radion

#endif // RADION_LIGHTMAP_UNWRAP_JOB_H
