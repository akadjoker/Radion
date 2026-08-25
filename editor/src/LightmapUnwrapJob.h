#ifndef RADION_LIGHTMAP_UNWRAP_JOB_H
#define RADION_LIGHTMAP_UNWRAP_JOB_H

#include "LightmapUnwrapper.h"
#include "Mesh.h"

#include <string>

struct SDL_Thread;
struct SDL_mutex;

namespace Radion
{

// xatlas is one long synchronous call with no way to give the caller a turn -
// the progress callback and the cancel it already supports are useless while
// the thread that would draw them is the one inside Generate(). This runs it
// on a thread of its own so the editor keeps its frame; nothing in here
// touches the GPU, so the only rule is that the result is collected on the
// main thread.
//
// Not a general job system. When there is a second consumer, this is what
// gets extracted into the Thread/ThreadPool that runtime/core still wants.
class LightmapUnwrapJob
{
public:
    LightmapUnwrapJob();
    ~LightmapUnwrapJob();

    LightmapUnwrapJob(const LightmapUnwrapJob&) = delete;
    LightmapUnwrapJob& operator=(const LightmapUnwrapJob&) = delete;

    // Copies `input` - the caller's MeshData may be replaced or freed while
    // this runs, and routinely is, since finishing one unwrap is what
    // replaces it.
    bool start(const MeshData& input, const LightmapUnwrapSettings& settings);

    bool running() const;
    // Asks the worker to stop at its next progress report. It does not return
    // immediately; running() stays true until it actually unwinds.
    void cancel();

    u32 percent() const;
    std::string stage() const;

    // True once, when the worker has finished and its result has been moved
    // into `output`. `succeeded` says whether it produced anything. Poll this
    // from the main thread every frame while running().
    bool collect(MeshData& output, bool& succeeded);

    // The atlas the last completed unwrap actually produced. Survives the
    // collect, so the panel can keep warning about a bake resolution that
    // does not match it.
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
