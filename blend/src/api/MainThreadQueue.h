#ifndef RADION_BLENDER_API_MAIN_THREAD_QUEUE_H
#define RADION_BLENDER_API_MAIN_THREAD_QUEUE_H

#include "api/CommandRegistry.h"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>

namespace Radion::BlenderApi
{

// The editor is single-threaded: mesh data, the undo stacks and the GL context
// all belong to the thread that runs the frame loop. HTTP requests arrive on
// other threads, so they hand their work here instead of touching any of it.
//
// A request thread calls run() and blocks; the frame loop calls drain() once a
// frame, which executes what has queued up, in arrival order, and wakes the
// waiting threads.
class MainThreadQueue
{
public:
    using Task = std::function<CommandOutcome()>;

    // Any thread except the one that drains. Returns Timeout when the frame
    // loop did not get to the task in time (the task is then dropped, never
    // run late), and Unavailable once the queue is closed.
    CommandOutcome run(Task task, std::chrono::milliseconds timeout);

    // The frame-loop thread. Returns how many tasks ran.
    size_t drain();

    // Refuses new work and fails what is still queued. Called before the
    // editor is torn down so no request thread is left waiting on a loop that
    // is about to stop.
    void close();
    // Accepts work again after close(), for a server that is switched off and
    // back on while the editor keeps running.
    void open();

private:
    struct Job
    {
        Task task;
        std::mutex mutex;
        std::condition_variable done;
        bool started = false;
        bool finished = false;
        bool cancelled = false;
        CommandOutcome outcome;
    };

    static CommandOutcome failure(CommandStatus status, const char* message);

    std::mutex mMutex;
    std::deque<std::shared_ptr<Job>> mJobs;
    bool mClosed = false;
};

} // namespace Radion::BlenderApi

#endif // RADION_BLENDER_API_MAIN_THREAD_QUEUE_H
