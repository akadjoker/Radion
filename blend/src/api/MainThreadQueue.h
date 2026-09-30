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

// All editor state belongs to the frame-loop thread; request threads queue work here and block until drain() runs it.
class MainThreadQueue
{
public:
    using Task = std::function<CommandOutcome()>;

    // Any thread but the draining one. Timeout drops the task (never runs late); Unavailable once closed.
    CommandOutcome run(Task task, std::chrono::milliseconds timeout);

    size_t drain();

    // Fails what is still queued; call before the editor is torn down so no request thread waits forever.
    void close();
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
