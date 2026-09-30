#include "api/MainThreadQueue.h"

#include <utility>

namespace Radion::BlenderApi
{

CommandOutcome MainThreadQueue::failure(CommandStatus status, const char* message)
{
    CommandOutcome outcome;
    outcome.status = status;
    outcome.message = message;
    return outcome;
}

CommandOutcome MainThreadQueue::run(Task task, std::chrono::milliseconds timeout)
{
    auto job = std::make_shared<Job>();
    job->task = std::move(task);

    {
        std::lock_guard<std::mutex> lock(mMutex);
        if (mClosed)
            return failure(CommandStatus::Unavailable, "the editor is shutting down");
        mJobs.push_back(job);
    }

    std::unique_lock<std::mutex> lock(job->mutex);
    if (!job->done.wait_for(lock, timeout, [&] { return job->finished; }))
    {
        // Withdraw an unstarted task so a timed-out request never runs late; a running one must be waited for.
        if (!job->started)
        {
            job->cancelled = true;
            return failure(CommandStatus::Timeout,
                           "the editor did not respond in time (is a modal dialog open?)");
        }
        job->done.wait(lock, [&] { return job->finished; });
    }
    return std::move(job->outcome);
}

size_t MainThreadQueue::drain()
{
    std::deque<std::shared_ptr<Job>> batch;
    {
        std::lock_guard<std::mutex> lock(mMutex);
        batch.swap(mJobs);
    }

    size_t executed = 0;
    for (const std::shared_ptr<Job>& job : batch)
    {
        {
            std::lock_guard<std::mutex> lock(job->mutex);
            if (job->cancelled)
                continue;
            job->started = true;
        }

        CommandOutcome outcome;
        try
        {
            outcome = job->task();
        }
        catch (const std::exception& error)
        {
            outcome = failure(CommandStatus::Failed, error.what());
        }
        catch (...)
        {
            outcome = failure(CommandStatus::Failed, "unknown error");
        }

        {
            std::lock_guard<std::mutex> lock(job->mutex);
            job->outcome = std::move(outcome);
            job->finished = true;
        }
        job->done.notify_all();
        ++executed;
    }
    return executed;
}

void MainThreadQueue::open()
{
    std::lock_guard<std::mutex> lock(mMutex);
    mClosed = false;
}

void MainThreadQueue::close()
{
    std::deque<std::shared_ptr<Job>> pending;
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mClosed = true;
        pending.swap(mJobs);
    }

    for (const std::shared_ptr<Job>& job : pending)
    {
        {
            std::lock_guard<std::mutex> lock(job->mutex);
            if (job->cancelled)
                continue;
            job->outcome = failure(CommandStatus::Unavailable, "the editor is shutting down");
            job->finished = true;
        }
        job->done.notify_all();
    }
}

} // namespace Radion::BlenderApi
