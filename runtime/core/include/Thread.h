#ifndef RADION_THREAD_H
#define RADION_THREAD_H

#include "Types.h"

struct SDL_Thread;
struct SDL_mutex;
struct SDL_cond;

namespace Radion
{

 

class Mutex
{
public:
    Mutex();
    ~Mutex();

    Mutex(const Mutex&) = delete;
    Mutex& operator=(const Mutex&) = delete;

    void lock();
    void unlock();

    bool valid() const
    {
        return mHandle != nullptr;
    }

private:
    friend class ConditionVariable;
    SDL_mutex* mHandle = nullptr;
};

// Unlocks however the scope is left, including early return.
class ScopedLock
{
public:
    explicit ScopedLock(Mutex& mutex) : mMutex(mutex)
    {
        mMutex.lock();
    }
    ~ScopedLock()
    {
        mMutex.unlock();
    }

    ScopedLock(const ScopedLock&) = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;

private:
    Mutex& mMutex;
};

// The mutex must be held on entry to wait() and is held again on return, making check-and-sleep atomic.
class ConditionVariable
{
public:
    ConditionVariable();
    ~ConditionVariable();

    ConditionVariable(const ConditionVariable&) = delete;
    ConditionVariable& operator=(const ConditionVariable&) = delete;

    void wait(Mutex& mutex);
    void signalOne();
    void signalAll();

private:
    SDL_cond* mHandle = nullptr;
};

class Thread
{
public:
    using Entry = void (*)(void* userData);

    Thread();
    ~Thread();

    Thread(const Thread&) = delete;
    Thread& operator=(const Thread&) = delete;

    // `name` shows in debuggers and profilers, so it is required.
    bool start(Entry entry, void* userData, const char* name);
    // Blocks until the entry function returns. Safe to call on a thread that
    // was never started, and safe to call twice.
    void join();
    bool joinable() const
    {
        return mHandle != nullptr;
    }

private:
    static int trampoline(void* self);

    SDL_Thread* mHandle = nullptr;
    Entry mEntry = nullptr;
    void* mUserData = nullptr;
};

// A fixed set of workers pulling from one queue; deliberately not a job system (no dependencies, priorities or stealing).
// A batch of jobs that can be waited on alone: enqueue() raises the count, a worker lowers it per job.
// An atomic counter (like wi::jobsystem::context) rather than a future: it owns nothing, so no lifetime rule to forget.
struct JobGroup
{
    u32 pending = 0;
};

class ThreadPool
{
public:
    using Job = void (*)(void* userData);

    ThreadPool();
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // Zero workers = one per core minus the caller.
    bool start(u32 workerCount = 0);
    void stop();

    bool running() const;
    u32 workerCount() const
    {
        return mWorkerCount;
    }

    void enqueue(Job job, void* userData);
    // Counted against `group`; the group must outlive the job.
    void enqueue(JobGroup& group, Job job, void* userData);
    // Blocks until the queue is empty AND no worker is mid-job.
    void wait();
    void wait(JobGroup& group);
    bool finished(const JobGroup& group) const;
    u32 pending() const;

    static u32 hardwareThreads();

private:
    static void workerMain(void* self);
    void runWorker();
    void enqueueInternal(JobGroup* group, Job job, void* userData);

    struct Entry
    {
        Job job = nullptr;
        void* userData = nullptr;
        JobGroup* group = nullptr;
    };

    static constexpr u32 kMaxWorkers = 32;
    Thread mWorkers[kMaxWorkers];
    u32 mWorkerCount = 0;

    // Kept as a plain ring so enqueue() from the main thread never allocates.
    static constexpr u32 kQueueCapacity = 1024;
    Entry mQueue[kQueueCapacity];
    u32 mHead = 0;
    u32 mTail = 0;
    u32 mQueued = 0;
    u32 mActive = 0;

    mutable Mutex mMutex;
    ConditionVariable mWork;
    ConditionVariable mIdle;
    bool mStopping = false;
};

// The engine's global pool, started on first use; one per process so subsystems don't oversubscribe the machine.
ThreadPool& Jobs();
// Stops the global pool without starting it. Call before SDL shuts down (workers are SDL threads).
void shutdownJobs();

} // namespace Radion

#endif // RADION_THREAD_H
