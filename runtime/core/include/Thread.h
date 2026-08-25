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

// Locks on construction and unlocks however the scope is left - including
// through an early return, which is where a hand-written unlock gets missed.
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

// Sleeping until there is something to do, instead of spinning. The mutex
// must be held on entry to wait() and is held again on return, which is what
// makes checking the condition and going to sleep one atomic step - the gap
// between them is where a wakeup gets lost.
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

    // `name` shows up in a debugger and in a profiler, so it is required
    // rather than optional - an unnamed thread in a stack of eight is a
    // thread nobody can tell apart.
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

// A fixed set of workers pulling from one queue. Deliberately not a general
// job system: no dependencies, no priorities, no work stealing. What it does
// is take work off the main thread and say when it is done, which is what the
// two things that wanted it actually needed.
// A batch of jobs that can be waited on by itself. Held by whoever launched
// the batch; enqueue() raises the count and a worker lowers it as each job
// finishes, so wait() on one of these blocks for that batch alone rather than
// for everything the pool happens to be doing.
//
// This is the shape the reference uses (wi::jobsystem::context - an atomic
// counter and nothing else) rather than a typed future. A future would have
// to outlive the job that fills it, and with no smart pointers in this engine
// that is a lifetime rule waiting to be forgotten. A counter owns nothing:
// the result lives wherever the caller already keeps it.
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

    // Zero workers means "one per core, minus the one calling this" - the
    // main thread is doing work too, and oversubscribing costs more in
    // contention than it buys.
    bool start(u32 workerCount = 0);
    // Finishes what is queued, then stops the workers. The destructor does
    // this too.
    void stop();

    bool running() const;
    u32 workerCount() const
    {
        return mWorkerCount;
    }

    void enqueue(Job job, void* userData);
    // Same, but counted against `group` so it can be waited on alone. The
    // group must outlive the job - it is where the worker reports back.
    void enqueue(JobGroup& group, Job job, void* userData);
    // Blocks until the queue is empty AND no worker is still inside a job.
    // Both halves matter: an empty queue with a worker mid-job is not done.
    void wait();
    // Blocks until this batch alone is finished. Returns immediately for a
    // group that was never used or has already completed.
    void wait(JobGroup& group);
    bool finished(const JobGroup& group) const;
    // Jobs queued or running right now.
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

// The engine's own pool, started on first use. One per process, the same
// shape the reference uses (wi::jobsystem is global too) and the same shape
// as Assets() and DebugDraw() here: work that wants a thread should not have
// to be handed one, and a pool per subsystem would oversubscribe the machine
// several times over.
ThreadPool& Jobs();

} // namespace Radion

#endif // RADION_THREAD_H
