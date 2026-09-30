#include "PCH.h"

#include "Thread.h"

#include "Log.h"

#include <SDL2/SDL.h>

namespace Radion
{

Mutex::Mutex()
{
    mHandle = SDL_CreateMutex();
    if (!mHandle)
        Log::error("Mutex: SDL_CreateMutex failed (%s)", SDL_GetError());
}

Mutex::~Mutex()
{
    if (mHandle)
        SDL_DestroyMutex(mHandle);
}

void Mutex::lock()
{
    if (mHandle)
        SDL_LockMutex(mHandle);
}

void Mutex::unlock()
{
    if (mHandle)
        SDL_UnlockMutex(mHandle);
}

ConditionVariable::ConditionVariable()
{
    mHandle = SDL_CreateCond();
    if (!mHandle)
        Log::error("ConditionVariable: SDL_CreateCond failed (%s)", SDL_GetError());
}

ConditionVariable::~ConditionVariable()
{
    if (mHandle)
        SDL_DestroyCond(mHandle);
}

void ConditionVariable::wait(Mutex& mutex)
{
    if (mHandle && mutex.mHandle)
        SDL_CondWait(mHandle, mutex.mHandle);
}

void ConditionVariable::signalOne()
{
    if (mHandle)
        SDL_CondSignal(mHandle);
}

void ConditionVariable::signalAll()
{
    if (mHandle)
        SDL_CondBroadcast(mHandle);
}

Thread::Thread()
{
}

Thread::~Thread()
{
    join();
}

int Thread::trampoline(void* self)
{
    Thread& thread = *static_cast<Thread*>(self);
    if (thread.mEntry)
        thread.mEntry(thread.mUserData);
    return 0;
}

bool Thread::start(Entry entry, void* userData, const char* name)
{
    if (mHandle || !entry)
        return false;
    mEntry = entry;
    mUserData = userData;
    mHandle = SDL_CreateThread(&Thread::trampoline, name ? name : "radion.thread", this);
    if (!mHandle)
    {
        Log::error("Thread: could not start '%s' (%s)", name ? name : "?", SDL_GetError());
        mEntry = nullptr;
        mUserData = nullptr;
        return false;
    }
    return true;
}

void Thread::join()
{
    if (!mHandle)
        return;
    SDL_WaitThread(mHandle, nullptr);
    mHandle = nullptr;
    mEntry = nullptr;
    mUserData = nullptr;
}

ThreadPool::ThreadPool()
{
}

ThreadPool::~ThreadPool()
{
    stop();
}

u32 ThreadPool::hardwareThreads()
{
    const int count = SDL_GetCPUCount();
    return count > 0 ? static_cast<u32>(count) : 1u;
}

bool ThreadPool::start(u32 workerCount)
{
    if (mWorkerCount != 0)
        return false;
    if (workerCount == 0)
    {
        // One per core minus the caller; oversubscribing costs more in contention than it wins.
        const u32 cores = hardwareThreads();
        workerCount = cores > 1 ? cores - 1 : 1;
    }
    workerCount = workerCount < kMaxWorkers ? workerCount : kMaxWorkers;

    {
        ScopedLock lock(mMutex);
        mStopping = false;
        mHead = 0;
        mTail = 0;
        mQueued = 0;
        mActive = 0;
    }

    char name[32];
    for (u32 i = 0; i < workerCount; ++i)
    {
        SDL_snprintf(name, sizeof(name), "radion.pool.%u", i);
        if (!mWorkers[i].start(&ThreadPool::workerMain, this, name))
        {
            // Shut down whatever did start, or running workers keep the process alive.
            mWorkerCount = i;
            stop();
            return false;
        }
    }
    mWorkerCount = workerCount;
    return true;
}

void ThreadPool::stop()
{
    if (mWorkerCount == 0)
        return;
    {
        ScopedLock lock(mMutex);
        mStopping = true;
    }
    // Broadcast, not signal: every worker sleeps on this condition and must see the flag.
    mWork.signalAll();
    for (u32 i = 0; i < mWorkerCount; ++i)
        mWorkers[i].join();
    mWorkerCount = 0;
}

bool ThreadPool::running() const
{
    return mWorkerCount != 0;
}

void ThreadPool::enqueue(Job job, void* userData)
{
    enqueueInternal(nullptr, job, userData);
}

void ThreadPool::enqueue(JobGroup& group, Job job, void* userData)
{
    enqueueInternal(&group, job, userData);
}

void ThreadPool::enqueueInternal(JobGroup* group, Job job, void* userData)
{
    if (!job)
        return;
    if (mWorkerCount == 0)
    {
        // Nowhere to queue it: run it here rather than drop it.
        job(userData);
        return;
    }

    bool full = false;
    {
        ScopedLock lock(mMutex);
        full = mQueued == kQueueCapacity;
        if (!full)
        {
            // Counted before the job can run, so a wait() between enqueue and pickup sees it.
            if (group)
                ++group->pending;
            mQueue[mTail] = {job, userData, group};
            mTail = (mTail + 1) % kQueueCapacity;
            ++mQueued;
        }
    }
    if (full)
    {
        // Run it here rather than resize the ring or lose it; shows as a stall on the caller.
        // Outside the lock: running arbitrary work under the pool mutex deadlocks jobs that enqueue jobs.
        job(userData);
        return;
    }
    mWork.signalOne();
}

void ThreadPool::runWorker()
{
    for (;;)
    {
        Entry entry;
        {
            ScopedLock lock(mMutex);
            while (mQueued == 0 && !mStopping)
                mWork.wait(mMutex);
            // Stop only after the queue drains, so stop() finishes what was asked.
            if (mQueued == 0 && mStopping)
                return;
            entry = mQueue[mHead];
            mHead = (mHead + 1) % kQueueCapacity;
            --mQueued;
            // Counted active BEFORE the lock drops: between taking and running the job the queue is empty, and wait() would call that done.
            ++mActive;
        }

        entry.job(entry.userData);

        {
            ScopedLock lock(mMutex);
            --mActive;
            if (entry.group && entry.group->pending > 0)
                --entry.group->pending;
            // One condition for both waits; each waiter re-checks its own predicate.
            if (mQueued == 0 && mActive == 0)
                mIdle.signalAll();
            else if (entry.group)
                mIdle.signalAll();
        }
    }
}

void ThreadPool::workerMain(void* self)
{
    static_cast<ThreadPool*>(self)->runWorker();
}

void ThreadPool::wait()
{
    if (mWorkerCount == 0)
        return;
    ScopedLock lock(mMutex);
    while (mQueued != 0 || mActive != 0)
        mIdle.wait(mMutex);
}

void ThreadPool::wait(JobGroup& group)
{
    if (mWorkerCount == 0)
        return;
    ScopedLock lock(mMutex);
    while (group.pending != 0)
        mIdle.wait(mMutex);
}

bool ThreadPool::finished(const JobGroup& group) const
{
    ScopedLock lock(mMutex);
    return group.pending == 0;
}

u32 ThreadPool::pending() const
{
    ScopedLock lock(mMutex);
    return mQueued + mActive;
}

ThreadPool& jobPool()
{
    static ThreadPool pool;
    return pool;
}

ThreadPool& Jobs()
{
    ThreadPool& pool = jobPool();
    if (!pool.running())
        pool.start();
    return pool;
}

void shutdownJobs()
{
    jobPool().stop();
}

} // namespace Radion
