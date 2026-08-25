#include "PCH.h"

#include "Thread.h"

#include "Log.h"

#include <SDL2/SDL.h>

namespace Radion
{

// ------------------------------------------------------------------- mutex

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

// --------------------------------------------------------------- condition

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

// ------------------------------------------------------------------ thread

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

// -------------------------------------------------------------------- pool

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
        // One per core less the caller's own. The main thread is doing work
        // too, and a worker per core on top of it spends more on contention
        // than it wins.
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
            // Whatever did start still has to be shut down, or the ones
            // running keep the process alive with nobody left to stop them.
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
    // Broadcast, not signal: every worker is asleep on the same condition and
    // all of them have to be woken to see the flag.
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
        // Nowhere to put it, so run it here rather than dropping it. A pool
        // that was never started must not silently swallow work.
        job(userData);
        return;
    }

    bool full = false;
    {
        ScopedLock lock(mMutex);
        full = mQueued == kQueueCapacity;
        if (!full)
        {
            // Counted before the job can possibly run, so a wait() that lands
            // between the enqueue and the worker picking it up still sees it
            // as outstanding.
            if (group)
                ++group->pending;
            mQueue[mTail] = {job, userData, group};
            mTail = (mTail + 1) % kQueueCapacity;
            ++mQueued;
        }
    }
    if (full)
    {
        // Run it here rather than resizing the ring under the workers, and
        // rather than losing it. It shows up as a stall on the calling
        // thread, which is the honest symptom of a queue that is too small.
        // Deliberately outside the lock - calling arbitrary work while
        // holding the pool's own mutex is how a job that enqueues another
        // job deadlocks the pool.
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
            // Stopping only after the queue is drained, so stop() finishes
            // what was asked for rather than throwing it away.
            if (mQueued == 0 && mStopping)
                return;
            entry = mQueue[mHead];
            mHead = (mHead + 1) % kQueueCapacity;
            --mQueued;
            // Counted as active BEFORE the lock is dropped: between taking
            // the job and running it the queue is empty, and a wait() that
            // only looked at the queue would call that done.
            ++mActive;
        }

        entry.job(entry.userData);

        {
            ScopedLock lock(mMutex);
            --mActive;
            if (entry.group && entry.group->pending > 0)
                --entry.group->pending;
            // One condition for both waits: a group finishing and the pool
            // going idle are both "something a waiter might be waiting for",
            // and every waiter re-checks its own predicate on waking.
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

ThreadPool& Jobs()
{
    static ThreadPool pool;
    if (!pool.running())
        pool.start();
    return pool;
}

} // namespace Radion
