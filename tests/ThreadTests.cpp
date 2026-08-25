#include "PCH.h"

#include "Thread.h"

#include <atomic>
#include <cstdio>
#include <vector>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "ThreadTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

// ------------------------------------------------------------------ thread

struct Counter
{
    u32 value = 0;
};

void bumpCounter(void* userData)
{
    static_cast<Counter*>(userData)->value += 1;
}

void testThreadRunsAndJoins()
{
    Counter counter;
    Thread thread;
    CHECK(!thread.joinable());
    CHECK(thread.start(&bumpCounter, &counter, "radion.test.one"));
    CHECK(thread.joinable());
    thread.join();
    CHECK(counter.value == 1);
    // Joining twice, and joining one that never started, must both be safe -
    // a destructor runs join() and cannot know which case it is in.
    thread.join();
    CHECK(!thread.joinable());

    Thread never;
    never.join();
    CHECK(!never.joinable());

    // A null entry is refused rather than starting a thread that crashes.
    Thread bad;
    CHECK(!bad.start(nullptr, nullptr, "radion.test.bad"));
}

// -------------------------------------------------------------------- pool

struct Accumulator
{
    std::atomic<u32> ran{0};
    std::atomic<u32> concurrent{0};
    std::atomic<u32> peakConcurrent{0};
};

void countJob(void* userData)
{
    Accumulator& state = *static_cast<Accumulator*>(userData);
    const u32 now = state.concurrent.fetch_add(1) + 1;
    u32 peak = state.peakConcurrent.load();
    while (now > peak && !state.peakConcurrent.compare_exchange_weak(peak, now))
    {
    }
    // Long enough that several workers really do overlap, short enough that
    // the test stays quick.
    for (volatile u32 spin = 0; spin < 20000; ++spin)
    {
    }
    state.ran.fetch_add(1);
    state.concurrent.fetch_sub(1);
}

void testPoolRunsEverything()
{
    ThreadPool pool;
    CHECK(!pool.running());
    CHECK(pool.start(4));
    CHECK(pool.running());
    CHECK(pool.workerCount() == 4);

    Accumulator state;
    constexpr u32 kJobs = 500;
    for (u32 i = 0; i < kJobs; ++i)
        pool.enqueue(&countJob, &state);
    pool.wait();

    // Every job ran exactly once, and wait() really waited: a queue that is
    // empty while a worker is still inside a job is not finished, and this is
    // what catches a wait() that only looks at the queue.
    CHECK(state.ran.load() == kJobs);
    CHECK(pool.pending() == 0);
    CHECK(state.concurrent.load() == 0);
    // With four workers and 500 jobs, at least two must have overlapped, or
    // the pool is running everything on one thread.
    CHECK(state.peakConcurrent.load() > 1);

    pool.stop();
    CHECK(!pool.running());
}

void testPoolWithoutStartStillRuns()
{
    // A pool nobody started must not swallow work - running it on the caller
    // is slow, but silently dropping it is a bug that surfaces far from here.
    ThreadPool pool;
    Counter counter;
    pool.enqueue(&bumpCounter, &counter);
    CHECK(counter.value == 1);
    pool.wait();
    CHECK(pool.pending() == 0);
}

void testPoolDrainsBeforeStopping()
{
    ThreadPool pool;
    CHECK(pool.start(2));

    Accumulator state;
    constexpr u32 kJobs = 200;
    for (u32 i = 0; i < kJobs; ++i)
        pool.enqueue(&countJob, &state);

    // stop() without wait(): the contract is that queued work still runs.
    // Throwing it away would lose whatever was in flight at shutdown.
    pool.stop();
    CHECK(state.ran.load() == kJobs);
}

void testPoolOverflowRunsOnCaller()
{
    // More jobs than the ring holds. The surplus has to run somewhere, and
    // the count at the end is what says none went missing.
    ThreadPool pool;
    CHECK(pool.start(2));

    Accumulator state;
    constexpr u32 kJobs = 4000; // well past kQueueCapacity
    for (u32 i = 0; i < kJobs; ++i)
        pool.enqueue(&countJob, &state);
    pool.wait();
    CHECK(state.ran.load() == kJobs);
    pool.stop();
}

void testPoolRestart()
{
    ThreadPool pool;
    CHECK(pool.start(2));
    Counter first;
    pool.enqueue(&bumpCounter, &first);
    pool.wait();
    pool.stop();

    // Starting again after a stop has to work - and starting one that is
    // already running has to be refused rather than leaking the first set.
    CHECK(pool.start(3));
    CHECK(pool.workerCount() == 3);
    CHECK(!pool.start(2));
    Counter second;
    pool.enqueue(&bumpCounter, &second);
    pool.wait();
    CHECK(second.value == 1);
    pool.stop();
}

void testDoubleStopIsSafe()
{
    ThreadPool pool;
    pool.start(2);
    pool.stop();
    pool.stop();
    CHECK(!pool.running());
}

// The pattern the whole thing exists for: work handed off, main thread free,
// result collected later. This is what the BVH double buffer and the lightmap
// unwrap both do.
struct Handoff
{
    Mutex mutex;
    u32 result = 0;
    bool done = false;
};

void produce(void* userData)
{
    Handoff& state = *static_cast<Handoff*>(userData);
    u32 total = 0;
    for (u32 i = 1; i <= 1000; ++i)
        total += i;
    ScopedLock lock(state.mutex);
    state.result = total;
    state.done = true;
}

void testHandoffPattern()
{
    Handoff state;
    ThreadPool pool;
    CHECK(pool.start(2));
    pool.enqueue(&produce, &state);

    // The main thread keeps working while that runs.
    u32 spun = 0;
    while (spun < 1000)
        ++spun;

    pool.wait();
    ScopedLock lock(state.mutex);
    CHECK(state.done);
    CHECK(state.result == 500500u);
    pool.stop();
}

void testJobGroupWaitsForItsOwnBatchOnly()
{
    ThreadPool pool;
    CHECK(pool.start(3));

    Accumulator mine;
    Accumulator theirs;
    JobGroup group;

    // One batch counted, another not. Waiting on the group must return as
    // soon as ITS jobs are done, whatever else the pool is still chewing on -
    // that is the whole difference from wait(), and what the BVH's
    // rebuild-in-the-background needs.
    for (u32 i = 0; i < 40; ++i)
        pool.enqueue(group, &countJob, &mine);
    for (u32 i = 0; i < 400; ++i)
        pool.enqueue(&countJob, &theirs);

    pool.wait(group);
    CHECK(mine.ran.load() == 40);
    CHECK(pool.finished(group));

    pool.wait();
    CHECK(theirs.ran.load() == 400);
    pool.stop();
}

void testJobGroupPollingWithoutBlocking()
{
    // The pattern the double-buffered BVH uses: fire it, carry on, and check
    // next frame. finished() must never block.
    ThreadPool pool;
    CHECK(pool.start(2));

    Accumulator state;
    JobGroup group;
    CHECK(pool.finished(group)); // never used, so already finished
    for (u32 i = 0; i < 30; ++i)
        pool.enqueue(group, &countJob, &state);

    u32 polls = 0;
    while (!pool.finished(group) && polls < 1000000)
        ++polls;
    CHECK(state.ran.load() == 30);

    // Reusable: the same group can carry the next batch.
    for (u32 i = 0; i < 10; ++i)
        pool.enqueue(group, &countJob, &state);
    pool.wait(group);
    CHECK(state.ran.load() == 40);
    pool.stop();
}

void testHardwareThreadsIsSane()
{
    const u32 count = ThreadPool::hardwareThreads();
    CHECK(count >= 1);
    CHECK(count < 4096);

    // The default worker count is one per core less the caller's own, and
    // never zero even on a single-core machine.
    ThreadPool pool;
    CHECK(pool.start());
    CHECK(pool.workerCount() >= 1);
    CHECK(pool.workerCount() <= (count > 1 ? count - 1 : 1));
    pool.stop();
}

} // namespace

int main()
{
    testThreadRunsAndJoins();
    testPoolRunsEverything();
    testPoolWithoutStartStillRuns();
    testPoolDrainsBeforeStopping();
    testPoolOverflowRunsOnCaller();
    testPoolRestart();
    testDoubleStopIsSafe();
    testHandoffPattern();
    testJobGroupWaitsForItsOwnBatchOnly();
    testJobGroupPollingWithoutBlocking();
    testHardwareThreadsIsSane();
    if (gFailures)
        std::fprintf(stderr, "%d thread test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
