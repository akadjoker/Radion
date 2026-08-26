#include "PCH.h"

#include "BoundsTree.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <random>

using namespace Radion;

namespace
{
int gFailures = 0;

void check(bool condition, const char* expression, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "BoundsTreeTests:%d: failed: %s\n", line, expression);
        ++gFailures;
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

AABB boxAt(const glm::vec3& center, const glm::vec3& half)
{
    AABB box;
    box.min = center - half;
    box.max = center + half;
    return box;
}

// The answer the tree has to agree with, every time. A hierarchy that is
// merely fast is worth nothing; the only property that matters is that it
// returns exactly what a full scan would.
std::vector<u32> bruteForce(const std::vector<AABB>& items, const AABB& query)
{
    std::vector<u32> out;
    for (u32 i = 0; i < items.size(); ++i)
        if (items[i].intersects(query))
            out.push_back(i);
    return out;
}

std::vector<u32> bruteForce(const std::vector<AABB>& items, const Frustum& frustum)
{
    std::vector<u32> out;
    for (u32 i = 0; i < items.size(); ++i)
        if (frustum.intersects(items[i]))
            out.push_back(i);
    return out;
}

bool sameSet(std::vector<u32> a, std::vector<u32> b)
{
    std::sort(a.begin(), a.end());
    std::sort(b.begin(), b.end());
    return a == b;
}

// Everything in `candidates` really is a candidate and nothing was lost: the
// tree's contract is a superset, so the property to check is containment, not
// equality.
bool containsAll(std::vector<u32> candidates, std::vector<u32> answer)
{
    std::sort(candidates.begin(), candidates.end());
    std::sort(answer.begin(), answer.end());
    return std::includes(candidates.begin(), candidates.end(), answer.begin(), answer.end());
}

// The exact test the tree deliberately leaves to the caller. Doing it here is
// what turns candidates into the answer, and comparing THAT against brute
// force is what proves the tree drops nothing.
template <typename Shape>
std::vector<u32> filtered(const BoundsTree& tree, const std::vector<u32>& candidates,
                          const Shape& shape)
{
    std::vector<u32> out;
    for (u32 item : candidates)
        if (shape.intersects(tree.itemBounds(item)))
            out.push_back(item);
    return out;
}

// Spread out, all in one place, and a mix - the three shapes a real scene
// takes, and the ones that tell the structures apart. Clustered is the case a
// sweep-and-prune collapses on and the one a tower of boxes actually is.
enum class Layout
{
    Spread,
    Clustered,
    Mixed
};

std::vector<AABB> makeItems(Layout layout, u32 count, u32 seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<f32> unit(-1.0f, 1.0f);
    std::vector<AABB> items;
    items.reserve(count);
    for (u32 i = 0; i < count; ++i)
    {
        glm::vec3 center;
        f32 size = 1.0f;
        switch (layout)
        {
        case Layout::Spread:
            center = glm::vec3(unit(rng), unit(rng) * 0.2f, unit(rng)) * 200.0f;
            size = 1.0f + std::abs(unit(rng));
            break;
        case Layout::Clustered:
            center = glm::vec3(unit(rng), unit(rng), unit(rng)) * 4.0f;
            size = 0.5f;
            break;
        case Layout::Mixed:
            if ((i % 4) == 0)
            {
                center = glm::vec3(unit(rng), unit(rng) * 0.2f, unit(rng)) * 200.0f;
                size = 2.0f + std::abs(unit(rng)) * 8.0f;
            }
            else
            {
                center = glm::vec3(unit(rng), unit(rng), unit(rng)) * 6.0f;
                size = 0.5f;
            }
            break;
        }
        items.push_back(boxAt(center, glm::vec3(size)));
    }
    return items;
}

f64 milliseconds(std::chrono::steady_clock::time_point start)
{
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<f64, std::milli>(end - start).count();
}

// ----------------------------------------------------------- correctness

void testEmptyAndSingle()
{
    BoundsTree tree;
    std::vector<u32> out;

    // Nothing in it must answer nothing, not crash and not return garbage.
    CHECK(!tree.valid());
    tree.queryCandidates(boxAt(glm::vec3(0.0f), glm::vec3(1.0f)), out);
    CHECK(out.empty());
    CHECK(!tree.refit(nullptr, 0));

    const AABB one = boxAt(glm::vec3(5.0f, 0.0f, 0.0f), glm::vec3(1.0f));
    tree.build(&one, 1);
    CHECK(tree.valid());
    CHECK(tree.itemCount() == 1);
    tree.queryCandidates(boxAt(glm::vec3(5.0f, 0.0f, 0.0f), glm::vec3(0.5f)), out);
    CHECK(out.size() == 1 && out[0] == 0);
    tree.queryCandidates(boxAt(glm::vec3(50.0f, 0.0f, 0.0f), glm::vec3(0.5f)), out);
    CHECK(out.empty());
}

void testMatchesBruteForce()
{
    for (u32 layoutIndex = 0; layoutIndex < 3; ++layoutIndex)
    {
        const Layout layout = static_cast<Layout>(layoutIndex);
        const std::vector<AABB> items = makeItems(layout, 2000, 1234 + layoutIndex);

        BoundsTree tree;
        tree.build(items.data(), static_cast<u32>(items.size()));
        CHECK(tree.valid());
        CHECK(tree.itemCount() == items.size());

        std::mt19937 rng(99);
        std::uniform_real_distribution<f32> unit(-1.0f, 1.0f);
        std::vector<u32> out;
        for (u32 trial = 0; trial < 200; ++trial)
        {
            const glm::vec3 center(unit(rng) * 200.0f, unit(rng) * 20.0f, unit(rng) * 200.0f);
            const AABB query = boxAt(center, glm::vec3(1.0f + std::abs(unit(rng)) * 20.0f));
            tree.queryCandidates(query, out);
            const std::vector<u32> answer = bruteForce(items, query);
            // Nothing real may be missing from the candidates, and filtering
            // them has to land exactly on the answer.
            if (!containsAll(out, answer) || !sameSet(filtered(tree, out, query), answer))
            {
                std::fprintf(stderr, "  layout %u trial %u: tree and brute force disagree\n",
                             layoutIndex, trial);
                ++gFailures;
                break;
            }
        }

        // A query covering everything has to return everything - the simplest
        // way to catch a leaf that was built but never linked in.
        AABB everything;
        for (const AABB& item : items)
            everything.merge(item);
        tree.queryCandidates(everything, out);
        CHECK(out.size() == items.size());
    }
}

void testFrustumMatchesBruteForce()
{
    const std::vector<AABB> items = makeItems(Layout::Mixed, 3000, 7);
    BoundsTree tree;
    tree.build(items.data(), static_cast<u32>(items.size()));

    std::vector<u32> out;
    std::mt19937 rng(4242);
    std::uniform_real_distribution<f32> unit(-1.0f, 1.0f);
    for (u32 trial = 0; trial < 40; ++trial)
    {
        const glm::vec3 eye(unit(rng) * 250.0f, 20.0f + std::abs(unit(rng)) * 40.0f,
                            unit(rng) * 250.0f);
        const glm::mat4 view = glm::lookAt(eye, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        const glm::mat4 projection = glm::perspective(glm::radians(60.0f), 16.0f / 9.0f, 0.5f,
                                                      600.0f);
        Frustum frustum;
        frustum.update(projection * view);

        tree.queryCandidates(frustum, out);
        const std::vector<u32> answer = bruteForce(items, frustum);
        // The fully-inside shortcut must not drop anything a plane test would
        // have accepted.
        if (!containsAll(out, answer) || !sameSet(filtered(tree, out, frustum), answer))
        {
            std::fprintf(stderr, "  frustum trial %u: tree and brute force disagree (%zu vs %zu)\n",
                         trial, out.size(), answer.size());
            ++gFailures;
            break;
        }
    }
}

void testRefitKeepsAnswersRight()
{
    std::vector<AABB> items = makeItems(Layout::Mixed, 1500, 21);
    BoundsTree tree;
    tree.build(items.data(), static_cast<u32>(items.size()));
    const u32 nodesBefore = tree.nodeCount();

    // Move everything, then refit. The answers still have to be exact - a
    // refit that leaves one node's box stale drops items silently.
    std::mt19937 rng(5);
    std::uniform_real_distribution<f32> unit(-1.0f, 1.0f);
    for (AABB& item : items)
    {
        const glm::vec3 shift(unit(rng) * 3.0f, unit(rng) * 3.0f, unit(rng) * 3.0f);
        item.min += shift;
        item.max += shift;
    }
    CHECK(tree.refit(items.data(), static_cast<u32>(items.size())));
    // Refit must not change the shape - that is the whole point of it.
    CHECK(tree.nodeCount() == nodesBefore);

    std::vector<u32> out;
    for (u32 trial = 0; trial < 100; ++trial)
    {
        const AABB query = boxAt(glm::vec3(unit(rng) * 200.0f, unit(rng) * 20.0f,
                                           unit(rng) * 200.0f),
                                 glm::vec3(5.0f));
        tree.queryCandidates(query, out);
        const std::vector<u32> answer = bruteForce(items, query);
        if (!containsAll(out, answer) || !sameSet(filtered(tree, out, query), answer))
        {
            std::fprintf(stderr, "  refit trial %u: tree and brute force disagree\n", trial);
            ++gFailures;
            break;
        }
    }

    // A different number of items cannot be refitted onto this shape, and
    // saying so beats answering from a tree that no longer describes them.
    CHECK(!tree.refit(items.data(), static_cast<u32>(items.size()) - 1));
}

void testDegenerateInputs()
{
    // Every box in the same place: the centres never separate, so a split
    // that only trusts the midpoint would recurse forever on the same set.
    std::vector<AABB> stacked(64, boxAt(glm::vec3(1.0f), glm::vec3(0.5f)));
    BoundsTree tree;
    tree.build(stacked.data(), static_cast<u32>(stacked.size()));
    CHECK(tree.valid());
    CHECK(tree.itemCount() == stacked.size());
    CHECK(tree.depth() < 48);

    std::vector<u32> out;
    tree.queryCandidates(boxAt(glm::vec3(1.0f), glm::vec3(0.1f)), out);
    CHECK(out.size() == stacked.size());

    // Boxes of zero size are points, and still have to be found.
    std::vector<AABB> points;
    for (u32 i = 0; i < 100; ++i)
    {
        const glm::vec3 at(static_cast<f32>(i), 0.0f, 0.0f);
        points.push_back(boxAt(at, glm::vec3(0.0f)));
    }
    BoundsTree pointTree;
    pointTree.build(points.data(), static_cast<u32>(points.size()));
    pointTree.queryCandidates(boxAt(glm::vec3(50.0f, 0.0f, 0.0f), glm::vec3(0.01f)), out);
    CHECK(out.size() == 1 && out[0] == 50);
}

void testQualityRisesAsThingsMove()
{
    // The measure that says when a refit has stopped being good enough.
    // Scattering the items without rebuilding has to make it worse, and a
    // rebuild has to bring it back - otherwise there is no signal to act on.
    std::vector<AABB> items = makeItems(Layout::Spread, 4000, 3);
    BoundsTree tree;
    tree.build(items.data(), static_cast<u32>(items.size()));
    const f32 fresh = tree.quality();
    CHECK(fresh > 0.0f);

    std::mt19937 rng(11);
    std::uniform_real_distribution<f32> unit(-1.0f, 1.0f);
    for (u32 pass = 0; pass < 40; ++pass)
    {
        for (AABB& item : items)
        {
            const glm::vec3 shift(unit(rng) * 8.0f, unit(rng) * 8.0f, unit(rng) * 8.0f);
            item.min += shift;
            item.max += shift;
        }
        tree.refit(items.data(), static_cast<u32>(items.size()));
    }
    const f32 decayed = tree.quality();

    BoundsTree rebuilt;
    rebuilt.build(items.data(), static_cast<u32>(items.size()));
    const f32 after = rebuilt.quality();

    std::fprintf(stderr, "  quality: fresh %.1f, after 40 refits %.1f, rebuilt %.1f\n",
                 static_cast<f64>(fresh), static_cast<f64>(decayed), static_cast<f64>(after));
    CHECK(decayed > fresh);
    CHECK(after < decayed);
}

// ------------------------------------------------------------- measurement

void benchmark()
{
    std::fprintf(stderr, "\n  --- BoundsTree, 20000 items ---\n");
    std::fprintf(stderr, "  %-10s %8s %8s %10s %10s %8s\n", "layout", "build", "refit", "query",
                 "brute", "quality");

    static const char* const names[] = {"spread", "clustered", "mixed"};
    for (u32 layoutIndex = 0; layoutIndex < 3; ++layoutIndex)
    {
        const std::vector<AABB> items =
            makeItems(static_cast<Layout>(layoutIndex), 20000, 900 + layoutIndex);
        const u32 count = static_cast<u32>(items.size());

        BoundsTree tree;
        auto start = std::chrono::steady_clock::now();
        tree.build(items.data(), count);
        const f64 buildMs = milliseconds(start);

        start = std::chrono::steady_clock::now();
        for (u32 i = 0; i < 100; ++i)
            tree.refit(items.data(), count);
        const f64 refitMs = milliseconds(start) / 100.0;

        std::mt19937 rng(5);
        std::uniform_real_distribution<f32> unit(-1.0f, 1.0f);
        std::vector<AABB> queries;
        for (u32 i = 0; i < 1000; ++i)
            queries.push_back(boxAt(glm::vec3(unit(rng) * 200.0f, unit(rng) * 20.0f,
                                              unit(rng) * 200.0f),
                                    glm::vec3(10.0f)));

        // Timed with the caller's own filtering included, because that is
        // what a real query costs - measuring only the traversal would flatter
        // the tree by leaving out work it deliberately hands back.
        std::vector<u32> out;
        start = std::chrono::steady_clock::now();
        usize found = 0;
        for (const AABB& query : queries)
        {
            tree.queryCandidates(query, out);
            for (u32 item : out)
                if (tree.itemBounds(item).intersects(query))
                    ++found;
        }
        const f64 queryMs = milliseconds(start) / static_cast<f64>(queries.size());

        start = std::chrono::steady_clock::now();
        usize bruteFound = 0;
        for (const AABB& query : queries)
            for (const AABB& item : items)
                if (item.intersects(query))
                    ++bruteFound;
        const f64 bruteMs = milliseconds(start) / static_cast<f64>(queries.size());

        // The measurement is worthless if the two disagree, so it is checked
        // rather than assumed.
        CHECK(found == bruteFound);

        std::fprintf(stderr, "  %-10s %7.2fms %7.3fms %8.4fms %8.4fms %8.1f\n",
                     names[layoutIndex], buildMs, refitMs, queryMs, bruteMs, tree.quality());
    }
    std::fprintf(stderr, "\n");
}

} // namespace

int main()
{
    testEmptyAndSingle();
    testMatchesBruteForce();
    testFrustumMatchesBruteForce();
    testRefitKeepsAnswersRight();
    testDegenerateInputs();
    testQualityRisesAsThingsMove();
    benchmark();
    if (gFailures)
        std::fprintf(stderr, "%d bounds tree test(s) failed\n", gFailures);
    return gFailures == 0 ? 0 : 1;
}
