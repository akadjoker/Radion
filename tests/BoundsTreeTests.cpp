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

AABB boxAt(const Math::vec3& center, const Math::vec3& half)
{
    AABB box;
    box.min = center - half;
    box.max = center + half;
    return box;
}

// Brute-force reference the tree must agree with exactly.
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

// The tree's contract is a superset: check containment, not equality.
bool containsAll(std::vector<u32> candidates, std::vector<u32> answer)
{
    std::sort(candidates.begin(), candidates.end());
    std::sort(answer.begin(), answer.end());
    return std::includes(candidates.begin(), candidates.end(), answer.begin(), answer.end());
}

// The exact test left to the caller; comparing it with brute force proves the tree drops nothing.
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

// Clustered is the case sweep-and-prune collapses on.
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
        Math::vec3 center;
        f32 size = 1.0f;
        switch (layout)
        {
        case Layout::Spread:
            center = Math::vec3(unit(rng), unit(rng) * 0.2f, unit(rng)) * 200.0f;
            size = 1.0f + std::abs(unit(rng));
            break;
        case Layout::Clustered:
            center = Math::vec3(unit(rng), unit(rng), unit(rng)) * 4.0f;
            size = 0.5f;
            break;
        case Layout::Mixed:
            if ((i % 4) == 0)
            {
                center = Math::vec3(unit(rng), unit(rng) * 0.2f, unit(rng)) * 200.0f;
                size = 2.0f + std::abs(unit(rng)) * 8.0f;
            }
            else
            {
                center = Math::vec3(unit(rng), unit(rng), unit(rng)) * 6.0f;
                size = 0.5f;
            }
            break;
        }
        items.push_back(boxAt(center, Math::vec3(size)));
    }
    return items;
}

f64 milliseconds(std::chrono::steady_clock::time_point start)
{
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<f64, std::milli>(end - start).count();
}

void testEmptyAndSingle()
{
    BoundsTree tree;
    std::vector<u32> out;

    CHECK(!tree.valid());
    tree.queryCandidates(boxAt(Math::vec3(0.0f), Math::vec3(1.0f)), out);
    CHECK(out.empty());
    CHECK(!tree.refit(nullptr, 0));

    const AABB one = boxAt(Math::vec3(5.0f, 0.0f, 0.0f), Math::vec3(1.0f));
    tree.build(&one, 1);
    CHECK(tree.valid());
    CHECK(tree.itemCount() == 1);
    tree.queryCandidates(boxAt(Math::vec3(5.0f, 0.0f, 0.0f), Math::vec3(0.5f)), out);
    CHECK(out.size() == 1 && out[0] == 0);
    tree.queryCandidates(boxAt(Math::vec3(50.0f, 0.0f, 0.0f), Math::vec3(0.5f)), out);
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
            const Math::vec3 center(unit(rng) * 200.0f, unit(rng) * 20.0f, unit(rng) * 200.0f);
            const AABB query = boxAt(center, Math::vec3(1.0f + std::abs(unit(rng)) * 20.0f));
            tree.queryCandidates(query, out);
            const std::vector<u32> answer = bruteForce(items, query);
            if (!containsAll(out, answer) || !sameSet(filtered(tree, out, query), answer))
            {
                std::fprintf(stderr, "  layout %u trial %u: tree and brute force disagree\n",
                             layoutIndex, trial);
                ++gFailures;
                break;
            }
        }

        // Catches a leaf that was built but never linked in.
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
        const Math::vec3 eye(unit(rng) * 250.0f, 20.0f + std::abs(unit(rng)) * 40.0f,
                            unit(rng) * 250.0f);
        const Math::mat4 view = Math::lookAt(eye, Math::vec3(0.0f), Math::vec3(0.0f, 1.0f, 0.0f));
        const Math::mat4 projection = Math::perspective(Math::radians(60.0f), 16.0f / 9.0f, 0.5f,
                                                      600.0f);
        Frustum frustum;
        frustum.update(projection * view);

        tree.queryCandidates(frustum, out);
        const std::vector<u32> answer = bruteForce(items, frustum);
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

    // A refit that leaves one node's box stale drops items silently.
    std::mt19937 rng(5);
    std::uniform_real_distribution<f32> unit(-1.0f, 1.0f);
    for (AABB& item : items)
    {
        const Math::vec3 shift(unit(rng) * 3.0f, unit(rng) * 3.0f, unit(rng) * 3.0f);
        item.min += shift;
        item.max += shift;
    }
    CHECK(tree.refit(items.data(), static_cast<u32>(items.size())));
    CHECK(tree.nodeCount() == nodesBefore);

    std::vector<u32> out;
    for (u32 trial = 0; trial < 100; ++trial)
    {
        const AABB query = boxAt(Math::vec3(unit(rng) * 200.0f, unit(rng) * 20.0f,
                                           unit(rng) * 200.0f),
                                 Math::vec3(5.0f));
        tree.queryCandidates(query, out);
        const std::vector<u32> answer = bruteForce(items, query);
        if (!containsAll(out, answer) || !sameSet(filtered(tree, out, query), answer))
        {
            std::fprintf(stderr, "  refit trial %u: tree and brute force disagree\n", trial);
            ++gFailures;
            break;
        }
    }

    CHECK(!tree.refit(items.data(), static_cast<u32>(items.size()) - 1));
}

void testDegenerateInputs()
{
    // Identical centres never separate: a midpoint-only split would recurse forever.
    std::vector<AABB> stacked(64, boxAt(Math::vec3(1.0f), Math::vec3(0.5f)));
    BoundsTree tree;
    tree.build(stacked.data(), static_cast<u32>(stacked.size()));
    CHECK(tree.valid());
    CHECK(tree.itemCount() == stacked.size());
    CHECK(tree.depth() < 48);

    std::vector<u32> out;
    tree.queryCandidates(boxAt(Math::vec3(1.0f), Math::vec3(0.1f)), out);
    CHECK(out.size() == stacked.size());

    std::vector<AABB> points;
    for (u32 i = 0; i < 100; ++i)
    {
        const Math::vec3 at(static_cast<f32>(i), 0.0f, 0.0f);
        points.push_back(boxAt(at, Math::vec3(0.0f)));
    }
    BoundsTree pointTree;
    pointTree.build(points.data(), static_cast<u32>(points.size()));
    pointTree.queryCandidates(boxAt(Math::vec3(50.0f, 0.0f, 0.0f), Math::vec3(0.01f)), out);
    CHECK(out.size() == 1 && out[0] == 50);
}

void testQualityRisesAsThingsMove()
{
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
            const Math::vec3 shift(unit(rng) * 8.0f, unit(rng) * 8.0f, unit(rng) * 8.0f);
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
            queries.push_back(boxAt(Math::vec3(unit(rng) * 200.0f, unit(rng) * 20.0f,
                                              unit(rng) * 200.0f),
                                    Math::vec3(10.0f)));

        // Timed with the caller's filtering included; traversal alone would flatter the tree.
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
