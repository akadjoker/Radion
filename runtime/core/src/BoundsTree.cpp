#include "PCH.h"

#include "BoundsTree.h"

#include <algorithm>

namespace Radion
{

namespace
{
// Deep enough for real scenes; also the traversal stack size. Past it splitting stops (worse tree, never a wrong answer).
constexpr u32 kMaxDepth = 48;

f32 surfaceArea(const AABB& box)
{
    if (box.empty())
        return 0.0f;
    const Math::vec3 size = Math::max(box.max - box.min, Math::vec3(0.0f));
    return 2.0f * (size.x * size.y + size.y * size.z + size.z * size.x);
}
} // namespace

void BoundsTree::clear()
{
    mNodes.clear();
    mBounds.clear();
    mOrder.clear();
    mNodeCount = 0;
    mDepth = 0;
    mStats = Stats();
}

void BoundsTree::setLeafCapacity(u32 capacity)
{
    mLeafCapacity = Math::max(capacity, 1u);
}

void BoundsTree::build(const AABB* bounds, u32 count)
{
    clear();
    if (!bounds || count == 0)
        return;

    // 2n-1 nodes bounds a binary tree; reserved up front because subdivide() holds a reference into mNodes.
    mNodes.resize(static_cast<usize>(count) * 2u);
    mBounds.assign(bounds, bounds + count);
    mOrder.resize(count);
    for (u32 i = 0; i < count; ++i)
        mOrder[i] = i;

    mNodeCount = 1;
    Node& root = mNodes[0];
    root = Node();
    root.offset = 0;
    root.count = count;
    updateNodeBounds(0, bounds);

    mDepth = 1;
    subdivide(0, bounds, 1);
}

void BoundsTree::updateNodeBounds(u32 nodeIndex, const AABB* bounds)
{
    Node& node = mNodes[nodeIndex];
    node.bounds = AABB();
    for (u32 i = 0; i < node.count; ++i)
        node.bounds.merge(bounds[mOrder[node.offset + i]]);
}

void BoundsTree::subdivide(u32 nodeIndex, const AABB* bounds, u32 depth)
{
    mDepth = Math::max(mDepth, depth);
    if (depth >= kMaxDepth)
        return;

    // Read what is needed before recursing: recursive calls take their own references into mNodes.
    const u32 count = mNodes[nodeIndex].count;
    const u32 offset = mNodes[nodeIndex].offset;
    if (count <= mLeafCapacity)
        return;

    // Middle of the longest axis, not SAH: rebuilt and refitted often, and SAH build time costs more than it saves.
    const Math::vec3 extent = mNodes[nodeIndex].bounds.max - mNodes[nodeIndex].bounds.min;
    u32 axis = 0;
    if (extent.y > extent.x)
        axis = 1;
    if (extent.z > extent[axis])
        axis = 2;
    const f32 split = mNodes[nodeIndex].bounds.min[axis] + extent[axis] * 0.5f;

    u32 left = offset;
    u32 right = offset + count;
    while (left < right)
    {
        const AABB& box = bounds[mOrder[left]];
        const f32 center = (box.min[axis] + box.max[axis]) * 0.5f;
        if (center < split)
            ++left;
        else
            std::swap(mOrder[left], mOrder[--right]);
    }

    u32 leftCount = left - offset;
    // All on one side (e.g. coincident centres) would recurse forever; split the run down the middle.
    if (leftCount == 0 || leftCount == count)
        leftCount = count / 2;

    const u32 leftChild = mNodeCount++;
    const u32 rightChild = mNodeCount++;
    mNodes[nodeIndex].left = leftChild;
    mNodes[nodeIndex].count = 0;

    mNodes[leftChild] = Node();
    mNodes[leftChild].offset = offset;
    mNodes[leftChild].count = leftCount;
    mNodes[rightChild] = Node();
    mNodes[rightChild].offset = offset + leftCount;
    mNodes[rightChild].count = count - leftCount;

    updateNodeBounds(leftChild, bounds);
    updateNodeBounds(rightChild, bounds);
    subdivide(leftChild, bounds, depth + 1);
    subdivide(rightChild, bounds, depth + 1);
}

bool BoundsTree::refit(const AABB* bounds, u32 count)
{
    if (mNodeCount == 0 || !bounds)
        return false;
    // Only valid for the exact set it was built from.
    if (count != static_cast<u32>(mOrder.size()))
        return false;
    std::copy(bounds, bounds + count, mBounds.begin());

    // Reverse pass: children always have higher indices than their parent, so no recursion is needed.
    for (u32 i = mNodeCount; i > 0; --i)
    {
        Node& node = mNodes[i - 1];
        node.bounds = AABB();
        if (node.isLeaf())
        {
            for (u32 j = 0; j < node.count; ++j)
                node.bounds.merge(bounds[mOrder[node.offset + j]]);
        }
        else
        {
            node.bounds.merge(mNodes[node.left].bounds);
            node.bounds.merge(mNodes[node.left + 1].bounds);
        }
    }
    return true;
}

f32 BoundsTree::quality() const
{
    if (mNodeCount == 0)
        return 0.0f;
    const f32 rootArea = surfaceArea(mNodes[0].bounds);
    if (rootArea <= 0.0f)
        return 0.0f;
    f32 total = 0.0f;
    for (u32 i = 0; i < mNodeCount; ++i)
        total += surfaceArea(mNodes[i].bounds);
    return total / rootArea;
}

void BoundsTree::queryCandidates(const AABB& box, std::vector<u32>& out) const
{
    out.clear();
    mStats = Stats();
    if (mNodeCount == 0)
        return;

    mStack.clear();
    mStack.push_back(0);
    while (!mStack.empty())
    {
        const Node& node = mNodes[mStack.back()];
        mStack.pop_back();
        ++mStats.nodesVisited;
        if (!node.bounds.intersects(box))
            continue;
        if (node.isLeaf())
        {
            ++mStats.leavesVisited;
            for (u32 i = 0; i < node.count; ++i)
                out.push_back(mOrder[node.offset + i]);
            continue;
        }
        mStack.push_back(node.left);
        mStack.push_back(node.left + 1);
    }
    mStats.itemsReturned = static_cast<u32>(out.size());
}

void BoundsTree::queryCandidates(const Sphere& sphere, std::vector<u32>& out) const
{
    out.clear();
    mStats = Stats();
    if (mNodeCount == 0)
        return;

    mStack.clear();
    mStack.push_back(0);
    while (!mStack.empty())
    {
        const Node& node = mNodes[mStack.back()];
        mStack.pop_back();
        ++mStats.nodesVisited;
        if (!sphere.intersects(node.bounds))
            continue;
        if (node.isLeaf())
        {
            ++mStats.leavesVisited;
            for (u32 i = 0; i < node.count; ++i)
                out.push_back(mOrder[node.offset + i]);
            continue;
        }
        mStack.push_back(node.left);
        mStack.push_back(node.left + 1);
    }
    mStats.itemsReturned = static_cast<u32>(out.size());
}

void BoundsTree::queryCandidates(const Frustum& frustum, std::vector<u32>& out) const
{
    out.clear();
    mStats = Stats();
    if (mNodeCount == 0)
        return;

    // Fully-inside nodes are marked; everything under them is taken without further plane tests.
    mStack.clear();
    mStack.push_back(0);
    mStack.push_back(0); // 0 = must test, 1 = wholly inside
    while (!mStack.empty())
    {
        const u32 inside = mStack.back();
        mStack.pop_back();
        const u32 nodeIndex = mStack.back();
        mStack.pop_back();
        const Node& node = mNodes[nodeIndex];
        ++mStats.nodesVisited;

        u32 childInside = inside;
        if (!inside)
        {
            const Containment result = frustum.classify(node.bounds);
            if (result == Containment::Outside)
                continue;
            childInside = result == Containment::Inside ? 1u : 0u;
        }

        if (node.isLeaf())
        {
            ++mStats.leavesVisited;
            for (u32 i = 0; i < node.count; ++i)
                out.push_back(mOrder[node.offset + i]);
            continue;
        }
        mStack.push_back(node.left);
        mStack.push_back(childInside);
        mStack.push_back(node.left + 1);
        mStack.push_back(childInside);
    }
    mStats.itemsReturned = static_cast<u32>(out.size());
}

void BoundsTree::queryCandidates(const Ray& ray, f32 maxDistance, std::vector<u32>& out) const
{
    out.clear();
    mStats = Stats();
    if (mNodeCount == 0)
        return;

    mStack.clear();
    mStack.push_back(0);
    while (!mStack.empty())
    {
        const Node& node = mNodes[mStack.back()];
        mStack.pop_back();
        ++mStats.nodesVisited;
        f32 distance = 0.0f;
        if (!ray.intersects(node.bounds, distance))
            continue;
        // Ray::intersects() reports the EXIT distance from inside the box; a range prune needs entry distance 0 there.
        if (distance > maxDistance && !node.bounds.contains(ray.origin))
            continue;
        if (node.isLeaf())
        {
            ++mStats.leavesVisited;
            for (u32 i = 0; i < node.count; ++i)
                out.push_back(mOrder[node.offset + i]);
            continue;
        }
        mStack.push_back(node.left);
        mStack.push_back(node.left + 1);
    }
    mStats.itemsReturned = static_cast<u32>(out.size());
}

} // namespace Radion
