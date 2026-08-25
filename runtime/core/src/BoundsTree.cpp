#include "PCH.h"

#include "BoundsTree.h"

#include <algorithm>

namespace Radion
{

namespace
{
// Deep enough for any tree that splits a real scene, and the size the
// traversal stack is built for. A build that would go deeper stops splitting
// instead, which costs a slightly worse tree - never a wrong answer, which is
// what silently running out of stack during a query would give.
constexpr u32 kMaxDepth = 48;

f32 surfaceArea(const AABB& box)
{
    if (box.empty())
        return 0.0f;
    const glm::vec3 size = glm::max(box.max - box.min, glm::vec3(0.0f));
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
    mLeafCapacity = glm::max(capacity, 1u);
}

void BoundsTree::build(const AABB* bounds, u32 count)
{
    clear();
    if (!bounds || count == 0)
        return;

    // A binary tree whose leaves hold at least one item each needs at most
    // 2n-1 nodes. Leaves here hold up to mLeafCapacity, so this is an upper
    // bound with room to spare and the vector never reallocates mid-build -
    // which matters, because subdivide() holds a reference into it.
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
    mDepth = glm::max(mDepth, depth);
    if (depth >= kMaxDepth)
        return;

    // Read out what is needed before recursing: the recursive calls take
    // their own references into mNodes, and holding one across them is how a
    // subtle aliasing bug gets in even when the vector cannot reallocate.
    const u32 count = mNodes[nodeIndex].count;
    const u32 offset = mNodes[nodeIndex].offset;
    if (count <= mLeafCapacity)
        return;

    // Split down the middle of the longest axis. Not a surface-area
    // heuristic: this is rebuilt often and refitted constantly, and the time
    // an SAH build costs is worth more than the traversal it saves here.
    const glm::vec3 extent = mNodes[nodeIndex].bounds.max - mNodes[nodeIndex].bounds.min;
    u32 axis = 0;
    if (extent.y > extent.x)
        axis = 1;
    if (extent.z > extent[axis])
        axis = 2;
    const f32 split = mNodes[nodeIndex].bounds.min[axis] + extent[axis] * 0.5f;

    // In-place partition of this node's own run of the order array.
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
    // Everything landed on one side - which happens whenever the centres
    // coincide, and would otherwise recurse forever on the same set. Split
    // the run down the middle instead so the tree still gets built.
    if (leftCount == 0 || leftCount == count)
        leftCount = count / 2;

    const u32 leftChild = mNodeCount++;
    const u32 rightChild = mNodeCount++;
    mNodes[nodeIndex].left = leftChild;
    mNodes[nodeIndex].count = 0; // no longer a leaf

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
    // The tree's shape encodes which item is in which leaf, so it is only
    // valid for the exact set it was built from.
    if (count != static_cast<u32>(mOrder.size()))
        return false;
    std::copy(bounds, bounds + count, mBounds.begin());

    // Backwards over the nodes: build() only ever gives a child a higher
    // index than its parent, so one linear pass in reverse always meets both
    // children before the parent that merges them. No recursion, no stack.
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

    // Nodes fully inside are marked, and everything under them is taken
    // without another plane test - which is most of a large tree once the
    // camera is anywhere but on top of it.
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
        // Ray::intersects() reports the EXIT distance when the origin is
        // inside the box - the entry distance a range prune wants is 0 there.
        // Without this, a short ray cast from inside the tree's own bounds (a
        // wheel's suspension probe standing on a large mesh, e.g.) culled the
        // root and returned nothing.
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
