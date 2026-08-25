#ifndef RADION_BOUNDS_TREE_H
#define RADION_BOUNDS_TREE_H

#include "Math.h"
#include "Types.h"

#include <vector>

namespace Radion
{

 
class BoundsTree
{
public:
    struct Node
    {
        AABB bounds;
        // Index of the first child; the second is always left + 1. Only
        // meaningful when count == 0.
        u32 left = 0;
        // Range into the item order for a leaf.
        u32 offset = 0;
        u32 count = 0;

        bool isLeaf() const
        {
            return count > 0;
        }
    };

    // Rebuilds from scratch. `bounds` is read but not kept.
    void build(const AABB* bounds, u32 count);

 
    bool refit(const AABB* bounds, u32 count);

    void clear();

    bool valid() const
    {
        return mNodeCount > 0;
    }
    u32 nodeCount() const
    {
        return mNodeCount;
    }
    u32 itemCount() const
    {
        return static_cast<u32>(mOrder.size());
    }
    u32 depth() const
    {
        return mDepth;
    }
    // For drawing the tree, and for nothing else - a caller that navigates by
    // node index is doing the traversal's job by hand.
    const Node& node(u32 index) const
    {
        return mNodes[index];
    }

    // Total surface area of every node divided by the root's. The standard
    // measure of how much a hierarchy costs to traverse: 1 would be a single
    // node, and it grows as nodes overlap. Compare it before and after a run
    // of refits to see when a rebuild has become worth it.
    f32 quality() const;

    // The box this item was last built or refitted with - so a caller can do
    // the exact test the queries below leave to it without keeping its own
    // copy in step.
    const AABB& itemBounds(u32 item) const
    {
        return mBounds[item];
    }

 
    void queryCandidates(const AABB& box, std::vector<u32>& out) const;
    void queryCandidates(const Sphere& sphere, std::vector<u32>& out) const;
    void queryCandidates(const Frustum& frustum, std::vector<u32>& out) const;
    void queryCandidates(const Ray& ray, f32 maxDistance, std::vector<u32>& out) const;

 
    template <typename NodeTest>
    void queryCandidatesIf(const NodeTest& test, std::vector<u32>& out) const
    {
        out.clear();
        mStats = Stats();
        if (mNodeCount == 0)
            return;

        mStack.clear();
        mStack.push_back(0);
        mStack.push_back(0); // 0 = still to test, 1 = wholly inside
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
                const Containment result = test(node.bounds);
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

    // Traversal counters from the last query, for measuring rather than
    // guessing which structure is cheaper.
    struct Stats
    {
        u32 nodesVisited = 0;
        u32 leavesVisited = 0;
        u32 itemsReturned = 0;
    };
    const Stats& lastQueryStats() const
    {
        return mStats;
    }

    // Leaves hold at most this many items; a node with fewer never splits.
    void setLeafCapacity(u32 capacity);
    u32 leafCapacity() const
    {
        return mLeafCapacity;
    }

private:
    void subdivide(u32 nodeIndex, const AABB* bounds, u32 depth);
    void updateNodeBounds(u32 nodeIndex, const AABB* bounds);

    std::vector<Node> mNodes;
    // Item boxes, indexed by the caller's own item index.
    std::vector<AABB> mBounds;
    // Item indices, permuted by build() so each leaf owns a contiguous run.
    std::vector<u32> mOrder;
    u32 mNodeCount = 0;
    u32 mDepth = 0;
    u32 mLeafCapacity = 2;
    mutable Stats mStats;
    // Traversal scratch, kept so a query allocates nothing.
    mutable std::vector<u32> mStack;
};

} // namespace Radion

#endif // RADION_BOUNDS_TREE_H
