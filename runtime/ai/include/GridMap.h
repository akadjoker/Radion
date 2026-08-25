#ifndef RADION_AI_GRIDMAP_H
#define RADION_AI_GRIDMAP_H

// GridMap.h - a square cost grid for grid-based pathfinding.
//
// Cells hold an integer traversal cost (1 by default); any cost >= Blocked
// (3000) is impassable. Costs are stored in one flat row-major vector for
// cache friendliness.

#include <vector>

namespace Radion::AI
{

class GridMap
{
public:
    explicit GridMap(int size);

    int size() const
    {
        return mSize;
    }

    int cost(int x, int y) const;
    void setCost(int x, int y, int cost);

    bool inBounds(int x, int y) const
    {
        return x >= 0 && y >= 0 && x < mSize && y < mSize;
    }
    bool isBlocked(int x, int y) const
    {
        return cost(x, y) >= Blocked;
    }
    void setBlocked(int x, int y)
    {
        setCost(x, y, Blocked);
    }

    static constexpr int DefaultCost = 1;
    static constexpr int Blocked = 3000;

private:
    int mSize;
    std::vector<int> mCosts; // row-major, index = y * mSize + x
};

} // namespace Radion::AI

#endif // RADION_AI_GRIDMAP_H
