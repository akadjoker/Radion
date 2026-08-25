// GridMap.cpp - implementation of the cost grid.

#include "PCH.h"

#include "GridMap.h"

namespace Radion::AI
{

GridMap::GridMap(int size) : mSize(size), mCosts(static_cast<std::size_t>(size) * size, DefaultCost)
{
}

int GridMap::cost(int x, int y) const
{
    if (!inBounds(x, y))
        return Blocked;
    return mCosts[static_cast<std::size_t>(y) * mSize + x];
}

void GridMap::setCost(int x, int y, int cost)
{
    if (!inBounds(x, y))
        return;
    mCosts[static_cast<std::size_t>(y) * mSize + x] = cost;
}

} // namespace Radion::AI
