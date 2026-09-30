#ifndef RADION_AI_GRIDPATHFINDER_H
#define RADION_AI_GRIDPATHFINDER_H

// g = parent.g + cell cost, h = heuristic * weight; the open list pops the lowest f.

#include "GridMap.h"

#include <vector>

namespace Radion::AI
{

// Diagonals cost the same as orthogonal moves, so only MaxDxDy never overestimates; Manhattan/Euclidean
// trade A* optimality for fewer expansions.
enum class GridHeuristic
{
    Manhattan, // |dx| + |dy|
    Euclidean, // floor(sqrt(dx^2 + dy^2))
    MaxDxDy    // max(|dx|, |dy|)
};

enum class GridSearchAlgorithm
{
    AStar,       // f = g + h (weighted, uses the heuristic)
    Dijkstra,    // f = g (weighted, ignores the heuristic)
    BestFirst,   // f = h (greedy best-first, uses the heuristic)
    BreadthFirst // unweighted FIFO
};

struct GridCellCoord
{
    int x = 0;
    int y = 0;
};

struct GridPathSettings
{
    GridSearchAlgorithm algorithm = GridSearchAlgorithm::AStar;
    GridHeuristic heuristic = GridHeuristic::MaxDxDy;
    float heuristicWeight = 1.0f;
};

class GridPathfinder
{
public:
    explicit GridPathfinder(const GridMap* grid) : mGrid(grid)
    {
    }

    // Fills outPath start..end inclusive; false if the goal is unreachable.
    bool findPath(int startX, int startY, int endX, int endY,
                  std::vector<GridCellCoord>& outPath) const;

    GridPathSettings& settings()
    {
        return mSettings;
    }
    const GridPathSettings& settings() const
    {
        return mSettings;
    }

private:
    int goalEstimate(int x, int y, int endX, int endY) const;

    bool searchInformed(int startX, int startY, int endX, int endY, bool useHeuristic,
                        std::vector<GridCellCoord>& outPath) const;
    bool searchBestFirst(int startX, int startY, int endX, int endY,
                         std::vector<GridCellCoord>& outPath) const;
    bool searchBreadthFirst(int startX, int startY, int endX, int endY,
                            std::vector<GridCellCoord>& outPath) const;

    const GridMap* mGrid; // non-owning
    GridPathSettings mSettings;
};

} // namespace Radion::AI

#endif // RADION_AI_GRIDPATHFINDER_H
