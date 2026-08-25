#ifndef RADION_AI_GRIDPATHFINDER_H
#define RADION_AI_GRIDPATHFINDER_H

// GridPathfinder.h - search over a GridMap.
//
// Single findPath() call over the grid. The search algorithm is selectable:
// A*, Dijkstra, greedy Best-First or Breadth-First, all using 8-directional
// neighbours (diagonals cost the same as orthogonal moves). The informed
// searches use g = parent.g + cell cost and h = the chosen heuristic scaled
// by the heuristic weight; the open list pops the lowest f.

#include "GridMap.h"

#include <vector>

namespace Radion::AI
{

// With diagonals costing the same as orthogonal moves, the true remaining
// cost between two open cells is max(|dx|, |dy|) - so MaxDxDy (the default)
// is the only heuristic here that never overestimates, and the only one A*
// is guaranteed optimal with. Manhattan and Euclidean overestimate whenever
// a diagonal shortcut exists; picking them for A* trades path optimality for
// fewer expansions, the same bounded-suboptimal deal as heuristicWeight > 1.
// Best-First is greedy either way, so there they are purely a flavor choice.
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

    // Runs the configured search from (startX, startY) to (endX, endY). On
    // success fills outPath with the full cell list from start to end
    // inclusive (start first) and returns true; returns false if the goal is
    // unreachable.
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

    // Shared weighted search for A* (useHeuristic) and Dijkstra (no heuristic).
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
