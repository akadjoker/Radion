// GridPathfinder.cpp - grid searches (A*, Dijkstra, Best-First, BFS).
//
// The node grid is a flat vector; each expansion is an O(1) contains via
// inOpen/inClosed flags, with the priority lists kept sorted by f.

#include "PCH.h"

#include "GridPathfinder.h"

namespace Radion::AI
{

namespace
{
struct Node
{
    int g = 0;
    int f = 0;
    int h = 0;
    Node* parent = nullptr;
    bool inOpen = false;
    bool inClosed = false;
};

// 8-directional neighbours.
const int kDx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
const int kDy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

// Walk the parent chain from the goal back to the start, then flip to
// produce the path start..end.
bool reconstructPath(const Node* end, const std::vector<Node>& nodes, int size,
                     std::vector<GridCellCoord>& outPath)
{
    std::vector<GridCellCoord> reversed;
    for (const Node* cur = end; cur != nullptr; cur = cur->parent)
    {
        std::size_t idx = static_cast<std::size_t>(cur - nodes.data());
        reversed.push_back(
            GridCellCoord{static_cast<int>(idx % size), static_cast<int>(idx / size)});
    }
    outPath.assign(reversed.rbegin(), reversed.rend());
    return true;
}
} // namespace

int GridPathfinder::goalEstimate(int x, int y, int endX, int endY) const
{
    int dx = endX - x;
    int dy = endY - y;
    if (dx < 0)
        dx = -dx;
    if (dy < 0)
        dy = -dy;

    int est = 0;
    switch (mSettings.heuristic)
    {
    case GridHeuristic::MaxDxDy:
        est = dx > dy ? dx : dy;
        break;
    case GridHeuristic::Euclidean:
        est = static_cast<int>(std::floor(std::sqrt(static_cast<float>(dx * dx + dy * dy))));
        break;
    case GridHeuristic::Manhattan:
    default:
        est = dx + dy;
        break;
    }
    return static_cast<int>(est * mSettings.heuristicWeight);
}

bool GridPathfinder::findPath(int startX, int startY, int endX, int endY,
                              std::vector<GridCellCoord>& outPath) const
{
    outPath.clear();

    if (!mGrid->inBounds(startX, startY) || !mGrid->inBounds(endX, endY))
        return false;
    if (mGrid->isBlocked(endX, endY))
        return false;
    if (startX == endX && startY == endY)
    {
        outPath.push_back(GridCellCoord{startX, startY});
        return true;
    }

    switch (mSettings.algorithm)
    {
    case GridSearchAlgorithm::AStar:
        return searchInformed(startX, startY, endX, endY, true, outPath);
    case GridSearchAlgorithm::Dijkstra:
        return searchInformed(startX, startY, endX, endY, false, outPath);
    case GridSearchAlgorithm::BestFirst:
        return searchBestFirst(startX, startY, endX, endY, outPath);
    case GridSearchAlgorithm::BreadthFirst:
    default:
        return searchBreadthFirst(startX, startY, endX, endY, outPath);
    }
}

// Shared weighted search: A* (useHeuristic) and Dijkstra (no heuristic).
bool GridPathfinder::searchInformed(int startX, int startY, int endX, int endY, bool useHeuristic,
                                    std::vector<GridCellCoord>& outPath) const
{
    const int size = mGrid->size();
    std::vector<Node> nodes(static_cast<std::size_t>(size) * size);
    const auto nodeAt = [&](int x, int y) -> Node&
    {
        return nodes[static_cast<std::size_t>(y) * size + x];
    };

    Node* start = &nodeAt(startX, startY);
    start->g = 0;
    start->h = useHeuristic ? goalEstimate(startX, startY, endX, endY) : 0;
    start->f = start->g + start->h;
    start->inOpen = true;

    // Open list kept sorted descending by f so pop_back() yields the lowest f.
    auto insertSorted = [](std::vector<Node*>& list, Node* node)
    {
        auto pos = std::lower_bound(list.begin(), list.end(), node,
                                    [](const Node* a, const Node* b)
                                    {
                                        return a->f > b->f;
                                    });
        list.insert(pos, node);
    };

    std::vector<Node*> open;
    open.reserve(static_cast<std::size_t>(size) * size);
    insertSorted(open, start);

    while (!open.empty())
    {
        Node* n = open.back();
        open.pop_back();
        n->inOpen = false;

        if (n == &nodeAt(endX, endY))
            return reconstructPath(n, nodes, size, outPath);

        std::size_t nIdx = static_cast<std::size_t>(n - nodes.data());
        int nx = static_cast<int>(nIdx % size);
        int ny = static_cast<int>(nIdx / size);

        for (int i = 0; i < 8; ++i)
        {
            int x = nx + kDx[i];
            int y = ny + kDy[i];
            if (!mGrid->inBounds(x, y) || mGrid->isBlocked(x, y))
                continue;

            Node& node = nodeAt(x, y);
            int newg = n->g + mGrid->cost(x, y);

            if ((node.inOpen || node.inClosed) && node.g <= newg)
                continue; // already have a cheaper or equal route to it

            node.parent = n;
            node.g = newg;
            node.h = useHeuristic ? goalEstimate(x, y, endX, endY) : 0;
            node.f = node.g + node.h;
            node.inClosed = false;

            if (!node.inOpen)
            {
                node.inOpen = true;
                insertSorted(open, &node);
            }
            else
            {
                // Cost changed - reposition within the sorted open list.
                auto pos = std::find(open.begin(), open.end(), &node);
                if (pos != open.end())
                    open.erase(pos);
                insertSorted(open, &node);
            }
        }

        n->inClosed = true;
    }

    return false; // goal unreachable
}

// Greedy best-first search: expand the node with the lowest heuristic.
bool GridPathfinder::searchBestFirst(int startX, int startY, int endX, int endY,
                                     std::vector<GridCellCoord>& outPath) const
{
    const int size = mGrid->size();
    std::vector<Node> nodes(static_cast<std::size_t>(size) * size);
    const auto nodeAt = [&](int x, int y) -> Node&
    {
        return nodes[static_cast<std::size_t>(y) * size + x];
    };

    Node* start = &nodeAt(startX, startY);
    start->h = goalEstimate(startX, startY, endX, endY);
    start->f = start->h;
    start->inOpen = true;

    auto insertSorted = [](std::vector<Node*>& list, Node* node)
    {
        auto pos = std::lower_bound(list.begin(), list.end(), node,
                                    [](const Node* a, const Node* b)
                                    {
                                        return a->f > b->f;
                                    });
        list.insert(pos, node);
    };

    std::vector<Node*> open;
    open.reserve(static_cast<std::size_t>(size) * size);
    insertSorted(open, start);

    while (!open.empty())
    {
        Node* n = open.back();
        open.pop_back();
        n->inOpen = false;

        if (n == &nodeAt(endX, endY))
            return reconstructPath(n, nodes, size, outPath);

        std::size_t nIdx = static_cast<std::size_t>(n - nodes.data());
        int nx = static_cast<int>(nIdx % size);
        int ny = static_cast<int>(nIdx / size);

        for (int i = 0; i < 8; ++i)
        {
            int x = nx + kDx[i];
            int y = ny + kDy[i];
            if (!mGrid->inBounds(x, y) || mGrid->isBlocked(x, y))
                continue;

            Node& node = nodeAt(x, y);
            if (node.inOpen || node.inClosed)
                continue; // greedy: never revisit a discovered node

            node.parent = n;
            node.h = goalEstimate(x, y, endX, endY);
            node.f = node.h;
            node.inOpen = true;
            insertSorted(open, &node);
        }

        n->inClosed = true;
    }

    return false; // goal unreachable
}

// Breadth-first search: unweighted FIFO expansion.
bool GridPathfinder::searchBreadthFirst(int startX, int startY, int endX, int endY,
                                        std::vector<GridCellCoord>& outPath) const
{
    const int size = mGrid->size();
    std::vector<Node> nodes(static_cast<std::size_t>(size) * size);
    const auto nodeAt = [&](int x, int y) -> Node&
    {
        return nodes[static_cast<std::size_t>(y) * size + x];
    };

    Node* start = &nodeAt(startX, startY);
    start->inOpen = true;

    std::deque<Node*> queue;
    queue.push_back(start);

    while (!queue.empty())
    {
        Node* n = queue.front();
        queue.pop_front();
        n->inOpen = false;

        if (n == &nodeAt(endX, endY))
            return reconstructPath(n, nodes, size, outPath);

        std::size_t nIdx = static_cast<std::size_t>(n - nodes.data());
        int nx = static_cast<int>(nIdx % size);
        int ny = static_cast<int>(nIdx / size);

        for (int i = 0; i < 8; ++i)
        {
            int x = nx + kDx[i];
            int y = ny + kDy[i];
            if (!mGrid->inBounds(x, y) || mGrid->isBlocked(x, y))
                continue;

            Node& node = nodeAt(x, y);
            if (node.inOpen || node.inClosed)
                continue; // already queued or expanded

            node.parent = n;
            node.inOpen = true;
            queue.push_back(&node);
        }

        n->inClosed = true;
    }

    return false; // goal unreachable
}

} // namespace Radion::AI
