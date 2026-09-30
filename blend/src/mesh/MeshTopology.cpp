#include "PCH.h"
#include "mesh/MeshTopology.h"

#include <cmath>

using namespace Radion;

namespace
{
struct CellKey
{
    s64 x = 0;
    s64 y = 0;
    s64 z = 0;

    bool operator==(const CellKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct CellHash
{
    usize operator()(const CellKey& key) const
    {
        // Three large odd multipliers, the usual spatial-hash mix.
        return static_cast<usize>(static_cast<u64>(key.x) * 73856093ull ^
                                  static_cast<u64>(key.y) * 19349663ull ^
                                  static_cast<u64>(key.z) * 83492791ull);
    }
};

CellKey cellOf(const Math::vec3& position, f32 size)
{
    return {static_cast<s64>(std::floor(position.x / size)),
            static_cast<s64>(std::floor(position.y / size)),
            static_cast<s64>(std::floor(position.z / size))};
}
} // namespace

u64 MeshTopology::edgeKey(u32 canonicalA, u32 canonicalB)
{
    if (canonicalA > canonicalB)
        std::swap(canonicalA, canonicalB);
    return (static_cast<u64>(canonicalA) << 32) | canonicalB;
}

s32 MeshTopology::findEdge(u32 canonicalA, u32 canonicalB) const
{
    const auto found = mEdgeIndex.find(edgeKey(canonicalA, canonicalB));
    return found == mEdgeIndex.end() ? -1 : static_cast<s32>(found->second);
}

void MeshTopology::build(const MeshData& mesh, f32 epsilon)
{
    const usize vertexCount = mesh.positions.size();
    mCanonical.assign(vertexCount, 0);
    mEdges.clear();
    mEdgeIndex.clear();

    // Weld by position; a group's representative is its lowest index.
    const f32 cell = epsilon > 0.0f ? epsilon : 1.0e-5f;
    std::unordered_map<CellKey, std::vector<u32>, CellHash> cells;
    cells.reserve(vertexCount);
    for (u32 i = 0; i < vertexCount; ++i)
    {
        const Math::vec3& p = mesh.positions[i];
        const CellKey home = cellOf(p, cell);
        u32 match = i;
        for (s64 dx = -1; dx <= 1 && match == i; ++dx)
        {
            for (s64 dy = -1; dy <= 1 && match == i; ++dy)
            {
                for (s64 dz = -1; dz <= 1 && match == i; ++dz)
                {
                    const auto found = cells.find({home.x + dx, home.y + dy, home.z + dz});
                    if (found == cells.end())
                        continue;
                    for (const u32 candidate : found->second)
                    {
                        if (Math::distance(mesh.positions[candidate], p) <= cell)
                        {
                            match = candidate;
                            break;
                        }
                    }
                }
            }
        }
        mCanonical[i] = match;
        if (match == i)
            cells[home].push_back(i);
    }

    const usize faceCount = mesh.indices.size() / 3;
    mFaceEdges.assign(faceCount, {-1, -1, -1});
    for (u32 face = 0; face < faceCount; ++face)
    {
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 from = mesh.indices[face * 3 + corner];
            const u32 to = mesh.indices[face * 3 + (corner + 1) % 3];
            if (from >= vertexCount || to >= vertexCount)
                continue;
            const u32 a = mCanonical[from];
            const u32 b = mCanonical[to];
            if (a == b)
                continue;

            const u64 key = edgeKey(a, b);
            auto found = mEdgeIndex.find(key);
            if (found == mEdgeIndex.end())
            {
                Edge edge;
                edge.a = std::min(a, b);
                edge.b = std::max(a, b);
                found = mEdgeIndex.emplace(key, static_cast<u32>(mEdges.size())).first;
                mEdges.push_back(std::move(edge));
            }
            mEdges[found->second].faces.push_back(face);
            mFaceEdges[face][corner] = static_cast<s32>(found->second);
        }
    }
}

std::vector<u32> MeshTopology::coincident(u32 vertex) const
{
    std::vector<u32> out;
    if (vertex >= mCanonical.size())
        return out;
    const u32 target = mCanonical[vertex];
    for (u32 i = 0; i < mCanonical.size(); ++i)
    {
        if (mCanonical[i] == target)
            out.push_back(i);
    }
    return out;
}

void MeshTopology::faceNeighbors(const MeshData&, u32 face, std::vector<u32>& out) const
{
    out.clear();
    if (face >= mFaceEdges.size())
        return;
    for (const s32 edge : mFaceEdges[face])
    {
        if (edge < 0)
            continue;
        for (const u32 other : mEdges[static_cast<usize>(edge)].faces)
        {
            if (other != face && std::find(out.begin(), out.end(), other) == out.end())
                out.push_back(other);
        }
    }
}

std::vector<std::vector<u32>> MeshTopology::boundaryLoops(const MeshData& mesh) const
{
    std::unordered_map<u32, std::vector<u32>> next;
    usize boundaryCount = 0;
    for (u32 face = 0; face < mFaceEdges.size(); ++face)
    {
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const s32 edge = mFaceEdges[face][corner];
            if (edge < 0 || !isBoundary(static_cast<u32>(edge)))
                continue;
            const u32 from = mCanonical[mesh.indices[face * 3 + corner]];
            const u32 to = mCanonical[mesh.indices[face * 3 + (corner + 1) % 3]];
            next[from].push_back(to);
            ++boundaryCount;
        }
    }

    // Start from the lowest vertex so the result does not depend on hash order.
    std::vector<u32> starts;
    starts.reserve(next.size());
    for (const auto& entry : next)
        starts.push_back(entry.first);
    std::sort(starts.begin(), starts.end());

    std::vector<std::vector<u32>> loops;
    for (const u32 start : starts)
    {
        if (next[start].empty())
            continue;

        std::vector<u32> loop;
        std::vector<std::pair<u32, u32>> walked;
        u32 at = start;
        bool closed = false;
        while (true)
        {
            std::vector<u32>& options = next[at];
            if (options.size() != 1)
                break; // a dead end, or a pinch where the way on is ambiguous
            const u32 to = options.front();
            loop.push_back(at);
            walked.emplace_back(at, to);
            at = to;
            if (at == start)
            {
                closed = true;
                break;
            }
            if (loop.size() > boundaryCount)
                break;
        }

        if (closed)
        {
            for (const auto& step : walked)
                next[step.first].clear();
            loops.push_back(std::move(loop));
        }
    }
    return loops;
}
