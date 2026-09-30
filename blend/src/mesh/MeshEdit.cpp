#include "PCH.h"
#include "mesh/MeshEdit.h"

#include <cmath>
#include <unordered_set>

using namespace Radion;

namespace
{
using Tri = std::array<u32, 3>;

bool fail(std::string* error, const std::string& message)
{
    if (error)
        *error = message;
    return false;
}

// The triangles of a mesh, each tagged with the submesh it belongs to. Edits
// work on this and write it back, which keeps the submesh ranges right without
// every edit knowing about them.
struct Faces
{
    std::vector<Tri> tri;
    std::vector<u32> submesh;
};

Faces readFaces(const MeshData& mesh)
{
    Faces faces;
    const usize count = mesh.indices.size() / 3;
    faces.tri.resize(count);
    faces.submesh.assign(count, 0);
    for (usize f = 0; f < count; ++f)
        faces.tri[f] = {mesh.indices[f * 3], mesh.indices[f * 3 + 1], mesh.indices[f * 3 + 2]};

    for (u32 s = 0; s < static_cast<u32>(mesh.submeshes.size()); ++s)
    {
        const SubMesh& submesh = mesh.submeshes[s];
        const usize first = submesh.indexOffset / 3;
        const usize last = std::min(count, static_cast<usize>(submesh.indexOffset + submesh.indexCount) / 3);
        for (usize f = first; f < last; ++f)
            faces.submesh[f] = s;
    }
    return faces;
}

// Writes the triangles back grouped by submesh (in their existing order within
// each) and returns, for each written triangle, where it stood in `faces`.
std::vector<u32> writeFaces(MeshData& mesh, const Faces& faces)
{
    const usize count = faces.tri.size();
    std::vector<u32> order;
    order.reserve(count);

    if (mesh.submeshes.empty())
    {
        for (u32 f = 0; f < count; ++f)
            order.push_back(f);
    }
    else
    {
        // A stable counting sort by submesh.
        std::vector<u32> start(mesh.submeshes.size() + 1, 0);
        for (usize f = 0; f < count; ++f)
            ++start[std::min<usize>(faces.submesh[f], mesh.submeshes.size() - 1) + 1];
        for (usize s = 1; s < start.size(); ++s)
            start[s] += start[s - 1];
        order.assign(count, 0);
        std::vector<u32> cursor(start.begin(), start.end() - 1);
        for (u32 f = 0; f < count; ++f)
            order[cursor[std::min<usize>(faces.submesh[f], mesh.submeshes.size() - 1)]++] = f;

        for (usize s = 0; s < mesh.submeshes.size(); ++s)
        {
            mesh.submeshes[s].indexOffset = start[s] * 3;
            mesh.submeshes[s].indexCount = (start[s + 1] - start[s]) * 3;
        }
    }

    mesh.indices.resize(count * 3);
    for (usize n = 0; n < count; ++n)
        for (usize c = 0; c < 3; ++c)
            mesh.indices[n * 3 + c] = faces.tri[order[n]][c];

    // Bounds follow the geometry: the whole mesh's and each submesh's own.
    mesh.bounds = AABB();
    for (const glm::vec3& p : mesh.positions)
        mesh.bounds.expand(p);
    for (SubMesh& submesh : mesh.submeshes)
    {
        submesh.bounds = AABB();
        const usize end = std::min<usize>(mesh.indices.size(), static_cast<usize>(submesh.indexOffset) + submesh.indexCount);
        for (usize i = submesh.indexOffset; i < end; ++i)
            submesh.bounds.expand(mesh.positions[mesh.indices[i]]);
    }
    return order;
}

template <typename T> bool aligned(const std::vector<T>& values, usize vertexCount)
{
    return values.size() == vertexCount;
}

u32 packColor(const glm::vec4& c)
{
    auto byte = [](f32 v) { return static_cast<u32>(glm::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f); };
    return byte(c.r) | (byte(c.g) << 8) | (byte(c.b) << 16) | (byte(c.a) << 24);
}

glm::vec4 unpackColor(u32 c)
{
    return glm::vec4(static_cast<f32>(c & 0xFF), static_cast<f32>((c >> 8) & 0xFF),
                     static_cast<f32>((c >> 16) & 0xFF), static_cast<f32>((c >> 24) & 0xFF)) /
           255.0f;
}

// A new vertex a fraction `t` of the way from vertex `a` to vertex `b`, with
// every attribute array that is in step with the positions carried along.
u32 lerpVertex(MeshData& mesh, u32 a, u32 b, f32 t)
{
    const usize n = mesh.positions.size();
    const u32 index = static_cast<u32>(n);

    const glm::vec3 position = glm::mix(mesh.positions[a], mesh.positions[b], t);
    if (aligned(mesh.normals, n))
    {
        const glm::vec3 normal = glm::mix(mesh.normals[a], mesh.normals[b], t);
        const f32 length = glm::length(normal);
        mesh.normals.push_back(length > 1.0e-8f ? normal / length : mesh.normals[a]);
    }
    if (aligned(mesh.tangents, n))
    {
        const glm::vec3 tangent = glm::mix(glm::vec3(mesh.tangents[a]), glm::vec3(mesh.tangents[b]), t);
        const f32 length = glm::length(tangent);
        // Handedness is a sign, not something to blend.
        mesh.tangents.push_back(glm::vec4(length > 1.0e-8f ? tangent / length : glm::vec3(mesh.tangents[a]),
                                          mesh.tangents[a].w));
    }
    if (aligned(mesh.uvs, n))
        mesh.uvs.push_back(glm::mix(mesh.uvs[a], mesh.uvs[b], t));
    if (aligned(mesh.uvs2, n))
        mesh.uvs2.push_back(glm::mix(mesh.uvs2[a], mesh.uvs2[b], t));
    if (aligned(mesh.colors, n))
        mesh.colors.push_back(packColor(glm::mix(unpackColor(mesh.colors[a]), unpackColor(mesh.colors[b]), t)));
    if (aligned(mesh.skin, n))
        mesh.skin.push_back(t < 0.5f ? mesh.skin[a] : mesh.skin[b]);
    mesh.positions.push_back(position);
    return index;
}

// The edge an id pair names, looked up through the canonical ids of its ends -
// an id that stopped being canonical (an earlier edit merged its point into
// another) still finds the edge it used to be part of. -1 when it is gone.
s32 findEdgeById(const MeshTopology& topology, const MeshData& mesh, u64 key)
{
    const u32 a = static_cast<u32>(key >> 32);
    const u32 b = static_cast<u32>(key & 0xFFFFFFFFu);
    if (a >= mesh.positions.size() || b >= mesh.positions.size())
        return -1;
    const u32 ca = topology.canonical(a);
    const u32 cb = topology.canonical(b);
    return ca == cb ? -1 : topology.findEdge(ca, cb);
}

u64 pairKey(u32 a, u32 b)
{
    if (a > b)
        std::swap(a, b);
    return (static_cast<u64>(a) << 32) | b;
}
} // namespace

bool MeshEdit::refineEdges(MeshData& mesh, const std::vector<EdgeSplit>& splits, RefineResult* result,
                           std::string* error)
{
    return refineEdges(mesh, splits, {}, result, error);
}

bool MeshEdit::refineEdges(MeshData& mesh, const std::vector<EdgeSplit>& splits, const std::vector<QuadCut>& quads,
                           RefineResult* result, std::string* error)
{
    if (mesh.positions.empty() || mesh.indices.size() < 3)
        return fail(error, "the mesh is empty");
    if (splits.empty())
        return fail(error, "no edges to cut");

    MeshTopology topology;
    topology.build(mesh);

    std::unordered_map<u64, f32> tOf;
    for (const EdgeSplit& split : splits)
    {
        if (topology.findEdge(static_cast<u32>(split.key >> 32), static_cast<u32>(split.key & 0xFFFFFFFFu)) < 0)
            return fail(error, "an edge to cut does not exist");
        if (!(split.t > 0.0f && split.t < 1.0f))
            return fail(error, "an edge can only be cut strictly between its ends");
        tOf[split.key] = split.t;
    }

    const Faces faces = readFaces(mesh);
    const usize faceCount = faces.tri.size();

    std::vector<u8> handledAsQuad(faceCount, 0);
    for (const QuadCut& quad : quads)
    {
        if (quad.faceA >= faceCount || quad.faceB >= faceCount || quad.faceA == quad.faceB ||
            handledAsQuad[quad.faceA] || handledAsQuad[quad.faceB])
            return fail(error, "a quad cut names a triangle that is out of range or already used");
        for (const u32 c : quad.corner)
            if (c >= mesh.positions.size())
                return fail(error, "a quad cut names a vertex that does not exist");
        const u64 sideA = MeshTopology::edgeKey(topology.canonical(quad.corner[0]), topology.canonical(quad.corner[1]));
        const u64 sideB = MeshTopology::edgeKey(topology.canonical(quad.corner[2]), topology.canonical(quad.corner[3]));
        if (!tOf.count(sideA) || !tOf.count(sideB))
            return fail(error, "the sides of a quad cut are not among the edges being cut");
        handledAsQuad[quad.faceA] = handledAsQuad[quad.faceB] = 1;
    }

    // Each cut triangle contributes at most 3 more; refuse before allocating.
    usize estimate = faceCount + quads.size() * 2;
    for (usize f = 0; f < faceCount; ++f)
    {
        if (handledAsQuad[f])
            continue;
        const Tri& t = faces.tri[f];
        u32 cuts = 0;
        for (u32 c = 0; c < 3; ++c)
        {
            const u32 a = topology.canonical(t[c]);
            const u32 b = topology.canonical(t[(c + 1) % 3]);
            if (a != b && tOf.count(MeshTopology::edgeKey(a, b)))
                ++cuts;
        }
        estimate += cuts;
    }
    if (estimate > kMaxTriangles)
        return fail(error, "the result would have more than " + std::to_string(kMaxTriangles) + " triangles");

    std::unordered_map<u64, u32> midpointOf; // index pair -> vertex
    RefineResult local;

    auto midpoint = [&](u32 from, u32 to) -> u32
    {
        const u64 key = pairKey(from, to);
        const auto found = midpointOf.find(key);
        if (found != midpointOf.end())
            return found->second;

        const u32 lo = std::min(from, to);
        const u32 hi = std::max(from, to);
        const u32 cLo = topology.canonical(lo);
        const u32 cHi = topology.canonical(hi);
        const u64 edge = MeshTopology::edgeKey(cLo, cHi);
        // `t` is measured from the lower canonical end; lerp from `lo`.
        const f32 t = tOf[edge];
        const u32 vertex = lerpVertex(mesh, lo, hi, cLo < cHi ? t : 1.0f - t);
        midpointOf.emplace(key, vertex);
        local.midpoints.push_back({vertex, edge});
        return vertex;
    };

    Faces out;
    out.tri.reserve(estimate);
    out.submesh.reserve(estimate);
    std::vector<u32> origin;
    origin.reserve(estimate);

    auto emit = [&](u32 source, u32 a, u32 b, u32 c)
    {
        out.tri.push_back({a, b, c});
        out.submesh.push_back(faces.submesh[source]);
        origin.push_back(source);
    };

    // Quads first: each becomes two quads, split across the midpoints.
    for (const QuadCut& quad : quads)
    {
        const u32 p = quad.corner[0];
        const u32 q = quad.corner[1];
        const u32 r = quad.corner[2];
        const u32 s = quad.corner[3];
        const u32 mA = midpoint(p, q);
        const u32 mB = midpoint(r, s);
        auto emitQuad = [&](u32 a, u32 b, u32 c, u32 d)
        {
            // The shorter diagonal keeps the triangles from being slivers.
            if (glm::distance(mesh.positions[a], mesh.positions[c]) <= glm::distance(mesh.positions[b], mesh.positions[d]))
            {
                emit(quad.faceA, a, b, c);
                emit(quad.faceA, a, c, d);
            }
            else
            {
                emit(quad.faceA, a, b, d);
                emit(quad.faceA, b, c, d);
            }
        };
        emitQuad(p, mA, mB, s);
        emitQuad(mA, q, r, mB);
    }

    for (u32 f = 0; f < faceCount; ++f)
    {
        if (handledAsQuad[f])
            continue;
        const Tri& v = faces.tri[f];
        bool cut[3];
        u32 cuts = 0;
        for (u32 c = 0; c < 3; ++c)
        {
            const u32 a = topology.canonical(v[c]);
            const u32 b = topology.canonical(v[(c + 1) % 3]);
            cut[c] = a != b && tOf.count(MeshTopology::edgeKey(a, b)) != 0;
            cuts += cut[c] ? 1 : 0;
        }

        if (cuts == 0)
        {
            emit(f, v[0], v[1], v[2]);
        }
        else if (cuts == 1)
        {
            const u32 i = cut[0] ? 0 : cut[1] ? 1 : 2;
            const u32 a = v[i];
            const u32 b = v[(i + 1) % 3];
            const u32 c = v[(i + 2) % 3];
            const u32 m = midpoint(a, b);
            emit(f, a, m, c);
            emit(f, m, b, c);
        }
        else if (cuts == 2)
        {
            // k is the uncut edge; s the corner opposite it, where the two cut
            // edges meet.
            const u32 k = !cut[0] ? 0 : !cut[1] ? 1 : 2;
            const u32 p = v[k];
            const u32 q = v[(k + 1) % 3];
            const u32 s = v[(k + 2) % 3];
            const u32 mA = midpoint(q, s);
            const u32 mB = midpoint(s, p);
            emit(f, mA, s, mB);
            // The quad p, q, mA, mB: split along its shorter diagonal.
            const f32 dPA = glm::distance(mesh.positions[p], mesh.positions[mA]);
            const f32 dQB = glm::distance(mesh.positions[q], mesh.positions[mB]);
            if (dPA <= dQB)
            {
                emit(f, p, q, mA);
                emit(f, p, mA, mB);
            }
            else
            {
                emit(f, p, q, mB);
                emit(f, q, mA, mB);
            }
        }
        else
        {
            const u32 m0 = midpoint(v[0], v[1]);
            const u32 m1 = midpoint(v[1], v[2]);
            const u32 m2 = midpoint(v[2], v[0]);
            emit(f, v[0], m0, m2);
            emit(f, m0, v[1], m1);
            emit(f, m2, m1, v[2]);
            emit(f, m0, m1, m2);
        }
    }

    const std::vector<u32> order = writeFaces(mesh, out);
    local.origin.resize(order.size());
    for (usize n = 0; n < order.size(); ++n)
        local.origin[n] = origin[order[n]];

    for (const auto& entry : midpointOf)
        local.byVertexPair[entry.first] = entry.second;

    if (result)
        *result = std::move(local);
    return true;
}

namespace
{
// Loop's smoothing positions for one round of subdivision, worked out on the
// mesh as it is before any cut. `inRegion` says which triangles take part: an
// edge between a region triangle and anything else behaves like a border, so the
// region stays attached to what it was attached to instead of pulling away.
struct LoopPositions
{
    std::unordered_map<u32, glm::vec3> vertex; // canonical id -> new position
    std::unordered_map<u64, glm::vec3> edge;   // canonical edge key -> new point
};

LoopPositions loopPositions(const MeshData& mesh, const MeshTopology& topology,
                            const std::vector<u8>& inRegion, const std::unordered_set<u64>& cutEdges)
{
    LoopPositions out;
    const std::vector<MeshTopology::Edge>& edges = topology.edges();

    auto interior = [&](const MeshTopology::Edge& edge)
    {
        return edge.faces.size() == 2 && inRegion[edge.faces[0]] && inRegion[edge.faces[1]];
    };
    // The canonical id of a vertex is the index of its first occurrence, so its
    // position is that vertex's.
    auto at = [&](u32 canonical) -> const glm::vec3& { return mesh.positions[canonical]; };

    std::unordered_map<u32, std::vector<u32>> incident;
    for (u32 e = 0; e < edges.size(); ++e)
    {
        incident[edges[e].a].push_back(e);
        incident[edges[e].b].push_back(e);
    }

    for (const u64 key : cutEdges)
    {
        const s32 index = topology.findEdge(static_cast<u32>(key >> 32), static_cast<u32>(key & 0xFFFFFFFFu));
        if (index < 0)
            continue;
        const MeshTopology::Edge& edge = edges[static_cast<usize>(index)];
        if (!interior(edge))
        {
            out.edge[key] = (at(edge.a) + at(edge.b)) * 0.5f;
            continue;
        }
        // The vertex each of the two triangles has beyond the edge.
        glm::vec3 beyond(0.0f);
        for (const u32 face : edge.faces)
        {
            for (u32 c = 0; c < 3; ++c)
            {
                const u32 canonical = topology.canonical(mesh.indices[face * 3 + c]);
                if (canonical != edge.a && canonical != edge.b)
                {
                    beyond += at(canonical);
                    break;
                }
            }
        }
        out.edge[key] = (at(edge.a) + at(edge.b)) * (3.0f / 8.0f) + beyond * (1.0f / 8.0f);
    }

    // Only a vertex on a cut edge moves; the rest of the surface stays put.
    std::unordered_set<u32> touched;
    for (const u64 key : cutEdges)
    {
        touched.insert(static_cast<u32>(key >> 32));
        touched.insert(static_cast<u32>(key & 0xFFFFFFFFu));
    }

    for (const u32 v : touched)
    {
        const auto list = incident.find(v);
        if (list == incident.end())
            continue;

        std::vector<u32> borderNeighbours;
        glm::vec3 neighbourSum(0.0f);
        bool allInterior = true;
        for (const u32 e : list->second)
        {
            const MeshTopology::Edge& edge = edges[e];
            const u32 other = edge.a == v ? edge.b : edge.a;
            neighbourSum += at(other);
            if (!interior(edge))
            {
                allInterior = false;
                borderNeighbours.push_back(other);
            }
        }

        const glm::vec3& self = at(v);
        if (allInterior && list->second.size() >= 3)
        {
            const f32 n = static_cast<f32>(list->second.size());
            const f32 a = 3.0f / 8.0f + 0.25f * std::cos(2.0f * 3.14159265358979f / n);
            const f32 beta = (1.0f / n) * (5.0f / 8.0f - a * a);
            out.vertex[v] = self * (1.0f - n * beta) + neighbourSum * beta;
        }
        else if (borderNeighbours.size() == 2)
        {
            out.vertex[v] = self * 0.75f + (at(borderNeighbours[0]) + at(borderNeighbours[1])) * 0.125f;
        }
        // A corner where more than two borders meet stays where it is.
    }
    return out;
}
} // namespace

bool MeshEdit::subdivide(MeshData& mesh, const std::vector<u32>& selected, u32 levels, bool smooth,
                         std::string* error)
{
    if (mesh.positions.empty() || mesh.indices.size() < 3)
        return fail(error, "the mesh is empty");
    if (levels < 1 || levels > 6)
        return fail(error, "levels must be between 1 and 6");

    MeshData work = mesh;
    const usize faceTotal = work.indices.size() / 3;

    std::vector<u8> inRegion(faceTotal, selected.empty() ? 1 : 0);
    for (const u32 face : selected)
    {
        if (face >= faceTotal)
            return fail(error, "a face index is out of range");
        inRegion[face] = 1;
    }

    for (u32 level = 0; level < levels; ++level)
    {
        MeshTopology topology;
        topology.build(work);

        std::unordered_set<u64> cut;
        for (u32 f = 0; f < inRegion.size(); ++f)
        {
            if (!inRegion[f])
                continue;
            for (const s32 e : topology.faceEdges(f))
            {
                if (e >= 0)
                    cut.insert(MeshTopology::edgeKey(topology.edges()[static_cast<usize>(e)].a,
                                                     topology.edges()[static_cast<usize>(e)].b));
            }
        }
        if (cut.empty())
            return fail(error, "there is nothing to subdivide");

        LoopPositions loop;
        if (smooth)
            loop = loopPositions(work, topology, inRegion, cut);

        const usize originalVertices = work.positions.size();
        std::vector<EdgeSplit> splits;
        splits.reserve(cut.size());
        for (const u64 key : cut)
            splits.push_back({key, 0.5f});

        RefineResult refined;
        if (!refineEdges(work, splits, &refined, error))
            return false;

        if (smooth)
        {
            for (const RefineResult::Midpoint& m : refined.midpoints)
            {
                const auto point = loop.edge.find(m.edge);
                if (point != loop.edge.end())
                    work.positions[m.vertex] = point->second;
            }
            for (u32 v = 0; v < originalVertices; ++v)
            {
                const auto moved = loop.vertex.find(topology.canonical(v));
                if (moved != loop.vertex.end())
                    work.positions[v] = moved->second;
            }
        }

        std::vector<u8> next(refined.origin.size());
        for (usize n = 0; n < next.size(); ++n)
            next[n] = inRegion[refined.origin[n]];
        inRegion = std::move(next);
    }

    // Loop moved vertices after the bounds were last taken.
    work.bounds = AABB();
    for (const glm::vec3& p : work.positions)
        work.bounds.expand(p);
    for (SubMesh& submesh : work.submeshes)
    {
        submesh.bounds = AABB();
        const usize end = std::min<usize>(work.indices.size(), static_cast<usize>(submesh.indexOffset) + submesh.indexCount);
        for (usize i = submesh.indexOffset; i < end; ++i)
            submesh.bounds.expand(work.positions[work.indices[i]]);
    }

    mesh = std::move(work);
    return true;
}

bool MeshEdit::turnEdge(MeshData& mesh, u64 edgeKey, std::string* error)
{
    MeshTopology topology;
    topology.build(mesh);
    const s32 index = findEdgeById(topology, mesh, edgeKey);
    if (index < 0)
        return fail(error, "the edge does not exist");
    const MeshTopology::Edge& edge = topology.edges()[static_cast<usize>(index)];
    if (edge.faces.size() != 2)
        return fail(error, "only an edge with exactly two triangles can be turned");

    Faces faces = readFaces(mesh);
    const u32 f1 = edge.faces[0];
    const u32 f2 = edge.faces[1];
    const Tri t1 = faces.tri[f1];
    const Tri t2 = faces.tri[f2];

    // Corner i of a triangle starts the side a->b in f1; the other triangle must
    // walk it b->a, or the two do not agree on which way is out.
    auto corner = [&](const Tri& t, u32 from, u32 to) -> s32
    {
        for (u32 c = 0; c < 3; ++c)
            if (topology.canonical(t[c]) == from && topology.canonical(t[(c + 1) % 3]) == to)
                return static_cast<s32>(c);
        return -1;
    };
    s32 i = corner(t1, edge.a, edge.b);
    u32 a = edge.a;
    u32 b = edge.b;
    if (i < 0)
    {
        i = corner(t1, edge.b, edge.a);
        a = edge.b;
        b = edge.a;
    }
    const s32 j = corner(t2, b, a);
    if (i < 0 || j < 0)
        return fail(error, "the two triangles do not agree on which way is out");

    const u32 a1 = t1[static_cast<usize>(i)];
    const u32 b1 = t1[(static_cast<usize>(i) + 1) % 3];
    const u32 c1 = t1[(static_cast<usize>(i) + 2) % 3];
    const u32 b2 = t2[static_cast<usize>(j)];
    const u32 a2 = t2[(static_cast<usize>(j) + 1) % 3];
    const u32 d2 = t2[(static_cast<usize>(j) + 2) % 3];
    if (a1 != a2 || b1 != b2)
        return fail(error, "the edge is a seam (its triangles do not share vertices); turning it would tear the UVs");

    const glm::vec3& pa = mesh.positions[a1];
    const glm::vec3& pb = mesh.positions[b1];
    const glm::vec3& pc = mesh.positions[c1];
    const glm::vec3& pd = mesh.positions[d2];
    if (topology.canonical(c1) == topology.canonical(d2))
        return fail(error, "both triangles have the same third corner");

    // The flip is only valid when the quad is convex: each new triangle has to
    // face the way the old pair did.
    const glm::vec3 old1 = glm::cross(pb - pa, pc - pa);
    const glm::vec3 old2 = glm::cross(pa - pb, pd - pb);
    const glm::vec3 reference = old1 + old2;
    const glm::vec3 new1 = glm::cross(pa - pc, pd - pc); // (c, a, d)
    const glm::vec3 new2 = glm::cross(pb - pd, pc - pd); // (d, b, c)
    if (glm::dot(new1, reference) <= 1.0e-12f || glm::dot(new2, reference) <= 1.0e-12f)
        return fail(error, "the quad is not convex, so turning the edge would fold the surface");

    faces.tri[f1] = {c1, a1, d2};
    faces.tri[f2] = {d2, b1, c1};
    writeFaces(mesh, faces);
    return true;
}

bool MeshEdit::collapseEdge(MeshData& mesh, u64 edgeKey, f32 t, std::string* error)
{
    if (!(t >= 0.0f && t <= 1.0f))
        return fail(error, "t must be between 0 and 1");

    MeshTopology topology;
    topology.build(mesh);
    const s32 index = findEdgeById(topology, mesh, edgeKey);
    if (index < 0)
        return fail(error, "the edge does not exist");
    const MeshTopology::Edge& edge = topology.edges()[static_cast<usize>(index)];

    Faces faces = readFaces(mesh);
    std::unordered_set<u32> gone(edge.faces.begin(), edge.faces.end());
    if (gone.size() >= faces.tri.size())
        return fail(error, "collapsing this edge would remove every triangle");

    const glm::vec3 target = glm::mix(mesh.positions[edge.a], mesh.positions[edge.b], t);
    for (u32 v = 0; v < mesh.positions.size(); ++v)
    {
        const u32 canonical = topology.canonical(v);
        if (canonical == edge.a || canonical == edge.b)
            mesh.positions[v] = target;
    }

    Faces kept;
    for (u32 f = 0; f < faces.tri.size(); ++f)
    {
        if (gone.count(f))
            continue;
        kept.tri.push_back(faces.tri[f]);
        kept.submesh.push_back(faces.submesh[f]);
    }
    writeFaces(mesh, kept);
    return true;
}

// ---------------------------------------------------------------------- knife

bool MeshEdit::knife(MeshData& mesh, const glm::vec3& normal, f32 offset, f32 epsilon, std::vector<u64>* cutEdges,
                     std::string* error)
{
    const f32 length = glm::length(normal);
    if (!(length > 1.0e-8f))
        return fail(error, "the plane normal must not be zero");
    if (mesh.positions.empty() || mesh.indices.size() < 3)
        return fail(error, "the mesh is empty");
    const glm::vec3 n = normal / length;
    const f32 planeOffset = offset / length;
    const f32 eps = std::max(epsilon, 0.0f);

    MeshTopology topology;
    topology.build(mesh);
    auto distance = [&](u32 canonical) { return glm::dot(n, mesh.positions[canonical]) - planeOffset; };

    std::vector<EdgeSplit> splits;
    for (const MeshTopology::Edge& edge : topology.edges())
    {
        const f32 da = distance(edge.a);
        const f32 db = distance(edge.b);
        // Strictly opposite sides, and neither end close enough to call it on the plane.
        if ((da > eps && db < -eps) || (da < -eps && db > eps))
            splits.push_back({MeshTopology::edgeKey(edge.a, edge.b), da / (da - db)});
    }

    if (splits.empty())
        return fail(error, "the plane does not cut through any triangle");

    if (!refineEdges(mesh, splits, nullptr, error))
        return false;

    if (cutEdges)
    {
        // The edges that now lie in the plane: both ends on it.
        MeshTopology after;
        after.build(mesh);
        cutEdges->clear();
        for (const MeshTopology::Edge& edge : after.edges())
        {
            if (std::abs(glm::dot(n, mesh.positions[edge.a]) - planeOffset) <= std::max(eps, 1.0e-4f) &&
                std::abs(glm::dot(n, mesh.positions[edge.b]) - planeOffset) <= std::max(eps, 1.0e-4f))
                cutEdges->push_back(MeshTopology::edgeKey(edge.a, edge.b));
        }
    }
    return true;
}

// ------------------------------------------------------------------- loop cut

namespace
{
// One step of a ring: the quad the edge (p -> q) is a side of, the side opposite
// it (s -> r, so that "a fraction along p->q" lands at the same fraction along
// s->r), and the triangle beyond that opposite side, if any.
struct RingStep
{
    u32 faceA = 0; // the two triangles of the quad
    u32 faceB = 0;
    std::array<u32, 4> corner = {0, 0, 0, 0}; // p, q, r, s as vertex indices
    u32 nextP = 0;                            // the opposite side as an oriented canonical pair
    u32 nextQ = 0;
    // True when the ring was walked from the start edge's q end: the near side
    // of this quad then runs q -> p of the start, and its fraction is mirrored.
    bool reversed = false;
};

// Finds the quad on the `source` triangle's side of the oriented canonical edge
// p -> q. Returns false when the triangle is not half of a usable quad.
bool quadAcross(const MeshData& mesh, const MeshTopology& topology, const Faces& faces, u32 source, u32 p, u32 q,
                RingStep& out)
{
    const Tri& t = faces.tri[source];
    // Rotate the triangle so that p -> q is its first side: (p, q, u).
    s32 start = -1;
    for (u32 c = 0; c < 3; ++c)
        if (topology.canonical(t[c]) == p && topology.canonical(t[(c + 1) % 3]) == q)
            start = static_cast<s32>(c);
    if (start < 0)
        return false;
    const u32 ip = t[static_cast<usize>(start)];
    const u32 iq = t[(static_cast<usize>(start) + 1) % 3];
    const u32 iu = t[(static_cast<usize>(start) + 2) % 3];

    // The two sides of the triangle that are not the ring edge are candidates for
    // the quad's diagonal; the right one has a triangle beyond it that makes the
    // far side of the quad parallel to the ring edge.
    struct Candidate
    {
        bool valid = false;
        f32 score = -1.0f;
        f32 parallel = -2.0f;
        u32 other = 0;                          // the partner triangle
        std::array<u32, 4> corner = {0, 0, 0, 0};
        u32 nextP = 0;
        u32 nextQ = 0;
    } best;

    for (int which = 0; which < 2; ++which)
    {
        // which 0: diagonal u-p (partner walks p -> u, third vertex w)
        // which 1: diagonal q-u (partner walks u -> q, third vertex w)
        const u32 from = which == 0 ? iu : iq;
        const u32 to = which == 0 ? ip : iu;
        const s32 edgeIndex = topology.findEdge(topology.canonical(from), topology.canonical(to));
        if (edgeIndex < 0)
            continue;
        const MeshTopology::Edge& diagonal = topology.edges()[static_cast<usize>(edgeIndex)];
        if (diagonal.faces.size() != 2)
            continue;
        const u32 partner = diagonal.faces[0] == source ? diagonal.faces[1] : diagonal.faces[0];

        // The partner must walk the diagonal the opposite way.
        const Tri& pt = faces.tri[partner];
        s32 pc = -1;
        for (u32 c = 0; c < 3; ++c)
            if (topology.canonical(pt[c]) == topology.canonical(to) && topology.canonical(pt[(c + 1) % 3]) == topology.canonical(from))
                pc = static_cast<s32>(c);
        if (pc < 0)
            continue;
        const u32 iw = pt[(static_cast<usize>(pc) + 2) % 3];

        // Quad as p, q, r, s.
        std::array<u32, 4> corner;
        if (which == 0)
            corner = {ip, iq, iu, iw}; // (p, q, u, w): opposite side u-w, oriented w -> u
        else
            corner = {ip, iq, iw, iu}; // (p, q, w, u): opposite side w-u, oriented u -> w
        const glm::vec3 along = glm::normalize(mesh.positions[corner[1]] - mesh.positions[corner[0]]);
        const glm::vec3 far = mesh.positions[corner[2]] - mesh.positions[corner[3]]; // r - s
        const f32 farLength = glm::length(far);
        if (!(farLength > 1.0e-8f))
            continue;
        const f32 parallel = glm::dot(along, far / farLength);
        if (parallel < 0.5f)
            continue;

        // Two triangles of neighbouring quads can make a parallelogram too, so
        // parallel sides alone do not say which side is the diagonal. Quads are
        // written as two consecutive triangles, and their diagonal is nearly
        // always the longest side of either; those two hints settle it.
        const f32 diagonalLength = glm::distance(mesh.positions[from], mesh.positions[to]);
        const f32 otherA = glm::distance(mesh.positions[ip], mesh.positions[iq]);
        const f32 otherB = glm::distance(mesh.positions[which == 0 ? iq : ip], mesh.positions[iu]);
        const bool consecutive = partner + 1 == source || source + 1 == partner;
        const bool longest = diagonalLength >= std::max(otherA, otherB) - 1.0e-6f;
        const f32 score = (consecutive ? 2.0f : 0.0f) + (longest ? 1.0f : 0.0f) + 0.5f * parallel;
        if (score > best.score)
        {
            best.valid = true;
            best.score = score;
            best.parallel = parallel;
            best.other = partner;
            best.corner = corner;
            best.nextP = topology.canonical(corner[3]);
            best.nextQ = topology.canonical(corner[2]);
        }
    }

    // A quad's far side runs the same way as the near one; anything much less
    // parallel than that is two triangles that just happen to touch.
    if (!best.valid || best.parallel < 0.5f)
        return false;

    out.faceA = source;
    out.faceB = best.other;
    out.corner = best.corner;
    out.nextP = best.nextP;
    out.nextQ = best.nextQ;
    return true;
}
} // namespace

bool MeshEdit::loopCut(MeshData& mesh, u64 edgeKey, u32 cuts, std::vector<u64>* newEdges, std::string* error)
{
    if (cuts < 1 || cuts > 32)
        return fail(error, "cuts must be between 1 and 32");

    u32 p = static_cast<u32>(edgeKey >> 32);
    u32 q = static_cast<u32>(edgeKey & 0xFFFFFFFFu);
    std::vector<u64> created;

    // With n cuts the ring is cut one loop at a time: the first at 1/(n+1) of the
    // way, then the remaining stretch at 1/n of what is left, and so on, so the
    // loops come out evenly spaced without any edge having two cuts at once.
    for (u32 step = 0; step < cuts; ++step)
    {
        MeshTopology topology;
        topology.build(mesh);
        if (p >= mesh.positions.size() || q >= mesh.positions.size())
            return fail(error, "the edge does not exist");
        p = topology.canonical(p);
        q = topology.canonical(q);
        const s32 startIndex = topology.findEdge(p, q);
        if (startIndex < 0)
            return fail(error, "the edge does not exist");

        const Faces faces = readFaces(mesh);
        const f32 fraction = 1.0f / static_cast<f32>(cuts - step + 1);

        // Walk the ring both ways from the start edge, collecting quads.
        std::vector<RingStep> quads;
        std::unordered_set<u64> seenEdges;
        seenEdges.insert(MeshTopology::edgeKey(p, q));
        std::unordered_set<u32> usedFaces;

        const MeshTopology::Edge& start = topology.edges()[static_cast<usize>(startIndex)];
        // Each triangle of the start edge seeds a direction.
        for (usize seedIndex = 0; seedIndex < start.faces.size(); ++seedIndex)
        {
            u32 source = start.faces[seedIndex];
            // A triangle walks the edge one way or the other; the side it is
            // walked q -> p is the one whose fractions are mirrored.
            bool walksPToQ = false;
            for (u32 c = 0; c < 3; ++c)
            {
                if (topology.canonical(faces.tri[source][c]) == p &&
                    topology.canonical(faces.tri[source][(c + 1) % 3]) == q)
                    walksPToQ = true;
            }
            const bool reversed = !walksPToQ;
            u32 cp = reversed ? q : p;
            u32 cq = reversed ? p : q;

            while (!usedFaces.count(source))
            {
                RingStep found;
                if (!quadAcross(mesh, topology, faces, source, cp, cq, found))
                    break;
                if (usedFaces.count(found.faceB))
                    break;
                found.reversed = reversed;
                usedFaces.insert(found.faceA);
                usedFaces.insert(found.faceB);
                quads.push_back(found);

                // On to the side beyond, through the triangle on its far side
                // (not the one already in this quad).
                const u64 nextKey = MeshTopology::edgeKey(found.nextP, found.nextQ);
                if (seenEdges.count(nextKey))
                    break; // the ring has closed
                seenEdges.insert(nextKey);

                const s32 nextIndex = topology.findEdge(found.nextP, found.nextQ);
                if (nextIndex < 0)
                    break;
                const MeshTopology::Edge& next = topology.edges()[static_cast<usize>(nextIndex)];
                if (next.faces.size() != 2)
                    break;
                source = next.faces[0] == found.faceA || next.faces[0] == found.faceB ? next.faces[1] : next.faces[0];
                cp = found.nextP;
                cq = found.nextQ;
            }
        }

        if (quads.empty())
            return fail(error, "the edge is not part of a ring of quads (is it a diagonal, or a border?)");

        // Every quad is cut a fraction along its near side p -> q and the same
        // fraction along its far side s -> r, so the new edge runs straight
        // across. `t` in a split is measured from an edge's lower canonical id.
        std::vector<EdgeSplit> splits;
        std::unordered_map<u64, f32> have;
        auto addOriented = [&](u32 iFrom, u32 iTo, f32 alongFraction)
        {
            const u32 from = topology.canonical(iFrom);
            const u32 to = topology.canonical(iTo);
            const u64 key = MeshTopology::edgeKey(from, to);
            if (have.count(key))
                return;
            const f32 t = from < to ? alongFraction : 1.0f - alongFraction;
            have[key] = t;
            splits.push_back({key, t});
        };
        std::vector<QuadCut> quadCuts;
        for (const RingStep& quad : quads)
        {
            const f32 alongFraction = quad.reversed ? 1.0f - fraction : fraction;
            addOriented(quad.corner[0], quad.corner[1], alongFraction);
            addOriented(quad.corner[3], quad.corner[2], alongFraction);

            QuadCut cut;
            cut.faceA = quad.faceA;
            cut.faceB = quad.faceB;
            cut.corner = quad.corner;
            quadCuts.push_back(cut);
        }

        RefineResult result;
        if (!refineEdges(mesh, splits, quadCuts, &result, error))
            return false;

        // The next loop is cut in what remains: from the new vertex on the start
        // edge toward its far end q.
        MeshTopology after;
        after.build(mesh);
        u32 newVertex = ~0u;
        for (const RefineResult::Midpoint& m : result.midpoints)
            if (m.edge == MeshTopology::edgeKey(p, q))
                newVertex = m.vertex;
        if (newVertex == ~0u)
            return fail(error, "the cut did not reach the start edge");

        std::unordered_set<u32> loopPoints;
        for (const RefineResult::Midpoint& m : result.midpoints)
            loopPoints.insert(after.canonical(m.vertex));
        for (const MeshTopology::Edge& edge : after.edges())
            if (loopPoints.count(edge.a) && loopPoints.count(edge.b))
                created.push_back(MeshTopology::edgeKey(edge.a, edge.b));

        p = after.canonical(newVertex);
        q = after.canonical(q);
    }

    if (newEdges)
        *newEdges = std::move(created);
    return true;
}

// ---------------------------------------------------------------------- inset

bool MeshEdit::inset(MeshData& mesh, const std::vector<u32>& selected, f32 thickness, f32 depth,
                     std::vector<u32>* innerFaces, std::string* error)
{
    if (mesh.positions.empty() || mesh.indices.size() < 3)
        return fail(error, "the mesh is empty");
    if (selected.empty())
        return fail(error, "no faces are selected");
    if (!(thickness >= 0.0f) || !std::isfinite(thickness) || !std::isfinite(depth))
        return fail(error, "thickness must be zero or more, and depth a finite number");
    if (thickness == 0.0f && depth == 0.0f)
        return fail(error, "an inset with no thickness and no depth changes nothing");

    MeshTopology topology;
    topology.build(mesh);
    Faces faces = readFaces(mesh);
    const usize faceCount = faces.tri.size();

    std::vector<u8> inRegion(faceCount, 0);
    for (const u32 face : selected)
    {
        if (face >= faceCount)
            return fail(error, "a face index is out of range");
        inRegion[face] = 1;
    }

    // The region's border: edges with exactly one triangle in the region.
    struct BorderEdge
    {
        u32 a; // canonical ids, walked a -> b by the region triangle that owns it
        u32 b;
        u32 face;
        u32 indexA; // the owning triangle's own vertex indices
        u32 indexB;
    };
    std::vector<BorderEdge> border;
    std::unordered_map<u32, std::vector<u32>> borderNeighbours; // canonical -> canonical neighbours on the border
    for (const MeshTopology::Edge& edge : topology.edges())
    {
        u32 inside = 0;
        u32 owner = 0;
        for (const u32 f : edge.faces)
        {
            if (inRegion[f])
            {
                ++inside;
                owner = f;
            }
        }
        if (inside != 1)
            continue;

        const Tri& t = faces.tri[owner];
        for (u32 c = 0; c < 3; ++c)
        {
            const u32 from = topology.canonical(t[c]);
            const u32 to = topology.canonical(t[(c + 1) % 3]);
            if (MeshTopology::edgeKey(from, to) == MeshTopology::edgeKey(edge.a, edge.b))
                border.push_back({from, to, owner, t[c], t[(c + 1) % 3]});
        }
        borderNeighbours[edge.a].push_back(edge.b);
        borderNeighbours[edge.b].push_back(edge.a);
    }
    if (border.empty())
        return fail(error, "the selection has no border (it is the whole closed surface)");

    // Per canonical vertex: the triangles of the region around it, for the
    // direction it moves in and the normal it is pushed along.
    std::unordered_map<u32, glm::vec3> normalSum;
    std::unordered_map<u32, glm::vec3> centroidSum;
    std::unordered_map<u32, u32> touching;
    for (u32 f = 0; f < faceCount; ++f)
    {
        if (!inRegion[f])
            continue;
        const glm::vec3& a = mesh.positions[faces.tri[f][0]];
        const glm::vec3& b = mesh.positions[faces.tri[f][1]];
        const glm::vec3& c = mesh.positions[faces.tri[f][2]];
        const glm::vec3 n = glm::cross(b - a, c - a); // area weighted
        const glm::vec3 centroid = (a + b + c) / 3.0f;
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 canonical = topology.canonical(faces.tri[f][corner]);
            normalSum[canonical] += n;
            centroidSum[canonical] += centroid;
            ++touching[canonical];
        }
    }
    auto unit = [](const glm::vec3& v, const glm::vec3& fallback)
    {
        const f32 length = glm::length(v);
        return length > 1.0e-10f ? v / length : fallback;
    };

    // Where each border vertex goes: inward across the surface by `thickness`
    // (measured to the border's sides, not along the corner's diagonal), then
    // along the surface normal by `depth`.
    std::unordered_map<u32, glm::vec3> innerPosition;
    for (const auto& entry : borderNeighbours)
    {
        const u32 c = entry.first;
        const glm::vec3& p = mesh.positions[c];
        const glm::vec3 normal = unit(normalSum[c], glm::vec3(0, 1, 0));
        const glm::vec3 towardRegion = unit(centroidSum[c] / static_cast<f32>(touching[c]) - p, normal);

        glm::vec3 direction = towardRegion;
        f32 length = thickness;
        if (entry.second.size() == 2)
        {
            const glm::vec3 e1 = unit(mesh.positions[entry.second[0]] - p, glm::vec3(1, 0, 0));
            const glm::vec3 e2 = unit(mesh.positions[entry.second[1]] - p, glm::vec3(1, 0, 0));
            const glm::vec3 bisector = e1 + e2;
            if (glm::length(bisector) < 1.0e-4f)
            {
                // A straight border: straight in, across the surface.
                direction = unit(glm::cross(normal, e1), towardRegion);
            }
            else
            {
                direction = glm::normalize(bisector);
            }
            if (glm::dot(direction, towardRegion) < 0.0f)
                direction = -direction;
            const f32 sinHalf = std::sqrt(std::max(0.0f, (1.0f - glm::dot(e1, e2)) * 0.5f));
            length = thickness / std::max(sinHalf, 0.3f);
        }
        innerPosition[c] = p + direction * length + normal * depth;
    }

    // Interior vertices of the region only follow the depth.
    std::unordered_map<u32, glm::vec3> shifted;
    if (depth != 0.0f)
    {
        for (const auto& entry : touching)
        {
            if (!borderNeighbours.count(entry.first))
                shifted[entry.first] = mesh.positions[entry.first] + unit(normalSum[entry.first], glm::vec3(0, 1, 0)) * depth;
        }
    }

    MeshData work = mesh;
    Faces out = faces;

    // A copy of every vertex index a region triangle uses at a border point.
    std::unordered_map<u32, u32> inner;
    auto innerOf = [&](u32 index) -> u32
    {
        const auto found = inner.find(index);
        if (found != inner.end())
            return found->second;
        const u32 copy = lerpVertex(work, index, index, 0.0f);
        work.positions[copy] = innerPosition[topology.canonical(index)];
        inner[index] = copy;
        return copy;
    };

    for (u32 f = 0; f < faceCount; ++f)
    {
        if (!inRegion[f])
            continue;
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 index = faces.tri[f][corner];
            if (borderNeighbours.count(topology.canonical(index)))
                out.tri[f][corner] = innerOf(index);
        }
    }

    // Every vertex in the region that is not on the border moves in place; a
    // vertex standing at one of those points (a seam copy) moves with it.
    if (depth != 0.0f)
    {
        for (u32 v = 0; v < mesh.positions.size(); ++v)
        {
            const auto found = shifted.find(topology.canonical(v));
            if (found != shifted.end())
                work.positions[v] = found->second;
        }
    }

    // The ring between the old border and the new.
    for (const BorderEdge& edge : border)
    {
        const u32 a = edge.indexA;
        const u32 b = edge.indexB;
        const u32 ia = innerOf(a);
        const u32 ib = innerOf(b);
        const u32 submesh = faces.submesh[edge.face];
        out.tri.push_back({a, b, ib});
        out.submesh.push_back(submesh);
        out.tri.push_back({a, ib, ia});
        out.submesh.push_back(submesh);
    }

    if (out.tri.size() > kMaxTriangles)
        return fail(error, "the result would have too many triangles");

    const std::vector<u32> order = writeFaces(work, out);
    if (innerFaces)
    {
        innerFaces->clear();
        for (u32 n = 0; n < order.size(); ++n)
            if (order[n] < faceCount && inRegion[order[n]])
                innerFaces->push_back(n);
    }
    mesh = std::move(work);
    return true;
}

// --------------------------------------------------------------------- bevel

bool MeshEdit::bevel(MeshData& mesh, const std::vector<u64>& edgeKeys, f32 width, std::string* error)
{
    if (mesh.positions.empty() || mesh.indices.size() < 3)
        return fail(error, "the mesh is empty");
    if (edgeKeys.empty())
        return fail(error, "no edges to bevel");
    if (!(width > 0.0f) || !std::isfinite(width))
        return fail(error, "the width must be greater than zero");

    MeshTopology topology;
    topology.build(mesh);
    const Faces faces = readFaces(mesh);

    // What each bevelled edge needs to know, read off the two triangles on it.
    struct Bevel
    {
        u32 a = 0; // canonical ends, in the direction the first triangle walks them
        u32 b = 0;
        u32 face1 = 0; // walks a -> b
        u32 face2 = 0; // walks b -> a
        u32 iA1 = 0, iB1 = 0, iC1 = 0; // the first triangle's own vertices: a, b and the one beyond
        u32 iB2 = 0, iA2 = 0, iD2 = 0; // the second's: b, a and the one beyond
        f32 height1 = 0.0f;
        f32 height2 = 0.0f;
    };
    std::vector<Bevel> bevels;
    std::unordered_set<u32> ends;
    std::unordered_set<u64> seenKeys;

    for (const u64 key : edgeKeys)
    {
        const s32 index = findEdgeById(topology, mesh, key);
        if (index < 0)
            return fail(error, "an edge to bevel does not exist");
        const MeshTopology::Edge& edge = topology.edges()[static_cast<usize>(index)];
        if (!seenKeys.insert(MeshTopology::edgeKey(edge.a, edge.b)).second)
            continue;
        if (edge.faces.size() != 2)
            return fail(error, "only an edge with exactly two triangles can be bevelled");

        Bevel bevel;
        auto walk = [&](u32 face, u32 from, u32 to, u32& iFrom, u32& iTo, u32& iBeyond) -> bool
        {
            const Tri& t = faces.tri[face];
            for (u32 c = 0; c < 3; ++c)
            {
                if (topology.canonical(t[c]) == from && topology.canonical(t[(c + 1) % 3]) == to)
                {
                    iFrom = t[c];
                    iTo = t[(c + 1) % 3];
                    iBeyond = t[(c + 2) % 3];
                    return true;
                }
            }
            return false;
        };

        bevel.a = edge.a;
        bevel.b = edge.b;
        bevel.face1 = edge.faces[0];
        bevel.face2 = edge.faces[1];
        if (!walk(bevel.face1, bevel.a, bevel.b, bevel.iA1, bevel.iB1, bevel.iC1))
        {
            std::swap(bevel.a, bevel.b);
            if (!walk(bevel.face1, bevel.a, bevel.b, bevel.iA1, bevel.iB1, bevel.iC1))
                return fail(error, "an edge's triangle does not have it as a side");
        }
        if (!walk(bevel.face2, bevel.b, bevel.a, bevel.iB2, bevel.iA2, bevel.iD2))
            return fail(error, "the two triangles on an edge do not agree on which way is out");
        if (topology.canonical(bevel.iC1) == topology.canonical(bevel.iD2))
            return fail(error, "both triangles on an edge have the same third corner");

        if (!ends.insert(bevel.a).second || !ends.insert(bevel.b).second)
            return fail(error, "two of the edges share a vertex; bevel them one after another");

        const glm::vec3& pa = mesh.positions[bevel.a];
        const glm::vec3& pb = mesh.positions[bevel.b];
        const glm::vec3 along = pb - pa;
        const f32 edgeLength = glm::length(along);
        if (!(edgeLength > 1.0e-8f))
            return fail(error, "an edge has no length");
        bevel.height1 = glm::length(glm::cross(along, mesh.positions[bevel.iC1] - pa)) / edgeLength;
        bevel.height2 = glm::length(glm::cross(along, mesh.positions[bevel.iD2] - pa)) / edgeLength;
        if (!(bevel.height1 > 1.0e-8f) || !(bevel.height2 > 1.0e-8f))
            return fail(error, "a triangle next to an edge is flat against it");
        if (width >= 0.9f * std::min(bevel.height1, bevel.height2))
            return fail(error, "the width is larger than the triangles next to an edge allow");
        bevels.push_back(bevel);
    }

    // Cut the four sides that lead away from each edge, where a line parallel to
    // it and `width` away crosses them.
    std::vector<EdgeSplit> splits;
    std::unordered_map<u64, f32> tOf;
    auto addCut = [&](u32 from, u32 toward, f32 fraction) -> bool
    {
        const u32 cf = topology.canonical(from);
        const u32 ct = topology.canonical(toward);
        const u64 key = MeshTopology::edgeKey(cf, ct);
        const f32 t = cf < ct ? fraction : 1.0f - fraction;
        const auto found = tOf.find(key);
        if (found != tOf.end())
            return std::abs(found->second - t) < 1.0e-4f;
        tOf[key] = t;
        splits.push_back({key, t});
        return true;
    };
    for (const Bevel& bv : bevels)
    {
        const f32 s1 = width / bv.height1;
        const f32 s2 = width / bv.height2;
        if (!addCut(bv.iA1, bv.iC1, s1) || !addCut(bv.iB1, bv.iC1, s1) || !addCut(bv.iA2, bv.iD2, s2) ||
            !addCut(bv.iB2, bv.iD2, s2))
            return fail(error, "two bevels are too close: they would cut the same edge in different places");
    }

    MeshData work = mesh;
    RefineResult refined;
    if (!refineEdges(work, splits, &refined, error))
        return false;

    auto midpointOf = [&](u32 from, u32 toward) -> u32
    {
        const auto found = refined.byVertexPair.find(RefineResult::pairKey(from, toward));
        return found == refined.byVertexPair.end() ? ~0u : found->second;
    };

    MeshTopology after;
    after.build(work);
    Faces current = readFaces(work);
    // The strips added below use vertices made after `after` was built; those
    // are never one of the old bevel vertices, so they stand for themselves.
    auto canonicalOf = [&](u32 vertex) { return vertex < after.vertexCount() ? after.canonical(vertex) : vertex; };

    for (const Bevel& bv : bevels)
    {
        // Each bevel is finished before the next begins, so a triangle that
        // touches two of them is refilled once and the second bevel sees the
        // result of the first.
        std::vector<u8> removed(current.tri.size(), 0);
        std::vector<Tri> extraTris;
        std::vector<u32> extraSubmesh;

        const u32 a1 = midpointOf(bv.iA1, bv.iC1);
        const u32 b1 = midpointOf(bv.iB1, bv.iC1);
        const u32 a2 = midpointOf(bv.iA2, bv.iD2);
        const u32 b2 = midpointOf(bv.iB2, bv.iD2);
        if (a1 == ~0u || b1 == ~0u || a2 == ~0u || b2 == ~0u)
            return fail(error, "a cut vertex went missing while bevelling");

        // The strips between each old edge and its new line.
        auto inSet = [&](u32 vertex, u32 n1, u32 n2)
        {
            const u32 c = canonicalOf(vertex);
            return c == bv.a || c == bv.b || vertex == n1 || vertex == n2;
        };
        for (u32 f = 0; f < current.tri.size(); ++f)
        {
            if (removed[f])
                continue;
            const Tri& t = current.tri[f];
            const bool side1 = inSet(t[0], a1, b1) && inSet(t[1], a1, b1) && inSet(t[2], a1, b1);
            const bool side2 = inSet(t[0], a2, b2) && inSet(t[1], a2, b2) && inSet(t[2], a2, b2);
            if (side1 || side2)
                removed[f] = 1;
        }

        // The two old vertices go: what is left of the triangles around each is
        // filled in again without it, from the new vertex on one side of the
        // strip round to the new vertex on the other.
        for (const u32 old : {bv.a, bv.b})
        {
            struct Link
            {
                u32 fromIndex;
                u32 toIndex;
                u32 face;
            };
            std::unordered_map<u32, Link> nextOf; // canonical of the link edge's start
            std::unordered_set<u32> hasIncoming;
            std::vector<u32> wedge;
            for (u32 f = 0; f < current.tri.size(); ++f)
            {
                if (removed[f])
                    continue;
                const Tri& t = current.tri[f];
                for (u32 c = 0; c < 3; ++c)
                {
                    if (canonicalOf(t[c]) != old)
                        continue;
                    const u32 u = t[(c + 1) % 3];
                    const u32 v = t[(c + 2) % 3];
                    wedge.push_back(f);
                    if (nextOf.count(canonicalOf(u)))
                        return fail(error, "the triangles around a bevelled vertex do not form a simple fan");
                    nextOf[canonicalOf(u)] = {u, v, f};
                    hasIncoming.insert(canonicalOf(v));
                }
            }
            if (wedge.empty())
                continue;

            u32 startCanonical = ~0u;
            for (const auto& entry : nextOf)
            {
                if (!hasIncoming.count(entry.first))
                {
                    if (startCanonical != ~0u)
                        return fail(error, "the triangles around a bevelled vertex are not one open fan");
                    startCanonical = entry.first;
                }
            }
            if (startCanonical == ~0u)
                return fail(error, "the triangles around a bevelled vertex close into a ring");

            std::vector<u32> path; // vertex indices along the link, start to end
            u32 at = startCanonical;
            while (true)
            {
                const auto step = nextOf.find(at);
                if (step == nextOf.end())
                    break;
                path.push_back(step->second.fromIndex);
                at = canonicalOf(step->second.toIndex);
                if (path.size() > wedge.size() + 1)
                    return fail(error, "the triangles around a bevelled vertex loop back on themselves");
                if (!nextOf.count(at))
                {
                    path.push_back(step->second.toIndex);
                    break;
                }
            }
            if (path.size() < 3)
            {
                for (const u32 f : wedge)
                    removed[f] = 1;
                continue;
            }

            const u32 submesh = current.submesh[wedge.front()];
            for (const u32 f : wedge)
                removed[f] = 1;
            for (usize i = 1; i + 1 < path.size(); ++i)
            {
                extraTris.push_back({path[0], path[i], path[i + 1]});
                extraSubmesh.push_back(submesh);
            }
        }

        // The new face: a strip from the line on one side to the line on the other.
        const glm::vec3 n1 = glm::cross(mesh.positions[bv.iB1] - mesh.positions[bv.iA1],
                                        mesh.positions[bv.iC1] - mesh.positions[bv.iA1]);
        const glm::vec3 n2 = glm::cross(mesh.positions[bv.iA2] - mesh.positions[bv.iB2],
                                        mesh.positions[bv.iD2] - mesh.positions[bv.iB2]);
        const glm::vec3 outward = n1 + n2;
        // The strip gets vertices of its own so it shades as a crisp face instead
        // of blending into the faces it joins.
        std::array<u32, 4> quad = {lerpVertex(work, a1, a1, 0.0f), lerpVertex(work, b1, b1, 0.0f),
                                   lerpVertex(work, b2, b2, 0.0f), lerpVertex(work, a2, a2, 0.0f)};
        const glm::vec3 q = glm::cross(work.positions[quad[1]] - work.positions[quad[0]],
                                       work.positions[quad[2]] - work.positions[quad[0]]);
        if (glm::dot(q, outward) < 0.0f)
            quad = {quad[0], quad[3], quad[2], quad[1]};
        extraTris.push_back({quad[0], quad[1], quad[2]});
        extraTris.push_back({quad[0], quad[2], quad[3]});
        extraSubmesh.push_back(faces.submesh[bv.face1]);
        extraSubmesh.push_back(faces.submesh[bv.face1]);

        Faces next;
        for (u32 f = 0; f < current.tri.size(); ++f)
        {
            if (removed[f])
                continue;
            next.tri.push_back(current.tri[f]);
            next.submesh.push_back(current.submesh[f]);
        }
        for (usize i = 0; i < extraTris.size(); ++i)
        {
            next.tri.push_back(extraTris[i]);
            next.submesh.push_back(extraSubmesh[i]);
        }
        current = std::move(next);
    }

    writeFaces(work, current);
    removeUnusedVertices(work);
    mesh = std::move(work);
    return true;
}

u32 MeshEdit::removeUnusedVertices(MeshData& mesh)
{
    const usize count = mesh.positions.size();
    std::vector<u8> used(count, 0);
    for (const u32 index : mesh.indices)
        if (index < count)
            used[index] = 1;

    std::vector<u32> remap(count, ~0u);
    u32 kept = 0;
    for (usize v = 0; v < count; ++v)
        if (used[v])
            remap[v] = kept++;
    if (kept == count)
        return 0;

    auto compact = [&](auto& values)
    {
        if (values.size() != count)
            return;
        usize out = 0;
        for (usize v = 0; v < count; ++v)
            if (used[v])
                values[out++] = values[v];
        values.resize(out);
    };
    compact(mesh.positions);
    compact(mesh.normals);
    compact(mesh.tangents);
    compact(mesh.uvs);
    compact(mesh.uvs2);
    compact(mesh.colors);
    compact(mesh.skin);

    for (u32& index : mesh.indices)
        index = remap[index];
    return static_cast<u32>(count) - kept;
}
