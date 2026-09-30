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

    // Each cut triangle contributes at most 3 more; refuse before allocating.
    usize estimate = faceCount;
    for (const Tri& t : faces.tri)
    {
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

    for (u32 f = 0; f < faceCount; ++f)
    {
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
