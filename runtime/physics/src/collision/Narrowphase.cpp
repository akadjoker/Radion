#include "PCH.h"

#include "collision/Narrowphase.h"

namespace Radion::Physics
{

namespace
{
constexpr f32 kEpsilon = 1e-6f;

// Overlap of the two shadows on `axis`, and which way the axis has to point
// to push B off A. Negative means a gap, which ends the test.
bool axisOverlap(const CollisionShape& a, const glm::mat4& transformA, const CollisionShape& b,
                 const glm::mat4& transformB, const glm::vec3& axis, f32 margin,
                 f32& penetration, glm::vec3& normal)
{
    const f32 length = glm::length(axis);
    if (length < kEpsilon)
        return true; // degenerate axis carries no information; not a separation

    const glm::vec3 unit = axis / length;
    f32 minA, maxA, minB, maxB;
    a.project(transformA, unit, minA, maxA);
    b.project(transformB, unit, minB, maxB);

    // overlapA is what moving B along -unit costs, overlapB what moving it
    // along +unit costs. Either going negative past the margin is a gap, and
    // a gap on any one axis ends the whole test.
    const f32 overlapA = maxB - minA;
    const f32 overlapB = maxA - minB;
    if (overlapA < -margin || overlapB < -margin)
        return false;

    if (overlapA < overlapB)
    {
        penetration = overlapA;
        normal = -unit;
    }
    else
    {
        penetration = overlapB;
        normal = unit;
    }
    return true;
}

// Sutherland-Hodgman against one plane, keeping what is behind it.
u32 clipPolygon(const glm::vec3* input, u32 count, const glm::vec3& planeNormal, f32 planeOffset,
                glm::vec3* output, u32 capacity)
{
    u32 written = 0;
    for (u32 i = 0; i < count && written + 1 < capacity; ++i)
    {
        const glm::vec3& current = input[i];
        const glm::vec3& next = input[(i + 1) % count];
        const f32 distanceCurrent = glm::dot(planeNormal, current) - planeOffset;
        const f32 distanceNext = glm::dot(planeNormal, next) - planeOffset;

        if (distanceCurrent <= 0.0f)
            output[written++] = current;
        // Sign change means the edge crosses the plane; add where it does.
        if (written < capacity && distanceCurrent * distanceNext < 0.0f)
        {
            const f32 t = distanceCurrent / (distanceCurrent - distanceNext);
            output[written++] = current + (next - current) * t;
        }
    }
    return written;
}

// Face of `box` whose outward normal is most opposed to `normal` - the face
// that is actually being pressed into the other shape.
u32 incidentFace(const BoxShape& box, const glm::mat4& transform, const glm::vec3& normal)
{
    u32 best = 0;
    f32 bestDot = 1.0e30f;
    for (u32 face = 0; face < 6; ++face)
    {
        const f32 value = glm::dot(BoxShape::faceNormal(transform, face), normal);
        if (value < bestDot)
        {
            bestDot = value;
            best = face;
        }
    }
    return best;
}

u32 reducePoints(const glm::vec3* points, const f32* depths, u32 count, ContactManifold& out);

// Generous enough for a shatter shard - VoronoiShatter cells run to a handful
// of faces and tens of vertices, nowhere near this.
constexpr u32 kHullArrayCapacity = 32;

// Outward normal of face `face`, world space, from its first triangle - the
// hull's own equivalent of BoxShape::faceNormal(). Built the same way
// VoronoiShatter builds its half-space planes: cross of the first two edges
// of the face loop.
glm::vec3 hullFaceNormal(const ConvexHullShape& hull, const glm::mat4& transform, u32 face)
{
    const ConvexHullShape::Edge* edge = &hull.edges()[static_cast<usize>(hull.faces()[face])];
    const int v0 = edge->getSourceVertex();
    const int v1 = edge->getTargetVertex();
    edge = edge->getNextEdgeOfFace();
    const int v2 = edge->getTargetVertex();
    const glm::vec3& p0 = hull.vertices()[static_cast<usize>(v0)];
    const glm::vec3& p1 = hull.vertices()[static_cast<usize>(v1)];
    const glm::vec3& p2 = hull.vertices()[static_cast<usize>(v2)];
    const glm::vec3 worldRaw = glm::mat3(transform) * glm::cross(p1 - p0, p2 - p0);
    const f32 length = glm::length(worldRaw);
    return length > kEpsilon ? worldRaw / length : glm::vec3(0.0f, 1.0f, 0.0f);
}

// Hull's own face whose outward normal is most opposed to `normal` - the
// same "most opposed wins" rule incidentFace() uses for a box, applied to
// however many faces the hull actually has.
u32 hullIncidentFace(const ConvexHullShape& hull, const glm::mat4& transform,
                     const glm::vec3& normal)
{
    u32 best = 0;
    f32 bestDot = 1.0e30f;
    const u32 faceCount = hull.faceCount();
    for (u32 face = 0; face < faceCount; ++face)
    {
        const f32 value = glm::dot(hullFaceNormal(hull, transform, face), normal);
        if (value < bestDot)
        {
            bestDot = value;
            best = face;
        }
    }
    return best;
}

// Walks face `face`'s half-edge loop once, writing each vertex's world
// position in order - the polygon clipPolygon() clips against, and the
// corner list its side planes are built from. Same walk VoronoiShatter's
// volume integration uses, minus the triangle fan: here the whole loop is
// wanted, not just the triangles of it.
u32 hullFacePolygon(const ConvexHullShape& hull, const glm::mat4& transform, u32 face,
                    glm::vec3* out, u32 capacity)
{
    const ConvexHullShape::Edge* start = &hull.edges()[static_cast<usize>(hull.faces()[face])];
    const ConvexHullShape::Edge* edge = start;
    u32 count = 0;
    do
    {
        if (count >= capacity)
            break;
        out[count++] =
            glm::vec3(transform * glm::vec4(hull.vertices()[static_cast<usize>(
                                                edge->getTargetVertex())],
                                            1.0f));
        edge = edge->getNextEdgeOfFace();
    } while (edge != start);

    // ConvexHullComputer's own face loop winds the OPPOSITE way round from
    // hullFaceNormal()'s cross(v1-v0, v2-v0) - verified against a known
    // interior point, not assumed: with the raw walk above, clipFaceAgainst-
    // Face()'s side planes reject the face's own centre. Reversed, they
    // agree with the normal the same way BoxShape's kFaces winding already
    // does, and clipFaceAgainstFace() needs no shape-specific case for it.
    for (u32 i = 0, j = count > 0 ? count - 1 : 0; i < j; ++i, --j)
    {
        const glm::vec3 temporary = out[i];
        out[i] = out[j];
        out[j] = temporary;
    }
    return count;
}

// One direction per edge of the hull, world space, unnormalized - each edge
// counted once, from whichever of its two half-edges has the smaller index.
// Stands in for a box's three edge directions when SAT needs a cross-product
// axis against another shape's edges.
u32 hullEdgeDirections(const ConvexHullShape& hull, const glm::mat4& transform, glm::vec3* out,
                       u32 capacity)
{
    const std::vector<ConvexHullShape::Edge>& edges = hull.edges();
    const std::vector<glm::vec3>& vertices = hull.vertices();
    const glm::mat3 rotation(transform);
    u32 count = 0;
    for (usize i = 0; i < edges.size() && count < capacity; ++i)
    {
        const ConvexHullShape::Edge& edge = edges[i];
        const usize reverseIndex = static_cast<usize>(edge.getReverseEdge() - edges.data());
        if (reverseIndex <= i)
            continue;
        const glm::vec3 direction =
            vertices[static_cast<usize>(edge.getTargetVertex())] -
            vertices[static_cast<usize>(edge.getSourceVertex())];
        out[count++] = rotation * direction;
    }
    return count;
}

// Single contact at the midpoint of the two support points along `normal` -
// what an edge-edge separation, or a grazing face pair clipPolygon() clipped
// down to nothing, both fall back to. The same block boxBox() repeats three
// times inline, factored out for the hull routines that need it twice more.
void supportPointContact(const CollisionShape& a, const glm::mat4& transformA,
                         const CollisionShape& b, const glm::mat4& transformB, f32 penetration,
                         ContactManifold& out)
{
    out.count = 1;
    const glm::vec3 pointA = a.support(transformA, out.normal);
    const glm::vec3 pointB = b.support(transformB, -out.normal);
    out.points[0].position = (pointA + pointB) * 0.5f;
    out.points[0].penetration = penetration;
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
}

// Clips the incident face's own polygon against the reference face's side
// planes and keeps what ends up behind the reference plane - the shape-
// agnostic half of boxBox()'s face case, generalized from four corners to
// however many either polygon has.
bool clipFaceAgainstFace(const glm::vec3* referencePolygon, u32 referenceCount,
                         const glm::vec3& referenceNormal, const glm::vec3* incidentPolygon,
                         u32 incidentCount, f32 margin, ContactManifold& out)
{
    if (referenceCount < 3 || incidentCount < 3)
        return false;

    glm::vec3 polygon[kHullArrayCapacity];
    glm::vec3 scratch[kHullArrayCapacity];
    u32 count = glm::min(incidentCount, kHullArrayCapacity);
    for (u32 i = 0; i < count; ++i)
        polygon[i] = incidentPolygon[i];

    const f32 referenceOffset = glm::dot(referenceNormal, referencePolygon[0]);
    for (u32 i = 0; i < referenceCount && count > 0; ++i)
    {
        const glm::vec3& edgeStart = referencePolygon[i];
        const glm::vec3& edgeEnd = referencePolygon[(i + 1) % referenceCount];
        const glm::vec3 edge = edgeEnd - edgeStart;
        // cross(normal, edge), not cross(edge, normal) - boxBox() found the
        // hard way that the other order points the side plane into the face
        // instead of out of it. Both the box's kFaces winding and the hull's
        // own face winding go outward-CCW, so the same sign works for either.
        const glm::vec3 planeNormal = glm::cross(referenceNormal, edge);
        const f32 planeLength = glm::length(planeNormal);
        if (planeLength < kEpsilon)
            continue;
        const glm::vec3 unit = planeNormal / planeLength;
        count = clipPolygon(polygon, count, unit, glm::dot(unit, edgeStart), scratch,
                            kHullArrayCapacity);
        for (u32 p = 0; p < count; ++p)
            polygon[p] = scratch[p];
    }

    glm::vec3 kept[kHullArrayCapacity];
    f32 depths[kHullArrayCapacity];
    u32 keptCount = 0;
    for (u32 i = 0; i < count; ++i)
    {
        const f32 depth = referenceOffset - glm::dot(referenceNormal, polygon[i]);
        if (depth < -margin)
            continue;
        kept[keptCount] = polygon[i] + referenceNormal * depth;
        depths[keptCount] = depth;
        ++keptCount;
    }
    if (keptCount == 0)
        return false;

    out.count = reducePoints(kept, depths, keptCount, out);
    for (u32 i = 0; i < out.count; ++i)
    {
        out.points[i].normalImpulse = 0.0f;
        out.points[i].tangentImpulse[0] = 0.0f;
        out.points[i].tangentImpulse[1] = 0.0f;
    }
    return out.count > 0;
}

// Keeps the deepest point, then the three that are furthest from it and from
// each other, so the patch spans the real contact area instead of clustering
// in one corner. A manifold of four points all but touching is a manifold of
// one, and a box balanced on it wobbles.
u32 reducePoints(const glm::vec3* points, const f32* depths, u32 count, ContactManifold& out)
{
    if (count == 0)
        return 0;
    u32 deepest = 0;
    for (u32 i = 1; i < count; ++i)
        if (depths[i] > depths[deepest])
            deepest = i;

    if (count <= ContactManifold::MaxPoints)
    {
        // The deepest point goes first even when nothing is being dropped.
        // Anything reading points[0] as "how far in are they" - and that is
        // the obvious reading - is otherwise handed whichever corner the
        // clipper happened to emit first, which on a tilted face is not the
        // deepest one.
        out.points[0].position = points[deepest];
        out.points[0].penetration = depths[deepest];
        u32 written = 1;
        for (u32 i = 0; i < count; ++i)
        {
            if (i == deepest)
                continue;
            out.points[written].position = points[i];
            out.points[written].penetration = depths[i];
            ++written;
        }
        return written;
    }

    u32 chosen[ContactManifold::MaxPoints] = {0, 0, 0, 0};
    u32 chosenCount = 0;
    chosen[chosenCount++] = deepest;

    while (chosenCount < ContactManifold::MaxPoints)
    {
        u32 best = 0;
        f32 bestDistance = -1.0f;
        for (u32 i = 0; i < count; ++i)
        {
            bool already = false;
            for (u32 c = 0; c < chosenCount; ++c)
                if (chosen[c] == i)
                    already = true;
            if (already)
                continue;
            f32 nearest = 1.0e30f;
            for (u32 c = 0; c < chosenCount; ++c)
                nearest = glm::min(nearest, glm::length(points[i] - points[chosen[c]]));
            if (nearest > bestDistance)
            {
                bestDistance = nearest;
                best = i;
            }
        }
        if (bestDistance < 0.0f)
            break;
        chosen[chosenCount++] = best;
    }

    for (u32 i = 0; i < chosenCount; ++i)
    {
        out.points[i].position = points[chosen[i]];
        out.points[i].penetration = depths[chosen[i]];
    }
    return chosenCount;
}
} // namespace

void ContactManifold::buildTangents()
{
    // Any vector not parallel to the normal will do; picking the world axis
    // the normal leans on least keeps the cross product away from zero.
    const glm::vec3 reference = std::abs(normal.x) < 0.57735f ? glm::vec3(1.0f, 0.0f, 0.0f)
                                                              : glm::vec3(0.0f, 1.0f, 0.0f);
    tangent[0] = glm::normalize(glm::cross(reference, normal));
    tangent[1] = glm::cross(normal, tangent[0]);
}

bool Narrowphase::sphereSphere(const SphereShape& a, const glm::mat4& transformA,
                               const SphereShape& b, const glm::mat4& transformB,
                               ContactManifold& out, f32 margin)
{
    const glm::vec3 centerA(transformA[3]);
    const glm::vec3 centerB(transformB[3]);
    const glm::vec3 delta = centerB - centerA;
    const f32 distance = glm::length(delta);
    const f32 total = a.radius() + b.radius();
    if (distance >= total + margin)
        return false;

    // Concentric spheres have no direction to separate along; any one will
    // do, and picking it here beats dividing by zero.
    out.normal = distance > kEpsilon ? delta / distance : glm::vec3(0.0f, 1.0f, 0.0f);
    out.buildTangents();
    out.count = 1;
    out.points[0].penetration = total - distance;
    out.points[0].position =
        centerA + out.normal * (a.radius() - out.points[0].penetration * 0.5f);
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
    return true;
}

bool Narrowphase::sphereBox(const SphereShape& a, const glm::mat4& transformA, const BoxShape& b,
                            const glm::mat4& transformB, ContactManifold& out, f32 margin)
{
    const glm::vec3 center(transformA[3]);
    const glm::mat3 rotation(transformB);
    const glm::vec3 boxCenter(transformB[3]);
    const glm::vec3 local = glm::transpose(rotation) * (center - boxCenter);
    const glm::vec3& half = b.halfExtents();

    const glm::vec3 closestLocal = glm::clamp(local, -half, half);
    const glm::vec3 offset = local - closestLocal;
    const f32 distanceSquared = glm::dot(offset, offset);
    const f32 reach = a.radius() + margin;
    if (distanceSquared > reach * reach)
        return false;

    glm::vec3 normalLocal;
    f32 penetration = 0.0f;
    if (distanceSquared > kEpsilon * kEpsilon)
    {
        const f32 distance = std::sqrt(distanceSquared);
        normalLocal = offset / distance;
        penetration = a.radius() - distance;
    }
    else
    {
        // Centre inside the box: the closest face is the one it is least far
        // from, and the sphere has to come out through that one.
        const glm::vec3 depth = half - glm::abs(local);
        if (depth.x <= depth.y && depth.x <= depth.z)
        {
            normalLocal = glm::vec3(local.x >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
            penetration = a.radius() + depth.x;
        }
        else if (depth.y <= depth.z)
        {
            normalLocal = glm::vec3(0.0f, local.y >= 0.0f ? 1.0f : -1.0f, 0.0f);
            penetration = a.radius() + depth.y;
        }
        else
        {
            normalLocal = glm::vec3(0.0f, 0.0f, local.z >= 0.0f ? 1.0f : -1.0f);
            penetration = a.radius() + depth.z;
        }
    }

    // The manifold's normal always points from A to B, and A is the sphere.
    out.normal = -glm::normalize(rotation * normalLocal);
    out.buildTangents();
    out.count = 1;
    out.points[0].penetration = penetration;
    out.points[0].position = boxCenter + rotation * closestLocal;
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
    return true;
}

bool Narrowphase::boxBox(const BoxShape& a, const glm::mat4& transformA, const BoxShape& b,
                         const glm::mat4& transformB, ContactManifold& out, f32 margin)
{
    // Fifteen axes: three face normals each, and the nine cross products of
    // their edge directions. Without the nine, two boxes meeting edge to edge
    // come back with a face normal that is not the real separating direction,
    // and slide along it.
    const glm::mat3 rotationA(transformA);
    const glm::mat3 rotationB(transformB);

    glm::vec3 axes[15];
    u32 axisCount = 0;
    for (u32 i = 0; i < 3; ++i)
        axes[axisCount++] = glm::normalize(glm::vec3(rotationA[i]));
    for (u32 i = 0; i < 3; ++i)
        axes[axisCount++] = glm::normalize(glm::vec3(rotationB[i]));
    for (u32 i = 0; i < 3; ++i)
        for (u32 j = 0; j < 3; ++j)
            axes[axisCount++] = glm::cross(glm::vec3(rotationA[i]), glm::vec3(rotationB[j]));

    f32 bestPenetration = 1.0e30f;
    glm::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    u32 bestAxis = 0;
    for (u32 i = 0; i < axisCount; ++i)
    {
        f32 penetration = 0.0f;
        glm::vec3 normal(0.0f);
        if (!axisOverlap(a, transformA, b, transformB, axes[i], margin, penetration, normal))
            return false;
        // A parallel edge pair gives a zero-length cross product, which
        // axisOverlap() reports as "no information" - it must not then win
        // the smallest-penetration contest with a stale zero.
        if (glm::length(axes[i]) < kEpsilon)
            continue;
        if (penetration < bestPenetration)
        {
            bestPenetration = penetration;
            bestNormal = normal;
            bestAxis = i;
        }
    }

    out.normal = bestNormal;
    out.buildTangents();

    // An edge-edge separation has no face to clip against: the contact is the
    // single point where the two edges are closest, found by walking out to
    // the support point on each side.
    if (bestAxis >= 6)
    {
        out.count = 1;
        const glm::vec3 pointA = a.support(transformA, out.normal);
        const glm::vec3 pointB = b.support(transformB, -out.normal);
        out.points[0].position = (pointA + pointB) * 0.5f;
        out.points[0].penetration = bestPenetration;
        out.points[0].normalImpulse = 0.0f;
        out.points[0].tangentImpulse[0] = 0.0f;
        out.points[0].tangentImpulse[1] = 0.0f;
        return true;
    }

    // Face case: clip the incident face against the side planes of the
    // reference face, then keep whatever ends up behind the reference plane.
    const bool referenceIsA = bestAxis < 3;
    const BoxShape& reference = referenceIsA ? a : b;
    const BoxShape& incident = referenceIsA ? b : a;
    const glm::mat4& referenceTransform = referenceIsA ? transformA : transformB;
    const glm::mat4& incidentTransform = referenceIsA ? transformB : transformA;
    // The reference face is the one pointing at the other box.
    const glm::vec3 referenceDirection = referenceIsA ? out.normal : -out.normal;

    u32 referenceFace = 0;
    f32 bestDot = -1.0e30f;
    for (u32 face = 0; face < 6; ++face)
    {
        const f32 value = glm::dot(BoxShape::faceNormal(referenceTransform, face),
                                   referenceDirection);
        if (value > bestDot)
        {
            bestDot = value;
            referenceFace = face;
        }
    }

    const glm::vec3 referenceNormal = BoxShape::faceNormal(referenceTransform, referenceFace);
    glm::vec3 referenceCorner[8];
    reference.corners(referenceTransform, referenceCorner);
    const u8* referenceIndices = BoxShape::faceCorners(referenceFace);
    const f32 referenceOffset = glm::dot(referenceNormal, referenceCorner[referenceIndices[0]]);

    const u32 incidentFaceIndex = incidentFace(incident, incidentTransform, referenceNormal);
    glm::vec3 incidentCorner[8];
    incident.corners(incidentTransform, incidentCorner);
    const u8* incidentIndices = BoxShape::faceCorners(incidentFaceIndex);

    constexpr u32 capacity = 16;
    glm::vec3 polygon[capacity];
    glm::vec3 scratch[capacity];
    u32 count = 4;
    for (u32 i = 0; i < 4; ++i)
        polygon[i] = incidentCorner[incidentIndices[i]];

    // Four side planes, each spanned by one edge of the reference face and
    // its normal, all pointing outwards.
    for (u32 i = 0; i < 4 && count > 0; ++i)
    {
        const glm::vec3& edgeStart = referenceCorner[referenceIndices[i]];
        const glm::vec3& edgeEnd = referenceCorner[referenceIndices[(i + 1) % 4]];
        const glm::vec3 edge = edgeEnd - edgeStart;
        // cross(normal, edge), not cross(edge, normal). With the winding in
        // kFaces the latter points INTO the face, and clipping against it
        // throws the whole incident polygon away - a face-face contact then
        // falls back to a single support point and a box balanced on it
        // tips. Checked against all six faces, not just the one.
        const glm::vec3 planeNormal = glm::cross(referenceNormal, edge);
        const f32 planeLength = glm::length(planeNormal);
        if (planeLength < kEpsilon)
            continue;
        const glm::vec3 unit = planeNormal / planeLength;
        count = clipPolygon(polygon, count, unit, glm::dot(unit, edgeStart), scratch, capacity);
        for (u32 p = 0; p < count; ++p)
            polygon[p] = scratch[p];
    }

    glm::vec3 kept[capacity];
    f32 depths[capacity];
    u32 keptCount = 0;
    for (u32 i = 0; i < count; ++i)
    {
        const f32 depth = referenceOffset - glm::dot(referenceNormal, polygon[i]);
        // Negative depth is a point in front of the reference face. Within
        // the margin it is kept as a speculative contact; past it, dropped.
        if (depth < -margin)
            continue;
        // Reported on the reference face, which is the surface the solver
        // should push along, not where the incident corner happens to sit.
        kept[keptCount] = polygon[i] + referenceNormal * depth;
        depths[keptCount] = depth;
        ++keptCount;
    }

    if (keptCount == 0)
    {
        // Clipping produced nothing - a grazing face pair. The axis test
        // already proved they overlap, so fall back to the support points
        // rather than dropping a contact the solver was told exists.
        out.count = 1;
        const glm::vec3 pointA = a.support(transformA, out.normal);
        const glm::vec3 pointB = b.support(transformB, -out.normal);
        out.points[0].position = (pointA + pointB) * 0.5f;
        out.points[0].penetration = bestPenetration;
        out.points[0].normalImpulse = 0.0f;
        out.points[0].tangentImpulse[0] = 0.0f;
        out.points[0].tangentImpulse[1] = 0.0f;
        return true;
    }

    out.count = reducePoints(kept, depths, keptCount, out);
    for (u32 i = 0; i < out.count; ++i)
    {
        out.points[i].normalImpulse = 0.0f;
        out.points[i].tangentImpulse[0] = 0.0f;
        out.points[i].tangentImpulse[1] = 0.0f;
    }
    return true;
}

namespace
{
// Two spheres of the given radii at the given centres, written straight into
// the manifold. Capsule contacts all reduce to this once the closest points
// on the segments are known.
bool sphereContact(const glm::vec3& centerA, f32 radiusA, const glm::vec3& centerB, f32 radiusB,
                   f32 margin, ContactManifold& out)
{
    const glm::vec3 delta = centerB - centerA;
    const f32 distance = glm::length(delta);
    const f32 total = radiusA + radiusB;
    if (distance >= total + margin)
        return false;

    out.normal = distance > kEpsilon ? delta / distance : glm::vec3(0.0f, 1.0f, 0.0f);
    out.buildTangents();
    out.count = 1;
    out.points[0].penetration = total - distance;
    out.points[0].position = centerA + out.normal * (radiusA - out.points[0].penetration * 0.5f);
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
    return true;
}
} // namespace

bool Narrowphase::capsuleSphere(const CapsuleShape& a, const glm::mat4& transformA,
                                const SphereShape& b, const glm::mat4& transformB,
                                ContactManifold& out, f32 margin)
{
    glm::vec3 lower, upper;
    a.segment(transformA, lower, upper);
    const glm::vec3 center(transformB[3]);
    const glm::vec3 closest = closestPointOnSegment(lower, upper, center);
    return sphereContact(closest, a.radius(), center, b.radius(), margin, out);
}

bool Narrowphase::capsuleCapsule(const CapsuleShape& a, const glm::mat4& transformA,
                                 const CapsuleShape& b, const glm::mat4& transformB,
                                 ContactManifold& out, f32 margin)
{
    glm::vec3 lowerA, upperA, lowerB, upperB;
    a.segment(transformA, lowerA, upperA);
    b.segment(transformB, lowerB, upperB);
    glm::vec3 closestA, closestB;
    closestPointsBetweenSegments(lowerA, upperA, lowerB, upperB, closestA, closestB);
    return sphereContact(closestA, a.radius(), closestB, b.radius(), margin, out);
}

bool Narrowphase::capsuleBox(const CapsuleShape& a, const glm::mat4& transformA, const BoxShape& b,
                             const glm::mat4& transformB, ContactManifold& out, f32 margin)
{
    glm::vec3 lower, upper;
    a.segment(transformA, lower, upper);
    const glm::mat3 rotationB(transformB);
    const glm::vec3 capsuleAxis = glm::normalize(glm::vec3(transformA[1]));

    // The box's three faces, the capsule's own axis, and the three cross
    // products between them. The last set is what catches a capsule lying
    // diagonally across an edge, which no face normal separates.
    glm::vec3 axes[7];
    u32 axisCount = 0;
    for (u32 i = 0; i < 3; ++i)
        axes[axisCount++] = glm::normalize(glm::vec3(rotationB[i]));
    axes[axisCount++] = capsuleAxis;
    for (u32 i = 0; i < 3; ++i)
        axes[axisCount++] = glm::cross(capsuleAxis, glm::vec3(rotationB[i]));

    f32 bestPenetration = 1.0e30f;
    glm::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    for (u32 i = 0; i < axisCount; ++i)
    {
        f32 penetration = 0.0f;
        glm::vec3 normal(0.0f);
        if (!axisOverlap(a, transformA, b, transformB, axes[i], margin, penetration, normal))
            return false;
        if (glm::length(axes[i]) < kEpsilon)
            continue;
        if (penetration < bestPenetration)
        {
            bestPenetration = penetration;
            bestNormal = normal;
        }
    }

    out.normal = bestNormal;
    out.buildTangents();

    // Lying along a face: the segment's two ends are both in contact, and one
    // point would let the capsule pivot around it. Clipped to the face so the
    // pair sits inside the surface rather than hanging off its edge.
    const f32 alignment = std::abs(glm::dot(capsuleAxis, out.normal));
    if (alignment < 0.05f)
    {
        // The box face TOWARDS the capsule, which is its extreme against the
        // normal - the normal runs from capsule to box, so supporting along
        // it lands on the far side of the box instead.
        const glm::vec3 onBoxFace = b.support(transformB, -out.normal);
        const f32 faceOffset = glm::dot(out.normal, onBoxFace);

        glm::vec3 points[2] = {lower, upper};
        u32 written = 0;
        for (u32 i = 0; i < 2; ++i)
        {
            // The capsule's own surface is its segment pushed a radius ALONG
            // the normal, that being the direction of the box.
            const glm::vec3 surface = points[i] + out.normal * a.radius();
            const f32 depth = glm::dot(out.normal, surface) - faceOffset;
            if (depth < -margin)
                continue;
            out.points[written].position = surface - out.normal * (depth * 0.5f);
            out.points[written].penetration = depth;
            out.points[written].normalImpulse = 0.0f;
            out.points[written].tangentImpulse[0] = 0.0f;
            out.points[written].tangentImpulse[1] = 0.0f;
            ++written;
        }
        if (written == 2)
        {
            out.count = 2;
            return true;
        }
    }

    // Anything else is one point: the closest point on the segment to the
    // box, pushed out by the radius.
    const glm::vec3 boxCenter(transformB[3]);
    const glm::vec3 nearSegment = closestPointOnSegment(lower, upper, boxCenter);
    const glm::vec3 localNear = glm::transpose(rotationB) * (nearSegment - boxCenter);
    const glm::vec3 onBox =
        boxCenter + rotationB * glm::clamp(localNear, -b.halfExtents(), b.halfExtents());
    const glm::vec3 refined = closestPointOnSegment(lower, upper, onBox);

    out.count = 1;
    out.points[0].penetration = bestPenetration;
    // Midway between the box's surface and the capsule's, which is its
    // segment pushed a radius along the normal - towards the box, not away.
    out.points[0].position = (onBox + (refined + out.normal * a.radius())) * 0.5f;
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
    return true;
}

bool Narrowphase::convexHullSphere(const ConvexHullShape& a, const glm::mat4& transformA,
                                   const SphereShape& b, const glm::mat4& transformB,
                                   ContactManifold& out, f32 margin)
{
    // A sphere has no faces or edges of its own to separate on, so the
    // hull's own face normals are the only axes SAT needs - the same
    // reasoning sphereBox() uses for its single face-or-corner test, just
    // over however many faces the hull has instead of six.
    const u32 faceCount = a.faceCount();
    if (faceCount == 0)
        return false;

    f32 bestPenetration = 1.0e30f;
    glm::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    for (u32 face = 0; face < faceCount; ++face)
    {
        const glm::vec3 axis = hullFaceNormal(a, transformA, face);
        f32 penetration = 0.0f;
        glm::vec3 normal(0.0f);
        if (!axisOverlap(a, transformA, b, transformB, axis, margin, penetration, normal))
            return false;
        if (penetration < bestPenetration)
        {
            bestPenetration = penetration;
            bestNormal = normal;
        }
    }

    out.normal = bestNormal;
    out.buildTangents();
    supportPointContact(a, transformA, b, transformB, bestPenetration, out);
    return true;
}

bool Narrowphase::convexHullCapsule(const ConvexHullShape& a, const glm::mat4& transformA,
                                    const CapsuleShape& b, const glm::mat4& transformB,
                                    ContactManifold& out, f32 margin)
{
    glm::vec3 lower, upper;
    b.segment(transformB, lower, upper);
    const glm::vec3 capsuleAxis = glm::normalize(glm::vec3(transformB[1]));

    const u32 hullFaceCount = a.faceCount();
    glm::vec3 hullEdges[kHullArrayCapacity];
    const u32 hullEdgeCount = hullEdgeDirections(a, transformA, hullEdges, kHullArrayCapacity);

    // The hull's own face normals, the capsule's axis, and the cross of that
    // axis with every one of the hull's edge directions - the same three
    // groups capsuleBox() tests against a box's three faces and three edge
    // directions, generalized to however many the hull actually has.
    glm::vec3 axes[kHullArrayCapacity * 2 + 1];
    u32 axisCount = 0;
    const u32 axisFaceCount = glm::min(hullFaceCount, kHullArrayCapacity);
    for (u32 face = 0; face < axisFaceCount; ++face)
        axes[axisCount++] = hullFaceNormal(a, transformA, face);
    axes[axisCount++] = capsuleAxis;
    for (u32 i = 0; i < hullEdgeCount; ++i)
        axes[axisCount++] = glm::cross(capsuleAxis, hullEdges[i]);

    f32 bestPenetration = 1.0e30f;
    glm::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    for (u32 i = 0; i < axisCount; ++i)
    {
        f32 penetration = 0.0f;
        glm::vec3 normal(0.0f);
        if (!axisOverlap(a, transformA, b, transformB, axes[i], margin, penetration, normal))
            return false;
        if (glm::length(axes[i]) < kEpsilon)
            continue;
        if (penetration < bestPenetration)
        {
            bestPenetration = penetration;
            bestNormal = normal;
        }
    }

    out.normal = bestNormal;
    out.buildTangents();

    const f32 alignment = std::abs(glm::dot(capsuleAxis, out.normal));
    if (alignment < 0.05f && hullFaceCount > 0)
    {
        // The hull face TOWARDS the capsule - out.normal already points hull
        // to capsule, so the reference face is whichever one's own normal
        // agrees with it most, the same max-dot search boxBox() runs for its
        // reference face.
        u32 referenceFace = 0;
        f32 bestDot = -1.0e30f;
        for (u32 face = 0; face < hullFaceCount; ++face)
        {
            const f32 value = glm::dot(hullFaceNormal(a, transformA, face), out.normal);
            if (value > bestDot)
            {
                bestDot = value;
                referenceFace = face;
            }
        }
        glm::vec3 facePolygon[kHullArrayCapacity];
        const u32 faceVertexCount =
            hullFacePolygon(a, transformA, referenceFace, facePolygon, kHullArrayCapacity);
        if (faceVertexCount > 0)
        {
            const f32 faceOffset = glm::dot(out.normal, facePolygon[0]);
            const glm::vec3 points[2] = {lower, upper};
            ContactPoint generated[2];
            u32 written = 0;
            for (u32 i = 0; i < 2; ++i)
            {
                // The capsule's own surface, pushed a radius towards the
                // hull - which is -out.normal here, since out.normal points
                // hull to capsule and this end has to face back the other
                // way.
                const glm::vec3 surface = points[i] - out.normal * b.radius();
                const f32 depth = faceOffset - glm::dot(out.normal, surface);
                if (depth < -margin)
                    continue;
                generated[written].position = surface + out.normal * (depth * 0.5f);
                generated[written].penetration = depth;
                generated[written].normalImpulse = 0.0f;
                generated[written].tangentImpulse[0] = 0.0f;
                generated[written].tangentImpulse[1] = 0.0f;
                ++written;
            }
            if (written == 2)
            {
                out.count = 2;
                out.points[0] = generated[0];
                out.points[1] = generated[1];
                return true;
            }
        }
    }

    // Anything else is one point: the support points on each shape along the
    // separating normal, the same fallback boxBox() and capsuleBox() both
    // take when there is no face to clip against.
    supportPointContact(a, transformA, b, transformB, bestPenetration, out);
    return true;
}

bool Narrowphase::convexHullBox(const ConvexHullShape& a, const glm::mat4& transformA,
                                const BoxShape& b, const glm::mat4& transformB,
                                ContactManifold& out, f32 margin)
{
    const glm::mat3 rotationB(transformB);
    // Clamped once here and used for every axis-array index below, so the
    // fixed-size arrays and the bestAxis category thresholds stay in
    // agreement even on a hull with more faces than kHullArrayCapacity.
    const u32 hullFaceCount = glm::min(a.faceCount(), kHullArrayCapacity);

    glm::vec3 hullEdges[kHullArrayCapacity];
    const u32 hullEdgeCount = hullEdgeDirections(a, transformA, hullEdges, kHullArrayCapacity);

    // The hull's own face normals stand in for boxBox()'s first three axes,
    // the box's three faces are its second three, and the cross products run
    // over the hull's actual edge directions against the box's three instead
    // of a fixed 3x3 grid.
    glm::vec3 axes[kHullArrayCapacity + 3 + kHullArrayCapacity * 3];
    u32 axisCount = 0;
    for (u32 face = 0; face < hullFaceCount; ++face)
        axes[axisCount++] = hullFaceNormal(a, transformA, face);
    for (u32 i = 0; i < 3; ++i)
        axes[axisCount++] = glm::normalize(glm::vec3(rotationB[i]));
    for (u32 i = 0; i < hullEdgeCount; ++i)
        for (u32 j = 0; j < 3; ++j)
            axes[axisCount++] = glm::cross(hullEdges[i], glm::vec3(rotationB[j]));

    f32 bestPenetration = 1.0e30f;
    glm::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    u32 bestAxis = 0;
    for (u32 i = 0; i < axisCount; ++i)
    {
        f32 penetration = 0.0f;
        glm::vec3 normal(0.0f);
        if (!axisOverlap(a, transformA, b, transformB, axes[i], margin, penetration, normal))
            return false;
        if (glm::length(axes[i]) < kEpsilon)
            continue;
        if (penetration < bestPenetration)
        {
            bestPenetration = penetration;
            bestNormal = normal;
            bestAxis = i;
        }
    }

    out.normal = bestNormal;
    out.buildTangents();

    if (bestAxis >= hullFaceCount + 3)
    {
        supportPointContact(a, transformA, b, transformB, bestPenetration, out);
        return true;
    }

    glm::vec3 referencePolygon[kHullArrayCapacity];
    u32 referenceCount = 0;
    glm::vec3 referenceNormal(0.0f);
    glm::vec3 incidentPolygon[kHullArrayCapacity];
    u32 incidentCount = 0;

    if (bestAxis < hullFaceCount)
    {
        // The hull is the reference shape: its own face pointing at the box.
        u32 referenceFace = 0;
        f32 bestDot = -1.0e30f;
        for (u32 face = 0; face < hullFaceCount; ++face)
        {
            const f32 value = glm::dot(hullFaceNormal(a, transformA, face), out.normal);
            if (value > bestDot)
            {
                bestDot = value;
                referenceFace = face;
            }
        }
        referenceNormal = hullFaceNormal(a, transformA, referenceFace);
        referenceCount =
            hullFacePolygon(a, transformA, referenceFace, referencePolygon, kHullArrayCapacity);

        const u32 incidentFaceIndex = incidentFace(b, transformB, referenceNormal);
        glm::vec3 boxCorners[8];
        b.corners(transformB, boxCorners);
        const u8* indices = BoxShape::faceCorners(incidentFaceIndex);
        incidentCount = 4;
        for (u32 i = 0; i < 4; ++i)
            incidentPolygon[i] = boxCorners[indices[i]];
    }
    else
    {
        // The box is the reference shape, pointing at the hull - the same
        // inline max-dot search boxBox() runs for its own reference face.
        u32 referenceFace = 0;
        f32 bestDot = -1.0e30f;
        for (u32 face = 0; face < 6; ++face)
        {
            const f32 value = glm::dot(BoxShape::faceNormal(transformB, face), -out.normal);
            if (value > bestDot)
            {
                bestDot = value;
                referenceFace = face;
            }
        }
        referenceNormal = BoxShape::faceNormal(transformB, referenceFace);
        glm::vec3 boxCorners[8];
        b.corners(transformB, boxCorners);
        const u8* indices = BoxShape::faceCorners(referenceFace);
        referenceCount = 4;
        for (u32 i = 0; i < 4; ++i)
            referencePolygon[i] = boxCorners[indices[i]];

        const u32 incidentFaceIndex = hullIncidentFace(a, transformA, referenceNormal);
        incidentCount =
            hullFacePolygon(a, transformA, incidentFaceIndex, incidentPolygon, kHullArrayCapacity);
    }

    if (!clipFaceAgainstFace(referencePolygon, referenceCount, referenceNormal, incidentPolygon,
                             incidentCount, margin, out))
        supportPointContact(a, transformA, b, transformB, bestPenetration, out);
    return true;
}

bool Narrowphase::convexHullConvexHull(const ConvexHullShape& a, const glm::mat4& transformA,
                                       const ConvexHullShape& b, const glm::mat4& transformB,
                                       ContactManifold& out, f32 margin)
{
    // Clamped once here and used for every axis-array index below, so the
    // fixed-size arrays and the bestAxis category thresholds stay in
    // agreement even on a hull with more faces than kHullArrayCapacity.
    const u32 faceCountA = glm::min(a.faceCount(), kHullArrayCapacity);
    const u32 faceCountB = glm::min(b.faceCount(), kHullArrayCapacity);

    glm::vec3 edgesA[kHullArrayCapacity];
    const u32 edgeCountA = hullEdgeDirections(a, transformA, edgesA, kHullArrayCapacity);
    glm::vec3 edgesB[kHullArrayCapacity];
    const u32 edgeCountB = hullEdgeDirections(b, transformB, edgesB, kHullArrayCapacity);

    // A's face normals, B's face normals, then the cross product of every
    // unique edge direction of A against every unique edge direction of B -
    // the same 3x3 edge grid boxBox() runs, generalized to however many edges
    // either hull actually has.
    glm::vec3 axes[kHullArrayCapacity * 2 + kHullArrayCapacity * kHullArrayCapacity];
    u32 axisCount = 0;
    for (u32 face = 0; face < faceCountA; ++face)
        axes[axisCount++] = hullFaceNormal(a, transformA, face);
    for (u32 face = 0; face < faceCountB; ++face)
        axes[axisCount++] = hullFaceNormal(b, transformB, face);
    for (u32 i = 0; i < edgeCountA; ++i)
        for (u32 j = 0; j < edgeCountB; ++j)
            axes[axisCount++] = glm::cross(edgesA[i], edgesB[j]);

    f32 bestPenetration = 1.0e30f;
    glm::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    u32 bestAxis = 0;
    for (u32 i = 0; i < axisCount; ++i)
    {
        f32 penetration = 0.0f;
        glm::vec3 normal(0.0f);
        if (!axisOverlap(a, transformA, b, transformB, axes[i], margin, penetration, normal))
            return false;
        if (glm::length(axes[i]) < kEpsilon)
            continue;
        if (penetration < bestPenetration)
        {
            bestPenetration = penetration;
            bestNormal = normal;
            bestAxis = i;
        }
    }

    out.normal = bestNormal;
    out.buildTangents();

    if (bestAxis >= faceCountA + faceCountB)
    {
        supportPointContact(a, transformA, b, transformB, bestPenetration, out);
        return true;
    }

    glm::vec3 referencePolygon[kHullArrayCapacity];
    u32 referenceCount = 0;
    glm::vec3 referenceNormal(0.0f);
    glm::vec3 incidentPolygon[kHullArrayCapacity];
    u32 incidentCount = 0;

    if (bestAxis < faceCountA)
    {
        u32 referenceFace = 0;
        f32 bestDot = -1.0e30f;
        for (u32 face = 0; face < faceCountA; ++face)
        {
            const f32 value = glm::dot(hullFaceNormal(a, transformA, face), out.normal);
            if (value > bestDot)
            {
                bestDot = value;
                referenceFace = face;
            }
        }
        referenceNormal = hullFaceNormal(a, transformA, referenceFace);
        referenceCount =
            hullFacePolygon(a, transformA, referenceFace, referencePolygon, kHullArrayCapacity);

        const u32 incidentFaceIndex = hullIncidentFace(b, transformB, referenceNormal);
        incidentCount =
            hullFacePolygon(b, transformB, incidentFaceIndex, incidentPolygon, kHullArrayCapacity);
    }
    else
    {
        u32 referenceFace = 0;
        f32 bestDot = -1.0e30f;
        for (u32 face = 0; face < faceCountB; ++face)
        {
            const f32 value = glm::dot(hullFaceNormal(b, transformB, face), -out.normal);
            if (value > bestDot)
            {
                bestDot = value;
                referenceFace = face;
            }
        }
        referenceNormal = hullFaceNormal(b, transformB, referenceFace);
        referenceCount =
            hullFacePolygon(b, transformB, referenceFace, referencePolygon, kHullArrayCapacity);

        const u32 incidentFaceIndex = hullIncidentFace(a, transformA, referenceNormal);
        incidentCount =
            hullFacePolygon(a, transformA, incidentFaceIndex, incidentPolygon, kHullArrayCapacity);
    }

    if (!clipFaceAgainstFace(referencePolygon, referenceCount, referenceNormal, incidentPolygon,
                             incidentCount, margin, out))
        supportPointContact(a, transformA, b, transformB, bestPenetration, out);
    return true;
}

bool Narrowphase::convexPlane(const CollisionShape& a, const glm::mat4& transformA,
                              const PlaneShape& b, const glm::mat4& transformB,
                              ContactManifold& out, f32 margin)
{
    const glm::vec3 planeNormal = glm::normalize(glm::mat3(transformB) * b.normal());
    const glm::vec3 planeOrigin =
        glm::vec3(transformB * glm::vec4(b.normal() * b.constant(), 1.0f));
    const f32 planeConstant = glm::dot(planeNormal, planeOrigin);
    const glm::vec3 vertex = a.support(transformA, -planeNormal);
    const f32 distance = glm::dot(planeNormal, vertex) - planeConstant;
    if (distance > margin)
        return false;

    out.normal = -planeNormal;
    out.buildTangents();
    out.count = 1;
    out.points[0].position = vertex - distance * planeNormal;
    out.points[0].penetration = -distance;
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
    return true;
}

bool Narrowphase::collide(const CollisionShape& a, const glm::mat4& transformA,
                          const CollisionShape& b, const glm::mat4& transformB,
                          ContactManifold& out, f32 margin)
{
    if (b.type() == ShapeType::Plane &&
        (a.type() == ShapeType::Sphere || a.type() == ShapeType::Box ||
         a.type() == ShapeType::Capsule || a.type() == ShapeType::ConvexHull))
        return convexPlane(a, transformA, static_cast<const PlaneShape&>(b), transformB, out,
                           margin);

    if (a.type() == ShapeType::Plane &&
        (b.type() == ShapeType::Sphere || b.type() == ShapeType::Box ||
         b.type() == ShapeType::Capsule || b.type() == ShapeType::ConvexHull))
    {
        if (!convexPlane(b, transformB, static_cast<const PlaneShape&>(a), transformA, out,
                         margin))
            return false;
        out.normal = -out.normal;
        out.buildTangents();
        return true;
    }

    if (a.type() == ShapeType::Sphere && b.type() == ShapeType::Sphere)
        return sphereSphere(static_cast<const SphereShape&>(a), transformA,
                            static_cast<const SphereShape&>(b), transformB, out, margin);

    if (a.type() == ShapeType::Sphere && b.type() == ShapeType::Box)
        return sphereBox(static_cast<const SphereShape&>(a), transformA,
                         static_cast<const BoxShape&>(b), transformB, out, margin);

    if (a.type() == ShapeType::Box && b.type() == ShapeType::Sphere)
    {
        // Solved in the other order and flipped, so there is one sphere-box
        // routine to be right rather than two to keep agreeing.
        if (!sphereBox(static_cast<const SphereShape&>(b), transformB,
                       static_cast<const BoxShape&>(a), transformA, out, margin))
            return false;
        out.normal = -out.normal;
        out.buildTangents();
        return true;
    }

    if (a.type() == ShapeType::Box && b.type() == ShapeType::Box)
        return boxBox(static_cast<const BoxShape&>(a), transformA,
                      static_cast<const BoxShape&>(b), transformB, out, margin);

    // Every capsule pair is solved with the capsule first and the result
    // flipped when it was asked the other way round, so there is one routine
    // per pair to be right about rather than two to keep in agreement.
    if (a.type() == ShapeType::Capsule && b.type() == ShapeType::Sphere)
        return capsuleSphere(static_cast<const CapsuleShape&>(a), transformA,
                             static_cast<const SphereShape&>(b), transformB, out, margin);
    if (a.type() == ShapeType::Capsule && b.type() == ShapeType::Capsule)
        return capsuleCapsule(static_cast<const CapsuleShape&>(a), transformA,
                              static_cast<const CapsuleShape&>(b), transformB, out, margin);
    if (a.type() == ShapeType::Capsule && b.type() == ShapeType::Box)
        return capsuleBox(static_cast<const CapsuleShape&>(a), transformA,
                          static_cast<const BoxShape&>(b), transformB, out, margin);

    if (b.type() == ShapeType::Capsule &&
        (a.type() == ShapeType::Sphere || a.type() == ShapeType::Box))
    {
        if (!collide(b, transformB, a, transformA, out, margin))
            return false;
        out.normal = -out.normal;
        out.buildTangents();
        return true;
    }

    // Every hull pair is solved with the hull first and the result flipped
    // when it was asked the other way round, the same rule every other pair
    // above already follows.
    if (a.type() == ShapeType::ConvexHull && b.type() == ShapeType::ConvexHull)
        return convexHullConvexHull(static_cast<const ConvexHullShape&>(a), transformA,
                                    static_cast<const ConvexHullShape&>(b), transformB, out,
                                    margin);
    if (a.type() == ShapeType::ConvexHull && b.type() == ShapeType::Sphere)
        return convexHullSphere(static_cast<const ConvexHullShape&>(a), transformA,
                                static_cast<const SphereShape&>(b), transformB, out, margin);
    if (a.type() == ShapeType::ConvexHull && b.type() == ShapeType::Box)
        return convexHullBox(static_cast<const ConvexHullShape&>(a), transformA,
                             static_cast<const BoxShape&>(b), transformB, out, margin);
    if (a.type() == ShapeType::ConvexHull && b.type() == ShapeType::Capsule)
        return convexHullCapsule(static_cast<const ConvexHullShape&>(a), transformA,
                                 static_cast<const CapsuleShape&>(b), transformB, out, margin);

    if (b.type() == ShapeType::ConvexHull &&
        (a.type() == ShapeType::Sphere || a.type() == ShapeType::Box ||
         a.type() == ShapeType::Capsule))
    {
        if (!collide(b, transformB, a, transformA, out, margin))
            return false;
        out.normal = -out.normal;
        out.buildTangents();
        return true;
    }

    return false;
}

namespace
{

// The normal a triangle contact should push along, A towards B.
//
// On the face it is the face normal. On a rim edge it is the direction to the
// closest point, which is what makes a body slide off a real edge. On an edge
// or corner SHARED with another triangle it is the face normal again: that
// seam is interior to the surface, and pushing along it is what stops a
// character dead when he walks from one triangle onto the next.
bool triangleContactNormal(const TriangleShape& triangle, const glm::mat4& transform,
                           TriangleFeature feature, const glm::vec3& offset, f32 distanceSquared,
                           glm::vec3& out)
{
    const bool degenerate = distanceSquared <= kEpsilon * kEpsilon;
    if (!degenerate && !triangle.featureIsInternal(feature))
    {
        out = offset / std::sqrt(distanceSquared);
        return true;
    }

    const glm::vec3 raw = glm::mat3(transform) * triangle.rawNormal();
    const f32 length = glm::length(raw);
    if (length < kEpsilon)
        return false;
    glm::vec3 faceNormal = raw / length;
    // Oriented to agree with which side the convex is actually on, so a body
    // under a ceiling triangle is not pushed up through it.
    if (!degenerate && glm::dot(faceNormal, offset) < 0.0f)
        faceNormal = -faceNormal;
    out = faceNormal;
    return true;
}

} // namespace

bool Narrowphase::sphereTriangle(const SphereShape& a, const glm::mat4& transformA,
                                 const TriangleShape& b, const glm::mat4& transformB,
                                 ContactManifold& out, f32 margin)
{
    const glm::vec3 center(transformA[3]);
    const glm::vec3 v0 = glm::vec3(transformB * glm::vec4(b.vertex(0), 1.0f));
    const glm::vec3 v1 = glm::vec3(transformB * glm::vec4(b.vertex(1), 1.0f));
    const glm::vec3 v2 = glm::vec3(transformB * glm::vec4(b.vertex(2), 1.0f));

    TriangleFeature feature = TriangleFeature::Face;
    const glm::vec3 closest = closestPointOnTriangle(v0, v1, v2, center, &feature);
    const glm::vec3 offset = closest - center;
    const f32 distanceSquared = glm::dot(offset, offset);
    const f32 reach = a.radius() + margin;
    if (distanceSquared > reach * reach)
        return false;

    glm::vec3 normal;
    if (!triangleContactNormal(b, transformB, feature, offset, distanceSquared, normal))
        return false;

    out.normal = normal;
    out.buildTangents();
    out.count = 1;
    out.points[0].position = closest;
    out.points[0].penetration = a.radius() - std::sqrt(distanceSquared);
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
    return true;
}

bool Narrowphase::boxTriangle(const BoxShape& a, const glm::mat4& transformA,
                              const TriangleShape& b, const glm::mat4& transformB,
                              ContactManifold& out, f32 margin)
{
    const glm::mat3 rotationA(transformA);
    const glm::vec3 v0 = glm::vec3(transformB * glm::vec4(b.vertex(0), 1.0f));
    const glm::vec3 v1 = glm::vec3(transformB * glm::vec4(b.vertex(1), 1.0f));
    const glm::vec3 v2 = glm::vec3(transformB * glm::vec4(b.vertex(2), 1.0f));

    const glm::vec3 faceNormalRaw = glm::cross(v1 - v0, v2 - v0);
    const f32 faceLength = glm::length(faceNormalRaw);
    if (faceLength < kEpsilon)
        return false;
    const glm::vec3 faceNormal = faceNormalRaw / faceLength;

    // Thirteen axes: the box's three face normals, the triangle's one, and
    // the nine cross products of the box's edges with the triangle's.
    const glm::vec3 triangleEdges[3] = {v1 - v0, v2 - v1, v0 - v2};
    glm::vec3 axes[13];
    u32 axisCount = 0;
    for (u32 i = 0; i < 3; ++i)
        axes[axisCount++] = glm::normalize(glm::vec3(rotationA[i]));
    axes[axisCount++] = faceNormal;
    for (u32 i = 0; i < 3; ++i)
        for (u32 j = 0; j < 3; ++j)
            axes[axisCount++] = glm::cross(glm::vec3(rotationA[i]), triangleEdges[j]);

    f32 bestPenetration = std::numeric_limits<f32>::max();
    glm::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    u32 bestAxis = 0;
    for (u32 i = 0; i < axisCount; ++i)
    {
        f32 penetration = 0.0f;
        glm::vec3 normal(0.0f);
        if (!axisOverlap(a, transformA, b, transformB, axes[i], margin, penetration, normal))
            return false;
        // Same trap boxBox() guards: a zero-length cross product carries no
        // information, and must not win the contest with a stale zero.
        if (glm::length(axes[i]) < kEpsilon)
            continue;
        if (penetration < bestPenetration)
        {
            bestPenetration = penetration;
            bestNormal = normal;
            bestAxis = i;
        }
    }

    out.normal = bestNormal;
    out.buildTangents();

    // Edge against edge, with no face on either side to clip: one point,
    // where the two are closest. Same fallback boxBox() takes.
    if (bestAxis > 3)
    {
        out.count = 1;
        const glm::vec3 pointA = a.support(transformA, out.normal);
        const glm::vec3 pointB = b.support(transformB, -out.normal);
        out.points[0].position = (pointA + pointB) * 0.5f;
        out.points[0].penetration = bestPenetration;
        out.points[0].normalImpulse = 0.0f;
        out.points[0].tangentImpulse[0] = 0.0f;
        out.points[0].tangentImpulse[1] = 0.0f;
        return true;
    }

    constexpr u32 capacity = 16;
    glm::vec3 polygon[capacity];
    glm::vec3 scratch[capacity];
    u32 count = 0;
    glm::vec3 referenceNormal(0.0f);
    f32 referenceOffset = 0.0f;
    glm::vec3 referenceInterior(0.0f);
    glm::vec3 edgeStart[4];
    glm::vec3 edgeEnd[4];
    u32 edgeCount = 0;

    glm::vec3 boxCorners[8];
    a.corners(transformA, boxCorners);

    if (bestAxis < 3)
    {
        // Box face is the reference: the one pointing at the triangle.
        u32 referenceFace = 0;
        f32 bestDot = -1.0e30f;
        for (u32 face = 0; face < 6; ++face)
        {
            const f32 value = glm::dot(BoxShape::faceNormal(transformA, face), out.normal);
            if (value > bestDot)
            {
                bestDot = value;
                referenceFace = face;
            }
        }
        referenceNormal = BoxShape::faceNormal(transformA, referenceFace);
        const u8* indices = BoxShape::faceCorners(referenceFace);
        referenceOffset = glm::dot(referenceNormal, boxCorners[indices[0]]);
        for (u32 i = 0; i < 4; ++i)
        {
            edgeStart[i] = boxCorners[indices[i]];
            edgeEnd[i] = boxCorners[indices[(i + 1) % 4]];
            referenceInterior += edgeStart[i] * 0.25f;
        }
        edgeCount = 4;

        polygon[count++] = v0;
        polygon[count++] = v1;
        polygon[count++] = v2;
    }
    else
    {
        // Triangle is the reference. Its outward normal has to face the box,
        // which is the opposite of the A-to-B normal.
        referenceNormal = -out.normal;
        referenceOffset = glm::dot(referenceNormal, v0);
        referenceInterior = (v0 + v1 + v2) / 3.0f;
        const glm::vec3 vertices[3] = {v0, v1, v2};
        for (u32 i = 0; i < 3; ++i)
        {
            edgeStart[i] = vertices[i];
            edgeEnd[i] = vertices[(i + 1) % 3];
        }
        edgeCount = 3;

        const u32 face = incidentFace(a, transformA, referenceNormal);
        const u8* indices = BoxShape::faceCorners(face);
        for (u32 i = 0; i < 4; ++i)
            polygon[count++] = boxCorners[indices[i]];
    }

    for (u32 i = 0; i < edgeCount && count > 0; ++i)
    {
        const glm::vec3 edge = edgeEnd[i] - edgeStart[i];
        glm::vec3 planeNormal = glm::cross(referenceNormal, edge);
        const f32 planeLength = glm::length(planeNormal);
        if (planeLength < kEpsilon)
            continue;
        glm::vec3 unit = planeNormal / planeLength;
        f32 offset = glm::dot(unit, edgeStart[i]);
        if (glm::dot(unit, referenceInterior) > offset)
        {
            unit = -unit;
            offset = -offset;
        }
        count = clipPolygon(polygon, count, unit, offset, scratch, capacity);
        for (u32 p = 0; p < count; ++p)
            polygon[p] = scratch[p];
    }

    glm::vec3 kept[capacity];
    f32 depths[capacity];
    u32 keptCount = 0;
    for (u32 i = 0; i < count; ++i)
    {
        const f32 depth = referenceOffset - glm::dot(referenceNormal, polygon[i]);
        if (depth < -margin)
            continue;
        kept[keptCount] = polygon[i] + referenceNormal * depth;
        depths[keptCount] = depth;
        ++keptCount;
    }
    if (keptCount == 0)
        return false;

    out.count = reducePoints(kept, depths, keptCount, out);
    return out.count > 0;
}

bool Narrowphase::capsuleTriangle(const CapsuleShape& a, const glm::mat4& transformA,
                                  const TriangleShape& b, const glm::mat4& transformB,
                                  ContactManifold& out, f32 margin)
{
    glm::vec3 lower, upper;
    a.segment(transformA, lower, upper);
    const glm::vec3 v0 = glm::vec3(transformB * glm::vec4(b.vertex(0), 1.0f));
    const glm::vec3 v1 = glm::vec3(transformB * glm::vec4(b.vertex(1), 1.0f));
    const glm::vec3 v2 = glm::vec3(transformB * glm::vec4(b.vertex(2), 1.0f));

    // Closest point on the triangle to each end and to the segment against
    // each edge; the nearest of those is the contact. Cheaper and steadier
    // than SAT here, because a capsule has no faces to separate on.
    TriangleFeature bestFeature = TriangleFeature::Face;
    glm::vec3 bestOnSegment = lower;
    glm::vec3 bestOnTriangle = closestPointOnTriangle(v0, v1, v2, lower, &bestFeature);
    f32 bestSquared = glm::dot(bestOnTriangle - lower, bestOnTriangle - lower);

    TriangleFeature upperFeature = TriangleFeature::Face;
    const glm::vec3 upperClosest = closestPointOnTriangle(v0, v1, v2, upper, &upperFeature);
    const f32 upperSquared = glm::dot(upperClosest - upper, upperClosest - upper);
    if (upperSquared < bestSquared)
    {
        bestSquared = upperSquared;
        bestOnSegment = upper;
        bestOnTriangle = upperClosest;
        bestFeature = upperFeature;
    }

    const glm::vec3 edges[3][2] = {{v0, v1}, {v1, v2}, {v2, v0}};
    static constexpr TriangleFeature edgeFeature[3] = {
        TriangleFeature::Edge0, TriangleFeature::Edge1, TriangleFeature::Edge2};
    for (u32 i = 0; i < 3; ++i)
    {
        glm::vec3 onSegment, onEdge;
        closestPointsBetweenSegments(lower, upper, edges[i][0], edges[i][1], onSegment, onEdge);
        const f32 squared = glm::dot(onEdge - onSegment, onEdge - onSegment);
        if (squared < bestSquared)
        {
            bestSquared = squared;
            bestOnSegment = onSegment;
            bestOnTriangle = onEdge;
            bestFeature = edgeFeature[i];
        }
    }

    const f32 reach = a.radius() + margin;
    if (bestSquared > reach * reach)
        return false;

    const glm::vec3 offset = bestOnTriangle - bestOnSegment;
    glm::vec3 normal;
    if (!triangleContactNormal(b, transformB, bestFeature, offset, bestSquared, normal))
        return false;

    out.normal = normal;
    out.buildTangents();
    out.count = 1;
    out.points[0].position = bestOnTriangle;
    out.points[0].penetration = a.radius() - std::sqrt(bestSquared);
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
    return true;
}

bool Narrowphase::convexTrimesh(const CollisionShape& convex, const glm::mat4& convexTransform,
                                const TrimeshShape& mesh, const glm::mat4& meshTransform,
                                std::vector<ContactManifold>& out, f32 margin)
{
    // The convex's world bounds, brought into the mesh's own space: the tree
    // was built there and moving one box in is cheaper than moving every
    // triangle out.
    AABB worldBox = convex.bounds(convexTransform);
    worldBox.min -= glm::vec3(margin);
    worldBox.max += glm::vec3(margin);

    // A body transform is rotation and translation only - RigidBody builds it
    // from a normalized quaternion and never writes a scale - so the inverse
    // is the transpose, not a general 4x4 inverse. This runs per pair per
    // substep against a terrain that never moves.
    const glm::mat3 rotation(meshTransform);
    const glm::mat3 inverseRotation = glm::transpose(rotation);
    glm::mat4 inverseTransform(inverseRotation);
    inverseTransform[3] = glm::vec4(-(inverseRotation * glm::vec3(meshTransform[3])), 1.0f);
    const AABB localBox = transformAABB(worldBox, inverseTransform);

    // Reused rather than allocated per call, the same way the reference keeps
    // its render queues (wiRenderer's `static thread_local RenderQueue`).
    static thread_local std::vector<u32> candidates;
    mesh.query(localBox, candidates);
    if (candidates.empty())
        return false;

    const usize before = out.size();
    for (u32 index : candidates)
    {
        const TriangleShape triangle = mesh.triangle(index);
        ContactManifold manifold;
        bool hit = false;
        switch (convex.type())
        {
        case ShapeType::Sphere:
            hit = sphereTriangle(static_cast<const SphereShape&>(convex), convexTransform, triangle,
                                 meshTransform, manifold, margin);
            break;
        case ShapeType::Box:
            hit = boxTriangle(static_cast<const BoxShape&>(convex), convexTransform, triangle,
                              meshTransform, manifold, margin);
            break;
        case ShapeType::Capsule:
            hit = capsuleTriangle(static_cast<const CapsuleShape&>(convex), convexTransform,
                                  triangle, meshTransform, manifold, margin);
            break;
        default:
            break;
        }
        if (hit)
            out.push_back(manifold);
    }
    return out.size() > before;
}

namespace
{

bool raycastPlane(const PlaneShape& shape, const glm::mat4& transform, const Ray& ray,
                  f32 maxDistance, ShapeRayHit& hit)
{
    const glm::vec3 normal = glm::normalize(glm::mat3(transform) * shape.normal());
    const glm::vec3 origin =
        glm::vec3(transform * glm::vec4(shape.normal() * shape.constant(), 1.0f));
    Plane plane;
    plane.normal = normal;
    plane.d = -glm::dot(normal, origin);
    f32 distance = 0.0f;
    if (!ray.intersects(plane, distance) || distance > maxDistance)
        return false;
    hit.distance = distance;
    hit.point = ray.at(distance);
    hit.normal = glm::dot(ray.direction, normal) < 0.0f ? normal : -normal;
    return true;
}

bool raycastSphere(const SphereShape& sphere, const glm::mat4& transform, const Ray& ray,
                   f32 maxDistance, ShapeRayHit& hit)
{
    Sphere world;
    world.center = glm::vec3(transform[3]);
    world.radius = sphere.radius();

    f32 t = 0.0f;
    if (!ray.intersects(world, t) || t > maxDistance)
        return false;

    hit.distance = t;
    hit.point = ray.at(t);
    const glm::vec3 offset = hit.point - world.center;
    const f32 length = glm::length(offset);
    hit.normal = length > kEpsilon ? offset / length : glm::vec3(0.0f, 1.0f, 0.0f);
    return true;
}

bool raycastBox(const BoxShape& box, const glm::mat4& transform, const Ray& ray, f32 maxDistance,
                ShapeRayHit& hit)
{
    const glm::mat3 rotation(transform);
    const glm::mat3 inverseRotation = glm::transpose(rotation);
    const glm::vec3 center(transform[3]);

    Ray localRay;
    localRay.origin = inverseRotation * (ray.origin - center);
    localRay.direction = inverseRotation * ray.direction;

    const glm::vec3& half = box.halfExtents();
    AABB local;
    local.min = -half;
    local.max = half;

    f32 t = 0.0f;
    if (!localRay.intersects(local, t) || t < 0.0f || t > maxDistance)
        return false;

    const glm::vec3 localPoint = localRay.at(t);
    glm::vec3 localNormal(0.0f);
    f32 bestGap = std::numeric_limits<f32>::max();
    for (u32 axis = 0; axis < 3; ++axis)
    {
        const f32 gap = half[axis] - std::abs(localPoint[axis]);
        if (gap < bestGap)
        {
            bestGap = gap;
            localNormal = glm::vec3(0.0f);
            localNormal[axis] = localPoint[axis] >= 0.0f ? 1.0f : -1.0f;
        }
    }

    hit.distance = t;
    hit.point = center + rotation * localPoint;
    hit.normal = rotation * localNormal;
    return true;
}

bool raycastCapsule(const CapsuleShape& capsule, const glm::mat4& transform, const Ray& ray,
                    f32 maxDistance, ShapeRayHit& hit)
{
    const glm::mat3 rotation(transform);
    const glm::mat3 inverseRotation = glm::transpose(rotation);
    const glm::vec3 center(transform[3]);

    Ray localRay;
    localRay.origin = inverseRotation * (ray.origin - center);
    localRay.direction = inverseRotation * ray.direction;

    const f32 radius = capsule.radius();
    const f32 halfHeight = capsule.halfHeight();
    const glm::vec3& o = localRay.origin;
    const glm::vec3& d = localRay.direction;

    bool found = false;
    f32 nearest = maxDistance;
    glm::vec3 localPoint(0.0f);
    glm::vec3 localNormal(0.0f, 1.0f, 0.0f);

    const f32 a = d.x * d.x + d.z * d.z;
    if (a > kEpsilon)
    {
        const f32 b = 2.0f * (o.x * d.x + o.z * d.z);
        const f32 c = o.x * o.x + o.z * o.z - radius * radius;
        const f32 discriminant = b * b - 4.0f * a * c;
        if (discriminant >= 0.0f)
        {
            const f32 root = std::sqrt(discriminant);
            const f32 roots[2] = {(-b - root) / (2.0f * a), (-b + root) / (2.0f * a)};
            for (f32 t : roots)
            {
                if (t < 0.0f || t >= nearest)
                    continue;
                const f32 y = o.y + d.y * t;
                if (y < -halfHeight || y > halfHeight)
                    continue;
                nearest = t;
                localPoint = o + d * t;
                localNormal = glm::normalize(glm::vec3(localPoint.x, 0.0f, localPoint.z));
                found = true;
            }
        }
    }

    Sphere cap;
    cap.radius = radius;
    cap.center = glm::vec3(0.0f, -halfHeight, 0.0f);
    f32 t = 0.0f;
    if (localRay.intersects(cap, t) && t < nearest)
    {
        nearest = t;
        localPoint = localRay.at(t);
        localNormal = glm::normalize(localPoint - cap.center);
        found = true;
    }
    cap.center = glm::vec3(0.0f, halfHeight, 0.0f);
    if (localRay.intersects(cap, t) && t < nearest)
    {
        nearest = t;
        localPoint = localRay.at(t);
        localNormal = glm::normalize(localPoint - cap.center);
        found = true;
    }

    if (!found)
        return false;

    hit.distance = nearest;
    hit.point = center + rotation * localPoint;
    hit.normal = rotation * localNormal;
    return true;
}

bool raycastTriangle(const TriangleShape& triangle, const glm::mat4& transform, const Ray& ray,
                     f32 maxDistance, ShapeRayHit& hit)
{
    const glm::vec3 v0 = glm::vec3(transform * glm::vec4(triangle.vertex(0), 1.0f));
    const glm::vec3 v1 = glm::vec3(transform * glm::vec4(triangle.vertex(1), 1.0f));
    const glm::vec3 v2 = glm::vec3(transform * glm::vec4(triangle.vertex(2), 1.0f));

    f32 t = 0.0f;
    if (!ray.intersects(v0, v1, v2, t) || t > maxDistance)
        return false;

    hit.distance = t;
    hit.point = ray.at(t);
    const glm::vec3 raw = glm::cross(v1 - v0, v2 - v0);
    const f32 length = glm::length(raw);
    hit.normal = length > kEpsilon ? raw / length : glm::vec3(0.0f, 1.0f, 0.0f);
    if (glm::dot(hit.normal, ray.direction) > 0.0f)
        hit.normal = -hit.normal;
    return true;
}

bool raycastTrimesh(const TrimeshShape& mesh, const glm::mat4& transform, const Ray& ray,
                    f32 maxDistance, ShapeRayHit& hit)
{
    const glm::mat3 rotation(transform);
    const glm::mat3 inverseRotation = glm::transpose(rotation);
    const glm::vec3 center(transform[3]);

    Ray localRay;
    localRay.origin = inverseRotation * (ray.origin - center);
    localRay.direction = inverseRotation * ray.direction;

    TrimeshShape::RayHit localHit;
    if (!mesh.raycast(localRay, maxDistance, localHit))
        return false;

    hit.distance = localHit.distance;
    hit.point = center + rotation * localHit.point;
    hit.normal = rotation * localHit.normal;
    return true;
}

bool overlapSphereShape(const SphereShape& sphere, const glm::mat4& transform,
                        const glm::vec3& centre, f32 radius)
{
    const glm::vec3 center(transform[3]);
    const f32 total = sphere.radius() + radius;
    return glm::dot(centre - center, centre - center) <= total * total;
}

bool overlapSphereShape(const BoxShape& box, const glm::mat4& transform, const glm::vec3& centre,
                        f32 radius)
{
    const glm::mat3 rotation(transform);
    const glm::vec3 boxCenter(transform[3]);
    const glm::vec3 local = glm::transpose(rotation) * (centre - boxCenter);
    const glm::vec3 closest = glm::clamp(local, -box.halfExtents(), box.halfExtents());
    const glm::vec3 offset = local - closest;
    return glm::dot(offset, offset) <= radius * radius;
}

bool overlapSphereShape(const CapsuleShape& capsule, const glm::mat4& transform,
                        const glm::vec3& centre, f32 radius)
{
    glm::vec3 lower, upper;
    capsule.segment(transform, lower, upper);
    const glm::vec3 closest = closestPointOnSegment(lower, upper, centre);
    const f32 total = capsule.radius() + radius;
    return glm::dot(centre - closest, centre - closest) <= total * total;
}

bool overlapSphereShape(const TriangleShape& triangle, const glm::mat4& transform,
                        const glm::vec3& centre, f32 radius)
{
    const glm::vec3 v0 = glm::vec3(transform * glm::vec4(triangle.vertex(0), 1.0f));
    const glm::vec3 v1 = glm::vec3(transform * glm::vec4(triangle.vertex(1), 1.0f));
    const glm::vec3 v2 = glm::vec3(transform * glm::vec4(triangle.vertex(2), 1.0f));
    const glm::vec3 closest = closestPointOnTriangle(v0, v1, v2, centre);
    const glm::vec3 offset = closest - centre;
    return glm::dot(offset, offset) <= radius * radius;
}

bool overlapSphereShape(const TrimeshShape& mesh, const glm::mat4& transform,
                        const glm::vec3& centre, f32 radius)
{
    const glm::mat3 rotation(transform);
    const glm::vec3 center(transform[3]);
    const glm::vec3 localCentre = glm::transpose(rotation) * (centre - center);

    static thread_local std::vector<u32> triangles;
    mesh.overlapSphere(localCentre, radius, triangles);
    return !triangles.empty();
}

bool overlapSphereShape(const PlaneShape& plane, const glm::mat4& transform,
                        const glm::vec3& centre, f32 radius)
{
    const glm::vec3 normal = glm::normalize(glm::mat3(transform) * plane.normal());
    const glm::vec3 origin =
        glm::vec3(transform * glm::vec4(plane.normal() * plane.constant(), 1.0f));
    return glm::dot(normal, centre - origin) <= radius;
}

} // namespace

bool Narrowphase::raycast(const CollisionShape& shape, const glm::mat4& transform, const Ray& ray,
                          f32 maxDistance, ShapeRayHit& hit)
{
    switch (shape.type())
    {
    case ShapeType::Sphere:
        return raycastSphere(static_cast<const SphereShape&>(shape), transform, ray, maxDistance,
                             hit);
    case ShapeType::Box:
        return raycastBox(static_cast<const BoxShape&>(shape), transform, ray, maxDistance, hit);
    case ShapeType::Capsule:
        return raycastCapsule(static_cast<const CapsuleShape&>(shape), transform, ray, maxDistance,
                              hit);
    case ShapeType::Plane:
        return raycastPlane(static_cast<const PlaneShape&>(shape), transform, ray, maxDistance, hit);
    case ShapeType::Triangle:
        return raycastTriangle(static_cast<const TriangleShape&>(shape), transform, ray,
                               maxDistance, hit);
    case ShapeType::Trimesh:
        return raycastTrimesh(static_cast<const TrimeshShape&>(shape), transform, ray, maxDistance,
                              hit);
    default:
        return false;
    }
}

bool Narrowphase::overlapSphere(const CollisionShape& shape, const glm::mat4& transform,
                                const glm::vec3& centre, f32 radius)
{
    switch (shape.type())
    {
    case ShapeType::Sphere:
        return overlapSphereShape(static_cast<const SphereShape&>(shape), transform, centre,
                                  radius);
    case ShapeType::Box:
        return overlapSphereShape(static_cast<const BoxShape&>(shape), transform, centre, radius);
    case ShapeType::Capsule:
        return overlapSphereShape(static_cast<const CapsuleShape&>(shape), transform, centre,
                                  radius);
    case ShapeType::Plane:
        return overlapSphereShape(static_cast<const PlaneShape&>(shape), transform, centre, radius);
    case ShapeType::Triangle:
        return overlapSphereShape(static_cast<const TriangleShape&>(shape), transform, centre,
                                  radius);
    case ShapeType::Trimesh:
        return overlapSphereShape(static_cast<const TrimeshShape&>(shape), transform, centre,
                                  radius);
    default:
        return false;
    }
}

} // namespace Radion::Physics
