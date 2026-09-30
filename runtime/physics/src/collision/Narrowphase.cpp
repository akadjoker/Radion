#include "PCH.h"

#include "collision/Narrowphase.h"

namespace Radion::Physics
{

namespace
{
constexpr f32 kEpsilon = 1e-6f;

// Overlap of the two shadows on `axis` and which way it must point to push B off A; negative means a gap, ending the test.
bool axisOverlap(const CollisionShape& a, const Math::mat4& transformA, const CollisionShape& b,
                 const Math::mat4& transformB, const Math::vec3& axis, f32 margin,
                 f32& penetration, Math::vec3& normal)
{
    const f32 length = Math::length(axis);
    if (length < kEpsilon)
        return true; // degenerate axis carries no information; not a separation

    const Math::vec3 unit = axis / length;
    f32 minA, maxA, minB, maxB;
    a.project(transformA, unit, minA, maxA);
    b.project(transformB, unit, minB, maxB);

    // overlapA is the cost of moving B along -unit, overlapB along +unit; negative past the margin on any axis is a gap.
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

u32 clipPolygon(const Math::vec3* input, u32 count, const Math::vec3& planeNormal, f32 planeOffset,
                Math::vec3* output, u32 capacity)
{
    u32 written = 0;
    for (u32 i = 0; i < count && written + 1 < capacity; ++i)
    {
        const Math::vec3& current = input[i];
        const Math::vec3& next = input[(i + 1) % count];
        const f32 distanceCurrent = Math::dot(planeNormal, current) - planeOffset;
        const f32 distanceNext = Math::dot(planeNormal, next) - planeOffset;

        if (distanceCurrent <= 0.0f)
            output[written++] = current;
        if (written < capacity && distanceCurrent * distanceNext < 0.0f)
        {
            const f32 t = distanceCurrent / (distanceCurrent - distanceNext);
            output[written++] = current + (next - current) * t;
        }
    }
    return written;
}

// Face of `box` most opposed to `normal`: the one being pressed into the other shape.
u32 incidentFace(const BoxShape& box, const Math::mat4& transform, const Math::vec3& normal)
{
    u32 best = 0;
    f32 bestDot = 1.0e30f;
    for (u32 face = 0; face < 6; ++face)
    {
        const f32 value = Math::dot(BoxShape::faceNormal(transform, face), normal);
        if (value < bestDot)
        {
            bestDot = value;
            best = face;
        }
    }
    return best;
}

u32 reducePoints(const Math::vec3* points, const f32* depths, u32 count, ContactManifold& out);

// Generous for a shatter shard (a handful of faces, tens of vertices).
constexpr u32 kHullArrayCapacity = 32;

// Outward face normal from its first triangle (cross of the first two edges of the face loop), like VoronoiShatter's planes.
Math::vec3 hullFaceNormal(const ConvexHullShape& hull, const Math::mat4& transform, u32 face)
{
    const ConvexHullShape::Edge* edge = &hull.edges()[static_cast<usize>(hull.faces()[face])];
    const int v0 = edge->getSourceVertex();
    const int v1 = edge->getTargetVertex();
    edge = edge->getNextEdgeOfFace();
    const int v2 = edge->getTargetVertex();
    const Math::vec3& p0 = hull.vertices()[static_cast<usize>(v0)];
    const Math::vec3& p1 = hull.vertices()[static_cast<usize>(v1)];
    const Math::vec3& p2 = hull.vertices()[static_cast<usize>(v2)];
    const Math::vec3 worldRaw = Math::mat3(transform) * Math::cross(p1 - p0, p2 - p0);
    const f32 length = Math::length(worldRaw);
    return length > kEpsilon ? worldRaw / length : Math::vec3(0.0f, 1.0f, 0.0f);
}

// Same most-opposed rule as incidentFace(), for any number of hull faces.
u32 hullIncidentFace(const ConvexHullShape& hull, const Math::mat4& transform,
                     const Math::vec3& normal)
{
    u32 best = 0;
    f32 bestDot = 1.0e30f;
    const u32 faceCount = hull.faceCount();
    for (u32 face = 0; face < faceCount; ++face)
    {
        const f32 value = Math::dot(hullFaceNormal(hull, transform, face), normal);
        if (value < bestDot)
        {
            bestDot = value;
            best = face;
        }
    }
    return best;
}

// Walks the face's half-edge loop once, writing each vertex's world position: the polygon clipPolygon() clips against.
u32 hullFacePolygon(const ConvexHullShape& hull, const Math::mat4& transform, u32 face,
                    Math::vec3* out, u32 capacity)
{
    const ConvexHullShape::Edge* start = &hull.edges()[static_cast<usize>(hull.faces()[face])];
    const ConvexHullShape::Edge* edge = start;
    u32 count = 0;
    do
    {
        if (count >= capacity)
            break;
        out[count++] =
            Math::vec3(transform * Math::vec4(hull.vertices()[static_cast<usize>(
                                                edge->getTargetVertex())],
                                            1.0f));
        edge = edge->getNextEdgeOfFace();
    } while (edge != start);

    // ConvexHullComputer's face loop winds OPPOSITE to hullFaceNormal()'s cross(v1-v0, v2-v0) (verified against a known interior point);
    // reversed, the side planes agree with BoxShape's kFaces winding and clipFaceAgainstFace() needs no shape-specific case.
    for (u32 i = 0, j = count > 0 ? count - 1 : 0; i < j; ++i, --j)
    {
        const Math::vec3 temporary = out[i];
        out[i] = out[j];
        out[j] = temporary;
    }
    return count;
}

// One direction per hull edge, world space, unnormalized, each edge counted once (smaller half-edge index); stands in for a box's edge directions in SAT cross axes.
u32 hullEdgeDirections(const ConvexHullShape& hull, const Math::mat4& transform, Math::vec3* out,
                       u32 capacity)
{
    const std::vector<ConvexHullShape::Edge>& edges = hull.edges();
    const std::vector<Math::vec3>& vertices = hull.vertices();
    const Math::mat3 rotation(transform);
    u32 count = 0;
    for (usize i = 0; i < edges.size() && count < capacity; ++i)
    {
        const ConvexHullShape::Edge& edge = edges[i];
        const usize reverseIndex = static_cast<usize>(edge.getReverseEdge() - edges.data());
        if (reverseIndex <= i)
            continue;
        const Math::vec3 direction =
            vertices[static_cast<usize>(edge.getTargetVertex())] -
            vertices[static_cast<usize>(edge.getSourceVertex())];
        out[count++] = rotation * direction;
    }
    return count;
}

// Single contact at the midpoint of the support points along `normal`: the fallback for edge-edge separation or a grazing face pair clipped to nothing.
void supportPointContact(const CollisionShape& a, const Math::mat4& transformA,
                         const CollisionShape& b, const Math::mat4& transformB, f32 penetration,
                         ContactManifold& out)
{
    out.count = 1;
    const Math::vec3 pointA = a.support(transformA, out.normal);
    const Math::vec3 pointB = b.support(transformB, -out.normal);
    out.points[0].position = (pointA + pointB) * 0.5f;
    out.points[0].penetration = penetration;
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
}

// Clips the incident face polygon against the reference face's side planes, keeping what is behind the reference plane (boxBox()'s face case, any polygon size).
bool clipFaceAgainstFace(const Math::vec3* referencePolygon, u32 referenceCount,
                         const Math::vec3& referenceNormal, const Math::vec3* incidentPolygon,
                         u32 incidentCount, f32 margin, ContactManifold& out)
{
    if (referenceCount < 3 || incidentCount < 3)
        return false;

    Math::vec3 polygon[kHullArrayCapacity];
    Math::vec3 scratch[kHullArrayCapacity];
    u32 count = Math::min(incidentCount, kHullArrayCapacity);
    for (u32 i = 0; i < count; ++i)
        polygon[i] = incidentPolygon[i];

    const f32 referenceOffset = Math::dot(referenceNormal, referencePolygon[0]);
    for (u32 i = 0; i < referenceCount && count > 0; ++i)
    {
        const Math::vec3& edgeStart = referencePolygon[i];
        const Math::vec3& edgeEnd = referencePolygon[(i + 1) % referenceCount];
        const Math::vec3 edge = edgeEnd - edgeStart;
        // cross(normal, edge), not cross(edge, normal): the other order points the side plane into the face. Box kFaces and hull faces both wind outward-CCW.
        const Math::vec3 planeNormal = Math::cross(referenceNormal, edge);
        const f32 planeLength = Math::length(planeNormal);
        if (planeLength < kEpsilon)
            continue;
        const Math::vec3 unit = planeNormal / planeLength;
        count = clipPolygon(polygon, count, unit, Math::dot(unit, edgeStart), scratch,
                            kHullArrayCapacity);
        for (u32 p = 0; p < count; ++p)
            polygon[p] = scratch[p];
    }

    Math::vec3 kept[kHullArrayCapacity];
    f32 depths[kHullArrayCapacity];
    u32 keptCount = 0;
    for (u32 i = 0; i < count; ++i)
    {
        const f32 depth = referenceOffset - Math::dot(referenceNormal, polygon[i]);
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

// Keeps the deepest point, then the three furthest from it and each other, so the patch spans the contact area; four clustered points act as one and a box wobbles.
u32 reducePoints(const Math::vec3* points, const f32* depths, u32 count, ContactManifold& out)
{
    if (count == 0)
        return 0;
    u32 deepest = 0;
    for (u32 i = 1; i < count; ++i)
        if (depths[i] > depths[deepest])
            deepest = i;

    if (count <= ContactManifold::MaxPoints)
    {
        // Deepest point goes first even when nothing is dropped, so points[0] really is the deepest (the clipper's first corner is not).
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
                nearest = Math::min(nearest, Math::length(points[i] - points[chosen[c]]));
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
    // Any vector not parallel to the normal works; the world axis it leans on least keeps the cross product away from zero.
    const Math::vec3 reference = std::abs(normal.x) < 0.57735f ? Math::vec3(1.0f, 0.0f, 0.0f)
                                                              : Math::vec3(0.0f, 1.0f, 0.0f);
    tangent[0] = Math::normalize(Math::cross(reference, normal));
    tangent[1] = Math::cross(normal, tangent[0]);
}

bool Narrowphase::sphereSphere(const SphereShape& a, const Math::mat4& transformA,
                               const SphereShape& b, const Math::mat4& transformB,
                               ContactManifold& out, f32 margin)
{
    const Math::vec3 centerA(transformA[3]);
    const Math::vec3 centerB(transformB[3]);
    const Math::vec3 delta = centerB - centerA;
    const f32 distance = Math::length(delta);
    const f32 total = a.radius() + b.radius();
    if (distance >= total + margin)
        return false;

    // Concentric spheres have no separation direction; pick any rather than divide by zero.
    out.normal = distance > kEpsilon ? delta / distance : Math::vec3(0.0f, 1.0f, 0.0f);
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

bool Narrowphase::sphereBox(const SphereShape& a, const Math::mat4& transformA, const BoxShape& b,
                            const Math::mat4& transformB, ContactManifold& out, f32 margin)
{
    const Math::vec3 center(transformA[3]);
    const Math::mat3 rotation(transformB);
    const Math::vec3 boxCenter(transformB[3]);
    const Math::vec3 local = Math::transpose(rotation) * (center - boxCenter);
    const Math::vec3& half = b.halfExtents();

    const Math::vec3 closestLocal = Math::clamp(local, -half, half);
    const Math::vec3 offset = local - closestLocal;
    const f32 distanceSquared = Math::dot(offset, offset);
    const f32 reach = a.radius() + margin;
    if (distanceSquared > reach * reach)
        return false;

    Math::vec3 normalLocal;
    f32 penetration = 0.0f;
    if (distanceSquared > kEpsilon * kEpsilon)
    {
        const f32 distance = std::sqrt(distanceSquared);
        normalLocal = offset / distance;
        penetration = a.radius() - distance;
    }
    else
    {
        // Centre inside the box: exit through the face it is least far from.
        const Math::vec3 depth = half - Math::abs(local);
        if (depth.x <= depth.y && depth.x <= depth.z)
        {
            normalLocal = Math::vec3(local.x >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
            penetration = a.radius() + depth.x;
        }
        else if (depth.y <= depth.z)
        {
            normalLocal = Math::vec3(0.0f, local.y >= 0.0f ? 1.0f : -1.0f, 0.0f);
            penetration = a.radius() + depth.y;
        }
        else
        {
            normalLocal = Math::vec3(0.0f, 0.0f, local.z >= 0.0f ? 1.0f : -1.0f);
            penetration = a.radius() + depth.z;
        }
    }

    // The manifold's normal always points from A to B, and A is the sphere.
    out.normal = -Math::normalize(rotation * normalLocal);
    out.buildTangents();
    out.count = 1;
    out.points[0].penetration = penetration;
    out.points[0].position = boxCenter + rotation * closestLocal;
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
    return true;
}

bool Narrowphase::boxBox(const BoxShape& a, const Math::mat4& transformA, const BoxShape& b,
                         const Math::mat4& transformB, ContactManifold& out, f32 margin)
{
    // Fifteen axes: six face normals and nine edge cross products; without the nine, edge-to-edge contact gets a face normal and slides along it.
    const Math::mat3 rotationA(transformA);
    const Math::mat3 rotationB(transformB);

    Math::vec3 axes[15];
    u32 axisCount = 0;
    for (u32 i = 0; i < 3; ++i)
        axes[axisCount++] = Math::normalize(Math::vec3(rotationA[i]));
    for (u32 i = 0; i < 3; ++i)
        axes[axisCount++] = Math::normalize(Math::vec3(rotationB[i]));
    for (u32 i = 0; i < 3; ++i)
        for (u32 j = 0; j < 3; ++j)
            axes[axisCount++] = Math::cross(Math::vec3(rotationA[i]), Math::vec3(rotationB[j]));

    f32 bestPenetration = 1.0e30f;
    Math::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    u32 bestAxis = 0;
    for (u32 i = 0; i < axisCount; ++i)
    {
        f32 penetration = 0.0f;
        Math::vec3 normal(0.0f);
        if (!axisOverlap(a, transformA, b, transformB, axes[i], margin, penetration, normal))
            return false;
        // A parallel edge pair gives a zero cross product, which axisOverlap() reports as no information; it must not win with a stale zero.
        if (Math::length(axes[i]) < kEpsilon)
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

    // Edge-edge separation has no face to clip: the contact is the closest point of the two edges, found via each side's support point.
    if (bestAxis >= 6)
    {
        out.count = 1;
        const Math::vec3 pointA = a.support(transformA, out.normal);
        const Math::vec3 pointB = b.support(transformB, -out.normal);
        out.points[0].position = (pointA + pointB) * 0.5f;
        out.points[0].penetration = bestPenetration;
        out.points[0].normalImpulse = 0.0f;
        out.points[0].tangentImpulse[0] = 0.0f;
        out.points[0].tangentImpulse[1] = 0.0f;
        return true;
    }

    const bool referenceIsA = bestAxis < 3;
    const BoxShape& reference = referenceIsA ? a : b;
    const BoxShape& incident = referenceIsA ? b : a;
    const Math::mat4& referenceTransform = referenceIsA ? transformA : transformB;
    const Math::mat4& incidentTransform = referenceIsA ? transformB : transformA;
    const Math::vec3 referenceDirection = referenceIsA ? out.normal : -out.normal;

    u32 referenceFace = 0;
    f32 bestDot = -1.0e30f;
    for (u32 face = 0; face < 6; ++face)
    {
        const f32 value = Math::dot(BoxShape::faceNormal(referenceTransform, face),
                                   referenceDirection);
        if (value > bestDot)
        {
            bestDot = value;
            referenceFace = face;
        }
    }

    const Math::vec3 referenceNormal = BoxShape::faceNormal(referenceTransform, referenceFace);
    Math::vec3 referenceCorner[8];
    reference.corners(referenceTransform, referenceCorner);
    const u8* referenceIndices = BoxShape::faceCorners(referenceFace);
    const f32 referenceOffset = Math::dot(referenceNormal, referenceCorner[referenceIndices[0]]);

    const u32 incidentFaceIndex = incidentFace(incident, incidentTransform, referenceNormal);
    Math::vec3 incidentCorner[8];
    incident.corners(incidentTransform, incidentCorner);
    const u8* incidentIndices = BoxShape::faceCorners(incidentFaceIndex);

    constexpr u32 capacity = 16;
    Math::vec3 polygon[capacity];
    Math::vec3 scratch[capacity];
    u32 count = 4;
    for (u32 i = 0; i < 4; ++i)
        polygon[i] = incidentCorner[incidentIndices[i]];

    for (u32 i = 0; i < 4 && count > 0; ++i)
    {
        const Math::vec3& edgeStart = referenceCorner[referenceIndices[i]];
        const Math::vec3& edgeEnd = referenceCorner[referenceIndices[(i + 1) % 4]];
        const Math::vec3 edge = edgeEnd - edgeStart;
        // cross(normal, edge), not cross(edge, normal): with kFaces winding the latter points INTO the face, clipping discards the whole incident
        // polygon, and a face-face contact degrades to one support point (a balanced box tips).
        const Math::vec3 planeNormal = Math::cross(referenceNormal, edge);
        const f32 planeLength = Math::length(planeNormal);
        if (planeLength < kEpsilon)
            continue;
        const Math::vec3 unit = planeNormal / planeLength;
        count = clipPolygon(polygon, count, unit, Math::dot(unit, edgeStart), scratch, capacity);
        for (u32 p = 0; p < count; ++p)
            polygon[p] = scratch[p];
    }

    Math::vec3 kept[capacity];
    f32 depths[capacity];
    u32 keptCount = 0;
    for (u32 i = 0; i < count; ++i)
    {
        const f32 depth = referenceOffset - Math::dot(referenceNormal, polygon[i]);
        // Negative depth is in front of the reference face; within the margin it is a speculative contact, past it dropped.
        if (depth < -margin)
            continue;
        // Reported on the reference face, the surface the solver pushes along, not where the incident corner sits.
        kept[keptCount] = polygon[i] + referenceNormal * depth;
        depths[keptCount] = depth;
        ++keptCount;
    }

    if (keptCount == 0)
    {
        // Clipping produced nothing (grazing face pair) though the axis test proved overlap: fall back to support points rather than drop a contact.
        out.count = 1;
        const Math::vec3 pointA = a.support(transformA, out.normal);
        const Math::vec3 pointB = b.support(transformB, -out.normal);
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
// Two spheres at given centres written into the manifold; every capsule contact reduces to this once closest points are known.
bool sphereContact(const Math::vec3& centerA, f32 radiusA, const Math::vec3& centerB, f32 radiusB,
                   f32 margin, ContactManifold& out)
{
    const Math::vec3 delta = centerB - centerA;
    const f32 distance = Math::length(delta);
    const f32 total = radiusA + radiusB;
    if (distance >= total + margin)
        return false;

    out.normal = distance > kEpsilon ? delta / distance : Math::vec3(0.0f, 1.0f, 0.0f);
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

bool Narrowphase::capsuleSphere(const CapsuleShape& a, const Math::mat4& transformA,
                                const SphereShape& b, const Math::mat4& transformB,
                                ContactManifold& out, f32 margin)
{
    Math::vec3 lower, upper;
    a.segment(transformA, lower, upper);
    const Math::vec3 center(transformB[3]);
    const Math::vec3 closest = closestPointOnSegment(lower, upper, center);
    return sphereContact(closest, a.radius(), center, b.radius(), margin, out);
}

bool Narrowphase::capsuleCapsule(const CapsuleShape& a, const Math::mat4& transformA,
                                 const CapsuleShape& b, const Math::mat4& transformB,
                                 ContactManifold& out, f32 margin)
{
    Math::vec3 lowerA, upperA, lowerB, upperB;
    a.segment(transformA, lowerA, upperA);
    b.segment(transformB, lowerB, upperB);
    Math::vec3 closestA, closestB;
    closestPointsBetweenSegments(lowerA, upperA, lowerB, upperB, closestA, closestB);
    return sphereContact(closestA, a.radius(), closestB, b.radius(), margin, out);
}

bool Narrowphase::capsuleBox(const CapsuleShape& a, const Math::mat4& transformA, const BoxShape& b,
                             const Math::mat4& transformB, ContactManifold& out, f32 margin)
{
    Math::vec3 lower, upper;
    a.segment(transformA, lower, upper);
    const Math::mat3 rotationB(transformB);
    const Math::vec3 capsuleAxis = Math::normalize(Math::vec3(transformA[1]));

    // Box faces, capsule axis and their cross products; the last set catches a capsule lying diagonally across an edge.
    Math::vec3 axes[7];
    u32 axisCount = 0;
    for (u32 i = 0; i < 3; ++i)
        axes[axisCount++] = Math::normalize(Math::vec3(rotationB[i]));
    axes[axisCount++] = capsuleAxis;
    for (u32 i = 0; i < 3; ++i)
        axes[axisCount++] = Math::cross(capsuleAxis, Math::vec3(rotationB[i]));

    f32 bestPenetration = 1.0e30f;
    Math::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    for (u32 i = 0; i < axisCount; ++i)
    {
        f32 penetration = 0.0f;
        Math::vec3 normal(0.0f);
        if (!axisOverlap(a, transformA, b, transformB, axes[i], margin, penetration, normal))
            return false;
        if (Math::length(axes[i]) < kEpsilon)
            continue;
        if (penetration < bestPenetration)
        {
            bestPenetration = penetration;
            bestNormal = normal;
        }
    }

    out.normal = bestNormal;
    out.buildTangents();

    // Lying along a face: both ends touch and one point would let the capsule pivot, so clip the segment to the face.
    const f32 alignment = std::abs(Math::dot(capsuleAxis, out.normal));
    if (alignment < 0.05f)
    {
        // The box face TOWARDS the capsule: the normal runs capsule to box, so supporting along it lands on the far side.
        const Math::vec3 onBoxFace = b.support(transformB, -out.normal);
        const f32 faceOffset = Math::dot(out.normal, onBoxFace);

        Math::vec3 points[2] = {lower, upper};
        u32 written = 0;
        for (u32 i = 0; i < 2; ++i)
        {
            // Capsule surface is its segment pushed a radius ALONG the normal (towards the box).
            const Math::vec3 surface = points[i] + out.normal * a.radius();
            const f32 depth = Math::dot(out.normal, surface) - faceOffset;
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

    // Otherwise one point: closest point on the segment to the box, pushed out by the radius.
    const Math::vec3 boxCenter(transformB[3]);
    const Math::vec3 nearSegment = closestPointOnSegment(lower, upper, boxCenter);
    const Math::vec3 localNear = Math::transpose(rotationB) * (nearSegment - boxCenter);
    const Math::vec3 onBox =
        boxCenter + rotationB * Math::clamp(localNear, -b.halfExtents(), b.halfExtents());
    const Math::vec3 refined = closestPointOnSegment(lower, upper, onBox);

    out.count = 1;
    out.points[0].penetration = bestPenetration;
    // Midway between box surface and capsule surface (segment pushed a radius along the normal, towards the box).
    out.points[0].position = (onBox + (refined + out.normal * a.radius())) * 0.5f;
    out.points[0].normalImpulse = 0.0f;
    out.points[0].tangentImpulse[0] = 0.0f;
    out.points[0].tangentImpulse[1] = 0.0f;
    return true;
}

bool Narrowphase::convexHullSphere(const ConvexHullShape& a, const Math::mat4& transformA,
                                   const SphereShape& b, const Math::mat4& transformB,
                                   ContactManifold& out, f32 margin)
{
    // A sphere has no faces or edges of its own, so the hull's face normals are the only SAT axes needed.
    const u32 faceCount = a.faceCount();
    if (faceCount == 0)
        return false;

    f32 bestPenetration = 1.0e30f;
    Math::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    for (u32 face = 0; face < faceCount; ++face)
    {
        const Math::vec3 axis = hullFaceNormal(a, transformA, face);
        f32 penetration = 0.0f;
        Math::vec3 normal(0.0f);
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

bool Narrowphase::convexHullCapsule(const ConvexHullShape& a, const Math::mat4& transformA,
                                    const CapsuleShape& b, const Math::mat4& transformB,
                                    ContactManifold& out, f32 margin)
{
    Math::vec3 lower, upper;
    b.segment(transformB, lower, upper);
    const Math::vec3 capsuleAxis = Math::normalize(Math::vec3(transformB[1]));

    const u32 hullFaceCount = a.faceCount();
    Math::vec3 hullEdges[kHullArrayCapacity];
    const u32 hullEdgeCount = hullEdgeDirections(a, transformA, hullEdges, kHullArrayCapacity);

    // Hull face normals, capsule axis, and the axis crossed with every hull edge direction.
    Math::vec3 axes[kHullArrayCapacity * 2 + 1];
    u32 axisCount = 0;
    const u32 axisFaceCount = Math::min(hullFaceCount, kHullArrayCapacity);
    for (u32 face = 0; face < axisFaceCount; ++face)
        axes[axisCount++] = hullFaceNormal(a, transformA, face);
    axes[axisCount++] = capsuleAxis;
    for (u32 i = 0; i < hullEdgeCount; ++i)
        axes[axisCount++] = Math::cross(capsuleAxis, hullEdges[i]);

    f32 bestPenetration = 1.0e30f;
    Math::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    for (u32 i = 0; i < axisCount; ++i)
    {
        f32 penetration = 0.0f;
        Math::vec3 normal(0.0f);
        if (!axisOverlap(a, transformA, b, transformB, axes[i], margin, penetration, normal))
            return false;
        if (Math::length(axes[i]) < kEpsilon)
            continue;
        if (penetration < bestPenetration)
        {
            bestPenetration = penetration;
            bestNormal = normal;
        }
    }

    out.normal = bestNormal;
    out.buildTangents();

    const f32 alignment = std::abs(Math::dot(capsuleAxis, out.normal));
    if (alignment < 0.05f && hullFaceCount > 0)
    {
        // Hull face TOWARDS the capsule: out.normal points hull to capsule, so pick the face whose normal agrees with it most.
        u32 referenceFace = 0;
        f32 bestDot = -1.0e30f;
        for (u32 face = 0; face < hullFaceCount; ++face)
        {
            const f32 value = Math::dot(hullFaceNormal(a, transformA, face), out.normal);
            if (value > bestDot)
            {
                bestDot = value;
                referenceFace = face;
            }
        }
        Math::vec3 facePolygon[kHullArrayCapacity];
        const u32 faceVertexCount =
            hullFacePolygon(a, transformA, referenceFace, facePolygon, kHullArrayCapacity);
        if (faceVertexCount > 0)
        {
            const f32 faceOffset = Math::dot(out.normal, facePolygon[0]);
            const Math::vec3 points[2] = {lower, upper};
            ContactPoint generated[2];
            u32 written = 0;
            for (u32 i = 0; i < 2; ++i)
            {
                // Capsule surface pushed a radius towards the hull, i.e. along -out.normal (which points hull to capsule).
                const Math::vec3 surface = points[i] - out.normal * b.radius();
                const f32 depth = faceOffset - Math::dot(out.normal, surface);
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

    // Otherwise one point: support points on each shape along the separating normal (fallback shared with boxBox()/capsuleBox()).
    supportPointContact(a, transformA, b, transformB, bestPenetration, out);
    return true;
}

bool Narrowphase::convexHullBox(const ConvexHullShape& a, const Math::mat4& transformA,
                                const BoxShape& b, const Math::mat4& transformB,
                                ContactManifold& out, f32 margin)
{
    const Math::mat3 rotationB(transformB);
    // Clamped once for every axis-array index below, so the fixed-size arrays and bestAxis thresholds agree on a hull with more faces than kHullArrayCapacity.
    const u32 hullFaceCount = Math::min(a.faceCount(), kHullArrayCapacity);

    Math::vec3 hullEdges[kHullArrayCapacity];
    const u32 hullEdgeCount = hullEdgeDirections(a, transformA, hullEdges, kHullArrayCapacity);

    // Hull face normals stand in for boxBox()'s first three axes, box faces are the next three, then hull edges crossed with the box's three.
    Math::vec3 axes[kHullArrayCapacity + 3 + kHullArrayCapacity * 3];
    u32 axisCount = 0;
    for (u32 face = 0; face < hullFaceCount; ++face)
        axes[axisCount++] = hullFaceNormal(a, transformA, face);
    for (u32 i = 0; i < 3; ++i)
        axes[axisCount++] = Math::normalize(Math::vec3(rotationB[i]));
    for (u32 i = 0; i < hullEdgeCount; ++i)
        for (u32 j = 0; j < 3; ++j)
            axes[axisCount++] = Math::cross(hullEdges[i], Math::vec3(rotationB[j]));

    f32 bestPenetration = 1.0e30f;
    Math::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    u32 bestAxis = 0;
    for (u32 i = 0; i < axisCount; ++i)
    {
        f32 penetration = 0.0f;
        Math::vec3 normal(0.0f);
        if (!axisOverlap(a, transformA, b, transformB, axes[i], margin, penetration, normal))
            return false;
        if (Math::length(axes[i]) < kEpsilon)
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

    Math::vec3 referencePolygon[kHullArrayCapacity];
    u32 referenceCount = 0;
    Math::vec3 referenceNormal(0.0f);
    Math::vec3 incidentPolygon[kHullArrayCapacity];
    u32 incidentCount = 0;

    if (bestAxis < hullFaceCount)
    {
        u32 referenceFace = 0;
        f32 bestDot = -1.0e30f;
        for (u32 face = 0; face < hullFaceCount; ++face)
        {
            const f32 value = Math::dot(hullFaceNormal(a, transformA, face), out.normal);
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
        Math::vec3 boxCorners[8];
        b.corners(transformB, boxCorners);
        const u8* indices = BoxShape::faceCorners(incidentFaceIndex);
        incidentCount = 4;
        for (u32 i = 0; i < 4; ++i)
            incidentPolygon[i] = boxCorners[indices[i]];
    }
    else
    {
        u32 referenceFace = 0;
        f32 bestDot = -1.0e30f;
        for (u32 face = 0; face < 6; ++face)
        {
            const f32 value = Math::dot(BoxShape::faceNormal(transformB, face), -out.normal);
            if (value > bestDot)
            {
                bestDot = value;
                referenceFace = face;
            }
        }
        referenceNormal = BoxShape::faceNormal(transformB, referenceFace);
        Math::vec3 boxCorners[8];
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

bool Narrowphase::convexHullConvexHull(const ConvexHullShape& a, const Math::mat4& transformA,
                                       const ConvexHullShape& b, const Math::mat4& transformB,
                                       ContactManifold& out, f32 margin)
{
    // Clamped once for every axis-array index below, so the fixed-size arrays and bestAxis thresholds agree on a hull with more faces than kHullArrayCapacity.
    const u32 faceCountA = Math::min(a.faceCount(), kHullArrayCapacity);
    const u32 faceCountB = Math::min(b.faceCount(), kHullArrayCapacity);

    Math::vec3 edgesA[kHullArrayCapacity];
    const u32 edgeCountA = hullEdgeDirections(a, transformA, edgesA, kHullArrayCapacity);
    Math::vec3 edgesB[kHullArrayCapacity];
    const u32 edgeCountB = hullEdgeDirections(b, transformB, edgesB, kHullArrayCapacity);

    // A's face normals, B's face normals, then every unique A edge crossed with every unique B edge.
    Math::vec3 axes[kHullArrayCapacity * 2 + kHullArrayCapacity * kHullArrayCapacity];
    u32 axisCount = 0;
    for (u32 face = 0; face < faceCountA; ++face)
        axes[axisCount++] = hullFaceNormal(a, transformA, face);
    for (u32 face = 0; face < faceCountB; ++face)
        axes[axisCount++] = hullFaceNormal(b, transformB, face);
    for (u32 i = 0; i < edgeCountA; ++i)
        for (u32 j = 0; j < edgeCountB; ++j)
            axes[axisCount++] = Math::cross(edgesA[i], edgesB[j]);

    f32 bestPenetration = 1.0e30f;
    Math::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    u32 bestAxis = 0;
    for (u32 i = 0; i < axisCount; ++i)
    {
        f32 penetration = 0.0f;
        Math::vec3 normal(0.0f);
        if (!axisOverlap(a, transformA, b, transformB, axes[i], margin, penetration, normal))
            return false;
        if (Math::length(axes[i]) < kEpsilon)
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

    Math::vec3 referencePolygon[kHullArrayCapacity];
    u32 referenceCount = 0;
    Math::vec3 referenceNormal(0.0f);
    Math::vec3 incidentPolygon[kHullArrayCapacity];
    u32 incidentCount = 0;

    if (bestAxis < faceCountA)
    {
        u32 referenceFace = 0;
        f32 bestDot = -1.0e30f;
        for (u32 face = 0; face < faceCountA; ++face)
        {
            const f32 value = Math::dot(hullFaceNormal(a, transformA, face), out.normal);
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
            const f32 value = Math::dot(hullFaceNormal(b, transformB, face), -out.normal);
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

bool Narrowphase::convexPlane(const CollisionShape& a, const Math::mat4& transformA,
                              const PlaneShape& b, const Math::mat4& transformB,
                              ContactManifold& out, f32 margin)
{
    const Math::vec3 planeNormal = Math::normalize(Math::mat3(transformB) * b.normal());
    const Math::vec3 planeOrigin =
        Math::vec3(transformB * Math::vec4(b.normal() * b.constant(), 1.0f));
    const f32 planeConstant = Math::dot(planeNormal, planeOrigin);
    const Math::vec3 vertex = a.support(transformA, -planeNormal);
    const f32 distance = Math::dot(planeNormal, vertex) - planeConstant;
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

bool Narrowphase::collide(const CollisionShape& a, const Math::mat4& transformA,
                          const CollisionShape& b, const Math::mat4& transformB,
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
        // Solved in the other order and flipped: one routine per pair to get right.
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

// Normal a triangle contact pushes along, A towards B: face normal on the face; direction to the closest point on a rim edge (slides off real edges);
// face normal again on an edge/corner SHARED with another triangle, since that seam is interior and pushing along it stops a walking character dead.
bool triangleContactNormal(const TriangleShape& triangle, const Math::mat4& transform,
                           TriangleFeature feature, const Math::vec3& offset, f32 distanceSquared,
                           Math::vec3& out)
{
    const bool degenerate = distanceSquared <= kEpsilon * kEpsilon;
    if (!degenerate && !triangle.featureIsInternal(feature))
    {
        out = offset / std::sqrt(distanceSquared);
        return true;
    }

    const Math::vec3 raw = Math::mat3(transform) * triangle.rawNormal();
    const f32 length = Math::length(raw);
    if (length < kEpsilon)
        return false;
    Math::vec3 faceNormal = raw / length;
    // Oriented to the side the convex is actually on, so a body under a ceiling triangle is not pushed up through it.
    if (!degenerate && Math::dot(faceNormal, offset) < 0.0f)
        faceNormal = -faceNormal;
    out = faceNormal;
    return true;
}

} // namespace

bool Narrowphase::sphereTriangle(const SphereShape& a, const Math::mat4& transformA,
                                 const TriangleShape& b, const Math::mat4& transformB,
                                 ContactManifold& out, f32 margin)
{
    const Math::vec3 center(transformA[3]);
    const Math::vec3 v0 = Math::vec3(transformB * Math::vec4(b.vertex(0), 1.0f));
    const Math::vec3 v1 = Math::vec3(transformB * Math::vec4(b.vertex(1), 1.0f));
    const Math::vec3 v2 = Math::vec3(transformB * Math::vec4(b.vertex(2), 1.0f));

    TriangleFeature feature = TriangleFeature::Face;
    const Math::vec3 closest = closestPointOnTriangle(v0, v1, v2, center, &feature);
    const Math::vec3 offset = closest - center;
    const f32 distanceSquared = Math::dot(offset, offset);
    const f32 reach = a.radius() + margin;
    if (distanceSquared > reach * reach)
        return false;

    Math::vec3 normal;
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

bool Narrowphase::boxTriangle(const BoxShape& a, const Math::mat4& transformA,
                              const TriangleShape& b, const Math::mat4& transformB,
                              ContactManifold& out, f32 margin)
{
    const Math::mat3 rotationA(transformA);
    const Math::vec3 v0 = Math::vec3(transformB * Math::vec4(b.vertex(0), 1.0f));
    const Math::vec3 v1 = Math::vec3(transformB * Math::vec4(b.vertex(1), 1.0f));
    const Math::vec3 v2 = Math::vec3(transformB * Math::vec4(b.vertex(2), 1.0f));

    const Math::vec3 faceNormalRaw = Math::cross(v1 - v0, v2 - v0);
    const f32 faceLength = Math::length(faceNormalRaw);
    if (faceLength < kEpsilon)
        return false;
    const Math::vec3 faceNormal = faceNormalRaw / faceLength;

    // Thirteen axes: box's three face normals, triangle's one, and nine edge cross products.
    const Math::vec3 triangleEdges[3] = {v1 - v0, v2 - v1, v0 - v2};
    Math::vec3 axes[13];
    u32 axisCount = 0;
    for (u32 i = 0; i < 3; ++i)
        axes[axisCount++] = Math::normalize(Math::vec3(rotationA[i]));
    axes[axisCount++] = faceNormal;
    for (u32 i = 0; i < 3; ++i)
        for (u32 j = 0; j < 3; ++j)
            axes[axisCount++] = Math::cross(Math::vec3(rotationA[i]), triangleEdges[j]);

    f32 bestPenetration = std::numeric_limits<f32>::max();
    Math::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    u32 bestAxis = 0;
    for (u32 i = 0; i < axisCount; ++i)
    {
        f32 penetration = 0.0f;
        Math::vec3 normal(0.0f);
        if (!axisOverlap(a, transformA, b, transformB, axes[i], margin, penetration, normal))
            return false;
        if (Math::length(axes[i]) < kEpsilon)
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

    if (bestAxis > 3)
    {
        out.count = 1;
        const Math::vec3 pointA = a.support(transformA, out.normal);
        const Math::vec3 pointB = b.support(transformB, -out.normal);
        out.points[0].position = (pointA + pointB) * 0.5f;
        out.points[0].penetration = bestPenetration;
        out.points[0].normalImpulse = 0.0f;
        out.points[0].tangentImpulse[0] = 0.0f;
        out.points[0].tangentImpulse[1] = 0.0f;
        return true;
    }

    constexpr u32 capacity = 16;
    Math::vec3 polygon[capacity];
    Math::vec3 scratch[capacity];
    u32 count = 0;
    Math::vec3 referenceNormal(0.0f);
    f32 referenceOffset = 0.0f;
    Math::vec3 referenceInterior(0.0f);
    Math::vec3 edgeStart[4];
    Math::vec3 edgeEnd[4];
    u32 edgeCount = 0;

    Math::vec3 boxCorners[8];
    a.corners(transformA, boxCorners);

    if (bestAxis < 3)
    {
        u32 referenceFace = 0;
        f32 bestDot = -1.0e30f;
        for (u32 face = 0; face < 6; ++face)
        {
            const f32 value = Math::dot(BoxShape::faceNormal(transformA, face), out.normal);
            if (value > bestDot)
            {
                bestDot = value;
                referenceFace = face;
            }
        }
        referenceNormal = BoxShape::faceNormal(transformA, referenceFace);
        const u8* indices = BoxShape::faceCorners(referenceFace);
        referenceOffset = Math::dot(referenceNormal, boxCorners[indices[0]]);
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
        // Triangle is the reference: its outward normal must face the box, opposite the A-to-B normal.
        referenceNormal = -out.normal;
        referenceOffset = Math::dot(referenceNormal, v0);
        referenceInterior = (v0 + v1 + v2) / 3.0f;
        const Math::vec3 vertices[3] = {v0, v1, v2};
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
        const Math::vec3 edge = edgeEnd[i] - edgeStart[i];
        Math::vec3 planeNormal = Math::cross(referenceNormal, edge);
        const f32 planeLength = Math::length(planeNormal);
        if (planeLength < kEpsilon)
            continue;
        Math::vec3 unit = planeNormal / planeLength;
        f32 offset = Math::dot(unit, edgeStart[i]);
        if (Math::dot(unit, referenceInterior) > offset)
        {
            unit = -unit;
            offset = -offset;
        }
        count = clipPolygon(polygon, count, unit, offset, scratch, capacity);
        for (u32 p = 0; p < count; ++p)
            polygon[p] = scratch[p];
    }

    Math::vec3 kept[capacity];
    f32 depths[capacity];
    u32 keptCount = 0;
    for (u32 i = 0; i < count; ++i)
    {
        const f32 depth = referenceOffset - Math::dot(referenceNormal, polygon[i]);
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

bool Narrowphase::capsuleTriangle(const CapsuleShape& a, const Math::mat4& transformA,
                                  const TriangleShape& b, const Math::mat4& transformB,
                                  ContactManifold& out, f32 margin)
{
    Math::vec3 lower, upper;
    a.segment(transformA, lower, upper);
    const Math::vec3 v0 = Math::vec3(transformB * Math::vec4(b.vertex(0), 1.0f));
    const Math::vec3 v1 = Math::vec3(transformB * Math::vec4(b.vertex(1), 1.0f));
    const Math::vec3 v2 = Math::vec3(transformB * Math::vec4(b.vertex(2), 1.0f));

    // Closest point on the triangle to each segment end and segment-vs-edge; the nearest is the contact. Steadier than SAT: a capsule has no faces.
    TriangleFeature bestFeature = TriangleFeature::Face;
    Math::vec3 bestOnSegment = lower;
    Math::vec3 bestOnTriangle = closestPointOnTriangle(v0, v1, v2, lower, &bestFeature);
    f32 bestSquared = Math::dot(bestOnTriangle - lower, bestOnTriangle - lower);

    TriangleFeature upperFeature = TriangleFeature::Face;
    const Math::vec3 upperClosest = closestPointOnTriangle(v0, v1, v2, upper, &upperFeature);
    const f32 upperSquared = Math::dot(upperClosest - upper, upperClosest - upper);
    if (upperSquared < bestSquared)
    {
        bestSquared = upperSquared;
        bestOnSegment = upper;
        bestOnTriangle = upperClosest;
        bestFeature = upperFeature;
    }

    const Math::vec3 edges[3][2] = {{v0, v1}, {v1, v2}, {v2, v0}};
    static constexpr TriangleFeature edgeFeature[3] = {
        TriangleFeature::Edge0, TriangleFeature::Edge1, TriangleFeature::Edge2};
    for (u32 i = 0; i < 3; ++i)
    {
        Math::vec3 onSegment, onEdge;
        closestPointsBetweenSegments(lower, upper, edges[i][0], edges[i][1], onSegment, onEdge);
        const f32 squared = Math::dot(onEdge - onSegment, onEdge - onSegment);
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

    const Math::vec3 offset = bestOnTriangle - bestOnSegment;
    Math::vec3 normal;
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

bool Narrowphase::convexTrimesh(const CollisionShape& convex, const Math::mat4& convexTransform,
                                const TrimeshShape& mesh, const Math::mat4& meshTransform,
                                std::vector<ContactManifold>& out, f32 margin)
{
    // Convex's world bounds brought into the mesh's space: moving one box in is cheaper than moving every triangle out.
    AABB worldBox = convex.bounds(convexTransform);
    worldBox.min -= Math::vec3(margin);
    worldBox.max += Math::vec3(margin);

    // A body transform is rotation+translation only (normalized quaternion, no scale), so the inverse is the transpose; this runs per pair per substep.
    const Math::mat3 rotation(meshTransform);
    const Math::mat3 inverseRotation = Math::transpose(rotation);
    Math::mat4 inverseTransform(inverseRotation);
    inverseTransform[3] = Math::vec4(-(inverseRotation * Math::vec3(meshTransform[3])), 1.0f);
    const AABB localBox = transformAABB(worldBox, inverseTransform);

    // Reused rather than allocated per call.
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

bool raycastPlane(const PlaneShape& shape, const Math::mat4& transform, const Ray& ray,
                  f32 maxDistance, ShapeRayHit& hit)
{
    const Math::vec3 normal = Math::normalize(Math::mat3(transform) * shape.normal());
    const Math::vec3 origin =
        Math::vec3(transform * Math::vec4(shape.normal() * shape.constant(), 1.0f));
    Plane plane;
    plane.normal = normal;
    plane.d = -Math::dot(normal, origin);
    f32 distance = 0.0f;
    if (!ray.intersects(plane, distance) || distance > maxDistance)
        return false;
    hit.distance = distance;
    hit.point = ray.at(distance);
    hit.normal = Math::dot(ray.direction, normal) < 0.0f ? normal : -normal;
    return true;
}

bool raycastSphere(const SphereShape& sphere, const Math::mat4& transform, const Ray& ray,
                   f32 maxDistance, ShapeRayHit& hit)
{
    Sphere world;
    world.center = Math::vec3(transform[3]);
    world.radius = sphere.radius();

    f32 t = 0.0f;
    if (!ray.intersects(world, t) || t > maxDistance)
        return false;

    hit.distance = t;
    hit.point = ray.at(t);
    const Math::vec3 offset = hit.point - world.center;
    const f32 length = Math::length(offset);
    hit.normal = length > kEpsilon ? offset / length : Math::vec3(0.0f, 1.0f, 0.0f);
    return true;
}

bool raycastBox(const BoxShape& box, const Math::mat4& transform, const Ray& ray, f32 maxDistance,
                ShapeRayHit& hit)
{
    const Math::mat3 rotation(transform);
    const Math::mat3 inverseRotation = Math::transpose(rotation);
    const Math::vec3 center(transform[3]);

    Ray localRay;
    localRay.origin = inverseRotation * (ray.origin - center);
    localRay.direction = inverseRotation * ray.direction;

    const Math::vec3& half = box.halfExtents();
    AABB local;
    local.min = -half;
    local.max = half;

    f32 t = 0.0f;
    if (!localRay.intersects(local, t) || t < 0.0f || t > maxDistance)
        return false;

    const Math::vec3 localPoint = localRay.at(t);
    Math::vec3 localNormal(0.0f);
    f32 bestGap = std::numeric_limits<f32>::max();
    for (u32 axis = 0; axis < 3; ++axis)
    {
        const f32 gap = half[axis] - std::abs(localPoint[axis]);
        if (gap < bestGap)
        {
            bestGap = gap;
            localNormal = Math::vec3(0.0f);
            localNormal[axis] = localPoint[axis] >= 0.0f ? 1.0f : -1.0f;
        }
    }

    hit.distance = t;
    hit.point = center + rotation * localPoint;
    hit.normal = rotation * localNormal;
    return true;
}

bool raycastCapsule(const CapsuleShape& capsule, const Math::mat4& transform, const Ray& ray,
                    f32 maxDistance, ShapeRayHit& hit)
{
    const Math::mat3 rotation(transform);
    const Math::mat3 inverseRotation = Math::transpose(rotation);
    const Math::vec3 center(transform[3]);

    Ray localRay;
    localRay.origin = inverseRotation * (ray.origin - center);
    localRay.direction = inverseRotation * ray.direction;

    const f32 radius = capsule.radius();
    const f32 halfHeight = capsule.halfHeight();
    const Math::vec3& o = localRay.origin;
    const Math::vec3& d = localRay.direction;

    bool found = false;
    f32 nearest = maxDistance;
    Math::vec3 localPoint(0.0f);
    Math::vec3 localNormal(0.0f, 1.0f, 0.0f);

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
                localNormal = Math::normalize(Math::vec3(localPoint.x, 0.0f, localPoint.z));
                found = true;
            }
        }
    }

    Sphere cap;
    cap.radius = radius;
    cap.center = Math::vec3(0.0f, -halfHeight, 0.0f);
    f32 t = 0.0f;
    if (localRay.intersects(cap, t) && t < nearest)
    {
        nearest = t;
        localPoint = localRay.at(t);
        localNormal = Math::normalize(localPoint - cap.center);
        found = true;
    }
    cap.center = Math::vec3(0.0f, halfHeight, 0.0f);
    if (localRay.intersects(cap, t) && t < nearest)
    {
        nearest = t;
        localPoint = localRay.at(t);
        localNormal = Math::normalize(localPoint - cap.center);
        found = true;
    }

    if (!found)
        return false;

    hit.distance = nearest;
    hit.point = center + rotation * localPoint;
    hit.normal = rotation * localNormal;
    return true;
}

bool raycastTriangle(const TriangleShape& triangle, const Math::mat4& transform, const Ray& ray,
                     f32 maxDistance, ShapeRayHit& hit)
{
    const Math::vec3 v0 = Math::vec3(transform * Math::vec4(triangle.vertex(0), 1.0f));
    const Math::vec3 v1 = Math::vec3(transform * Math::vec4(triangle.vertex(1), 1.0f));
    const Math::vec3 v2 = Math::vec3(transform * Math::vec4(triangle.vertex(2), 1.0f));

    f32 t = 0.0f;
    if (!ray.intersects(v0, v1, v2, t) || t > maxDistance)
        return false;

    hit.distance = t;
    hit.point = ray.at(t);
    const Math::vec3 raw = Math::cross(v1 - v0, v2 - v0);
    const f32 length = Math::length(raw);
    hit.normal = length > kEpsilon ? raw / length : Math::vec3(0.0f, 1.0f, 0.0f);
    if (Math::dot(hit.normal, ray.direction) > 0.0f)
        hit.normal = -hit.normal;
    return true;
}

bool raycastTrimesh(const TrimeshShape& mesh, const Math::mat4& transform, const Ray& ray,
                    f32 maxDistance, ShapeRayHit& hit)
{
    const Math::mat3 rotation(transform);
    const Math::mat3 inverseRotation = Math::transpose(rotation);
    const Math::vec3 center(transform[3]);

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

bool overlapSphereShape(const SphereShape& sphere, const Math::mat4& transform,
                        const Math::vec3& centre, f32 radius)
{
    const Math::vec3 center(transform[3]);
    const f32 total = sphere.radius() + radius;
    return Math::dot(centre - center, centre - center) <= total * total;
}

bool overlapSphereShape(const BoxShape& box, const Math::mat4& transform, const Math::vec3& centre,
                        f32 radius)
{
    const Math::mat3 rotation(transform);
    const Math::vec3 boxCenter(transform[3]);
    const Math::vec3 local = Math::transpose(rotation) * (centre - boxCenter);
    const Math::vec3 closest = Math::clamp(local, -box.halfExtents(), box.halfExtents());
    const Math::vec3 offset = local - closest;
    return Math::dot(offset, offset) <= radius * radius;
}

bool overlapSphereShape(const CapsuleShape& capsule, const Math::mat4& transform,
                        const Math::vec3& centre, f32 radius)
{
    Math::vec3 lower, upper;
    capsule.segment(transform, lower, upper);
    const Math::vec3 closest = closestPointOnSegment(lower, upper, centre);
    const f32 total = capsule.radius() + radius;
    return Math::dot(centre - closest, centre - closest) <= total * total;
}

bool overlapSphereShape(const TriangleShape& triangle, const Math::mat4& transform,
                        const Math::vec3& centre, f32 radius)
{
    const Math::vec3 v0 = Math::vec3(transform * Math::vec4(triangle.vertex(0), 1.0f));
    const Math::vec3 v1 = Math::vec3(transform * Math::vec4(triangle.vertex(1), 1.0f));
    const Math::vec3 v2 = Math::vec3(transform * Math::vec4(triangle.vertex(2), 1.0f));
    const Math::vec3 closest = closestPointOnTriangle(v0, v1, v2, centre);
    const Math::vec3 offset = closest - centre;
    return Math::dot(offset, offset) <= radius * radius;
}

bool overlapSphereShape(const TrimeshShape& mesh, const Math::mat4& transform,
                        const Math::vec3& centre, f32 radius)
{
    const Math::mat3 rotation(transform);
    const Math::vec3 center(transform[3]);
    const Math::vec3 localCentre = Math::transpose(rotation) * (centre - center);

    static thread_local std::vector<u32> triangles;
    mesh.overlapSphere(localCentre, radius, triangles);
    return !triangles.empty();
}

bool overlapSphereShape(const PlaneShape& plane, const Math::mat4& transform,
                        const Math::vec3& centre, f32 radius)
{
    const Math::vec3 normal = Math::normalize(Math::mat3(transform) * plane.normal());
    const Math::vec3 origin =
        Math::vec3(transform * Math::vec4(plane.normal() * plane.constant(), 1.0f));
    return Math::dot(normal, centre - origin) <= radius;
}

} // namespace

bool Narrowphase::raycast(const CollisionShape& shape, const Math::mat4& transform, const Ray& ray,
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

bool Narrowphase::overlapSphere(const CollisionShape& shape, const Math::mat4& transform,
                                const Math::vec3& centre, f32 radius)
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
