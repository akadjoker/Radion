#include "PCH.h"

#include "collision/CollisionShape.h"

#include "DebugDraw3D.h"
#include "VoronoiShatter.h"
#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

namespace
{
// Corner i takes +halfExtents on axis a when bit a of i is set, which is the
// same numbering the volume mesher uses for a cell's corners.
Math::vec3 cornerSign(u32 index)
{
    return Math::vec3((index & 1) ? 1.0f : -1.0f, (index & 2) ? 1.0f : -1.0f,
                     (index & 4) ? 1.0f : -1.0f);
}

// Faces in axis order: -x, +x, -y, +y, -z, +z. Each lists its four corners
// wound so the polygon is convex when walked in order - a face clipped from
// corners in the wrong order produces a bowtie, which is the same trap the
// volume mesher's quad case had.
constexpr u8 kFaces[6][4] = {{0, 2, 6, 4}, {1, 5, 7, 3}, {0, 4, 5, 1},
                             {2, 3, 7, 6}, {0, 1, 3, 2}, {4, 6, 7, 5}};
} // namespace

void CollisionShape::project(const Math::mat4& transform, const Math::vec3& axis, f32& minimum,
                             f32& maximum) const
{
    minimum = Math::dot(axis, support(transform, -axis));
    maximum = Math::dot(axis, support(transform, axis));
}

// ------------------------------------------------------------------- sphere

SphereShape::SphereShape(f32 radius) : mRadius(Math::max(radius, 0.0f))
{
}

Math::mat3 SphereShape::inertia(f32 mass) const
{
    return Inertia::solidSphere(mass, mRadius);
}

AABB SphereShape::bounds(const Math::mat4& transform) const
{
    const Math::vec3 center(transform[3]);
    const Math::mat3 rotation(transform);
    const Math::vec3 extent(
        mRadius * Math::length(Math::vec3(rotation[0][0], rotation[1][0], rotation[2][0])),
        mRadius * Math::length(Math::vec3(rotation[0][1], rotation[1][1], rotation[2][1])),
        mRadius * Math::length(Math::vec3(rotation[0][2], rotation[1][2], rotation[2][2])));
    AABB box;
    box.min = center - extent;
    box.max = center + extent;
    return box;
}

Math::vec3 SphereShape::support(const Math::mat4& transform, const Math::vec3& direction) const
{
    const Math::vec3 center(transform[3]);
    const f32 length = Math::length(direction);
    if (length < 1e-8f)
        return center;
    return center + (direction / length) * mRadius;
}

void SphereShape::debugDraw(const Math::mat4& transform, Color color) const
{
    const Math::vec3 center(transform[3]);
    // The BODY's axes, not the world's. Drawn on world axes a sphere looks
    // identical however it is turned, so a rolling one reads as a sliding
    // one - and rolling is exactly what a sphere's debug view is for.
    const Math::vec3 right = Math::normalize(Math::vec3(transform[0]));
    const Math::vec3 up = Math::normalize(Math::vec3(transform[1]));
    const Math::vec3 forward = Math::normalize(Math::vec3(transform[2]));
    constexpr u32 segments = 24;
    DebugDraw().circle(center, right, up, mRadius, segments, color);
    DebugDraw().circle(center, right, forward, mRadius, segments, color);
    DebugDraw().circle(center, up, forward, mRadius, segments, color);
}

// ---------------------------------------------------------------------- box

BoxShape::BoxShape(const Math::vec3& halfExtents)
    : mHalfExtents(Math::max(halfExtents, Math::vec3(0.0f)))
{
}

Math::mat3 BoxShape::inertia(f32 mass) const
{
    return Inertia::box(mass, mHalfExtents);
}

void BoxShape::corners(const Math::mat4& transform, Math::vec3 out[8]) const
{
    for (u32 i = 0; i < 8; ++i)
        out[i] = Math::vec3(transform * Math::vec4(cornerSign(i) * mHalfExtents, 1.0f));
}

const u8* BoxShape::faceCorners(u32 index)
{
    return kFaces[index % 6];
}

Math::vec3 BoxShape::faceNormal(const Math::mat4& transform, u32 index)
{
    const u32 axis = (index % 6) / 2;
    const f32 sign = (index % 2) ? 1.0f : -1.0f;
    return Math::normalize(Math::vec3(transform[axis]) * sign);
}

AABB BoxShape::bounds(const Math::mat4& transform) const
{
    // The rotated box's AABB, without walking all eight corners: each world
    // axis reaches as far as the sum of the absolute row of the rotation
    // times the half extents.
    const Math::vec3 center(transform[3]);
    const Math::mat3 rotation(transform);
    const Math::vec3 extent(
        Math::dot(Math::abs(Math::vec3(rotation[0][0], rotation[1][0], rotation[2][0])), mHalfExtents),
        Math::dot(Math::abs(Math::vec3(rotation[0][1], rotation[1][1], rotation[2][1])), mHalfExtents),
        Math::dot(Math::abs(Math::vec3(rotation[0][2], rotation[1][2], rotation[2][2])),
                 mHalfExtents));
    AABB box;
    box.min = center - extent;
    box.max = center + extent;
    return box;
}

Math::vec3 BoxShape::support(const Math::mat4& transform, const Math::vec3& direction) const
{
    // The furthest corner is the one whose every local axis agrees in sign
    // with the direction - no search over eight, three comparisons.
    const Math::mat3 rotation(transform);
    const Math::vec3 local = Math::transpose(rotation) * direction;
    const Math::vec3 corner(local.x >= 0.0f ? mHalfExtents.x : -mHalfExtents.x,
                           local.y >= 0.0f ? mHalfExtents.y : -mHalfExtents.y,
                           local.z >= 0.0f ? mHalfExtents.z : -mHalfExtents.z);
    return Math::vec3(transform * Math::vec4(corner, 1.0f));
}

// ------------------------------------------------------------------ capsule

CapsuleShape::CapsuleShape(f32 radius, f32 halfHeight)
    : mRadius(Math::max(radius, 0.0f)), mHalfHeight(Math::max(halfHeight, 0.0f))
{
}

Math::mat3 CapsuleShape::inertia(f32 mass) const
{
    return Inertia::capsuleY(mass, mRadius, mHalfHeight * 2.0f);
}

void CapsuleShape::segment(const Math::mat4& transform, Math::vec3& lower, Math::vec3& upper) const
{
    const Math::vec3 axis = Math::vec3(transform[1]) * mHalfHeight;
    const Math::vec3 center(transform[3]);
    lower = center - axis;
    upper = center + axis;
}

AABB CapsuleShape::bounds(const Math::mat4& transform) const
{
    Math::vec3 lower, upper;
    segment(transform, lower, upper);
    AABB box;
    box.min = Math::min(lower, upper) - Math::vec3(mRadius);
    box.max = Math::max(lower, upper) + Math::vec3(mRadius);
    return box;
}

Math::vec3 CapsuleShape::support(const Math::mat4& transform, const Math::vec3& direction) const
{
    Math::vec3 lower, upper;
    segment(transform, lower, upper);
    // Whichever end is further along the direction, then out by the radius.
    const Math::vec3 end = Math::dot(direction, upper) >= Math::dot(direction, lower) ? upper : lower;
    const f32 length = Math::length(direction);
    if (length < 1e-8f)
        return end;
    return end + (direction / length) * mRadius;
}

PlaneShape::PlaneShape(const Math::vec3& normal, f32 constant)
    : mNormal(Math::dot(normal, normal) > 1.0e-12f ? Math::normalize(normal)
                                                       : Math::vec3(0.0f, 1.0f, 0.0f)),
      mConstant(constant)
{
}

Math::mat3 PlaneShape::inertia(f32) const
{
    return Math::mat3(0.0f);
}

AABB PlaneShape::bounds(const Math::mat4&) const
{
    AABB box;
    box.min = Math::vec3(-3.402823466e+38F);
    box.max = Math::vec3(3.402823466e+38F);
    return box;
}

Math::vec3 PlaneShape::support(const Math::mat4& transform, const Math::vec3&) const
{
    return Math::vec3(transform * Math::vec4(mNormal * mConstant, 1.0f));
}

void PlaneShape::debugDraw(const Math::mat4& transform, Color color) const
{
    Math::vec3 first;
    Math::vec3 second;
    if (std::abs(mNormal.z) > 0.70710678f)
    {
        const f32 inverseLength = 1.0f / std::sqrt(mNormal.y * mNormal.y + mNormal.z * mNormal.z);
        first = Math::vec3(0.0f, -mNormal.z * inverseLength, mNormal.y * inverseLength);
    }
    else
    {
        const f32 inverseLength = 1.0f / std::sqrt(mNormal.x * mNormal.x + mNormal.y * mNormal.y);
        first = Math::vec3(-mNormal.y * inverseLength, mNormal.x * inverseLength, 0.0f);
    }
    second = Math::cross(mNormal, first);
    const Math::vec3 origin = mNormal * mConstant;
    DebugDraw().line(Math::vec3(transform * Math::vec4(origin + first * 100.0f, 1.0f)),
                     Math::vec3(transform * Math::vec4(origin - first * 100.0f, 1.0f)), color);
    DebugDraw().line(Math::vec3(transform * Math::vec4(origin + second * 100.0f, 1.0f)),
                     Math::vec3(transform * Math::vec4(origin - second * 100.0f, 1.0f)), color);
}

void CapsuleShape::debugDraw(const Math::mat4& transform, Color color) const
{
    Math::vec3 lower, upper;
    segment(transform, lower, upper);
    const Math::vec3 up = Math::normalize(Math::vec3(transform[1]));
    const Math::vec3 right = Math::normalize(Math::vec3(transform[0]));
    const Math::vec3 forward = Math::normalize(Math::vec3(transform[2]));
    constexpr u32 segments = 20;

    DebugDraw().circle(lower, right, forward, mRadius, segments, color);
    DebugDraw().circle(upper, right, forward, mRadius, segments, color);
    // Two side lines each way, so the capsule reads as a volume rather than
    // two loose rings.
    DebugDraw().line(lower + right * mRadius, upper + right * mRadius, color);
    DebugDraw().line(lower - right * mRadius, upper - right * mRadius, color);
    DebugDraw().line(lower + forward * mRadius, upper + forward * mRadius, color);
    DebugDraw().line(lower - forward * mRadius, upper - forward * mRadius, color);
    // The rounded caps, which is what tells a capsule from a cylinder on
    // screen - and the difference that matters when it meets a step.
    DebugDraw().circle(upper, right, up, mRadius, segments, color);
    DebugDraw().circle(upper, forward, up, mRadius, segments, color);
    DebugDraw().circle(lower, right, up, mRadius, segments, color);
    DebugDraw().circle(lower, forward, up, mRadius, segments, color);
}

// ----------------------------------------------------------------- segments

Math::vec3 closestPointOnSegment(const Math::vec3& a, const Math::vec3& b, const Math::vec3& point)
{
    const Math::vec3 direction = b - a;
    const f32 lengthSquared = Math::dot(direction, direction);
    if (lengthSquared < 1e-12f)
        return a;
    const f32 t = Math::clamp(Math::dot(point - a, direction) / lengthSquared, 0.0f, 1.0f);
    return a + direction * t;
}

void closestPointsBetweenSegments(const Math::vec3& p1, const Math::vec3& q1, const Math::vec3& p2,
                                  const Math::vec3& q2, Math::vec3& c1, Math::vec3& c2)
{
    const Math::vec3 d1 = q1 - p1;
    const Math::vec3 d2 = q2 - p2;
    const Math::vec3 r = p1 - p2;
    const f32 a = Math::dot(d1, d1);
    const f32 e = Math::dot(d2, d2);
    const f32 f = Math::dot(d2, r);

    constexpr f32 epsilon = 1e-12f;
    f32 s = 0.0f;
    f32 t = 0.0f;

    if (a <= epsilon && e <= epsilon)
    {
        c1 = p1;
        c2 = p2;
        return;
    }
    if (a <= epsilon)
    {
        t = Math::clamp(f / e, 0.0f, 1.0f);
    }
    else
    {
        const f32 c = Math::dot(d1, r);
        if (e <= epsilon)
        {
            s = Math::clamp(-c / a, 0.0f, 1.0f);
        }
        else
        {
            const f32 b = Math::dot(d1, d2);
            const f32 denominator = a * e - b * b;
            // Zero denominator means the segments are parallel - there is no
            // single closest pair, so any point on one will do and the other
            // is clamped to it. Dividing here is the classic crash.
            s = denominator > epsilon ? Math::clamp((b * f - c * e) / denominator, 0.0f, 1.0f)
                                      : 0.0f;
            t = (b * s + f) / e;
            // Clamping t can move it off the segment, in which case s has to
            // be solved again against the clamped t.
            if (t < 0.0f)
            {
                t = 0.0f;
                s = Math::clamp(-c / a, 0.0f, 1.0f);
            }
            else if (t > 1.0f)
            {
                t = 1.0f;
                s = Math::clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
}

void BoxShape::debugDraw(const Math::mat4& transform, Color color) const
{
    Math::vec3 point[8];
    corners(transform, point);
    static constexpr u8 edges[12][2] = {{0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7},
                                        {7, 6}, {6, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (const auto& edge : edges)
        DebugDraw().line(point[edge[0]], point[edge[1]], color);
}

Math::vec3 closestPointOnTriangle(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                                 const Math::vec3& point, TriangleFeature* feature)
{
    TriangleFeature landed = TriangleFeature::Face;
    struct Report
    {
        TriangleFeature* out;
        const TriangleFeature* value;
        ~Report()
        {
            if (out)
                *out = *value;
        }
    } report{feature, &landed};

    const Math::vec3 ab = b - a;
    const Math::vec3 ac = c - a;
    const Math::vec3 ap = point - a;
    const f32 d1 = Math::dot(ab, ap);
    const f32 d2 = Math::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
    {
        landed = TriangleFeature::Vertex0;
        return a;
    }

    const Math::vec3 bp = point - b;
    const f32 d3 = Math::dot(ab, bp);
    const f32 d4 = Math::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3)
    {
        landed = TriangleFeature::Vertex1;
        return b;
    }

    const f32 vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
    {
        const f32 denominator = d1 - d3;
        const f32 v = Math::abs(denominator) > 1e-12f ? d1 / denominator : 0.0f;
        landed = TriangleFeature::Edge0;
        return a + ab * v;
    }

    const Math::vec3 cp = point - c;
    const f32 d5 = Math::dot(ab, cp);
    const f32 d6 = Math::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6)
    {
        landed = TriangleFeature::Vertex2;
        return c;
    }

    const f32 vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
    {
        const f32 denominator = d2 - d6;
        const f32 w = Math::abs(denominator) > 1e-12f ? d2 / denominator : 0.0f;
        landed = TriangleFeature::Edge2;
        return a + ac * w;
    }

    const f32 va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
    {
        const f32 denominator = (d4 - d3) + (d5 - d6);
        const f32 w = Math::abs(denominator) > 1e-12f ? (d4 - d3) / denominator : 0.0f;
        landed = TriangleFeature::Edge1;
        return b + (c - b) * w;
    }

    const f32 denominator = va + vb + vc;
    if (Math::abs(denominator) <= 1e-12f)
        return a;
    const f32 inverse = 1.0f / denominator;
    return a + ab * (vb * inverse) + ac * (vc * inverse);
}

TriangleShape::TriangleShape(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                             u8 sharedEdges)
    : mSharedEdges(sharedEdges)
{
    mVertices[0] = a;
    mVertices[1] = b;
    mVertices[2] = c;
}

bool TriangleShape::featureIsInternal(TriangleFeature feature) const
{
    switch (feature)
    {
    case TriangleFeature::Face:
        return true;
    case TriangleFeature::Edge0:
        return edgeIsShared(0);
    case TriangleFeature::Edge1:
        return edgeIsShared(1);
    case TriangleFeature::Edge2:
        return edgeIsShared(2);
    // A vertex is only inside the surface when both edges meeting there are.
    // One free edge and it is a real corner, which has to push outwards.
    case TriangleFeature::Vertex0:
        return edgeIsShared(2) && edgeIsShared(0);
    case TriangleFeature::Vertex1:
        return edgeIsShared(0) && edgeIsShared(1);
    case TriangleFeature::Vertex2:
        return edgeIsShared(1) && edgeIsShared(2);
    }
    return false;
}

Math::vec3 TriangleShape::rawNormal() const
{
    return Math::cross(mVertices[1] - mVertices[0], mVertices[2] - mVertices[0]);
}

Math::mat3 TriangleShape::inertia(f32) const
{
    return Math::mat3(0.0f);
}

AABB TriangleShape::bounds(const Math::mat4& transform) const
{
    AABB box;
    for (const Math::vec3& vertex : mVertices)
        box.expand(Math::vec3(transform * Math::vec4(vertex, 1.0f)));
    return box;
}

Math::vec3 TriangleShape::support(const Math::mat4& transform, const Math::vec3& direction) const
{
    Math::vec3 best = Math::vec3(transform * Math::vec4(mVertices[0], 1.0f));
    f32 bestDot = Math::dot(best, direction);
    for (u32 i = 1; i < 3; ++i)
    {
        const Math::vec3 world = Math::vec3(transform * Math::vec4(mVertices[i], 1.0f));
        const f32 dot = Math::dot(world, direction);
        if (dot > bestDot)
        {
            bestDot = dot;
            best = world;
        }
    }
    return best;
}

void TriangleShape::debugDraw(const Math::mat4& transform, Color color) const
{
    const Math::vec3 a = Math::vec3(transform * Math::vec4(mVertices[0], 1.0f));
    const Math::vec3 b = Math::vec3(transform * Math::vec4(mVertices[1], 1.0f));
    const Math::vec3 c = Math::vec3(transform * Math::vec4(mVertices[2], 1.0f));
    DebugDraw().line(a, b, color);
    DebugDraw().line(b, c, color);
    DebugDraw().line(c, a, color);
}

TrimeshShape::TrimeshShape(const Math::vec3* vertices, u32 vertexCount, const u32* indices,
                           u32 indexCount)
{
    if (vertices && vertexCount > 0)
        mVertices.assign(vertices, vertices + vertexCount);
    if (indices && indexCount >= 3)
        mIndices.assign(indices, indices + (indexCount - indexCount % 3));

    const u32 count = triangleCount();
    std::vector<AABB> triangleBounds(count);
    for (u32 i = 0; i < count; ++i)
    {
        AABB& box = triangleBounds[i];
        for (u32 corner = 0; corner < 3; ++corner)
        {
            const u32 index = mIndices[i * 3 + corner];
            if (index < mVertices.size())
                box.expand(mVertices[index]);
        }
        mBounds.merge(box);
    }
    mTree.build(triangleBounds.data(), count);
    buildAdjacency();
}

void TrimeshShape::buildAdjacency()
{
    const u32 count = triangleCount();
    mSharedEdges.assign(count, 0);
    if (count == 0)
        return;

    // Every edge, keyed by its two vertex indices in ascending order so the
    // two triangles that use it in opposite winding still land on one key.
    struct EdgeOwner
    {
        u64 key;
        u32 triangle;
        u32 opposite;
        u8 edge;
    };
    std::vector<EdgeOwner> edges;
    edges.reserve(static_cast<usize>(count) * 3);
    for (u32 triangle = 0; triangle < count; ++triangle)
    {
        for (u8 edge = 0; edge < 3; ++edge)
        {
            const u32 from = mIndices[triangle * 3 + edge];
            const u32 to = mIndices[triangle * 3 + (edge + 1) % 3];
            const u32 opposite = mIndices[triangle * 3 + (edge + 2) % 3];
            const u64 low = from < to ? from : to;
            const u64 high = from < to ? to : from;
            edges.push_back({(low << 32) | high, triangle, opposite, edge});
        }
    }

    std::sort(edges.begin(), edges.end(),
              [](const EdgeOwner& a, const EdgeOwner& b)
              {
                  return a.key < b.key;
              });

    for (usize i = 0; i + 1 < edges.size();)
    {
        usize end = i + 1;
        while (end < edges.size() && edges[end].key == edges[i].key)
            ++end;
        // Shared is not enough: the edge also has to be flat or concave.
        //
        // A step's lip is shared by the tread and the riser, and it is
        // CONVEX - the surface turns away there. Treating it as interior
        // hands a contact the tread's own normal, which points straight up,
        // and a character walking into the step is lifted onto it however
        // tall it is. That is a ratchet up any wall built from two surfaces
        // meeting at an edge. Only flat and concave edges are seams.
        if (end - i == 2)
        {
            const u32 t0 = edges[i].triangle;
            const u32 t1 = edges[i + 1].triangle;
            const Math::vec3 normal0 = triangle(t0).rawNormal();
            const Math::vec3 anchor = mVertices[static_cast<u32>(edges[i].key >> 32)];
            const Math::vec3 opposite = mVertices[edges[i + 1].opposite];
            const f32 side = Math::dot(normal0, opposite - anchor);
            const f32 tolerance = Math::length(normal0) * 1.0e-4f;
            if (side >= -tolerance)
            {
                mSharedEdges[t0] |= static_cast<u8>(1u << edges[i].edge);
                mSharedEdges[t1] |= static_cast<u8>(1u << edges[i + 1].edge);
            }
        }
        i = end;
    }
}

Math::mat3 TrimeshShape::inertia(f32) const
{
    return Math::mat3(0.0f);
}

AABB TrimeshShape::bounds(const Math::mat4& transform) const
{
    return transformAABB(mBounds, transform);
}

Math::vec3 TrimeshShape::support(const Math::mat4& transform, const Math::vec3& direction) const
{
    Math::vec3 best(0.0f);
    f32 bestDot = -std::numeric_limits<f32>::max();
    for (const Math::vec3& vertex : mVertices)
    {
        const Math::vec3 world = Math::vec3(transform * Math::vec4(vertex, 1.0f));
        const f32 dot = Math::dot(world, direction);
        if (dot > bestDot)
        {
            bestDot = dot;
            best = world;
        }
    }
    return best;
}

TriangleShape TrimeshShape::triangle(u32 index) const
{
    const u32 base = index * 3;
    return TriangleShape(mVertices[mIndices[base]], mVertices[mIndices[base + 1]],
                         mVertices[mIndices[base + 2]],
                         index < mSharedEdges.size() ? mSharedEdges[index] : 0);
}

void TrimeshShape::query(const AABB& localBox, std::vector<u32>& out) const
{
    mTree.queryCandidates(localBox, out);
}

namespace
{

bool pointInTriangle(const Math::vec3& p, const Math::vec3& v1, const Math::vec3& v2,
                     const Math::vec3& v3, const Math::vec3& normal)
{
    Math::vec3 e = v2 - v1;
    Math::vec3 d = v1 - p;
    if (Math::dot(d, Math::cross(e, normal)) < 0.0f)
        return false;
    e = v3 - v2;
    d = v2 - p;
    if (Math::dot(d, Math::cross(e, normal)) < 0.0f)
        return false;
    e = v1 - v3;
    d = v3 - p;
    if (Math::dot(d, Math::cross(e, normal)) < 0.0f)
        return false;
    return true;
}

// Smallest non-negative quadratic root, or false when both roots are negative.
bool solveCollision(f32 a, f32 b, f32 c, f32& t)
{
    const f32 disc = b * b - 4.0f * a * c;
    if (disc < 0.0f)
        return false;
    const f32 sq = std::sqrt(disc);
    const f32 inv = 1.0f / (2.0f * a);
    f32 t0 = (-b - sq) * inv;
    f32 t1 = (-b + sq) * inv;
    if (t1 < t0)
        std::swap(t0, t1);
    if (t1 < 0.0f)
        return false;
    t = (t0 < 0.0f) ? t1 : t0;
    return true;
}

// tMax is in/out: the caller's current best, tightened to this plane's hit.
// A negative tMax means the sphere already overlaps the plane.
bool sphereIntersectPlane(const Math::vec3& center, f32 radius, const Math::vec3& velocity,
                          const Math::vec3& planeNormal, const Math::vec3& planePoint, f32& tMax)
{
    const f32 numer = Math::dot(center - planePoint, planeNormal) - radius;
    const f32 denom = Math::dot(velocity, planeNormal);

    if (numer < 0.0f || denom > -1e-7f)
    {
        if (denom > -1e-5f)
            return false; // moving away
        if (numer < -radius)
            return false; // too far behind to matter
        tMax = numer;     // penetration depth, negative
        return true;
    }

    const f32 t = -(numer / denom);
    if (t < 0.0f || t > tMax)
        return false;
    tMax = t;
    return true;
}

bool sphereIntersectPoint(const Math::vec3& center, f32 radius, const Math::vec3& velocity,
                          const Math::vec3& point, f32& tMax, Math::vec3& normal)
{
    const Math::vec3 l = center - point;
    const f32 l2 = Math::dot(l, l);
    const f32 a = Math::dot(velocity, velocity);
    const f32 b = 2.0f * Math::dot(velocity, l);
    const f32 c = l2 - radius * radius;

    if (c < 0.0f)
    {
        const f32 len = std::sqrt(l2);
        const f32 t = len - radius;
        if (tMax < t)
            return false;
        normal = l / len;
        tMax = t;
        return true;
    }

    if (tMax < 0.0f)
        return false;
    f32 t = 0.0f;
    if (!solveCollision(a, b, c, t))
        return false;
    if (t > tMax)
        return false;
    normal = Math::normalize(center + velocity * t - point);
    tMax = t;
    return true;
}

bool sphereIntersectSegment(const Math::vec3& center, f32 radius, const Math::vec3& velocity,
                            const Math::vec3& v1, const Math::vec3& v2, f32& tMax, Math::vec3& normal)
{
    Math::vec3 e = v2 - v1;
    const Math::vec3 l = center - v1;
    const f32 eLen = Math::length(e);
    if (eLen < 1e-5f)
        return false;
    e /= eLen;

    const Math::vec3 x = Math::cross(l, e);
    const Math::vec3 y = Math::cross(velocity, e);
    const f32 a = Math::dot(y, y);
    const f32 b = 2.0f * Math::dot(x, y);
    const f32 c = Math::dot(x, x) - radius * radius;

    if (c < 0.0f)
    {
        const f32 d = Math::dot(l, e);
        if (d < 0.0f)
            return sphereIntersectPoint(center, radius, velocity, v1, tMax, normal);
        if (d > eLen)
            return sphereIntersectPoint(center, radius, velocity, v2, tMax, normal);
        const Math::vec3 onEdge = v1 + e * d;
        normal = Math::normalize(center - onEdge);
        const f32 t = Math::length(center - onEdge) - radius;
        if (tMax < t)
            return false;
        tMax = t;
        return true;
    }

    if (tMax < 0.0f)
        return false;
    f32 t = 0.0f;
    if (!solveCollision(a, b, c, t))
        return false;
    if (t > tMax)
        return false;

    const Math::vec3 collCenter = center + velocity * t;
    const f32 d = Math::dot(collCenter - v1, e);
    if (d < 0.0f)
        return sphereIntersectPoint(center, radius, velocity, v1, tMax, normal);
    if (d > eLen)
        return sphereIntersectPoint(center, radius, velocity, v2, tMax, normal);

    normal = Math::normalize(collCenter - (v1 + e * d));
    tMax = t;
    return true;
}

bool sphereIntersectTriangle(const Math::vec3& center, f32 radius, const Math::vec3& velocity,
                             const Math::vec3& v0, const Math::vec3& v1, const Math::vec3& v2,
                             const Math::vec3& normal, f32& tMax, Math::vec3& outNormal)
{
    f32 t = tMax;
    if (!sphereIntersectPlane(center, radius, velocity, normal, v0, t))
        return false;

    const Math::vec3 collCenter = (t < 0.0f) ? center - normal * t : center + velocity * t;
    if (pointInTriangle(collCenter, v0, v1, v2, normal))
    {
        outNormal = normal;
        tMax = t;
        return true;
    }

    bool hit = false;
    hit |= sphereIntersectSegment(center, radius, velocity, v0, v1, tMax, outNormal);
    hit |= sphereIntersectSegment(center, radius, velocity, v1, v2, tMax, outNormal);
    hit |= sphereIntersectSegment(center, radius, velocity, v2, v0, tMax, outNormal);
    return hit;
}

} // namespace

bool TrimeshShape::sweepEllipsoid(const Math::vec3& localCentre, const Math::vec3& radii,
                                  const Math::vec3& velocity, SweepHit& hit) const
{
    if (radii.x <= 0.0f || radii.y <= 0.0f || radii.z <= 0.0f)
        return false;

    // Everything the swept volume could reach: both ends of the path grown by
    // the largest half-extent. The tree throws away the rest without ever
    // touching a triangle.
    const f32 largest = Math::max(radii.x, Math::max(radii.y, radii.z));
    const Math::vec3 pad(largest + 0.01f);
    AABB swept;
    swept.expand(localCentre - pad);
    swept.expand(localCentre + pad);
    swept.expand(localCentre + velocity - pad);
    swept.expand(localCentre + velocity + pad);

    static thread_local std::vector<u32> candidates;
    mTree.queryCandidates(swept, candidates);
    if (candidates.empty())
        return false;

    // Scaled into ellipsoid space the test becomes a unit sphere, which is
    // the only shape the swept intersections above know how to handle. The
    // normal scales by radii on the way back, not by 1/radii.
    const Math::vec3 inverseRadii(1.0f / radii.x, 1.0f / radii.y, 1.0f / radii.z);
    const Math::vec3 eCentre = localCentre * inverseRadii;
    const Math::vec3 eVelocity = velocity * inverseRadii;

    // The running best starts at 1.0, not infinity: a hit beyond the end of
    // the swept segment (t > 1) is not a contact for this query, and the
    // reference rejects it the same way (nearestDistance >= 1 means no
    // collision). Without this the sweep reports a wall five units away with
    // t = 50 as the closest contact, and the slide moves the character all
    // the way there - the castle's "touch a wall and it flies off".
    f32 best = 1.0f;
    bool found = false;
    for (u32 index : candidates)
    {
        const u32 base = index * 3;
        const Math::vec3& v0 = mVertices[mIndices[base]];
        const Math::vec3& v1 = mVertices[mIndices[base + 1]];
        const Math::vec3& v2 = mVertices[mIndices[base + 2]];
        const Math::vec3 raw = Math::cross(v1 - v0, v2 - v0);
        if (Math::length(raw) < 1.0e-12f)
            continue;

        const Math::vec3 e0 = v0 * inverseRadii;
        const Math::vec3 e1 = v1 * inverseRadii;
        const Math::vec3 e2 = v2 * inverseRadii;
        // One-sided, from the winding. Orienting the normal towards the
        // sweeper instead was tried and is much worse: an ellipsoid sitting
        // slightly inside a surface then flips its contact normal according
        // to which side the centre landed on, and the character is thrown one
        // way on one frame and the other way on the next.
        const Math::vec3 eNormal = Math::normalize(Math::normalize(raw) * radii);

        f32 t = best;
        Math::vec3 eHitNormal(0.0f);
        if (!sphereIntersectTriangle(eCentre, 1.0f, eVelocity, e0, e1, e2, eNormal, t, eHitNormal))
            continue;
        best = t;
        hit.t = t;
        hit.triangle = index;
        hit.normal = Math::normalize(eHitNormal * inverseRadii);
        found = true;
    }
    return found;
}

bool TrimeshShape::sweepSphere(const Math::vec3& localCentre, f32 radius, const Math::vec3& velocity,
                               SweepHit& hit) const
{
    return sweepEllipsoid(localCentre, Math::vec3(radius), velocity, hit);
}

Math::vec3 TrimeshShape::slideCamera(const Math::vec3& from, const Math::vec3& to, f32 radius,
                                    f32 margin) const
{
    const Math::vec3 delta = to - from;
    if (Math::dot(delta, delta) < 1.0e-6f)
        return to;

    // One swept sphere from the anchor to the desired position. hit.t is the
    // fraction of the way there; a negative t means the anchor itself is
    // inside something, and the camera stays at the anchor. The contact
    // normal points away from the surface, so adding it keeps the sphere
    // clear instead of grazing it.
    SweepHit hit;
    if (sweepSphere(from, radius, delta, hit))
    {
        const f32 t = Math::clamp(hit.t, 0.0f, 1.0f);
        Math::vec3 position = from + delta * t;
        if (margin > 0.0f)
            position += hit.normal * margin;
        return position;
    }
    return to;
}

bool TrimeshShape::raycast(const Ray& localRay, f32 maxDistance, RayHit& hit) const
{
    static thread_local std::vector<u32> candidates;
    mTree.queryCandidates(localRay, maxDistance, candidates);
    if (candidates.empty())
        return false;

    bool found = false;
    f32 nearest = maxDistance;
    for (u32 index : candidates)
    {
        const u32 base = index * 3;
        const Math::vec3& v0 = mVertices[mIndices[base]];
        const Math::vec3& v1 = mVertices[mIndices[base + 1]];
        const Math::vec3& v2 = mVertices[mIndices[base + 2]];
        f32 distance = 0.0f;
        // The tree hands back whatever boxes the ray crossed, in no
        // particular order, so every candidate has to be tested and the
        // nearest kept - stopping at the first hit returns the wrong triangle.
        if (!localRay.intersects(v0, v1, v2, distance) || distance >= nearest)
            continue;
        nearest = distance;
        hit.triangle = index;
        hit.distance = distance;
        hit.point = localRay.at(distance);
        const Math::vec3 raw = Math::cross(v1 - v0, v2 - v0);
        const f32 length = Math::length(raw);
        hit.normal = length > 1.0e-8f ? raw / length : Math::vec3(0.0f, 1.0f, 0.0f);
        found = true;
    }
    return found;
}

void TrimeshShape::overlapSphere(const Math::vec3& localCentre, f32 radius,
                                 std::vector<u32>& out) const
{
    out.clear();
    Sphere sphere;
    sphere.center = localCentre;
    sphere.radius = radius;

    static thread_local std::vector<u32> candidates;
    mTree.queryCandidates(sphere, candidates);
    const f32 radiusSquared = radius * radius;
    for (u32 index : candidates)
    {
        const u32 base = index * 3;
        const Math::vec3 closest =
            closestPointOnTriangle(mVertices[mIndices[base]], mVertices[mIndices[base + 1]],
                                   mVertices[mIndices[base + 2]], localCentre);
        const Math::vec3 delta = closest - localCentre;
        if (Math::dot(delta, delta) <= radiusSquared)
            out.push_back(index);
    }
}

void TrimeshShape::windingErrors(std::vector<u32>& out) const
{
    out.clear();
    const u32 count = triangleCount();
    if (count == 0)
        return;

    struct Use
    {
        u64 key;
        u32 from;
        u32 triangle;
    };
    std::vector<Use> uses;
    uses.reserve(static_cast<usize>(count) * 3);
    for (u32 triangle = 0; triangle < count; ++triangle)
    {
        for (u32 edge = 0; edge < 3; ++edge)
        {
            const u32 from = mIndices[triangle * 3 + edge];
            const u32 to = mIndices[triangle * 3 + (edge + 1) % 3];
            const u64 low = from < to ? from : to;
            const u64 high = from < to ? to : from;
            uses.push_back({(low << 32) | high, from, triangle});
        }
    }
    std::sort(uses.begin(), uses.end(),
              [](const Use& a, const Use& b)
              {
                  return a.key < b.key;
              });

    for (usize i = 0; i + 1 < uses.size();)
    {
        usize end = i + 1;
        while (end < uses.size() && uses[end].key == uses[i].key)
            ++end;
        // Traversed the same way by both: one of the two is wound backwards.
        if (end - i == 2 && uses[i].from == uses[i + 1].from)
        {
            out.push_back(uses[i].triangle);
            out.push_back(uses[i + 1].triangle);
        }
        i = end;
    }

    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
}

void TrimeshShape::debugDrawFaceNormals(const Math::mat4& transform, f32 length, Color color,
                                        Color flippedColor) const
{
    std::vector<u32> flipped;
    windingErrors(flipped);

    const Math::mat3 rotation(transform);
    const u32 count = triangleCount();
    for (u32 i = 0; i < count; ++i)
    {
        const u32 base = i * 3;
        const Math::vec3& v0 = mVertices[mIndices[base]];
        const Math::vec3& v1 = mVertices[mIndices[base + 1]];
        const Math::vec3& v2 = mVertices[mIndices[base + 2]];
        const Math::vec3 raw = Math::cross(v1 - v0, v2 - v0);
        if (Math::length(raw) < 1.0e-12f)
            continue;

        const Math::vec3 centre = Math::vec3(transform * Math::vec4((v0 + v1 + v2) / 3.0f, 1.0f));
        const Math::vec3 normal = Math::normalize(rotation * Math::normalize(raw));
        const bool bad = std::binary_search(flipped.begin(), flipped.end(), i);
        DebugDraw().line(centre, centre + normal * length, bad ? flippedColor : color);
    }
}

void TrimeshShape::debugDraw(const Math::mat4& transform, Color color) const
{
    const u32 count = triangleCount();
    for (u32 i = 0; i < count; ++i)
        triangle(i).debugDraw(transform, color);
}

// ------------------------------------------------------------- convex hull

ConvexHullShape::ConvexHullShape(const Math::vec3* vertices, u32 vertexCount, const Edge* edges,
                                 u32 edgeCount, const int* faces, u32 faceCount)
{
    if (vertices && vertexCount > 0)
        mVertices.assign(vertices, vertices + vertexCount);
    if (edges && edgeCount > 0)
        mEdges.assign(edges, edges + edgeCount);
    if (faces && faceCount > 0)
        mFaces.assign(faces, faces + faceCount);
}

ConvexHullShape::ConvexHullShape(const Radion::Geometry::Shard& shard)
    : mVertices(shard.vertices), mEdges(shard.edges), mFaces(shard.faces)
{
}

Math::mat3 ConvexHullShape::inertia(f32 mass) const
{
    return Inertia::convexHull(mass, mVertices.data(), static_cast<u32>(mVertices.size()),
                               mEdges.data(), static_cast<u32>(mEdges.size()), mFaces.data(),
                               static_cast<u32>(mFaces.size()));
}

AABB ConvexHullShape::bounds(const Math::mat4& transform) const
{
    AABB box;
    for (const Math::vec3& vertex : mVertices)
        box.expand(Math::vec3(transform * Math::vec4(vertex, 1.0f)));
    return box;
}

Math::vec3 ConvexHullShape::support(const Math::mat4& transform, const Math::vec3& direction) const
{
    const Math::mat3 rotation(transform);
    const Math::vec3 localDirection = Math::transpose(rotation) * direction;

    Math::vec3 bestLocal(0.0f);
    f32 bestDot = -std::numeric_limits<f32>::max();
    for (const Math::vec3& vertex : mVertices)
    {
        const f32 dot = Math::dot(vertex, localDirection);
        if (dot > bestDot)
        {
            bestDot = dot;
            bestLocal = vertex;
        }
    }
    return Math::vec3(transform * Math::vec4(bestLocal, 1.0f));
}

void ConvexHullShape::debugDraw(const Math::mat4& transform, Color color) const
{
    // Every edge is stored twice, once each direction - drawn only from the
    // half whose own index is the smaller of the pair, so each edge of the
    // hull is one line and not two overlapping ones.
    for (usize i = 0; i < mEdges.size(); ++i)
    {
        const Edge& edge = mEdges[i];
        const usize reverseIndex = static_cast<usize>(edge.getReverseEdge() - mEdges.data());
        if (reverseIndex <= i)
            continue;
        const Math::vec3 a =
            Math::vec3(transform * Math::vec4(mVertices[edge.getSourceVertex()], 1.0f));
        const Math::vec3 b =
            Math::vec3(transform * Math::vec4(mVertices[edge.getTargetVertex()], 1.0f));
        DebugDraw().line(a, b, color);
    }
}

} // namespace Radion::Physics
