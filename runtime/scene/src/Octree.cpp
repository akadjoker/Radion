#include "PCH.h"

#include "Octree.h"

#include "DebugDraw3D.h"

#include <array>
#include <cmath>

namespace Radion
{

namespace
{

bool pointInTriangle(const glm::vec3& p, const glm::vec3& v1, const glm::vec3& v2,
                     const glm::vec3& v3, const glm::vec3& normal)
{
    glm::vec3 e = v2 - v1;
    glm::vec3 d = v1 - p;
    if (glm::dot(d, glm::cross(e, normal)) < 0.0f)
        return false;
    e = v3 - v2;
    d = v2 - p;
    if (glm::dot(d, glm::cross(e, normal)) < 0.0f)
        return false;
    e = v1 - v3;
    d = v3 - p;
    if (glm::dot(d, glm::cross(e, normal)) < 0.0f)
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
// A negative tMax means the sphere already overlaps the plane (penetration).
bool sphereIntersectPlane(const glm::vec3& center, f32 radius, const glm::vec3& velocity,
                          const glm::vec3& planeNormal, const glm::vec3& planePoint, f32& tMax)
{
    const f32 numer = glm::dot(center - planePoint, planeNormal) - radius;
    const f32 denom = glm::dot(velocity, planeNormal);

    // Overlap: only counts if the sphere is actually within reach of the
    // plane and not moving away from it.
    if (numer < 0.0f || denom > -1e-7f)
    {
        // A sphere already inside the plane is in contact whether it moves or
        // not: with zero velocity denom is zero, and the "moving away" guard
        // used to swallow every resting overlap.
        const bool embedded = numer < 0.0f;
        if (!embedded && denom > -1e-5f)
            return false; // not touching and not approaching
        if (embedded && denom > 1e-5f)
            return false; // touching but on its way out
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

bool sphereIntersectPoint(const glm::vec3& center, f32 radius, const glm::vec3& velocity,
                          const glm::vec3& point, f32& tMax, glm::vec3& normal)
{
    const glm::vec3 l = center - point;
    const f32 l2 = glm::dot(l, l);
    const f32 a = glm::dot(velocity, velocity);
    const f32 b = 2.0f * glm::dot(velocity, l);
    const f32 c = l2 - radius * radius;

    if (c < 0.0f) // overlapping the point
    {
        const f32 len = std::sqrt(l2);
        const f32 t = len - radius; // penetration depth
        if (tMax < t)
            return false;
        normal = l / len;
        tMax = t;
        return true;
    }

    if (tMax < 0.0f)
        return false; // already checking overlaps
    f32 t = 0.0f;
    if (!solveCollision(a, b, c, t))
        return false;
    if (t > tMax)
        return false;
    normal = glm::normalize(center + velocity * t - point);
    tMax = t;
    return true;
}

bool sphereIntersectSegment(const glm::vec3& center, f32 radius, const glm::vec3& velocity,
                            const glm::vec3& v1, const glm::vec3& v2, f32& tMax, glm::vec3& normal)
{
    glm::vec3 e = v2 - v1;
    const glm::vec3 l = center - v1;
    const f32 eLen = glm::length(e);
    if (eLen < 1e-5f)
        return false; // degenerate edge
    e /= eLen;

    const glm::vec3 x = glm::cross(l, e); // distance from segment line
    const glm::vec3 y = glm::cross(velocity, e);
    const f32 a = glm::dot(y, y);
    const f32 b = 2.0f * glm::dot(x, y);
    const f32 c = glm::dot(x, x) - radius * radius;

    if (c < 0.0f) // center inside the segment's cylinder
    {
        const f32 d = glm::dot(l, e);
        if (d < 0.0f)
            return sphereIntersectPoint(center, radius, velocity, v1, tMax, normal);
        if (d > eLen)
            return sphereIntersectPoint(center, radius, velocity, v2, tMax, normal);
        const glm::vec3 onEdge = v1 + e * d;
        normal = glm::normalize(center - onEdge);
        const f32 t = glm::length(center - onEdge) - radius;
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

    const glm::vec3 collCenter = center + velocity * t;
    const f32 d = glm::dot(collCenter - v1, e);
    if (d < 0.0f)
        return sphereIntersectPoint(center, radius, velocity, v1, tMax, normal);
    if (d > eLen)
        return sphereIntersectPoint(center, radius, velocity, v2, tMax, normal);

    normal = glm::normalize(collCenter - (v1 + e * d));
    tMax = t;
    return true;
}

// Swept sphere vs triangle; tMax is the running best and gets tightened on a
// hit. Out-normal is the contact normal.
bool sphereIntersectTriangle(const glm::vec3& center, f32 radius, const glm::vec3& velocity,
                             const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
                             const glm::vec3& normal, f32& tMax, glm::vec3& outNormal)
{
    f32 t = tMax;
    if (!sphereIntersectPlane(center, radius, velocity, normal, v0, t))
        return false;

    const glm::vec3 collCenter = (t < 0.0f) ? center - normal * t : center + velocity * t;
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

bool containsAABB(const AABB& outer, const AABB& inner)
{
    return inner.min.x >= outer.min.x && inner.min.y >= outer.min.y && inner.min.z >= outer.min.z &&
           inner.max.x <= outer.max.x && inner.max.y <= outer.max.y && inner.max.z <= outer.max.z;
}

} // namespace

// ---------------------------------------------------------------- lifecycle

void TriangleOctree::clear()
{
    mTriangles.clear();
    mNodes.clear();
    mBounds = AABB();
    mStats = Stats();
}

void TriangleOctree::addCollisionMesh(const CollisionMesh& mesh, const glm::mat4& transform)
{
    if (mesh.indices.size() < 3)
        return;
    const u32 triangleCount = static_cast<u32>(mesh.indices.size() / 3);
    mTriangles.reserve(mTriangles.size() + triangleCount);

    for (u32 t = 0; t < triangleCount; ++t)
    {
        const u32 base = t * 3;
        const u32 i0 = mesh.indices[base + 0];
        const u32 i1 = mesh.indices[base + 1];
        const u32 i2 = mesh.indices[base + 2];
        if (i0 >= mesh.positions.size() || i1 >= mesh.positions.size() ||
            i2 >= mesh.positions.size())
            continue;

        const glm::vec3 v0 = glm::vec3(transform * glm::vec4(mesh.positions[i0], 1.0f));
        const glm::vec3 v1 = glm::vec3(transform * glm::vec4(mesh.positions[i1], 1.0f));
        const glm::vec3 v2 = glm::vec3(transform * glm::vec4(mesh.positions[i2], 1.0f));

        const glm::vec3 e1 = v1 - v0;
        const glm::vec3 e2 = v2 - v0;
        const glm::vec3 normal = glm::normalize(glm::cross(e1, e2));
        // Skip degenerate (zero-area) triangles - they never collide and
        // would otherwise sit in the tree doing nothing but costing memory.
        if (!std::isfinite(normal.x) || !std::isfinite(normal.y) || !std::isfinite(normal.z))
            continue;

        Triangle tri;
        tri.v0 = v0;
        tri.v1 = v1;
        tri.v2 = v2;
        tri.normal = normal;
        tri.centroid = (v0 + v1 + v2) / 3.0f;
        tri.bounds = AABB();
        tri.bounds.expand(v0);
        tri.bounds.expand(v1);
        tri.bounds.expand(v2);
        tri.source = mSourceCount;
        tri.index = t;

        mBounds.expand(v0);
        mBounds.expand(v1);
        mBounds.expand(v2);
        mTriangles.push_back(tri);
    }
    ++mSourceCount;
}

void TriangleOctree::build(u32 maxDepth, u32 maxTriangles)
{
    mNodes.clear();
    mStats = Stats();
    mStats.triangleCount = static_cast<u32>(mTriangles.size());
    if (mTriangles.empty() || mBounds.empty())
        return;

    mMaxDepth = std::clamp(maxDepth, 1u, 16u);
    const u32 capped = std::max(1u, maxTriangles);

    mNodes.reserve(1u << 8u);
    const u32 root = createNode(mBounds, 0);
    for (u32 i = 0; i < mTriangles.size(); ++i)
        insertTriangle(i, root, 0, mMaxDepth, capped);

    mStats.nodeCount = static_cast<u32>(mNodes.size());
    for (const Node& node : mNodes)
        if (node.children[0] == kNoNode)
            ++mStats.leafCount;
}

// ------------------------------------------------------------------- build

u32 TriangleOctree::createNode(const AABB& bounds, u8 depth)
{
    Node node;
    node.bounds = bounds;
    node.depth = depth;
    mNodes.push_back(node);
    mStats.maxDepth = std::max<u32>(mStats.maxDepth, depth);
    return static_cast<u32>(mNodes.size() - 1);
}

bool TriangleOctree::nodeSplittable(const Node& node) const
{
    // A node with no extent (degenerate input) would subdivide forever into
    // empty boxes; depth alone can't stop that when the trigger is a triangle
    // count that never fits. Refuse to split a box that is effectively a point.
    const glm::vec3 ext = node.bounds.extents();
    return ext.x > 1e-4f && ext.y > 1e-4f && ext.z > 1e-4f;
}

void TriangleOctree::subdivide(u32 nodeIndex, u8 depth, u32 maxDepth, u32 maxTriangles)
{
    // Only ever split a node with no children yet. Splitting one that already
    // has children would overwrite their indices (orphaning every subtree
    // built so far) and lose their triangles. insertTriangle() guarantees
    // this is only called on a leaf; the guard is a cheap safety net.
    if (mNodes[nodeIndex].children[0] != kNoNode)
        return;

    const AABB bounds = mNodes[nodeIndex].bounds; // copy: createNode() may realloc
    const glm::vec3 c = bounds.center();

    for (u32 i = 0; i < 8; ++i)
    {
        AABB child;
        child.min = glm::vec3((i & 1u) ? c.x : bounds.min.x, (i & 2u) ? c.y : bounds.min.y,
                              (i & 4u) ? c.z : bounds.min.z);
        child.max = glm::vec3((i & 1u) ? bounds.max.x : c.x, (i & 2u) ? bounds.max.y : c.y,
                              (i & 4u) ? bounds.max.z : c.z);
        mNodes[nodeIndex].children[i] = createNode(child, static_cast<u8>(depth + 1));
    }

    std::vector<u32> tris = std::move(mNodes[nodeIndex].triangles);
    for (u32 tri : tris)
        insertTriangle(tri, nodeIndex, depth, maxDepth, maxTriangles);
}

void TriangleOctree::insertTriangle(u32 triangleIndex, u32 nodeIndex, u8 depth, u32 maxDepth,
                                    u32 maxTriangles)
{
    Node& node = mNodes[nodeIndex];
    const Triangle& tri = mTriangles[triangleIndex];

    // Only a leaf splits, and only when it overflows. A triangle that
    // straddles a split plane can never be pushed into a single child, so if
    // splitting a node that already has children were allowed, such a
    // triangle would overflow the node forever and subdivide() would recurse
    // without end (the two giant ground triangles in the demo cross every
    // plane - this is what used to hang the build).
    if (node.children[0] == kNoNode)
    {
        if (depth < maxDepth && nodeSplittable(node) && node.triangles.size() + 1 > maxTriangles)
        {
            subdivide(nodeIndex, depth, maxDepth, maxTriangles); // node is a leaf here
            // The split emptied node.triangles; place this one too.
            insertTriangle(triangleIndex, nodeIndex, depth, maxDepth, maxTriangles);
            return;
        }
        node.triangles.push_back(triangleIndex);
        return;
    }

    if (depth >= maxDepth)
    {
        node.triangles.push_back(triangleIndex);
        return;
    }

    // Recurse into the single child that fully contains the triangle; a
    // triangle straddling a split plane stays at this node. Storing it here
    // never re-splits this node, so it cannot loop.
    u32 fits = 0;
    u32 fitChild = kNoNode;
    for (u32 i = 0; i < 8; ++i)
    {
        const u32 child = node.children[i];
        if (child != kNoNode && containsAABB(mNodes[child].bounds, tri.bounds))
        {
            ++fits;
            fitChild = child;
        }
    }
    if (fits == 1)
    {
        insertTriangle(triangleIndex, fitChild, static_cast<u8>(depth + 1), maxDepth, maxTriangles);
        return;
    }

    node.triangles.push_back(triangleIndex);
}

// ----------------------------------------------------------------- queries

void TriangleOctree::collectInternal(u32 nodeIndex, const AABB& region, std::vector<u32>& out) const
{
    const Node& node = mNodes[nodeIndex];
    if (!node.bounds.intersects(region))
        return;
    for (u32 tri : node.triangles)
        if (mTriangles[tri].bounds.intersects(region))
            out.push_back(tri);
    for (u32 i = 0; i < 8; ++i)
        if (node.children[i] != kNoNode)
            collectInternal(node.children[i], region, out);
}

void TriangleOctree::collect(const AABB& region, std::vector<u32>& out) const
{
    if (mNodes.empty())
        return;
    collectInternal(0, region, out);
}

void TriangleOctree::raycastNode(u32 nodeIndex, const Ray& ray, RayHit& best, bool& hit) const
{
    const Node& node = mNodes[nodeIndex];
    f32 nodeT = 0.0f;
    if (!ray.intersects(node.bounds, nodeT))
        return;

    for (u32 triIndex : node.triangles)
    {
        const Triangle& tri = mTriangles[triIndex];
        f32 t = 0.0f;
        if (ray.intersects(tri.v0, tri.v1, tri.v2, t) && (!hit || t < best.t))
        {
            best.t = t;
            best.point = ray.at(t);
            best.normal = tri.normal;
            best.source = tri.source;
            best.triangle = tri.index;
            hit = true;
        }
    }

    for (u32 i = 0; i < 8; ++i)
        if (node.children[i] != kNoNode)
            raycastNode(node.children[i], ray, best, hit);
}

bool TriangleOctree::raycast(const Ray& ray, RayHit& out) const
{
    if (mNodes.empty())
        return false;
    bool hit = false;
    RayHit best;
    raycastNode(0, ray, best, hit);
    if (hit)
        out = best;
    return hit;
}

void TriangleOctree::sweepNode(u32 nodeIndex, const AABB& sweptBounds, const glm::vec3& center,
                               const glm::vec3& radii, const glm::vec3& invRadii,
                               const glm::vec3& velocity, f32& bestT, SweepHit& out, bool& hit,
                               u32& visited) const
{
    const Node& node = mNodes[nodeIndex];
    if (!node.bounds.intersects(sweptBounds))
        return;

    for (u32 triIndex : node.triangles)
    {
        const Triangle& tri = mTriangles[triIndex];
        if (!tri.bounds.intersects(sweptBounds))
            continue;
        ++visited;

        // Transform the triangle into ellipsoid space (scale by 1/radii); the
        // sweep then tests a unit sphere, exactly like CCollision does. The
        // normal scales by `radii` before normalizing - see the study.
        const glm::vec3 e0 = tri.v0 * invRadii;
        const glm::vec3 e1 = tri.v1 * invRadii;
        const glm::vec3 e2 = tri.v2 * invRadii;
        const glm::vec3 eNormal = glm::normalize(tri.normal * radii);
        const glm::vec3 eCenter = center * invRadii;
        const glm::vec3 eVelocity = velocity * invRadii;

        f32 t = bestT;
        glm::vec3 eHitNormal;
        if (sphereIntersectTriangle(eCenter, 1.0f, eVelocity, e0, e1, e2, eNormal, t, eHitNormal))
        {
            out.t = t;
            out.normal = glm::normalize(eHitNormal * invRadii);
            out.source = tri.source;
            out.triangle = tri.index;
            const glm::vec3 eCollCenter =
                (t < 0.0f) ? eCenter - eHitNormal * t : eCenter + eVelocity * t;
            out.point = (eCollCenter - eHitNormal) * radii;
            out.collided = true;
            bestT = t;
            hit = true;
        }
    }

    for (u32 i = 0; i < 8; ++i)
        if (node.children[i] != kNoNode)
            sweepNode(node.children[i], sweptBounds, center, radii, invRadii, velocity, bestT, out,
                      hit, visited);
}

bool TriangleOctree::sweepEllipsoid(const glm::vec3& center, const glm::vec3& radii,
                                    const glm::vec3& velocity, SweepHit& out) const
{
    if (mNodes.empty())
        return false;

    // The swept volume's bounding box: the start and end of the path, both
    // grown by the largest ellipsoid half-extent. Anything outside it cannot
    // be touched, so whole subtrees are skipped without ever testing a
    // triangle (this is the octree's reason to exist).
    const f32 maxR = glm::max(radii.x, glm::max(radii.y, radii.z));
    const glm::vec3 pad(maxR + 0.01f);
    AABB swept;
    swept.expand(center - pad);
    swept.expand(center + pad);
    swept.expand(center + velocity - pad);
    swept.expand(center + velocity + pad);

    const glm::vec3 invRadii(1.0f / radii.x, 1.0f / radii.y, 1.0f / radii.z);
    // Same as the physics TrimeshShape sweep: the running best starts at 1.0,
    // so a hit beyond the end of the swept segment (t > 1) is rejected. The
    // SweepHit contract says t is "a fraction of the query velocity in [0,1]",
    // and the reference (Nettle CollideAndSlide) treats nearestDistance >= 1
    // as no collision. Letting t reach infinity let a wall several units away
    // report t = 50 and teleport the character to it.
    f32 bestT = 1.0f;
    SweepHit best;
    bool hit = false;
    u32 visited = 0;
    sweepNode(0, swept, center, radii, invRadii, velocity, bestT, best, hit, visited);
    mStats.trianglesVisited = visited;
    if (hit)
        out = best;
    return hit;
}

bool TriangleOctree::sweepSphere(const glm::vec3& center, f32 radius, const glm::vec3& velocity,
                                 SweepHit& out) const
{
    return sweepEllipsoid(center, glm::vec3(radius), velocity, out);
}

// ------------------------------------------------------------------- debug

void TriangleOctree::drawDebug(DebugDraw3D& debug, u32 maxDepth, bool leavesOnly) const
{
    for (const Node& node : mNodes)
    {
        if (node.depth > maxDepth)
            continue;
        if (leavesOnly && node.children[0] != kNoNode)
            continue;
        debug.box(node.bounds, Color::Green);
    }
}

} // namespace Radion
