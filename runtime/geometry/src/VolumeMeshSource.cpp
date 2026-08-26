#include "PCH.h"

#include "VolumeMeshSource.h"

#include "Mesh.h"

namespace Radion::Volume
{

namespace
{

// Real-Time Collision Detection, 5.1.5: the closest point is found by which
// of the triangle's seven Voronoi regions the point falls in, rather than by
// projecting onto the plane and hoping the result lands inside.
Math::Vec3 closestPointOnTriangle(const Math::Vec3& p, const Math::Vec3& a, const Math::Vec3& b,
                                 const Math::Vec3& c)
{
    const Math::Vec3 ab = b - a;
    const Math::Vec3 ac = c - a;
    const Math::Vec3 ap = p - a;

    const f32 d1 = glm::dot(ab, ap);
    const f32 d2 = glm::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f)
        return a;

    const Math::Vec3 bp = p - b;
    const f32 d3 = glm::dot(ab, bp);
    const f32 d4 = glm::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3)
        return b;

    const f32 vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
        return a + ab * (d1 / (d1 - d3));

    const Math::Vec3 cp = p - c;
    const f32 d5 = glm::dot(ab, cp);
    const f32 d6 = glm::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6)
        return c;

    const f32 vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
        return a + ac * (d2 / (d2 - d6));

    const f32 va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));

    const f32 denominator = 1.0f / (va + vb + vc);
    return a + ab * (vb * denominator) + ac * (vc * denominator);
}

// Three directions that share no plane, so a ray grazing an edge in one of
// them is not grazing the same edge in the others.
const Math::Vec3 kParityDirections[3] = {
    Math::Vec3(0.5773502691896258f, 0.5773502691896258f, 0.5773502691896258f),
    Math::Vec3(-0.7071067811865475f, 0.3162277660168379f, 0.6324555320336759f),
    Math::Vec3(0.2672612419124244f, -0.8017837257372732f, 0.5345224838248488f),
};

} // namespace

MeshSource::MeshSource()
{
}

MeshSource::~MeshSource()
{
}

void MeshSource::clear()
{
    mCorners.clear();
    mTree.clear();
    mBounds = AABB();
    mDiagonal = 0.0f;
}

bool MeshSource::valid() const
{
    return !mCorners.empty() && mTree.valid();
}

u32 MeshSource::triangleCount() const
{
    return static_cast<u32>(mCorners.size() / 3);
}

bool MeshSource::build(const MeshData& mesh)
{
    clear();

    const usize vertexCount = mesh.positions.size();
    const usize faceCount = mesh.indices.size() / 3;
    if (vertexCount == 0 || faceCount == 0)
        return false;

    mCorners.reserve(faceCount * 3);
    std::vector<AABB> triangleBounds;
    triangleBounds.reserve(faceCount);

    for (usize face = 0; face < faceCount; ++face)
    {
        const u32 i0 = mesh.indices[face * 3 + 0];
        const u32 i1 = mesh.indices[face * 3 + 1];
        const u32 i2 = mesh.indices[face * 3 + 2];
        if (i0 >= vertexCount || i1 >= vertexCount || i2 >= vertexCount)
            continue;

        const Math::Vec3& a = mesh.positions[i0];
        const Math::Vec3& b = mesh.positions[i1];
        const Math::Vec3& c = mesh.positions[i2];

        AABB box;
        box.expand(a);
        box.expand(b);
        box.expand(c);
        triangleBounds.push_back(box);

        mCorners.push_back(a);
        mCorners.push_back(b);
        mCorners.push_back(c);

        mBounds.expand(a);
        mBounds.expand(b);
        mBounds.expand(c);
    }

    if (triangleBounds.empty())
    {
        clear();
        return false;
    }

    mTree.build(triangleBounds.data(), static_cast<u32>(triangleBounds.size()));
    mDiagonal = glm::length(mBounds.max - mBounds.min);
    return mTree.valid();
}

f32 MeshSource::unsignedDistance(const Math::Vec3& position) const
{
    // A sphere query returns every triangle whose box meets it, so a
    // candidate found within the radius cannot be beaten by one outside it.
    // When the sphere comes back empty, or holds nothing that close, it grows
    // and the query runs again - a handful of rounds at most, against
    // walking every triangle in the mesh.
    f32 radius = mDiagonal > 0.0f ? mDiagonal * 0.05f : 1.0f;
    const f32 limit = mDiagonal > 0.0f ? mDiagonal * 4.0f : 1e6f;

    for (;;)
    {
        Sphere sphere;
        sphere.center = position;
        sphere.radius = radius;

        mCandidates.clear();
        mTree.queryCandidates(sphere, mCandidates);

        f32 best = std::numeric_limits<f32>::max();
        for (usize i = 0; i < mCandidates.size(); ++i)
        {
            const usize base = static_cast<usize>(mCandidates[i]) * 3;
            const Math::Vec3 closest = closestPointOnTriangle(position, mCorners[base],
                                                             mCorners[base + 1],
                                                             mCorners[base + 2]);
            const f32 distance = glm::length(position - closest);
            if (distance < best)
                best = distance;
        }

        if (best <= radius)
            return best;

        if (radius >= limit)
            return best == std::numeric_limits<f32>::max() ? limit : best;

        // Jump straight to whatever the nearest candidate turned out to be
        // rather than doubling blindly past it.
        radius = best < std::numeric_limits<f32>::max() ? glm::min(best, limit)
                                                        : glm::min(radius * 2.0f, limit);
    }
}

bool MeshSource::isInside(const Math::Vec3& position) const
{
    if (!mBounds.contains(position))
        return false;

    const f32 reach = mDiagonal > 0.0f ? mDiagonal * 2.0f : 1e6f;
    u32 votesInside = 0;

    for (u32 d = 0; d < 3; ++d)
    {
        Ray ray;
        ray.origin = position;
        ray.direction = kParityDirections[d];

        mRayCandidates.clear();
        mTree.queryCandidates(ray, reach, mRayCandidates);

        u32 crossings = 0;
        for (usize i = 0; i < mRayCandidates.size(); ++i)
        {
            const usize base = static_cast<usize>(mRayCandidates[i]) * 3;
            f32 t = 0.0f;
            if (ray.intersects(mCorners[base], mCorners[base + 1], mCorners[base + 2], t) &&
                t <= reach)
                ++crossings;
        }

        if ((crossings & 1u) != 0u)
            ++votesInside;
    }

    return votesInside >= 2;
}

f32 MeshSource::sampleDensity(const Math::Vec3& position) const
{
    if (!valid())
        return -1.0f;

    const f32 distance = unsignedDistance(position);
    return isInside(position) ? distance : -distance;
}

} // namespace Radion::Volume
