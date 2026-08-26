// VoxelMap.cpp - a world-aligned grid of solid/empty voxels packed 64 to a
// 4x4x4 block, one bit per voxel.

#include "PCH.h"

#include "VoxelMap.h"

#include <cmath>

namespace Radion::AI
{

namespace
{

// 3D array index to flattened 1D array index.
u32 flatten3D(const glm::uvec3& coord, const glm::uvec3& dim)
{
    return coord.z * dim.x * dim.y + coord.y * dim.x + coord.x;
}

// Flattened array index to 3D array index.
glm::uvec3 unflatten3D(u32 idx, const glm::uvec3& dim)
{
    const u32 z = idx / (dim.x * dim.y);
    idx -= z * dim.x * dim.y;
    const u32 y = idx / dim.x;
    const u32 x = idx % dim.x;
    return glm::uvec3(x, y, z);
}

// The block index and bit mask for one voxel coordinate - the same
// addressing at every call site that touches the packed storage.
void voxelAddress(const glm::uvec3& coord, const glm::uvec3& resolutionDiv4,
                  u32& outIndex, u64& outMask)
{
    const glm::uvec3 macroCoord = coord / 4u;
    const glm::uvec3 subCoord = coord % 4u;
    outIndex = flatten3D(macroCoord, resolutionDiv4);
    const u32 bit = flatten3D(subCoord, glm::uvec3(4, 4, 4));
    outMask = 1ull << bit;
}

glm::vec3 worldToUvw(const glm::vec3& worldPos, const glm::vec3& center,
                     const glm::vec3& resolutionRcp, const glm::vec3& voxelSizeRcp)
{
    const glm::vec3 diff = (worldPos - center) * resolutionRcp * voxelSizeRcp;
    return diff * glm::vec3(0.5f, -0.5f, 0.5f) + glm::vec3(0.5f);
}

glm::vec3 uvwToWorld(const glm::vec3& uvw, const glm::vec3& center,
                     const glm::vec3& resolution, const glm::vec3& voxelSize)
{
    glm::vec3 pos = uvw * 2.0f - glm::vec3(1.0f);
    pos *= glm::vec3(1.0f, -1.0f, 1.0f);
    pos *= voxelSize;
    pos *= resolution;
    pos += center;
    return pos;
}

// Shared tail of every inject_* preamble: two pixel-space corners (order not
// guaranteed - the coordinate flip in worldToUvw can swap min and max) turned
// into an inclusive-exclusive voxel coordinate range, clamped to the grid.
void clampVoxelRange(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& resolution,
                     glm::uvec3& outMin, glm::uvec3& outMax)
{
    glm::vec3 rangeMin = glm::min(p0, p1);
    glm::vec3 rangeMax = glm::max(p0, p1);

    rangeMin = glm::floor(rangeMin);
    rangeMax = glm::ceil(rangeMax + glm::vec3(0.0001f));

    rangeMin = glm::max(rangeMin, glm::vec3(0.0f));
    rangeMax = glm::min(rangeMax, resolution);

    outMin = glm::uvec3(rangeMin);
    outMax = glm::uvec3(rangeMax);
}

// Triangle-vs-AABB separating axis test: 3 box axes, 1 triangle normal, and
// the 9 cross products of a box axis with a triangle edge.
bool triangleIntersectsAABB(const glm::vec3& boxCenter, const glm::vec3& boxHalfExtent,
                            const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    const glm::vec3 p0 = v0 - boxCenter;
    const glm::vec3 p1 = v1 - boxCenter;
    const glm::vec3 p2 = v2 - boxCenter;

    const glm::vec3 e0 = p1 - p0;
    const glm::vec3 e1 = p2 - p1;
    const glm::vec3 e2 = p0 - p2;

    const glm::vec3 edges[3] = {e0, e1, e2};
    const glm::vec3 boxAxes[3] = {glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f),
                                  glm::vec3(0.0f, 0.0f, 1.0f)};

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            const glm::vec3 axis = glm::cross(boxAxes[i], edges[j]);
            const f32 r = boxHalfExtent.x * std::abs(axis.x) +
                         boxHalfExtent.y * std::abs(axis.y) +
                         boxHalfExtent.z * std::abs(axis.z);

            const f32 d0 = glm::dot(p0, axis);
            const f32 d1 = glm::dot(p1, axis);
            const f32 d2 = glm::dot(p2, axis);

            const f32 projMin = std::min(d0, std::min(d1, d2));
            const f32 projMax = std::max(d0, std::max(d1, d2));
            if (projMin > r || projMax < -r)
                return false;
        }
    }

    for (int axis = 0; axis < 3; ++axis)
    {
        const f32 projMin = std::min(p0[axis], std::min(p1[axis], p2[axis]));
        const f32 projMax = std::max(p0[axis], std::max(p1[axis], p2[axis]));
        if (projMin > boxHalfExtent[axis] || projMax < -boxHalfExtent[axis])
            return false;
    }

    const glm::vec3 normal = glm::cross(e0, e1);
    const f32 r = boxHalfExtent.x * std::abs(normal.x) +
                 boxHalfExtent.y * std::abs(normal.y) +
                 boxHalfExtent.z * std::abs(normal.z);
    const f32 distance = glm::dot(normal, p0);
    if (std::abs(distance) > r)
        return false;

    return true;
}

glm::vec3 closestPointOnSegment(const glm::vec3& a, const glm::vec3& b, const glm::vec3& point)
{
    const glm::vec3 ab = b - a;
    const f32 lengthSq = glm::dot(ab, ab);
    if (lengthSq < Epsilon)
        return a;
    f32 t = glm::dot(point - a, ab) / lengthSq;
    t = glm::clamp(t, 0.0f, 1.0f);
    return a + ab * t;
}

// base and tip are the capsule's poles, not the centres of its end spheres:
// the segment shrinks by one radius at each end before the distance test, so
// the whole shape spans exactly base..tip. A degenerate axis leaves the
// segment as it is - normalising a zero vector is what the shrink cannot do.
bool pointInCapsule(const glm::vec3& point, const glm::vec3& base, const glm::vec3& tip,
                    f32 radius)
{
    glm::vec3 a = base;
    glm::vec3 b = tip;
    const glm::vec3 axis = tip - base;
    if (glm::dot(axis, axis) >= Epsilon)
    {
        const glm::vec3 offset = glm::normalize(axis) * radius;
        a = base + offset;
        b = tip - offset;
    }

    const glm::vec3 closest = closestPointOnSegment(a, b, point);
    const glm::vec3 delta = point - closest;
    return glm::dot(delta, delta) <= radius * radius;
}

void aabbCorners(const glm::vec3& center, const glm::vec3& halfExtent, glm::vec3 outCorners[8])
{
    outCorners[0] = center + glm::vec3(-halfExtent.x, -halfExtent.y, -halfExtent.z);
    outCorners[1] = center + glm::vec3(-halfExtent.x, halfExtent.y, -halfExtent.z);
    outCorners[2] = center + glm::vec3(-halfExtent.x, halfExtent.y, halfExtent.z);
    outCorners[3] = center + glm::vec3(-halfExtent.x, -halfExtent.y, halfExtent.z);
    outCorners[4] = center + glm::vec3(halfExtent.x, -halfExtent.y, -halfExtent.z);
    outCorners[5] = center + glm::vec3(halfExtent.x, halfExtent.y, -halfExtent.z);
    outCorners[6] = center + glm::vec3(halfExtent.x, halfExtent.y, halfExtent.z);
    outCorners[7] = center + glm::vec3(halfExtent.x, -halfExtent.y, halfExtent.z);
}

} // namespace

void VoxelMap::create(u32 dimensionX, u32 dimensionY, u32 dimensionZ)
{
    mResolution.x = std::max(4u, dimensionX);
    mResolution.y = std::max(4u, dimensionY);
    mResolution.z = std::max(4u, dimensionZ);

    mResolutionDiv4.x = (mResolution.x + 3u) / 4u;
    mResolutionDiv4.y = (mResolution.y + 3u) / 4u;
    mResolutionDiv4.z = (mResolution.z + 3u) / 4u;

    mResolutionRcp.x = 1.0f / static_cast<f32>(mResolution.x);
    mResolutionRcp.y = 1.0f / static_cast<f32>(mResolution.y);
    mResolutionRcp.z = 1.0f / static_cast<f32>(mResolution.z);

    mVoxels.clear();
    mVoxels.resize(static_cast<usize>(mResolutionDiv4.x) * mResolutionDiv4.y * mResolutionDiv4.z);
}

void VoxelMap::clearVoxels()
{
    std::fill(mVoxels.begin(), mVoxels.end(), 0ull);
}

bool VoxelMap::valid() const
{
    return !mVoxels.empty();
}

const glm::uvec3& VoxelMap::resolution() const
{
    return mResolution;
}

const glm::vec3& VoxelMap::center() const
{
    return mCenter;
}

void VoxelMap::setCenter(const glm::vec3& center)
{
    mCenter = center;
}

const glm::vec3& VoxelMap::voxelSize() const
{
    return mVoxelSize;
}

void VoxelMap::setVoxelSize(f32 size)
{
    setVoxelSize(glm::vec3(size, size, size));
}

void VoxelMap::setVoxelSize(const glm::vec3& size)
{
    mVoxelSize = size;
    mVoxelSizeRcp.x = 1.0f / mVoxelSize.x;
    mVoxelSizeRcp.y = 1.0f / mVoxelSize.y;
    mVoxelSizeRcp.z = 1.0f / mVoxelSize.z;
}

usize VoxelMap::memorySize() const
{
    return mVoxels.size() * sizeof(u64);
}

AABB VoxelMap::bounds() const
{
    AABB box;
    const glm::vec3 halfWidth = glm::vec3(mResolution) * mVoxelSize;
    box.min = mCenter - halfWidth;
    box.max = mCenter + halfWidth;
    return box;
}

void VoxelMap::fromBounds(const AABB& box)
{
    mCenter = box.center();
    const glm::vec3 halfWidth = box.extents();
    setVoxelSize(glm::vec3(halfWidth.x / static_cast<f32>(mResolution.x),
                          halfWidth.y / static_cast<f32>(mResolution.y),
                          halfWidth.z / static_cast<f32>(mResolution.z)));
}

glm::uvec3 VoxelMap::worldToCoord(const glm::vec3& worldPosition) const
{
    const glm::vec3 uvw = worldToUvw(worldPosition, mCenter, mResolutionRcp, mVoxelSizeRcp);
    return glm::uvec3(uvw * glm::vec3(mResolution));
}

glm::ivec3 VoxelMap::worldToCoordSigned(const glm::vec3& worldPosition) const
{
    const glm::vec3 uvw = worldToUvw(worldPosition, mCenter, mResolutionRcp, mVoxelSizeRcp);
    return glm::ivec3(uvw * glm::vec3(mResolution));
}

glm::vec3 VoxelMap::coordToWorld(const glm::uvec3& coord) const
{
    const glm::vec3 uvw = (glm::vec3(coord) + glm::vec3(0.5f)) * mResolutionRcp;
    return uvwToWorld(uvw, mCenter, glm::vec3(mResolution), mVoxelSize);
}

glm::vec3 VoxelMap::coordToWorld(const glm::ivec3& coord) const
{
    const glm::vec3 uvw = (glm::vec3(coord) + glm::vec3(0.5f)) * mResolutionRcp;
    return uvwToWorld(uvw, mCenter, glm::vec3(mResolution), mVoxelSize);
}

bool VoxelMap::validCoord(const glm::uvec3& coord) const
{
    return coord.x < mResolution.x && coord.y < mResolution.y && coord.z < mResolution.z;
}

bool VoxelMap::validCoord(const glm::ivec3& coord) const
{
    return validCoord(glm::uvec3(static_cast<u32>(coord.x), static_cast<u32>(coord.y),
                                 static_cast<u32>(coord.z)));
}

bool VoxelMap::voxel(const glm::uvec3& coord) const
{
    if (!validCoord(coord))
        return false; // outside of resolution

    const glm::uvec3 macroCoord = coord / 4u;
    const u32 idx = flatten3D(macroCoord, mResolutionDiv4);
    const u64 block = mVoxels[idx];
    if (block == 0)
        return false; // whole block is empty

    const glm::uvec3 subCoord = coord % 4u;
    const u32 bit = flatten3D(subCoord, glm::uvec3(4, 4, 4));
    const u64 mask = 1ull << bit;
    return (block & mask) != 0ull;
}

bool VoxelMap::voxel(const glm::ivec3& coord) const
{
    return voxel(glm::uvec3(static_cast<u32>(coord.x), static_cast<u32>(coord.y),
                            static_cast<u32>(coord.z)));
}

bool VoxelMap::voxel(const glm::vec3& worldPosition) const
{
    return voxel(worldToCoord(worldPosition));
}

void VoxelMap::setVoxel(const glm::uvec3& coord, bool value)
{
    if (!validCoord(coord))
        return;

    u32 index;
    u64 mask;
    voxelAddress(coord, mResolutionDiv4, index, mask);
    if (value)
        mVoxels[index] |= mask;
    else
        mVoxels[index] &= ~mask;
}

void VoxelMap::setVoxel(const glm::ivec3& coord, bool value)
{
    setVoxel(glm::uvec3(static_cast<u32>(coord.x), static_cast<u32>(coord.y),
                        static_cast<u32>(coord.z)),
             value);
}

void VoxelMap::setVoxel(const glm::vec3& worldPosition, bool value)
{
    setVoxel(worldToCoord(worldPosition), value);
}

void VoxelMap::injectTriangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                              bool subtract)
{
    const glm::vec3 resolutionF = glm::vec3(mResolution);

    const glm::vec3 pa = worldToUvw(a, mCenter, mResolutionRcp, mVoxelSizeRcp) * resolutionF;
    const glm::vec3 pb = worldToUvw(b, mCenter, mResolutionRcp, mVoxelSizeRcp) * resolutionF;
    const glm::vec3 pc = worldToUvw(c, mCenter, mResolutionRcp, mVoxelSizeRcp) * resolutionF;

    const glm::vec3 normal = glm::cross(pb - pa, pc - pa);
    if (normal == glm::vec3(0.0f))
        return;

    glm::uvec3 mini, maxi;
    clampVoxelRange(glm::min(pa, glm::min(pb, pc)), glm::max(pa, glm::max(pb, pc)), resolutionF,
                    mini, maxi);

    for (u32 x = mini.x; x < maxi.x; ++x)
    {
        for (u32 y = mini.y; y < maxi.y; ++y)
        {
            for (u32 z = mini.z; z < maxi.z; ++z)
            {
                const glm::vec3 voxelCenter(static_cast<f32>(x) + 0.5f, static_cast<f32>(y) + 0.5f,
                                            static_cast<f32>(z) + 0.5f);
                if (!triangleIntersectsAABB(voxelCenter, glm::vec3(0.5f), pa, pb, pc))
                    continue;

                u32 index;
                u64 mask;
                voxelAddress(glm::uvec3(x, y, z), mResolutionDiv4, index, mask);
                // Single-threaded here, so a plain OR/AND is enough.
                if (subtract)
                    mVoxels[index] &= ~mask;
                else
                    mVoxels[index] |= mask;
            }
        }
    }
}

void VoxelMap::injectBounds(const AABB& box, bool subtract)
{
    const glm::vec3 resolutionF = glm::vec3(mResolution);

    const glm::vec3 pixelMin = worldToUvw(box.min, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;
    const glm::vec3 pixelMax = worldToUvw(box.max, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;

    glm::uvec3 mini, maxi;
    clampVoxelRange(pixelMin, pixelMax, resolutionF, mini, maxi);

    for (u32 x = mini.x; x < maxi.x; ++x)
    {
        for (u32 y = mini.y; y < maxi.y; ++y)
        {
            for (u32 z = mini.z; z < maxi.z; ++z)
            {
                u32 index;
                u64 mask;
                voxelAddress(glm::uvec3(x, y, z), mResolutionDiv4, index, mask);
                if (subtract)
                    mVoxels[index] &= ~mask;
                else
                    mVoxels[index] |= mask;
            }
        }
    }
}

void VoxelMap::injectSphere(const Sphere& sphere, bool subtract)
{
    const glm::vec3 resolutionF = glm::vec3(mResolution);

    AABB box;
    box.min = sphere.center - glm::vec3(sphere.radius);
    box.max = sphere.center + glm::vec3(sphere.radius);

    const glm::vec3 pixelMin = worldToUvw(box.min, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;
    const glm::vec3 pixelMax = worldToUvw(box.max, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;

    glm::uvec3 mini, maxi;
    clampVoxelRange(pixelMin, pixelMax, resolutionF, mini, maxi);

    for (u32 x = mini.x; x < maxi.x; ++x)
    {
        for (u32 y = mini.y; y < maxi.y; ++y)
        {
            for (u32 z = mini.z; z < maxi.z; ++z)
            {
                const glm::uvec3 coord(x, y, z);
                const glm::vec3 voxelCenter = coordToWorld(coord);

                AABB voxelBox;
                voxelBox.min = voxelCenter - mVoxelSize;
                voxelBox.max = voxelCenter + mVoxelSize;
                if (!sphere.intersects(voxelBox))
                    continue;

                u32 index;
                u64 mask;
                voxelAddress(coord, mResolutionDiv4, index, mask);
                if (subtract)
                    mVoxels[index] &= ~mask;
                else
                    mVoxels[index] |= mask;
            }
        }
    }
}

void VoxelMap::injectCapsule(const glm::vec3& base, const glm::vec3& tip, f32 radius,
                             bool subtract)
{
    const glm::vec3 resolutionF = glm::vec3(mResolution);

    AABB box;
    box.expand(base);
    box.expand(tip);
    box.min -= glm::vec3(radius);
    box.max += glm::vec3(radius);

    const glm::vec3 pixelMin = worldToUvw(box.min, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;
    const glm::vec3 pixelMax = worldToUvw(box.max, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;

    glm::uvec3 mini, maxi;
    clampVoxelRange(pixelMin, pixelMax, resolutionF, mini, maxi);

    for (u32 x = mini.x; x < maxi.x; ++x)
    {
        for (u32 y = mini.y; y < maxi.y; ++y)
        {
            for (u32 z = mini.z; z < maxi.z; ++z)
            {
                const glm::uvec3 coord(x, y, z);
                const glm::vec3 voxelCenter = coordToWorld(coord);

                bool intersects = pointInCapsule(voxelCenter, base, tip, radius);
                if (!intersects)
                {
                    glm::vec3 corners[8];
                    aabbCorners(voxelCenter, mVoxelSize, corners);
                    for (int i = 0; i < 8 && !intersects; ++i)
                        intersects = pointInCapsule(corners[i], base, tip, radius);
                }
                if (!intersects)
                    continue;

                u32 index;
                u64 mask;
                voxelAddress(coord, mResolutionDiv4, index, mask);
                if (subtract)
                    mVoxels[index] &= ~mask;
                else
                    mVoxels[index] |= mask;
            }
        }
    }
}

void VoxelMap::add(const VoxelMap& other)
{
    if (mVoxels.size() != other.mVoxels.size())
        return;
    for (usize i = 0; i < mVoxels.size(); ++i)
        mVoxels[i] |= other.mVoxels[i];
}

void VoxelMap::subtract(const VoxelMap& other)
{
    if (mVoxels.size() != other.mVoxels.size())
        return;
    for (usize i = 0; i < mVoxels.size(); ++i)
        mVoxels[i] &= ~other.mVoxels[i];
}

void VoxelMap::floodFill()
{
    VoxelMap traversed;
    traversed.create(mResolution.x, mResolution.y, mResolution.z);
    std::vector<glm::ivec3> stack;

    for (usize i = 0; i < mVoxels.size(); ++i)
    {
        if (mVoxels[i] == ~0ull)
            continue; // whole block is filled already

        const glm::uvec3 coord = unflatten3D(static_cast<u32>(i), mResolutionDiv4);
        for (u32 bit = 0; bit < 64; ++bit)
        {
            const glm::uvec3 subCoord = unflatten3D(bit, glm::uvec3(4, 4, 4));
            const glm::ivec3 origin(static_cast<s32>(coord.x * 4u + subCoord.x),
                                    static_cast<s32>(coord.y * 4u + subCoord.y),
                                    static_cast<s32>(coord.z * 4u + subCoord.z));
            if (voxel(origin))
                continue; // voxel is filled, abort

            traversed.clearVoxels();
            stack.clear();

            stack.push_back(origin);
            bool exit = false;

            do
            {
                const glm::ivec3 current = stack.back();
                stack.pop_back();
                traversed.setVoxel(current, true);

                const glm::ivec3 neighbors[6] = {
                    glm::ivec3(current.x - 1, current.y, current.z),
                    glm::ivec3(current.x + 1, current.y, current.z),
                    glm::ivec3(current.x, current.y - 1, current.z),
                    glm::ivec3(current.x, current.y + 1, current.z),
                    glm::ivec3(current.x, current.y, current.z - 1),
                    glm::ivec3(current.x, current.y, current.z + 1),
                };
                for (const glm::ivec3& neighbor : neighbors)
                {
                    if (!validCoord(neighbor))
                    {
                        exit = true; // got out of the voxel grid, origin cannot be filled
                        break;
                    }
                    if (traversed.voxel(neighbor))
                        continue; // don't go to a previously traversed voxel again
                    if (!voxel(neighbor))
                        stack.push_back(neighbor); // empty neighbor, keep traversing
                }
            } while (!stack.empty() && !exit);

            if (!exit)
                setVoxel(origin, true); // no exit found, mark voxel as solid
        }
    }
}

bool VoxelMap::visible(const glm::uvec3& start, const glm::uvec3& goal) const
{
    const s32 dx = static_cast<s32>(goal.x) - static_cast<s32>(start.x);
    const s32 dy = static_cast<s32>(goal.y) - static_cast<s32>(start.y);
    const s32 dz = static_cast<s32>(goal.z) - static_cast<s32>(start.z);

    const s32 adx = dx < 0 ? -dx : dx;
    const s32 ady = dy < 0 ? -dy : dy;
    const s32 adz = dz < 0 ? -dz : dz;
    const s32 step = std::max(adx, std::max(ady, adz));
    if (step == 0)
        return true; // start and goal coincide, avoid dividing by zero below

    const f32 stepX = static_cast<f32>(dx) / static_cast<f32>(step);
    const f32 stepY = static_cast<f32>(dy) / static_cast<f32>(step);
    const f32 stepZ = static_cast<f32>(dz) / static_cast<f32>(step);

    f32 x = static_cast<f32>(start.x);
    f32 y = static_cast<f32>(start.y);
    f32 z = static_cast<f32>(start.z);

    for (s32 i = 0; i < step; ++i)
    {
        const glm::uvec3 coord(static_cast<u32>(std::round(x)), static_cast<u32>(std::round(y)),
                               static_cast<u32>(std::round(z)));
        if (coord == goal)
            return true;
        if (voxel(coord))
            return false;

        x += stepX;
        y += stepY;
        z += stepZ;
    }
    return true;
}

bool VoxelMap::visible(const glm::vec3& observer, const glm::vec3& subject) const
{
    const glm::uvec3 start = worldToCoord(observer);
    const glm::uvec3 goal = worldToCoord(subject);
    return visible(start, goal);
}

bool VoxelMap::visible(const glm::vec3& observer, const AABB& subject) const
{
    const glm::uvec3 start = worldToCoord(observer);

    const glm::vec3 resolutionF = glm::vec3(mResolution);
    const glm::vec3 pixelMin = worldToUvw(subject.min, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;
    const glm::vec3 pixelMax = worldToUvw(subject.max, mCenter, mResolutionRcp, mVoxelSizeRcp) *
                               resolutionF;

    glm::uvec3 mini, maxi;
    clampVoxelRange(pixelMin, pixelMax, resolutionF, mini, maxi);

    for (u32 x = mini.x; x < maxi.x; ++x)
    {
        for (u32 y = mini.y; y < maxi.y; ++y)
        {
            for (u32 z = mini.z; z < maxi.z; ++z)
            {
                if (visible(start, glm::uvec3(x, y, z)))
                    return true;
            }
        }
    }
    return false;
}

} // namespace Radion::AI
