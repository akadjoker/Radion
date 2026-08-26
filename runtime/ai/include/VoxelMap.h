#ifndef RADION_AI_VOXELMAP_H
#define RADION_AI_VOXELMAP_H

// VoxelMap.h - a world-aligned grid of solid/empty voxels.
//
// The 3D counterpart of GridMap: that one carries a traversal cost per 2D
// cell, this one carries occupancy per volume element - one bit each, 64 of
// them packed into a u64 as a 4x4x4 block, so an empty block is one integer
// compare instead of 64 tests. What it buys over a navmesh is the two things
// a surface cannot answer: whether a volume is free (flying, swimming) and
// whether one point can see another through the world.

#include "Math.h"
#include "Types.h"

#include <vector>

namespace Radion::AI
{

class VoxelMap
{
public:
    // Resolution is clamped to a minimum of 4 on every axis - one block.
    void create(u32 dimensionX, u32 dimensionY, u32 dimensionZ);
    // Zeroes every voxel, keeping the allocation and the resolution.
    void clearVoxels();
    bool valid() const;

    const Math::uvec3& resolution() const;
    const Math::vec3& center() const;
    void setCenter(const Math::vec3& center);
    const Math::vec3& voxelSize() const;
    void setVoxelSize(f32 size);
    void setVoxelSize(const Math::vec3& size);
    usize memorySize() const;

    AABB bounds() const;
    // Centres the grid on the box and sizes the voxels so the current
    // resolution spans it. Does not resize the grid.
    void fromBounds(const AABB& box);

    Math::uvec3 worldToCoord(const Math::vec3& worldPosition) const;
    // The same mapping without the unsigned wrap, for callers that have to
    // tell "before the grid" from "past the end of it".
    Math::ivec3 worldToCoordSigned(const Math::vec3& worldPosition) const;
    Math::vec3 coordToWorld(const Math::uvec3& coord) const;
    Math::vec3 coordToWorld(const Math::ivec3& coord) const;

    bool validCoord(const Math::uvec3& coord) const;
    bool validCoord(const Math::ivec3& coord) const;
    bool voxel(const Math::uvec3& coord) const;
    bool voxel(const Math::ivec3& coord) const;
    bool voxel(const Math::vec3& worldPosition) const;
    void setVoxel(const Math::uvec3& coord, bool value);
    void setVoxel(const Math::ivec3& coord, bool value);
    void setVoxel(const Math::vec3& worldPosition, bool value);

    // Marks every voxel the shape touches. Subtract clears them instead, so
    // the same call carves a doorway out of a wall already injected.
    void injectTriangle(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                        bool subtract = false);
    void injectBounds(const AABB& box, bool subtract = false);
    void injectSphere(const Sphere& sphere, bool subtract = false);
    // No Capsule type in Math.h, so the segment and radius are passed
    // directly. base and tip are the poles, so the shape spans exactly
    // base..tip - not the centres of the end spheres, which would make it a
    // radius longer at each end.
    void injectCapsule(const Math::vec3& base, const Math::vec3& tip, f32 radius,
                       bool subtract = false);

    // Both grids must have the same resolution; a mismatch is ignored.
    void add(const VoxelMap& other);
    void subtract(const VoxelMap& other);

    // Fills every void that does not reach the outside, so a closed building
    // becomes solid instead of a shell with navigable air inside it.
    void floodFill();

    // Line of sight, stepped one voxel at a time. The subject's own voxel
    // does not block: reaching it is what "visible" means.
    bool visible(const Math::uvec3& start, const Math::uvec3& goal) const;
    bool visible(const Math::vec3& observer, const Math::vec3& subject) const;
    // True as soon as any voxel of the box is reachable, so a target is seen
    // when any part of it is - not only its centre.
    bool visible(const Math::vec3& observer, const AABB& subject) const;

private:
    Math::uvec3 mResolution = Math::uvec3(0);
    Math::uvec3 mResolutionDiv4 = Math::uvec3(0);
    Math::vec3 mResolutionRcp = Math::vec3(0.0f);
    Math::vec3 mCenter = Math::vec3(0.0f);
    Math::vec3 mVoxelSize = Math::vec3(0.25f);
    Math::vec3 mVoxelSizeRcp = Math::vec3(4.0f);
    std::vector<u64> mVoxels; // one element is 4 * 4 * 4 = 64 voxels
};

} // namespace Radion::AI

#endif // RADION_AI_VOXELMAP_H
