#ifndef RADION_AI_VOXELMAP_H
#define RADION_AI_VOXELMAP_H

// One bit per voxel, 64 packed into a u64 as a 4x4x4 block, so an empty block is one compare.

#include "Math.h"
#include "Types.h"

#include <vector>

namespace Radion::AI
{

class VoxelMap
{
public:
    // Resolution is clamped to a minimum of 4 per axis (one block).
    void create(u32 dimensionX, u32 dimensionY, u32 dimensionZ);
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
    void fromBounds(const AABB& box);

    Math::uvec3 worldToCoord(const Math::vec3& worldPosition) const;
    // No unsigned wrap, to tell "before the grid" from "past the end".
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

    // Subtract clears voxels instead.
    void injectTriangle(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                        bool subtract = false);
    void injectBounds(const AABB& box, bool subtract = false);
    void injectSphere(const Sphere& sphere, bool subtract = false);
    // base and tip are the poles, so the shape spans exactly base..tip.
    void injectCapsule(const Math::vec3& base, const Math::vec3& tip, f32 radius,
                       bool subtract = false);

    // Both grids must have the same resolution; a mismatch is ignored.
    void add(const VoxelMap& other);
    void subtract(const VoxelMap& other);

    void floodFill();

    // Stepped one voxel at a time; the subject's own voxel does not block.
    bool visible(const Math::uvec3& start, const Math::uvec3& goal) const;
    bool visible(const Math::vec3& observer, const Math::vec3& subject) const;
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
