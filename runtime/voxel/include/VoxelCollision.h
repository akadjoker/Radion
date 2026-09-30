#ifndef RADION_VOXEL_COLLISION_H
#define RADION_VOXEL_COLLISION_H

#include "VoxelWorld.h"

#include "Math.h"

namespace Radion
{
namespace Voxel
{

struct VoxelMoveResult
{
    Math::vec3 position = Math::vec3(0.0f);
    bool grounded = false;
    bool ceiling = false;
    bool wall = false;
};

// Collision against the grid, not a mesh: the world changes whenever a block breaks, and a box against unit cubes is exact.
class VoxelCollision
{
public:
    // One axis at a time, vertical first (else a box crosses a floor seam and catches the edge); displacement is split
    // so no substep crosses a whole block, which stops fast falls passing through the ground.
    static VoxelMoveResult moveBox(const VoxelWorld& world, const BlockRegistry& blocks,
                                   const Math::vec3& position, const Math::vec3& halfExtents,
                                   const Math::vec3& displacement);

    static bool overlaps(const VoxelWorld& world, const BlockRegistry& blocks,
                         const Math::vec3& position, const Math::vec3& halfExtents);
    // True when the box is resting on something solid, tested just below it.
    static bool grounded(const VoxelWorld& world, const BlockRegistry& blocks,
                         const Math::vec3& position, const Math::vec3& halfExtents);
};

} // namespace Voxel
} // namespace Radion

#endif // RADION_VOXEL_COLLISION_H
