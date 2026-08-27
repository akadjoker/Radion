#ifndef RADION_VOXEL_COLLISION_H
#define RADION_VOXEL_COLLISION_H

#include "VoxelWorld.h"

#include <glm/vec3.hpp>

namespace Radion
{
namespace Voxel
{

struct VoxelMoveResult
{
    glm::vec3 position = glm::vec3(0.0f);
    bool grounded = false;
    bool ceiling = false;
    bool wall = false;
};

// Collision against the grid itself rather than against a mesh of it. A voxel
// world changes every time somebody breaks a block, and any triangle
// structure would have to be rebuilt for it; the blocks are already the
// answer, and an axis-aligned box against unit cubes is exact.
class VoxelCollision
{
public:
    // Resolves one axis at a time, vertical first: a box crossing the seam
    // between two blocks of a flat floor would otherwise catch on the edge of
    // the second one. Displacement is split so no substep crosses a whole
    // block, which is what keeps a fast fall from passing through the ground.
    static VoxelMoveResult moveBox(const VoxelWorld& world, const BlockRegistry& blocks,
                                   const glm::vec3& position, const glm::vec3& halfExtents,
                                   const glm::vec3& displacement);

    static bool overlaps(const VoxelWorld& world, const BlockRegistry& blocks,
                         const glm::vec3& position, const glm::vec3& halfExtents);
    // True when the box is resting on something solid, tested just below it.
    static bool grounded(const VoxelWorld& world, const BlockRegistry& blocks,
                         const glm::vec3& position, const glm::vec3& halfExtents);
};

} // namespace Voxel
} // namespace Radion

#endif // RADION_VOXEL_COLLISION_H
