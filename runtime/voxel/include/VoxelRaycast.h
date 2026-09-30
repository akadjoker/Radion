#ifndef RADION_VOXEL_RAYCAST_H
#define RADION_VOXEL_RAYCAST_H

#include "VoxelWorld.h"

#include "Math.h"

namespace Radion
{
namespace Voxel
{

struct VoxelRaycastHit
{
    VoxelCoord block;
    VoxelCoord previousBlock;
    BlockFace face = BlockFace::Count;
    f32 distance = 0.0f;
};

// Traverses the voxel grid and returns the first solid block along the ray.
// Transparent or non-solid blocks do not stop the traversal.
bool raycast(const VoxelWorld& world, const BlockRegistry& blocks, Math::vec3 origin,
             Math::vec3 direction, f32 maxDistance, VoxelRaycastHit& hit);

} // namespace Voxel
} // namespace Radion

#endif // RADION_VOXEL_RAYCAST_H