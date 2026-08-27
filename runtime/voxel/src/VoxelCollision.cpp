#include "VoxelCollision.h"

#include <algorithm>
#include <cmath>

namespace Radion
{
namespace Voxel
{
namespace
{
// Kept off the surface by this much after a contact, so the next test does
// not find the box already touching what it just landed on.
constexpr f32 Skin = 0.001f;
// No substep crosses a whole block, whatever the frame time was.
constexpr f32 MaxStep = 0.4f;

bool blocksMovement(const BlockRegistry& blocks, BlockId id)
{
    if (id == AirBlockId)
        return false;
    const BlockDefinition* definition = blocks.find(id);
    return definition && definition->solid;
}

bool boxOverlapsSolid(const VoxelWorld& world, const BlockRegistry& blocks, const glm::vec3& min,
                      const glm::vec3& max)
{
    const s32 firstX = static_cast<s32>(std::floor(min.x));
    const s32 lastX = static_cast<s32>(std::floor(max.x));
    const s32 firstY = static_cast<s32>(std::floor(min.y));
    const s32 lastY = static_cast<s32>(std::floor(max.y));
    const s32 firstZ = static_cast<s32>(std::floor(min.z));
    const s32 lastZ = static_cast<s32>(std::floor(max.z));

    for (s32 z = firstZ; z <= lastZ; ++z)
        for (s32 y = firstY; y <= lastY; ++y)
            for (s32 x = firstX; x <= lastX; ++x)
                if (blocksMovement(blocks, world.block({x, y, z})))
                    return true;
    return false;
}

// One axis of one substep. `axis` indexes the vector; the box is moved and, if
// that put it inside a block, snapped back to the boundary it just crossed.
// Unit cubes on integer coordinates are what make the snap a floor/ceil rather
// than a search.
bool resolveAxis(const VoxelWorld& world, const BlockRegistry& blocks, glm::vec3& position,
                 const glm::vec3& halfExtents, f32 delta, int axis)
{
    if (delta == 0.0f)
        return false;

    position[axis] += delta;
    glm::vec3 min = position - halfExtents;
    glm::vec3 max = position + halfExtents;
    if (!boxOverlapsSolid(world, blocks, min, max))
        return false;

    if (delta > 0.0f)
    {
        const f32 boundary = std::floor(max[axis]);
        position[axis] = boundary - halfExtents[axis] - Skin;
    }
    else
    {
        const f32 boundary = std::ceil(min[axis]);
        position[axis] = boundary + halfExtents[axis] + Skin;
    }
    return true;
}
} // namespace

bool VoxelCollision::overlaps(const VoxelWorld& world, const BlockRegistry& blocks,
                              const glm::vec3& position, const glm::vec3& halfExtents)
{
    return boxOverlapsSolid(world, blocks, position - halfExtents, position + halfExtents);
}

bool VoxelCollision::grounded(const VoxelWorld& world, const BlockRegistry& blocks,
                              const glm::vec3& position, const glm::vec3& halfExtents)
{
    glm::vec3 min = position - halfExtents;
    glm::vec3 max = position + halfExtents;
    max.y = min.y;
    min.y -= 2.0f * Skin;
    return boxOverlapsSolid(world, blocks, min, max);
}

VoxelMoveResult VoxelCollision::moveBox(const VoxelWorld& world, const BlockRegistry& blocks,
                                        const glm::vec3& position, const glm::vec3& halfExtents,
                                        const glm::vec3& displacement)
{
    VoxelMoveResult result;
    result.position = position;

    const f32 longest = std::max(std::abs(displacement.x),
                                 std::max(std::abs(displacement.y), std::abs(displacement.z)));
    const s32 steps = std::max(1, static_cast<s32>(std::ceil(longest / MaxStep)));
    const glm::vec3 step = displacement / static_cast<f32>(steps);

    for (s32 i = 0; i < steps; ++i)
    {
        if (resolveAxis(world, blocks, result.position, halfExtents, step.y, 1))
        {
            if (step.y < 0.0f)
                result.grounded = true;
            else
                result.ceiling = true;
        }
        if (resolveAxis(world, blocks, result.position, halfExtents, step.x, 0))
            result.wall = true;
        if (resolveAxis(world, blocks, result.position, halfExtents, step.z, 2))
            result.wall = true;
    }

    if (!result.grounded)
        result.grounded = grounded(world, blocks, result.position, halfExtents);
    return result;
}

} // namespace Voxel
} // namespace Radion
