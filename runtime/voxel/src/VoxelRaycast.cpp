#include "VoxelRaycast.h"

#include <cmath>

namespace Radion
{
namespace Voxel
{
namespace
{
struct AxisStep
{
    s32 direction = 0;
    f32 nextBoundary = 0.0f;
    f32 delta = 0.0f;
};

AxisStep makeAxisStep(f32 position, f32 direction)
{
    if (direction > 0.0f)
        return {1, (std::floor(position) + 1.0f - position) / direction, 1.0f / direction};
    if (direction < 0.0f)
        return {-1, (position - std::floor(position)) / -direction, 1.0f / -direction};
    return {0, INFINITY, INFINITY};
}

} // namespace

bool raycast(const VoxelWorld& world, const BlockRegistry& blocks, glm::vec3 origin,
             glm::vec3 direction, f32 maxDistance, VoxelRaycastHit& hit)
{
    const f32 directionLength = glm::length(direction);
    if (directionLength <= 0.0f || maxDistance < 0.0f)
        return false;
    direction /= directionLength;

    VoxelCoord current{static_cast<s32>(std::floor(origin.x)), static_cast<s32>(std::floor(origin.y)),
                       static_cast<s32>(std::floor(origin.z))};
    const BlockDefinition* definition = blocks.find(world.block(current));
    if (definition && definition->solid)
    {
        hit = {current, current, BlockFace::Count, 0.0f};
        return true;
    }

    AxisStep x = makeAxisStep(origin.x, direction.x);
    AxisStep y = makeAxisStep(origin.y, direction.y);
    AxisStep z = makeAxisStep(origin.z, direction.z);
    f32 distance = 0.0f;
    VoxelCoord previous = current;
    BlockFace face = BlockFace::Count;

    while (distance <= maxDistance)
    {
        if (x.nextBoundary <= y.nextBoundary && x.nextBoundary <= z.nextBoundary)
        {
            distance = x.nextBoundary;
            x.nextBoundary += x.delta;
            previous = current;
            current.x += x.direction;
            face = x.direction > 0 ? BlockFace::NegativeX : BlockFace::PositiveX;
        }
        else if (y.nextBoundary <= z.nextBoundary)
        {
            distance = y.nextBoundary;
            y.nextBoundary += y.delta;
            previous = current;
            current.y += y.direction;
            face = y.direction > 0 ? BlockFace::NegativeY : BlockFace::PositiveY;
        }
        else
        {
            distance = z.nextBoundary;
            z.nextBoundary += z.delta;
            previous = current;
            current.z += z.direction;
            face = z.direction > 0 ? BlockFace::NegativeZ : BlockFace::PositiveZ;
        }

        if (distance > maxDistance)
            break;

        definition = blocks.find(world.block(current));
        if (definition && definition->solid)
        {
            hit = {current, previous, face, distance};
            return true;
        }
    }
    return false;
}

} // namespace Voxel
} // namespace Radion