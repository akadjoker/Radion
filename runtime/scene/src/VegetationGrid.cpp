#include "PCH.h"

#include "VegetationGrid.h"

#include "Terrain.h"

#include <glm/gtc/matrix_inverse.hpp>

namespace Radion
{

VegetationGrid::VegetationGrid()
{
}

bool VegetationGrid::create(u32 gridSize, f32 worldSize, const glm::mat4& localToWorld)
{
    if (gridSize == 0 || worldSize <= 0.0f)
        return false;
    return create(gridSize, gridSize, worldSize, worldSize, localToWorld);
}

bool VegetationGrid::create(u32 width, u32 depth, f32 worldSizeX, f32 worldSizeZ,
                            const glm::mat4& localToWorld)
{
    if (width == 0 || depth == 0 || worldSizeX <= 0.0f || worldSizeZ <= 0.0f)
        return false;

    mWidth = width;
    mDepth = depth;
    mWorldSizeX = worldSizeX;
    mWorldSizeZ = worldSizeZ;
    mCellSizeX = worldSizeX / static_cast<f32>(width);
    mCellSizeZ = worldSizeZ / static_cast<f32>(depth);

    setTransform(localToWorld);

    mCells.resize(static_cast<usize>(width) * static_cast<usize>(depth));

    for (u32 z = 0; z < depth; ++z)
    {
        for (u32 x = 0; x < width; ++x)
        {
            Cell& cell = *cellAt(x, z);
            cell.position = cellLocalPosition(x, z);
            cell.normal = glm::vec3(0.0f, 1.0f, 0.0f);
            cell.occupant = Occupant::None;
            cell.species = 0;
            cell.scale = 1.0f;
            cell.yaw = 0.0f;

            
        }
    }

    return true;
}

void VegetationGrid::clear()
{
    for (Cell& cell : mCells)
        cell.occupant = Occupant::None;
}

void VegetationGrid::setTransform(const glm::mat4& localToWorld)
{
    mLocalToWorld = localToWorld;
    updateInverseTransform();
}

const glm::mat4& VegetationGrid::transform() const
{
    return mLocalToWorld;
}

void VegetationGrid::updateInverseTransform()
{
    mWorldToLocal = glm::inverse(mLocalToWorld);
}

bool VegetationGrid::paintGrass(const glm::vec3& worldPos, f32 scale)
{
    s32 x = 0;
    s32 z = 0;
    if (!worldToCell(worldPos, x, z))
        return false;
    return paintGrassAt(static_cast<u32>(x), static_cast<u32>(z), scale);
}

bool VegetationGrid::paintTree(const glm::vec3& worldPos, u32 species, f32 scale,
                               f32 yawDegrees)
{
    s32 x = 0;
    s32 z = 0;
    if (!worldToCell(worldPos, x, z))
        return false;
    return paintTreeAt(static_cast<u32>(x), static_cast<u32>(z), species, scale, yawDegrees);
}

bool VegetationGrid::paintGrassAt(u32 x, u32 z, f32 scale)
{
    Cell* cell = cellAt(x, z);
    if (!cell || cell->occupant != Occupant::None)
        return false;

    cell->occupant = Occupant::Grass;
    cell->scale = scale > 0.0f ? scale : 1.0f;
    return true;
}

bool VegetationGrid::paintTreeAt(u32 x, u32 z, u32 species, f32 scale, f32 yawDegrees)
{
    Cell* cell = cellAt(x, z);
    if (!cell)
        return false;

    cell->occupant = Occupant::Tree;
    cell->species = species;
    cell->scale = scale > 0.0f ? scale : 1.0f;
    cell->yaw = glm::radians(yawDegrees);
    return true;
}

bool VegetationGrid::erase(const glm::vec3& worldPos)
{
    s32 x = 0;
    s32 z = 0;
    if (!worldToCell(worldPos, x, z))
        return false;
    return eraseAt(static_cast<u32>(x), static_cast<u32>(z));
}

bool VegetationGrid::eraseAt(u32 x, u32 z)
{
    Cell* cell = cellAt(x, z);
    if (!cell)
        return false;
    cell->occupant = Occupant::None;
    return true;
}

u32 VegetationGrid::width() const
{
    return mWidth;
}

u32 VegetationGrid::depth() const
{
    return mDepth;
}

f32 VegetationGrid::worldSizeX() const
{
    return mWorldSizeX;
}

f32 VegetationGrid::worldSizeZ() const
{
    return mWorldSizeZ;
}

u32 VegetationGrid::cellCount() const
{
    return static_cast<u32>(mCells.size());
}

u32 VegetationGrid::occupiedCount() const
{
    u32 count = 0;
    for (const Cell& cell : mCells)
        if (cell.occupant != Occupant::None)
            ++count;
    return count;
}

const VegetationGrid::Cell* VegetationGrid::cell(u32 x, u32 z) const
{
    return cellAt(x, z);
}

const std::vector<VegetationGrid::Cell>& VegetationGrid::cells() const
{
    return mCells;
}

bool VegetationGrid::worldToCell(const glm::vec3& world, s32& x, s32& z) const
{
    if (mWidth == 0 || mDepth == 0)
        return false;

    const glm::vec3 local = glm::vec3(mWorldToLocal * glm::vec4(world, 1.0f));

    x = static_cast<s32>((local.x + mWorldSizeX * 0.5f) / mCellSizeX);
    z = static_cast<s32>((local.z + mWorldSizeZ * 0.5f) / mCellSizeZ);

    if (x < 0 || x >= static_cast<s32>(mWidth) || z < 0 || z >= static_cast<s32>(mDepth))
        return false;
    return true;
}

VegetationGrid::Cell* VegetationGrid::cellAt(u32 x, u32 z)
{
    if (x >= mWidth || z >= mDepth)
        return nullptr;
    return &mCells[static_cast<usize>(z) * mWidth + x];
}

const VegetationGrid::Cell* VegetationGrid::cellAt(u32 x, u32 z) const
{
    if (x >= mWidth || z >= mDepth)
        return nullptr;
    return &mCells[static_cast<usize>(z) * mWidth + x];
}

glm::vec3 VegetationGrid::cellLocalPosition(u32 x, u32 z) const
{
    const f32 localX = (static_cast<f32>(x) + 0.5f) * mCellSizeX - mWorldSizeX * 0.5f;
    const f32 localZ = (static_cast<f32>(z) + 0.5f) * mCellSizeZ - mWorldSizeZ * 0.5f;
    return glm::vec3(localX, 0.0f, localZ);
}

} // namespace Radion
