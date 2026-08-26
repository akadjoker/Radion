#ifndef RADION_VEGETATION_GRID_H
#define RADION_VEGETATION_GRID_H

#include "Types.h"

#include <glm/glm.hpp>
#include <vector>

namespace Radion
{

class Terrain;

class VegetationGrid
{
public:
    enum class Occupant : u8
    {
        None,
        Grass,
        Tree
    };

    struct Cell
    {
        Math::Vec3 position = Math::Vec3(0.0f);           // local space
        Math::Vec3 normal = Math::Vec3(0.0f, 1.0f, 0.0f); // local space
        Occupant occupant = Occupant::None;
        u32 species = 0;
        f32 scale = 1.0f;
        f32 yaw = 0.0f; // radians
    };

    VegetationGrid();

    // Square grid covering [-worldSize/2, worldSize/2] in local X and Z.
    bool create(u32 gridSize, f32 worldSize, const Math::Mat4& localToWorld = Math::Mat4(1.0f));

    // Rectangular grid covering [-worldSizeX/2, worldSizeX/2] in X and
    // [-worldSizeZ/2, worldSizeZ/2] in Z.
    bool create(u32 width, u32 depth, f32 worldSizeX, f32 worldSizeZ,
                const Math::Mat4& localToWorld = Math::Mat4(1.0f));

    void clear();

    // Change the local-to-world transform used for painting and terrain sampling.
    void setTransform(const Math::Mat4& localToWorld);
    const Math::Mat4& transform() const;

    // Paint using world coordinates. The point is transformed to local space,
    // mapped to a cell, and the occupancy rules are applied.
    bool paintGrass(const Math::Vec3& worldPos, f32 scale = 1.0f);
    bool paintTree(const Math::Vec3& worldPos, u32 species, f32 scale = 1.0f, f32 yawDegrees = 0.0f);

    // Paint by cell index. Fails if out of bounds.
    bool paintGrassAt(u32 x, u32 z, f32 scale = 1.0f);
    bool paintTreeAt(u32 x, u32 z, u32 species, f32 scale = 1.0f, f32 yawDegrees = 0.0f);

    bool erase(const Math::Vec3& worldPos);
    bool eraseAt(u32 x, u32 z);

    u32 width() const;
    u32 depth() const;
    f32 worldSizeX() const;
    f32 worldSizeZ() const;
    u32 cellCount() const;
    u32 occupiedCount() const;

    const Cell* cell(u32 x, u32 z) const;
    const std::vector<Cell>& cells() const;

private:
    std::vector<Cell> mCells;
    u32 mWidth = 0;
    u32 mDepth = 0;
    f32 mWorldSizeX = 1.0f;
    f32 mWorldSizeZ = 1.0f;
    f32 mCellSizeX = 1.0f;
    f32 mCellSizeZ = 1.0f;
    Math::Mat4 mLocalToWorld = Math::Mat4(1.0f);
    Math::Mat4 mWorldToLocal = Math::Mat4(1.0f);

    bool worldToCell(const Math::Vec3& world, s32& x, s32& z) const;
    Cell* cellAt(u32 x, u32 z);
    const Cell* cellAt(u32 x, u32 z) const;
    Math::Vec3 cellLocalPosition(u32 x, u32 z) const;
    void updateInverseTransform();
};

} // namespace Radion

#endif // RADION_VEGETATION_GRID_H
