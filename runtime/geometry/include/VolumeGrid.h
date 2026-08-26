#ifndef RADION_VOLUME_GRID_H
#define RADION_VOLUME_GRID_H

#include "VolumeSource.h"
#include "ByteArray.h"

#include <vector>

namespace Radion { struct MeshData; }

namespace Radion::Volume
{

struct MeshingStats;

enum class VolumeOperation { Union, Difference, Intersection };

class GridSource final : public Source
{
public:
    // dimensions is the number of voxel samples, not the number of cells.
    GridSource(Math::uvec3 dimensions, Math::vec3 origin, f32 cellSize,
               f32 initialDensity = -1.0f);

    Math::uvec3 dimensions() const { return m_dimensions; }
    Math::vec3 origin() const { return m_origin; }
    f32 cellSize() const { return m_cellSize; }
    bool valid() const { return !m_values.empty(); }
    AABB bounds() const;
    Math::vec3 voxelPosition(u32 x, u32 y, u32 z) const;
    bool worldToVoxel(const Math::vec3& position, Math::uvec3& voxel) const;

    f32 voxel(u32 x, u32 y, u32 z) const;
    bool setVoxel(u32 x, u32 y, u32 z, f32 density);
    void fill(const Source& source);
    AABB apply(VolumeOperation operation, const Source& brush, const AABB& affected);
    bool buildMesh(MeshData& output, MeshingStats* stats = nullptr) const;

    bool save(ByteArray& output) const;
    static bool load(ByteArray& input, GridSource& output);

    f32 sampleDensity(const Math::vec3& position) const override;
    Sample sample(const Math::vec3& position) const override;

private:
    usize index(u32 x, u32 y, u32 z) const;
    bool contains(u32 x, u32 y, u32 z) const;
    f32 sampleClamped(const Math::ivec3& point) const;

    Math::uvec3 m_dimensions{0};
    Math::vec3 m_origin{0.0f};
    f32 m_cellSize = 0.0f;
    std::vector<f32> m_values;
};

} // namespace Radion::Volume

#endif // RADION_VOLUME_GRID_H
