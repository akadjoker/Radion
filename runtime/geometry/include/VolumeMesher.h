#ifndef RADION_VOLUME_MESHER_H
#define RADION_VOLUME_MESHER_H

#include "Mesh.h"
#include "VolumeSource.h"

namespace Radion::Volume
{

struct MeshingSettings
{
    AABB bounds;
    f32 voxelSize = 0.5f;
    f32 isoLevel = 0.0f;
    bool generateUVs = false;
};

struct MeshingStats
{
    u64 samples = 0;
    u32 cells = 0;
    u32 triangles = 0;
};

// Builds CPU geometry. An empty result is valid when the source does not cross
// the requested volume; invalid settings return false and leave `out` intact.
bool buildMesh(const Source& source, const MeshingSettings& settings,
               MeshData& out, MeshingStats* stats = nullptr);

} // namespace Radion::Volume

#endif // RADION_VOLUME_MESHER_H
