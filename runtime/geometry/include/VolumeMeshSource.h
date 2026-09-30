#ifndef RADION_VOLUME_MESH_SOURCE_H
#define RADION_VOLUME_MESH_SOURCE_H

#include "BoundsTree.h"
#include "Math.h"
#include "VolumeSource.h"

#include <vector>

namespace Radion
{
struct MeshData;
}

namespace Radion::Volume
{

// Mesh as a density field (distance to nearest triangle, positive inside) so CSG works on modelled geometry.
// Sign is by ray parity, three rays voting: only meaningful on a closed mesh (boundary edges in Mesh Health mean not solid).
// Sampling is not reentrant: the tree's candidate lists are reused between calls (millions of samples per pass).
class MeshSource final : public Source
{
public:
    MeshSource();
    ~MeshSource() override;

    MeshSource(const MeshSource&) = delete;
    MeshSource& operator=(const MeshSource&) = delete;

    // Copies the triangles and builds the tree; the MeshData is not kept. False when empty.
    bool build(const MeshData& mesh);
    void clear();
    bool valid() const;

    f32 sampleDensity(const Math::vec3& position) const override;

    // The mesh's own box, grown a little or the surface sits exactly on the grid edge.
    const AABB& bounds() const
    {
        return mBounds;
    }

    u32 triangleCount() const;

private:
    f32 unsignedDistance(const Math::vec3& position) const;
    bool isInside(const Math::vec3& position) const;

    // Three corners per triangle, flattened: an index buffer would cost a second indirection per candidate.
    std::vector<Math::vec3> mCorners;
    BoundsTree mTree;
    AABB mBounds;
    f32 mDiagonal = 0.0f;

    mutable std::vector<u32> mCandidates;
    mutable std::vector<u32> mRayCandidates;
};

} // namespace Radion::Volume

#endif // RADION_VOLUME_MESH_SOURCE_H
