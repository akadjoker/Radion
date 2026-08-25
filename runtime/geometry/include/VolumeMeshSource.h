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

// A loaded mesh as a density field, which is what lets everything in
// VolumeCSG.h - Union, Intersection, Difference - work on geometry that was
// modelled rather than described. Without it the CSG can only combine spheres,
// boxes, planes and noise with each other.
//
// Density is the distance to the nearest triangle, positive inside, so it
// follows the same sign convention as every other Source.
//
// The sign comes from counting how many triangles a ray leaving the point
// crosses: odd means it started inside. That answer only means anything on a
// closed surface - an open one has no inside, and a ray leaving through the
// hole gives the opposite parity to one that does not. Three rays vote, which
// covers a ray grazing an edge but not a mesh that is genuinely open. The Mesh
// Health panel reports boundary edges, and a mesh with any is not a solid.
//
// Sampling is not reentrant: the candidate lists the tree queries fill are
// kept between calls rather than allocated per sample, and there are millions
// of samples in a meshing pass.
class MeshSource final : public Source
{
public:
    MeshSource();
    ~MeshSource() override;

    MeshSource(const MeshSource&) = delete;
    MeshSource& operator=(const MeshSource&) = delete;

    // Takes a copy of the triangles and builds the tree over them; the
    // MeshData is not kept. False when there are no triangles to take.
    bool build(const MeshData& mesh);
    void clear();
    bool valid() const;

    f32 sampleDensity(const glm::vec3& position) const override;

    // The mesh's own box, which is what a meshing pass wants for its bounds -
    // grown a little, or the surface sits exactly on the edge of the grid.
    const AABB& bounds() const
    {
        return mBounds;
    }

    u32 triangleCount() const;

private:
    f32 unsignedDistance(const glm::vec3& position) const;
    bool isInside(const glm::vec3& position) const;

    // Three corners per triangle, flattened: the tree indexes triangles, and
    // an index buffer here would cost a second indirection per candidate.
    std::vector<glm::vec3> mCorners;
    BoundsTree mTree;
    AABB mBounds;
    f32 mDiagonal = 0.0f;

    mutable std::vector<u32> mCandidates;
    mutable std::vector<u32> mRayCandidates;
};

} // namespace Radion::Volume

#endif // RADION_VOLUME_MESH_SOURCE_H
