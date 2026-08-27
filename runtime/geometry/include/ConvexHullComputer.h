#ifndef RADION_GEOMETRY_CONVEX_HULL_COMPUTER_H
#define RADION_GEOMETRY_CONVEX_HULL_COMPUTER_H

#include "Types.h"

#include "Math.h"
#include <vector>

namespace Radion::Geometry
{

// Incremental 3D convex hull construction (Preparata & Hong).
class ConvexHullComputer
{
public:
    class Edge
    {
    public:
        int getSourceVertex() const;
        int getTargetVertex() const;
        const Edge* getNextEdgeOfVertex() const;
        const Edge* getNextEdgeOfFace() const;
        const Edge* getReverseEdge() const;

    private:
        int next;
        int reverse;
        int targetVertex;

        friend class ConvexHullComputer;
    };

    std::vector<Math::vec3> vertices;
    std::vector<Edge> edges;
    std::vector<int> faces;

    // Computes the convex hull of "count" vertices stored in "coords". "stride" is the
    // difference in bytes between the addresses of consecutive vertices. If "shrink" is
    // positive, the convex hull is shrunk by that amount (each face is moved by "shrink"
    // length units towards the center along its normal). If "shrinkClamp" is positive,
    // "shrink" is clamped to not exceed "shrinkClamp * innerRadius", where "innerRadius"
    // is the minimum distance of a face to the center of the convex hull.
    //
    // The returned value is the amount by which the hull has been shrunk. If it is negative,
    // the amount was so large that the resulting convex hull is empty.
    f32 compute(const float* coords, int stride, int count, f32 shrink, f32 shrinkClamp);
};

} // namespace Radion::Geometry

#endif // RADION_GEOMETRY_CONVEX_HULL_COMPUTER_H
