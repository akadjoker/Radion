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

    // Hull of "count" vertices in "coords", "stride" bytes apart. A positive "shrink" moves each face that far towards the center
    // along its normal, clamped to "shrinkClamp * innerRadius" if shrinkClamp is positive. Returns the shrink applied;
    // negative means the hull became empty.
    f32 compute(const float* coords, int stride, int count, f32 shrink, f32 shrinkClamp);
};

} // namespace Radion::Geometry

#endif // RADION_GEOMETRY_CONVEX_HULL_COMPUTER_H
