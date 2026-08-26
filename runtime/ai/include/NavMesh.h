#ifndef RADION_AI_NAVMESH_H
#define RADION_AI_NAVMESH_H

// NavMesh.h - walkable surface generated from level geometry, and shortest
// paths across it.
//
// Unlike WaypointNetwork, nothing here is authored by hand: the walkable
// floor is derived from the level's own triangles, so a path can never cut
// through a wall or hang in mid-air the way a hand-placed graph linked by
// line of sight can.

#include "Types.h"

#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace Radion::AI
{

// Voxelisation and agent metrics the surface is built for. An agent taller
// or wider than these will not fit through everything the mesh claims is
// walkable, so they describe the agent the mesh is FOR, not any agent.
struct NavMeshConfig
{
    f32 cellSize = 0.30f;
    f32 cellHeight = 0.20f;
    f32 agentHeight = 2.00f;
    f32 agentRadius = 0.60f;
    f32 agentMaxClimb = 0.90f; // step/stair height the agent can walk up
    f32 agentMaxSlope = 45.0f; // degrees
    s32 regionMinSize = 8;
    s32 regionMergeSize = 20;
    f32 edgeMaxLen = 12.0f;
    f32 edgeMaxError = 1.30f;
    s32 vertsPerPoly = 6;
    f32 detailSampleDist = 6.00f;
    f32 detailSampleMaxError = 1.00f;
};

class NavMesh
{
public:
    NavMesh() = default;
    ~NavMesh();

    NavMesh(const NavMesh&) = delete;
    NavMesh& operator=(const NavMesh&) = delete;

    // World-space triangles, the same arrays a mesh already holds. Replaces
    // whatever was built before. False leaves the previous mesh released and
    // valid() false.
    //
    // groundSeed, if given, prunes the surface down to whatever a walking
    // agent can actually reach from that point - polygon-adjacency BFS, not
    // a height/slope heuristic, so a flat roof with no stairs down to it
    // comes out excluded (both from queries and from debugTriangles())
    // exactly like the recast slope test alone cannot tell apart from real
    // ground. Omit it (or pass a point off the mesh) to keep every
    // walkable-slope surface, roofs included, the way build() always did.
    bool build(const f32* vertices, s32 vertexCount, const s32* indices, s32 triangleCount,
               const NavMeshConfig& config, const Math::Vec3* groundSeed = nullptr);
    void release();
    bool valid() const;

    // Corner points from start to end, including both. False when either end
    // is off the mesh (beyond `searchExtents`) or no route exists. A partial
    // route - the goal unreachable but progress possible - still returns true
    // and ends at the closest reachable point.
    bool findPath(const Math::Vec3& start, const Math::Vec3& end, std::vector<Math::Vec3>& outPath,
                  const Math::Vec3& searchExtents = Math::Vec3(2.0f, 4.0f, 2.0f)) const;

    // Closest point actually on the walkable surface, for snapping a spawn
    // or a click onto it. False when nothing is within `searchExtents`.
    bool nearestPoint(const Math::Vec3& point, Math::Vec3& out,
                      const Math::Vec3& searchExtents = Math::Vec3(2.0f, 4.0f, 2.0f)) const;

    // Slides `from` towards `to` across the walkable surface and returns
    // where it ends up, which is `to` only when the whole segment stays on
    // the mesh - otherwise the move is clipped against the surface boundary
    // and slides along it. This is what makes crossing a wall impossible
    // rather than merely unlikely: a route only suggests a direction, and
    // steering that cuts a corner, or avoidance pushing an agent sideways,
    // leaves the floor on its own. `out.y` is placed on the surface.
    //
    // `from` is expected to already be on the mesh - it is the agent's last
    // constrained position, not an arbitrary point. False when it is not,
    // leaving `out` untouched.
    bool moveAlongSurface(const Math::Vec3& from, const Math::Vec3& to, Math::Vec3& out,
                          const Math::Vec3& searchExtents = Math::Vec3(2.0f, 4.0f, 2.0f)) const;

    // Triangles of the walkable surface itself, world space, for drawing it.
    // Rebuilt only by build(), so a caller may keep the reference.
    const std::vector<Math::Vec3>& debugTriangles() const;

    // Persists the built surface itself - the Detour tile's raw bytes, not
    // the Recast recipe that produced them - so loading skips the whole
    // voxelise/rasterise/contour/poly-mesh pipeline build() runs. False (and
    // nothing written) when the mesh is not built yet.
    bool save(const std::string& filename) const;
    // Replaces whatever was built before, straight from those bytes -
    // false, and the previous mesh released, if the file is missing or does
    // not parse as one this format wrote.
    bool load(const std::string& filename);

private:
    // dtNavMesh/dtNavMeshQuery, kept out of this header so Recast's own
    // includes stay inside NavMesh.cpp.
    void* mNavMesh = nullptr;
    void* mNavQuery = nullptr;
    std::vector<Math::Vec3> mDebugTriangles;
};

} // namespace Radion::AI

#endif // RADION_AI_NAVMESH_H
