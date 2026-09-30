#ifndef RADION_AI_NAVMESH_H
#define RADION_AI_NAVMESH_H

// Derived from the level's triangles, so a path never cuts through a wall.

#include "Types.h"

#include "Math.h"
#include <string>
#include <vector>

namespace Radion::AI
{

// Agent metrics the mesh is built for; larger agents will not fit everywhere it claims walkable.
struct NavMeshConfig
{
    f32 cellSize = 0.30f;
    f32 cellHeight = 0.20f;
    f32 agentHeight = 2.00f;
    f32 agentRadius = 0.60f;
    f32 agentMaxClimb = 0.90f;
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

    // Replaces any prior mesh. groundSeed prunes to what is reachable by polygon-adjacency BFS from that point
    // (excludes unreachable roofs); omit to keep every walkable-slope surface.
    bool build(const f32* vertices, s32 vertexCount, const s32* indices, s32 triangleCount,
               const NavMeshConfig& config, const Math::vec3* groundSeed = nullptr);
    void release();
    bool valid() const;

    // False when either end is off the mesh (beyond searchExtents) or no route exists; a partial route
    // still returns true, ending at the closest reachable point.
    bool findPath(const Math::vec3& start, const Math::vec3& end, std::vector<Math::vec3>& outPath,
                  const Math::vec3& searchExtents = Math::vec3(2.0f, 4.0f, 2.0f)) const;

    bool nearestPoint(const Math::vec3& point, Math::vec3& out,
                      const Math::vec3& searchExtents = Math::vec3(2.0f, 4.0f, 2.0f)) const;

    // Slides `from` toward `to`, clipped at the surface boundary; `from` must already be on the mesh
    // (false otherwise, `out` untouched). out.y is placed on the surface.
    bool moveAlongSurface(const Math::vec3& from, const Math::vec3& to, Math::vec3& out,
                          const Math::vec3& searchExtents = Math::vec3(2.0f, 4.0f, 2.0f)) const;

    // Rebuilt only by build(), so a caller may keep the reference.
    const std::vector<Math::vec3>& debugTriangles() const;

    // Persists the built Detour tile bytes, not the Recast recipe, so loading skips the build pipeline.
    bool save(const std::string& filename) const;
    // False, and the previous mesh released, if the file is missing or invalid.
    bool load(const std::string& filename);

private:
    // Kept out of this header so Recast includes stay inside NavMesh.cpp.
    void* mNavMesh = nullptr;
    void* mNavQuery = nullptr;
    std::vector<Math::vec3> mDebugTriangles;
};

} // namespace Radion::AI

#endif // RADION_AI_NAVMESH_H
