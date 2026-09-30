#ifndef RADION_NAVMESHSURFACE_H
#define RADION_NAVMESHSURFACE_H

#include "Component.h"
#include "NavMesh.h"

#include <string>

namespace Radion
{

// Marks its owner's mesh as the ground AI walks on. Attach it to the level
// object in the editor, bake, and everything that navigates queries the
// surface it produced - the walkable floor comes from the geometry itself,
// so nothing has to be placed by hand and no route can leave the mesh.
//
// The build is explicit, never automatic: baking a large level is slow, and
// doing it silently on load (or on every transform tweak) would be paid for
// at the worst possible moment.
class NavMeshSurface final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::NavMeshSurface;

    AI::NavMeshConfig& config();
    const AI::NavMeshConfig& config() const;

    // World-space point build() treats as "definitely ground" - only the
    // surface reachable from here by ordinary walking (polygon adjacency,
    // not height or slope) survives the bake. A roof passes the same
    // walkable-slope test the ground does; without a seed it stays in the
    // surface, an island with no stairs down to it. Defaults to the owner's
    // own position, which is ground level for most levels as authored.
    void setGroundSeed(const Math::vec3& worldPosition);
    const Math::vec3& groundSeed() const;

    // Rebuilds from the owner's MeshRenderer, baked through the owner's own
    // world transform so the surface lands where the level is actually
    // drawn. False (and the previous surface released) when the owner has no
    // readable mesh or Recast rejects it.
    bool build();
    bool built() const;

    const AI::NavMesh& navMesh() const;

    // Seconds the last build() took - what the editor shows to justify
    // keeping the bake explicit.
    f32 lastBuildMilliseconds() const;

    // The built surface's own bytes, not the Recast recipe - skips the
    // whole voxelise/rasterise/contour/poly-mesh pipeline on load. Loading
    // replaces whatever was built; the path is remembered so the scene
    // serializer can ask for it again without the caller repeating it.
    bool saveNavData(const std::string& filename);
    bool loadNavData(const std::string& filename);
    const std::string& navDataFile() const;

private:
    friend class GameObject;

    NavMeshSurface();

    AI::NavMeshConfig mConfig;
    AI::NavMesh mNavMesh;
    f32 mLastBuildMilliseconds = 0.0f;
    std::string mNavDataFile;
    Math::vec3 mGroundSeed = Math::vec3(0.0f);
    bool mGroundSeedSet = false;
};

} // namespace Radion

#endif // RADION_NAVMESHSURFACE_H
