#ifndef RADION_NAVMESHSURFACE_H
#define RADION_NAVMESHSURFACE_H

#include "Component.h"
#include "NavMesh.h"

#include <string>

namespace Radion
{

// The build is explicit, never automatic: baking a large level is slow.
class NavMeshSurface final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::NavMeshSurface;

    AI::NavMeshConfig& config();
    const AI::NavMeshConfig& config() const;

    // Only surface reachable by walking from here survives the bake (drops unreachable roofs). Defaults to the owner's position.
    void setGroundSeed(const Math::vec3& worldPosition);
    const Math::vec3& groundSeed() const;

    // False (previous surface released) when the owner has no readable mesh or Recast rejects it.
    bool build();
    bool built() const;

    const AI::NavMesh& navMesh() const;

    f32 lastBuildMilliseconds() const;

    // Skips the Recast pipeline on load.
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
