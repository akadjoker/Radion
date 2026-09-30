#ifndef RADION_AI_NAVMESHBEHAVIOR_H
#define RADION_AI_NAVMESHBEHAVIOR_H

#include "Behavior.h"

#include "Math.h"
#include <vector>

namespace Radion
{
class Agent;
}

namespace Radion::AI
{

class NavMesh;

class NavMeshBehavior final : public Behavior
{
public:
    struct Settings
    {
        float turnRate = 0.35f;
        float goalRadius = 1.0f;
        float cornerRadius = 0.6f;
        float avoidDistance = 0.0f; // <= 0 disables agent avoidance
        // Seconds between route queries; findPath() is a real A*.
        float repathInterval = 0.35f;
        // A route is recomputed only once the goal has moved this far from where the current one was found.
        float goalMoveThreshold = 1.0f;
        // Max distance off the mesh a point may sit and still snap onto it.
        Math::vec3 searchExtents = Math::vec3(2.0f, 6.0f, 2.0f);
    };

    // Defined in the .cpp: a default argument cannot value-initialize a nested class mid-definition of the enclosing one.
    explicit NavMeshBehavior(const NavMesh* navMesh = nullptr);
    NavMeshBehavior(const NavMesh* navMesh, const Settings& settings);

    void setNavMesh(const NavMesh* navMesh)
    {
        mNavMesh = navMesh;
    }
    const NavMesh* navMesh() const
    {
        return mNavMesh;
    }

    void iterate(float timeDelta, Radion::Agent& entity) override;
    const char* name() const override
    {
        return "NavMesh Behavior";
    }
    BehaviorType type() const override
    {
        return BehaviorType::NavMesh;
    }
    u32 paramCount() const override;
    const BehaviorParam& paramInfo(u32 index) const override;
    f32 paramFloat(u32 index) const override;
    void setParamFloat(u32 index, f32 value) override;
    Math::vec3 paramVec3(u32 index) const override;
    void setParamVec3(u32 index, const Math::vec3& value) override;

    Settings& settings()
    {
        return mSettings;
    }
    const Settings& settings() const
    {
        return mSettings;
    }

private:
    // Route state for this behavior's one agent.
    struct Route
    {
        std::vector<Math::vec3> corners;
        usize next = 0;
        float sinceRepath = 0.0f;
        Math::vec3 goalWhenFound = Math::vec3(0.0f);
        bool hasRoute = false;
        Math::vec3 surfacePosition = Math::vec3(0.0f);
        bool onSurface = false;
    };

    void constrainToSurface(Radion::Agent& entity, Route& route);
    void applyAvoidance(Radion::Agent& entity);

    const NavMesh* mNavMesh;
    Settings mSettings;
    Route mRoute;
};

} // namespace Radion::AI

#endif // RADION_AI_NAVMESHBEHAVIOR_H
