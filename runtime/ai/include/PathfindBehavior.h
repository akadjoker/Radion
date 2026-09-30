#ifndef RADION_AI_PATHFINDBEHAVIOR_H
#define RADION_AI_PATHFINDBEHAVIOR_H

#include "Behavior.h"
#include "WaypointNetwork.h"

#include "Math.h"

namespace Radion
{
class Agent;
}

namespace Radion::AI
{

class WaypointNetwork;

class PathfindBehavior final : public Behavior
{
public:
    struct Settings
    {
        float turnRate = 0.2f;
        float goalRadius = 50.0f;
        float avoidDistance = 0.0f;            // <= 0 disables avoidance
        float maxTimeBeforeAgitation = 25.0f;
        float maxTimeBeforeLineOfSight = 0.5f;
        // Seconds between A* searches; stops a failed search being retried every frame.
        float repathInterval = 0.35f;
        Math::vec3 upVector = Math::vec3(0.0f, 1.0f, 0.0f);
        WaypointNetwork* waypointNetwork = nullptr; // non-owning
        // Line-of-sight functor for the goal short-circuit; nullptr means always visible.
        const WaypointVisibility* visibility = nullptr;
    };

    // No-arg overload default-constructs Settings in the .cpp (see NavMeshBehavior.h).
    PathfindBehavior();
    explicit PathfindBehavior(const Settings& settings);

    void iterate(float timeDelta, Radion::Agent& entity) override;
    void applyAvoidance(Radion::Agent& entity);
    const char* name() const override
    {
        return "Pathfind Behavior";
    }
    BehaviorType type() const override
    {
        return BehaviorType::Pathfind;
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
    Settings mSettings;
    float mSinceRepath = 0.0f;
};

} // namespace Radion::AI

#endif // RADION_AI_PATHFINDBEHAVIOR_H
