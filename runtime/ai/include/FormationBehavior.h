#ifndef RADION_AI_FORMATIONBEHAVIOR_H
#define RADION_AI_FORMATIONBEHAVIOR_H

// squadId() == 0 is the leader (player controlled; skips the formation).
// Local frame: right = +X, up = +Y, forward = +Z.

#include "Behavior.h"

#include "Math.h"

namespace Radion
{
class Agent;
}

namespace Radion::AI
{

enum class SquadFormation
{
    Pentagon,
    Diamond,
    Abreast,
    SingleFile,
    Wedge,
    V,
    Circle,
    Count
};

class FormationBehavior final : public Behavior
{
public:
    FormationBehavior(float goalRadius = 1.0f, float formationRadius = 1.0f);

    void iterate(float timeDelta, Radion::Agent& entity) override;
    const char* name() const override
    {
        return "Formation Behavior";
    }
    BehaviorType type() const override
    {
        return BehaviorType::Formation;
    }
    u32 paramCount() const override;
    const BehaviorParam& paramInfo(u32 index) const override;
    f32 paramFloat(u32 index) const override;
    void setParamFloat(u32 index, f32 value) override;

    // Goal radius for the point man; every other member uses formationRadius().
    float goalRadius() const
    {
        return mGoalRadius;
    }
    void setGoalRadius(float radius)
    {
        mGoalRadius = radius;
    }
    float formationRadius() const
    {
        return mFormationRadius;
    }
    void setFormationRadius(float radius)
    {
        mFormationRadius = radius;
    }

private:
    void singleFile(Radion::Agent& entity, Math::vec3& goal, Math::vec3& dir) const;
    void abreast(Radion::Agent& entity, Math::vec3& goal, Math::vec3& dir) const;
    void diamond(Radion::Agent& entity, Math::vec3& goal, Math::vec3& dir) const;
    void pentagon(Radion::Agent& entity, Math::vec3& goal, Math::vec3& dir) const;
    void wedge(Radion::Agent& entity, Math::vec3& goal, Math::vec3& dir) const;
    void vFormation(Radion::Agent& entity, Math::vec3& goal, Math::vec3& dir) const;
    void circle(Radion::Agent& entity, Math::vec3& goal, Math::vec3& dir) const;

    float mGoalRadius;
    float mFormationRadius;

    Radion::Agent* mSquadLeader = nullptr; // non-owning
    Radion::Agent* mPointMan = nullptr;    // non-owning

    Math::vec3 mPointManLook;
    Math::vec3 mPointManRight;
    Math::vec3 mLeaderLook;
    Math::vec3 mLeaderRight;
};

} // namespace Radion::AI

#endif // RADION_AI_FORMATIONBEHAVIOR_H
