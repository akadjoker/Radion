#ifndef RADION_AI_FORMATIONBEHAVIOR_H
#define RADION_AI_FORMATIONBEHAVIOR_H

// FormationBehavior.h - places squad members into a formation around the
// squad leader and point man.
//
// squadId() == 0 is the leader (player controlled; skips the formation).
//
// Orientation convention: the local frame is right = +X, up = +Y,
// forward (look) = +Z.

#include "Behavior.h"

#include <glm/glm.hpp>

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

    // Goal radius handed to the point man's Agent::setGoalRadius(); the
    // radius handed to every other member is formationRadius() instead.
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
    void singleFile(Radion::Agent& entity, glm::vec3& goal, glm::vec3& dir) const;
    void abreast(Radion::Agent& entity, glm::vec3& goal, glm::vec3& dir) const;
    void diamond(Radion::Agent& entity, glm::vec3& goal, glm::vec3& dir) const;
    void pentagon(Radion::Agent& entity, glm::vec3& goal, glm::vec3& dir) const;
    void wedge(Radion::Agent& entity, glm::vec3& goal, glm::vec3& dir) const;
    void vFormation(Radion::Agent& entity, glm::vec3& goal, glm::vec3& dir) const;
    void circle(Radion::Agent& entity, glm::vec3& goal, glm::vec3& dir) const;

    float mGoalRadius;
    float mFormationRadius;

    // DESVIO 2: with behaviors owned one-per-agent instead of shared
    // (Agent::addBehavior()), this cache is this member's alone - it used to
    // be shared by every member using the same FormationBehavior instance,
    // recomputed by whichever one happened to run last. See Steering.cpp's
    // WanderBehavior comment for the same change.
    Radion::Agent* mSquadLeader = nullptr; // non-owning
    Radion::Agent* mPointMan = nullptr;    // non-owning

    glm::vec3 mPointManLook;
    glm::vec3 mPointManRight;
    glm::vec3 mLeaderLook;
    glm::vec3 mLeaderRight;
};

} // namespace Radion::AI

#endif // RADION_AI_FORMATIONBEHAVIOR_H
