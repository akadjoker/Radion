#ifndef RADION_AI_FORMATIONBEHAVIOR_H
#define RADION_AI_FORMATIONBEHAVIOR_H

// FormationBehavior.h - places squad members into a formation around the
// squad leader and point man.
//
// Requires the owning entity to be a SquadEntity (dynamic_cast) with
// squadId() != 0 (the leader, id 0, is player controlled and skips the
// formation).
//
// Orientation convention: the local frame is right = +X, up = +Y,
// forward (look) = +Z.

#include "Behavior.h"

#include <glm/glm.hpp>

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

class SquadEntity;

class FormationBehavior final : public Behavior
{
public:
    FormationBehavior(float goalRadius, float formationRadius);

    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Formation Behavior";
    }

private:
    void singleFile(Entity& entity, Math::Vec3& goal, Math::Vec3& dir) const;
    void abreast(Entity& entity, Math::Vec3& goal, Math::Vec3& dir) const;
    void diamond(Entity& entity, Math::Vec3& goal, Math::Vec3& dir) const;
    void pentagon(Entity& entity, Math::Vec3& goal, Math::Vec3& dir) const;
    void wedge(Entity& entity, Math::Vec3& goal, Math::Vec3& dir) const;
    void vFormation(Entity& entity, Math::Vec3& goal, Math::Vec3& dir) const;
    void circle(Entity& entity, Math::Vec3& goal, Math::Vec3& dir) const;

    float mGoalRadius;
    float mFormationRadius;

    SquadEntity* mSquadLeader = nullptr; // non-owning
    SquadEntity* mPointMan = nullptr;    // non-owning

    Math::Vec3 mPointManLook;
    Math::Vec3 mPointManRight;
    Math::Vec3 mLeaderLook;
    Math::Vec3 mLeaderRight;
};

} // namespace Radion::AI

#endif // RADION_AI_FORMATIONBEHAVIOR_H
