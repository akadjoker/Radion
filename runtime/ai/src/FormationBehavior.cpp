// FormationBehavior.cpp - formation placement for squad members.
//
// All offset arithmetic is done on the XZ plane; a zero-length guard protects
// the final orientation build (a zero dir would feed NaNs through).
//
// Pentagon/Diamond/Abreast/SingleFile are a faithful port of
// docs/ai/AI_Demo/Source/FormationBehavior.cpp (offsets verified line by
// line). Wedge, V and Circle have no counterpart there - that reference only
// defines the first four - they are original additions, not a port.
//
// Pentagon's mirrored flank direction deliberately does not match the
// reference: it computed v2 by negating v1's world-X component, which only
// mirrors correctly while the leader faces world +Z. Here v2 is the -45
// degree rotation of the leader's look (mirrored across the plane spanned by
// the look direction and the rotation axis), which is correct for any leader
// heading.

#include "PCH.h"

#include "FormationBehavior.h"

#include "Entity.h"
#include "Group.h"
#include "SquadEntity.h"
#include "World.h"

namespace Radion::AI
{

FormationBehavior::FormationBehavior(float goalRadius, float formationRadius)
    : mGoalRadius(goalRadius), mFormationRadius(formationRadius)
{
}

void FormationBehavior::iterate(float timeDelta, Entity& entity)
{
    (void)timeDelta;

    SquadEntity& squadmate = dynamic_cast<SquadEntity&>(entity);

    // The player controls the squad leader, so it never forms up.
    if (squadmate.squadId() == 0)
        return;

    mSquadLeader = nullptr;
    mPointMan = nullptr;

    // Cache pointers to the squad leader (id 0) and point man (id 1).
    for (Group* group : squadmate.world().groups())
    {
        for (Entity* e : group->entities())
        {
            SquadEntity* sm = dynamic_cast<SquadEntity*>(e);
            if (!sm)
                continue;
            if (sm->squadId() == 0)
                mSquadLeader = sm;
            if (sm->squadId() == 1)
                mPointMan = sm;
        }
    }

    if (!mPointMan || !mSquadLeader)
        return;

    // Orientation basis vectors: right = +X, up = +Y, forward (look) = +Z.
    Math::Mat3 pointManBasis = glm::mat3_cast(mPointMan->orientation());
    mPointManLook = pointManBasis[2];
    mPointManRight = pointManBasis[0];
    Math::Mat3 leaderBasis = glm::mat3_cast(mSquadLeader->orientation());
    mLeaderLook = leaderBasis[2];
    mLeaderRight = leaderBasis[0];

    Math::Vec3 goal(0.0f);
    Math::Vec3 dir(0.0f);
    switch (static_cast<SquadFormation>(mSquadLeader->squadFormation()))
    {
    case SquadFormation::Pentagon:
        pentagon(entity, goal, dir);
        break;
    case SquadFormation::Diamond:
        diamond(entity, goal, dir);
        break;
    case SquadFormation::Abreast:
        abreast(entity, goal, dir);
        break;
    case SquadFormation::SingleFile:
        singleFile(entity, goal, dir);
        break;
    case SquadFormation::Wedge:
        wedge(entity, goal, dir);
        break;
    case SquadFormation::V:
        vFormation(entity, goal, dir);
        break;
    case SquadFormation::Circle:
        circle(entity, goal, dir);
        break;
    default:
        goal = squadmate.goal();
        dir = goal - squadmate.position();
        break;
    }

    // The formation slot becomes the member's goal radius.
    squadmate.setGoalRadius(mFormationRadius);
    mPointMan->setGoalRadius(mGoalRadius);

    // Build the facing orientation from the formation direction (XZ only).
    dir.y = 0.0f;
    Math::Vec3 up(0.0f, 1.0f, 0.0f);
    Math::Vec3 lk = dir;
    float lkLen = glm::length(lk);
    // Near a formation slot the direction is dominated by floating-point
    // noise. Rebuilding the quaternion from that tiny vector makes a stopped
    // agent flip between two headings every frame.
    if (lkLen > 0.1f)
    {
        lk /= lkLen;
        Math::Vec3 rt = glm::normalize(glm::cross(up, lk));
        Math::Mat3 basis(rt, up, lk); // columns: right, up, forward
        squadmate.setGoal(goal);
        squadmate.setOrientation(glm::quat_cast(basis));
    }
    else
    {
        // No direction yet - keep the goal, leave the current orientation.
        squadmate.setGoal(goal);
    }
}

void FormationBehavior::diamond(Entity& entity, Math::Vec3& goal, Math::Vec3& dir) const
{
    const SquadEntity& squadmate = dynamic_cast<const SquadEntity&>(entity);
    switch (squadmate.squadId())
    {
    case 1: // point man
        goal = squadmate.goal();
        dir = goal - squadmate.position();
        break;
    case 2: // right flank
        goal = mPointMan->position() - (mPointManLook * 20.0f) + (mPointManRight * 30.0f);
        dir = mPointManRight;
        break;
    case 3: // left flank
        goal = mPointMan->position() - (mPointManLook * 20.0f) - (mPointManRight * 30.0f);
        dir = -mPointManRight;
        break;
    case 4: // rear guard
        goal = mPointMan->position() - (mPointManLook * 90.0f);
        dir = -mPointManLook;
        break;
    default: // anyone else just heads to the goal
        goal = squadmate.goal();
        dir = goal - squadmate.position();
        break;
    }
}

void FormationBehavior::abreast(Entity& entity, Math::Vec3& goal, Math::Vec3& dir) const
{
    const SquadEntity& squadmate = dynamic_cast<const SquadEntity&>(entity);
    switch (squadmate.squadId())
    {
    case 1: // point man
        goal = squadmate.goal();
        break;
    case 2: // right flank
        goal = mPointMan->position() + (mPointManRight * 40.0f);
        break;
    case 3: // left flank
        goal = mPointMan->position() + (mPointManRight * 85.0f);
        break;
    case 4: // rear guard
        goal = mPointMan->position() + (mPointManRight * 120.0f);
        break;
    default:
        goal = squadmate.goal();
        break;
    }

    // Everyone orients on the goal.
    dir = goal - squadmate.position();
}

void FormationBehavior::singleFile(Entity& entity, Math::Vec3& goal, Math::Vec3& dir) const
{
    const SquadEntity& squadmate = dynamic_cast<const SquadEntity&>(entity);
    switch (squadmate.squadId())
    {
    case 1: // point man
        goal = squadmate.goal();
        dir = goal - squadmate.position();
        break;
    case 2:
        goal = mPointMan->position() - (mPointManLook * 100.0f);
        dir = mPointManLook;
        break;
    case 3:
        goal = mPointMan->position() - (mPointManLook * 190.0f);
        dir = mPointManLook;
        break;
    case 4:
        goal = mPointMan->position() - (mPointManLook * 240.0f);
        dir = mPointManLook;
        break;
    default:
        goal = squadmate.goal();
        dir = goal - squadmate.position();
        break;
    }
}

void FormationBehavior::pentagon(Entity& entity, Math::Vec3& goal, Math::Vec3& dir) const
{
    const SquadEntity& squadmate = dynamic_cast<const SquadEntity&>(entity);

    // Rotate the leader's look by 45 degrees about the world up axis.  The
    // opposite diagonal must be rotated in the leader's local frame; mirroring
    // the global X component only works while the leader faces +Z.
    Math::Vec3 v1 = glm::angleAxis(glm::radians(45.0f), Math::Vec3(0.0f, 1.0f, 0.0f)) * mLeaderLook;
    Math::Vec3 v2 = glm::angleAxis(glm::radians(-45.0f), Math::Vec3(0.0f, 1.0f, 0.0f)) * mLeaderLook;

    switch (squadmate.squadId())
    {
    case 2: // right flank
        goal = mSquadLeader->position() - (mLeaderLook * 30.0f) + (mLeaderRight * 50.0f);
        dir = v1;
        break;
    case 3: // left flank
        goal = mSquadLeader->position() - (mLeaderLook * 30.0f) - (mLeaderRight * 50.0f);
        dir = v2;
        break;
    case 1: // point man (rear guard 2)
        goal = mSquadLeader->position() - (mLeaderLook * 90.0f) - (mLeaderRight * 25.0f);
        dir = -v1;
        break;
    case 4: // rear guard
        goal = mSquadLeader->position() - (mLeaderLook * 90.0f) + (mLeaderRight * 25.0f);
        dir = -v2;
        break;
    default:
        goal = squadmate.goal();
        dir = goal - squadmate.position();
        break;
    }
}

void FormationBehavior::wedge(Entity& entity, Math::Vec3& goal, Math::Vec3& dir) const
{
    const SquadEntity& member = dynamic_cast<const SquadEntity&>(entity);
    const Math::Vec3& leader = mSquadLeader->position();
    switch (member.squadId())
    {
    case 1: goal = leader + mLeaderLook * 25.0f; dir = mLeaderLook; break;
    case 2: goal = leader - mLeaderLook * 20.0f + mLeaderRight * 25.0f; dir = mLeaderLook; break;
    case 3: goal = leader - mLeaderLook * 20.0f - mLeaderRight * 25.0f; dir = mLeaderLook; break;
    case 4: goal = leader - mLeaderLook * 55.0f; dir = mLeaderLook; break;
    default: goal = member.goal(); dir = goal - member.position(); break;
    }
}

void FormationBehavior::vFormation(Entity& entity, Math::Vec3& goal, Math::Vec3& dir) const
{
    const SquadEntity& member = dynamic_cast<const SquadEntity&>(entity);
    const Math::Vec3& leader = mSquadLeader->position();
    switch (member.squadId())
    {
    case 1: goal = leader + mLeaderLook * 25.0f; break;
    case 2: goal = leader - mLeaderLook * 15.0f + mLeaderRight * 30.0f; break;
    case 3: goal = leader - mLeaderLook * 15.0f - mLeaderRight * 30.0f; break;
    case 4: goal = leader - mLeaderLook * 55.0f + mLeaderRight * 55.0f; break;
    default: goal = member.goal(); break;
    }
    dir = goal - member.position();
}

void FormationBehavior::circle(Entity& entity, Math::Vec3& goal, Math::Vec3& dir) const
{
    const SquadEntity& member = dynamic_cast<const SquadEntity&>(entity);
    const float angle = glm::radians(90.0f * static_cast<float>(member.squadId() - 1));
    const Math::Vec3 offset = mLeaderRight * (std::cos(angle) * 45.0f) +
                             mLeaderLook * (std::sin(angle) * 45.0f);
    goal = mSquadLeader->position() + offset;
    dir = mSquadLeader->position() - member.position();
}

} // namespace Radion::AI
