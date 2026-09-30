// Offsets are on the XZ plane; a zero-length guard protects the final orientation build (NaNs).

#include "PCH.h"

#include "FormationBehavior.h"

#include "Agent.h"
#include "Scene.h"

namespace Radion::AI
{

using Radion::Agent;
using Radion::Scene;

namespace
{
const BehaviorParam kFormationParams[] = {
    {"Goal Radius", BehaviorParam::Kind::Float, 0.0f, 20.0f,
     "Goal radius handed to the point man - how close it has to get to its own slot."},
    {"Formation Radius", BehaviorParam::Kind::Float, 0.0f, 20.0f,
     "Goal radius handed to every other member - how close it has to get to its formation slot."},
};
} // namespace

FormationBehavior::FormationBehavior(float goalRadius, float formationRadius)
    : mGoalRadius(goalRadius), mFormationRadius(formationRadius)
{
}

u32 FormationBehavior::paramCount() const
{
    return static_cast<u32>(sizeof(kFormationParams) / sizeof(kFormationParams[0]));
}

const BehaviorParam& FormationBehavior::paramInfo(u32 index) const
{
    return kFormationParams[index];
}

f32 FormationBehavior::paramFloat(u32 index) const
{
    switch (index)
    {
    case 0: return mGoalRadius;
    case 1: return mFormationRadius;
    default: return 0.0f;
    }
}

void FormationBehavior::setParamFloat(u32 index, f32 value)
{
    switch (index)
    {
    case 0: mGoalRadius = value; break;
    case 1: mFormationRadius = value; break;
    default: break;
    }
}

void FormationBehavior::iterate(float timeDelta, Agent& entity)
{
    (void)timeDelta;

    // The player controls the squad leader, so it never forms up.
    if (entity.squadId() == 0)
        return;

    // Leader comes from this agent's squad link, not a scene scan (which picked another squad's leader).
    mSquadLeader = entity.squadLeader();
    mPointMan = nullptr;
    if (!mSquadLeader)
        return;

    for (Agent* member : mSquadLeader->squadMembers())
    {
        if (member && member->squadId() == 1)
        {
            mPointMan = member;
            break;
        }
    }

    if (!mPointMan || !mSquadLeader)
        return;

    Math::mat3 pointManBasis = Math::mat3_cast(mPointMan->orientation());
    mPointManLook = pointManBasis[2];
    mPointManRight = pointManBasis[0];
    Math::mat3 leaderBasis = Math::mat3_cast(mSquadLeader->orientation());
    mLeaderLook = leaderBasis[2];
    mLeaderRight = leaderBasis[0];

    Math::vec3 goal(0.0f);
    Math::vec3 dir(0.0f);
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
        goal = entity.goal();
        dir = goal - entity.position();
        break;
    }

    entity.setGoalRadius(mFormationRadius);
    mPointMan->setGoalRadius(mGoalRadius);

    dir.y = 0.0f;
    Math::vec3 up(0.0f, 1.0f, 0.0f);
    Math::vec3 lk = dir;
    float lkLen = Math::length(lk);
    // Near a slot the direction is float noise; rebuilding the quaternion would flip a stopped agent between headings.
    if (lkLen > 0.1f)
    {
        lk /= lkLen;
        Math::vec3 rt = Math::normalize(Math::cross(up, lk));
        Math::mat3 basis(rt, up, lk); // columns: right, up, forward
        entity.setGoal(goal);
        entity.setOrientation(Math::quat_cast(basis));
    }
    else
    {
        entity.setGoal(goal);
    }
}

void FormationBehavior::diamond(Agent& entity, Math::vec3& goal, Math::vec3& dir) const
{
    const Agent& squadmate = entity;
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

void FormationBehavior::abreast(Agent& entity, Math::vec3& goal, Math::vec3& dir) const
{
    const Agent& squadmate = entity;
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

    dir = goal - squadmate.position();
}

void FormationBehavior::singleFile(Agent& entity, Math::vec3& goal, Math::vec3& dir) const
{
    const Agent& squadmate = entity;
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

void FormationBehavior::pentagon(Agent& entity, Math::vec3& goal, Math::vec3& dir) const
{
    const Agent& squadmate = entity;

    // Rotate the leader's look 45 degrees about world up, in the leader's local frame (mirroring global X only works facing +Z).
    Math::vec3 v1 = Math::angleAxis(Math::radians(45.0f), Math::vec3(0.0f, 1.0f, 0.0f)) * mLeaderLook;
    Math::vec3 v2 = Math::angleAxis(Math::radians(-45.0f), Math::vec3(0.0f, 1.0f, 0.0f)) * mLeaderLook;

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

void FormationBehavior::wedge(Agent& entity, Math::vec3& goal, Math::vec3& dir) const
{
    const Agent& member = entity;
    const Math::vec3& leader = mSquadLeader->position();
    switch (member.squadId())
    {
    case 1: goal = leader + mLeaderLook * 25.0f; dir = mLeaderLook; break;
    case 2: goal = leader - mLeaderLook * 20.0f + mLeaderRight * 25.0f; dir = mLeaderLook; break;
    case 3: goal = leader - mLeaderLook * 20.0f - mLeaderRight * 25.0f; dir = mLeaderLook; break;
    case 4: goal = leader - mLeaderLook * 55.0f; dir = mLeaderLook; break;
    default: goal = member.goal(); dir = goal - member.position(); break;
    }
}

void FormationBehavior::vFormation(Agent& entity, Math::vec3& goal, Math::vec3& dir) const
{
    const Agent& member = entity;
    const Math::vec3& leader = mSquadLeader->position();
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

void FormationBehavior::circle(Agent& entity, Math::vec3& goal, Math::vec3& dir) const
{
    const Agent& member = entity;
    const float angle = Math::radians(90.0f * static_cast<float>(member.squadId() - 1));
    const Math::vec3 offset = mLeaderRight * (std::cos(angle) * 45.0f) +
                             mLeaderLook * (std::sin(angle) * 45.0f);
    goal = mSquadLeader->position() + offset;
    dir = mSquadLeader->position() - member.position();
}

} // namespace Radion::AI
