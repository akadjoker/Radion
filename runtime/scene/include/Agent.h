#ifndef RADION_AGENT_H
#define RADION_AGENT_H

// Agent.h - a flocking/steering AI agent, one per GameObject.
//
// Ported from runtime/ai's AI::Entity + AI::SquadEntity + AI::SquadLeaderEntity
// (now gone): the whole role hierarchy collapses into one Component, with a
// squadId() of 0 marking the leader (SquadLeaderEntity's member list/POIs/
// command state lives here, guarded by that convention, until Squad exists).

#include "Component.h"
#include "FormationBehavior.h" // AI::SquadFormation
#include "SquadEntity.h"       // AI::SquadCommand
#include "WaypointNetwork.h"   // AI::Path, AI::WaypointID

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

namespace Radion::AI
{
class Behavior;
class PointOfInterest;
class PointsOfInterest;
class StateMachine;
class Waypoint;
class WaypointNetwork;
} // namespace Radion::AI

namespace Radion
{

class Scene;
class Agent;

using AgentType = Radion::u32;

// Sense data entry: a visible agent plus its distance, kept in
// Agent::mVisibleGroupMembers / mVisibleEnemies sorted ascending by distance.
struct EntityDist
{
    f32 distance = 0.0f;
    Agent* entity = nullptr;
};

class Agent final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Agent;

    struct Settings
    {
        AgentType type = 0;
        f32 senseRange = 4.0f;        // radius in which others are "sensed"
        f32 maxVelocityChange = 1.0f; // max acceleration per update()
        f32 maxSpeed = 5.0f;
        f32 desiredSpeed = 2.0f;
        f32 radius = 0.5f;      // bounding-sphere radius
        f32 moveXScalar = 1.0f; // per-axis velocity scalars (0 = locked)
        f32 moveYScalar = 0.0f;
        f32 moveZScalar = 1.0f;
    };

    ~Agent() override;

    // Replaces the Entity(World&, Settings&) constructor: the Agent already
    // exists (GameObject::addComponent<Agent>() built it), this just applies
    // the tuning.
    void applySettings(const Settings& settings);

    // Stepped by Scene::update() for every simulating agent - sensing,
    // behaviors and the steering integration in one pass.
    void update(f32 deltaTime);

    void setFriendMask(AgentType mask)
    {
        mFriendMask = mask;
        mEnemyMask = ~mask;
    }
    AgentType friendMask() const
    {
        return mFriendMask;
    }
    AgentType enemyMask() const
    {
        return mEnemyMask;
    }
    AgentType type() const
    {
        return mAgentType;
    }

    // Sense data refreshed each update(); sorted ascending by distance. Both
    // lists only ever hold living agents - a dead one stops being sensed by
    // anyone the same frame its health reaches zero.
    const std::vector<EntityDist>& visibleGroupMembers() const
    {
        return mVisibleGroupMembers;
    }
    const std::vector<EntityDist>& visibleEnemies() const
    {
        return mVisibleEnemies;
    }

    // Combat: plain hit points, no regen. update() freezes a dead agent in
    // place (skips sensing, behaviors and movement) rather than removing it
    // from the Scene's list, so a caller can still find it, play a death
    // pose on it, or hide its GameObject.
    f32 health() const
    {
        return mHealth;
    }
    void setHealth(f32 health)
    {
        mHealth = glm::max(health, 0.0f);
    }
    bool alive() const
    {
        return mHealth > 0.0f;
    }
    void applyDamage(f32 amount)
    {
        mHealth = glm::max(mHealth - amount, 0.0f);
    }

    // CombatBehavior's own per-agent cooldown between shots - kept here with
    // the rest of the agent's steering state (mDesiredMoveVector and the
    // sense lists) rather than inside the Behavior, which is where it lived
    // back when one Behavior instance served a whole flock.
    f32 attackCooldown() const
    {
        return mAttackCooldown;
    }
    void setAttackCooldown(f32 seconds)
    {
        mAttackCooldown = glm::max(seconds, 0.0f);
    }
    void tickAttackCooldown(f32 timeDelta)
    {
        mAttackCooldown = glm::max(mAttackCooldown - timeDelta, 0.0f);
    }

    // One shot's worth of event state, valid for the frame CombatBehavior
    // actually fires and cleared at the top of the next update() - a caller
    // polls this once per frame (after Scene::update()) to trigger a muzzle
    // flash / tracer / fire animation without CombatBehavior needing to know
    // anything about rendering.
    bool firedThisFrame() const
    {
        return mFiredThisFrame;
    }
    Agent* lastFireTarget() const
    {
        return mLastFireTarget;
    }
    void markFired(Agent* target)
    {
        mFiredThisFrame = true;
        mLastFireTarget = target;
    }

    // Behaviors are owned: the destructor and clearBehaviors()/
    // removeBehavior() delete them. addBehavior() takes a heap-allocated
    // instance, same ownership-by-reference convention World::add(Group&)/
    // Group::add(Entity&) used.
    void addBehavior(AI::Behavior& behavior);
    bool removeBehavior(AI::Behavior& behavior);
    void clearBehaviors();
    usize behaviorCount() const;
    AI::Behavior* behaviorAt(usize index) const;

    void setGroupId(u32 id)
    {
        mGroupId = id;
    }
    u32 groupId() const
    {
        return mGroupId;
    }

    // Scene the agent is registered with, for behaviors that need to scan
    // every agent (FormationBehavior's leader/point-man lookup, the
    // Pathfind/NavMesh avoidance passes) - mirrors RigidBody::scene().
    Scene* scene() const
    {
        return mScene;
    }

    const glm::vec3& position() const
    {
        return mPosition;
    }
    void setPosition(const glm::vec3& position)
    {
        mPosition = position;
    }
    const glm::vec3& velocity() const
    {
        return mVelocity;
    }
    void setVelocity(const glm::vec3& velocity)
    {
        mVelocity = velocity;
    }
    const glm::quat& orientation() const
    {
        return mOrientation;
    }
    void setOrientation(const glm::quat& orientation)
    {
        mOrientation = orientation;
    }
    const glm::vec3& desiredMove() const
    {
        return mDesiredMoveVector;
    }
    void setDesiredMove(const glm::vec3& move)
    {
        mDesiredMoveVector = move;
    }

    f32 maxSpeed() const
    {
        return mMaxSpeed;
    }
    // Read every update() (Behavior.cpp's own speed-matching and flee
    // terms), never cached - safe to change on a live agent, not just at
    // construction through Settings.
    void setMaxSpeed(f32 speed)
    {
        mMaxSpeed = glm::max(speed, 0.0f);
    }
    f32 desiredSpeed() const
    {
        return mDesiredSpeed;
    }
    void setDesiredSpeed(f32 speed)
    {
        mDesiredSpeed = glm::max(speed, 0.0f);
    }
    f32 senseRange() const
    {
        return mSenseRange;
    }

    // ---- vehicle interface --------------------------------------------------
    // Local frame convention (consistent with FormationBehavior): right = +X,
    // up = +Y, forward (look) = +Z, all read from the orientation quaternion.
    // This is the reference demos' convention (D3DX-era: forward = +Z), kept
    // as-is because the AI's own math is internally self-consistent and
    // tested against it.
    //
    // It does NOT match the rest of runtime/scene: GameObject::forward() is
    // rotation * (0,0,-1) - forward = -Z (right() and up() do agree: +X,
    // +Y). pushOwnerPose()/pullAgentPose() apply the 180 degree turn around
    // up() that reconciles the two conventions - see their comments.

    glm::vec3 forward() const
    {
        return glm::mat3_cast(mOrientation)[2];
    }
    glm::vec3 side() const
    {
        return glm::mat3_cast(mOrientation)[0];
    }
    glm::vec3 up() const
    {
        return glm::mat3_cast(mOrientation)[1];
    }

    // Velocity is a free vector; speed() is its magnitude and setSpeed()
    // rescales it.
    f32 speed() const
    {
        return glm::length(mVelocity);
    }
    void setSpeed(f32 newSpeed);

    f32 radius() const
    {
        return mRadius;
    }
    void setRadius(f32 radius)
    {
        mRadius = radius;
    }

    // maxForce() is the steering-force limit; it maps to the acceleration
    // limit maxVelocityChange that update() clamps against.
    f32 maxForce() const
    {
        return mMaxVelocityChange;
    }
    void setMaxForce(f32 force)
    {
        mMaxVelocityChange = force;
    }

    // Predicted position in `predictionTime` seconds (straight-line
    // extrapolation).
    glm::vec3 predictFuturePosition(f32 predictionTime) const
    {
        return mPosition + (mVelocity * predictionTime);
    }

    // Transform helpers in the vehicle's local frame.
    glm::vec3 localizeDirection(const glm::vec3& globalDirection) const;
    glm::vec3 localizePosition(const glm::vec3& globalPosition) const;
    glm::vec3 globalizePosition(const glm::vec3& localPosition) const;
    glm::vec3 globalizeDirection(const glm::vec3& localDirection) const;

    // Rotate the orientation so forward() points along the current velocity,
    // keeping up as close as possible.
    void alignWithVelocity();

    // ---- squad-member state (ported from AI::SquadEntity) -------------------

    void setPath(const AI::Path& path)
    {
        mPath = path;
    }
    AI::Path& path()
    {
        return mPath;
    }
    const AI::Path& path() const
    {
        return mPath;
    }

    void setGoal(const glm::vec3& goal)
    {
        mGoalPosition = goal;
    }
    const glm::vec3& goal() const
    {
        return mGoalPosition;
    }

    void setNextWaypoint(AI::WaypointID wp)
    {
        mNextWaypoint = wp;
    }
    AI::WaypointID nextWaypoint() const
    {
        return mNextWaypoint;
    }
    void setCurrentWaypoint(AI::WaypointID wp)
    {
        mCurrentWaypoint = wp;
    }
    AI::WaypointID currentWaypoint() const
    {
        return mCurrentWaypoint;
    }

    void setWaypointNetwork(AI::WaypointNetwork* network)
    {
        mWaypointNetwork = network;
    }
    AI::WaypointNetwork* waypointNetwork() const
    {
        return mWaypointNetwork;
    }

    bool hasValidWaypoint() const
    {
        return mNextWaypoint != 0;
    }
    bool hasValidPath() const
    {
        return !mPath.empty();
    }

    bool waypointReached();
    bool goalReached();
    void onWaypointReached();
    void onGoalReached();
    void onWaitingForCommand();

    // Owned (unlike AI::SquadEntity::setStateMachine(), which was not):
    // replacing or destroying the Agent deletes the previous machine, so a
    // caller that builds one (SquadAI.h) hands it over and never deletes it
    // itself.
    void setStateMachine(AI::StateMachine* machine);
    AI::StateMachine* stateMachine() const
    {
        return mStateMachine;
    }

    void resetTimeSinceWaypointReached()
    {
        mTimeSinceNextWaypointReached = 0.0f;
    }
    void incrementTimeSinceWaypointReached(f32 deltaTime)
    {
        mTimeSinceNextWaypointReached += deltaTime;
    }
    f32 timeSinceWaypointReached() const
    {
        return mTimeSinceNextWaypointReached;
    }
    void resetTimeSinceGoalReached()
    {
        mTimeSinceGoalReached = 0.0f;
    }
    void incrementTimeSinceGoalReached(f32 deltaTime)
    {
        mTimeSinceGoalReached += deltaTime;
    }
    f32 timeSinceGoalReached() const
    {
        return mTimeSinceGoalReached;
    }
    void resetTimeSinceLOSTest()
    {
        mTimeSinceLOSTested = 0.0f;
    }
    void incrementTimeSinceLOSTest(f32 deltaTime)
    {
        mTimeSinceLOSTested += deltaTime;
    }
    f32 timeSinceLOSTest() const
    {
        return mTimeSinceLOSTested;
    }

    void setLOSStatus(bool status)
    {
        mLOSStatus = status;
    }
    bool losStatus() const
    {
        return mLOSStatus;
    }

    // -1 = no squad. 0 identifies the leader (player-controlled; skips
    // formation and owns the member list/POI/command state below) until
    // Squad exists as its own Component.
    void setSquadId(int id)
    {
        mSquadId = id;
    }
    int squadId() const
    {
        return mSquadId;
    }

    void setGoalRadius(f32 radius)
    {
        mGoalRadius = radius;
    }
    f32 goalRadius() const
    {
        return mGoalRadius;
    }

    // Formation is the squad's, not the soldier's - stays here only until
    // Squad exists to own it (Phase 3).
    void setSquadFormation(int formation)
    {
        mSquadFormation = formation;
    }
    int squadFormation() const
    {
        return mSquadFormation;
    }

    // On the leader (squadId() == 0) this also dispatches the command to
    // every squad member and, for AttackTarget, sends the squad at the
    // selected point of interest - AI::SquadLeaderEntity::setCommand()
    // folded into one method, dispatching on squadId() instead of a vtable.
    void setCommand(AI::SquadCommand command);
    AI::SquadCommand command() const
    {
        return mCommand;
    }

    // ---- leader-only state (ported from AI::SquadLeaderEntity) ---------------

    bool hasCommandChanged() const
    {
        return !mCommandAcknowledged || mCommand != mLastCommand;
    }
    void acknowledgeCommand()
    {
        mLastCommand = mCommand;
        mCommandAcknowledged = true;
    }

    void sendSquadToRandomPOI();
    void sendSquadToRandomWaypoint();
    void commandSquadToRallyOnLeader();
    void sendSquadToTarget();

    void setSelectedPointOfInterest(AI::PointOfInterest* poi)
    {
        mSelectedPointOfInterest = poi;
    }
    AI::PointOfInterest* selectedPointOfInterest() const
    {
        return mSelectedPointOfInterest;
    }
    void setSelectedWaypoint(AI::Waypoint* wp)
    {
        mSelectedWaypoint = wp;
    }
    AI::Waypoint* selectedWaypoint() const
    {
        return mSelectedWaypoint;
    }
    void setPointsOfInterest(AI::PointsOfInterest* pois)
    {
        mPointsOfInterest = pois;
    }
    AI::PointsOfInterest* pointsOfInterest() const
    {
        return mPointsOfInterest;
    }

    // Squad members are NOT owned by the leader.
    void addSquadMember(Agent* member)
    {
        mSquadMembers.push_back(member);
    }
    void removeSquadMember(Agent* member);
    void clearSquadMembers()
    {
        mSquadMembers.clear();
    }
    std::vector<Agent*>& squadMembers()
    {
        return mSquadMembers;
    }
    const std::vector<Agent*>& squadMembers() const
    {
        return mSquadMembers;
    }

    // ---- pose sync with the owning GameObject --------------------------------

    void setSyncPosition(bool sync)
    {
        mSyncPosition = sync;
    }
    bool syncPosition() const
    {
        return mSyncPosition;
    }
    void setSyncRotation(bool sync)
    {
        mSyncRotation = sync;
    }
    bool syncRotation() const
    {
        return mSyncRotation;
    }

private:
    friend class GameObject;
    friend class Scene;

    Agent();

    void updateVisibility();
    bool visibilityTest(const Agent& other, f32& dist) const;

    bool simulating() const;
    void pushOwnerPose();
    bool ownerMoved() const;
    void pullAgentPose();

    Scene* mScene = nullptr;
    u32 mGroupId = 0;
    std::vector<AI::Behavior*> mBehaviors; // owned
    AgentType mFriendMask = 0;
    AgentType mEnemyMask = ~AgentType(0);
    AgentType mAgentType = 0;

    glm::vec3 mPosition = glm::vec3(0.0f);
    glm::vec3 mVelocity = glm::vec3(0.0f);
    glm::quat mOrientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    glm::vec3 mDesiredMoveVector = glm::vec3(0.0f);

    f32 mSenseRange = 4.0f;
    f32 mMaxVelocityChange = 1.0f;
    f32 mMaxSpeed = 5.0f;
    f32 mDesiredSpeed = 2.0f;
    f32 mRadius = 0.5f;
    f32 mMoveXScalar = 1.0f;
    f32 mMoveYScalar = 0.0f;
    f32 mMoveZScalar = 1.0f;

    std::vector<EntityDist> mVisibleGroupMembers; // sorted by distance
    std::vector<EntityDist> mVisibleEnemies;      // sorted by distance

    f32 mHealth = 100.0f;
    f32 mAttackCooldown = 0.0f;
    bool mFiredThisFrame = false;
    Agent* mLastFireTarget = nullptr;

    // Squad-member state (AI::SquadEntity).
    AI::WaypointNetwork* mWaypointNetwork = nullptr;
    AI::WaypointID mNextWaypoint = 0;
    AI::WaypointID mCurrentWaypoint = 0;
    glm::vec3 mGoalPosition = glm::vec3(0.0f);
    AI::Path mPath;
    AI::StateMachine* mStateMachine = nullptr; // owned
    AI::SquadCommand mCommand = AI::SquadCommand::PatrolPointsOfInterest;
    bool mLOSStatus = false;
    f32 mTimeSinceNextWaypointReached = 0.0f;
    f32 mTimeSinceGoalReached = 0.0f;
    f32 mTimeSinceLOSTested = 100.0f; // first LOS test fires immediately
    f32 mGoalRadius = 25.0f;
    int mSquadId = -1;
    int mSquadFormation = static_cast<int>(AI::SquadFormation::Abreast);

    // Leader-only state (AI::SquadLeaderEntity), valid when squadId() == 0.
    std::vector<Agent*> mSquadMembers; // non-owning
    AI::PointsOfInterest* mPointsOfInterest = nullptr;
    AI::PointOfInterest* mSelectedPointOfInterest = nullptr;
    AI::Waypoint* mSelectedWaypoint = nullptr;
    AI::SquadCommand mLastCommand = AI::SquadCommand::PatrolPointsOfInterest;
    bool mCommandAcknowledged = false;

    bool mSyncPosition = true;
    bool mSyncRotation = true;
    glm::vec3 mSyncedPosition = glm::vec3(0.0f);
    glm::quat mSyncedRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
};

} // namespace Radion

#endif // RADION_AGENT_H
