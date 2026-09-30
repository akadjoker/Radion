#ifndef RADION_AGENT_H
#define RADION_AGENT_H

#include "BehaviorFactory.h"
#include "Component.h"
#include "FormationBehavior.h"
#include "SquadEntity.h"
#include "WaypointNetwork.h"

#include "Math.h"
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
        f32 senseRange = 4.0f;
        f32 maxVelocityChange = 1.0f;
        f32 maxSpeed = 5.0f;
        f32 desiredSpeed = 2.0f;
        f32 radius = 0.5f;
        f32 moveXScalar = 1.0f;
        f32 moveYScalar = 0.0f;
        f32 moveZScalar = 1.0f;
    };

    ~Agent() override;

    void applySettings(const Settings& settings);

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
    // Leaves friend/enemy masks alone, unlike applySettings().
    void setType(AgentType type)
    {
        mAgentType = type;
    }

    // Sorted ascending by distance; only living agents are held.
    const std::vector<EntityDist>& visibleGroupMembers() const
    {
        return mVisibleGroupMembers;
    }
    const std::vector<EntityDist>& visibleEnemies() const
    {
        return mVisibleEnemies;
    }

    // update() freezes a dead agent in place rather than removing it from the Scene's list.
    f32 health() const
    {
        return mHealth;
    }
    void setHealth(f32 health)
    {
        mHealth = Math::max(health, 0.0f);
    }
    bool alive() const
    {
        return mHealth > 0.0f;
    }
    void applyDamage(f32 amount)
    {
        mHealth = Math::max(mHealth - amount, 0.0f);
    }

    f32 attackCooldown() const
    {
        return mAttackCooldown;
    }
    void setAttackCooldown(f32 seconds)
    {
        mAttackCooldown = Math::max(seconds, 0.0f);
    }
    void tickAttackCooldown(f32 timeDelta)
    {
        mAttackCooldown = Math::max(mAttackCooldown - timeDelta, 0.0f);
    }

    // Valid for the frame CombatBehavior fires; cleared at the top of the next update().
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

    // Takes ownership; null when the agent refuses the behavior.
    template <class T, class... Args> T* addBehavior(Args&&... args)
    {
        T* behavior = new T(static_cast<Args&&>(args)...);
        if (adoptBehavior(behavior))
            return behavior;
        // Freshly built, so deleting on refusal cannot strand anything.
        delete behavior;
        return nullptr;
    }

    AI::Behavior* addBehavior(AI::BehaviorType type);

    // Takes ownership. False, behavior untouched, when it already belongs to an agent (which still frees it).
    bool adoptBehavior(AI::Behavior* behavior);

    bool removeBehavior(AI::Behavior& behavior);
    bool removeBehavior(AI::BehaviorType type);
    AI::Behavior* behavior(AI::BehaviorType type) const;
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

    Scene* scene() const
    {
        return mScene;
    }

    const Math::vec3& position() const
    {
        return mPosition;
    }
    void setPosition(const Math::vec3& position)
    {
        mPosition = position;
    }
    const Math::vec3& velocity() const
    {
        return mVelocity;
    }
    void setVelocity(const Math::vec3& velocity)
    {
        mVelocity = velocity;
    }
    const Math::quat& orientation() const
    {
        return mOrientation;
    }
    void setOrientation(const Math::quat& orientation)
    {
        mOrientation = orientation;
    }
    const Math::vec3& desiredMove() const
    {
        return mDesiredMoveVector;
    }
    void setDesiredMove(const Math::vec3& move)
    {
        mDesiredMoveVector = move;
    }

    f32 maxSpeed() const
    {
        return mMaxSpeed;
    }
    void setMaxSpeed(f32 speed)
    {
        mMaxSpeed = Math::max(speed, 0.0f);
    }
    f32 desiredSpeed() const
    {
        return mDesiredSpeed;
    }
    void setDesiredSpeed(f32 speed)
    {
        mDesiredSpeed = Math::max(speed, 0.0f);
    }
    f32 senseRange() const
    {
        return mSenseRange;
    }
    void setSenseRange(f32 range)
    {
        mSenseRange = Math::max(range, 0.0f);
    }

    // 0 locks an axis (Y by default), 1 leaves it free; above 1 destabilises the integration.
    f32 moveXScalar() const
    {
        return mMoveXScalar;
    }
    f32 moveYScalar() const
    {
        return mMoveYScalar;
    }
    f32 moveZScalar() const
    {
        return mMoveZScalar;
    }
    void setMoveScalars(f32 x, f32 y, f32 z)
    {
        mMoveXScalar = x;
        mMoveYScalar = y;
        mMoveZScalar = z;
    }

    // Local frame: right = +X, up = +Y, forward = +Z. Unlike GameObject::forward() (-Z);
    // pushOwnerPose()/pullAgentPose() apply the 180 degree turn between the two.

    Math::vec3 forward() const
    {
        return Math::mat3_cast(mOrientation)[2];
    }
    Math::vec3 side() const
    {
        return Math::mat3_cast(mOrientation)[0];
    }
    Math::vec3 up() const
    {
        return Math::mat3_cast(mOrientation)[1];
    }

    f32 speed() const
    {
        return Math::length(mVelocity);
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

    f32 maxForce() const
    {
        return mMaxVelocityChange;
    }
    void setMaxForce(f32 force)
    {
        mMaxVelocityChange = force;
    }

    Math::vec3 predictFuturePosition(f32 predictionTime) const
    {
        return mPosition + (mVelocity * predictionTime);
    }

    Math::vec3 localizeDirection(const Math::vec3& globalDirection) const;
    Math::vec3 localizePosition(const Math::vec3& globalPosition) const;
    Math::vec3 globalizePosition(const Math::vec3& localPosition) const;
    Math::vec3 globalizeDirection(const Math::vec3& localDirection) const;

    void alignWithVelocity();

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

    void setGoal(const Math::vec3& goal)
    {
        mGoalPosition = goal;
    }
    const Math::vec3& goal() const
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

    // Owned: replacing or destroying the Agent deletes the previous machine.
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

    // -1 = no squad, 0 = the leader (owns member list/POI/command state).
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

    void setSquadFormation(int formation)
    {
        mSquadFormation = formation;
    }
    int squadFormation() const
    {
        return mSquadFormation;
    }

    // On the leader (squadId() == 0) also dispatches the command to every squad member.
    void setCommand(AI::SquadCommand command);
    AI::SquadCommand command() const
    {
        return mCommand;
    }

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

    // Members are not owned by the leader; the link is recorded on both ends so either side's destruction unlinks cleanly.
    void addSquadMember(Agent* member);
    void removeSquadMember(Agent* member);
    void clearSquadMembers();
    Agent* squadLeader() const
    {
        return mSquadLeader;
    }
    std::vector<Agent*>& squadMembers()
    {
        return mSquadMembers;
    }
    const std::vector<Agent*>& squadMembers() const
    {
        return mSquadMembers;
    }

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

    Math::vec3 mPosition = Math::vec3(0.0f);
    Math::vec3 mVelocity = Math::vec3(0.0f);
    Math::quat mOrientation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    Math::vec3 mDesiredMoveVector = Math::vec3(0.0f);

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

    AI::WaypointNetwork* mWaypointNetwork = nullptr;
    AI::WaypointID mNextWaypoint = 0;
    AI::WaypointID mCurrentWaypoint = 0;
    Math::vec3 mGoalPosition = Math::vec3(0.0f);
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

    std::vector<Agent*> mSquadMembers; // non-owning, back-linked by mSquadLeader
    Agent* mSquadLeader = nullptr;     // the agent whose mSquadMembers holds this one
    AI::PointsOfInterest* mPointsOfInterest = nullptr;
    AI::PointOfInterest* mSelectedPointOfInterest = nullptr;
    AI::Waypoint* mSelectedWaypoint = nullptr;
    AI::SquadCommand mLastCommand = AI::SquadCommand::PatrolPointsOfInterest;
    bool mCommandAcknowledged = false;

    bool mSyncPosition = true;
    bool mSyncRotation = true;
    Math::vec3 mSyncedPosition = Math::vec3(0.0f);
    Math::quat mSyncedRotation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
};

} // namespace Radion

#endif // RADION_AGENT_H
