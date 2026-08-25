#ifndef RADION_PHYSICS_DYNAMICS_PHYSICSWORLD_H
#define RADION_PHYSICS_DYNAMICS_PHYSICSWORLD_H

#include "Containers.h"
#include "collision/Broadphase.h"
#include "collision/CollisionFilter.h"
#include "dynamics/ContactSolver.h"
#include "dynamics/RigidBody.h"

#include <vector>

namespace Radion::Physics
{

class CollisionShape;
class Joint;

struct BodyHandle
{
    u32 id = 0xFFFFFFFFu;
    u32 generation = 0;

    bool valid() const
    {
        return id != 0xFFFFFFFFu && generation != 0;
    }
};

// What happened to a pair this step. The three are what a component's
// onCollisionEnter/Stay/Exit are built from, and they fall out of the same
// per-pair bookkeeping warm starting already needs - the cache that carries
// impulses from one step to the next is exactly the record of which pairs
// were touching last step.
enum class ContactEvent : u8
{
    Enter,
    Stay,
    Exit
};

struct ContactEventInfo
{
    u32 bodyA = 0;
    u32 bodyB = 0;
    ContactEvent event = ContactEvent::Enter;
    // Empty for Exit - by then there is no contact left to describe.
    glm::vec3 normal{0.0f};
    glm::vec3 point{0.0f};
    f32 penetration = 0.0f;
};

struct WorldRayHit
{
    u32 body = 0xFFFFFFFFu;
    glm::vec3 point{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    f32 distance = 0.0f;
};

// A body and the shape it collides with. The world does not own either: they
// live wherever the caller keeps them, which in the engine is a component on
// a GameObject.
struct BodyEntry
{
    RigidBody* body = nullptr;
    CollisionShape* shape = nullptr;
    CollisionFilter filter;
    f32 friction = 0.5f;
    f32 restitution = 0.0f;
    bool enabled = true;
};

// Drives one simulation step end to end: broadphase, narrowphase, contact
// cache, solver, integrate. Deliberately small and free of any scene
// knowledge - in the engine the Scene owns the bodies and answers the
// broadphase question with the octree it already keeps, and this is what
// that arrangement replaces piece by piece.
class PhysicsWorld
{
public:
    using EventCallback = void (*)(const ContactEventInfo& info, void* userData);
    // Called once per fixed step, after that step's velocities have been
    // integrated into position - the same point in the loop where an
    // action like a vehicle updates itself against the freshly moved world.
    using StepCallback = void (*)(f32 step, void* userData);

    PhysicsWorld();

    // Returns the stable id used by pairs and events. Storage is dense:
    // removal swap-pops an internal slot, while this id continues to name the
    // same body through the id-to-slot table.
    u32 addBody(const BodyEntry& entry);
    void removeBody(u32 id);
    void clear();

    void addJoint(Joint* joint);
    void removeJoint(Joint* joint);
    usize jointCount() const
    {
        return mJoints.size();
    }

    BodyEntry* body(u32 id);
    const BodyEntry* body(u32 id) const;
    BodyHandle bodyHandle(u32 id) const;
    BodyEntry* body(BodyHandle handle);
    const BodyEntry* body(BodyHandle handle) const;
    usize bodyCount() const
    {
        return mBodies.size();
    }

    void setGravity(const glm::vec3& gravity);
    const glm::vec3& gravity() const
    {
        return mGravity;
    }

    // Seconds per simulation step. Physics runs at this rate whatever the
    // frame rate: a solver tuned at one step size behaves differently at
    // another, and a stack that stands at 120 Hz can collapse at 30.
    void setFixedStep(f32 seconds);
    f32 fixedStep() const
    {
        return mFixedStep;
    }

    void setSolverSettings(const ContactSolverSettings& settings)
    {
        mSolver.setSettings(settings);
    }

    void setEventCallback(EventCallback callback, void* userData);
    void setStepCallback(StepCallback callback, void* userData);

    // How many steps a pair may go without a manifold before it counts as
    // separated. One is not enough: the step a body lands on, the position
    // pass pushes it out far enough to clear the surface for a single step,
    // and a pair that broke there would report Exit and then Enter again for
    // one bounce - and lose its accumulated impulses with it, which is the
    // stack sagging by one step's worth every landing.
    void setContactPersistence(u32 steps);
    u32 contactPersistence() const
    {
        return mContactPersistence;
    }

    // Distance within which two shapes still count as in contact, with a
    // negative penetration. See Narrowphase::collide - it is what stops a
    // pair dying every time the position pass lifts a body clear of what it
    // is resting on.
    void setContactMargin(f32 margin);
    f32 contactMargin() const
    {
        return mContactMargin;
    }

    // Consumes `deltaTime` in fixed steps, keeping the remainder for next
    // time. More than `maxStepsPerUpdate` worth of backlog is dropped rather
    // than simulated: a long stall would otherwise be paid off with a burst
    // of steps that stalls again, and the frame rate never recovers.
    void update(f32 deltaTime);
    // One step, exactly. What the tests drive.
    void step(f32 duration);

    void debugDraw() const;

    bool raycast(const Ray& ray, f32 maxDistance, const QueryFilter& filter,
                 WorldRayHit& hit) const;
    void overlapSphere(const glm::vec3& centre, f32 radius, const QueryFilter& filter,
                       std::vector<u32>& out) const;
    void queryAABB(const AABB& bounds, const QueryFilter& filter, std::vector<u32>& out) const;

    // Lightweight area effects. Radial strength is signed: positive pushes
    // away (explosion), negative pulls inward (attraction). Effects decay
    // linearly to zero at radius and only affect dynamic bodies accepted by
    // the query filter. An impulse is instantaneous; forces must be added
    // before step() and are integrated during that step.
    u32 applyRadialImpulse(const glm::vec3& centre, f32 radius, f32 strength,
                           const QueryFilter& filter = QueryFilter());
    u32 addRadialForce(const glm::vec3& centre, f32 radius, f32 strength,
                       const QueryFilter& filter = QueryFilter());
    u32 addDirectionalForce(const glm::vec3& centre, f32 radius, const glm::vec3& force,
                            const QueryFilter& filter = QueryFilter());

    usize contactCount() const
    {
        return mContacts.size();
    }
    // Pairs that were touching after the last step.
    const std::vector<Contact>& contacts() const
    {
        return mContacts;
    }

private:
    static constexpr u32 InvalidIndex = 0xFFFFFFFFu;

    struct PendingAdd
    {
        u32 id = InvalidIndex;
        BodyEntry entry;
    };

    struct CachedPoint
    {
        glm::vec3 position{0.0f};
        f32 normalImpulse = 0.0f;
        f32 tangentImpulse[2] = {0.0f, 0.0f};
    };
    struct CachedPair
    {
        // Room for several manifolds, not one: a convex against a trimesh
        // makes one per triangle it touches, and they all belong to the same
        // pair. warmStartFromCache() matches points by position rather than
        // by slot, so a shared pool needs no per-manifold bookkeeping.
        static constexpr u32 MaxPoints = ContactManifold::MaxPoints * 4;
        CachedPoint points[MaxPoints];
        u32 count = 0;
        u32 lastStep = 0;
        bool reported = false;
    };

    static u64 pairKey(u32 a, u32 b);
    // Bodies joined by contacts have to sleep as one. Decided per body, a
    // stack freezes half-settled: the boxes on top stop moving first and go
    // to sleep, a sleeping body does not integrate, and it hangs in the air
    // while the ones underneath are still sinking their last millimetres.
    void propagateSleep();
    u32 islandRoot(u32 index);
    void warmStartFromCache(u32 a, u32 b, ContactManifold& manifold);
    void storeInCache(u32 a, u32 b, const ContactManifold& manifold);
    void emitExits();
    u32 allocateId();
    u32 slotForId(u32 id) const;
    void insertBody(u32 id, const BodyEntry& entry);
    void removeBodyImmediate(u32 id);
    void clearImmediate();
    void flushPendingMutations();
    void dispatchEvents();

    // Bodies and their ids are parallel dense arrays. Public ids never move;
    // mIdToSlot is updated when the last slot is swapped over a removed one.
    std::vector<BodyEntry> mBodies;
    std::vector<u32> mBodyIds;
    std::vector<u32> mBodyGenerations;
    std::vector<u32> mIdToSlot;
    std::vector<u32> mFreeIds;
    // IDs removed by one collision callback cannot be reused until the whole
    // event batch finishes; later events may still mention the old number.
    std::vector<u32> mRetiredIds;
    u32 mNextBodyGeneration = 1;

    // Any future hook that runs while step() owns temporary references queues
    // mutations here. Collision events themselves are dispatched after the
    // simulation unlocks, so their callbacks can mutate immediately.
    std::vector<PendingAdd> mPendingAdds;
    std::vector<u32> mPendingRemovals;
    bool mStepping = false;
    bool mClearPending = false;
    bool mDispatchingEvents = false;

    Broadphase mBroadphase;
    std::vector<BroadphasePair> mPairs;
    // Scratch for one pair's manifolds, reused every step. A convex against a
    // trimesh produces one per triangle it touches.
    std::vector<ContactManifold> mManifolds;
    std::vector<Contact> mContacts;
    std::vector<Joint*> mJoints;
    std::vector<ContactEventInfo> mEventQueue;
    HashMap<u64, CachedPair> mCache;
    // Union-find scratch, kept as a member so a step allocates nothing.
    std::vector<u32> mIslandParent;
    std::vector<u8> mIslandAwake;

    ContactSolver mSolver;
    glm::vec3 mGravity{0.0f, -9.81f, 0.0f};
    f32 mFixedStep = 1.0f / 120.0f;
    f32 mAccumulator = 0.0f;
    u32 mMaxStepsPerUpdate = 8;
    u32 mStepIndex = 0;
    u32 mContactPersistence = 2;
    f32 mContactMargin = 0.04f;

    EventCallback mEventCallback = nullptr;
    void* mEventUserData = nullptr;
    StepCallback mStepCallback = nullptr;
    void* mStepUserData = nullptr;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_DYNAMICS_PHYSICSWORLD_H
