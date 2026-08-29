#ifndef RADION_PHYSICS_DYNAMICS_RIGIDBODY_H
#define RADION_PHYSICS_DYNAMICS_RIGIDBODY_H

#include "ConvexHullComputer.h"
#include "Math.h"
#include "Types.h"

namespace Radion::Physics
{

// What the simulation is allowed to do with a body.
enum class BodyType : u8
{
    // Never moves and never responds to a force. Infinite mass and infinite
    // inertia, which is what makes a contact against it push only the other
    // side. The floor, a wall, the world.
    Static,

    // Integrated from forces and torques. The only kind that falls.
    Dynamic,

    // Moved by whoever owns it - an animation, a lift, a moving platform -
    // and unmoved by any contact. It still carries a velocity, so a dynamic
    // body resting on one is pushed along by it rather than sliding off a
    // surface that teleports underneath.
    Kinematic
};

// Moments of inertia about the centre of mass, for a body of uniform density.
// They belong here and not with the collision shapes: a body needs one to
// rotate correctly whether or not anything ever collides with it, and these
// are the standard closed forms rather than anything measured off geometry.
struct Inertia
{
    // `halfExtents` are half the box's size on each axis, matching how every
    // AABB in the engine is described.
    static glm::mat3 box(f32 mass, const glm::vec3& halfExtents);
    static glm::mat3 solidSphere(f32 mass, f32 radius);
    static glm::mat3 hollowSphere(f32 mass, f32 radius);
    // Along +Y, which is the engine's up and the axis a capsule stands on.
    static glm::mat3 cylinderY(f32 mass, f32 radius, f32 height);
    static glm::mat3 capsuleY(f32 mass, f32 radius, f32 cylinderHeight);
    // Any convex polyhedron, from its own half-edge mesh - the ONLY one of
    // these that measures the tensor off actual geometry instead of a
    // closed form. `faces` holds one edge index per face, walked with
    // getNextEdgeOfFace() the way ConvexHullComputer itself expects.
    static glm::mat3 convexHull(f32 mass, const glm::vec3* vertices, u32 vertexCount,
                                const Radion::Geometry::ConvexHullComputer::Edge* edges,
                                u32 edgeCount, const int* faces, u32 faceCount);
};

// One rigid body: position and orientation, their first derivatives, and the
// accumulators a step's forces and torques land in. It knows nothing about
// collision, about the scene, or about who owns it - contacts and constraints
// reach it through the impulse and force calls like anything else does.
//
// The inertia tensor is kept in body space and rotated into world space once
// per step. Deviations from the reference port are marked where they occur:
// the three body types above and a per-body sleep epsilon.
class RigidBody
{
public:
    RigidBody();

    void setBodyType(BodyType type);
    BodyType bodyType() const
    {
        return mBodyType;
    }
    // False for Static and Kinematic - both are infinite mass, and dividing
    // by their mass is the mistake this exists to make impossible.
    bool isDynamic() const
    {
        return mBodyType == BodyType::Dynamic;
    }

    // Mass of zero is refused rather than silently producing infinities: a
    // body with no mass is not a body, and "immovable" is setBodyType(Static).
    void setMass(f32 mass);
    f32 mass() const;
    void setInverseMass(f32 inverseMass);
    f32 inverseMass() const
    {
        return mInverseMass;
    }
    bool hasFiniteMass() const
    {
        return mInverseMass > 0.0f;
    }

    // In BODY space. calculateDerivedData() rotates it into world space; the
    // world-space one is what the solver divides a torque by.
    void setInertiaTensor(const glm::mat3& inertiaTensor);
    glm::mat3 inertiaTensor() const;
    void setInverseInertiaTensor(const glm::mat3& inverseInertiaTensor);
    const glm::mat3& inverseInertiaTensor() const
    {
        return mInverseInertiaTensor;
    }
    const glm::mat3& inverseInertiaTensorWorld() const
    {
        return mInverseInertiaTensorWorld;
    }

    // Per-second retention, applied as pow(damping, dt) so the result does
    // not change with the step size. 1 is no damping at all, which the
    // reference warns against: an integrator adds energy on its own, and
    // nothing to take it back out is how a stack slowly explodes.
    void setDamping(f32 linear, f32 angular);
    f32 linearDamping() const
    {
        return mLinearDamping;
    }
    f32 angularDamping() const
    {
        return mAngularDamping;
    }

    void setPosition(const glm::vec3& position);
    const glm::vec3& position() const
    {
        return mPosition;
    }
    void setOrientation(const glm::quat& orientation);
    const glm::quat& orientation() const
    {
        return mOrientation;
    }

    void setVelocity(const glm::vec3& velocity);
    const glm::vec3& velocity() const
    {
        return mVelocity;
    }
    void setAngularVelocity(const glm::vec3& angularVelocity);
    const glm::vec3& angularVelocity() const
    {
        return mAngularVelocity;
    }

    // Constant acceleration that ignores mass - gravity, and only gravity.
    // Kept apart from the force accumulator because a force has to be scaled
    // by inverse mass, and gravity accelerates everything equally.
    void setAcceleration(const glm::vec3& acceleration);
    const glm::vec3& acceleration() const
    {
        return mAcceleration;
    }
    // Acceleration the last step actually ran with, gravity included. Contact
    // resolution needs it to tell a resting body from a colliding one.
    const glm::vec3& lastFrameAcceleration() const
    {
        return mLastFrameAcceleration;
    }

    const glm::mat4& transform() const
    {
        return mTransform;
    }
    glm::vec3 pointToWorld(const glm::vec3& local) const;
    glm::vec3 pointToLocal(const glm::vec3& world) const;
    glm::vec3 directionToWorld(const glm::vec3& local) const;
    glm::vec3 directionToLocal(const glm::vec3& world) const;
    // World-space velocity of a point attached to the body - v + w x r. What
    // a contact point is actually moving at, and the reason a spinning body
    // scrapes rather than slides.
    glm::vec3 velocityAtPoint(const glm::vec3& worldPoint) const;

    // Forces accumulate over a step and are cleared by integrate().
    void addForce(const glm::vec3& force);
    void addForceAtPoint(const glm::vec3& force, const glm::vec3& worldPoint);
    void addForceAtBodyPoint(const glm::vec3& force, const glm::vec3& localPoint);
    void addTorque(const glm::vec3& torque);
    void clearAccumulators();

    // Impulses change velocity immediately instead of over a step. Contacts
    // and constraints are resolved this way; a force cannot be, because the
    // correction has to hold at the end of the step it was computed for.
    void applyLinearImpulse(const glm::vec3& impulse);
    void applyAngularImpulse(const glm::vec3& impulse);
    void applyImpulseAtPoint(const glm::vec3& impulse, const glm::vec3& worldPoint);
    // Split-impulse position correction: changes pose without adding kinetic
    // energy. Contacts and joints use this to remove drift.
    void applyPositionImpulseAtPoint(const glm::vec3& impulse, const glm::vec3& worldPoint);

    // A sleeping body is skipped by integrate() entirely. Waking is what any
    // force or impulse arriving does; falling asleep is decided by integrate()
    // from a running average of motion.
    void setAwake(bool awake);
    bool awake() const
    {
        return mAwake;
    }
    void setCanSleep(bool canSleep);
    bool canSleep() const
    {
        return mCanSleep;
    }
    void setSleepEpsilon(f32 epsilon);
    f32 sleepEpsilon() const
    {
        return mSleepEpsilon;
    }
    f32 motion() const
    {
        return mMotion;
    }

    // With this set, integrate() still keeps the motion average up to date
    // but never falls asleep on its own - the caller decides. PhysicsWorld
    // sets it, because sleep belongs to a whole contact island and not to one
    // body: a box that stops moving before the stack under it has finished
    // settling must not drop out on its own, or it hangs in the air.
    void setSleepDeferred(bool deferred);
    bool sleepDeferred() const
    {
        return mSleepDeferred;
    }

    // Fast, small dynamic bodies (bullets, thrown projectiles) tunnel through
    // thin static geometry when the discrete step moves them further than
    // their own size in one go. PhysicsWorld sweeps a marked body's centre
    // from its pre-step to post-step position each step and clamps it back
    // to just short of the first static hit - see PhysicsWorld::solveBulletSweeps().
    // Costs a raycast per bullet body per step, so leave it off for anything
    // that does not need it.
    void setBullet(bool bullet)
    {
        mBullet = bullet;
    }
    bool isBullet() const
    {
        return mBullet;
    }

    // Rebuilds the transform and the world-space inverse inertia tensor from
    // position and orientation. integrate() ends with this; call it by hand
    // after setting state directly, or everything derived stays a step stale.
    void calculateDerivedData();

    // Split integration lets the world apply forces before solving contacts
    // and joints, then advance poses with the corrected velocities.
    void integrateForces(f32 duration);
    void integrateVelocity(f32 duration);

    // Convenience path for standalone users and the focused body tests.
    void integrate(f32 duration);

private:
    void applyBodyTypeMass();

    BodyType mBodyType = BodyType::Dynamic;

    f32 mInverseMass = 1.0f;
    glm::mat3 mInverseInertiaTensor{1.0f};
    glm::mat3 mInverseInertiaTensorWorld{1.0f};

    f32 mLinearDamping = 0.99f;
    f32 mAngularDamping = 0.99f;

    glm::vec3 mPosition{0.0f};
    glm::quat mOrientation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 mVelocity{0.0f};
    glm::vec3 mAngularVelocity{0.0f};

    glm::vec3 mAcceleration{0.0f};
    glm::vec3 mLastFrameAcceleration{0.0f};

    glm::vec3 mForceAccumulator{0.0f};
    glm::vec3 mTorqueAccumulator{0.0f};

    glm::mat4 mTransform{1.0f};

    // Mass and inertia as the caller set them, kept through a change to
    // Static or Kinematic so switching back to Dynamic restores the body
    // instead of leaving it with the infinite mass those two forced on it.
    f32 mDynamicInverseMass = 1.0f;
    glm::mat3 mDynamicInverseInertiaTensor{1.0f};

    bool mAwake = true;
    bool mCanSleep = true;
    bool mSleepDeferred = false;
    bool mBullet = false;
    f32 mMotion = 0.0f;
    f32 mSleepEpsilon = 0.3f;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_DYNAMICS_RIGIDBODY_H
