#ifndef RADION_PHYSICS_DYNAMICS_RIGIDBODY_H
#define RADION_PHYSICS_DYNAMICS_RIGIDBODY_H

#include "Component.h"
#include "ConvexHullComputer.h"
#include "Math.h"
#include "Types.h"
#include "collision/CollisionFilter.h"

namespace Radion
{
class Scene;
}

namespace Radion::Physics
{

class CollisionShape;

enum class BodyType : u8
{
    // Never moves or responds to force (infinite mass and inertia), so a contact against it pushes only the other side.
    Static,

    Dynamic,

    // Moved by its owner (animation, lift, platform), unmoved by contacts; it still carries a velocity so a resting dynamic body is carried rather than slid off.
    Kinematic
};

// Moments of inertia about the centre of mass for uniform density; standard closed forms, kept here because a body needs one to rotate whether or not it collides.
struct Inertia
{
    // `halfExtents` are half the box size per axis, as every AABB in the engine.
    static Math::mat3 box(f32 mass, const Math::vec3& halfExtents);
    static Math::mat3 solidSphere(f32 mass, f32 radius);
    static Math::mat3 hollowSphere(f32 mass, f32 radius);
    // Along +Y, which is the engine's up and the axis a capsule stands on.
    static Math::mat3 cylinderY(f32 mass, f32 radius, f32 height);
    static Math::mat3 capsuleY(f32 mass, f32 radius, f32 cylinderHeight);
    // Any convex polyhedron from its half-edge mesh: the ONLY one measured off actual geometry. `faces` holds one edge index per face,
    // walked with getNextEdgeOfFace().
    static Math::mat3 convexHull(f32 mass, const Math::vec3* vertices, u32 vertexCount,
                                const Radion::Geometry::ConvexHullComputer::Edge* edges,
                                u32 edgeCount, const int* faces, u32 faceCount);
};

enum class RigidBodyShape : u8
{
    None,
    Sphere,
    Box,
    Capsule
};

// One rigid body: pose, first derivatives and force/torque accumulators; it knows nothing of collision or the scene.
// As a Component it carries shape, filter, friction, restitution, enabled and the pose sync with its owner GameObject;
// a loose body (Ragdoll part, CharacterRigidBody, test) has no owner to sync with.
// The inertia tensor is kept in body space and rotated to world once per step.
class RigidBody : public Radion::Component
{
public:
    static constexpr ComponentType Type = ComponentType::RigidBody;

    RigidBody();
    ~RigidBody() override;
    // Component deletes copy; move lets a loose body live in a std::vector or return from a factory. Shape ownership moves; the source keeps no shape, so it does not double free.
    RigidBody(RigidBody&& other) noexcept;
    RigidBody& operator=(RigidBody&& other) noexcept;

    void setBodyType(BodyType type);
    BodyType bodyType() const
    {
        return mBodyType;
    }
    // False for Static and Kinematic (infinite mass): makes dividing by their mass impossible.
    bool isDynamic() const
    {
        return mBodyType == BodyType::Dynamic;
    }

    // Mass of zero is refused ("immovable" is setBodyType(Static)). mass() returns the authored value, not the current inverse mass:
    // a Static/Kinematic body keeps its mass for when it returns to Dynamic (inspector and serializer want that).
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

    // In BODY space; calculateDerivedData() rotates it to world, which the solver divides torque by.
    void setInertiaTensor(const Math::mat3& inertiaTensor);
    Math::mat3 inertiaTensor() const;
    void setInverseInertiaTensor(const Math::mat3& inverseInertiaTensor);
    const Math::mat3& inverseInertiaTensor() const
    {
        return mInverseInertiaTensor;
    }
    const Math::mat3& inverseInertiaTensorWorld() const
    {
        return mInverseInertiaTensorWorld;
    }

    // Per-second retention applied as pow(damping, dt), independent of step size. 1 is no damping, which the reference warns against:
    // integrators add energy and a stack slowly explodes.
    void setDamping(f32 linear, f32 angular);
    f32 linearDamping() const
    {
        return mLinearDamping;
    }
    f32 angularDamping() const
    {
        return mAngularDamping;
    }

    void setPosition(const Math::vec3& position);
    const Math::vec3& position() const
    {
        return mPosition;
    }
    void setOrientation(const Math::quat& orientation);
    const Math::quat& orientation() const
    {
        return mOrientation;
    }

    void setVelocity(const Math::vec3& velocity);
    const Math::vec3& velocity() const
    {
        return mVelocity;
    }
    void setAngularVelocity(const Math::vec3& angularVelocity);
    const Math::vec3& angularVelocity() const
    {
        return mAngularVelocity;
    }

    // Constant acceleration ignoring mass (gravity only); kept apart from the force accumulator, which is scaled by inverse mass.
    void setAcceleration(const Math::vec3& acceleration);
    const Math::vec3& acceleration() const
    {
        return mAcceleration;
    }
    // Acceleration the last step ran with, gravity included; contact resolution tells resting from colliding with it.
    const Math::vec3& lastFrameAcceleration() const
    {
        return mLastFrameAcceleration;
    }

    const Math::mat4& transform() const
    {
        return mTransform;
    }
    Math::vec3 pointToWorld(const Math::vec3& local) const;
    Math::vec3 pointToLocal(const Math::vec3& world) const;
    Math::vec3 directionToWorld(const Math::vec3& local) const;
    Math::vec3 directionToLocal(const Math::vec3& world) const;
    Math::vec3 velocityAtPoint(const Math::vec3& worldPoint) const;

    void addForce(const Math::vec3& force);
    void addForceAtPoint(const Math::vec3& force, const Math::vec3& worldPoint);
    void addForceAtBodyPoint(const Math::vec3& force, const Math::vec3& localPoint);
    void addTorque(const Math::vec3& torque);
    void clearAccumulators();

    // Impulses change velocity immediately; contacts and constraints use them because the correction must hold at the end of the step it was computed for.
    void applyLinearImpulse(const Math::vec3& impulse);
    void applyAngularImpulse(const Math::vec3& impulse);
    void applyImpulseAtPoint(const Math::vec3& impulse, const Math::vec3& worldPoint);
    // Split-impulse position correction: changes pose without adding kinetic energy (removes drift).
    void applyPositionImpulseAtPoint(const Math::vec3& impulse, const Math::vec3& worldPoint);

    // A sleeping body is skipped by integrate(); any force or impulse wakes it; sleep is decided from a running average of motion.
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

    // integrate() still updates the motion average but never sleeps on its own: Scene sets it because sleep belongs to a contact island,
    // and a box that stops before the stack under it settles would hang in the air.
    void setSleepDeferred(bool deferred);
    bool sleepDeferred() const
    {
        return mSleepDeferred;
    }

    // Fast small bodies tunnel through thin static geometry. Scene sweeps a marked body's centre pre- to post-step and clamps it short of the first
    // static hit (Scene::solveBulletSweeps()); costs a raycast per body per step.
    void setBullet(bool bullet)
    {
        mBullet = bullet;
    }
    bool isBullet() const
    {
        return mBullet;
    }

    // Rebuilds transform and world inverse inertia from position/orientation; integrate() ends with it; call after setting state directly, or derived data stays a step stale.
    void calculateDerivedData();

    // Split integration lets the world apply forces before solving contacts and joints, then advance poses with the corrected velocities.
    void integrateForces(f32 duration);
    void integrateVelocity(f32 duration);

    void integrate(f32 duration);

    void setShape(CollisionShape* shape); // non-owning; frees a previously owned one
    CollisionShape* shape() const
    {
        return mShape;
    }
    void setSphere(f32 radius);
    void setBox(const Math::vec3& halfExtents);
    void setCapsule(f32 radius, f32 height); // height is total, cap to cap
    RigidBodyShape shapeKind() const
    {
        return mShapeKind;
    }
    f32 radius() const
    {
        return mRadius;
    }
    const Math::vec3& halfExtents() const
    {
        return mHalfExtents;
    }
    f32 height() const
    {
        return mHeight;
    }
    f32 capsuleSegmentHalfHeight() const;

    void setFilter(const CollisionFilter& filter);
    const CollisionFilter& filter() const
    {
        return mFilter;
    }
    void setCollisionGroup(u32 group);
    u32 collisionGroup() const
    {
        return mFilter.group;
    }
    void setCollisionMask(u32 mask);
    u32 collisionMask() const
    {
        return mFilter.mask;
    }
    void setFriction(f32 friction);
    f32 friction() const
    {
        return mFriction;
    }
    void setRestitution(f32 restitution);
    f32 restitution() const
    {
        return mRestitution;
    }
    void setEnabled(bool enabled);
    bool enabled() const
    {
        return mEnabled;
    }

    Radion::Scene* scene() const
    {
        return mScene;
    }

private:
    friend class Radion::Scene;

    void applyBodyTypeMass();

    bool simulating() const;
    void pushOwnerPose();
    bool ownerMoved() const;
    void pullBodyPose();
    void rebuildOwnedShape();
    void notifyShapeChanged();
    void moveFrom(RigidBody& other) noexcept;

    BodyType mBodyType = BodyType::Dynamic;

    f32 mInverseMass = 1.0f;
    Math::mat3 mInverseInertiaTensor{1.0f};
    Math::mat3 mInverseInertiaTensorWorld{1.0f};

    f32 mLinearDamping = 0.99f;
    f32 mAngularDamping = 0.99f;

    Math::vec3 mPosition{0.0f};
    Math::quat mOrientation{1.0f, 0.0f, 0.0f, 0.0f};
    Math::vec3 mVelocity{0.0f};
    Math::vec3 mAngularVelocity{0.0f};

    Math::vec3 mAcceleration{0.0f};
    Math::vec3 mLastFrameAcceleration{0.0f};

    Math::vec3 mForceAccumulator{0.0f};
    Math::vec3 mTorqueAccumulator{0.0f};

    Math::mat4 mTransform{1.0f};

    // Authored mass and inertia, kept through Static/Kinematic so switching back to Dynamic restores them.
    f32 mDynamicInverseMass = 1.0f;
    Math::mat3 mDynamicInverseInertiaTensor{1.0f};

    bool mAwake = true;
    bool mCanSleep = true;
    bool mSleepDeferred = false;
    bool mBullet = false;
    f32 mMotion = 0.0f;
    f32 mSleepEpsilon = 0.3f;

    CollisionShape* mShape = nullptr;
    bool mOwnsShape = false;
    RigidBodyShape mShapeKind = RigidBodyShape::None;
    f32 mRadius = 0.5f;
    Math::vec3 mHalfExtents{0.5f};
    f32 mHeight = 1.0f;
    CollisionFilter mFilter;
    f32 mFriction = 0.5f;
    f32 mRestitution = 0.0f;
    bool mEnabled = true;

    Radion::Scene* mScene = nullptr;
    u32 mBodyKey = 0;
    u32 mStepSlot = 0;
    Math::vec3 mSyncedPosition{0.0f};
    Math::quat mSyncedRotation{1.0f, 0.0f, 0.0f, 0.0f};
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_DYNAMICS_RIGIDBODY_H
