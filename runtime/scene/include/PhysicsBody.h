#ifndef RADION_PHYSICS_BODY_H
#define RADION_PHYSICS_BODY_H

#include "Component.h"
#include "Types.h"
#include "dynamics/RigidBody.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace Radion::Physics
{
class CollisionShape;
class PhysicsWorld;
} // namespace Radion::Physics

namespace Radion
{

enum class PhysicsBodyShape : u8
{
    Sphere,
    Box,
    Capsule
};

// A Physics::RigidBody plus what Physics::BodyEntry carries around it (see
// dynamics/PhysicsWorld.h) - shape, filter, friction, restitution, enabled -
// attached to a GameObject. Scene registers it with its Physics::PhysicsWorld
// from componentAdded() and unregisters it from componentRemoved(); this
// class never reaches for a world on its own.
class PhysicsBody final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::PhysicsBody;

    void setBodyType(Physics::BodyType type);
    Physics::BodyType bodyType() const;

    void setMass(f32 mass);
    f32 mass() const;

    void setSphere(f32 radius);
    void setBox(const glm::vec3& halfExtents);
    void setCapsule(f32 radius, f32 height); // height is total, cap to cap
    PhysicsBodyShape shape() const;
    f32 radius() const;
    const glm::vec3& halfExtents() const;
    f32 height() const;
    f32 capsuleSegmentHalfHeight() const; // physics CapsuleShape's segment, half of it

    void setFriction(f32 friction);
    f32 friction() const;
    void setRestitution(f32 restitution);
    f32 restitution() const;

    void setCollisionGroup(u32 group);
    u32 collisionGroup() const;
    void setCollisionMask(u32 mask);
    u32 collisionMask() const;

    void setEnabled(bool enabled);
    bool enabled() const;

    void setVelocity(const glm::vec3& velocity);
    const glm::vec3& velocity() const;
    void setAngularVelocity(const glm::vec3& angularVelocity);
    const glm::vec3& angularVelocity() const;

    void addForce(const glm::vec3& force);
    void addForceAtPoint(const glm::vec3& force, const glm::vec3& worldPoint);
    void addTorque(const glm::vec3& torque);
    void applyLinearImpulse(const glm::vec3& impulse);
    void applyAngularImpulse(const glm::vec3& impulse);
    void applyImpulseAtPoint(const glm::vec3& impulse, const glm::vec3& worldPoint);

private:
    friend class GameObject;
    friend class Scene;

    PhysicsBody();
    ~PhysicsBody() override;

    // Scene-only. bind/unbind register this body with the world that
    // simulates it; the pose methods move the pose between the owner's
    // transform and the RigidBody around each step.
    void bindToWorld(Physics::PhysicsWorld& world);
    void unbindFromWorld();
    bool simulating() const;
    void pushOwnerPose();
    bool ownerMoved() const;
    void pullBodyPose();

    void rebuildShape();
    void rebuildInertia();
    // Pushes shape/filter/friction/restitution/enabled into the live
    // Physics::BodyEntry, when this body is currently registered - a setter
    // called after bindToWorld() would otherwise never reach the copy
    // PhysicsWorld actually simulates against.
    void syncEntry();

    Physics::RigidBody mBody;
    Physics::CollisionShape* mShape = nullptr;
    Physics::PhysicsWorld* mWorld = nullptr;
    u32 mBodyId = 0xFFFFFFFFu;
    u32 mBodyGeneration = 0;
    glm::vec3 mSyncedPosition{0.0f};
    glm::quat mSyncedRotation{1.0f, 0.0f, 0.0f, 0.0f};

    PhysicsBodyShape mShapeType = PhysicsBodyShape::Sphere;
    f32 mRadius = 0.5f;
    glm::vec3 mHalfExtents{0.5f, 0.5f, 0.5f};
    f32 mHeight = 1.0f;
    f32 mMass = 1.0f;
    f32 mFriction = 0.5f;
    f32 mRestitution = 0.0f;
    u32 mCollisionGroup = 1;
    u32 mCollisionMask = 0xFFFFFFFFu;
    bool mEnabled = true;
};

} // namespace Radion

#endif // RADION_PHYSICS_BODY_H
