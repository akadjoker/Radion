#include "PCH.h"

#include "PhysicsBody.h"

#include "GameObject.h"
#include "collision/CollisionShape.h"
#include "dynamics/PhysicsWorld.h"

namespace Radion
{

namespace
{
constexpr f32 kPoseSyncEpsilon = 1e-5f;
}

PhysicsBody::PhysicsBody() : Component(Type)
{
    rebuildShape();
}

PhysicsBody::~PhysicsBody()
{
    delete mShape;
}

void PhysicsBody::setBodyType(Physics::BodyType type)
{
    mBody.setBodyType(type);
    syncEntry();
}

Physics::BodyType PhysicsBody::bodyType() const
{
    return mBody.bodyType();
}

void PhysicsBody::setMass(f32 mass)
{
    if (!(mass > 0.0f) || !std::isfinite(mass))
        return;
    mMass = mass;
    mBody.setMass(mMass);
    rebuildInertia();
}

f32 PhysicsBody::mass() const
{
    return mMass;
}

void PhysicsBody::setSphere(f32 radius)
{
    mShapeType = PhysicsBodyShape::Sphere;
    mRadius = radius;
    rebuildShape();
}

void PhysicsBody::setBox(const glm::vec3& halfExtents)
{
    mShapeType = PhysicsBodyShape::Box;
    mHalfExtents = halfExtents;
    rebuildShape();
}

void PhysicsBody::setCapsule(f32 radius, f32 height)
{
    mShapeType = PhysicsBodyShape::Capsule;
    mRadius = radius;
    mHeight = height;
    rebuildShape();
}

PhysicsBodyShape PhysicsBody::shape() const
{
    return mShapeType;
}

f32 PhysicsBody::radius() const
{
    return mRadius;
}

const glm::vec3& PhysicsBody::halfExtents() const
{
    return mHalfExtents;
}

f32 PhysicsBody::height() const
{
    return mHeight;
}

f32 PhysicsBody::capsuleSegmentHalfHeight() const
{
    return glm::max(mHeight * 0.5f - mRadius, 0.0f);
}

void PhysicsBody::setFriction(f32 friction)
{
    mFriction = friction;
    syncEntry();
}

f32 PhysicsBody::friction() const
{
    return mFriction;
}

void PhysicsBody::setRestitution(f32 restitution)
{
    mRestitution = restitution;
    syncEntry();
}

f32 PhysicsBody::restitution() const
{
    return mRestitution;
}

void PhysicsBody::setCollisionGroup(u32 group)
{
    mCollisionGroup = group;
    syncEntry();
}

u32 PhysicsBody::collisionGroup() const
{
    return mCollisionGroup;
}

void PhysicsBody::setCollisionMask(u32 mask)
{
    mCollisionMask = mask;
    syncEntry();
}

u32 PhysicsBody::collisionMask() const
{
    return mCollisionMask;
}

void PhysicsBody::setEnabled(bool enabled)
{
    mEnabled = enabled;
    syncEntry();
}

bool PhysicsBody::enabled() const
{
    return mEnabled;
}

void PhysicsBody::setVelocity(const glm::vec3& velocity)
{
    mBody.setVelocity(velocity);
}

const glm::vec3& PhysicsBody::velocity() const
{
    return mBody.velocity();
}

void PhysicsBody::setAngularVelocity(const glm::vec3& angularVelocity)
{
    mBody.setAngularVelocity(angularVelocity);
}

const glm::vec3& PhysicsBody::angularVelocity() const
{
    return mBody.angularVelocity();
}

void PhysicsBody::addForce(const glm::vec3& force)
{
    mBody.addForce(force);
}

void PhysicsBody::addForceAtPoint(const glm::vec3& force, const glm::vec3& worldPoint)
{
    mBody.addForceAtPoint(force, worldPoint);
}

void PhysicsBody::addTorque(const glm::vec3& torque)
{
    mBody.addTorque(torque);
}

void PhysicsBody::applyLinearImpulse(const glm::vec3& impulse)
{
    mBody.applyLinearImpulse(impulse);
}

void PhysicsBody::applyAngularImpulse(const glm::vec3& impulse)
{
    mBody.applyAngularImpulse(impulse);
}

void PhysicsBody::applyImpulseAtPoint(const glm::vec3& impulse, const glm::vec3& worldPoint)
{
    mBody.applyImpulseAtPoint(impulse, worldPoint);
}

void PhysicsBody::bindToWorld(Physics::PhysicsWorld& world)
{
    if (mWorld)
        unbindFromWorld();
    pushOwnerPose();

    Physics::BodyEntry entry;
    entry.body = &mBody;
    entry.shape = mShape;
    entry.filter.group = mCollisionGroup;
    entry.filter.mask = mCollisionMask;
    entry.friction = mFriction;
    entry.restitution = mRestitution;
    entry.enabled = mEnabled;

    const u32 id = world.addBody(entry);
    const Physics::BodyHandle handle = world.bodyHandle(id);
    mWorld = &world;
    mBodyId = handle.id;
    mBodyGeneration = handle.generation;
}

void PhysicsBody::unbindFromWorld()
{
    if (mWorld)
        mWorld->removeBody(mBodyId);
    mWorld = nullptr;
    mBodyId = 0xFFFFFFFFu;
    mBodyGeneration = 0;
}

bool PhysicsBody::simulating() const
{
    const GameObject* object = owner();
    return active() && object && object->isActiveInHierarchy() && !object->disposed();
}

void PhysicsBody::pushOwnerPose()
{
    const GameObject* object = owner();
    if (!object)
        return;
    mSyncedPosition = object->globalPosition();
    mSyncedRotation = object->globalRotation();
    mBody.setPosition(mSyncedPosition);
    mBody.setOrientation(mSyncedRotation);
}

bool PhysicsBody::ownerMoved() const
{
    const GameObject* object = owner();
    if (!object)
        return false;
    const glm::vec3 delta = object->globalPosition() - mSyncedPosition;
    if (glm::dot(delta, delta) > kPoseSyncEpsilon * kPoseSyncEpsilon)
        return true;
    const f32 alignment = glm::abs(glm::dot(object->globalRotation(), mSyncedRotation));
    return alignment < 1.0f - kPoseSyncEpsilon;
}

void PhysicsBody::pullBodyPose()
{
    GameObject* object = owner();
    if (!object)
        return;
    object->setGlobalPosition(mBody.position());
    object->setGlobalRotation(mBody.orientation());
    mSyncedPosition = object->globalPosition();
    mSyncedRotation = object->globalRotation();
}

void PhysicsBody::rebuildShape()
{
    delete mShape;
    switch (mShapeType)
    {
    case PhysicsBodyShape::Sphere:
        mShape = new Physics::SphereShape(mRadius);
        break;
    case PhysicsBodyShape::Box:
        mShape = new Physics::BoxShape(mHalfExtents);
        break;
    case PhysicsBodyShape::Capsule:
        mShape = new Physics::CapsuleShape(mRadius, capsuleSegmentHalfHeight());
        break;
    }
    rebuildInertia();
    syncEntry();
}

void PhysicsBody::rebuildInertia()
{
    glm::mat3 inertia(1.0f);
    switch (mShapeType)
    {
    case PhysicsBodyShape::Sphere:
        inertia = Physics::Inertia::solidSphere(mMass, mRadius);
        break;
    case PhysicsBodyShape::Box:
        inertia = Physics::Inertia::box(mMass, mHalfExtents);
        break;
    case PhysicsBodyShape::Capsule:
        inertia = Physics::Inertia::capsuleY(mMass, mRadius, capsuleSegmentHalfHeight() * 2.0f);
        break;
    }
    mBody.setInertiaTensor(inertia);
}

void PhysicsBody::syncEntry()
{
    if (!mWorld)
        return;
    Physics::BodyEntry* entry = mWorld->body(Physics::BodyHandle{mBodyId, mBodyGeneration});
    if (!entry)
        return;
    entry->shape = mShape;
    entry->filter.group = mCollisionGroup;
    entry->filter.mask = mCollisionMask;
    entry->friction = mFriction;
    entry->restitution = mRestitution;
    entry->enabled = mEnabled;
    if (mBody.bodyType() == Physics::BodyType::Static)
        mWorld->markStaticBroadphaseDirty();
}

} // namespace Radion
