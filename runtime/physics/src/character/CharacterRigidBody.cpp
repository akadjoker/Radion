#include "PCH.h"

#include "character/CharacterRigidBody.h"

#include "collision/Narrowphase.h"
#include "dynamics/PhysicsWorld.h"

namespace Radion::Physics
{

CharacterRigidBody::CharacterRigidBody()
{
}

void CharacterRigidBody::setShape(f32 radius, f32 height)
{
    mRadius = glm::max(radius, 0.01f);
    mHeight = glm::max(height, 0.0f);
    mShape = CapsuleShape(mRadius, mHeight * 0.5f);
}

void CharacterRigidBody::setMass(f32 mass)
{
    mMass = glm::max(mass, 0.001f);
    if (mBodyId != kInvalidBodyId)
        mBody.setMass(mMass);
}

void CharacterRigidBody::setFriction(f32 friction)
{
    mFriction = glm::max(friction, 0.0f);
    if (mWorld && mBodyId != kInvalidBodyId)
        if (BodyEntry* entry = mWorld->body(mBodyId))
            entry->friction = mFriction;
}

void CharacterRigidBody::setMaxSlopeAngle(f32 degrees)
{
    mMaxSlopeAngleDegrees = glm::clamp(degrees, 0.0f, 89.0f);
    mMaxSlopeAngleCosine = std::cos(glm::radians(mMaxSlopeAngleDegrees));
}

void CharacterRigidBody::setLayer(u32 layer)
{
    mLayer = layer;
    if (mWorld && mBodyId != kInvalidBodyId)
        if (BodyEntry* entry = mWorld->body(mBodyId))
            entry->filter.group = mLayer;
}

void CharacterRigidBody::setMask(u32 mask)
{
    mMask = mask;
    if (mWorld && mBodyId != kInvalidBodyId)
        if (BodyEntry* entry = mWorld->body(mBodyId))
            entry->filter.mask = mMask;
}

void CharacterRigidBody::addToWorld(PhysicsWorld& world, const Math::Vec3& position)
{
    mBody.setBodyType(BodyType::Dynamic);
    mBody.setMass(mMass);
    mBody.setInverseInertiaTensor(Math::Mat3(0.0f));
    mBody.setPosition(position);

    BodyEntry entry;
    entry.body = &mBody;
    entry.shape = &mShape;
    entry.filter.group = mLayer;
    entry.filter.mask = mMask;
    entry.friction = mFriction;
    entry.restitution = 0.0f;

    mWorld = &world;
    mBodyId = world.addBody(entry);
}

void CharacterRigidBody::removeFromWorld()
{
    if (mWorld && mBodyId != kInvalidBodyId)
        mWorld->removeBody(mBodyId);
    mWorld = nullptr;
    mBodyId = kInvalidBodyId;
}

void CharacterRigidBody::setLinearVelocity(const Math::Vec3& velocity)
{
    mBody.setVelocity(velocity);
    if (glm::dot(velocity, velocity) > 1.0e-12f && !mBody.awake())
        mBody.setAwake(true);
}

const Math::Vec3& CharacterRigidBody::linearVelocity() const
{
    return mBody.velocity();
}

void CharacterRigidBody::addLinearVelocity(const Math::Vec3& velocity)
{
    setLinearVelocity(mBody.velocity() + velocity);
}

void CharacterRigidBody::addImpulse(const Math::Vec3& impulse)
{
    mBody.applyLinearImpulse(impulse);
}

const Math::Vec3& CharacterRigidBody::position() const
{
    return mBody.position();
}

Math::Mat4 CharacterRigidBody::transform() const
{
    return mBody.transform();
}

void CharacterRigidBody::postSimulation(f32 maxSeparationDistance)
{
    if (!mWorld || mBodyId == kInvalidBodyId)
        return;

    const Math::Vec3 characterPosition = mBody.position();
    const Math::Mat4 characterTransform = mBody.transform();

    u32 groundBodyId = kInvalidBodyId;
    Math::Vec3 groundNormal(0.0f);
    Math::Vec3 groundPosition(0.0f);
    f32 bestDot = -std::numeric_limits<f32>::max();

    AABB candidateBounds = mShape.bounds(characterTransform);
    candidateBounds.min -= Math::Vec3(maxSeparationDistance);
    candidateBounds.max += Math::Vec3(maxSeparationDistance);
    QueryFilter query;
    query.collision = {mLayer, mMask};
    query.ignoredBody = mBodyId;
    mWorld->queryAABB(candidateBounds, query, mCandidates);

    for (u32 id : mCandidates)
    {
        BodyEntry* entry = mWorld->body(id);
        if (!entry || !entry->body || !entry->shape || !entry->enabled)
            continue;

        mManifolds.clear();
        if (entry->shape->type() == ShapeType::Trimesh)
        {
            const TrimeshShape& mesh = static_cast<const TrimeshShape&>(*entry->shape);
            Narrowphase::convexTrimesh(mShape, characterTransform, mesh, entry->body->transform(),
                                       mManifolds, maxSeparationDistance);
        }
        else
        {
            ContactManifold manifold;
            if (Narrowphase::collide(mShape, characterTransform, *entry->shape,
                                     entry->body->transform(), manifold, maxSeparationDistance))
                mManifolds.push_back(manifold);
        }

        for (const ContactManifold& manifold : mManifolds)
        {
            if (manifold.count == 0)
                continue;
            const Math::Vec3 normal = -manifold.normal;
            const f32 dot = glm::dot(normal, mUp);
            if (dot > bestDot)
            {
                bestDot = dot;
                groundNormal = normal;
                groundPosition = manifold.points[0].position;
                groundBodyId = id;
            }
        }
    }

    mGroundBodyId = groundBodyId;
    if (groundBodyId == kInvalidBodyId)
    {
        mGroundState = GroundState::InAir;
        mGroundNormal = Math::Vec3(0.0f);
        mGroundPosition = Math::Vec3(0.0f);
        mGroundVelocity = Math::Vec3(0.0f);
        return;
    }

    mGroundNormal = groundNormal;
    mGroundPosition = groundPosition;

    const Math::Vec3 localGroundPosition = groundPosition - characterPosition;
    if (mSupportingVolume.distance(localGroundPosition) > 0.0f)
        mGroundState = GroundState::NotSupported;
    else if (glm::dot(groundNormal, mUp) < mMaxSlopeAngleCosine)
        mGroundState = GroundState::OnSteepGround;
    else
        mGroundState = GroundState::OnGround;

    const BodyEntry* groundEntry = mWorld->body(groundBodyId);
    mGroundVelocity = (groundEntry && groundEntry->body) ? groundEntry->body->velocityAtPoint(groundPosition)
                                                          : Math::Vec3(0.0f);
}

} // namespace Radion::Physics
