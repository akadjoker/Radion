#include "PCH.h"

#include "character/CharacterRigidBody.h"

#include "Scene.h"
#include "collision/Narrowphase.h"

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
    mBody.setMass(mMass);
}

void CharacterRigidBody::setFriction(f32 friction)
{
    mFriction = glm::max(friction, 0.0f);
    mBody.setFriction(mFriction);
}

void CharacterRigidBody::setMaxSlopeAngle(f32 degrees)
{
    mMaxSlopeAngleDegrees = glm::clamp(degrees, 0.0f, 89.0f);
    mMaxSlopeAngleCosine = std::cos(glm::radians(mMaxSlopeAngleDegrees));
}

void CharacterRigidBody::setLayer(u32 layer)
{
    mLayer = layer;
    mBody.setCollisionGroup(mLayer);
}

void CharacterRigidBody::setMask(u32 mask)
{
    mMask = mask;
    mBody.setCollisionMask(mMask);
}

void CharacterRigidBody::addToWorld(Radion::Scene& scene, const glm::vec3& position)
{
    mBody.setBodyType(BodyType::Dynamic);
    mBody.setMass(mMass);
    mBody.setInverseInertiaTensor(glm::mat3(0.0f));
    mBody.setPosition(position);

    mBody.setShape(&mShape);
    mBody.setFilter({mLayer, mMask});
    mBody.setFriction(mFriction);
    mBody.setRestitution(0.0f);

    scene.addBody(mBody);
}

void CharacterRigidBody::removeFromWorld()
{
    if (mBody.scene())
        mBody.scene()->removeBody(mBody);
}

void CharacterRigidBody::setLinearVelocity(const glm::vec3& velocity)
{
    mBody.setVelocity(velocity);
    if (glm::dot(velocity, velocity) > 1.0e-12f && !mBody.awake())
        mBody.setAwake(true);
}

const glm::vec3& CharacterRigidBody::linearVelocity() const
{
    return mBody.velocity();
}

void CharacterRigidBody::addLinearVelocity(const glm::vec3& velocity)
{
    setLinearVelocity(mBody.velocity() + velocity);
}

void CharacterRigidBody::addImpulse(const glm::vec3& impulse)
{
    mBody.applyLinearImpulse(impulse);
}

const glm::vec3& CharacterRigidBody::position() const
{
    return mBody.position();
}

glm::mat4 CharacterRigidBody::transform() const
{
    return mBody.transform();
}

void CharacterRigidBody::postSimulation(f32 maxSeparationDistance)
{
    if (!isInWorld())
        return;

    const glm::vec3 characterPosition = mBody.position();
    const glm::mat4 characterTransform = mBody.transform();

    RigidBody* groundBody = nullptr;
    glm::vec3 groundNormal(0.0f);
    glm::vec3 groundPosition(0.0f);
    f32 bestDot = -std::numeric_limits<f32>::max();

    AABB candidateBounds = mShape.bounds(characterTransform);
    candidateBounds.min -= glm::vec3(maxSeparationDistance);
    candidateBounds.max += glm::vec3(maxSeparationDistance);
    QueryFilter query;
    query.collision = {mLayer, mMask};
    query.ignoredBody = &mBody;
    mBody.scene()->queryAABB(candidateBounds, query, mCandidates);

    for (RigidBody* candidate : mCandidates)
    {
        if (!candidate || !candidate->shape() || !candidate->enabled())
            continue;

        mManifolds.clear();
        if (candidate->shape()->type() == ShapeType::Trimesh)
        {
            const TrimeshShape& mesh = static_cast<const TrimeshShape&>(*candidate->shape());
            Narrowphase::convexTrimesh(mShape, characterTransform, mesh, candidate->transform(),
                                       mManifolds, maxSeparationDistance);
        }
        else
        {
            ContactManifold manifold;
            if (Narrowphase::collide(mShape, characterTransform, *candidate->shape(),
                                     candidate->transform(), manifold, maxSeparationDistance))
                mManifolds.push_back(manifold);
        }

        for (const ContactManifold& manifold : mManifolds)
        {
            if (manifold.count == 0)
                continue;
            const glm::vec3 normal = -manifold.normal;
            const f32 dot = glm::dot(normal, mUp);
            if (dot > bestDot)
            {
                bestDot = dot;
                groundNormal = normal;
                groundPosition = manifold.points[0].position;
                groundBody = candidate;
            }
        }
    }

    mGroundBody = groundBody;
    if (!groundBody)
    {
        mGroundState = GroundState::InAir;
        mGroundNormal = glm::vec3(0.0f);
        mGroundPosition = glm::vec3(0.0f);
        mGroundVelocity = glm::vec3(0.0f);
        return;
    }

    mGroundNormal = groundNormal;
    mGroundPosition = groundPosition;

    const glm::vec3 localGroundPosition = groundPosition - characterPosition;
    if (mSupportingVolume.distance(localGroundPosition) > 0.0f)
        mGroundState = GroundState::NotSupported;
    else if (glm::dot(groundNormal, mUp) < mMaxSlopeAngleCosine)
        mGroundState = GroundState::OnSteepGround;
    else
        mGroundState = GroundState::OnGround;

    mGroundVelocity = groundBody->velocityAtPoint(groundPosition);
}

} // namespace Radion::Physics
