#include "PCH.h"

#include "dynamics/DistanceJoint.h"

#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

DistanceJoint::DistanceJoint(RigidBody& a, const Math::Vec3& worldAnchorA, RigidBody& b,
                             const Math::Vec3& worldAnchorB)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(a.pointToLocal(worldAnchorA)),
      mLocalAnchorB(b.pointToLocal(worldAnchorB))
{
    const f32 distance = glm::length(worldAnchorB - worldAnchorA);
    mMinDistance = distance;
    mMaxDistance = distance;
}

DistanceJoint::DistanceJoint(RigidBody& a, const Math::Vec3& localAnchorA, RigidBody& b,
                             const Math::Vec3& localAnchorB, f32 minDistance, f32 maxDistance)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB)
{
    setDistance(minDistance, maxDistance);
}

RigidBody* DistanceJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* DistanceJoint::bodyB() const
{
    return mBodyB;
}

const Math::Vec3& DistanceJoint::localAnchorA() const
{
    return mLocalAnchorA;
}

const Math::Vec3& DistanceJoint::localAnchorB() const
{
    return mLocalAnchorB;
}

Math::Vec3 DistanceJoint::worldAnchorA() const
{
    return mBodyA->pointToWorld(mLocalAnchorA);
}

Math::Vec3 DistanceJoint::worldAnchorB() const
{
    return mBodyB->pointToWorld(mLocalAnchorB);
}

void DistanceJoint::setDistance(f32 minDistance, f32 maxDistance)
{
    mMinDistance = glm::max(minDistance, 0.0f);
    mMaxDistance = glm::max(maxDistance, mMinDistance);
}

f32 DistanceJoint::minDistance() const
{
    return mMinDistance;
}

f32 DistanceJoint::maxDistance() const
{
    return mMaxDistance;
}

void DistanceJoint::calculateProperties()
{
    const Math::Vec3 pointA = worldAnchorA();
    const Math::Vec3 pointB = worldAnchorB();
    const Math::Vec3 delta = pointB - pointA;
    const f32 length = glm::length(delta);
    if (length > 1.0e-6f)
        mWorldNormal = delta / length;

    mArmA = pointB - mBodyA->position();
    mArmB = pointB - mBodyB->position();

    if (mMinDistance == mMaxDistance)
    {
        mMinImpulse = -M_INFINITY;
        mMaxImpulse = M_INFINITY;
        mActive = true;
    }
    else if (length <= mMinDistance)
    {
        mMinImpulse = 0.0f;
        mMaxImpulse = M_INFINITY;
        mActive = true;
    }
    else if (length >= mMaxDistance)
    {
        mMinImpulse = -M_INFINITY;
        mMaxImpulse = 0.0f;
        mActive = true;
    }
    else
    {
        mActive = false;
    }

    if (!mActive)
    {
        mEffectiveMass = 0.0f;
        return;
    }

    f32 inverseEffectiveMass = mBodyA->inverseMass() + mBodyB->inverseMass();
    inverseEffectiveMass += glm::dot(
        mWorldNormal, mBodyA->inverseInertiaTensorWorld() * glm::cross(mArmA, mWorldNormal));
    inverseEffectiveMass += glm::dot(
        mWorldNormal, mBodyB->inverseInertiaTensorWorld() * glm::cross(mArmB, mWorldNormal));
    mEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mEffectiveMass == 0.0f)
        mActive = false;
}

void DistanceJoint::setup(f32 duration)
{
    calculateProperties();
    if (mPreviousDuration > 0.0f)
        mTotalImpulse *= duration / mPreviousDuration;
    else
        mTotalImpulse = 0.0f;
    mTotalImpulse = glm::clamp(mTotalImpulse, mMinImpulse, mMaxImpulse);
    mPreviousDuration = duration;
}

void DistanceJoint::applyVelocityImpulse(f32 impulse)
{
    if (mBodyA->isDynamic())
    {
        mBodyA->setVelocity(mBodyA->velocity() - impulse * mBodyA->inverseMass() * mWorldNormal);
        mBodyA->setAngularVelocity(
            mBodyA->angularVelocity() -
            impulse * (mBodyA->inverseInertiaTensorWorld() * glm::cross(mArmA, mWorldNormal)));
    }
    if (mBodyB->isDynamic())
    {
        mBodyB->setVelocity(mBodyB->velocity() + impulse * mBodyB->inverseMass() * mWorldNormal);
        mBodyB->setAngularVelocity(
            mBodyB->angularVelocity() +
            impulse * (mBodyB->inverseInertiaTensorWorld() * glm::cross(mArmB, mWorldNormal)));
    }
}

void DistanceJoint::warmStart()
{
    if (mActive)
        applyVelocityImpulse(mTotalImpulse);
}

void DistanceJoint::solveVelocity()
{
    if (!mActive)
        return;
    const f32 relativeVelocity =
        glm::dot(mWorldNormal, mBodyA->velocity() - mBodyB->velocity()) +
        glm::dot(glm::cross(mArmA, mWorldNormal), mBodyA->angularVelocity()) -
        glm::dot(glm::cross(mArmB, mWorldNormal), mBodyB->angularVelocity());
    const f32 lambda = mEffectiveMass * relativeVelocity;
    const f32 previous = mTotalImpulse;
    mTotalImpulse = glm::clamp(previous + lambda, mMinImpulse, mMaxImpulse);
    applyVelocityImpulse(mTotalImpulse - previous);
}

void DistanceJoint::solvePosition(f32 baumgarte)
{
    calculateProperties();
    if (!mActive)
        return;
    const Math::Vec3 pointA = worldAnchorA();
    const Math::Vec3 pointB = worldAnchorB();
    const f32 distance = glm::dot(pointB - pointA, mWorldNormal);
    f32 error = 0.0f;
    if (distance < mMinDistance)
        error = distance - mMinDistance;
    else if (distance > mMaxDistance)
        error = distance - mMaxDistance;
    if (error == 0.0f)
        return;
    const f32 lambda = -mEffectiveMass * baumgarte * error;
    mBodyA->applyPositionImpulseAtPoint(-(lambda * mWorldNormal), pointB);
    mBodyB->applyPositionImpulseAtPoint(lambda * mWorldNormal, pointB);
}

}
