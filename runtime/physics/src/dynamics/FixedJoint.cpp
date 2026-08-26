#include "PCH.h"

#include "dynamics/FixedJoint.h"

#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

FixedJoint::FixedJoint(RigidBody& a, RigidBody& b, const Math::Vec3& worldAnchor)
    : FixedJoint(a, a.pointToLocal(worldAnchor), b, b.pointToLocal(worldAnchor))
{
}

FixedJoint::FixedJoint(RigidBody& a, const Math::Vec3& localAnchorA, RigidBody& b,
                       const Math::Vec3& localAnchorB)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB),
      mInverseInitialOrientation(glm::conjugate(b.orientation()) * a.orientation())
{
}

RigidBody* FixedJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* FixedJoint::bodyB() const
{
    return mBodyB;
}

void FixedJoint::calculatePositionProperties()
{
    mArmA = mBodyA->directionToWorld(mLocalAnchorA);
    mArmB = mBodyB->directionToWorld(mLocalAnchorB);
    Math::Mat3 inverseEffectiveMass(0.0f);
    const Math::Vec3 axes[] = {Math::Vec3(1.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f),
                              Math::Vec3(0.0f, 0.0f, 1.0f)};
    for (u32 axis = 0; axis < 3; ++axis)
    {
        Math::Vec3 response = axes[axis] * (mBodyA->inverseMass() + mBodyB->inverseMass());
        response += glm::cross(
            mBodyA->inverseInertiaTensorWorld() * glm::cross(mArmA, axes[axis]), mArmA);
        response += glm::cross(
            mBodyB->inverseInertiaTensorWorld() * glm::cross(mArmB, axes[axis]), mArmB);
        inverseEffectiveMass[axis] = response;
    }
    const f32 determinant = glm::determinant(inverseEffectiveMass);
    if (std::abs(determinant) > 1.0e-9f && std::isfinite(determinant))
        mPositionEffectiveMass = glm::inverse(inverseEffectiveMass);
    else
    {
        mPositionEffectiveMass = Math::Mat3(0.0f);
        mTotalPositionImpulse = Math::Vec3(0.0f);
    }
}

void FixedJoint::calculateRotationProperties()
{
    const Math::Mat3 inverseInertiaSum =
        mBodyA->inverseInertiaTensorWorld() + mBodyB->inverseInertiaTensorWorld();
    const f32 determinant = glm::determinant(inverseInertiaSum);
    if (std::abs(determinant) > 1.0e-9f && std::isfinite(determinant))
        mRotationEffectiveMass = glm::inverse(inverseInertiaSum);
    else
    {
        mRotationEffectiveMass = Math::Mat3(0.0f);
        mTotalRotationImpulse = Math::Vec3(0.0f);
    }
}

void FixedJoint::setup(f32 duration)
{
    calculatePositionProperties();
    calculateRotationProperties();
    if (mPreviousDuration > 0.0f)
    {
        mTotalPositionImpulse *= duration / mPreviousDuration;
        mTotalRotationImpulse *= duration / mPreviousDuration;
    }
    else
    {
        mTotalPositionImpulse = Math::Vec3(0.0f);
        mTotalRotationImpulse = Math::Vec3(0.0f);
    }
    mPreviousDuration = duration;
}

void FixedJoint::applyVelocityImpulse(const Math::Vec3& impulse)
{
    if (mBodyA->isDynamic())
    {
        mBodyA->setVelocity(mBodyA->velocity() - impulse * mBodyA->inverseMass());
        mBodyA->setAngularVelocity(
            mBodyA->angularVelocity() -
            mBodyA->inverseInertiaTensorWorld() * glm::cross(mArmA, impulse));
    }
    if (mBodyB->isDynamic())
    {
        mBodyB->setVelocity(mBodyB->velocity() + impulse * mBodyB->inverseMass());
        mBodyB->setAngularVelocity(
            mBodyB->angularVelocity() +
            mBodyB->inverseInertiaTensorWorld() * glm::cross(mArmB, impulse));
    }
}

void FixedJoint::applyAngularVelocityImpulse(const Math::Vec3& impulse)
{
    if (mBodyA->isDynamic())
        mBodyA->setAngularVelocity(mBodyA->angularVelocity() -
                                   mBodyA->inverseInertiaTensorWorld() * impulse);
    if (mBodyB->isDynamic())
        mBodyB->setAngularVelocity(mBodyB->angularVelocity() +
                                   mBodyB->inverseInertiaTensorWorld() * impulse);
}

void FixedJoint::warmStart()
{
    applyAngularVelocityImpulse(mTotalRotationImpulse);
    applyVelocityImpulse(mTotalPositionImpulse);
}

void FixedJoint::solveVelocity()
{
    const Math::Vec3 rotationImpulse =
        mRotationEffectiveMass * (mBodyA->angularVelocity() - mBodyB->angularVelocity());
    mTotalRotationImpulse += rotationImpulse;
    applyAngularVelocityImpulse(rotationImpulse);

    const Math::Vec3 relativeVelocity =
        mBodyB->velocity() + glm::cross(mBodyB->angularVelocity(), mArmB) -
        mBodyA->velocity() - glm::cross(mBodyA->angularVelocity(), mArmA);
    const Math::Vec3 positionImpulse = -(mPositionEffectiveMass * relativeVelocity);
    mTotalPositionImpulse += positionImpulse;
    applyVelocityImpulse(positionImpulse);
}

void FixedJoint::solvePosition(f32 baumgarte)
{
    calculateRotationProperties();
    Math::Quaternion diff =
        mBodyB->orientation() * mInverseInitialOrientation * glm::conjugate(mBodyA->orientation());
    if (diff.w < 0.0f)
        diff = -diff;
    const Math::Vec3 rotationError(2.0f * diff.x, 2.0f * diff.y, 2.0f * diff.z);
    if (rotationError != Math::Vec3(0.0f))
    {
        const Math::Vec3 lambda = -baumgarte * (mRotationEffectiveMass * rotationError);
        if (mBodyA->isDynamic())
        {
            const Math::Vec3 step = mBodyA->inverseInertiaTensorWorld() * -lambda;
            const Math::Quaternion spin(0.0f, step);
            mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
        }
        if (mBodyB->isDynamic())
        {
            const Math::Vec3 step = mBodyB->inverseInertiaTensorWorld() * lambda;
            const Math::Quaternion spin(0.0f, step);
            mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
        }
    }

    calculatePositionProperties();
    const Math::Vec3 pointA = mBodyA->position() + mArmA;
    const Math::Vec3 pointB = mBodyB->position() + mArmB;
    const Math::Vec3 impulse = -(mPositionEffectiveMass * (pointB - pointA)) * baumgarte;
    mBodyA->applyPositionImpulseAtPoint(-impulse, pointA);
    mBodyB->applyPositionImpulseAtPoint(impulse, pointB);
}

}
