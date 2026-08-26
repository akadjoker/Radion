#include "PCH.h"

#include "dynamics/FixedJoint.h"

#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

FixedJoint::FixedJoint(RigidBody& a, RigidBody& b, const glm::vec3& worldAnchor)
    : FixedJoint(a, a.pointToLocal(worldAnchor), b, b.pointToLocal(worldAnchor))
{
}

FixedJoint::FixedJoint(RigidBody& a, const glm::vec3& localAnchorA, RigidBody& b,
                       const glm::vec3& localAnchorB)
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
    glm::mat3 inverseEffectiveMass(0.0f);
    const glm::vec3 axes[] = {glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f),
                              glm::vec3(0.0f, 0.0f, 1.0f)};
    for (u32 axis = 0; axis < 3; ++axis)
    {
        glm::vec3 response = axes[axis] * (mBodyA->inverseMass() + mBodyB->inverseMass());
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
        mPositionEffectiveMass = glm::mat3(0.0f);
        mTotalPositionImpulse = glm::vec3(0.0f);
    }
}

void FixedJoint::calculateRotationProperties()
{
    const glm::mat3 inverseInertiaSum =
        mBodyA->inverseInertiaTensorWorld() + mBodyB->inverseInertiaTensorWorld();
    const f32 determinant = glm::determinant(inverseInertiaSum);
    if (std::abs(determinant) > 1.0e-9f && std::isfinite(determinant))
        mRotationEffectiveMass = glm::inverse(inverseInertiaSum);
    else
    {
        mRotationEffectiveMass = glm::mat3(0.0f);
        mTotalRotationImpulse = glm::vec3(0.0f);
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
        mTotalPositionImpulse = glm::vec3(0.0f);
        mTotalRotationImpulse = glm::vec3(0.0f);
    }
    mPreviousDuration = duration;
}

void FixedJoint::applyVelocityImpulse(const glm::vec3& impulse)
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

void FixedJoint::applyAngularVelocityImpulse(const glm::vec3& impulse)
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
    const glm::vec3 rotationImpulse =
        mRotationEffectiveMass * (mBodyA->angularVelocity() - mBodyB->angularVelocity());
    mTotalRotationImpulse += rotationImpulse;
    applyAngularVelocityImpulse(rotationImpulse);

    const glm::vec3 relativeVelocity =
        mBodyB->velocity() + glm::cross(mBodyB->angularVelocity(), mArmB) -
        mBodyA->velocity() - glm::cross(mBodyA->angularVelocity(), mArmA);
    const glm::vec3 positionImpulse = -(mPositionEffectiveMass * relativeVelocity);
    mTotalPositionImpulse += positionImpulse;
    applyVelocityImpulse(positionImpulse);
}

void FixedJoint::solvePosition(f32 baumgarte)
{
    calculateRotationProperties();
    glm::quat diff =
        mBodyB->orientation() * mInverseInitialOrientation * glm::conjugate(mBodyA->orientation());
    if (diff.w < 0.0f)
        diff = -diff;
    const glm::vec3 rotationError(2.0f * diff.x, 2.0f * diff.y, 2.0f * diff.z);
    if (rotationError != glm::vec3(0.0f))
    {
        const glm::vec3 lambda = -baumgarte * (mRotationEffectiveMass * rotationError);
        if (mBodyA->isDynamic())
        {
            const glm::vec3 step = mBodyA->inverseInertiaTensorWorld() * -lambda;
            const glm::quat spin(0.0f, step);
            mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
        }
        if (mBodyB->isDynamic())
        {
            const glm::vec3 step = mBodyB->inverseInertiaTensorWorld() * lambda;
            const glm::quat spin(0.0f, step);
            mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
        }
    }

    calculatePositionProperties();
    const glm::vec3 pointA = mBodyA->position() + mArmA;
    const glm::vec3 pointB = mBodyB->position() + mArmB;
    const glm::vec3 impulse = -(mPositionEffectiveMass * (pointB - pointA)) * baumgarte;
    mBodyA->applyPositionImpulseAtPoint(-impulse, pointA);
    mBodyB->applyPositionImpulseAtPoint(impulse, pointB);
}

}
