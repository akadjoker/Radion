#include "PCH.h"

#include "dynamics/PointJoint.h"

#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

PointJoint::PointJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor)
    : PointJoint(a, a.pointToLocal(worldAnchor), b, b.pointToLocal(worldAnchor))
{
}

PointJoint::PointJoint(RigidBody& a, const Math::vec3& localAnchorA, RigidBody& b,
                       const Math::vec3& localAnchorB)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB)
{
}

RigidBody* PointJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* PointJoint::bodyB() const
{
    return mBodyB;
}

const Math::vec3& PointJoint::localAnchorA() const
{
    return mLocalAnchorA;
}

const Math::vec3& PointJoint::localAnchorB() const
{
    return mLocalAnchorB;
}

Math::vec3 PointJoint::worldAnchorA() const
{
    return mBodyA->pointToWorld(mLocalAnchorA);
}

Math::vec3 PointJoint::worldAnchorB() const
{
    return mBodyB->pointToWorld(mLocalAnchorB);
}

void PointJoint::setMotor(const Math::vec3& localAxisA, f32 targetVelocity, f32 maxTorque)
{
    const f32 length = Math::length(localAxisA);
    if (length <= 1.0e-6f || !std::isfinite(targetVelocity) || !std::isfinite(maxTorque))
        return;
    mMotorLocalAxisA = localAxisA / length;
    mMotorTargetVelocity = targetVelocity;
    mMotorMaxTorque = Math::max(maxTorque, 0.0f);
    mMotorEnabled = mMotorMaxTorque > 0.0f;
}

void PointJoint::disableMotor()
{
    mMotorEnabled = false;
    mMotorTotalImpulse = 0.0f;
}

void PointJoint::calculateProperties()
{
    mArmA = mBodyA->directionToWorld(mLocalAnchorA);
    mArmB = mBodyB->directionToWorld(mLocalAnchorB);
    Math::mat3 inverseEffectiveMass(0.0f);
    const Math::vec3 axes[] = {Math::vec3(1.0f, 0.0f, 0.0f), Math::vec3(0.0f, 1.0f, 0.0f),
                              Math::vec3(0.0f, 0.0f, 1.0f)};
    for (u32 axis = 0; axis < 3; ++axis)
    {
        Math::vec3 response = axes[axis] * (mBodyA->inverseMass() + mBodyB->inverseMass());
        response += Math::cross(
            mBodyA->inverseInertiaTensorWorld() * Math::cross(mArmA, axes[axis]), mArmA);
        response += Math::cross(
            mBodyB->inverseInertiaTensorWorld() * Math::cross(mArmB, axes[axis]), mArmB);
        inverseEffectiveMass[axis] = response;
    }
    const f32 determinant = Math::determinant(inverseEffectiveMass);
    if (std::abs(determinant) > 1.0e-9f && std::isfinite(determinant))
        mEffectiveMass = Math::inverse(inverseEffectiveMass);
    else
    {
        mEffectiveMass = Math::mat3(0.0f);
        mTotalImpulse = Math::vec3(0.0f);
    }
}

void PointJoint::setup(f32 duration)
{
    calculateProperties();
    mMotorWorldAxis = mBodyA->directionToWorld(mMotorLocalAxisA);
    const f32 inverseMotorMass =
        Math::dot(mMotorWorldAxis,
                 (mBodyA->inverseInertiaTensorWorld() + mBodyB->inverseInertiaTensorWorld()) *
                     mMotorWorldAxis);
    mMotorEffectiveMass = inverseMotorMass > 1.0e-9f ? 1.0f / inverseMotorMass : 0.0f;
    mMotorMaxImpulse = mMotorMaxTorque * duration;
    if (mPreviousDuration > 0.0f)
    {
        mTotalImpulse *= duration / mPreviousDuration;
        mMotorTotalImpulse *= duration / mPreviousDuration;
    }
    else
    {
        mTotalImpulse = Math::vec3(0.0f);
        mMotorTotalImpulse = 0.0f;
    }
    mMotorTotalImpulse = Math::clamp(mMotorTotalImpulse, -mMotorMaxImpulse, mMotorMaxImpulse);
    mPreviousDuration = duration;
}

void PointJoint::applyVelocityImpulse(const Math::vec3& impulse)
{
    if (mBodyA->isDynamic())
    {
        mBodyA->setVelocity(mBodyA->velocity() - impulse * mBodyA->inverseMass());
        mBodyA->setAngularVelocity(
            mBodyA->angularVelocity() -
            mBodyA->inverseInertiaTensorWorld() * Math::cross(mArmA, impulse));
    }
    if (mBodyB->isDynamic())
    {
        mBodyB->setVelocity(mBodyB->velocity() + impulse * mBodyB->inverseMass());
        mBodyB->setAngularVelocity(
            mBodyB->angularVelocity() +
            mBodyB->inverseInertiaTensorWorld() * Math::cross(mArmB, impulse));
    }
}

void PointJoint::warmStart()
{
    applyVelocityImpulse(mTotalImpulse);
    if (mMotorEnabled)
        applyMotorImpulse(mMotorTotalImpulse);
}

void PointJoint::applyMotorImpulse(f32 impulse)
{
    const Math::vec3 angularImpulse = mMotorWorldAxis * impulse;
    if (mBodyA->isDynamic())
        mBodyA->setAngularVelocity(
            mBodyA->angularVelocity() - mBodyA->inverseInertiaTensorWorld() * angularImpulse);
    if (mBodyB->isDynamic())
        mBodyB->setAngularVelocity(
            mBodyB->angularVelocity() + mBodyB->inverseInertiaTensorWorld() * angularImpulse);
}

void PointJoint::solveVelocity()
{
    const Math::vec3 relativeVelocity =
        mBodyB->velocity() + Math::cross(mBodyB->angularVelocity(), mArmB) -
        mBodyA->velocity() - Math::cross(mBodyA->angularVelocity(), mArmA);
    const Math::vec3 impulse = -(mEffectiveMass * relativeVelocity);
    mTotalImpulse += impulse;
    applyVelocityImpulse(impulse);
    if (mMotorEnabled && mMotorEffectiveMass > 0.0f)
    {
        const f32 relativeVelocity =
            Math::dot(mBodyB->angularVelocity() - mBodyA->angularVelocity(), mMotorWorldAxis);
        const f32 motorImpulse = (mMotorTargetVelocity - relativeVelocity) * mMotorEffectiveMass;
        const f32 previous = mMotorTotalImpulse;
        mMotorTotalImpulse =
            Math::clamp(previous + motorImpulse, -mMotorMaxImpulse, mMotorMaxImpulse);
        applyMotorImpulse(mMotorTotalImpulse - previous);
    }
}

void PointJoint::solvePosition(f32 baumgarte)
{
    calculateProperties();
    const Math::vec3 pointA = mBodyA->position() + mArmA;
    const Math::vec3 pointB = mBodyB->position() + mArmB;
    const Math::vec3 impulse = -(mEffectiveMass * (pointB - pointA)) * baumgarte;
    mBodyA->applyPositionImpulseAtPoint(-impulse, pointA);
    mBodyB->applyPositionImpulseAtPoint(impulse, pointB);
}

}
