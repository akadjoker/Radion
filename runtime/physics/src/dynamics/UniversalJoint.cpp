#include "PCH.h"

#include "dynamics/UniversalJoint.h"

#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

namespace
{

f32 rotationAngleAroundAxis(const Math::quat& q, const Math::vec3& axis)
{
    if (q.w == 0.0f)
        return Math::pi<f32>();
    return 2.0f * std::atan(Math::dot(Math::vec3(q.x, q.y, q.z), axis) / q.w);
}

f32 centerAngleAroundZero(f32 angle)
{
    while (angle < -Math::pi<f32>())
        angle += Math::two_pi<f32>();
    while (angle > Math::pi<f32>())
        angle -= Math::two_pi<f32>();
    return angle;
}

Math::vec3 normalizedPerpendicular(const Math::vec3& v)
{
    if (std::abs(v.x) > std::abs(v.y))
    {
        const f32 length = std::sqrt(v.x * v.x + v.z * v.z);
        return Math::vec3(v.z, 0.0f, -v.x) / length;
    }
    const f32 length = std::sqrt(v.y * v.y + v.z * v.z);
    return Math::vec3(0.0f, v.z, -v.y) / length;
}

}

UniversalJoint::UniversalJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                               const Math::vec3& worldAxisA, const Math::vec3& worldAxisB)
    : UniversalJoint(a, a.pointToLocal(worldAnchor), a.directionToLocal(Math::normalize(worldAxisA)),
                     b, b.pointToLocal(worldAnchor), b.directionToLocal(Math::normalize(worldAxisB)))
{
}

UniversalJoint::UniversalJoint(RigidBody& a, const Math::vec3& localAnchorA,
                               const Math::vec3& localAxisA, RigidBody& b,
                               const Math::vec3& localAnchorB, const Math::vec3& localAxisB)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB),
      mLocalAxisA(Math::normalize(localAxisA)), mLocalAxisB(Math::normalize(localAxisB)),
      mInverseInitialOrientation(Math::conjugate(b.orientation()) * a.orientation())
{
}

RigidBody* UniversalJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* UniversalJoint::bodyB() const
{
    return mBodyB;
}

void UniversalJoint::setLimitsA(f32 minAngle, f32 maxAngle)
{
    mLimitsMinA = Math::clamp(minAngle, -Math::pi<f32>(), 0.0f);
    mLimitsMaxA = Math::clamp(maxAngle, 0.0f, Math::pi<f32>());
    mHasLimitsA = mLimitsMinA > -Math::pi<f32>() || mLimitsMaxA < Math::pi<f32>();
}

void UniversalJoint::setLimitsB(f32 minAngle, f32 maxAngle)
{
    mLimitsMinB = Math::clamp(minAngle, -Math::pi<f32>(), 0.0f);
    mLimitsMaxB = Math::clamp(maxAngle, 0.0f, Math::pi<f32>());
    mHasLimitsB = mLimitsMinB > -Math::pi<f32>() || mLimitsMaxB < Math::pi<f32>();
}

f32 UniversalJoint::currentAngleA() const
{
    const Math::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mBodyA->orientation());
    return rotationAngleAroundAxis(diff, mBodyA->directionToWorld(mLocalAxisA));
}

f32 UniversalJoint::currentAngleB() const
{
    const Math::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mBodyA->orientation());
    return rotationAngleAroundAxis(diff, mBodyB->directionToWorld(mLocalAxisB));
}

void UniversalJoint::setMotorA(f32 targetAngularVelocity, f32 maxTorque)
{
    if (!std::isfinite(targetAngularVelocity) || !std::isfinite(maxTorque))
        return;
    mMotorTargetVelocityA = targetAngularVelocity;
    mMotorMaxTorqueA = Math::max(maxTorque, 0.0f);
    mMotorEnabledA = mMotorMaxTorqueA > 0.0f;
}

void UniversalJoint::setMotorB(f32 targetAngularVelocity, f32 maxTorque)
{
    if (!std::isfinite(targetAngularVelocity) || !std::isfinite(maxTorque))
        return;
    mMotorTargetVelocityB = targetAngularVelocity;
    mMotorMaxTorqueB = Math::max(maxTorque, 0.0f);
    mMotorEnabledB = mMotorMaxTorqueB > 0.0f;
}

void UniversalJoint::disableMotorA()
{
    mMotorEnabledA = false;
    mTotalMotorImpulseA = 0.0f;
}

void UniversalJoint::disableMotorB()
{
    mMotorEnabledB = false;
    mTotalMotorImpulseB = 0.0f;
}

void UniversalJoint::calculatePositionProperties()
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
        mPositionEffectiveMass = Math::inverse(inverseEffectiveMass);
    else
    {
        mPositionEffectiveMass = Math::mat3(0.0f);
        mTotalPositionImpulse = Math::vec3(0.0f);
    }
}

void UniversalJoint::calculatePerpendicularityProperties()
{
    mAxisA = Math::normalize(mBodyA->directionToWorld(mLocalAxisA));
    mAxisB = Math::normalize(mBodyB->directionToWorld(mLocalAxisB));

    const f32 k = Math::dot(mAxisA, mAxisB);
    const Math::vec3 axisBPerpendicular = mAxisB - k * mAxisA;
    const f32 length = Math::length(axisBPerpendicular);
    mPerpendicularAxis = length > 1.0e-6f ? Math::normalize(Math::cross(mAxisA, axisBPerpendicular))
                                          : normalizedPerpendicular(mAxisA);
    mPerpendicularity = k;

    const f32 inverseEffectiveMass = Math::dot(
        mPerpendicularAxis, mBodyA->inverseInertiaTensorWorld() * mPerpendicularAxis +
                                mBodyB->inverseInertiaTensorWorld() * mPerpendicularAxis);
    mPerpendicularEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mPerpendicularEffectiveMass == 0.0f)
        mTotalPerpendicularImpulse = 0.0f;
}

void UniversalJoint::calculateAngles()
{
    const Math::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mBodyA->orientation());
    mThetaA = rotationAngleAroundAxis(diff, mAxisA);
    mThetaB = rotationAngleAroundAxis(diff, mAxisB);
}

void UniversalJoint::calculateLimitProperties(bool hasLimits, f32 theta, f32 minAngle, f32 maxAngle,
                                              const Math::vec3& axis, bool& active,
                                              f32& effectiveMass)
{
    active = hasLimits && (theta <= minAngle || theta >= maxAngle);
    if (!active)
    {
        effectiveMass = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = Math::dot(
        axis, mBodyA->inverseInertiaTensorWorld() * axis + mBodyB->inverseInertiaTensorWorld() * axis);
    effectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (effectiveMass == 0.0f)
        active = false;
}

void UniversalJoint::calculateMotorProperties(bool enabled, const Math::vec3& axis,
                                              f32& effectiveMass)
{
    if (!enabled)
    {
        effectiveMass = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = Math::dot(
        axis, mBodyA->inverseInertiaTensorWorld() * axis + mBodyB->inverseInertiaTensorWorld() * axis);
    effectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
}

void UniversalJoint::setup(f32 duration)
{
    calculatePositionProperties();
    calculatePerpendicularityProperties();
    calculateAngles();
    calculateLimitProperties(mHasLimitsA, mThetaA, mLimitsMinA, mLimitsMaxA, mAxisA, mLimitActiveA,
                             mLimitEffectiveMassA);
    calculateLimitProperties(mHasLimitsB, mThetaB, mLimitsMinB, mLimitsMaxB, mAxisB, mLimitActiveB,
                             mLimitEffectiveMassB);
    calculateMotorProperties(mMotorEnabledA, mAxisA, mMotorEffectiveMassA);
    calculateMotorProperties(mMotorEnabledB, mAxisB, mMotorEffectiveMassB);
    mMotorMaxImpulseA = mMotorMaxTorqueA * duration;
    mMotorMaxImpulseB = mMotorMaxTorqueB * duration;
    if (mPreviousDuration > 0.0f)
    {
        const f32 ratio = duration / mPreviousDuration;
        mTotalPositionImpulse *= ratio;
        mTotalPerpendicularImpulse *= ratio;
        mTotalLimitImpulseA *= ratio;
        mTotalLimitImpulseB *= ratio;
        mTotalMotorImpulseA *= ratio;
        mTotalMotorImpulseB *= ratio;
    }
    else
    {
        mTotalPositionImpulse = Math::vec3(0.0f);
        mTotalPerpendicularImpulse = 0.0f;
        mTotalLimitImpulseA = 0.0f;
        mTotalLimitImpulseB = 0.0f;
        mTotalMotorImpulseA = 0.0f;
        mTotalMotorImpulseB = 0.0f;
    }
    mTotalMotorImpulseA = Math::clamp(mTotalMotorImpulseA, -mMotorMaxImpulseA, mMotorMaxImpulseA);
    mTotalMotorImpulseB = Math::clamp(mTotalMotorImpulseB, -mMotorMaxImpulseB, mMotorMaxImpulseB);
    mPreviousDuration = duration;
}

void UniversalJoint::applyLinearImpulse(const Math::vec3& impulse)
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

void UniversalJoint::applyAngularImpulse(const Math::vec3& impulse)
{
    if (mBodyA->isDynamic())
        mBodyA->setAngularVelocity(mBodyA->angularVelocity() -
                                   mBodyA->inverseInertiaTensorWorld() * impulse);
    if (mBodyB->isDynamic())
        mBodyB->setAngularVelocity(mBodyB->angularVelocity() +
                                   mBodyB->inverseInertiaTensorWorld() * impulse);
}

void UniversalJoint::warmStart()
{
    if (mMotorEnabledA)
        applyAngularImpulse(mAxisA * mTotalMotorImpulseA);
    if (mMotorEnabledB)
        applyAngularImpulse(mAxisB * mTotalMotorImpulseB);
    applyLinearImpulse(mTotalPositionImpulse);
    applyAngularImpulse(mPerpendicularAxis * mTotalPerpendicularImpulse);
    if (mLimitActiveA)
        applyAngularImpulse(mAxisA * mTotalLimitImpulseA);
    if (mLimitActiveB)
        applyAngularImpulse(mAxisB * mTotalLimitImpulseB);
}

void UniversalJoint::solveVelocity()
{
    if (mMotorEnabledA)
    {
        const f32 relative = Math::dot(mAxisA, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = (relative + mMotorTargetVelocityA) * mMotorEffectiveMassA;
        const f32 previous = mTotalMotorImpulseA;
        mTotalMotorImpulseA = Math::clamp(previous + impulse, -mMotorMaxImpulseA, mMotorMaxImpulseA);
        applyAngularImpulse(mAxisA * (mTotalMotorImpulseA - previous));
    }
    if (mMotorEnabledB)
    {
        const f32 relative = Math::dot(mAxisB, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = (relative + mMotorTargetVelocityB) * mMotorEffectiveMassB;
        const f32 previous = mTotalMotorImpulseB;
        mTotalMotorImpulseB = Math::clamp(previous + impulse, -mMotorMaxImpulseB, mMotorMaxImpulseB);
        applyAngularImpulse(mAxisB * (mTotalMotorImpulseB - previous));
    }

    const Math::vec3 relativeVelocity =
        mBodyB->velocity() + Math::cross(mBodyB->angularVelocity(), mArmB) -
        mBodyA->velocity() - Math::cross(mBodyA->angularVelocity(), mArmA);
    const Math::vec3 positionImpulse = -(mPositionEffectiveMass * relativeVelocity);
    mTotalPositionImpulse += positionImpulse;
    applyLinearImpulse(positionImpulse);

    const f32 perpJv =
        Math::dot(mPerpendicularAxis, mBodyA->angularVelocity() - mBodyB->angularVelocity());
    const f32 perpImpulse = mPerpendicularEffectiveMass * perpJv;
    mTotalPerpendicularImpulse += perpImpulse;
    applyAngularImpulse(mPerpendicularAxis * perpImpulse);

    if (mLimitActiveA)
    {
        f32 minImpulse = -M_INFINITY;
        f32 maxImpulse = M_INFINITY;
        if (mLimitsMinA != mLimitsMaxA)
        {
            const f32 distanceToMin = centerAngleAroundZero(mThetaA - mLimitsMinA);
            const f32 distanceToMax = centerAngleAroundZero(mThetaA - mLimitsMaxA);
            if (std::abs(distanceToMin) < std::abs(distanceToMax))
                minImpulse = 0.0f;
            else
                maxImpulse = 0.0f;
        }
        const f32 relative = Math::dot(mAxisA, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = mLimitEffectiveMassA * relative;
        const f32 previous = mTotalLimitImpulseA;
        mTotalLimitImpulseA = Math::clamp(previous + impulse, minImpulse, maxImpulse);
        applyAngularImpulse(mAxisA * (mTotalLimitImpulseA - previous));
    }

    if (mLimitActiveB)
    {
        f32 minImpulse = -M_INFINITY;
        f32 maxImpulse = M_INFINITY;
        if (mLimitsMinB != mLimitsMaxB)
        {
            const f32 distanceToMin = centerAngleAroundZero(mThetaB - mLimitsMinB);
            const f32 distanceToMax = centerAngleAroundZero(mThetaB - mLimitsMaxB);
            if (std::abs(distanceToMin) < std::abs(distanceToMax))
                minImpulse = 0.0f;
            else
                maxImpulse = 0.0f;
        }
        const f32 relative = Math::dot(mAxisB, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = mLimitEffectiveMassB * relative;
        const f32 previous = mTotalLimitImpulseB;
        mTotalLimitImpulseB = Math::clamp(previous + impulse, minImpulse, maxImpulse);
        applyAngularImpulse(mAxisB * (mTotalLimitImpulseB - previous));
    }
}

void UniversalJoint::solvePosition(f32 baumgarte)
{
    calculatePositionProperties();
    const Math::vec3 pointA = mBodyA->position() + mArmA;
    const Math::vec3 pointB = mBodyB->position() + mArmB;
    const Math::vec3 positionImpulse = -(mPositionEffectiveMass * (pointB - pointA)) * baumgarte;
    mBodyA->applyPositionImpulseAtPoint(-positionImpulse, pointA);
    mBodyB->applyPositionImpulseAtPoint(positionImpulse, pointB);

    calculatePerpendicularityProperties();
    if (mPerpendicularity != 0.0f)
    {
        const f32 lambda = -mPerpendicularEffectiveMass * baumgarte * mPerpendicularity;
        if (mBodyA->isDynamic())
        {
            const Math::vec3 step = mBodyA->inverseInertiaTensorWorld() * mPerpendicularAxis * -lambda;
            const Math::quat spin(0.0f, step);
            mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
        }
        if (mBodyB->isDynamic())
        {
            const Math::vec3 step = mBodyB->inverseInertiaTensorWorld() * mPerpendicularAxis * lambda;
            const Math::quat spin(0.0f, step);
            mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
        }
    }

    if (mHasLimitsA || mHasLimitsB)
    {
        calculatePerpendicularityProperties();
        calculateAngles();
    }

    if (mHasLimitsA)
    {
        calculateLimitProperties(mHasLimitsA, mThetaA, mLimitsMinA, mLimitsMaxA, mAxisA,
                                 mLimitActiveA, mLimitEffectiveMassA);
        if (mLimitActiveA)
        {
            const f32 distanceToMin = centerAngleAroundZero(mThetaA - mLimitsMinA);
            const f32 distanceToMax = centerAngleAroundZero(mThetaA - mLimitsMaxA);
            const f32 error = std::abs(distanceToMin) < std::abs(distanceToMax) ? distanceToMin
                                                                                 : distanceToMax;
            const f32 lambda = -mLimitEffectiveMassA * baumgarte * error;
            if (mBodyA->isDynamic())
            {
                const Math::vec3 step = mBodyA->inverseInertiaTensorWorld() * mAxisA * -lambda;
                const Math::quat spin(0.0f, step);
                mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
            }
            if (mBodyB->isDynamic())
            {
                const Math::vec3 step = mBodyB->inverseInertiaTensorWorld() * mAxisA * lambda;
                const Math::quat spin(0.0f, step);
                mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
            }
        }
    }

    if (mHasLimitsB)
    {
        calculateLimitProperties(mHasLimitsB, mThetaB, mLimitsMinB, mLimitsMaxB, mAxisB,
                                 mLimitActiveB, mLimitEffectiveMassB);
        if (mLimitActiveB)
        {
            const f32 distanceToMin = centerAngleAroundZero(mThetaB - mLimitsMinB);
            const f32 distanceToMax = centerAngleAroundZero(mThetaB - mLimitsMaxB);
            const f32 error = std::abs(distanceToMin) < std::abs(distanceToMax) ? distanceToMin
                                                                                 : distanceToMax;
            const f32 lambda = -mLimitEffectiveMassB * baumgarte * error;
            if (mBodyA->isDynamic())
            {
                const Math::vec3 step = mBodyA->inverseInertiaTensorWorld() * mAxisB * -lambda;
                const Math::quat spin(0.0f, step);
                mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
            }
            if (mBodyB->isDynamic())
            {
                const Math::vec3 step = mBodyB->inverseInertiaTensorWorld() * mAxisB * lambda;
                const Math::quat spin(0.0f, step);
                mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
            }
        }
    }
}

}
