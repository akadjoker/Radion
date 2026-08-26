#include "PCH.h"

#include "dynamics/UniversalJoint.h"

#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

namespace
{

f32 rotationAngleAroundAxis(const glm::quat& q, const glm::vec3& axis)
{
    if (q.w == 0.0f)
        return glm::pi<f32>();
    return 2.0f * std::atan(glm::dot(glm::vec3(q.x, q.y, q.z), axis) / q.w);
}

f32 centerAngleAroundZero(f32 angle)
{
    while (angle < -glm::pi<f32>())
        angle += glm::two_pi<f32>();
    while (angle > glm::pi<f32>())
        angle -= glm::two_pi<f32>();
    return angle;
}

glm::vec3 normalizedPerpendicular(const glm::vec3& v)
{
    if (std::abs(v.x) > std::abs(v.y))
    {
        const f32 length = std::sqrt(v.x * v.x + v.z * v.z);
        return glm::vec3(v.z, 0.0f, -v.x) / length;
    }
    const f32 length = std::sqrt(v.y * v.y + v.z * v.z);
    return glm::vec3(0.0f, v.z, -v.y) / length;
}

}

UniversalJoint::UniversalJoint(RigidBody& a, RigidBody& b, const glm::vec3& worldAnchor,
                               const glm::vec3& worldAxisA, const glm::vec3& worldAxisB)
    : UniversalJoint(a, a.pointToLocal(worldAnchor), a.directionToLocal(glm::normalize(worldAxisA)),
                     b, b.pointToLocal(worldAnchor), b.directionToLocal(glm::normalize(worldAxisB)))
{
}

UniversalJoint::UniversalJoint(RigidBody& a, const glm::vec3& localAnchorA,
                               const glm::vec3& localAxisA, RigidBody& b,
                               const glm::vec3& localAnchorB, const glm::vec3& localAxisB)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB),
      mLocalAxisA(glm::normalize(localAxisA)), mLocalAxisB(glm::normalize(localAxisB)),
      mInverseInitialOrientation(glm::conjugate(b.orientation()) * a.orientation())
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
    mLimitsMinA = glm::clamp(minAngle, -glm::pi<f32>(), 0.0f);
    mLimitsMaxA = glm::clamp(maxAngle, 0.0f, glm::pi<f32>());
    mHasLimitsA = mLimitsMinA > -glm::pi<f32>() || mLimitsMaxA < glm::pi<f32>();
}

void UniversalJoint::setLimitsB(f32 minAngle, f32 maxAngle)
{
    mLimitsMinB = glm::clamp(minAngle, -glm::pi<f32>(), 0.0f);
    mLimitsMaxB = glm::clamp(maxAngle, 0.0f, glm::pi<f32>());
    mHasLimitsB = mLimitsMinB > -glm::pi<f32>() || mLimitsMaxB < glm::pi<f32>();
}

f32 UniversalJoint::currentAngleA() const
{
    const glm::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           glm::conjugate(mBodyA->orientation());
    return rotationAngleAroundAxis(diff, mBodyA->directionToWorld(mLocalAxisA));
}

f32 UniversalJoint::currentAngleB() const
{
    const glm::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           glm::conjugate(mBodyA->orientation());
    return rotationAngleAroundAxis(diff, mBodyB->directionToWorld(mLocalAxisB));
}

void UniversalJoint::setMotorA(f32 targetAngularVelocity, f32 maxTorque)
{
    if (!std::isfinite(targetAngularVelocity) || !std::isfinite(maxTorque))
        return;
    mMotorTargetVelocityA = targetAngularVelocity;
    mMotorMaxTorqueA = glm::max(maxTorque, 0.0f);
    mMotorEnabledA = mMotorMaxTorqueA > 0.0f;
}

void UniversalJoint::setMotorB(f32 targetAngularVelocity, f32 maxTorque)
{
    if (!std::isfinite(targetAngularVelocity) || !std::isfinite(maxTorque))
        return;
    mMotorTargetVelocityB = targetAngularVelocity;
    mMotorMaxTorqueB = glm::max(maxTorque, 0.0f);
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

void UniversalJoint::calculatePerpendicularityProperties()
{
    mAxisA = glm::normalize(mBodyA->directionToWorld(mLocalAxisA));
    mAxisB = glm::normalize(mBodyB->directionToWorld(mLocalAxisB));

    const f32 k = glm::dot(mAxisA, mAxisB);
    const glm::vec3 axisBPerpendicular = mAxisB - k * mAxisA;
    const f32 length = glm::length(axisBPerpendicular);
    mPerpendicularAxis = length > 1.0e-6f ? glm::normalize(glm::cross(mAxisA, axisBPerpendicular))
                                          : normalizedPerpendicular(mAxisA);
    mPerpendicularity = k;

    const f32 inverseEffectiveMass = glm::dot(
        mPerpendicularAxis, mBodyA->inverseInertiaTensorWorld() * mPerpendicularAxis +
                                mBodyB->inverseInertiaTensorWorld() * mPerpendicularAxis);
    mPerpendicularEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mPerpendicularEffectiveMass == 0.0f)
        mTotalPerpendicularImpulse = 0.0f;
}

void UniversalJoint::calculateAngles()
{
    const glm::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           glm::conjugate(mBodyA->orientation());
    mThetaA = rotationAngleAroundAxis(diff, mAxisA);
    mThetaB = rotationAngleAroundAxis(diff, mAxisB);
}

void UniversalJoint::calculateLimitProperties(bool hasLimits, f32 theta, f32 minAngle, f32 maxAngle,
                                              const glm::vec3& axis, bool& active,
                                              f32& effectiveMass)
{
    active = hasLimits && (theta <= minAngle || theta >= maxAngle);
    if (!active)
    {
        effectiveMass = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = glm::dot(
        axis, mBodyA->inverseInertiaTensorWorld() * axis + mBodyB->inverseInertiaTensorWorld() * axis);
    effectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (effectiveMass == 0.0f)
        active = false;
}

void UniversalJoint::calculateMotorProperties(bool enabled, const glm::vec3& axis,
                                              f32& effectiveMass)
{
    if (!enabled)
    {
        effectiveMass = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = glm::dot(
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
        mTotalPositionImpulse = glm::vec3(0.0f);
        mTotalPerpendicularImpulse = 0.0f;
        mTotalLimitImpulseA = 0.0f;
        mTotalLimitImpulseB = 0.0f;
        mTotalMotorImpulseA = 0.0f;
        mTotalMotorImpulseB = 0.0f;
    }
    mTotalMotorImpulseA = glm::clamp(mTotalMotorImpulseA, -mMotorMaxImpulseA, mMotorMaxImpulseA);
    mTotalMotorImpulseB = glm::clamp(mTotalMotorImpulseB, -mMotorMaxImpulseB, mMotorMaxImpulseB);
    mPreviousDuration = duration;
}

void UniversalJoint::applyLinearImpulse(const glm::vec3& impulse)
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

void UniversalJoint::applyAngularImpulse(const glm::vec3& impulse)
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
        const f32 relative = glm::dot(mAxisA, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = (relative + mMotorTargetVelocityA) * mMotorEffectiveMassA;
        const f32 previous = mTotalMotorImpulseA;
        mTotalMotorImpulseA = glm::clamp(previous + impulse, -mMotorMaxImpulseA, mMotorMaxImpulseA);
        applyAngularImpulse(mAxisA * (mTotalMotorImpulseA - previous));
    }
    if (mMotorEnabledB)
    {
        const f32 relative = glm::dot(mAxisB, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = (relative + mMotorTargetVelocityB) * mMotorEffectiveMassB;
        const f32 previous = mTotalMotorImpulseB;
        mTotalMotorImpulseB = glm::clamp(previous + impulse, -mMotorMaxImpulseB, mMotorMaxImpulseB);
        applyAngularImpulse(mAxisB * (mTotalMotorImpulseB - previous));
    }

    const glm::vec3 relativeVelocity =
        mBodyB->velocity() + glm::cross(mBodyB->angularVelocity(), mArmB) -
        mBodyA->velocity() - glm::cross(mBodyA->angularVelocity(), mArmA);
    const glm::vec3 positionImpulse = -(mPositionEffectiveMass * relativeVelocity);
    mTotalPositionImpulse += positionImpulse;
    applyLinearImpulse(positionImpulse);

    const f32 perpJv =
        glm::dot(mPerpendicularAxis, mBodyA->angularVelocity() - mBodyB->angularVelocity());
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
        const f32 relative = glm::dot(mAxisA, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = mLimitEffectiveMassA * relative;
        const f32 previous = mTotalLimitImpulseA;
        mTotalLimitImpulseA = glm::clamp(previous + impulse, minImpulse, maxImpulse);
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
        const f32 relative = glm::dot(mAxisB, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = mLimitEffectiveMassB * relative;
        const f32 previous = mTotalLimitImpulseB;
        mTotalLimitImpulseB = glm::clamp(previous + impulse, minImpulse, maxImpulse);
        applyAngularImpulse(mAxisB * (mTotalLimitImpulseB - previous));
    }
}

void UniversalJoint::solvePosition(f32 baumgarte)
{
    calculatePositionProperties();
    const glm::vec3 pointA = mBodyA->position() + mArmA;
    const glm::vec3 pointB = mBodyB->position() + mArmB;
    const glm::vec3 positionImpulse = -(mPositionEffectiveMass * (pointB - pointA)) * baumgarte;
    mBodyA->applyPositionImpulseAtPoint(-positionImpulse, pointA);
    mBodyB->applyPositionImpulseAtPoint(positionImpulse, pointB);

    calculatePerpendicularityProperties();
    if (mPerpendicularity != 0.0f)
    {
        const f32 lambda = -mPerpendicularEffectiveMass * baumgarte * mPerpendicularity;
        if (mBodyA->isDynamic())
        {
            const glm::vec3 step = mBodyA->inverseInertiaTensorWorld() * mPerpendicularAxis * -lambda;
            const glm::quat spin(0.0f, step);
            mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
        }
        if (mBodyB->isDynamic())
        {
            const glm::vec3 step = mBodyB->inverseInertiaTensorWorld() * mPerpendicularAxis * lambda;
            const glm::quat spin(0.0f, step);
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
                const glm::vec3 step = mBodyA->inverseInertiaTensorWorld() * mAxisA * -lambda;
                const glm::quat spin(0.0f, step);
                mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
            }
            if (mBodyB->isDynamic())
            {
                const glm::vec3 step = mBodyB->inverseInertiaTensorWorld() * mAxisA * lambda;
                const glm::quat spin(0.0f, step);
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
                const glm::vec3 step = mBodyA->inverseInertiaTensorWorld() * mAxisB * -lambda;
                const glm::quat spin(0.0f, step);
                mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
            }
            if (mBodyB->isDynamic())
            {
                const glm::vec3 step = mBodyB->inverseInertiaTensorWorld() * mAxisB * lambda;
                const glm::quat spin(0.0f, step);
                mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
            }
        }
    }
}

}
