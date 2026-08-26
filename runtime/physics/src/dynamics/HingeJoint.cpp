#include "PCH.h"

#include "dynamics/HingeJoint.h"

#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

namespace
{

Math::Vec3 normalizedPerpendicular(const Math::Vec3& v)
{
    if (std::abs(v.x) > std::abs(v.y))
    {
        const f32 length = std::sqrt(v.x * v.x + v.z * v.z);
        return Math::Vec3(v.z, 0.0f, -v.x) / length;
    }
    const f32 length = std::sqrt(v.y * v.y + v.z * v.z);
    return Math::Vec3(0.0f, v.z, -v.y) / length;
}

f32 rotationAngleAroundAxis(const Math::Quaternion& q, const Math::Vec3& axis)
{
    if (q.w == 0.0f)
        return glm::pi<f32>();
    return 2.0f * std::atan(glm::dot(Math::Vec3(q.x, q.y, q.z), axis) / q.w);
}

f32 centerAngleAroundZero(f32 angle)
{
    while (angle < -glm::pi<f32>())
        angle += glm::two_pi<f32>();
    while (angle > glm::pi<f32>())
        angle -= glm::two_pi<f32>();
    return angle;
}

Math::Quaternion invInitialOrientationXZ(const Math::Vec3& xAxisA, const Math::Vec3& zAxisA,
                                  const Math::Vec3& xAxisB, const Math::Vec3& zAxisB)
{
    if (xAxisA == xAxisB && zAxisA == zAxisB)
        return Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    const Math::Mat3 basisA(xAxisA, glm::cross(zAxisA, xAxisA), zAxisA);
    const Math::Mat3 basisB(xAxisB, glm::cross(zAxisB, xAxisB), zAxisB);
    return glm::quat_cast(basisB) * glm::conjugate(glm::quat_cast(basisA));
}

}

HingeJoint::HingeJoint(RigidBody& a, RigidBody& b, const Math::Vec3& worldAnchor,
                       const Math::Vec3& worldHingeAxis)
    : HingeJoint(a, a.pointToLocal(worldAnchor), a.directionToLocal(glm::normalize(worldHingeAxis)),
                a.directionToLocal(normalizedPerpendicular(glm::normalize(worldHingeAxis))), b,
                b.pointToLocal(worldAnchor), b.directionToLocal(glm::normalize(worldHingeAxis)),
                b.directionToLocal(normalizedPerpendicular(glm::normalize(worldHingeAxis))))
{
}

HingeJoint::HingeJoint(RigidBody& a, const Math::Vec3& localAnchorA,
                       const Math::Vec3& localHingeAxisA, const Math::Vec3& localNormalAxisA,
                       RigidBody& b, const Math::Vec3& localAnchorB,
                       const Math::Vec3& localHingeAxisB, const Math::Vec3& localNormalAxisB)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB),
      mLocalHingeAxisA(glm::normalize(localHingeAxisA)),
      mLocalHingeAxisB(glm::normalize(localHingeAxisB)),
      mLocalNormalAxisA(glm::normalize(localNormalAxisA)),
      mLocalNormalAxisB(glm::normalize(localNormalAxisB)),
      mInverseInitialOrientation(invInitialOrientationXZ(mLocalNormalAxisA, mLocalHingeAxisA,
                                                          mLocalNormalAxisB, mLocalHingeAxisB))
{
}

RigidBody* HingeJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* HingeJoint::bodyB() const
{
    return mBodyB;
}

void HingeJoint::setLimits(f32 minAngle, f32 maxAngle)
{
    mLimitsMin = glm::clamp(minAngle, -glm::pi<f32>(), 0.0f);
    mLimitsMax = glm::clamp(maxAngle, 0.0f, glm::pi<f32>());
    mHasLimits = mLimitsMin > -glm::pi<f32>() || mLimitsMax < glm::pi<f32>();
}

f32 HingeJoint::currentAngle() const
{
    const Math::Quaternion diff = mBodyB->orientation() * mInverseInitialOrientation *
                           glm::conjugate(mBodyA->orientation());
    return rotationAngleAroundAxis(diff, mBodyA->directionToWorld(mLocalHingeAxisA));
}

void HingeJoint::setMotor(f32 targetAngularVelocity, f32 maxTorque)
{
    if (!std::isfinite(targetAngularVelocity) || !std::isfinite(maxTorque))
        return;
    mMotorTargetVelocity = targetAngularVelocity;
    mMotorMaxTorque = glm::max(maxTorque, 0.0f);
    mMotorEnabled = mMotorMaxTorque > 0.0f;
}

void HingeJoint::disableMotor()
{
    mMotorEnabled = false;
    mTotalMotorImpulse = 0.0f;
}

void HingeJoint::calculatePositionProperties()
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

void HingeJoint::calculateHingeRotationProperties()
{
    mA1 = glm::normalize(mBodyA->directionToWorld(mLocalHingeAxisA));
    Math::Vec3 a2 = glm::normalize(mBodyB->directionToWorld(mLocalHingeAxisB));

    const f32 dot = glm::dot(mA1, a2);
    if (dot <= 1.0e-3f)
    {
        Math::Vec3 perpendicular = a2 - dot * mA1;
        if (glm::dot(perpendicular, perpendicular) < 1.0e-6f)
            perpendicular = normalizedPerpendicular(mA1);
        else
            perpendicular = glm::normalize(perpendicular);
        a2 = glm::normalize(0.99f * perpendicular + 0.01f * mA1);
    }

    mB2 = normalizedPerpendicular(a2);
    mC2 = glm::cross(a2, mB2);
    mB2xA1 = glm::cross(mB2, mA1);
    mC2xA1 = glm::cross(mC2, mA1);

    const Math::Mat3 inverseInertiaSum =
        mBodyA->inverseInertiaTensorWorld() + mBodyB->inverseInertiaTensorWorld();
    glm::mat2 inverseEffectiveMass;
    inverseEffectiveMass[0][0] = glm::dot(mB2xA1, inverseInertiaSum * mB2xA1);
    inverseEffectiveMass[0][1] = glm::dot(mB2xA1, inverseInertiaSum * mC2xA1);
    inverseEffectiveMass[1][0] = glm::dot(mC2xA1, inverseInertiaSum * mB2xA1);
    inverseEffectiveMass[1][1] = glm::dot(mC2xA1, inverseInertiaSum * mC2xA1);

    const f32 determinant = glm::determinant(inverseEffectiveMass);
    if (std::abs(determinant) > 1.0e-9f && std::isfinite(determinant))
        mHingeRotationEffectiveMass = glm::inverse(inverseEffectiveMass);
    else
    {
        mHingeRotationEffectiveMass = glm::mat2(0.0f);
        mTotalHingeRotationImpulse = Math::Vec2(0.0f);
    }
}

void HingeJoint::calculateAxisAndAngle()
{
    mA1 = glm::normalize(mBodyA->directionToWorld(mLocalHingeAxisA));
    const Math::Quaternion diff = mBodyB->orientation() * mInverseInitialOrientation *
                           glm::conjugate(mBodyA->orientation());
    mTheta = rotationAngleAroundAxis(diff, mA1);
}

f32 HingeJoint::smallestAngleToLimit() const
{
    const f32 distanceToMin = centerAngleAroundZero(mTheta - mLimitsMin);
    const f32 distanceToMax = centerAngleAroundZero(mTheta - mLimitsMax);
    return std::abs(distanceToMin) < std::abs(distanceToMax) ? distanceToMin : distanceToMax;
}

bool HingeJoint::minLimitClosest() const
{
    const f32 distanceToMin = centerAngleAroundZero(mTheta - mLimitsMin);
    const f32 distanceToMax = centerAngleAroundZero(mTheta - mLimitsMax);
    return std::abs(distanceToMin) < std::abs(distanceToMax);
}

void HingeJoint::calculateLimitProperties(f32 duration)
{
    (void)duration;
    mLimitActive = mHasLimits && (mTheta <= mLimitsMin || mTheta >= mLimitsMax);
    if (!mLimitActive)
    {
        mLimitEffectiveMass = 0.0f;
        mTotalLimitImpulse = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = glm::dot(
        mA1, mBodyA->inverseInertiaTensorWorld() * mA1 + mBodyB->inverseInertiaTensorWorld() * mA1);
    mLimitEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mLimitEffectiveMass == 0.0f)
        mLimitActive = false;
}

void HingeJoint::calculateMotorProperties()
{
    if (!mMotorEnabled)
    {
        mMotorEffectiveMass = 0.0f;
        mTotalMotorImpulse = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = glm::dot(
        mA1, mBodyA->inverseInertiaTensorWorld() * mA1 + mBodyB->inverseInertiaTensorWorld() * mA1);
    mMotorEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
}

void HingeJoint::setup(f32 duration)
{
    calculatePositionProperties();
    calculateHingeRotationProperties();
    calculateAxisAndAngle();
    calculateLimitProperties(duration);
    calculateMotorProperties();
    mMotorMaxImpulse = mMotorMaxTorque * duration;
    if (mPreviousDuration > 0.0f)
    {
        const f32 ratio = duration / mPreviousDuration;
        mTotalPositionImpulse *= ratio;
        mTotalHingeRotationImpulse *= ratio;
        mTotalLimitImpulse *= ratio;
        mTotalMotorImpulse *= ratio;
    }
    else
    {
        mTotalPositionImpulse = Math::Vec3(0.0f);
        mTotalHingeRotationImpulse = Math::Vec2(0.0f);
        mTotalLimitImpulse = 0.0f;
        mTotalMotorImpulse = 0.0f;
    }
    mTotalMotorImpulse = glm::clamp(mTotalMotorImpulse, -mMotorMaxImpulse, mMotorMaxImpulse);
    mPreviousDuration = duration;
}

void HingeJoint::applyVelocityImpulse(const Math::Vec3& impulse)
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

void HingeJoint::applyAngularVelocityImpulse(const Math::Vec3& impulse)
{
    if (mBodyA->isDynamic())
        mBodyA->setAngularVelocity(mBodyA->angularVelocity() -
                                   mBodyA->inverseInertiaTensorWorld() * impulse);
    if (mBodyB->isDynamic())
        mBodyB->setAngularVelocity(mBodyB->angularVelocity() +
                                   mBodyB->inverseInertiaTensorWorld() * impulse);
}

void HingeJoint::warmStart()
{
    if (mMotorEnabled)
        applyAngularVelocityImpulse(mA1 * mTotalMotorImpulse);
    applyVelocityImpulse(mTotalPositionImpulse);
    applyAngularVelocityImpulse(mB2xA1 * mTotalHingeRotationImpulse.x +
                                mC2xA1 * mTotalHingeRotationImpulse.y);
    if (mLimitActive)
        applyAngularVelocityImpulse(mA1 * mTotalLimitImpulse);
}

void HingeJoint::solveVelocity()
{
    if (mMotorEnabled)
    {
        const f32 relativeVelocity =
            glm::dot(mA1, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = (relativeVelocity + mMotorTargetVelocity) * mMotorEffectiveMass;
        const f32 previous = mTotalMotorImpulse;
        mTotalMotorImpulse =
            glm::clamp(previous + impulse, -mMotorMaxImpulse, mMotorMaxImpulse);
        applyAngularVelocityImpulse(mA1 * (mTotalMotorImpulse - previous));
    }

    const Math::Vec3 relativeVelocity =
        mBodyB->velocity() + glm::cross(mBodyB->angularVelocity(), mArmB) -
        mBodyA->velocity() - glm::cross(mBodyA->angularVelocity(), mArmA);
    const Math::Vec3 positionImpulse = -(mPositionEffectiveMass * relativeVelocity);
    mTotalPositionImpulse += positionImpulse;
    applyVelocityImpulse(positionImpulse);

    const Math::Vec3 deltaAngular = mBodyA->angularVelocity() - mBodyB->angularVelocity();
    const Math::Vec2 jv(glm::dot(mB2xA1, deltaAngular), glm::dot(mC2xA1, deltaAngular));
    const Math::Vec2 hingeImpulse = mHingeRotationEffectiveMass * jv;
    mTotalHingeRotationImpulse += hingeImpulse;
    applyAngularVelocityImpulse(mB2xA1 * hingeImpulse.x + mC2xA1 * hingeImpulse.y);

    if (mLimitActive)
    {
        f32 minImpulse = -M_INFINITY;
        f32 maxImpulse = M_INFINITY;
        if (mLimitsMin != mLimitsMax)
        {
            if (minLimitClosest())
                minImpulse = 0.0f;
            else
                maxImpulse = 0.0f;
        }
        const f32 relative = glm::dot(mA1, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = mLimitEffectiveMass * relative;
        const f32 previous = mTotalLimitImpulse;
        mTotalLimitImpulse = glm::clamp(previous + impulse, minImpulse, maxImpulse);
        applyAngularVelocityImpulse(mA1 * (mTotalLimitImpulse - previous));
    }
}

void HingeJoint::solvePosition(f32 baumgarte)
{
    calculatePositionProperties();
    const Math::Vec3 pointA = mBodyA->position() + mArmA;
    const Math::Vec3 pointB = mBodyB->position() + mArmB;
    const Math::Vec3 positionImpulse = -(mPositionEffectiveMass * (pointB - pointA)) * baumgarte;
    mBodyA->applyPositionImpulseAtPoint(-positionImpulse, pointA);
    mBodyB->applyPositionImpulseAtPoint(positionImpulse, pointB);

    calculateHingeRotationProperties();
    const Math::Vec2 c(glm::dot(mA1, mB2), glm::dot(mA1, mC2));
    if (c != Math::Vec2(0.0f))
    {
        const Math::Vec2 lambda = -baumgarte * (mHingeRotationEffectiveMass * c);
        const Math::Vec3 impulse = mB2xA1 * lambda.x + mC2xA1 * lambda.y;
        if (mBodyA->isDynamic())
        {
            const Math::Vec3 step = mBodyA->inverseInertiaTensorWorld() * -impulse;
            const Math::Quaternion spin(0.0f, step);
            mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
        }
        if (mBodyB->isDynamic())
        {
            const Math::Vec3 step = mBodyB->inverseInertiaTensorWorld() * impulse;
            const Math::Quaternion spin(0.0f, step);
            mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
        }
    }

    if (mHasLimits)
    {
        calculateAxisAndAngle();
        calculateLimitProperties(0.0f);
        if (mLimitActive)
        {
            const f32 error = smallestAngleToLimit();
            const f32 lambda = -mLimitEffectiveMass * baumgarte * error;
            if (mBodyA->isDynamic())
            {
                const Math::Vec3 step = mBodyA->inverseInertiaTensorWorld() * mA1 * -lambda;
                const Math::Quaternion spin(0.0f, step);
                mBodyA->setOrientation(mBodyA->orientation() +
                                       0.5f * spin * mBodyA->orientation());
            }
            if (mBodyB->isDynamic())
            {
                const Math::Vec3 step = mBodyB->inverseInertiaTensorWorld() * mA1 * lambda;
                const Math::Quaternion spin(0.0f, step);
                mBodyB->setOrientation(mBodyB->orientation() +
                                       0.5f * spin * mBodyB->orientation());
            }
        }
    }
}

}
