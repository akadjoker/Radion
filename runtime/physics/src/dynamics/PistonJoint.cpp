#include "PCH.h"

#include "dynamics/PistonJoint.h"

#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

namespace
{

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

Math::quat invInitialOrientationXZ(const Math::vec3& xAxisA, const Math::vec3& zAxisA,
                                  const Math::vec3& xAxisB, const Math::vec3& zAxisB)
{
    if (xAxisA == xAxisB && zAxisA == zAxisB)
        return Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const Math::mat3 basisA(xAxisA, Math::cross(zAxisA, xAxisA), zAxisA);
    const Math::mat3 basisB(xAxisB, Math::cross(zAxisB, xAxisB), zAxisB);
    return Math::quat_cast(basisB) * Math::conjugate(Math::quat_cast(basisA));
}

}

PistonJoint::PistonJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                         const Math::vec3& worldAxis)
    : PistonJoint(a, a.pointToLocal(worldAnchor), a.directionToLocal(Math::normalize(worldAxis)),
                 a.directionToLocal(normalizedPerpendicular(Math::normalize(worldAxis))), b,
                 b.pointToLocal(worldAnchor), b.directionToLocal(Math::normalize(worldAxis)),
                 b.directionToLocal(normalizedPerpendicular(Math::normalize(worldAxis))))
{
}

PistonJoint::PistonJoint(RigidBody& a, const Math::vec3& localAnchorA, const Math::vec3& localAxisA,
                         const Math::vec3& localNormalAxisA, RigidBody& b,
                         const Math::vec3& localAnchorB, const Math::vec3& localAxisB,
                         const Math::vec3& localNormalAxisB)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB),
      mLocalAxisA(Math::normalize(localAxisA)), mLocalAxisB(Math::normalize(localAxisB)),
      mLocalNormalAxisA(Math::normalize(localNormalAxisA)),
      mLocalNormalAxisA2(Math::cross(mLocalAxisA, mLocalNormalAxisA)),
      mInverseInitialOrientation(invInitialOrientationXZ(mLocalNormalAxisA, mLocalAxisA,
                                                          Math::normalize(localNormalAxisB),
                                                          mLocalAxisB))
{
}

RigidBody* PistonJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* PistonJoint::bodyB() const
{
    return mBodyB;
}

void PistonJoint::setLinearLimits(f32 minDistance, f32 maxDistance)
{
    mLinearLimitsMin = minDistance;
    mLinearLimitsMax = maxDistance;
    mHasLinearLimits = mLinearLimitsMin != -M_INFINITY || mLinearLimitsMax != M_INFINITY;
}

void PistonJoint::setAngularLimits(f32 minAngle, f32 maxAngle)
{
    mAngularLimitsMin = Math::clamp(minAngle, -Math::pi<f32>(), 0.0f);
    mAngularLimitsMax = Math::clamp(maxAngle, 0.0f, Math::pi<f32>());
    mHasAngularLimits = mAngularLimitsMin > -Math::pi<f32>() || mAngularLimitsMax < Math::pi<f32>();
}

f32 PistonJoint::currentPosition() const
{
    const Math::vec3 armA = mBodyA->directionToWorld(mLocalAnchorA);
    const Math::vec3 armB = mBodyB->directionToWorld(mLocalAnchorB);
    const Math::vec3 offset = (mBodyB->position() - mBodyA->position()) + armB - armA;
    return Math::dot(offset, mBodyA->directionToWorld(mLocalAxisA));
}

f32 PistonJoint::currentAngle() const
{
    const Math::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mBodyA->orientation());
    return rotationAngleAroundAxis(diff, mBodyA->directionToWorld(mLocalAxisA));
}

void PistonJoint::setLinearMotor(f32 targetVelocity, f32 maxForce)
{
    if (!std::isfinite(targetVelocity) || !std::isfinite(maxForce))
        return;
    mLinearMotorTargetVelocity = targetVelocity;
    mLinearMotorMaxForce = Math::max(maxForce, 0.0f);
    mLinearMotorEnabled = mLinearMotorMaxForce > 0.0f;
}

void PistonJoint::disableLinearMotor()
{
    mLinearMotorEnabled = false;
    mTotalLinearMotorImpulse = 0.0f;
}

void PistonJoint::setAngularMotor(f32 targetAngularVelocity, f32 maxTorque)
{
    if (!std::isfinite(targetAngularVelocity) || !std::isfinite(maxTorque))
        return;
    mAngularMotorTargetVelocity = targetAngularVelocity;
    mAngularMotorMaxTorque = Math::max(maxTorque, 0.0f);
    mAngularMotorEnabled = mAngularMotorMaxTorque > 0.0f;
}

void PistonJoint::disableAngularMotor()
{
    mAngularMotorEnabled = false;
    mTotalAngularMotorImpulse = 0.0f;
}

void PistonJoint::calculateArmsAndOffset()
{
    mArmA = mBodyA->directionToWorld(mLocalAnchorA);
    mArmB = mBodyB->directionToWorld(mLocalAnchorB);
    mOffset = (mBodyB->position() - mBodyA->position()) + mArmB - mArmA;
}

void PistonJoint::calculatePositionLockProperties()
{
    mN1 = mBodyA->directionToWorld(mLocalNormalAxisA);
    mN2 = mBodyA->directionToWorld(mLocalNormalAxisA2);

    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    const Math::vec3 r1x1 = Math::cross(armAPlusOffset, mN1);
    const Math::vec3 r1x2 = Math::cross(armAPlusOffset, mN2);
    const Math::vec3 r2x1 = Math::cross(mArmB, mN1);
    const Math::vec3 r2x2 = Math::cross(mArmB, mN2);

    const f32 inverseMassSum = mBodyA->inverseMass() + mBodyB->inverseMass();
    Math::mat2 inverseEffectiveMass(0.0f);
    inverseEffectiveMass[0][0] = inverseMassSum + Math::dot(r1x1, mBodyA->inverseInertiaTensorWorld() * r1x1) +
                                 Math::dot(r2x1, mBodyB->inverseInertiaTensorWorld() * r2x1);
    inverseEffectiveMass[0][1] = Math::dot(r1x1, mBodyA->inverseInertiaTensorWorld() * r1x2) +
                                 Math::dot(r2x1, mBodyB->inverseInertiaTensorWorld() * r2x2);
    inverseEffectiveMass[1][0] = Math::dot(r1x2, mBodyA->inverseInertiaTensorWorld() * r1x1) +
                                 Math::dot(r2x2, mBodyB->inverseInertiaTensorWorld() * r2x1);
    inverseEffectiveMass[1][1] = inverseMassSum + Math::dot(r1x2, mBodyA->inverseInertiaTensorWorld() * r1x2) +
                                 Math::dot(r2x2, mBodyB->inverseInertiaTensorWorld() * r2x2);

    const f32 determinant = Math::determinant(inverseEffectiveMass);
    if (std::abs(determinant) > 1.0e-9f && std::isfinite(determinant))
        mPositionLockEffectiveMass = Math::inverse(inverseEffectiveMass);
    else
    {
        mPositionLockEffectiveMass = Math::mat2(0.0f);
        mTotalPositionLockImpulse = Math::vec2(0.0f);
    }
}

void PistonJoint::calculateRotationLockProperties()
{
    mA1 = Math::normalize(mBodyA->directionToWorld(mLocalAxisA));
    Math::vec3 a2 = Math::normalize(mBodyB->directionToWorld(mLocalAxisB));

    const f32 dot = Math::dot(mA1, a2);
    if (dot <= 1.0e-3f)
    {
        Math::vec3 perpendicular = a2 - dot * mA1;
        if (Math::dot(perpendicular, perpendicular) < 1.0e-6f)
            perpendicular = normalizedPerpendicular(mA1);
        else
            perpendicular = Math::normalize(perpendicular);
        a2 = Math::normalize(0.99f * perpendicular + 0.01f * mA1);
    }

    mB2 = normalizedPerpendicular(a2);
    mC2 = Math::cross(a2, mB2);
    mB2xA1 = Math::cross(mB2, mA1);
    mC2xA1 = Math::cross(mC2, mA1);

    const Math::mat3 inverseInertiaSum =
        mBodyA->inverseInertiaTensorWorld() + mBodyB->inverseInertiaTensorWorld();
    Math::mat2 inverseEffectiveMass;
    inverseEffectiveMass[0][0] = Math::dot(mB2xA1, inverseInertiaSum * mB2xA1);
    inverseEffectiveMass[0][1] = Math::dot(mB2xA1, inverseInertiaSum * mC2xA1);
    inverseEffectiveMass[1][0] = Math::dot(mC2xA1, inverseInertiaSum * mB2xA1);
    inverseEffectiveMass[1][1] = Math::dot(mC2xA1, inverseInertiaSum * mC2xA1);

    const f32 determinant = Math::determinant(inverseEffectiveMass);
    if (std::abs(determinant) > 1.0e-9f && std::isfinite(determinant))
        mRotationLockEffectiveMass = Math::inverse(inverseEffectiveMass);
    else
    {
        mRotationLockEffectiveMass = Math::mat2(0.0f);
        mTotalRotationLockImpulse = Math::vec2(0.0f);
    }
}

void PistonJoint::calculateAxisAndPosition()
{
    mSlidePosition = Math::dot(mOffset, mA1);
    const Math::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mBodyA->orientation());
    mTheta = rotationAngleAroundAxis(diff, mA1);
}

void PistonJoint::calculateLinearLimitProperties()
{
    mLinearLimitActive =
        mHasLinearLimits && (mSlidePosition <= mLinearLimitsMin || mSlidePosition >= mLinearLimitsMax);
    if (!mLinearLimitActive)
    {
        mLinearLimitEffectiveMass = 0.0f;
        mTotalLinearLimitImpulse = 0.0f;
        return;
    }
    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    f32 inverseEffectiveMass = mBodyA->inverseMass() + mBodyB->inverseMass();
    inverseEffectiveMass +=
        Math::dot(mA1, mBodyA->inverseInertiaTensorWorld() * Math::cross(armAPlusOffset, mA1));
    inverseEffectiveMass +=
        Math::dot(mA1, mBodyB->inverseInertiaTensorWorld() * Math::cross(mArmB, mA1));
    mLinearLimitEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mLinearLimitEffectiveMass == 0.0f)
        mLinearLimitActive = false;
}

void PistonJoint::calculateAngularLimitProperties(f32 duration)
{
    (void)duration;
    mAngularLimitActive =
        mHasAngularLimits && (mTheta <= mAngularLimitsMin || mTheta >= mAngularLimitsMax);
    if (!mAngularLimitActive)
    {
        mAngularLimitEffectiveMass = 0.0f;
        mTotalAngularLimitImpulse = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = Math::dot(
        mA1, mBodyA->inverseInertiaTensorWorld() * mA1 + mBodyB->inverseInertiaTensorWorld() * mA1);
    mAngularLimitEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mAngularLimitEffectiveMass == 0.0f)
        mAngularLimitActive = false;
}

void PistonJoint::calculateLinearMotorProperties()
{
    if (!mLinearMotorEnabled)
    {
        mLinearMotorEffectiveMass = 0.0f;
        return;
    }
    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    f32 inverseEffectiveMass = mBodyA->inverseMass() + mBodyB->inverseMass();
    inverseEffectiveMass +=
        Math::dot(mA1, mBodyA->inverseInertiaTensorWorld() * Math::cross(armAPlusOffset, mA1));
    inverseEffectiveMass +=
        Math::dot(mA1, mBodyB->inverseInertiaTensorWorld() * Math::cross(mArmB, mA1));
    mLinearMotorEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
}

void PistonJoint::calculateAngularMotorProperties()
{
    if (!mAngularMotorEnabled)
    {
        mAngularMotorEffectiveMass = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = Math::dot(
        mA1, mBodyA->inverseInertiaTensorWorld() * mA1 + mBodyB->inverseInertiaTensorWorld() * mA1);
    mAngularMotorEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
}

void PistonJoint::setup(f32 duration)
{
    calculateArmsAndOffset();
    calculatePositionLockProperties();
    calculateRotationLockProperties();
    calculateAxisAndPosition();
    calculateLinearLimitProperties();
    calculateAngularLimitProperties(duration);
    calculateLinearMotorProperties();
    calculateAngularMotorProperties();
    mLinearMotorMaxImpulse = mLinearMotorMaxForce * duration;
    mAngularMotorMaxImpulse = mAngularMotorMaxTorque * duration;
    if (mPreviousDuration > 0.0f)
    {
        const f32 ratio = duration / mPreviousDuration;
        mTotalPositionLockImpulse *= ratio;
        mTotalRotationLockImpulse *= ratio;
        mTotalLinearLimitImpulse *= ratio;
        mTotalAngularLimitImpulse *= ratio;
        mTotalLinearMotorImpulse *= ratio;
        mTotalAngularMotorImpulse *= ratio;
    }
    else
    {
        mTotalPositionLockImpulse = Math::vec2(0.0f);
        mTotalRotationLockImpulse = Math::vec2(0.0f);
        mTotalLinearLimitImpulse = 0.0f;
        mTotalAngularLimitImpulse = 0.0f;
        mTotalLinearMotorImpulse = 0.0f;
        mTotalAngularMotorImpulse = 0.0f;
    }
    mTotalLinearMotorImpulse =
        Math::clamp(mTotalLinearMotorImpulse, -mLinearMotorMaxImpulse, mLinearMotorMaxImpulse);
    mTotalAngularMotorImpulse =
        Math::clamp(mTotalAngularMotorImpulse, -mAngularMotorMaxImpulse, mAngularMotorMaxImpulse);
    mPreviousDuration = duration;
}

void PistonJoint::applyLinearImpulse(const Math::vec3& impulse)
{
    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    if (mBodyA->isDynamic())
    {
        mBodyA->setVelocity(mBodyA->velocity() - impulse * mBodyA->inverseMass());
        mBodyA->setAngularVelocity(
            mBodyA->angularVelocity() -
            mBodyA->inverseInertiaTensorWorld() * Math::cross(armAPlusOffset, impulse));
    }
    if (mBodyB->isDynamic())
    {
        mBodyB->setVelocity(mBodyB->velocity() + impulse * mBodyB->inverseMass());
        mBodyB->setAngularVelocity(
            mBodyB->angularVelocity() +
            mBodyB->inverseInertiaTensorWorld() * Math::cross(mArmB, impulse));
    }
}

void PistonJoint::applyAngularImpulse(const Math::vec3& impulse)
{
    if (mBodyA->isDynamic())
        mBodyA->setAngularVelocity(mBodyA->angularVelocity() -
                                   mBodyA->inverseInertiaTensorWorld() * impulse);
    if (mBodyB->isDynamic())
        mBodyB->setAngularVelocity(mBodyB->angularVelocity() +
                                   mBodyB->inverseInertiaTensorWorld() * impulse);
}

void PistonJoint::warmStart()
{
    if (mLinearMotorEnabled)
        applyLinearImpulse(mA1 * mTotalLinearMotorImpulse);
    if (mAngularMotorEnabled)
        applyAngularImpulse(mA1 * mTotalAngularMotorImpulse);
    applyLinearImpulse(mN1 * mTotalPositionLockImpulse.x + mN2 * mTotalPositionLockImpulse.y);
    applyAngularImpulse(mB2xA1 * mTotalRotationLockImpulse.x + mC2xA1 * mTotalRotationLockImpulse.y);
    if (mLinearLimitActive)
        applyLinearImpulse(mA1 * mTotalLinearLimitImpulse);
    if (mAngularLimitActive)
        applyAngularImpulse(mA1 * mTotalAngularLimitImpulse);
}

void PistonJoint::solveVelocity()
{
    const Math::vec3 armAPlusOffset = mArmA + mOffset;

    if (mLinearMotorEnabled)
    {
        const f32 jv = Math::dot(mA1, mBodyA->velocity() - mBodyB->velocity()) +
                      Math::dot(Math::cross(armAPlusOffset, mA1), mBodyA->angularVelocity()) -
                      Math::dot(Math::cross(mArmB, mA1), mBodyB->angularVelocity());
        const f32 impulse = (jv + mLinearMotorTargetVelocity) * mLinearMotorEffectiveMass;
        const f32 previous = mTotalLinearMotorImpulse;
        mTotalLinearMotorImpulse =
            Math::clamp(previous + impulse, -mLinearMotorMaxImpulse, mLinearMotorMaxImpulse);
        applyLinearImpulse(mA1 * (mTotalLinearMotorImpulse - previous));
    }

    if (mAngularMotorEnabled)
    {
        const f32 relativeVelocity =
            Math::dot(mA1, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = (relativeVelocity + mAngularMotorTargetVelocity) * mAngularMotorEffectiveMass;
        const f32 previous = mTotalAngularMotorImpulse;
        mTotalAngularMotorImpulse =
            Math::clamp(previous + impulse, -mAngularMotorMaxImpulse, mAngularMotorMaxImpulse);
        applyAngularImpulse(mA1 * (mTotalAngularMotorImpulse - previous));
    }

    const Math::vec3 deltaLinear = mBodyA->velocity() - mBodyB->velocity();
    Math::vec2 positionJv;
    positionJv.x = Math::dot(mN1, deltaLinear) +
                  Math::dot(Math::cross(armAPlusOffset, mN1), mBodyA->angularVelocity()) -
                  Math::dot(Math::cross(mArmB, mN1), mBodyB->angularVelocity());
    positionJv.y = Math::dot(mN2, deltaLinear) +
                  Math::dot(Math::cross(armAPlusOffset, mN2), mBodyA->angularVelocity()) -
                  Math::dot(Math::cross(mArmB, mN2), mBodyB->angularVelocity());
    const Math::vec2 positionImpulse = mPositionLockEffectiveMass * positionJv;
    mTotalPositionLockImpulse += positionImpulse;
    applyLinearImpulse(mN1 * positionImpulse.x + mN2 * positionImpulse.y);

    const Math::vec3 deltaAngular = mBodyA->angularVelocity() - mBodyB->angularVelocity();
    const Math::vec2 rotationJv(Math::dot(mB2xA1, deltaAngular), Math::dot(mC2xA1, deltaAngular));
    const Math::vec2 rotationImpulse = mRotationLockEffectiveMass * rotationJv;
    mTotalRotationLockImpulse += rotationImpulse;
    applyAngularImpulse(mB2xA1 * rotationImpulse.x + mC2xA1 * rotationImpulse.y);

    if (mLinearLimitActive)
    {
        f32 minImpulse = -M_INFINITY;
        f32 maxImpulse = M_INFINITY;
        if (mLinearLimitsMin != mLinearLimitsMax)
        {
            if (mSlidePosition <= mLinearLimitsMin)
                minImpulse = 0.0f;
            else
                maxImpulse = 0.0f;
        }
        const f32 relative = Math::dot(mA1, mBodyA->velocity() - mBodyB->velocity()) +
                             Math::dot(Math::cross(armAPlusOffset, mA1), mBodyA->angularVelocity()) -
                             Math::dot(Math::cross(mArmB, mA1), mBodyB->angularVelocity());
        const f32 impulse = mLinearLimitEffectiveMass * relative;
        const f32 previous = mTotalLinearLimitImpulse;
        mTotalLinearLimitImpulse = Math::clamp(previous + impulse, minImpulse, maxImpulse);
        applyLinearImpulse(mA1 * (mTotalLinearLimitImpulse - previous));
    }

    if (mAngularLimitActive)
    {
        f32 minImpulse = -M_INFINITY;
        f32 maxImpulse = M_INFINITY;
        if (mAngularLimitsMin != mAngularLimitsMax)
        {
            const f32 distanceToMin = centerAngleAroundZero(mTheta - mAngularLimitsMin);
            const f32 distanceToMax = centerAngleAroundZero(mTheta - mAngularLimitsMax);
            if (std::abs(distanceToMin) < std::abs(distanceToMax))
                minImpulse = 0.0f;
            else
                maxImpulse = 0.0f;
        }
        const f32 relative = Math::dot(mA1, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = mAngularLimitEffectiveMass * relative;
        const f32 previous = mTotalAngularLimitImpulse;
        mTotalAngularLimitImpulse = Math::clamp(previous + impulse, minImpulse, maxImpulse);
        applyAngularImpulse(mA1 * (mTotalAngularLimitImpulse - previous));
    }
}

void PistonJoint::solvePosition(f32 baumgarte)
{
    calculateArmsAndOffset();
    calculatePositionLockProperties();
    const Math::vec2 c(Math::dot(mOffset, mN1), Math::dot(mOffset, mN2));
    if (c != Math::vec2(0.0f))
    {
        const Math::vec2 lambda = -baumgarte * (mPositionLockEffectiveMass * c);
        const Math::vec3 impulse = mN1 * lambda.x + mN2 * lambda.y;
        const Math::vec3 armAPlusOffset = mArmA + mOffset;
        mBodyA->applyPositionImpulseAtPoint(-impulse, mBodyA->position() + armAPlusOffset);
        mBodyB->applyPositionImpulseAtPoint(impulse, mBodyB->position() + mArmB);
    }

    calculateRotationLockProperties();
    const Math::vec2 rc(Math::dot(mA1, mB2), Math::dot(mA1, mC2));
    if (rc != Math::vec2(0.0f))
    {
        const Math::vec2 lambda = -baumgarte * (mRotationLockEffectiveMass * rc);
        const Math::vec3 impulse = mB2xA1 * lambda.x + mC2xA1 * lambda.y;
        if (mBodyA->isDynamic())
        {
            const Math::vec3 step = mBodyA->inverseInertiaTensorWorld() * -impulse;
            const Math::quat spin(0.0f, step);
            mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
        }
        if (mBodyB->isDynamic())
        {
            const Math::vec3 step = mBodyB->inverseInertiaTensorWorld() * impulse;
            const Math::quat spin(0.0f, step);
            mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
        }
    }

    if (mHasLinearLimits)
    {
        calculateArmsAndOffset();
        calculateAxisAndPosition();
        calculateLinearLimitProperties();
        if (mLinearLimitActive)
        {
            f32 error = mSlidePosition <= mLinearLimitsMin ? mSlidePosition - mLinearLimitsMin
                                                            : mSlidePosition - mLinearLimitsMax;
            const f32 lambda = -mLinearLimitEffectiveMass * baumgarte * error;
            const Math::vec3 armAPlusOffset = mArmA + mOffset;
            mBodyA->applyPositionImpulseAtPoint(-(lambda * mA1), mBodyA->position() + armAPlusOffset);
            mBodyB->applyPositionImpulseAtPoint(lambda * mA1, mBodyB->position() + mArmB);
        }
    }

    if (mHasAngularLimits)
    {
        calculateAxisAndPosition();
        calculateAngularLimitProperties(0.0f);
        if (mAngularLimitActive)
        {
            const f32 distanceToMin = centerAngleAroundZero(mTheta - mAngularLimitsMin);
            const f32 distanceToMax = centerAngleAroundZero(mTheta - mAngularLimitsMax);
            const f32 error = std::abs(distanceToMin) < std::abs(distanceToMax) ? distanceToMin
                                                                                 : distanceToMax;
            const f32 lambda = -mAngularLimitEffectiveMass * baumgarte * error;
            if (mBodyA->isDynamic())
            {
                const Math::vec3 step = mBodyA->inverseInertiaTensorWorld() * mA1 * -lambda;
                const Math::quat spin(0.0f, step);
                mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
            }
            if (mBodyB->isDynamic())
            {
                const Math::vec3 step = mBodyB->inverseInertiaTensorWorld() * mA1 * lambda;
                const Math::quat spin(0.0f, step);
                mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
            }
        }
    }
}

}
