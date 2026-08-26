#include "PCH.h"

#include "dynamics/PistonJoint.h"

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

PistonJoint::PistonJoint(RigidBody& a, RigidBody& b, const Math::Vec3& worldAnchor,
                         const Math::Vec3& worldAxis)
    : PistonJoint(a, a.pointToLocal(worldAnchor), a.directionToLocal(glm::normalize(worldAxis)),
                 a.directionToLocal(normalizedPerpendicular(glm::normalize(worldAxis))), b,
                 b.pointToLocal(worldAnchor), b.directionToLocal(glm::normalize(worldAxis)),
                 b.directionToLocal(normalizedPerpendicular(glm::normalize(worldAxis))))
{
}

PistonJoint::PistonJoint(RigidBody& a, const Math::Vec3& localAnchorA, const Math::Vec3& localAxisA,
                         const Math::Vec3& localNormalAxisA, RigidBody& b,
                         const Math::Vec3& localAnchorB, const Math::Vec3& localAxisB,
                         const Math::Vec3& localNormalAxisB)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB),
      mLocalAxisA(glm::normalize(localAxisA)), mLocalAxisB(glm::normalize(localAxisB)),
      mLocalNormalAxisA(glm::normalize(localNormalAxisA)),
      mLocalNormalAxisA2(glm::cross(mLocalAxisA, mLocalNormalAxisA)),
      mInverseInitialOrientation(invInitialOrientationXZ(mLocalNormalAxisA, mLocalAxisA,
                                                          glm::normalize(localNormalAxisB),
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
    mAngularLimitsMin = glm::clamp(minAngle, -glm::pi<f32>(), 0.0f);
    mAngularLimitsMax = glm::clamp(maxAngle, 0.0f, glm::pi<f32>());
    mHasAngularLimits = mAngularLimitsMin > -glm::pi<f32>() || mAngularLimitsMax < glm::pi<f32>();
}

f32 PistonJoint::currentPosition() const
{
    const Math::Vec3 armA = mBodyA->directionToWorld(mLocalAnchorA);
    const Math::Vec3 armB = mBodyB->directionToWorld(mLocalAnchorB);
    const Math::Vec3 offset = (mBodyB->position() - mBodyA->position()) + armB - armA;
    return glm::dot(offset, mBodyA->directionToWorld(mLocalAxisA));
}

f32 PistonJoint::currentAngle() const
{
    const Math::Quaternion diff = mBodyB->orientation() * mInverseInitialOrientation *
                           glm::conjugate(mBodyA->orientation());
    return rotationAngleAroundAxis(diff, mBodyA->directionToWorld(mLocalAxisA));
}

void PistonJoint::setLinearMotor(f32 targetVelocity, f32 maxForce)
{
    if (!std::isfinite(targetVelocity) || !std::isfinite(maxForce))
        return;
    mLinearMotorTargetVelocity = targetVelocity;
    mLinearMotorMaxForce = glm::max(maxForce, 0.0f);
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
    mAngularMotorMaxTorque = glm::max(maxTorque, 0.0f);
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

    const Math::Vec3 armAPlusOffset = mArmA + mOffset;
    const Math::Vec3 r1x1 = glm::cross(armAPlusOffset, mN1);
    const Math::Vec3 r1x2 = glm::cross(armAPlusOffset, mN2);
    const Math::Vec3 r2x1 = glm::cross(mArmB, mN1);
    const Math::Vec3 r2x2 = glm::cross(mArmB, mN2);

    const f32 inverseMassSum = mBodyA->inverseMass() + mBodyB->inverseMass();
    glm::mat2 inverseEffectiveMass(0.0f);
    inverseEffectiveMass[0][0] = inverseMassSum + glm::dot(r1x1, mBodyA->inverseInertiaTensorWorld() * r1x1) +
                                 glm::dot(r2x1, mBodyB->inverseInertiaTensorWorld() * r2x1);
    inverseEffectiveMass[0][1] = glm::dot(r1x1, mBodyA->inverseInertiaTensorWorld() * r1x2) +
                                 glm::dot(r2x1, mBodyB->inverseInertiaTensorWorld() * r2x2);
    inverseEffectiveMass[1][0] = glm::dot(r1x2, mBodyA->inverseInertiaTensorWorld() * r1x1) +
                                 glm::dot(r2x2, mBodyB->inverseInertiaTensorWorld() * r2x1);
    inverseEffectiveMass[1][1] = inverseMassSum + glm::dot(r1x2, mBodyA->inverseInertiaTensorWorld() * r1x2) +
                                 glm::dot(r2x2, mBodyB->inverseInertiaTensorWorld() * r2x2);

    const f32 determinant = glm::determinant(inverseEffectiveMass);
    if (std::abs(determinant) > 1.0e-9f && std::isfinite(determinant))
        mPositionLockEffectiveMass = glm::inverse(inverseEffectiveMass);
    else
    {
        mPositionLockEffectiveMass = glm::mat2(0.0f);
        mTotalPositionLockImpulse = Math::Vec2(0.0f);
    }
}

void PistonJoint::calculateRotationLockProperties()
{
    mA1 = glm::normalize(mBodyA->directionToWorld(mLocalAxisA));
    Math::Vec3 a2 = glm::normalize(mBodyB->directionToWorld(mLocalAxisB));

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
        mRotationLockEffectiveMass = glm::inverse(inverseEffectiveMass);
    else
    {
        mRotationLockEffectiveMass = glm::mat2(0.0f);
        mTotalRotationLockImpulse = Math::Vec2(0.0f);
    }
}

void PistonJoint::calculateAxisAndPosition()
{
    mSlidePosition = glm::dot(mOffset, mA1);
    const Math::Quaternion diff = mBodyB->orientation() * mInverseInitialOrientation *
                           glm::conjugate(mBodyA->orientation());
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
    const Math::Vec3 armAPlusOffset = mArmA + mOffset;
    f32 inverseEffectiveMass = mBodyA->inverseMass() + mBodyB->inverseMass();
    inverseEffectiveMass +=
        glm::dot(mA1, mBodyA->inverseInertiaTensorWorld() * glm::cross(armAPlusOffset, mA1));
    inverseEffectiveMass +=
        glm::dot(mA1, mBodyB->inverseInertiaTensorWorld() * glm::cross(mArmB, mA1));
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
    const f32 inverseEffectiveMass = glm::dot(
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
    const Math::Vec3 armAPlusOffset = mArmA + mOffset;
    f32 inverseEffectiveMass = mBodyA->inverseMass() + mBodyB->inverseMass();
    inverseEffectiveMass +=
        glm::dot(mA1, mBodyA->inverseInertiaTensorWorld() * glm::cross(armAPlusOffset, mA1));
    inverseEffectiveMass +=
        glm::dot(mA1, mBodyB->inverseInertiaTensorWorld() * glm::cross(mArmB, mA1));
    mLinearMotorEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
}

void PistonJoint::calculateAngularMotorProperties()
{
    if (!mAngularMotorEnabled)
    {
        mAngularMotorEffectiveMass = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = glm::dot(
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
        mTotalPositionLockImpulse = Math::Vec2(0.0f);
        mTotalRotationLockImpulse = Math::Vec2(0.0f);
        mTotalLinearLimitImpulse = 0.0f;
        mTotalAngularLimitImpulse = 0.0f;
        mTotalLinearMotorImpulse = 0.0f;
        mTotalAngularMotorImpulse = 0.0f;
    }
    mTotalLinearMotorImpulse =
        glm::clamp(mTotalLinearMotorImpulse, -mLinearMotorMaxImpulse, mLinearMotorMaxImpulse);
    mTotalAngularMotorImpulse =
        glm::clamp(mTotalAngularMotorImpulse, -mAngularMotorMaxImpulse, mAngularMotorMaxImpulse);
    mPreviousDuration = duration;
}

void PistonJoint::applyLinearImpulse(const Math::Vec3& impulse)
{
    const Math::Vec3 armAPlusOffset = mArmA + mOffset;
    if (mBodyA->isDynamic())
    {
        mBodyA->setVelocity(mBodyA->velocity() - impulse * mBodyA->inverseMass());
        mBodyA->setAngularVelocity(
            mBodyA->angularVelocity() -
            mBodyA->inverseInertiaTensorWorld() * glm::cross(armAPlusOffset, impulse));
    }
    if (mBodyB->isDynamic())
    {
        mBodyB->setVelocity(mBodyB->velocity() + impulse * mBodyB->inverseMass());
        mBodyB->setAngularVelocity(
            mBodyB->angularVelocity() +
            mBodyB->inverseInertiaTensorWorld() * glm::cross(mArmB, impulse));
    }
}

void PistonJoint::applyAngularImpulse(const Math::Vec3& impulse)
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
    const Math::Vec3 armAPlusOffset = mArmA + mOffset;

    if (mLinearMotorEnabled)
    {
        const f32 jv = glm::dot(mA1, mBodyA->velocity() - mBodyB->velocity()) +
                      glm::dot(glm::cross(armAPlusOffset, mA1), mBodyA->angularVelocity()) -
                      glm::dot(glm::cross(mArmB, mA1), mBodyB->angularVelocity());
        const f32 impulse = (jv + mLinearMotorTargetVelocity) * mLinearMotorEffectiveMass;
        const f32 previous = mTotalLinearMotorImpulse;
        mTotalLinearMotorImpulse =
            glm::clamp(previous + impulse, -mLinearMotorMaxImpulse, mLinearMotorMaxImpulse);
        applyLinearImpulse(mA1 * (mTotalLinearMotorImpulse - previous));
    }

    if (mAngularMotorEnabled)
    {
        const f32 relativeVelocity =
            glm::dot(mA1, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = (relativeVelocity + mAngularMotorTargetVelocity) * mAngularMotorEffectiveMass;
        const f32 previous = mTotalAngularMotorImpulse;
        mTotalAngularMotorImpulse =
            glm::clamp(previous + impulse, -mAngularMotorMaxImpulse, mAngularMotorMaxImpulse);
        applyAngularImpulse(mA1 * (mTotalAngularMotorImpulse - previous));
    }

    const Math::Vec3 deltaLinear = mBodyA->velocity() - mBodyB->velocity();
    Math::Vec2 positionJv;
    positionJv.x = glm::dot(mN1, deltaLinear) +
                  glm::dot(glm::cross(armAPlusOffset, mN1), mBodyA->angularVelocity()) -
                  glm::dot(glm::cross(mArmB, mN1), mBodyB->angularVelocity());
    positionJv.y = glm::dot(mN2, deltaLinear) +
                  glm::dot(glm::cross(armAPlusOffset, mN2), mBodyA->angularVelocity()) -
                  glm::dot(glm::cross(mArmB, mN2), mBodyB->angularVelocity());
    const Math::Vec2 positionImpulse = mPositionLockEffectiveMass * positionJv;
    mTotalPositionLockImpulse += positionImpulse;
    applyLinearImpulse(mN1 * positionImpulse.x + mN2 * positionImpulse.y);

    const Math::Vec3 deltaAngular = mBodyA->angularVelocity() - mBodyB->angularVelocity();
    const Math::Vec2 rotationJv(glm::dot(mB2xA1, deltaAngular), glm::dot(mC2xA1, deltaAngular));
    const Math::Vec2 rotationImpulse = mRotationLockEffectiveMass * rotationJv;
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
        const f32 relative = glm::dot(mA1, mBodyA->velocity() - mBodyB->velocity()) +
                             glm::dot(glm::cross(armAPlusOffset, mA1), mBodyA->angularVelocity()) -
                             glm::dot(glm::cross(mArmB, mA1), mBodyB->angularVelocity());
        const f32 impulse = mLinearLimitEffectiveMass * relative;
        const f32 previous = mTotalLinearLimitImpulse;
        mTotalLinearLimitImpulse = glm::clamp(previous + impulse, minImpulse, maxImpulse);
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
        const f32 relative = glm::dot(mA1, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = mAngularLimitEffectiveMass * relative;
        const f32 previous = mTotalAngularLimitImpulse;
        mTotalAngularLimitImpulse = glm::clamp(previous + impulse, minImpulse, maxImpulse);
        applyAngularImpulse(mA1 * (mTotalAngularLimitImpulse - previous));
    }
}

void PistonJoint::solvePosition(f32 baumgarte)
{
    calculateArmsAndOffset();
    calculatePositionLockProperties();
    const Math::Vec2 c(glm::dot(mOffset, mN1), glm::dot(mOffset, mN2));
    if (c != Math::Vec2(0.0f))
    {
        const Math::Vec2 lambda = -baumgarte * (mPositionLockEffectiveMass * c);
        const Math::Vec3 impulse = mN1 * lambda.x + mN2 * lambda.y;
        const Math::Vec3 armAPlusOffset = mArmA + mOffset;
        mBodyA->applyPositionImpulseAtPoint(-impulse, mBodyA->position() + armAPlusOffset);
        mBodyB->applyPositionImpulseAtPoint(impulse, mBodyB->position() + mArmB);
    }

    calculateRotationLockProperties();
    const Math::Vec2 rc(glm::dot(mA1, mB2), glm::dot(mA1, mC2));
    if (rc != Math::Vec2(0.0f))
    {
        const Math::Vec2 lambda = -baumgarte * (mRotationLockEffectiveMass * rc);
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
            const Math::Vec3 armAPlusOffset = mArmA + mOffset;
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
                const Math::Vec3 step = mBodyA->inverseInertiaTensorWorld() * mA1 * -lambda;
                const Math::Quaternion spin(0.0f, step);
                mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
            }
            if (mBodyB->isDynamic())
            {
                const Math::Vec3 step = mBodyB->inverseInertiaTensorWorld() * mA1 * lambda;
                const Math::Quaternion spin(0.0f, step);
                mBodyB->setOrientation(mBodyB->orientation() + 0.5f * spin * mBodyB->orientation());
            }
        }
    }
}

}
