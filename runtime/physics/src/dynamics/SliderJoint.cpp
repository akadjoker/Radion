#include "PCH.h"

#include "dynamics/SliderJoint.h"

#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

namespace
{

glm::quat invInitialOrientationXY(const glm::vec3& xAxisA, const glm::vec3& yAxisA,
                                  const glm::vec3& xAxisB, const glm::vec3& yAxisB)
{
    if (xAxisA == xAxisB && yAxisA == yAxisB)
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const glm::mat3 basisA(xAxisA, yAxisA, glm::cross(xAxisA, yAxisA));
    const glm::mat3 basisB(xAxisB, yAxisB, glm::cross(xAxisB, yAxisB));
    return glm::quat_cast(basisB) * glm::conjugate(glm::quat_cast(basisA));
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

SliderJoint::SliderJoint(RigidBody& a, RigidBody& b, const glm::vec3& worldAnchor,
                         const glm::vec3& worldSliderAxis)
    : SliderJoint(a, a.pointToLocal(worldAnchor),
                 a.directionToLocal(glm::normalize(worldSliderAxis)),
                 a.directionToLocal(normalizedPerpendicular(glm::normalize(worldSliderAxis))), b,
                 b.pointToLocal(worldAnchor),
                 b.directionToLocal(glm::normalize(worldSliderAxis)),
                 b.directionToLocal(normalizedPerpendicular(glm::normalize(worldSliderAxis))))
{
}

SliderJoint::SliderJoint(RigidBody& a, const glm::vec3& localAnchorA,
                         const glm::vec3& localSliderAxisA, const glm::vec3& localNormalAxisA,
                         RigidBody& b, const glm::vec3& localAnchorB,
                         const glm::vec3& localSliderAxisB, const glm::vec3& localNormalAxisB)
    : mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA), mLocalAnchorB(localAnchorB),
      mLocalSliderAxisA(glm::normalize(localSliderAxisA)),
      mLocalNormalAxisA(glm::normalize(localNormalAxisA)),
      mLocalNormalAxisA2(glm::cross(mLocalSliderAxisA, mLocalNormalAxisA)),
      mInverseInitialOrientation(invInitialOrientationXY(
          mLocalSliderAxisA, mLocalNormalAxisA, glm::normalize(localSliderAxisB),
          glm::normalize(localNormalAxisB)))
{
}

RigidBody* SliderJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* SliderJoint::bodyB() const
{
    return mBodyB;
}

void SliderJoint::setLimits(f32 minDistance, f32 maxDistance)
{
    mLimitsMin = minDistance;
    mLimitsMax = maxDistance;
    mHasLimits = mLimitsMin != -M_INFINITY || mLimitsMax != M_INFINITY;
}

f32 SliderJoint::currentPosition() const
{
    const glm::vec3 armA = mBodyA->directionToWorld(mLocalAnchorA);
    const glm::vec3 armB = mBodyB->directionToWorld(mLocalAnchorB);
    const glm::vec3 offset = (mBodyB->position() - mBodyA->position()) + armB - armA;
    return glm::dot(offset, mBodyA->directionToWorld(mLocalSliderAxisA));
}

void SliderJoint::setMotor(f32 targetVelocity, f32 maxForce)
{
    if (!std::isfinite(targetVelocity) || !std::isfinite(maxForce))
        return;
    mMotorTargetVelocity = targetVelocity;
    mMotorMaxForce = glm::max(maxForce, 0.0f);
    mMotorEnabled = mMotorMaxForce > 0.0f;
}

void SliderJoint::disableMotor()
{
    mMotorEnabled = false;
    mTotalMotorImpulse = 0.0f;
}

void SliderJoint::calculateArmsAndOffset()
{
    mArmA = mBodyA->directionToWorld(mLocalAnchorA);
    mArmB = mBodyB->directionToWorld(mLocalAnchorB);
    mOffset = (mBodyB->position() - mBodyA->position()) + mArmB - mArmA;
}

void SliderJoint::calculatePositionLockProperties()
{
    mN1 = mBodyA->directionToWorld(mLocalNormalAxisA);
    mN2 = mBodyA->directionToWorld(mLocalNormalAxisA2);

    const glm::vec3 armAPlusOffset = mArmA + mOffset;
    const glm::vec3 r1x1 = glm::cross(armAPlusOffset, mN1);
    const glm::vec3 r1x2 = glm::cross(armAPlusOffset, mN2);
    const glm::vec3 r2x1 = glm::cross(mArmB, mN1);
    const glm::vec3 r2x2 = glm::cross(mArmB, mN2);

    glm::mat2 inverseEffectiveMass(0.0f);
    const f32 inverseMassSum = mBodyA->inverseMass() + mBodyB->inverseMass();
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
        mTotalPositionLockImpulse = glm::vec2(0.0f);
    }
}

void SliderJoint::calculateRotationProperties()
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

void SliderJoint::calculateSlideAxisAndPosition()
{
    mWorldSliderAxis = glm::normalize(mBodyA->directionToWorld(mLocalSliderAxisA));
    mSlidePosition = glm::dot(mOffset, mWorldSliderAxis);
}

void SliderJoint::calculateLimitProperties()
{
    mLimitActive = mHasLimits && (mSlidePosition <= mLimitsMin || mSlidePosition >= mLimitsMax);
    if (!mLimitActive)
    {
        mLimitEffectiveMass = 0.0f;
        mTotalLimitImpulse = 0.0f;
        return;
    }
    const glm::vec3 armAPlusOffset = mArmA + mOffset;
    f32 inverseEffectiveMass = mBodyA->inverseMass() + mBodyB->inverseMass();
    inverseEffectiveMass += glm::dot(
        mWorldSliderAxis,
        mBodyA->inverseInertiaTensorWorld() * glm::cross(armAPlusOffset, mWorldSliderAxis));
    inverseEffectiveMass += glm::dot(
        mWorldSliderAxis, mBodyB->inverseInertiaTensorWorld() * glm::cross(mArmB, mWorldSliderAxis));
    mLimitEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mLimitEffectiveMass == 0.0f)
        mLimitActive = false;
}

void SliderJoint::calculateMotorProperties()
{
    if (!mMotorEnabled)
    {
        mMotorEffectiveMass = 0.0f;
        return;
    }
    const glm::vec3 armAPlusOffset = mArmA + mOffset;
    f32 inverseEffectiveMass = mBodyA->inverseMass() + mBodyB->inverseMass();
    inverseEffectiveMass += glm::dot(
        mWorldSliderAxis,
        mBodyA->inverseInertiaTensorWorld() * glm::cross(armAPlusOffset, mWorldSliderAxis));
    inverseEffectiveMass += glm::dot(
        mWorldSliderAxis, mBodyB->inverseInertiaTensorWorld() * glm::cross(mArmB, mWorldSliderAxis));
    mMotorEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
}

void SliderJoint::setup(f32 duration)
{
    calculateArmsAndOffset();
    calculatePositionLockProperties();
    calculateRotationProperties();
    calculateSlideAxisAndPosition();
    calculateLimitProperties();
    calculateMotorProperties();
    mMotorMaxImpulse = mMotorMaxForce * duration;
    if (mPreviousDuration > 0.0f)
    {
        const f32 ratio = duration / mPreviousDuration;
        mTotalPositionLockImpulse *= ratio;
        mTotalRotationImpulse *= ratio;
        mTotalLimitImpulse *= ratio;
        mTotalMotorImpulse *= ratio;
    }
    else
    {
        mTotalPositionLockImpulse = glm::vec2(0.0f);
        mTotalRotationImpulse = glm::vec3(0.0f);
        mTotalLimitImpulse = 0.0f;
        mTotalMotorImpulse = 0.0f;
    }
    mTotalMotorImpulse = glm::clamp(mTotalMotorImpulse, -mMotorMaxImpulse, mMotorMaxImpulse);
    mPreviousDuration = duration;
}

void SliderJoint::applyVelocityImpulse(const glm::vec3& impulse)
{
    const glm::vec3 armAPlusOffset = mArmA + mOffset;
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

void SliderJoint::applyAngularVelocityImpulse(const glm::vec3& impulse)
{
    if (mBodyA->isDynamic())
        mBodyA->setAngularVelocity(mBodyA->angularVelocity() -
                                   mBodyA->inverseInertiaTensorWorld() * impulse);
    if (mBodyB->isDynamic())
        mBodyB->setAngularVelocity(mBodyB->angularVelocity() +
                                   mBodyB->inverseInertiaTensorWorld() * impulse);
}

void SliderJoint::warmStart()
{
    if (mMotorEnabled)
        applyVelocityImpulse(mWorldSliderAxis * mTotalMotorImpulse);
    applyVelocityImpulse(mN1 * mTotalPositionLockImpulse.x + mN2 * mTotalPositionLockImpulse.y);
    applyAngularVelocityImpulse(mTotalRotationImpulse);
    if (mLimitActive)
        applyVelocityImpulse(mWorldSliderAxis * mTotalLimitImpulse);
}

void SliderJoint::solveVelocity()
{
    if (mMotorEnabled)
    {
        const glm::vec3 armAPlusOffset = mArmA + mOffset;
        const f32 jv = glm::dot(mWorldSliderAxis, mBodyA->velocity() - mBodyB->velocity()) +
                      glm::dot(glm::cross(armAPlusOffset, mWorldSliderAxis),
                               mBodyA->angularVelocity()) -
                      glm::dot(glm::cross(mArmB, mWorldSliderAxis), mBodyB->angularVelocity());
        const f32 impulse = (jv + mMotorTargetVelocity) * mMotorEffectiveMass;
        const f32 previous = mTotalMotorImpulse;
        mTotalMotorImpulse = glm::clamp(previous + impulse, -mMotorMaxImpulse, mMotorMaxImpulse);
        applyVelocityImpulse(mWorldSliderAxis * (mTotalMotorImpulse - previous));
    }

    const glm::vec3 armAPlusOffset = mArmA + mOffset;
    const glm::vec3 deltaLinear = mBodyA->velocity() - mBodyB->velocity();
    glm::vec2 jv;
    jv.x = glm::dot(mN1, deltaLinear) + glm::dot(glm::cross(armAPlusOffset, mN1), mBodyA->angularVelocity()) -
          glm::dot(glm::cross(mArmB, mN1), mBodyB->angularVelocity());
    jv.y = glm::dot(mN2, deltaLinear) + glm::dot(glm::cross(armAPlusOffset, mN2), mBodyA->angularVelocity()) -
          glm::dot(glm::cross(mArmB, mN2), mBodyB->angularVelocity());
    const glm::vec2 lockImpulse = mPositionLockEffectiveMass * jv;
    mTotalPositionLockImpulse += lockImpulse;
    applyVelocityImpulse(mN1 * lockImpulse.x + mN2 * lockImpulse.y);

    const glm::vec3 rotationImpulse =
        mRotationEffectiveMass * (mBodyA->angularVelocity() - mBodyB->angularVelocity());
    mTotalRotationImpulse += rotationImpulse;
    applyAngularVelocityImpulse(rotationImpulse);

    if (mLimitActive)
    {
        f32 minImpulse = -M_INFINITY;
        f32 maxImpulse = M_INFINITY;
        if (mLimitsMin != mLimitsMax)
        {
            if (mSlidePosition <= mLimitsMin)
                minImpulse = 0.0f;
            else
                maxImpulse = 0.0f;
        }
        const f32 relative = glm::dot(mWorldSliderAxis, mBodyA->velocity() - mBodyB->velocity()) +
                             glm::dot(glm::cross(armAPlusOffset, mWorldSliderAxis),
                                      mBodyA->angularVelocity()) -
                             glm::dot(glm::cross(mArmB, mWorldSliderAxis), mBodyB->angularVelocity());
        const f32 impulse = mLimitEffectiveMass * relative;
        const f32 previous = mTotalLimitImpulse;
        mTotalLimitImpulse = glm::clamp(previous + impulse, minImpulse, maxImpulse);
        applyVelocityImpulse(mWorldSliderAxis * (mTotalLimitImpulse - previous));
    }
}

void SliderJoint::solvePosition(f32 baumgarte)
{
    calculateArmsAndOffset();
    calculatePositionLockProperties();
    const glm::vec2 c(glm::dot(mOffset, mN1), glm::dot(mOffset, mN2));
    if (c != glm::vec2(0.0f))
    {
        const glm::vec2 lambda = -baumgarte * (mPositionLockEffectiveMass * c);
        const glm::vec3 impulse = mN1 * lambda.x + mN2 * lambda.y;
        const glm::vec3 armAPlusOffset = mArmA + mOffset;
        mBodyA->applyPositionImpulseAtPoint(-impulse, mBodyA->position() + armAPlusOffset);
        mBodyB->applyPositionImpulseAtPoint(impulse, mBodyB->position() + mArmB);
    }

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

    if (mHasLimits)
    {
        calculateArmsAndOffset();
        calculateSlideAxisAndPosition();
        calculateLimitProperties();
        if (mLimitActive)
        {
            f32 error = 0.0f;
            if (mSlidePosition <= mLimitsMin)
                error = mSlidePosition - mLimitsMin;
            else
                error = mSlidePosition - mLimitsMax;
            const f32 lambda = -mLimitEffectiveMass * baumgarte * error;
            const glm::vec3 armAPlusOffset = mArmA + mOffset;
            mBodyA->applyPositionImpulseAtPoint(-(lambda * mWorldSliderAxis),
                                                mBodyA->position() + armAPlusOffset);
            mBodyB->applyPositionImpulseAtPoint(lambda * mWorldSliderAxis,
                                                mBodyB->position() + mArmB);
        }
    }
}

}
