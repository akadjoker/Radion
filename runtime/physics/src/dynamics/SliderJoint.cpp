#include "PCH.h"

#include "dynamics/SliderJoint.h"

#include "GameObject.h"
#include "Scene.h"
#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

namespace
{

Math::quat invInitialOrientationXY(const Math::vec3& xAxisA, const Math::vec3& yAxisA,
                                  const Math::vec3& xAxisB, const Math::vec3& yAxisB)
{
    if (xAxisA == xAxisB && yAxisA == yAxisB)
        return Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const Math::mat3 basisA(xAxisA, yAxisA, Math::cross(xAxisA, yAxisA));
    const Math::mat3 basisB(xAxisB, yAxisB, Math::cross(xAxisB, yAxisB));
    return Math::quat_cast(basisB) * Math::conjugate(Math::quat_cast(basisA));
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

SliderJoint::SliderJoint() : Joint(JointKind::Slider)
{
}

SliderJoint::SliderJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                         const Math::vec3& worldSliderAxis)
    : Joint(JointKind::Slider)
{
    configure(a, b, worldAnchor, worldSliderAxis);
}

SliderJoint::SliderJoint(RigidBody& a, const Math::vec3& localAnchorA,
                         const Math::vec3& localSliderAxisA, const Math::vec3& localNormalAxisA,
                         RigidBody& b, const Math::vec3& localAnchorB,
                         const Math::vec3& localSliderAxisB, const Math::vec3& localNormalAxisB)
    : Joint(JointKind::Slider), mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA),
      mLocalAnchorB(localAnchorB), mLocalSliderAxisA(Math::normalize(localSliderAxisA)),
      mLocalNormalAxisA(Math::normalize(localNormalAxisA)),
      mLocalNormalAxisA2(Math::cross(mLocalSliderAxisA, mLocalNormalAxisA)),
      mInverseInitialOrientation(invInitialOrientationXY(
          mLocalSliderAxisA, mLocalNormalAxisA, Math::normalize(localSliderAxisB),
          Math::normalize(localNormalAxisB)))
{
}

void SliderJoint::configure(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                            const Math::vec3& worldSliderAxis)
{
    mBodyA = &a;
    mBodyB = &b;
    mLocalAnchorA = a.pointToLocal(worldAnchor);
    mLocalAnchorB = b.pointToLocal(worldAnchor);
    mLocalSliderAxisA = Math::normalize(a.directionToLocal(Math::normalize(worldSliderAxis)));
    mLocalNormalAxisA = Math::normalize(
        a.directionToLocal(normalizedPerpendicular(Math::normalize(worldSliderAxis))));
    mLocalNormalAxisA2 = Math::cross(mLocalSliderAxisA, mLocalNormalAxisA);
    const Math::vec3 localSliderAxisB = b.directionToLocal(Math::normalize(worldSliderAxis));
    const Math::vec3 localNormalAxisB =
        b.directionToLocal(normalizedPerpendicular(Math::normalize(worldSliderAxis)));
    mInverseInitialOrientation = invInitialOrientationXY(
        mLocalSliderAxisA, mLocalNormalAxisA, Math::normalize(localSliderAxisB),
        Math::normalize(localNormalAxisB));
}

void SliderJoint::rebuild()
{
    GameObject* self = owner();
    GameObject* other = connectedBody();
    if (!self || !other)
        return;
    RigidBody* a = self->getComponent<RigidBody>();
    RigidBody* b = other->getComponent<RigidBody>();
    if (!a || !b)
        return;
    const Math::vec3 worldAxis = self->globalRotation() * Math::normalize(mAuthoredAxis);
    configure(*a, *b, self->globalPosition(), worldAxis);
    self->scene()->addJoint(this);
    mBuilt = true;
}

void SliderJoint::setAuthoredAxis(const Math::vec3& axis)
{
    if (Math::length(axis) > 1.0e-6f)
        mAuthoredAxis = Math::normalize(axis);
}

const Math::vec3& SliderJoint::authoredAxis() const
{
    return mAuthoredAxis;
}

RigidBody* SliderJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* SliderJoint::bodyB() const
{
    return mBodyB;
}

Math::vec3 SliderJoint::anchorWorldA() const
{
    return mBodyA->pointToWorld(mLocalAnchorA);
}

Math::vec3 SliderJoint::anchorWorldB() const
{
    return mBodyB->pointToWorld(mLocalAnchorB);
}

Math::vec3 SliderJoint::axisWorld() const
{
    return mBodyA->directionToWorld(mLocalSliderAxisA);
}

void SliderJoint::setLimits(f32 minDistance, f32 maxDistance)
{
    mLimitsMin = minDistance;
    mLimitsMax = maxDistance;
    mHasLimits = mLimitsMin != -M_INFINITY || mLimitsMax != M_INFINITY;
}

f32 SliderJoint::minDistance() const
{
    return mLimitsMin;
}

f32 SliderJoint::maxDistance() const
{
    return mLimitsMax;
}

f32 SliderJoint::currentPosition() const
{
    const Math::vec3 armA = mBodyA->directionToWorld(mLocalAnchorA);
    const Math::vec3 armB = mBodyB->directionToWorld(mLocalAnchorB);
    const Math::vec3 offset = (mBodyB->position() - mBodyA->position()) + armB - armA;
    return Math::dot(offset, mBodyA->directionToWorld(mLocalSliderAxisA));
}

void SliderJoint::setMotor(f32 targetVelocity, f32 maxForce)
{
    if (!std::isfinite(targetVelocity) || !std::isfinite(maxForce))
        return;
    if (targetVelocity != mMotorTargetVelocity || !mMotorEnabled)
        wakeBodies();
    mMotorTargetVelocity = targetVelocity;
    mMotorMaxForce = Math::max(maxForce, 0.0f);
    mMotorEnabled = mMotorMaxForce > 0.0f;
}

f32 SliderJoint::motorTargetVelocity() const
{
    return mMotorTargetVelocity;
}

f32 SliderJoint::motorMaxForce() const
{
    return mMotorMaxForce;
}

bool SliderJoint::motorEnabled() const
{
    return mMotorEnabled;
}

void SliderJoint::disableMotor()
{
    mMotorEnabled = false;
    mServoEnabled = false;
    mTotalMotorImpulse = 0.0f;
}

void SliderJoint::setServo(f32 targetPosition, f32 maxForce, f32 maxSpeed)
{
    if (!std::isfinite(targetPosition) || !std::isfinite(maxForce) || !std::isfinite(maxSpeed))
        return;
    // A settled body is asleep and skipped, so a new target must wake it (see HingeJoint::setServo()).
    if (targetPosition != mServoTargetPosition || !mServoEnabled)
        wakeBodies();
    mServoTargetPosition = targetPosition;
    mServoMaxSpeed = Math::max(maxSpeed, 0.0f);
    mMotorMaxForce = Math::max(maxForce, 0.0f);
    mServoEnabled = mMotorMaxForce > 0.0f;
    mMotorEnabled = mServoEnabled;
}

f32 SliderJoint::servoMaxSpeed() const
{
    return mServoMaxSpeed;
}

void SliderJoint::disableServo()
{
    mServoEnabled = false;
    mMotorEnabled = false;
    mTotalMotorImpulse = 0.0f;
}

f32 SliderJoint::servoTargetPosition() const
{
    return mServoTargetPosition;
}

bool SliderJoint::servoEnabled() const
{
    return mServoEnabled;
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

    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    const Math::vec3 r1x1 = Math::cross(armAPlusOffset, mN1);
    const Math::vec3 r1x2 = Math::cross(armAPlusOffset, mN2);
    const Math::vec3 r2x1 = Math::cross(mArmB, mN1);
    const Math::vec3 r2x2 = Math::cross(mArmB, mN2);

    Math::mat2 inverseEffectiveMass(0.0f);
    const f32 inverseMassSum = mBodyA->inverseMass() + mBodyB->inverseMass();
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

void SliderJoint::calculateRotationProperties()
{
    const Math::mat3 inverseInertiaSum =
        mBodyA->inverseInertiaTensorWorld() + mBodyB->inverseInertiaTensorWorld();
    const f32 determinant = Math::determinant(inverseInertiaSum);
    if (std::abs(determinant) > 1.0e-9f && std::isfinite(determinant))
        mRotationEffectiveMass = Math::inverse(inverseInertiaSum);
    else
    {
        mRotationEffectiveMass = Math::mat3(0.0f);
        mTotalRotationImpulse = Math::vec3(0.0f);
    }
}

void SliderJoint::calculateSlideAxisAndPosition()
{
    mWorldSliderAxis = Math::normalize(mBodyA->directionToWorld(mLocalSliderAxisA));
    mSlidePosition = Math::dot(mOffset, mWorldSliderAxis);
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
    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    f32 inverseEffectiveMass = mBodyA->inverseMass() + mBodyB->inverseMass();
    inverseEffectiveMass += Math::dot(
        mWorldSliderAxis,
        mBodyA->inverseInertiaTensorWorld() * Math::cross(armAPlusOffset, mWorldSliderAxis));
    inverseEffectiveMass += Math::dot(
        mWorldSliderAxis, mBodyB->inverseInertiaTensorWorld() * Math::cross(mArmB, mWorldSliderAxis));
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
    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    f32 inverseEffectiveMass = mBodyA->inverseMass() + mBodyB->inverseMass();
    inverseEffectiveMass += Math::dot(
        mWorldSliderAxis,
        mBodyA->inverseInertiaTensorWorld() * Math::cross(armAPlusOffset, mWorldSliderAxis));
    inverseEffectiveMass += Math::dot(
        mWorldSliderAxis, mBodyB->inverseInertiaTensorWorld() * Math::cross(mArmB, mWorldSliderAxis));
    mMotorEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
}

void SliderJoint::setup(f32 duration)
{
    calculateArmsAndOffset();
    calculatePositionLockProperties();
    calculateRotationProperties();
    calculateSlideAxisAndPosition();
    calculateLimitProperties();
    // As HingeJoint::setup(): servo = velocity motor fed the speed that closes the error in one step, rationed by mMotorMaxImpulse.
    if (mServoEnabled && duration > 0.0f)
    {
        f32 target = mServoTargetPosition;
        if (mHasLimits)
            target = Math::clamp(target, mLimitsMin, mLimitsMax);
        const f32 error = target - currentPosition();
        // A servo with work left keeps its bodies awake (see HingeJoint::setup()).
        if (std::abs(error) > 0.001f)
            wakeBodies();
        f32 velocity = error / duration;
        if (mServoMaxSpeed > 0.0f)
            velocity = Math::clamp(velocity, -mServoMaxSpeed, mServoMaxSpeed);
        mMotorTargetVelocity = velocity;
    }
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
        mTotalPositionLockImpulse = Math::vec2(0.0f);
        mTotalRotationImpulse = Math::vec3(0.0f);
        mTotalLimitImpulse = 0.0f;
        mTotalMotorImpulse = 0.0f;
    }
    mTotalMotorImpulse = Math::clamp(mTotalMotorImpulse, -mMotorMaxImpulse, mMotorMaxImpulse);
    mPreviousDuration = duration;
}

void SliderJoint::applyVelocityImpulse(const Math::vec3& impulse)
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

void SliderJoint::applyAngularVelocityImpulse(const Math::vec3& impulse)
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
        const Math::vec3 armAPlusOffset = mArmA + mOffset;
        const f32 jv = Math::dot(mWorldSliderAxis, mBodyA->velocity() - mBodyB->velocity()) +
                      Math::dot(Math::cross(armAPlusOffset, mWorldSliderAxis),
                               mBodyA->angularVelocity()) -
                      Math::dot(Math::cross(mArmB, mWorldSliderAxis), mBodyB->angularVelocity());
        const f32 impulse = (jv + mMotorTargetVelocity) * mMotorEffectiveMass;
        const f32 previous = mTotalMotorImpulse;
        mTotalMotorImpulse = Math::clamp(previous + impulse, -mMotorMaxImpulse, mMotorMaxImpulse);
        applyVelocityImpulse(mWorldSliderAxis * (mTotalMotorImpulse - previous));
    }

    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    const Math::vec3 deltaLinear = mBodyA->velocity() - mBodyB->velocity();
    Math::vec2 jv;
    jv.x = Math::dot(mN1, deltaLinear) + Math::dot(Math::cross(armAPlusOffset, mN1), mBodyA->angularVelocity()) -
          Math::dot(Math::cross(mArmB, mN1), mBodyB->angularVelocity());
    jv.y = Math::dot(mN2, deltaLinear) + Math::dot(Math::cross(armAPlusOffset, mN2), mBodyA->angularVelocity()) -
          Math::dot(Math::cross(mArmB, mN2), mBodyB->angularVelocity());
    const Math::vec2 lockImpulse = mPositionLockEffectiveMass * jv;
    mTotalPositionLockImpulse += lockImpulse;
    applyVelocityImpulse(mN1 * lockImpulse.x + mN2 * lockImpulse.y);

    const Math::vec3 rotationImpulse =
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
        const f32 relative = Math::dot(mWorldSliderAxis, mBodyA->velocity() - mBodyB->velocity()) +
                             Math::dot(Math::cross(armAPlusOffset, mWorldSliderAxis),
                                      mBodyA->angularVelocity()) -
                             Math::dot(Math::cross(mArmB, mWorldSliderAxis), mBodyB->angularVelocity());
        const f32 impulse = mLimitEffectiveMass * relative;
        const f32 previous = mTotalLimitImpulse;
        mTotalLimitImpulse = Math::clamp(previous + impulse, minImpulse, maxImpulse);
        applyVelocityImpulse(mWorldSliderAxis * (mTotalLimitImpulse - previous));
    }
}

void SliderJoint::solvePosition(f32 baumgarte)
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

    calculateRotationProperties();
    Math::quat diff =
        mBodyB->orientation() * mInverseInitialOrientation * Math::conjugate(mBodyA->orientation());
    if (diff.w < 0.0f)
        diff = -diff;
    const Math::vec3 rotationError(2.0f * diff.x, 2.0f * diff.y, 2.0f * diff.z);
    if (rotationError != Math::vec3(0.0f))
    {
        const Math::vec3 lambda = -baumgarte * (mRotationEffectiveMass * rotationError);
        if (mBodyA->isDynamic())
        {
            const Math::vec3 step = mBodyA->inverseInertiaTensorWorld() * -lambda;
            const Math::quat spin(0.0f, step);
            mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
        }
        if (mBodyB->isDynamic())
        {
            const Math::vec3 step = mBodyB->inverseInertiaTensorWorld() * lambda;
            const Math::quat spin(0.0f, step);
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
            const Math::vec3 armAPlusOffset = mArmA + mOffset;
            mBodyA->applyPositionImpulseAtPoint(-(lambda * mWorldSliderAxis),
                                                mBodyA->position() + armAPlusOffset);
            mBodyB->applyPositionImpulseAtPoint(lambda * mWorldSliderAxis,
                                                mBodyB->position() + mArmB);
        }
    }
}

}
