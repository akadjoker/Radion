#include "PCH.h"

#include "dynamics/HingeJoint.h"

#include "GameObject.h"
#include "Scene.h"
#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

namespace
{

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

glm::quat invInitialOrientationXZ(const glm::vec3& xAxisA, const glm::vec3& zAxisA,
                                  const glm::vec3& xAxisB, const glm::vec3& zAxisB)
{
    if (xAxisA == xAxisB && zAxisA == zAxisB)
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const glm::mat3 basisA(xAxisA, glm::cross(zAxisA, xAxisA), zAxisA);
    const glm::mat3 basisB(xAxisB, glm::cross(zAxisB, xAxisB), zAxisB);
    return glm::quat_cast(basisB) * glm::conjugate(glm::quat_cast(basisA));
}

}

HingeJoint::HingeJoint() : Joint(JointKind::Hinge)
{
}

HingeJoint::HingeJoint(RigidBody& a, RigidBody& b, const glm::vec3& worldAnchor,
                       const glm::vec3& worldHingeAxis)
    : Joint(JointKind::Hinge)
{
    configure(a, b, worldAnchor, worldHingeAxis);
}

HingeJoint::HingeJoint(RigidBody& a, const glm::vec3& localAnchorA,
                       const glm::vec3& localHingeAxisA, const glm::vec3& localNormalAxisA,
                       RigidBody& b, const glm::vec3& localAnchorB,
                       const glm::vec3& localHingeAxisB, const glm::vec3& localNormalAxisB)
    : Joint(JointKind::Hinge), mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA),
      mLocalAnchorB(localAnchorB), mLocalHingeAxisA(glm::normalize(localHingeAxisA)),
      mLocalHingeAxisB(glm::normalize(localHingeAxisB)),
      mLocalNormalAxisA(glm::normalize(localNormalAxisA)),
      mLocalNormalAxisB(glm::normalize(localNormalAxisB)),
      mInverseInitialOrientation(invInitialOrientationXZ(mLocalNormalAxisA, mLocalHingeAxisA,
                                                          mLocalNormalAxisB, mLocalHingeAxisB))
{
}

void HingeJoint::configure(RigidBody& a, RigidBody& b, const glm::vec3& worldAnchor,
                           const glm::vec3& worldHingeAxis)
{
    mBodyA = &a;
    mBodyB = &b;
    mLocalAnchorA = a.pointToLocal(worldAnchor);
    mLocalAnchorB = b.pointToLocal(worldAnchor);
    mLocalHingeAxisA = glm::normalize(a.directionToLocal(glm::normalize(worldHingeAxis)));
    mLocalHingeAxisB = glm::normalize(b.directionToLocal(glm::normalize(worldHingeAxis)));
    mLocalNormalAxisA = glm::normalize(
        a.directionToLocal(normalizedPerpendicular(glm::normalize(worldHingeAxis))));
    mLocalNormalAxisB = glm::normalize(
        b.directionToLocal(normalizedPerpendicular(glm::normalize(worldHingeAxis))));
    mInverseInitialOrientation = invInitialOrientationXZ(mLocalNormalAxisA, mLocalHingeAxisA,
                                                         mLocalNormalAxisB, mLocalHingeAxisB);
}

void HingeJoint::rebuild()
{
    GameObject* self = owner();
    GameObject* other = connectedBody();
    if (!self || !other)
        return;
    RigidBody* a = self->getComponent<RigidBody>();
    RigidBody* b = other->getComponent<RigidBody>();
    if (!a || !b)
        return;
    const glm::vec3 worldAxis = self->globalRotation() * glm::normalize(mAuthoredAxis);
    configure(*a, *b, self->globalPosition(), worldAxis);
    self->scene()->addJoint(this);
    mBuilt = true;
}

void HingeJoint::setAuthoredAxis(const glm::vec3& axis)
{
    if (glm::length(axis) > 1.0e-6f)
        mAuthoredAxis = glm::normalize(axis);
}

const glm::vec3& HingeJoint::authoredAxis() const
{
    return mAuthoredAxis;
}

RigidBody* HingeJoint::bodyA() const
{
    return mBodyA;
}

RigidBody* HingeJoint::bodyB() const
{
    return mBodyB;
}

glm::vec3 HingeJoint::anchorWorldA() const
{
    return mBodyA->pointToWorld(mLocalAnchorA);
}

glm::vec3 HingeJoint::anchorWorldB() const
{
    return mBodyB->pointToWorld(mLocalAnchorB);
}

glm::vec3 HingeJoint::axisWorld() const
{
    return glm::normalize(mBodyA->directionToWorld(mLocalHingeAxisA));
}

void HingeJoint::setLimits(f32 minAngle, f32 maxAngle)
{
    mLimitsMin = glm::clamp(minAngle, -glm::pi<f32>(), 0.0f);
    mLimitsMax = glm::clamp(maxAngle, 0.0f, glm::pi<f32>());
    mHasLimits = mLimitsMin > -glm::pi<f32>() || mLimitsMax < glm::pi<f32>();
}

f32 HingeJoint::minAngle() const
{
    return mLimitsMin;
}

f32 HingeJoint::maxAngle() const
{
    return mLimitsMax;
}

f32 HingeJoint::currentAngle() const
{
    const glm::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
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

f32 HingeJoint::motorTargetVelocity() const
{
    return mMotorTargetVelocity;
}

f32 HingeJoint::motorMaxTorque() const
{
    return mMotorMaxTorque;
}

bool HingeJoint::motorEnabled() const
{
    return mMotorEnabled;
}

void HingeJoint::disableMotor()
{
    mMotorEnabled = false;
    mServoEnabled = false;
    mTotalMotorImpulse = 0.0f;
}

void HingeJoint::setServo(f32 targetAngle, f32 maxTorque, f32 maxAngularVelocity)
{
    if (!std::isfinite(targetAngle) || !std::isfinite(maxTorque) ||
        !std::isfinite(maxAngularVelocity))
        return;
    mServoTargetAngle = targetAngle;
    mServoMaxAngularVelocity = glm::max(maxAngularVelocity, 0.0f);
    mMotorMaxTorque = glm::max(maxTorque, 0.0f);
    mServoEnabled = mMotorMaxTorque > 0.0f;
    mMotorEnabled = mServoEnabled;
}

f32 HingeJoint::servoMaxAngularVelocity() const
{
    return mServoMaxAngularVelocity;
}

void HingeJoint::disableServo()
{
    mServoEnabled = false;
    mMotorEnabled = false;
    mTotalMotorImpulse = 0.0f;
}

f32 HingeJoint::servoTargetAngle() const
{
    return mServoTargetAngle;
}

bool HingeJoint::servoEnabled() const
{
    return mServoEnabled;
}

void HingeJoint::calculatePositionProperties()
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

void HingeJoint::calculateHingeRotationProperties()
{
    mA1 = glm::normalize(mBodyA->directionToWorld(mLocalHingeAxisA));
    glm::vec3 a2 = glm::normalize(mBodyB->directionToWorld(mLocalHingeAxisB));

    const f32 dot = glm::dot(mA1, a2);
    if (dot <= 1.0e-3f)
    {
        glm::vec3 perpendicular = a2 - dot * mA1;
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

    const glm::mat3 inverseInertiaSum =
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
        mTotalHingeRotationImpulse = glm::vec2(0.0f);
    }
}

void HingeJoint::calculateAxisAndAngle()
{
    mA1 = glm::normalize(mBodyA->directionToWorld(mLocalHingeAxisA));
    const glm::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
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
    // The servo is the velocity motor fed a speed recomputed from the angle
    // error every step - the speed that would close the whole error in one
    // step, which mMotorMaxImpulse below is what actually rations out. Same
    // as btHingeConstraint::setMotorTarget(), except the target is held here
    // instead of being handed in again every frame by the caller.
    if (mServoEnabled && duration > 0.0f)
    {
        f32 target = mServoTargetAngle;
        if (mHasLimits)
            target = glm::clamp(target, mLimitsMin, mLimitsMax);
        f32 velocity = (target - currentAngle()) / duration;
        if (mServoMaxAngularVelocity > 0.0f)
            velocity = glm::clamp(velocity, -mServoMaxAngularVelocity, mServoMaxAngularVelocity);
        mMotorTargetVelocity = velocity;
    }
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
        mTotalPositionImpulse = glm::vec3(0.0f);
        mTotalHingeRotationImpulse = glm::vec2(0.0f);
        mTotalLimitImpulse = 0.0f;
        mTotalMotorImpulse = 0.0f;
    }
    mTotalMotorImpulse = glm::clamp(mTotalMotorImpulse, -mMotorMaxImpulse, mMotorMaxImpulse);
    mPreviousDuration = duration;
}

void HingeJoint::applyVelocityImpulse(const glm::vec3& impulse)
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

void HingeJoint::applyAngularVelocityImpulse(const glm::vec3& impulse)
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

    const glm::vec3 relativeVelocity =
        mBodyB->velocity() + glm::cross(mBodyB->angularVelocity(), mArmB) -
        mBodyA->velocity() - glm::cross(mBodyA->angularVelocity(), mArmA);
    const glm::vec3 positionImpulse = -(mPositionEffectiveMass * relativeVelocity);
    mTotalPositionImpulse += positionImpulse;
    applyVelocityImpulse(positionImpulse);

    const glm::vec3 deltaAngular = mBodyA->angularVelocity() - mBodyB->angularVelocity();
    const glm::vec2 jv(glm::dot(mB2xA1, deltaAngular), glm::dot(mC2xA1, deltaAngular));
    const glm::vec2 hingeImpulse = mHingeRotationEffectiveMass * jv;
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
    const glm::vec3 pointA = mBodyA->position() + mArmA;
    const glm::vec3 pointB = mBodyB->position() + mArmB;
    const glm::vec3 positionImpulse = -(mPositionEffectiveMass * (pointB - pointA)) * baumgarte;
    mBodyA->applyPositionImpulseAtPoint(-positionImpulse, pointA);
    mBodyB->applyPositionImpulseAtPoint(positionImpulse, pointB);

    calculateHingeRotationProperties();
    const glm::vec2 c(glm::dot(mA1, mB2), glm::dot(mA1, mC2));
    if (c != glm::vec2(0.0f))
    {
        const glm::vec2 lambda = -baumgarte * (mHingeRotationEffectiveMass * c);
        const glm::vec3 impulse = mB2xA1 * lambda.x + mC2xA1 * lambda.y;
        if (mBodyA->isDynamic())
        {
            const glm::vec3 step = mBodyA->inverseInertiaTensorWorld() * -impulse;
            const glm::quat spin(0.0f, step);
            mBodyA->setOrientation(mBodyA->orientation() + 0.5f * spin * mBodyA->orientation());
        }
        if (mBodyB->isDynamic())
        {
            const glm::vec3 step = mBodyB->inverseInertiaTensorWorld() * impulse;
            const glm::quat spin(0.0f, step);
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
                const glm::vec3 step = mBodyA->inverseInertiaTensorWorld() * mA1 * -lambda;
                const glm::quat spin(0.0f, step);
                mBodyA->setOrientation(mBodyA->orientation() +
                                       0.5f * spin * mBodyA->orientation());
            }
            if (mBodyB->isDynamic())
            {
                const glm::vec3 step = mBodyB->inverseInertiaTensorWorld() * mA1 * lambda;
                const glm::quat spin(0.0f, step);
                mBodyB->setOrientation(mBodyB->orientation() +
                                       0.5f * spin * mBodyB->orientation());
            }
        }
    }
}

}
