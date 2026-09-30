#include "PCH.h"

#include "dynamics/HingeJoint.h"

#include "GameObject.h"
#include "JointAxis.h"
#include "Scene.h"
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

HingeJoint::HingeJoint() : Joint(JointKind::Hinge)
{
}

HingeJoint::HingeJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                       const Math::vec3& worldHingeAxis)
    : Joint(JointKind::Hinge)
{
    configure(a, b, worldAnchor, worldHingeAxis);
}

HingeJoint::HingeJoint(RigidBody& a, const Math::vec3& localAnchorA,
                       const Math::vec3& localHingeAxisA, const Math::vec3& localNormalAxisA,
                       RigidBody& b, const Math::vec3& localAnchorB,
                       const Math::vec3& localHingeAxisB, const Math::vec3& localNormalAxisB)
    : Joint(JointKind::Hinge), mBodyA(&a), mBodyB(&b), mLocalAnchorA(localAnchorA),
      mLocalAnchorB(localAnchorB),
      mLocalHingeAxisA(detail::normalizedAxisOr(localHingeAxisA, Math::vec3(0.0f, 1.0f, 0.0f))),
      mLocalHingeAxisB(detail::normalizedAxisOr(localHingeAxisB, Math::vec3(0.0f, 1.0f, 0.0f))),
      mLocalNormalAxisA(detail::normalizedAxisOr(localNormalAxisA, Math::vec3(1.0f, 0.0f, 0.0f))),
      mLocalNormalAxisB(detail::normalizedAxisOr(localNormalAxisB, Math::vec3(1.0f, 0.0f, 0.0f))),
      mInverseInitialOrientation(invInitialOrientationXZ(mLocalNormalAxisA, mLocalHingeAxisA,
                                                          mLocalNormalAxisB, mLocalHingeAxisB))
{
}

void HingeJoint::configure(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                           const Math::vec3& worldHingeAxis)
{
    mBodyA = &a;
    mBodyB = &b;
    mLocalAnchorA = a.pointToLocal(worldAnchor);
    mLocalAnchorB = b.pointToLocal(worldAnchor);
    // One guarded normalise up front: everything below is derived from it,
    // so a degenerate axis caught here cannot reach any of the four frames.
    const Math::vec3 axis = detail::normalizedAxisOr(worldHingeAxis, Math::vec3(0.0f, 1.0f, 0.0f));
    mLocalHingeAxisA = Math::normalize(a.directionToLocal(axis));
    mLocalHingeAxisB = Math::normalize(b.directionToLocal(axis));
    mLocalNormalAxisA = Math::normalize(a.directionToLocal(normalizedPerpendicular(axis)));
    mLocalNormalAxisB = Math::normalize(b.directionToLocal(normalizedPerpendicular(axis)));
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
    const Math::vec3 worldAxis = self->globalRotation() * Math::normalize(mAuthoredAxis);
    configure(*a, *b, self->globalPosition(), worldAxis);
    self->scene()->addJoint(this);
    mBuilt = true;
}

void HingeJoint::setAuthoredAxis(const Math::vec3& axis)
{
    if (Math::length(axis) > 1.0e-6f)
        mAuthoredAxis = Math::normalize(axis);
}

const Math::vec3& HingeJoint::authoredAxis() const
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

Math::vec3 HingeJoint::anchorWorldA() const
{
    return mBodyA->pointToWorld(mLocalAnchorA);
}

Math::vec3 HingeJoint::anchorWorldB() const
{
    return mBodyB->pointToWorld(mLocalAnchorB);
}

Math::vec3 HingeJoint::axisWorld() const
{
    return Math::normalize(mBodyA->directionToWorld(mLocalHingeAxisA));
}

void HingeJoint::setLimits(f32 minAngle, f32 maxAngle)
{
    mLimitsMin = Math::clamp(minAngle, -Math::pi<f32>(), 0.0f);
    mLimitsMax = Math::clamp(maxAngle, 0.0f, Math::pi<f32>());
    mHasLimits = mLimitsMin > -Math::pi<f32>() || mLimitsMax < Math::pi<f32>();
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
    const Math::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mBodyA->orientation());
    return rotationAngleAroundAxis(diff, mBodyA->directionToWorld(mLocalHingeAxisA));
}

void HingeJoint::setMotor(f32 targetAngularVelocity, f32 maxTorque)
{
    if (!std::isfinite(targetAngularVelocity) || !std::isfinite(maxTorque))
        return;
    // Same reason as setServo(): a new speed on a sleeping joint is an order
    // nobody is stepping.
    if (targetAngularVelocity != mMotorTargetVelocity || !mMotorEnabled)
        wakeBodies();
    mMotorTargetVelocity = targetAngularVelocity;
    mMotorMaxTorque = Math::max(maxTorque, 0.0f);
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
    // A new target has to wake the joint: a body that has settled is asleep
    // and the solver skips it, so the order would land on a joint nobody is
    // stepping. Only on a change - a controller holding a target calls this
    // every frame, and waking on every call means never sleeping again.
    if (targetAngle != mServoTargetAngle || !mServoEnabled)
        wakeBodies();
    mServoTargetAngle = targetAngle;
    mServoMaxAngularVelocity = Math::max(maxAngularVelocity, 0.0f);
    mMotorMaxTorque = Math::max(maxTorque, 0.0f);
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

void HingeJoint::calculateHingeRotationProperties()
{
    mA1 = Math::normalize(mBodyA->directionToWorld(mLocalHingeAxisA));
    Math::vec3 a2 = Math::normalize(mBodyB->directionToWorld(mLocalHingeAxisB));

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
        mHingeRotationEffectiveMass = Math::inverse(inverseEffectiveMass);
    else
    {
        mHingeRotationEffectiveMass = Math::mat2(0.0f);
        mTotalHingeRotationImpulse = Math::vec2(0.0f);
    }
}

void HingeJoint::calculateAxisAndAngle()
{
    mA1 = Math::normalize(mBodyA->directionToWorld(mLocalHingeAxisA));
    const Math::quat diff = mBodyB->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mBodyA->orientation());
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
    const f32 inverseEffectiveMass = Math::dot(
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
    const f32 inverseEffectiveMass = Math::dot(
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
            target = Math::clamp(target, mLimitsMin, mLimitsMax);
        const f32 error = target - currentAngle();
        // A servo still short of its target has work to do, and a sleeping
        // body is skipped by the solver - so it would sit there forever with
        // the order accepted and nothing moving. Waking on the error rather
        // than on the command also covers the target being set before the
        // joint was built, when there were no bodies to wake yet. Once it
        // arrives the error falls under this and the body sleeps again.
        if (std::abs(error) > 0.001f)
            wakeBodies();
        f32 velocity = error / duration;
        if (mServoMaxAngularVelocity > 0.0f)
            velocity = Math::clamp(velocity, -mServoMaxAngularVelocity, mServoMaxAngularVelocity);
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
        mTotalPositionImpulse = Math::vec3(0.0f);
        mTotalHingeRotationImpulse = Math::vec2(0.0f);
        mTotalLimitImpulse = 0.0f;
        mTotalMotorImpulse = 0.0f;
    }
    mTotalMotorImpulse = Math::clamp(mTotalMotorImpulse, -mMotorMaxImpulse, mMotorMaxImpulse);
    mPreviousDuration = duration;
}

void HingeJoint::applyVelocityImpulse(const Math::vec3& impulse)
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

void HingeJoint::applyAngularVelocityImpulse(const Math::vec3& impulse)
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
            Math::dot(mA1, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = (relativeVelocity + mMotorTargetVelocity) * mMotorEffectiveMass;
        const f32 previous = mTotalMotorImpulse;
        mTotalMotorImpulse =
            Math::clamp(previous + impulse, -mMotorMaxImpulse, mMotorMaxImpulse);
        applyAngularVelocityImpulse(mA1 * (mTotalMotorImpulse - previous));
    }

    const Math::vec3 relativeVelocity =
        mBodyB->velocity() + Math::cross(mBodyB->angularVelocity(), mArmB) -
        mBodyA->velocity() - Math::cross(mBodyA->angularVelocity(), mArmA);
    const Math::vec3 positionImpulse = -(mPositionEffectiveMass * relativeVelocity);
    mTotalPositionImpulse += positionImpulse;
    applyVelocityImpulse(positionImpulse);

    const Math::vec3 deltaAngular = mBodyA->angularVelocity() - mBodyB->angularVelocity();
    const Math::vec2 jv(Math::dot(mB2xA1, deltaAngular), Math::dot(mC2xA1, deltaAngular));
    const Math::vec2 hingeImpulse = mHingeRotationEffectiveMass * jv;
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
        const f32 relative = Math::dot(mA1, mBodyA->angularVelocity() - mBodyB->angularVelocity());
        const f32 impulse = mLimitEffectiveMass * relative;
        const f32 previous = mTotalLimitImpulse;
        mTotalLimitImpulse = Math::clamp(previous + impulse, minImpulse, maxImpulse);
        applyAngularVelocityImpulse(mA1 * (mTotalLimitImpulse - previous));
    }
}

void HingeJoint::solvePosition(f32 baumgarte)
{
    calculatePositionProperties();
    const Math::vec3 pointA = mBodyA->position() + mArmA;
    const Math::vec3 pointB = mBodyB->position() + mArmB;
    const Math::vec3 positionImpulse = -(mPositionEffectiveMass * (pointB - pointA)) * baumgarte;
    mBodyA->applyPositionImpulseAtPoint(-positionImpulse, pointA);
    mBodyB->applyPositionImpulseAtPoint(positionImpulse, pointB);

    calculateHingeRotationProperties();
    const Math::vec2 c(Math::dot(mA1, mB2), Math::dot(mA1, mC2));
    if (c != Math::vec2(0.0f))
    {
        const Math::vec2 lambda = -baumgarte * (mHingeRotationEffectiveMass * c);
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
                const Math::vec3 step = mBodyA->inverseInertiaTensorWorld() * mA1 * -lambda;
                const Math::quat spin(0.0f, step);
                mBodyA->setOrientation(mBodyA->orientation() +
                                       0.5f * spin * mBodyA->orientation());
            }
            if (mBodyB->isDynamic())
            {
                const Math::vec3 step = mBodyB->inverseInertiaTensorWorld() * mA1 * lambda;
                const Math::quat spin(0.0f, step);
                mBodyB->setOrientation(mBodyB->orientation() +
                                       0.5f * spin * mBodyB->orientation());
            }
        }
    }
}

}
