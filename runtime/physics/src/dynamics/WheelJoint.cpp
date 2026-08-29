
#include "PCH.h"

#include "dynamics/WheelJoint.h"

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

} // namespace

WheelJoint::WheelJoint(RigidBody& chassis, RigidBody& wheel, const glm::vec3& worldAnchor,
                       const glm::vec3& worldSuspensionAxis, const glm::vec3& worldSpinAxis)
    : mChassis(&chassis), mWheel(&wheel), mLocalAnchorChassis(chassis.pointToLocal(worldAnchor)),
      mLocalAnchorWheel(wheel.pointToLocal(worldAnchor)),
      mLocalSuspensionAxis(chassis.directionToLocal(glm::normalize(worldSuspensionAxis))),
      mLocalSpinAxis(wheel.directionToLocal(glm::normalize(worldSpinAxis))),
      mLocalNormalAxis(chassis.directionToLocal(normalizedPerpendicular(glm::normalize(worldSuspensionAxis)))),
      mInverseInitialOrientation(glm::conjugate(wheel.orientation()) * chassis.orientation())
{
}

RigidBody* WheelJoint::bodyA() const
{
    return mChassis;
}

RigidBody* WheelJoint::bodyB() const
{
    return mWheel;
}

void WheelJoint::setSteeringLimits(f32 minAngle, f32 maxAngle)
{
    mSteeringLimitsMin = glm::clamp(minAngle, -glm::pi<f32>(), 0.0f);
    mSteeringLimitsMax = glm::clamp(maxAngle, 0.0f, glm::pi<f32>());
    mHasSteeringLimits = mSteeringLimitsMin > -glm::pi<f32>() || mSteeringLimitsMax < glm::pi<f32>();
}

void WheelJoint::setSteeringMotor(f32 targetAngularVelocity, f32 maxTorque)
{
    if (!std::isfinite(targetAngularVelocity) || !std::isfinite(maxTorque))
        return;
    mSteeringMotorTargetVelocity = targetAngularVelocity;
    mSteeringMotorMaxTorque = glm::max(maxTorque, 0.0f);
    mSteeringMotorEnabled = mSteeringMotorMaxTorque > 0.0f;
}

void WheelJoint::disableSteeringMotor()
{
    mSteeringMotorEnabled = false;
    mTotalSteeringMotorImpulse = 0.0f;
}

f32 WheelJoint::steeringAngle() const
{
    return mSteeringAngle;
}

void WheelJoint::setSpinMotor(f32 targetAngularVelocity, f32 maxTorque)
{
    if (!std::isfinite(targetAngularVelocity) || !std::isfinite(maxTorque))
        return;
    mSpinMotorTargetVelocity = targetAngularVelocity;
    mSpinMotorMaxTorque = glm::max(maxTorque, 0.0f);
    mSpinMotorEnabled = mSpinMotorMaxTorque > 0.0f;
}

void WheelJoint::disableSpinMotor()
{
    mSpinMotorEnabled = false;
    mTotalSpinMotorImpulse = 0.0f;
}

f32 WheelJoint::spinAngle() const
{
    return mSpinAngleValue;
}

f32 WheelJoint::spinAngularVelocity() const
{
    return glm::dot(mAxisB, mWheel->angularVelocity() - mChassis->angularVelocity());
}

void WheelJoint::calculateArmsAndOffset()
{
    mArmA = mChassis->directionToWorld(mLocalAnchorChassis);
    mArmB = mWheel->directionToWorld(mLocalAnchorWheel);
    mOffset = (mWheel->position() - mChassis->position()) + mArmB - mArmA;
}

void WheelJoint::calculatePositionLockProperties()
{
    mAxisA = mChassis->directionToWorld(mLocalSuspensionAxis);
    mN1 = mChassis->directionToWorld(mLocalNormalAxis);
    mN2 = glm::cross(mAxisA, mN1);

    const glm::vec3 armAPlusOffset = mArmA + mOffset;
    const glm::vec3 r1x1 = glm::cross(armAPlusOffset, mN1);
    const glm::vec3 r1x2 = glm::cross(armAPlusOffset, mN2);
    const glm::vec3 r2x1 = glm::cross(mArmB, mN1);
    const glm::vec3 r2x2 = glm::cross(mArmB, mN2);

    const f32 inverseMassSum = mChassis->inverseMass() + mWheel->inverseMass();
    glm::mat2 inverseEffectiveMass(0.0f);
    inverseEffectiveMass[0][0] = inverseMassSum + glm::dot(r1x1, mChassis->inverseInertiaTensorWorld() * r1x1) +
                                 glm::dot(r2x1, mWheel->inverseInertiaTensorWorld() * r2x1);
    inverseEffectiveMass[0][1] = glm::dot(r1x1, mChassis->inverseInertiaTensorWorld() * r1x2) +
                                 glm::dot(r2x1, mWheel->inverseInertiaTensorWorld() * r2x2);
    inverseEffectiveMass[1][0] = glm::dot(r1x2, mChassis->inverseInertiaTensorWorld() * r1x1) +
                                 glm::dot(r2x2, mWheel->inverseInertiaTensorWorld() * r2x1);
    inverseEffectiveMass[1][1] = inverseMassSum + glm::dot(r1x2, mChassis->inverseInertiaTensorWorld() * r1x2) +
                                 glm::dot(r2x2, mWheel->inverseInertiaTensorWorld() * r2x2);

    const f32 determinant = glm::determinant(inverseEffectiveMass);
    if (std::abs(determinant) > 1.0e-9f && std::isfinite(determinant))
        mPositionLockEffectiveMass = glm::inverse(inverseEffectiveMass);
    else
    {
        mPositionLockEffectiveMass = glm::mat2(0.0f);
        mTotalPositionLockImpulse = glm::vec2(0.0f);
    }
}

void WheelJoint::calculatePerpendicularityProperties()
{
    mAxisA = glm::normalize(mChassis->directionToWorld(mLocalSuspensionAxis));
    mAxisB = glm::normalize(mWheel->directionToWorld(mLocalSpinAxis));

    const f32 k = glm::dot(mAxisA, mAxisB);
    const glm::vec3 axisBPerpendicular = mAxisB - k * mAxisA;
    const f32 length = glm::length(axisBPerpendicular);
    mPerpendicularAxis = length > 1.0e-6f ? glm::normalize(glm::cross(mAxisA, axisBPerpendicular))
                                          : normalizedPerpendicular(mAxisA);
    mPerpendicularity = k;

    const f32 inverseEffectiveMass = glm::dot(
        mPerpendicularAxis, mChassis->inverseInertiaTensorWorld() * mPerpendicularAxis +
                                mWheel->inverseInertiaTensorWorld() * mPerpendicularAxis);
    mPerpendicularEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mPerpendicularEffectiveMass == 0.0f)
        mTotalPerpendicularImpulse = 0.0f;
}

void WheelJoint::calculateAngles()
{
    mSlidePosition = glm::dot(mOffset, mAxisA);
    const glm::quat diff = mWheel->orientation() * mInverseInitialOrientation *
                           glm::conjugate(mChassis->orientation());
    mSteeringAngle = rotationAngleAroundAxis(diff, mAxisA);
    mSpinAngleValue = rotationAngleAroundAxis(diff, mAxisB);
}

void WheelJoint::calculateSteeringLimitProperties()
{
    mSteeringLimitActive =
        mHasSteeringLimits && (mSteeringAngle <= mSteeringLimitsMin || mSteeringAngle >= mSteeringLimitsMax);
    if (!mSteeringLimitActive)
    {
        mSteeringLimitEffectiveMass = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = glm::dot(
        mAxisA, mChassis->inverseInertiaTensorWorld() * mAxisA + mWheel->inverseInertiaTensorWorld() * mAxisA);
    mSteeringLimitEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mSteeringLimitEffectiveMass == 0.0f)
        mSteeringLimitActive = false;
}

void WheelJoint::calculateSteeringMotorProperties()
{
    if (!mSteeringMotorEnabled)
    {
        mSteeringMotorEffectiveMass = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = glm::dot(
        mAxisA, mChassis->inverseInertiaTensorWorld() * mAxisA + mWheel->inverseInertiaTensorWorld() * mAxisA);
    mSteeringMotorEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
}

void WheelJoint::calculateSpinMotorProperties()
{
    if (!mSpinMotorEnabled)
    {
        mSpinMotorEffectiveMass = 0.0f;
        return;
    }
    const f32 inverseEffectiveMass = glm::dot(
        mAxisB, mChassis->inverseInertiaTensorWorld() * mAxisB + mWheel->inverseInertiaTensorWorld() * mAxisB);
    mSpinMotorEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
}

void WheelJoint::applyLinearImpulse(const glm::vec3& impulse)
{
    const glm::vec3 armAPlusOffset = mArmA + mOffset;
    if (mChassis->isDynamic())
    {
        mChassis->setVelocity(mChassis->velocity() - impulse * mChassis->inverseMass());
        mChassis->setAngularVelocity(
            mChassis->angularVelocity() -
            mChassis->inverseInertiaTensorWorld() * glm::cross(armAPlusOffset, impulse));
    }
    if (mWheel->isDynamic())
    {
        mWheel->setVelocity(mWheel->velocity() + impulse * mWheel->inverseMass());
        mWheel->setAngularVelocity(
            mWheel->angularVelocity() +
            mWheel->inverseInertiaTensorWorld() * glm::cross(mArmB, impulse));
    }
}

void WheelJoint::applyAngularImpulse(const glm::vec3& impulse)
{
    if (mChassis->isDynamic())
        mChassis->setAngularVelocity(mChassis->angularVelocity() -
                                     mChassis->inverseInertiaTensorWorld() * impulse);
    if (mWheel->isDynamic())
        mWheel->setAngularVelocity(mWheel->angularVelocity() +
                                   mWheel->inverseInertiaTensorWorld() * impulse);
}

// F = -k*x - c*v, applied once per step like a real spring: nothing here is
// clamped or warm started, it is recomputed fresh from the current travel
// and closing speed every setup() the same way gravity is reapplied every
// step rather than carried over.
void WheelJoint::applySuspensionForce(f32 duration)
{
    if (mSuspensionStiffness <= 0.0f && mSuspensionDamping <= 0.0f)
        return;

    const f32 displacement = mSlidePosition - mSuspensionRestLength;
    const glm::vec3 armAPlusOffset = mArmA + mOffset;
    // dot(axisA, velA - velB) + ... is -(d mSlidePosition / dt) in this file's
    // convention (mSlidePosition follows velB - velA, see solveVelocity's
    // positionJv), so the restoring damping term needs a "+", not the naive
    // "-c*v": flipping it back to "-" reintroduces energy every step instead
    // of removing it and blows the spring up within a few dozen steps.
    const f32 velocityTowardChassis =
        glm::dot(mAxisA, mChassis->velocity() - mWheel->velocity()) +
        glm::dot(glm::cross(armAPlusOffset, mAxisA), mChassis->angularVelocity()) -
        glm::dot(glm::cross(mArmB, mAxisA), mWheel->angularVelocity());

    const f32 force = -mSuspensionStiffness * displacement + mSuspensionDamping * velocityTowardChassis;
    applyLinearImpulse(mAxisA * (force * duration));
}

void WheelJoint::setup(f32 duration)
{
    calculateArmsAndOffset();
    calculatePositionLockProperties();
    calculatePerpendicularityProperties();
    calculateAngles();
    calculateSteeringLimitProperties();
    calculateSteeringMotorProperties();
    calculateSpinMotorProperties();
    mSteeringMotorMaxImpulse = mSteeringMotorMaxTorque * duration;
    mSpinMotorMaxImpulse = mSpinMotorMaxTorque * duration;
    if (mPreviousDuration > 0.0f)
    {
        const f32 ratio = duration / mPreviousDuration;
        mTotalPositionLockImpulse *= ratio;
        mTotalPerpendicularImpulse *= ratio;
        mTotalSteeringLimitImpulse *= ratio;
        mTotalSteeringMotorImpulse *= ratio;
        mTotalSpinMotorImpulse *= ratio;
    }
    else
    {
        mTotalPositionLockImpulse = glm::vec2(0.0f);
        mTotalPerpendicularImpulse = 0.0f;
        mTotalSteeringLimitImpulse = 0.0f;
        mTotalSteeringMotorImpulse = 0.0f;
        mTotalSpinMotorImpulse = 0.0f;
    }
    mTotalSteeringMotorImpulse =
        glm::clamp(mTotalSteeringMotorImpulse, -mSteeringMotorMaxImpulse, mSteeringMotorMaxImpulse);
    mTotalSpinMotorImpulse = glm::clamp(mTotalSpinMotorImpulse, -mSpinMotorMaxImpulse, mSpinMotorMaxImpulse);
    mPreviousDuration = duration;

    applySuspensionForce(duration);
}

void WheelJoint::warmStart()
{
    if (mSteeringMotorEnabled)
        applyAngularImpulse(mAxisA * mTotalSteeringMotorImpulse);
    if (mSpinMotorEnabled)
        applyAngularImpulse(mAxisB * mTotalSpinMotorImpulse);
    applyLinearImpulse(mN1 * mTotalPositionLockImpulse.x + mN2 * mTotalPositionLockImpulse.y);
    applyAngularImpulse(mPerpendicularAxis * mTotalPerpendicularImpulse);
    if (mSteeringLimitActive)
        applyAngularImpulse(mAxisA * mTotalSteeringLimitImpulse);
}

void WheelJoint::solveVelocity()
{
    if (mSteeringMotorEnabled)
    {
        const f32 relative = glm::dot(mAxisA, mChassis->angularVelocity() - mWheel->angularVelocity());
        const f32 impulse = (relative + mSteeringMotorTargetVelocity) * mSteeringMotorEffectiveMass;
        const f32 previous = mTotalSteeringMotorImpulse;
        mTotalSteeringMotorImpulse =
            glm::clamp(previous + impulse, -mSteeringMotorMaxImpulse, mSteeringMotorMaxImpulse);
        applyAngularImpulse(mAxisA * (mTotalSteeringMotorImpulse - previous));
    }

    if (mSpinMotorEnabled)
    {
        // chassis - wheel, matching applyAngularImpulse's sign convention
        // (subtracts from chassis, adds to wheel) - using wheel - chassis
        // here turns the servo into positive feedback instead of driving the
        // relative velocity to the target.
        const f32 relative = glm::dot(mAxisB, mChassis->angularVelocity() - mWheel->angularVelocity());
        const f32 impulse = (relative + mSpinMotorTargetVelocity) * mSpinMotorEffectiveMass;
        const f32 previous = mTotalSpinMotorImpulse;
        mTotalSpinMotorImpulse = glm::clamp(previous + impulse, -mSpinMotorMaxImpulse, mSpinMotorMaxImpulse);
        applyAngularImpulse(mAxisB * (mTotalSpinMotorImpulse - previous));
    }

    const glm::vec3 armAPlusOffset = mArmA + mOffset;
    const glm::vec3 deltaLinear = mChassis->velocity() - mWheel->velocity();
    glm::vec2 positionJv;
    positionJv.x = glm::dot(mN1, deltaLinear) +
                  glm::dot(glm::cross(armAPlusOffset, mN1), mChassis->angularVelocity()) -
                  glm::dot(glm::cross(mArmB, mN1), mWheel->angularVelocity());
    positionJv.y = glm::dot(mN2, deltaLinear) +
                  glm::dot(glm::cross(armAPlusOffset, mN2), mChassis->angularVelocity()) -
                  glm::dot(glm::cross(mArmB, mN2), mWheel->angularVelocity());
    const glm::vec2 positionImpulse = mPositionLockEffectiveMass * positionJv;
    mTotalPositionLockImpulse += positionImpulse;
    applyLinearImpulse(mN1 * positionImpulse.x + mN2 * positionImpulse.y);

    const f32 perpJv = glm::dot(mPerpendicularAxis, mChassis->angularVelocity() - mWheel->angularVelocity());
    const f32 perpImpulse = mPerpendicularEffectiveMass * perpJv;
    mTotalPerpendicularImpulse += perpImpulse;
    applyAngularImpulse(mPerpendicularAxis * perpImpulse);

    if (mSteeringLimitActive)
    {
        f32 minImpulse = -M_INFINITY;
        f32 maxImpulse = M_INFINITY;
        if (mSteeringLimitsMin != mSteeringLimitsMax)
        {
            const f32 distanceToMin = centerAngleAroundZero(mSteeringAngle - mSteeringLimitsMin);
            const f32 distanceToMax = centerAngleAroundZero(mSteeringAngle - mSteeringLimitsMax);
            if (std::abs(distanceToMin) < std::abs(distanceToMax))
                minImpulse = 0.0f;
            else
                maxImpulse = 0.0f;
        }
        const f32 relative = glm::dot(mAxisA, mChassis->angularVelocity() - mWheel->angularVelocity());
        const f32 impulse = mSteeringLimitEffectiveMass * relative;
        const f32 previous = mTotalSteeringLimitImpulse;
        mTotalSteeringLimitImpulse = glm::clamp(previous + impulse, minImpulse, maxImpulse);
        applyAngularImpulse(mAxisA * (mTotalSteeringLimitImpulse - previous));
    }
}

void WheelJoint::solvePosition(f32 baumgarte)
{
    calculateArmsAndOffset();
    calculatePositionLockProperties();
    const glm::vec2 c(glm::dot(mOffset, mN1), glm::dot(mOffset, mN2));
    if (c != glm::vec2(0.0f))
    {
        const glm::vec2 lambda = -baumgarte * (mPositionLockEffectiveMass * c);
        const glm::vec3 impulse = mN1 * lambda.x + mN2 * lambda.y;
        const glm::vec3 armAPlusOffset = mArmA + mOffset;
        mChassis->applyPositionImpulseAtPoint(-impulse, mChassis->position() + armAPlusOffset);
        mWheel->applyPositionImpulseAtPoint(impulse, mWheel->position() + mArmB);
    }

    calculatePerpendicularityProperties();
    if (mPerpendicularity != 0.0f)
    {
        const f32 lambda = -mPerpendicularEffectiveMass * baumgarte * mPerpendicularity;
        if (mChassis->isDynamic())
        {
            const glm::vec3 step = mChassis->inverseInertiaTensorWorld() * mPerpendicularAxis * -lambda;
            const glm::quat spin(0.0f, step);
            mChassis->setOrientation(mChassis->orientation() + 0.5f * spin * mChassis->orientation());
        }
        if (mWheel->isDynamic())
        {
            const glm::vec3 step = mWheel->inverseInertiaTensorWorld() * mPerpendicularAxis * lambda;
            const glm::quat spin(0.0f, step);
            mWheel->setOrientation(mWheel->orientation() + 0.5f * spin * mWheel->orientation());
        }
    }

    if (mHasSteeringLimits)
    {
        calculateArmsAndOffset();
        calculateAngles();
        calculateSteeringLimitProperties();
        if (mSteeringLimitActive)
        {
            const f32 distanceToMin = centerAngleAroundZero(mSteeringAngle - mSteeringLimitsMin);
            const f32 distanceToMax = centerAngleAroundZero(mSteeringAngle - mSteeringLimitsMax);
            const f32 error = std::abs(distanceToMin) < std::abs(distanceToMax) ? distanceToMin : distanceToMax;
            const f32 lambda = -mSteeringLimitEffectiveMass * baumgarte * error;
            if (mChassis->isDynamic())
            {
                const glm::vec3 step = mChassis->inverseInertiaTensorWorld() * mAxisA * -lambda;
                const glm::quat spin(0.0f, step);
                mChassis->setOrientation(mChassis->orientation() + 0.5f * spin * mChassis->orientation());
            }
            if (mWheel->isDynamic())
            {
                const glm::vec3 step = mWheel->inverseInertiaTensorWorld() * mAxisA * lambda;
                const glm::quat spin(0.0f, step);
                mWheel->setOrientation(mWheel->orientation() + 0.5f * spin * mWheel->orientation());
            }
        }
    }
}

} // namespace Radion::Physics
