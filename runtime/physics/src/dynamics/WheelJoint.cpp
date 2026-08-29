
#include "PCH.h"

#include "dynamics/WheelJoint.h"

#include "GameObject.h"
#include "JointAxis.h"
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

} // namespace

WheelJoint::WheelJoint(RigidBody& chassis, RigidBody& wheel, const glm::vec3& worldAnchor,
                       const glm::vec3& worldSuspensionAxis, const glm::vec3& worldSpinAxis)
    : Joint(JointKind::Wheel)
{
    configure(chassis, wheel, worldAnchor, worldSuspensionAxis, worldSpinAxis);
}

WheelJoint::WheelJoint() : Joint(JointKind::Wheel)
{
}

void WheelJoint::configure(RigidBody& chassis, RigidBody& wheel, const glm::vec3& worldAnchor,
                           const glm::vec3& worldSuspensionAxis, const glm::vec3& worldSpinAxis)
{
    const glm::vec3 suspension =
        detail::normalizedAxisOr(worldSuspensionAxis, glm::vec3(0.0f, -1.0f, 0.0f));
    mChassis = &chassis;
    mWheel = &wheel;
    mLocalAnchorChassis = chassis.pointToLocal(worldAnchor);
    mLocalAnchorWheel = wheel.pointToLocal(worldAnchor);
    mLocalSuspensionAxis = chassis.directionToLocal(suspension);
    mLocalSpinAxis = wheel.directionToLocal(
        detail::normalizedAxisOr(worldSpinAxis, glm::vec3(1.0f, 0.0f, 0.0f)));
    mLocalNormalAxis = chassis.directionToLocal(normalizedPerpendicular(suspension));
    mInverseInitialOrientation = glm::conjugate(wheel.orientation()) * chassis.orientation();
}

void WheelJoint::rebuild()
{
    GameObject* self = owner();
    GameObject* other = connectedBody();
    if (!self || !other)
        return;
    RigidBody* wheelBody = self->getComponent<RigidBody>();
    RigidBody* chassisBody = other->getComponent<RigidBody>();
    if (!wheelBody || !chassisBody)
        return;
    // The owner is the wheel and the connected body the chassis: a car is
    // authored as four wheel objects hanging off one chassis, so the joint
    // lives on the part there are several of.
    Scene* scene = self->scene();
    if (!scene)
        return;
    const glm::vec3 suspensionAxis = self->globalRotation() * glm::normalize(mAuthoredSuspensionAxis);
    const glm::vec3 spinAxis = self->globalRotation() * glm::normalize(mAuthoredSpinAxis);
    configure(*chassisBody, *wheelBody, self->globalPosition(), suspensionAxis, spinAxis);
    scene->addJoint(this);
    mBuilt = true;
}

// Both only record the axis and drop the built flag, the way
// HingeJoint::setAuthoredAxis() does - the Scene rebuilds an unbuilt joint
// on its own. Rebuilding here would run before the object is even in a
// scene, which is where owner()->scene() is still null.
void WheelJoint::setAuthoredSuspensionAxis(const glm::vec3& axis)
{
    if (glm::length(axis) <= 1.0e-6f)
        return;
    mAuthoredSuspensionAxis = glm::normalize(axis);
    mBuilt = false;
}

void WheelJoint::setAuthoredSpinAxis(const glm::vec3& axis)
{
    if (glm::length(axis) <= 1.0e-6f)
        return;
    mAuthoredSpinAxis = glm::normalize(axis);
    mBuilt = false;
}

RigidBody* WheelJoint::bodyA() const
{
    return mChassis;
}

RigidBody* WheelJoint::bodyB() const
{
    return mWheel;
}

glm::vec3 WheelJoint::anchorWorldA() const
{
    return mChassis->pointToWorld(mLocalAnchorChassis);
}

glm::vec3 WheelJoint::anchorWorldB() const
{
    return mWheel->pointToWorld(mLocalAnchorWheel);
}

glm::vec3 WheelJoint::axisWorld() const
{
    return mChassis->directionToWorld(mLocalSuspensionAxis);
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
    mSteeringServoEnabled = false;
    mTotalSteeringMotorImpulse = 0.0f;
}

void WheelJoint::setSteeringServo(f32 targetAngle, f32 maxTorque, f32 maxAngularVelocity)
{
    if (!std::isfinite(targetAngle) || !std::isfinite(maxTorque) ||
        !std::isfinite(maxAngularVelocity))
        return;
    mSteeringServoTargetAngle = targetAngle;
    mSteeringServoMaxAngularVelocity = glm::max(maxAngularVelocity, 0.0f);
    mSteeringMotorMaxTorque = glm::max(maxTorque, 0.0f);
    mSteeringServoEnabled = mSteeringMotorMaxTorque > 0.0f;
    mSteeringMotorEnabled = mSteeringServoEnabled;
}

void WheelJoint::disableSteeringServo()
{
    mSteeringServoEnabled = false;
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
void WheelJoint::calculateSuspensionProperties(f32 duration)
{
    if (mSuspensionStiffness <= 0.0f && mSuspensionDamping <= 0.0f)
    {
        mSuspensionEffectiveMass = 0.0f;
        mTotalSuspensionImpulse = 0.0f;
        return;
    }

    // The row is the suspension axis itself, with the same arms the two
    // perpendicular rows use (calculatePositionLockProperties()).
    const glm::vec3 armAPlusOffset = mArmA + mOffset;
    const glm::vec3 r1 = glm::cross(armAPlusOffset, mAxisA);
    const glm::vec3 r2 = glm::cross(mArmB, mAxisA);
    const f32 inverseEffectiveMass = mChassis->inverseMass() + mWheel->inverseMass() +
                                     glm::dot(r1, mChassis->inverseInertiaTensorWorld() * r1) +
                                     glm::dot(r2, mWheel->inverseInertiaTensorWorld() * r2);
    if (inverseEffectiveMass <= 1.0e-9f)
    {
        mSuspensionEffectiveMass = 0.0f;
        mTotalSuspensionImpulse = 0.0f;
        return;
    }

    // C is how far the strut sits from where the spring wants it. Compressed
    // (slide below rest) is negative, and solveVelocity subtracts the bias,
    // so a compressed strut pushes the wheel away along mAxisA - which is
    // the direction that axis points, chassis towards ground.
    const f32 positionError = mSlidePosition - mSuspensionRestLength;
    mSuspensionSpring.calculate(duration, inverseEffectiveMass, 0.0f, positionError,
                                mSuspensionStiffness, mSuspensionDamping, mSuspensionEffectiveMass);
}

void WheelJoint::setup(f32 duration)
{
    calculateArmsAndOffset();
    calculatePositionLockProperties();
    calculatePerpendicularityProperties();
    calculateAngles();
    // The steering servo feeds the motor below, so it runs before the motor's
    // properties are worked out - and after calculateAngles(), which is what
    // refreshes the steering angle the error is measured from. Clamped into
    // the steering limits: a rack cannot be commanded past its own stops.
    if (mSteeringServoEnabled && duration > 0.0f)
    {
        f32 target = mSteeringServoTargetAngle;
        if (mHasSteeringLimits)
            target = glm::clamp(target, mSteeringLimitsMin, mSteeringLimitsMax);
        f32 velocity = (target - mSteeringAngle) / duration;
        if (mSteeringServoMaxAngularVelocity > 0.0f)
            velocity = glm::clamp(velocity, -mSteeringServoMaxAngularVelocity,
                                  mSteeringServoMaxAngularVelocity);
        mSteeringMotorTargetVelocity = velocity;
    }
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

    calculateSuspensionProperties(duration);
}

void WheelJoint::warmStart()
{
    if (mSteeringMotorEnabled)
        applyAngularImpulse(mAxisA * mTotalSteeringMotorImpulse);
    if (mSpinMotorEnabled)
        applyAngularImpulse(mAxisB * mTotalSpinMotorImpulse);
    applyLinearImpulse(mN1 * mTotalPositionLockImpulse.x + mN2 * mTotalPositionLockImpulse.y);
    if (mSuspensionEffectiveMass > 0.0f)
        applyLinearImpulse(mAxisA * mTotalSuspensionImpulse);
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

    // The suspension is the third row of the same position constraint - the
    // one along the axis - left soft instead of locked.
    if (mSuspensionEffectiveMass > 0.0f)
    {
        const f32 suspensionJv =
            glm::dot(mAxisA, deltaLinear) +
            glm::dot(glm::cross(armAPlusOffset, mAxisA), mChassis->angularVelocity()) -
            glm::dot(glm::cross(mArmB, mAxisA), mWheel->angularVelocity());
        // The rows above solve lambda = +mass * Jv, with Jv measured
        // chassis - wheel; the spring's bias is written for the usual
        // lambda = -mass * (Jv + bias), so here it subtracts.
        const f32 impulse = mSuspensionEffectiveMass *
                            (suspensionJv - mSuspensionSpring.bias(mTotalSuspensionImpulse));
        mTotalSuspensionImpulse += impulse;
        applyLinearImpulse(mAxisA * impulse);
    }

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
