
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

} // namespace

WheelJoint::WheelJoint(RigidBody& chassis, RigidBody& wheel, const Math::vec3& worldAnchor,
                       const Math::vec3& worldSuspensionAxis, const Math::vec3& worldSpinAxis)
    : Joint(JointKind::Wheel)
{
    configure(chassis, wheel, worldAnchor, worldSuspensionAxis, worldSpinAxis);
}

WheelJoint::WheelJoint() : Joint(JointKind::Wheel)
{
}

void WheelJoint::configure(RigidBody& chassis, RigidBody& wheel, const Math::vec3& worldAnchor,
                           const Math::vec3& worldSuspensionAxis, const Math::vec3& worldSpinAxis)
{
    const Math::vec3 suspension =
        detail::normalizedAxisOr(worldSuspensionAxis, Math::vec3(0.0f, -1.0f, 0.0f));
    mChassis = &chassis;
    mWheel = &wheel;
    mLocalAnchorChassis = chassis.pointToLocal(worldAnchor);
    mLocalAnchorWheel = wheel.pointToLocal(worldAnchor);
    mLocalSuspensionAxis = chassis.directionToLocal(suspension);
    mLocalSpinAxis = wheel.directionToLocal(
        detail::normalizedAxisOr(worldSpinAxis, Math::vec3(1.0f, 0.0f, 0.0f)));
    mLocalNormalAxis = chassis.directionToLocal(normalizedPerpendicular(suspension));
    mInverseInitialOrientation = Math::conjugate(wheel.orientation()) * chassis.orientation();
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
    const Math::vec3 suspensionAxis = self->globalRotation() * Math::normalize(mAuthoredSuspensionAxis);
    const Math::vec3 spinAxis = self->globalRotation() * Math::normalize(mAuthoredSpinAxis);
    configure(*chassisBody, *wheelBody, self->globalPosition(), suspensionAxis, spinAxis);
    scene->addJoint(this);
    mBuilt = true;
}

// Both only record the axis and drop the built flag, the way
// HingeJoint::setAuthoredAxis() does - the Scene rebuilds an unbuilt joint
// on its own. Rebuilding here would run before the object is even in a
// scene, which is where owner()->scene() is still null.
void WheelJoint::setAuthoredSuspensionAxis(const Math::vec3& axis)
{
    if (Math::length(axis) <= 1.0e-6f)
        return;
    mAuthoredSuspensionAxis = Math::normalize(axis);
    mBuilt = false;
}

void WheelJoint::setAuthoredSpinAxis(const Math::vec3& axis)
{
    if (Math::length(axis) <= 1.0e-6f)
        return;
    mAuthoredSpinAxis = Math::normalize(axis);
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

Math::vec3 WheelJoint::anchorWorldA() const
{
    return mChassis->pointToWorld(mLocalAnchorChassis);
}

Math::vec3 WheelJoint::anchorWorldB() const
{
    return mWheel->pointToWorld(mLocalAnchorWheel);
}

Math::vec3 WheelJoint::axisWorld() const
{
    return mChassis->directionToWorld(mLocalSuspensionAxis);
}

void WheelJoint::setSteeringLimits(f32 minAngle, f32 maxAngle)
{
    mSteeringLimitsMin = Math::clamp(minAngle, -Math::pi<f32>(), 0.0f);
    mSteeringLimitsMax = Math::clamp(maxAngle, 0.0f, Math::pi<f32>());
    mHasSteeringLimits = mSteeringLimitsMin > -Math::pi<f32>() || mSteeringLimitsMax < Math::pi<f32>();
}

void WheelJoint::setSteeringMotor(f32 targetAngularVelocity, f32 maxTorque)
{
    if (!std::isfinite(targetAngularVelocity) || !std::isfinite(maxTorque))
        return;
    mSteeringMotorTargetVelocity = targetAngularVelocity;
    mSteeringMotorMaxTorque = Math::max(maxTorque, 0.0f);
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
    // See HingeJoint::setServo(): a new target on a sleeping wheel is an
    // order the solver never steps.
    if (targetAngle != mSteeringServoTargetAngle || !mSteeringServoEnabled)
        wakeBodies();
    mSteeringServoTargetAngle = targetAngle;
    mSteeringServoMaxAngularVelocity = Math::max(maxAngularVelocity, 0.0f);
    mSteeringMotorMaxTorque = Math::max(maxTorque, 0.0f);
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
    if (targetAngularVelocity != mSpinMotorTargetVelocity || !mSpinMotorEnabled)
        wakeBodies();
    mSpinMotorTargetVelocity = targetAngularVelocity;
    mSpinMotorMaxTorque = Math::max(maxTorque, 0.0f);
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
    return Math::dot(mAxisB, mWheel->angularVelocity() - mChassis->angularVelocity());
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
    mN2 = Math::cross(mAxisA, mN1);

    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    const Math::vec3 r1x1 = Math::cross(armAPlusOffset, mN1);
    const Math::vec3 r1x2 = Math::cross(armAPlusOffset, mN2);
    const Math::vec3 r2x1 = Math::cross(mArmB, mN1);
    const Math::vec3 r2x2 = Math::cross(mArmB, mN2);

    const f32 inverseMassSum = mChassis->inverseMass() + mWheel->inverseMass();
    Math::mat2 inverseEffectiveMass(0.0f);
    inverseEffectiveMass[0][0] = inverseMassSum + Math::dot(r1x1, mChassis->inverseInertiaTensorWorld() * r1x1) +
                                 Math::dot(r2x1, mWheel->inverseInertiaTensorWorld() * r2x1);
    inverseEffectiveMass[0][1] = Math::dot(r1x1, mChassis->inverseInertiaTensorWorld() * r1x2) +
                                 Math::dot(r2x1, mWheel->inverseInertiaTensorWorld() * r2x2);
    inverseEffectiveMass[1][0] = Math::dot(r1x2, mChassis->inverseInertiaTensorWorld() * r1x1) +
                                 Math::dot(r2x2, mWheel->inverseInertiaTensorWorld() * r2x1);
    inverseEffectiveMass[1][1] = inverseMassSum + Math::dot(r1x2, mChassis->inverseInertiaTensorWorld() * r1x2) +
                                 Math::dot(r2x2, mWheel->inverseInertiaTensorWorld() * r2x2);

    const f32 determinant = Math::determinant(inverseEffectiveMass);
    if (std::abs(determinant) > 1.0e-9f && std::isfinite(determinant))
        mPositionLockEffectiveMass = Math::inverse(inverseEffectiveMass);
    else
    {
        mPositionLockEffectiveMass = Math::mat2(0.0f);
        mTotalPositionLockImpulse = Math::vec2(0.0f);
    }
}

void WheelJoint::calculatePerpendicularityProperties()
{
    mAxisA = Math::normalize(mChassis->directionToWorld(mLocalSuspensionAxis));
    mAxisB = Math::normalize(mWheel->directionToWorld(mLocalSpinAxis));

    const f32 k = Math::dot(mAxisA, mAxisB);
    const Math::vec3 axisBPerpendicular = mAxisB - k * mAxisA;
    const f32 length = Math::length(axisBPerpendicular);
    mPerpendicularAxis = length > 1.0e-6f ? Math::normalize(Math::cross(mAxisA, axisBPerpendicular))
                                          : normalizedPerpendicular(mAxisA);
    mPerpendicularity = k;

    const f32 inverseEffectiveMass = Math::dot(
        mPerpendicularAxis, mChassis->inverseInertiaTensorWorld() * mPerpendicularAxis +
                                mWheel->inverseInertiaTensorWorld() * mPerpendicularAxis);
    mPerpendicularEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
    if (mPerpendicularEffectiveMass == 0.0f)
        mTotalPerpendicularImpulse = 0.0f;
}

void WheelJoint::calculateAngles()
{
    mSlidePosition = Math::dot(mOffset, mAxisA);
    const Math::quat diff = mWheel->orientation() * mInverseInitialOrientation *
                           Math::conjugate(mChassis->orientation());
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
    const f32 inverseEffectiveMass = Math::dot(
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
    const f32 inverseEffectiveMass = Math::dot(
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
    const f32 inverseEffectiveMass = Math::dot(
        mAxisB, mChassis->inverseInertiaTensorWorld() * mAxisB + mWheel->inverseInertiaTensorWorld() * mAxisB);
    mSpinMotorEffectiveMass = inverseEffectiveMass > 1.0e-9f ? 1.0f / inverseEffectiveMass : 0.0f;
}

void WheelJoint::applyLinearImpulse(const Math::vec3& impulse)
{
    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    if (mChassis->isDynamic())
    {
        mChassis->setVelocity(mChassis->velocity() - impulse * mChassis->inverseMass());
        mChassis->setAngularVelocity(
            mChassis->angularVelocity() -
            mChassis->inverseInertiaTensorWorld() * Math::cross(armAPlusOffset, impulse));
    }
    if (mWheel->isDynamic())
    {
        mWheel->setVelocity(mWheel->velocity() + impulse * mWheel->inverseMass());
        mWheel->setAngularVelocity(
            mWheel->angularVelocity() +
            mWheel->inverseInertiaTensorWorld() * Math::cross(mArmB, impulse));
    }
}

void WheelJoint::applyAngularImpulse(const Math::vec3& impulse)
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
    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    const Math::vec3 r1 = Math::cross(armAPlusOffset, mAxisA);
    const Math::vec3 r2 = Math::cross(mArmB, mAxisA);
    const f32 inverseEffectiveMass = mChassis->inverseMass() + mWheel->inverseMass() +
                                     Math::dot(r1, mChassis->inverseInertiaTensorWorld() * r1) +
                                     Math::dot(r2, mWheel->inverseInertiaTensorWorld() * r2);
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
            target = Math::clamp(target, mSteeringLimitsMin, mSteeringLimitsMax);
        const f32 error = target - mSteeringAngle;
        // See HingeJoint::setup().
        if (std::abs(error) > 0.001f)
            wakeBodies();
        f32 velocity = error / duration;
        if (mSteeringServoMaxAngularVelocity > 0.0f)
            velocity = Math::clamp(velocity, -mSteeringServoMaxAngularVelocity,
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
        mTotalPositionLockImpulse = Math::vec2(0.0f);
        mTotalPerpendicularImpulse = 0.0f;
        mTotalSteeringLimitImpulse = 0.0f;
        mTotalSteeringMotorImpulse = 0.0f;
        mTotalSpinMotorImpulse = 0.0f;
    }
    mTotalSteeringMotorImpulse =
        Math::clamp(mTotalSteeringMotorImpulse, -mSteeringMotorMaxImpulse, mSteeringMotorMaxImpulse);
    mTotalSpinMotorImpulse = Math::clamp(mTotalSpinMotorImpulse, -mSpinMotorMaxImpulse, mSpinMotorMaxImpulse);
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
        const f32 relative = Math::dot(mAxisA, mChassis->angularVelocity() - mWheel->angularVelocity());
        const f32 impulse = (relative + mSteeringMotorTargetVelocity) * mSteeringMotorEffectiveMass;
        const f32 previous = mTotalSteeringMotorImpulse;
        mTotalSteeringMotorImpulse =
            Math::clamp(previous + impulse, -mSteeringMotorMaxImpulse, mSteeringMotorMaxImpulse);
        applyAngularImpulse(mAxisA * (mTotalSteeringMotorImpulse - previous));
    }

    if (mSpinMotorEnabled)
    {
        // chassis - wheel, matching applyAngularImpulse's sign convention
        // (subtracts from chassis, adds to wheel) - using wheel - chassis
        // here turns the servo into positive feedback instead of driving the
        // relative velocity to the target.
        const f32 relative = Math::dot(mAxisB, mChassis->angularVelocity() - mWheel->angularVelocity());
        const f32 impulse = (relative + mSpinMotorTargetVelocity) * mSpinMotorEffectiveMass;
        const f32 previous = mTotalSpinMotorImpulse;
        mTotalSpinMotorImpulse = Math::clamp(previous + impulse, -mSpinMotorMaxImpulse, mSpinMotorMaxImpulse);
        applyAngularImpulse(mAxisB * (mTotalSpinMotorImpulse - previous));
    }

    const Math::vec3 armAPlusOffset = mArmA + mOffset;
    const Math::vec3 deltaLinear = mChassis->velocity() - mWheel->velocity();
    Math::vec2 positionJv;
    positionJv.x = Math::dot(mN1, deltaLinear) +
                  Math::dot(Math::cross(armAPlusOffset, mN1), mChassis->angularVelocity()) -
                  Math::dot(Math::cross(mArmB, mN1), mWheel->angularVelocity());
    positionJv.y = Math::dot(mN2, deltaLinear) +
                  Math::dot(Math::cross(armAPlusOffset, mN2), mChassis->angularVelocity()) -
                  Math::dot(Math::cross(mArmB, mN2), mWheel->angularVelocity());
    const Math::vec2 positionImpulse = mPositionLockEffectiveMass * positionJv;
    mTotalPositionLockImpulse += positionImpulse;
    applyLinearImpulse(mN1 * positionImpulse.x + mN2 * positionImpulse.y);

    // The suspension is the third row of the same position constraint - the
    // one along the axis - left soft instead of locked.
    if (mSuspensionEffectiveMass > 0.0f)
    {
        const f32 suspensionJv =
            Math::dot(mAxisA, deltaLinear) +
            Math::dot(Math::cross(armAPlusOffset, mAxisA), mChassis->angularVelocity()) -
            Math::dot(Math::cross(mArmB, mAxisA), mWheel->angularVelocity());
        // The rows above solve lambda = +mass * Jv, with Jv measured
        // chassis - wheel; the spring's bias is written for the usual
        // lambda = -mass * (Jv + bias), so here it subtracts.
        const f32 impulse = mSuspensionEffectiveMass *
                            (suspensionJv - mSuspensionSpring.bias(mTotalSuspensionImpulse));
        mTotalSuspensionImpulse += impulse;
        applyLinearImpulse(mAxisA * impulse);
    }

    const f32 perpJv = Math::dot(mPerpendicularAxis, mChassis->angularVelocity() - mWheel->angularVelocity());
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
        const f32 relative = Math::dot(mAxisA, mChassis->angularVelocity() - mWheel->angularVelocity());
        const f32 impulse = mSteeringLimitEffectiveMass * relative;
        const f32 previous = mTotalSteeringLimitImpulse;
        mTotalSteeringLimitImpulse = Math::clamp(previous + impulse, minImpulse, maxImpulse);
        applyAngularImpulse(mAxisA * (mTotalSteeringLimitImpulse - previous));
    }
}

void WheelJoint::solvePosition(f32 baumgarte)
{
    calculateArmsAndOffset();
    calculatePositionLockProperties();
    const Math::vec2 c(Math::dot(mOffset, mN1), Math::dot(mOffset, mN2));
    if (c != Math::vec2(0.0f))
    {
        const Math::vec2 lambda = -baumgarte * (mPositionLockEffectiveMass * c);
        const Math::vec3 impulse = mN1 * lambda.x + mN2 * lambda.y;
        const Math::vec3 armAPlusOffset = mArmA + mOffset;
        mChassis->applyPositionImpulseAtPoint(-impulse, mChassis->position() + armAPlusOffset);
        mWheel->applyPositionImpulseAtPoint(impulse, mWheel->position() + mArmB);
    }

    calculatePerpendicularityProperties();
    if (mPerpendicularity != 0.0f)
    {
        const f32 lambda = -mPerpendicularEffectiveMass * baumgarte * mPerpendicularity;
        if (mChassis->isDynamic())
        {
            const Math::vec3 step = mChassis->inverseInertiaTensorWorld() * mPerpendicularAxis * -lambda;
            const Math::quat spin(0.0f, step);
            mChassis->setOrientation(mChassis->orientation() + 0.5f * spin * mChassis->orientation());
        }
        if (mWheel->isDynamic())
        {
            const Math::vec3 step = mWheel->inverseInertiaTensorWorld() * mPerpendicularAxis * lambda;
            const Math::quat spin(0.0f, step);
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
                const Math::vec3 step = mChassis->inverseInertiaTensorWorld() * mAxisA * -lambda;
                const Math::quat spin(0.0f, step);
                mChassis->setOrientation(mChassis->orientation() + 0.5f * spin * mChassis->orientation());
            }
            if (mWheel->isDynamic())
            {
                const Math::vec3 step = mWheel->inverseInertiaTensorWorld() * mAxisA * lambda;
                const Math::quat spin(0.0f, step);
                mWheel->setOrientation(mWheel->orientation() + 0.5f * spin * mWheel->orientation());
            }
        }
    }
}

} // namespace Radion::Physics
