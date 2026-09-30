#include "PCH.h"

#include "dynamics/MotorcycleController.h"

#include "dynamics/RaycastVehicle.h"
#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

namespace
{

const Math::vec3 kLocalUp(0.0f, 1.0f, 0.0f);
const Math::vec3 kLocalForward(0.0f, 0.0f, 1.0f);

Math::vec3 normalizedOr(const Math::vec3& value, const Math::vec3& fallback)
{
    const f32 lengthSquared = Math::dot(value, value);
    return lengthSquared > 1.0e-12f ? value / std::sqrt(lengthSquared) : fallback;
}

f32 sign(f32 value)
{
    return value < 0.0f ? -1.0f : 1.0f;
}

f32 signedAngleAroundForward(const Math::vec3& from, const Math::vec3& to, const Math::vec3& forward)
{
    const f32 d = Math::clamp(Math::dot(from, to), -1.0f, 1.0f);
    return -sign(Math::dot(Math::cross(from, to), forward)) * std::acos(d);
}

} // namespace

MotorcycleController::MotorcycleController(RaycastVehicle& vehicle, RigidBody& chassis)
    : mVehicle(vehicle), mChassis(chassis)
{
}

void MotorcycleController::setSteerInput(f32 input)
{
    if (std::isfinite(input))
        mSteerInput = Math::clamp(input, -1.0f, 1.0f);
}

void MotorcycleController::setMaxSteerAngle(f32 angle)
{
    mMaxSteerAngle = angle;
}

void MotorcycleController::setCasterAngle(f32 angle)
{
    mCasterAngle = angle;
}

void MotorcycleController::setMaxLeanAngle(f32 angle)
{
    mMaxLeanAngle = angle;
}

void MotorcycleController::setLeanSpring(f32 springConstant, f32 damping)
{
    mLeanSpringConstant = springConstant;
    mLeanSpringDamping = damping;
}

void MotorcycleController::setLeanSpringIntegration(f32 coefficient, f32 decay)
{
    mLeanSpringIntegrationCoefficient = coefficient;
    mLeanSpringIntegrationCoefficientDecay = decay;
}

void MotorcycleController::setLeanSmoothingFactor(f32 factor)
{
    mLeanSmoothingFactor = Math::clamp(factor, 0.0f, 1.0f);
}

void MotorcycleController::setLeanControllerEnabled(bool enabled)
{
    mEnableLeanController = enabled;
}

bool MotorcycleController::leanControllerEnabled() const
{
    return mEnableLeanController;
}

f32 MotorcycleController::currentLeanAngle() const
{
    const Math::vec3 forward = mChassis.directionToWorld(kLocalForward);
    const Math::vec3 up = mChassis.directionToWorld(kLocalUp);
    Math::vec3 flatUp = Math::vec3(0.0f, 1.0f, 0.0f) - forward * forward.y;
    flatUp = normalizedOr(flatUp, Math::vec3(0.0f, 1.0f, 0.0f));
    return signedAngleAroundForward(up, flatUp, forward);
}

f32 MotorcycleController::wheelBase() const
{
    f32 low = std::numeric_limits<f32>::max();
    f32 high = -std::numeric_limits<f32>::max();
    for (u32 i = 0; i < mVehicle.wheelCount(); ++i)
    {
        const RaycastVehicle::Wheel& wheel = mVehicle.wheel(i);
        const Math::vec3 fullyExtended =
            wheel.chassisConnectionLocal + wheel.directionLocal * wheel.restLength;
        const f32 value = Math::dot(fullyExtended, kLocalForward);
        low = Math::min(low, value);
        high = Math::max(high, value);
    }
    return high - low;
}

void MotorcycleController::preUpdate(f32 step, const Math::vec3& gravity)
{
    const Math::vec3 forward = mChassis.directionToWorld(kLocalForward);
    const Math::vec3 worldUp(0.0f, 1.0f, 0.0f);

    if (mEnableLeanController)
    {
        // Target lean follows the total impulse the ground applied to the
        // wheels last step: supported weight plus cornering force, which is
        // exactly the direction a rider balances against.
        Math::vec3 targetLean(0.0f);
        for (u32 i = 0; i < mVehicle.wheelCount(); ++i)
        {
            const RaycastVehicle::Wheel& wheel = mVehicle.wheel(i);
            if (wheel.inContact)
                targetLean += wheel.contactNormal * wheel.appliedSuspensionImpulse +
                              wheel.lateralWorld * wheel.appliedSideImpulse;
        }
        targetLean = normalizedOr(targetLean, worldUp);

        mTargetLean = mLeanSmoothingFactor * mTargetLean +
                      (1.0f - mLeanSmoothingFactor) * targetLean;

        mTargetLean -= forward * Math::dot(mTargetLean, forward);
        mTargetLean = normalizedOr(mTargetLean, worldUp);

        Math::vec3 adjustedWorldUp = worldUp - forward * Math::dot(worldUp, forward);
        adjustedWorldUp = normalizedOr(adjustedWorldUp, worldUp);
        const f32 leanAngle = signedAngleAroundForward(mTargetLean, adjustedWorldUp, forward);
        if (std::abs(leanAngle) > mMaxLeanAngle)
            mTargetLean = Math::angleAxis(sign(leanAngle) * mMaxLeanAngle, forward) *
                          adjustedWorldUp;

        const Math::vec3 up = mChassis.directionToWorld(kLocalUp);
        const f32 deltaAngle = signedAngleAroundForward(mTargetLean, up, forward);
        mLeanSpringIntegratedDeltaAngle += deltaAngle * step;
    }
    else
    {
        mTargetLean = worldUp;
        mLeanSpringIntegratedDeltaAngle = 0.0f;
    }

    const f32 maxSteerAngleFactor =
        wheelBase() * std::tan(mMaxLeanAngle) * Math::length(gravity);

    const f32 velocity = Math::dot(mChassis.velocity(), forward);
    const f32 velocitySquared = velocity * velocity;

    const f32 steerStrength = std::abs(mSteerInput);
    const f32 steerSign = -sign(mSteerInput);

    const f32 cosCasterAngle = std::cos(mCasterAngle);
    f32 steerAngle = steerStrength * mMaxSteerAngle;
    if (velocitySquared > 1.0e-6f && cosCasterAngle > 1.0e-6f)
    {
        const f32 asinArgument =
            maxSteerAngleFactor / (velocitySquared * cosCasterAngle);
        if (asinArgument < 1.0f)
            steerAngle = Math::min(steerAngle, std::asin(asinArgument));
    }

    for (u32 i = 0; i < mVehicle.wheelCount(); ++i)
        if (mVehicle.wheel(i).isFrontWheel)
            mVehicle.setSteering(steerSign * steerAngle, i);
}

void MotorcycleController::postUpdate(f32 step)
{
    if (!mEnableLeanController)
        return;

    bool allInContact = true;
    for (u32 i = 0; i < mVehicle.wheelCount(); ++i)
    {
        const RaycastVehicle::Wheel& wheel = mVehicle.wheel(i);
        if (!wheel.inContact || wheel.appliedSuspensionImpulse <= 0.0f)
        {
            allInContact = false;
            break;
        }
    }

    if (!allInContact)
    {
        mLeanSpringIntegratedDeltaAngle *=
            Math::max(0.0f, 1.0f - mLeanSpringIntegrationCoefficientDecay * step);
        return;
    }

    const Math::vec3 forward = mChassis.directionToWorld(kLocalForward);
    const Math::vec3 up = mChassis.directionToWorld(kLocalUp);

    const f32 deltaAngle = signedAngleAroundForward(mTargetLean, up, forward);
    const f32 angleRate = Math::dot(mChassis.angularVelocity(), forward);

    const f32 totalImpulse = (mLeanSpringConstant * deltaAngle - mLeanSpringDamping * angleRate +
                              mLeanSpringIntegrationCoefficient *
                                  mLeanSpringIntegratedDeltaAngle) *
                             step;

    const Math::vec3 oldAngularVelocity = mChassis.angularVelocity();
    mChassis.applyAngularImpulse(totalImpulse * forward);

    // The angular impulse alone drags every contact point sideways; a linear
    // impulse on the centre of mass cancels the average of that so the lean
    // torque rolls the bike instead of pushing it off its line.
    const Math::vec3 deltaAngularVelocity = mChassis.angularVelocity() - oldAngularVelocity;
    Math::vec3 linearAcceleration(0.0f);
    f32 totalLambda = 0.0f;
    for (u32 i = 0; i < mVehicle.wheelCount(); ++i)
    {
        const RaycastVehicle::Wheel& wheel = mVehicle.wheel(i);
        const f32 lambda = wheel.appliedSuspensionImpulse;
        totalLambda += lambda;
        const Math::vec3 r = wheel.contactPoint - mChassis.position();
        linearAcceleration += lambda * Math::cross(deltaAngularVelocity, r);
    }
    if (totalLambda > 0.0f && mChassis.inverseMass() > 0.0f)
    {
        const Math::vec3 linearImpulse =
            -linearAcceleration / (totalLambda * mChassis.inverseMass());
        mChassis.applyLinearImpulse(linearImpulse);
    }
}

}
