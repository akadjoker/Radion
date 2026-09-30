#include "PCH.h"

#include "dynamics/Aerodynamics.h"

#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

AeroSurface::AeroSurface(const Math::mat3& tensor, const Math::vec3& position)
    : mTensor(tensor)
    , mPosition(position)
{
}

void AeroSurface::applyTo(RigidBody& body, const Math::vec3& windspeed) const
{
    const Math::vec3 velocity = body.velocity() + windspeed;
    const Math::vec3 bodyVelocity = body.directionToLocal(velocity);
    const Math::vec3 bodyForce = activeTensor() * bodyVelocity;
    body.addForceAtBodyPoint(body.directionToWorld(bodyForce), mPosition);
}

AeroControlSurface::AeroControlSurface(const Math::mat3& base, const Math::mat3& minimum,
                                       const Math::mat3& maximum, const Math::vec3& position)
    : AeroSurface(base, position)
    , mMinimum(minimum)
    , mMaximum(maximum)
{
}

void AeroControlSurface::setControl(f32 control)
{
    mControl = Math::clamp(control, -1.0f, 1.0f);
}

Math::mat3 AeroControlSurface::activeTensor() const
{
    if (mControl <= -1.0f)
        return mMinimum;
    if (mControl >= 1.0f)
        return mMaximum;
    if (mControl < 0.0f)
        return mTensor + (mTensor - mMinimum) * mControl;
    if (mControl > 0.0f)
        return mTensor + (mMaximum - mTensor) * mControl;
    return mTensor;
}

Airplane::Airplane()
{
    setDefaultSurfaces();
}

void Airplane::setDefaultSurfaces()
{
    // Math::mat3 takes columns; the reference tensors are written row-major,
    // so each one below is transposed. The wings' -1 at row 1, column 0 maps
    // forward velocity onto upward force.
    const Math::mat3 wingBase = Math::transpose(Math::mat3(0.0f, 0.0f, 0.0f,
                                                        -1.0f, -0.5f, 0.0f,
                                                        0.0f, 0.0f, 0.0f));
    const Math::mat3 wingMin = Math::transpose(Math::mat3(0.0f, 0.0f, 0.0f,
                                                       -0.995f, -0.5f, 0.0f,
                                                       0.0f, 0.0f, 0.0f));
    const Math::mat3 wingMax = Math::transpose(Math::mat3(0.0f, 0.0f, 0.0f,
                                                       -1.005f, -0.5f, 0.0f,
                                                       0.0f, 0.0f, 0.0f));

    mRightWing = AeroControlSurface(wingBase, wingMin, wingMax, Math::vec3(-1.0f, 0.0f, 2.0f));
    mLeftWing = AeroControlSurface(wingBase, wingMin, wingMax, Math::vec3(-1.0f, 0.0f, -2.0f));

    const Math::mat3 rudderBase(0.0f);
    const Math::mat3 rudderMin = Math::transpose(Math::mat3(0.0f, 0.0f, 0.0f,
                                                         0.0f, 0.0f, 0.0f,
                                                         0.01f, 0.0f, 0.0f));
    const Math::mat3 rudderMax = Math::transpose(Math::mat3(0.0f, 0.0f, 0.0f,
                                                         0.0f, 0.0f, 0.0f,
                                                         -0.01f, 0.0f, 0.0f));
    mRudder = AeroControlSurface(rudderBase, rudderMin, rudderMax, Math::vec3(2.0f, 0.5f, 0.0f));

    const Math::mat3 tailTensor = Math::transpose(Math::mat3(0.0f, 0.0f, 0.0f,
                                                          -1.0f, -0.5f, 0.0f,
                                                          0.0f, 0.0f, -0.1f));
    mTail = AeroSurface(tailTensor, Math::vec3(2.0f, 0.0f, 0.0f));
}

void Airplane::setRoll(f32 control)
{
    mRoll = Math::clamp(control, -1.0f, 1.0f);
    mLeftWing.setControl(mRoll);
    mRightWing.setControl(-mRoll);
}

void Airplane::setYaw(f32 control)
{
    mYaw = Math::clamp(control, -1.0f, 1.0f);
    mRudder.setControl(mYaw);
}

void Airplane::setThrust(f32 thrust)
{
    mThrust = Math::max(thrust, 0.0f);
}

void Airplane::applyForces()
{
    if (!mBody)
        return;

    mBody->addForce(mBody->directionToWorld(Math::vec3(-mThrust, 0.0f, 0.0f)));

    mLeftWing.applyTo(*mBody, mWindspeed);
    mRightWing.applyTo(*mBody, mWindspeed);
    mRudder.applyTo(*mBody, mWindspeed);
    mTail.applyTo(*mBody, mWindspeed);
}

f32 Airplane::airspeed() const
{
    if (!mBody)
        return 0.0f;
    return Math::length(mBody->velocity() + mWindspeed);
}

} // namespace Radion::Physics
