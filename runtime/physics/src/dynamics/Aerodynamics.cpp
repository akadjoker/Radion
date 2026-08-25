#include "PCH.h"

#include "dynamics/Aerodynamics.h"

#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

AeroSurface::AeroSurface(const glm::mat3& tensor, const glm::vec3& position)
    : mTensor(tensor)
    , mPosition(position)
{
}

void AeroSurface::applyTo(RigidBody& body, const glm::vec3& windspeed) const
{
    const glm::vec3 velocity = body.velocity() + windspeed;
    const glm::vec3 bodyVelocity = body.directionToLocal(velocity);
    const glm::vec3 bodyForce = activeTensor() * bodyVelocity;
    body.addForceAtBodyPoint(body.directionToWorld(bodyForce), mPosition);
}

AeroControlSurface::AeroControlSurface(const glm::mat3& base, const glm::mat3& minimum,
                                       const glm::mat3& maximum, const glm::vec3& position)
    : AeroSurface(base, position)
    , mMinimum(minimum)
    , mMaximum(maximum)
{
}

void AeroControlSurface::setControl(f32 control)
{
    mControl = glm::clamp(control, -1.0f, 1.0f);
}

glm::mat3 AeroControlSurface::activeTensor() const
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
    // glm::mat3 takes columns; the reference tensors are written row-major,
    // so each one below is transposed. The wings' -1 at row 1, column 0 maps
    // forward velocity onto upward force.
    const glm::mat3 wingBase = glm::transpose(glm::mat3(0.0f, 0.0f, 0.0f,
                                                        -1.0f, -0.5f, 0.0f,
                                                        0.0f, 0.0f, 0.0f));
    const glm::mat3 wingMin = glm::transpose(glm::mat3(0.0f, 0.0f, 0.0f,
                                                       -0.995f, -0.5f, 0.0f,
                                                       0.0f, 0.0f, 0.0f));
    const glm::mat3 wingMax = glm::transpose(glm::mat3(0.0f, 0.0f, 0.0f,
                                                       -1.005f, -0.5f, 0.0f,
                                                       0.0f, 0.0f, 0.0f));

    mRightWing = AeroControlSurface(wingBase, wingMin, wingMax, glm::vec3(-1.0f, 0.0f, 2.0f));
    mLeftWing = AeroControlSurface(wingBase, wingMin, wingMax, glm::vec3(-1.0f, 0.0f, -2.0f));

    const glm::mat3 rudderBase(0.0f);
    const glm::mat3 rudderMin = glm::transpose(glm::mat3(0.0f, 0.0f, 0.0f,
                                                         0.0f, 0.0f, 0.0f,
                                                         0.01f, 0.0f, 0.0f));
    const glm::mat3 rudderMax = glm::transpose(glm::mat3(0.0f, 0.0f, 0.0f,
                                                         0.0f, 0.0f, 0.0f,
                                                         -0.01f, 0.0f, 0.0f));
    mRudder = AeroControlSurface(rudderBase, rudderMin, rudderMax, glm::vec3(2.0f, 0.5f, 0.0f));

    const glm::mat3 tailTensor = glm::transpose(glm::mat3(0.0f, 0.0f, 0.0f,
                                                          -1.0f, -0.5f, 0.0f,
                                                          0.0f, 0.0f, -0.1f));
    mTail = AeroSurface(tailTensor, glm::vec3(2.0f, 0.0f, 0.0f));
}

void Airplane::setRoll(f32 control)
{
    mRoll = glm::clamp(control, -1.0f, 1.0f);
    mLeftWing.setControl(mRoll);
    mRightWing.setControl(-mRoll);
}

void Airplane::setYaw(f32 control)
{
    mYaw = glm::clamp(control, -1.0f, 1.0f);
    mRudder.setControl(mYaw);
}

void Airplane::setThrust(f32 thrust)
{
    mThrust = glm::max(thrust, 0.0f);
}

void Airplane::applyForces()
{
    if (!mBody)
        return;

    mBody->addForce(mBody->directionToWorld(glm::vec3(-mThrust, 0.0f, 0.0f)));

    mLeftWing.applyTo(*mBody, mWindspeed);
    mRightWing.applyTo(*mBody, mWindspeed);
    mRudder.applyTo(*mBody, mWindspeed);
    mTail.applyTo(*mBody, mWindspeed);
}

f32 Airplane::airspeed() const
{
    if (!mBody)
        return 0.0f;
    return glm::length(mBody->velocity() + mWindspeed);
}

} // namespace Radion::Physics
