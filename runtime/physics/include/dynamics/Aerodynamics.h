#ifndef RADION_PHYSICS_AERODYNAMICS_H
#define RADION_PHYSICS_AERODYNAMICS_H

#include "Math.h"
#include "Types.h"

namespace Radion::Physics
{

class RigidBody;

class AeroSurface
{
public:
    AeroSurface() = default;
    AeroSurface(const Math::Mat3& tensor, const Math::Vec3& position);
    virtual ~AeroSurface() = default;

    void applyTo(RigidBody& body, const Math::Vec3& windspeed) const;

    void setTensor(const Math::Mat3& tensor)
    {
        mTensor = tensor;
    }
    const Math::Mat3& tensor() const
    {
        return mTensor;
    }
    void setPosition(const Math::Vec3& position)
    {
        mPosition = position;
    }
    const Math::Vec3& position() const
    {
        return mPosition;
    }

protected:
    virtual Math::Mat3 activeTensor() const
    {
        return mTensor;
    }

    Math::Mat3 mTensor{0.0f};
    Math::Vec3 mPosition{0.0f};
};

class AeroControlSurface final : public AeroSurface
{
public:
    AeroControlSurface() = default;
    AeroControlSurface(const Math::Mat3& base, const Math::Mat3& minimum, const Math::Mat3& maximum,
                       const Math::Vec3& position);

    void setControl(f32 control);
    f32 control() const
    {
        return mControl;
    }

protected:
    Math::Mat3 activeTensor() const override;

private:
    Math::Mat3 mMinimum{0.0f};
    Math::Mat3 mMaximum{0.0f};
    f32 mControl = 0.0f;
};

class Airplane
{
public:
    Airplane();

    void setDefaultSurfaces();

    void setBody(RigidBody* body)
    {
        mBody = body;
    }
    RigidBody* body() const
    {
        return mBody;
    }

    void setRoll(f32 control);
    void setYaw(f32 control);
    void setThrust(f32 thrust);
    f32 thrust() const
    {
        return mThrust;
    }
    f32 roll() const
    {
        return mRoll;
    }
    f32 yaw() const
    {
        return mYaw;
    }

    void setWindspeed(const Math::Vec3& windspeed)
    {
        mWindspeed = windspeed;
    }
    const Math::Vec3& windspeed() const
    {
        return mWindspeed;
    }

    void applyForces();

    f32 airspeed() const;

    AeroControlSurface& leftWing()
    {
        return mLeftWing;
    }
    AeroControlSurface& rightWing()
    {
        return mRightWing;
    }
    AeroControlSurface& rudder()
    {
        return mRudder;
    }
    AeroSurface& tail()
    {
        return mTail;
    }

private:
    RigidBody* mBody = nullptr;
    AeroControlSurface mLeftWing;
    AeroControlSurface mRightWing;
    AeroControlSurface mRudder;
    AeroSurface mTail;
    Math::Vec3 mWindspeed{0.0f};
    f32 mThrust = 10.0f;
    f32 mRoll = 0.0f;
    f32 mYaw = 0.0f;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_AERODYNAMICS_H
