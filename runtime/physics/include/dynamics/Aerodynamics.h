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
    AeroSurface(const glm::mat3& tensor, const glm::vec3& position);
    virtual ~AeroSurface() = default;

    void applyTo(RigidBody& body, const glm::vec3& windspeed) const;

    void setTensor(const glm::mat3& tensor)
    {
        mTensor = tensor;
    }
    const glm::mat3& tensor() const
    {
        return mTensor;
    }
    void setPosition(const glm::vec3& position)
    {
        mPosition = position;
    }
    const glm::vec3& position() const
    {
        return mPosition;
    }

protected:
    virtual glm::mat3 activeTensor() const
    {
        return mTensor;
    }

    glm::mat3 mTensor{0.0f};
    glm::vec3 mPosition{0.0f};
};

class AeroControlSurface final : public AeroSurface
{
public:
    AeroControlSurface() = default;
    AeroControlSurface(const glm::mat3& base, const glm::mat3& minimum, const glm::mat3& maximum,
                       const glm::vec3& position);

    void setControl(f32 control);
    f32 control() const
    {
        return mControl;
    }

protected:
    glm::mat3 activeTensor() const override;

private:
    glm::mat3 mMinimum{0.0f};
    glm::mat3 mMaximum{0.0f};
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

    void setWindspeed(const glm::vec3& windspeed)
    {
        mWindspeed = windspeed;
    }
    const glm::vec3& windspeed() const
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
    glm::vec3 mWindspeed{0.0f};
    f32 mThrust = 10.0f;
    f32 mRoll = 0.0f;
    f32 mYaw = 0.0f;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_AERODYNAMICS_H
