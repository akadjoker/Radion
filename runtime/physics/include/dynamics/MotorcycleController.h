#ifndef RADION_PHYSICS_DYNAMICS_MOTORCYCLECONTROLLER_H
#define RADION_PHYSICS_DYNAMICS_MOTORCYCLECONTROLLER_H

#include "Math.h"
#include "Types.h"

namespace Radion::Physics
{

class RaycastVehicle;
class RigidBody;

// Keeps a two-wheeled raycast vehicle upright and leaning into its turns.
// A bike is an inverted pendulum: without this it falls over immediately. A
// PID lean spring drives the chassis roll towards the direction of the total
// ground impulse on the wheels, and the steering angle is clamped by the
// physics of countersteering so a fast bike cannot ask for a turn tighter
// than its maximum lean allows. Call preUpdate before the vehicle's update
// in the same fixed step and postUpdate right after it.
class MotorcycleController
{
public:
    MotorcycleController(RaycastVehicle& vehicle, RigidBody& chassis);

    void preUpdate(f32 step, const glm::vec3& gravity);
    void postUpdate(f32 step);

    // -1..1, positive steers left; the controller owns the front wheel's
    // steering so the lean limit can clamp it.
    void setSteerInput(f32 input);
    void setMaxSteerAngle(f32 angle);
    // Angle between the steering axis and straight up. A real bike's fork is
    // raked back; it feeds the steering geometry below.
    void setCasterAngle(f32 angle);

    void setMaxLeanAngle(f32 angle);
    void setLeanSpring(f32 springConstant, f32 damping);
    void setLeanSpringIntegration(f32 coefficient, f32 decay);
    void setLeanSmoothingFactor(f32 factor);
    // Off lets the bike fall over - a crash, a kickstand moment.
    void setLeanControllerEnabled(bool enabled);
    bool leanControllerEnabled() const;

    f32 currentLeanAngle() const;

private:
    f32 wheelBase() const;

    RaycastVehicle& mVehicle;
    RigidBody& mChassis;

    f32 mSteerInput = 0.0f;
    f32 mMaxSteerAngle = 0.7f;
    f32 mCasterAngle = 0.0f;

    f32 mMaxLeanAngle = 0.7853982f;
    f32 mLeanSpringConstant = 5000.0f;
    f32 mLeanSpringDamping = 1000.0f;
    f32 mLeanSpringIntegrationCoefficient = 0.0f;
    f32 mLeanSpringIntegrationCoefficientDecay = 4.0f;
    f32 mLeanSmoothingFactor = 0.8f;
    bool mEnableLeanController = true;

    glm::vec3 mTargetLean{0.0f, 1.0f, 0.0f};
    f32 mLeanSpringIntegratedDeltaAngle = 0.0f;
};

}

#endif
