#ifndef RADION_PHYSICS_DYNAMICS_HINGEJOINT_H
#define RADION_PHYSICS_DYNAMICS_HINGEJOINT_H

#include "Math.h"
#include "dynamics/Joint.h"

namespace Radion::Physics
{

class HingeJoint final : public Joint
{
public:
    HingeJoint();
    HingeJoint(RigidBody& a, RigidBody& b, const glm::vec3& worldAnchor,
               const glm::vec3& worldHingeAxis);
    HingeJoint(RigidBody& a, const glm::vec3& localAnchorA, const glm::vec3& localHingeAxisA,
               const glm::vec3& localNormalAxisA, RigidBody& b, const glm::vec3& localAnchorB,
               const glm::vec3& localHingeAxisB, const glm::vec3& localNormalAxisB);
    void configure(RigidBody& a, RigidBody& b, const glm::vec3& worldAnchor,
                  const glm::vec3& worldHingeAxis);
    void rebuild() override;

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    glm::vec3 anchorWorldA() const override;
    glm::vec3 anchorWorldB() const override;
    bool hasAxis() const override
    {
        return true;
    }
    glm::vec3 axisWorld() const override;

    void setLimits(f32 minAngle, f32 maxAngle);
    f32 minAngle() const;
    f32 maxAngle() const;
    f32 currentAngle() const;
    void setMotor(f32 targetAngularVelocity, f32 maxTorque);
    void disableMotor();
    f32 motorTargetVelocity() const;
    f32 motorMaxTorque() const;
    bool motorEnabled() const;

    // Hold an angle instead of a speed: what a servo is, and what a robot
    // joint is commanded with. Drives the motor above - every step, setup()
    // turns the remaining error into the speed that would close it in one
    // step, and maxTorque is what stops that from being instantaneous. A
    // target outside the joint's limits is clamped into them.
    //
    // Once set, it is held: the target stays until changed, so a caller
    // (or a command arriving over a socket) names an angle and stops
    // thinking about it.
    // maxAngularVelocity is the servo's rated speed, and leaving it at 0
    // (unlimited) is only safe when the torque budget is tight. Uncapped,
    // the commanded speed is the whole error divided by one step - a
    // proportional gain of 1/dt, with no damping under it - so a joint given
    // torque to spare overshoots, comes back, and oscillates without ever
    // settling: measured at +-27 degrees on a three-link chain in
    // testServoChainSagUnderLoad(). Real actuators have a rated speed and
    // that is what keeps this stable, not a tuning constant.
    void setServo(f32 targetAngle, f32 maxTorque, f32 maxAngularVelocity = 0.0f);
    void disableServo();
    f32 servoTargetAngle() const;
    f32 servoMaxAngularVelocity() const;
    bool servoEnabled() const;

    void setAuthoredAxis(const glm::vec3& axis);
    const glm::vec3& authoredAxis() const;

private:
    void calculatePositionProperties();
    void calculateHingeRotationProperties();
    void calculateAxisAndAngle();
    void calculateLimitProperties(f32 duration);
    void calculateMotorProperties();
    f32 smallestAngleToLimit() const;
    bool minLimitClosest() const;
    void applyVelocityImpulse(const glm::vec3& impulse);
    void applyAngularVelocityImpulse(const glm::vec3& impulse);

    // Null until rebuild() resolves them. Uninitialised, the component path -
    // where a joint exists from the moment it is added and is only wired up
    // later - had two garbage pointers that anything asking the joint about
    // its bodies would follow.
    RigidBody* mBodyA = nullptr;
    RigidBody* mBodyB = nullptr;
    glm::vec3 mLocalAnchorA;
    glm::vec3 mLocalAnchorB;
    glm::vec3 mLocalHingeAxisA;
    glm::vec3 mLocalHingeAxisB;
    glm::vec3 mLocalNormalAxisA;
    glm::vec3 mLocalNormalAxisB;
    glm::quat mInverseInitialOrientation;
    glm::vec3 mAuthoredAxis{0.0f, 1.0f, 0.0f};

    glm::vec3 mArmA{0.0f};
    glm::vec3 mArmB{0.0f};
    glm::mat3 mPositionEffectiveMass{0.0f};
    glm::vec3 mTotalPositionImpulse{0.0f};

    glm::vec3 mA1{0.0f, 1.0f, 0.0f};
    glm::vec3 mB2{1.0f, 0.0f, 0.0f};
    glm::vec3 mC2{0.0f, 0.0f, 1.0f};
    glm::vec3 mB2xA1{0.0f};
    glm::vec3 mC2xA1{0.0f};
    glm::mat2 mHingeRotationEffectiveMass{0.0f};
    glm::vec2 mTotalHingeRotationImpulse{0.0f};

    f32 mTheta = 0.0f;
    f32 mLimitsMin = -glm::pi<f32>();
    f32 mLimitsMax = glm::pi<f32>();
    bool mHasLimits = false;
    f32 mLimitEffectiveMass = 0.0f;
    f32 mTotalLimitImpulse = 0.0f;
    bool mLimitActive = false;

    f32 mMotorTargetVelocity = 0.0f;
    f32 mMotorMaxTorque = 0.0f;
    bool mMotorEnabled = false;
    f32 mMotorEffectiveMass = 0.0f;
    f32 mMotorMaxImpulse = 0.0f;
    f32 mTotalMotorImpulse = 0.0f;

    // The servo owns mMotorTargetVelocity while it is on, rewriting it every
    // setup() from the angle error.
    f32 mServoTargetAngle = 0.0f;
    f32 mServoMaxAngularVelocity = 0.0f; // 0 = unlimited
    bool mServoEnabled = false;

    f32 mPreviousDuration = 0.0f;
};

}

#endif
