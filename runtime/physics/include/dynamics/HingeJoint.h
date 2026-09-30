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
    HingeJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
               const Math::vec3& worldHingeAxis);
    HingeJoint(RigidBody& a, const Math::vec3& localAnchorA, const Math::vec3& localHingeAxisA,
               const Math::vec3& localNormalAxisA, RigidBody& b, const Math::vec3& localAnchorB,
               const Math::vec3& localHingeAxisB, const Math::vec3& localNormalAxisB);
    void configure(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                  const Math::vec3& worldHingeAxis);
    void rebuild() override;

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    Math::vec3 anchorWorldA() const override;
    Math::vec3 anchorWorldB() const override;
    bool hasAxis() const override
    {
        return true;
    }
    Math::vec3 axisWorld() const override;

    void setLimits(f32 minAngle, f32 maxAngle);
    f32 minAngle() const;
    f32 maxAngle() const;
    f32 currentAngle() const;
    void setMotor(f32 targetAngularVelocity, f32 maxTorque);
    void disableMotor();
    f32 motorTargetVelocity() const;
    f32 motorMaxTorque() const;
    bool motorEnabled() const;

    // Hold an angle instead of a speed. Each step setup() turns the remaining error into the speed that would close it in one step,
    // and maxTorque stops that being instantaneous. The target is clamped to the limits and held until changed.
    // maxAngularVelocity is the servo's rated speed; 0 (unlimited) is only safe with a tight torque budget: uncapped, speed = error/dt is a
    // proportional gain of 1/dt with no damping, so it oscillates (+-27 degrees on a three-link chain in testServoChainSagUnderLoad()).
    void setServo(f32 targetAngle, f32 maxTorque, f32 maxAngularVelocity = 0.0f);
    void disableServo();
    f32 servoTargetAngle() const;
    f32 servoMaxAngularVelocity() const;
    bool servoEnabled() const;

    void setAuthoredAxis(const Math::vec3& axis);
    const Math::vec3& authoredAxis() const;

private:
    void calculatePositionProperties();
    void calculateHingeRotationProperties();
    void calculateAxisAndAngle();
    void calculateLimitProperties(f32 duration);
    void calculateMotorProperties();
    f32 smallestAngleToLimit() const;
    bool minLimitClosest() const;
    void applyVelocityImpulse(const Math::vec3& impulse);
    void applyAngularVelocityImpulse(const Math::vec3& impulse);

    // Null until rebuild() resolves them; uninitialised, the component path (joint exists before being wired up) had two garbage pointers.
    RigidBody* mBodyA = nullptr;
    RigidBody* mBodyB = nullptr;
    Math::vec3 mLocalAnchorA;
    Math::vec3 mLocalAnchorB;
    Math::vec3 mLocalHingeAxisA;
    Math::vec3 mLocalHingeAxisB;
    Math::vec3 mLocalNormalAxisA;
    Math::vec3 mLocalNormalAxisB;
    Math::quat mInverseInitialOrientation;
    Math::vec3 mAuthoredAxis{0.0f, 1.0f, 0.0f};

    Math::vec3 mArmA{0.0f};
    Math::vec3 mArmB{0.0f};
    Math::mat3 mPositionEffectiveMass{0.0f};
    Math::vec3 mTotalPositionImpulse{0.0f};

    Math::vec3 mA1{0.0f, 1.0f, 0.0f};
    Math::vec3 mB2{1.0f, 0.0f, 0.0f};
    Math::vec3 mC2{0.0f, 0.0f, 1.0f};
    Math::vec3 mB2xA1{0.0f};
    Math::vec3 mC2xA1{0.0f};
    Math::mat2 mHingeRotationEffectiveMass{0.0f};
    Math::vec2 mTotalHingeRotationImpulse{0.0f};

    f32 mTheta = 0.0f;
    f32 mLimitsMin = -Math::pi<f32>();
    f32 mLimitsMax = Math::pi<f32>();
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
