#ifndef RADION_PHYSICS_DYNAMICS_WHEELJOINT_H
#define RADION_PHYSICS_DYNAMICS_WHEELJOINT_H

#include "Math.h"
#include "dynamics/Joint.h"
#include "dynamics/SoftSpring.h"

namespace Radion::Physics
{

class WheelJoint final : public Joint
{
public:
    // `worldSuspensionAxis` points chassis mount to ground (strut travel); `worldSpinAxis` is the rolling axis. They need only be linearly independent:
    // the perpendicularity constraint squares the spin axis to the suspension once solving starts.
    WheelJoint(RigidBody& chassis, RigidBody& wheel, const Math::vec3& worldAnchor,
              const Math::vec3& worldSuspensionAxis, const Math::vec3& worldSpinAxis);

    // Empty, for the component path: rebuild() wires it (as HingeJoint); axes come from setAuthoredSuspensionAxis()/setAuthoredSpinAxis().
    WheelJoint();

    void configure(RigidBody& chassis, RigidBody& wheel, const Math::vec3& worldAnchor,
                   const Math::vec3& worldSuspensionAxis, const Math::vec3& worldSpinAxis);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;
    void rebuild() override;

    // Both in the owner's own local space. Suspension points from the
    // chassis mount towards the ground; spin is the axle.
    void setAuthoredSuspensionAxis(const Math::vec3& axis);
    const Math::vec3& authoredSuspensionAxis() const
    {
        return mAuthoredSuspensionAxis;
    }
    void setAuthoredSpinAxis(const Math::vec3& axis);
    const Math::vec3& authoredSpinAxis() const
    {
        return mAuthoredSpinAxis;
    }
    f32 suspensionRestLength() const
    {
        return mSuspensionRestLength;
    }
    f32 suspensionStiffness() const
    {
        return mSuspensionStiffness;
    }
    f32 suspensionDamping() const
    {
        return mSuspensionDamping;
    }
    // Steer to an angle instead of a speed; as HingeJoint::setServo(): holds the target, each step turns the error into motor speed capped by the rack's rated speed.
    void setSteeringServo(f32 targetAngle, f32 maxTorque, f32 maxAngularVelocity = 0.0f);
    void disableSteeringServo();
    f32 steeringServoTargetAngle() const
    {
        return mSteeringServoTargetAngle;
    }
    f32 steeringServoMaxAngularVelocity() const
    {
        return mSteeringServoMaxAngularVelocity;
    }
    bool steeringServoEnabled() const
    {
        return mSteeringServoEnabled;
    }

    f32 steeringMotorTargetVelocity() const
    {
        return mSteeringMotorTargetVelocity;
    }
    f32 steeringMotorMaxTorque() const
    {
        return mSteeringMotorMaxTorque;
    }
    f32 spinMotorTargetVelocity() const
    {
        return mSpinMotorTargetVelocity;
    }
    f32 spinMotorMaxTorque() const
    {
        return mSpinMotorMaxTorque;
    }
    f32 minSteeringAngle() const
    {
        return mSteeringLimitsMin;
    }
    f32 maxSteeringAngle() const
    {
        return mSteeringLimitsMax;
    }

    Math::vec3 anchorWorldA() const override;
    Math::vec3 anchorWorldB() const override;
    bool hasAxis() const override
    {
        return true;
    }
    Math::vec3 axisWorld() const override;

    // Spring-damper along the suspension axis. `restLength` is the anchor separation where the spring applies no force; F = -k*x - c*v.
    // Solved as a soft constraint row (SoftSpring), not an external force: real car springs are stiff (~75 kN/m for 1500 kg sitting 20 cm in)
    // and an explicit spring that stiff explodes.
    void setSuspension(f32 restLength, f32 stiffness, f32 damping)
    {
        mSuspensionRestLength = restLength;
        mSuspensionStiffness = Math::max(stiffness, 0.0f);
        mSuspensionDamping = Math::max(damping, 0.0f);
    }
    f32 suspensionTravel() const
    {
        return mSlidePosition;
    }

    void setSteeringLimits(f32 minAngle, f32 maxAngle);
    void setSteeringMotor(f32 targetAngularVelocity, f32 maxTorque);
    void disableSteeringMotor();
    f32 steeringAngle() const;

    void setSpinMotor(f32 targetAngularVelocity, f32 maxTorque);
    void disableSpinMotor();
    f32 spinAngle() const;
    f32 spinAngularVelocity() const;

private:
    void calculateArmsAndOffset();
    void calculatePositionLockProperties();
    void calculatePerpendicularityProperties();
    void calculateAngles();
    void calculateSteeringLimitProperties();
    void calculateSteeringMotorProperties();
    void calculateSpinMotorProperties();
    void calculateSuspensionProperties(f32 duration);
    void applyLinearImpulse(const Math::vec3& impulse);
    void applyAngularImpulse(const Math::vec3& impulse);

    RigidBody* mChassis = nullptr;
    RigidBody* mWheel = nullptr;
    Math::vec3 mLocalAnchorChassis{0.0f};
    Math::vec3 mLocalAnchorWheel{0.0f};
    Math::vec3 mLocalSuspensionAxis{0.0f, -1.0f, 0.0f};
    Math::vec3 mLocalSpinAxis{1.0f, 0.0f, 0.0f};
    Math::vec3 mLocalNormalAxis{1.0f, 0.0f, 0.0f};
    Math::quat mInverseInitialOrientation{1.0f, 0.0f, 0.0f, 0.0f};
    // What the editor edits, before the joint is wired to a body - down and
    // along the axle, in the owner's local space.
    Math::vec3 mAuthoredSuspensionAxis{0.0f, -1.0f, 0.0f};
    Math::vec3 mAuthoredSpinAxis{1.0f, 0.0f, 0.0f};

    Math::vec3 mArmA{0.0f};
    Math::vec3 mArmB{0.0f};
    Math::vec3 mOffset{0.0f};

    // Suspension axis in world space, and the two lateral directions locked
    // rigidly against it.
    Math::vec3 mAxisA{0.0f, -1.0f, 0.0f};
    Math::vec3 mN1{1.0f, 0.0f, 0.0f};
    Math::vec3 mN2{0.0f, 0.0f, 1.0f};
    Math::mat2 mPositionLockEffectiveMass{0.0f};
    Math::vec2 mTotalPositionLockImpulse{0.0f};

    // Spin axis in world space and the perpendicularity row that keeps it
    // square to the suspension axis.
    Math::vec3 mAxisB{1.0f, 0.0f, 0.0f};
    Math::vec3 mPerpendicularAxis{0.0f, 0.0f, 1.0f};
    f32 mPerpendicularity = 0.0f;
    f32 mPerpendicularEffectiveMass = 0.0f;
    f32 mTotalPerpendicularImpulse = 0.0f;

    f32 mSlidePosition = 0.0f;
    f32 mSteeringAngle = 0.0f;
    f32 mSpinAngleValue = 0.0f;

    f32 mSuspensionRestLength = 0.0f;
    f32 mSuspensionStiffness = 0.0f;
    f32 mSuspensionDamping = 0.0f;
    SoftSpring mSuspensionSpring;
    f32 mSuspensionEffectiveMass = 0.0f;
    f32 mTotalSuspensionImpulse = 0.0f;

    f32 mSteeringLimitsMin = -Math::pi<f32>();
    f32 mSteeringLimitsMax = Math::pi<f32>();
    bool mHasSteeringLimits = false;
    bool mSteeringLimitActive = false;
    f32 mSteeringLimitEffectiveMass = 0.0f;
    f32 mTotalSteeringLimitImpulse = 0.0f;

    // The steering servo owns mSteeringMotorTargetVelocity while it is on,
    // rewriting it from the angle error every setup().
    f32 mSteeringServoTargetAngle = 0.0f;
    f32 mSteeringServoMaxAngularVelocity = 0.0f;
    bool mSteeringServoEnabled = false;

    f32 mSteeringMotorTargetVelocity = 0.0f;
    f32 mSteeringMotorMaxTorque = 0.0f;
    f32 mSteeringMotorMaxImpulse = 0.0f;
    bool mSteeringMotorEnabled = false;
    f32 mSteeringMotorEffectiveMass = 0.0f;
    f32 mTotalSteeringMotorImpulse = 0.0f;

    f32 mSpinMotorTargetVelocity = 0.0f;
    f32 mSpinMotorMaxTorque = 0.0f;
    f32 mSpinMotorMaxImpulse = 0.0f;
    bool mSpinMotorEnabled = false;
    f32 mSpinMotorEffectiveMass = 0.0f;
    f32 mTotalSpinMotorImpulse = 0.0f;

    f32 mPreviousDuration = 0.0f;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_DYNAMICS_WHEELJOINT_H
