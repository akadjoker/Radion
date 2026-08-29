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
    // `worldSuspensionAxis` points from the chassis mount towards the ground
    // (the strut's travel direction); `worldSpinAxis` is the wheel's own
    // rolling axis. They do not need to start perpendicular - only linearly
    // independent - the perpendicularity constraint pulls the spin axis
    // square to the suspension axis as soon as the joint starts solving.
    WheelJoint(RigidBody& chassis, RigidBody& wheel, const glm::vec3& worldAnchor,
              const glm::vec3& worldSuspensionAxis, const glm::vec3& worldSpinAxis);

    // Empty, for the component path: the editor adds one of these to an
    // object and rebuild() wires it to the connected body, the same way
    // HingeJoint does. The two axes come from setAuthoredSuspensionAxis()
    // and setAuthoredSpinAxis() instead of from the constructor.
    WheelJoint();

    void configure(RigidBody& chassis, RigidBody& wheel, const glm::vec3& worldAnchor,
                   const glm::vec3& worldSuspensionAxis, const glm::vec3& worldSpinAxis);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;
    void rebuild() override;

    // Both in the owner's own local space. Suspension points from the
    // chassis mount towards the ground; spin is the axle.
    void setAuthoredSuspensionAxis(const glm::vec3& axis);
    const glm::vec3& authoredSuspensionAxis() const
    {
        return mAuthoredSuspensionAxis;
    }
    void setAuthoredSpinAxis(const glm::vec3& axis);
    const glm::vec3& authoredSpinAxis() const
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

    glm::vec3 anchorWorldA() const override;
    glm::vec3 anchorWorldB() const override;
    bool hasAxis() const override
    {
        return true;
    }
    glm::vec3 axisWorld() const override;

    // Spring-damper along the suspension axis. `restLength` is the anchor
    // separation (in the suspension direction) where the spring applies no
    // force; `stiffness` and `damping` are the usual F = -k*x - c*v terms,
    // in force per metre and force per metre-per-second.
    //
    // Solved as a soft constraint row on the suspension axis (SoftSpring),
    // not as a force pushed in from outside. A real car's springs are stiff -
    // a 1500 kg car sitting 20 cm into its travel needs about 75 kN/m - and
    // an explicitly integrated spring that stiff gains energy every step
    // until it explodes. This one does not, at any stiffness.
    void setSuspension(f32 restLength, f32 stiffness, f32 damping)
    {
        mSuspensionRestLength = restLength;
        mSuspensionStiffness = glm::max(stiffness, 0.0f);
        mSuspensionDamping = glm::max(damping, 0.0f);
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
    void applyLinearImpulse(const glm::vec3& impulse);
    void applyAngularImpulse(const glm::vec3& impulse);

    RigidBody* mChassis = nullptr;
    RigidBody* mWheel = nullptr;
    glm::vec3 mLocalAnchorChassis{0.0f};
    glm::vec3 mLocalAnchorWheel{0.0f};
    glm::vec3 mLocalSuspensionAxis{0.0f, -1.0f, 0.0f};
    glm::vec3 mLocalSpinAxis{1.0f, 0.0f, 0.0f};
    glm::vec3 mLocalNormalAxis{1.0f, 0.0f, 0.0f};
    glm::quat mInverseInitialOrientation{1.0f, 0.0f, 0.0f, 0.0f};
    // What the editor edits, before the joint is wired to a body - down and
    // along the axle, in the owner's local space.
    glm::vec3 mAuthoredSuspensionAxis{0.0f, -1.0f, 0.0f};
    glm::vec3 mAuthoredSpinAxis{1.0f, 0.0f, 0.0f};

    glm::vec3 mArmA{0.0f};
    glm::vec3 mArmB{0.0f};
    glm::vec3 mOffset{0.0f};

    // Suspension axis in world space, and the two lateral directions locked
    // rigidly against it.
    glm::vec3 mAxisA{0.0f, -1.0f, 0.0f};
    glm::vec3 mN1{1.0f, 0.0f, 0.0f};
    glm::vec3 mN2{0.0f, 0.0f, 1.0f};
    glm::mat2 mPositionLockEffectiveMass{0.0f};
    glm::vec2 mTotalPositionLockImpulse{0.0f};

    // Spin axis in world space and the perpendicularity row that keeps it
    // square to the suspension axis.
    glm::vec3 mAxisB{1.0f, 0.0f, 0.0f};
    glm::vec3 mPerpendicularAxis{0.0f, 0.0f, 1.0f};
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

    f32 mSteeringLimitsMin = -glm::pi<f32>();
    f32 mSteeringLimitsMax = glm::pi<f32>();
    bool mHasSteeringLimits = false;
    bool mSteeringLimitActive = false;
    f32 mSteeringLimitEffectiveMass = 0.0f;
    f32 mTotalSteeringLimitImpulse = 0.0f;

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
