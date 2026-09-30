#ifndef RADION_PHYSICS_DYNAMICS_UNIVERSALJOINT_H
#define RADION_PHYSICS_DYNAMICS_UNIVERSALJOINT_H

#include "Math.h"
#include "dynamics/Joint.h"

namespace Radion::Physics
{

// Ball-and-socket point plus two independent hinge axes held perpendicular (CV joint, wrist gimbal), each axis limited/motored.
// DOF split follows ODE's Universal joint (fisica/ODE/ode/src/joints/universal.cpp); the perpendicularity lock is ported from its getInfo2.
// Per-axis angles are our own simplification: a relative orientation captured at construction (as FixedJoint), twist read via HingeJoint's swing-twist
// projection, not ODE's "cross frame" getAngles(); not cross-checked against ODE's convention beyond both reading zero at construction.
class UniversalJoint final : public Joint
{
public:
    UniversalJoint();
    UniversalJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                   const Math::vec3& worldAxisA, const Math::vec3& worldAxisB);
    UniversalJoint(RigidBody& a, const Math::vec3& localAnchorA, const Math::vec3& localAxisA,
                   RigidBody& b, const Math::vec3& localAnchorB, const Math::vec3& localAxisB);
    void configure(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor,
                  const Math::vec3& worldAxisA, const Math::vec3& worldAxisB);
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

    void setLimitsA(f32 minAngle, f32 maxAngle);
    void setLimitsB(f32 minAngle, f32 maxAngle);
    f32 currentAngleA() const;
    f32 currentAngleB() const;
    void setMotorA(f32 targetAngularVelocity, f32 maxTorque);
    void setMotorB(f32 targetAngularVelocity, f32 maxTorque);
    void disableMotorA();
    void disableMotorB();

    void setAuthoredAxis(const Math::vec3& axis);
    const Math::vec3& authoredAxis() const;

private:
    void calculatePositionProperties();
    void calculatePerpendicularityProperties();
    void calculateAngles();
    void calculateLimitProperties(bool hasLimits, f32 theta, f32 minAngle, f32 maxAngle,
                                  const Math::vec3& axis, bool& active, f32& effectiveMass);
    void calculateMotorProperties(bool enabled, const Math::vec3& axis, f32& effectiveMass);
    void applyLinearImpulse(const Math::vec3& impulse);
    void applyAngularImpulse(const Math::vec3& impulse);

    // Null until rebuild() resolves them - see HingeJoint.
    RigidBody* mBodyA = nullptr;
    RigidBody* mBodyB = nullptr;
    Math::vec3 mLocalAnchorA;
    Math::vec3 mLocalAnchorB;
    Math::vec3 mLocalAxisA;
    Math::vec3 mLocalAxisB;
    Math::quat mInverseInitialOrientation;
    Math::vec3 mAuthoredAxis{1.0f, 0.0f, 0.0f};

    Math::vec3 mArmA{0.0f};
    Math::vec3 mArmB{0.0f};
    Math::mat3 mPositionEffectiveMass{0.0f};
    Math::vec3 mTotalPositionImpulse{0.0f};

    Math::vec3 mAxisA{1.0f, 0.0f, 0.0f};
    Math::vec3 mAxisB{0.0f, 1.0f, 0.0f};
    Math::vec3 mPerpendicularAxis{0.0f, 0.0f, 1.0f};
    f32 mPerpendicularity = 0.0f;
    f32 mPerpendicularEffectiveMass = 0.0f;
    f32 mTotalPerpendicularImpulse = 0.0f;

    f32 mThetaA = 0.0f;
    f32 mThetaB = 0.0f;

    f32 mLimitsMinA = -Math::pi<f32>();
    f32 mLimitsMaxA = Math::pi<f32>();
    bool mHasLimitsA = false;
    bool mLimitActiveA = false;
    f32 mLimitEffectiveMassA = 0.0f;
    f32 mTotalLimitImpulseA = 0.0f;

    f32 mLimitsMinB = -Math::pi<f32>();
    f32 mLimitsMaxB = Math::pi<f32>();
    bool mHasLimitsB = false;
    bool mLimitActiveB = false;
    f32 mLimitEffectiveMassB = 0.0f;
    f32 mTotalLimitImpulseB = 0.0f;

    f32 mMotorTargetVelocityA = 0.0f;
    f32 mMotorMaxTorqueA = 0.0f;
    bool mMotorEnabledA = false;
    f32 mMotorEffectiveMassA = 0.0f;
    f32 mMotorMaxImpulseA = 0.0f;
    f32 mTotalMotorImpulseA = 0.0f;

    f32 mMotorTargetVelocityB = 0.0f;
    f32 mMotorMaxTorqueB = 0.0f;
    bool mMotorEnabledB = false;
    f32 mMotorEffectiveMassB = 0.0f;
    f32 mMotorMaxImpulseB = 0.0f;
    f32 mTotalMotorImpulseB = 0.0f;

    f32 mPreviousDuration = 0.0f;
};

}

#endif
