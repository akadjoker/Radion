#ifndef RADION_PHYSICS_DYNAMICS_UNIVERSALJOINT_H
#define RADION_PHYSICS_DYNAMICS_UNIVERSALJOINT_H

#include "Math.h"
#include "dynamics/Joint.h"

namespace Radion::Physics
{

// A ball-and-socket point plus two independent hinge axes held perpendicular
// to each other - a CV joint or a robot wrist gimbal, distinct from a 3-DOF
// PointJoint by keeping both rotation axes tracked and independently
// limited/motored. The DOF split follows ODE's Universal joint (fisica/ODE/
// ode/src/joints/universal.cpp: 3 point rows + 1 perpendicularity row,
// 2 free rotations). The perpendicularity lock is ported from that file's
// getInfo2 directly; the per-axis angle used for limits and motors is our
// own simplification - a single relative orientation captured at
// construction (as FixedJoint does), with each axis's twist read from it via
// the same swing-twist projection HingeJoint already uses, rather than
// ODE's "cross frame" getAngles(). Deliberate deviation, not yet cross
// checked against ODE's own angle convention beyond both reading zero at
// construction.
class UniversalJoint final : public Joint
{
public:
    UniversalJoint(RigidBody& a, RigidBody& b, const Math::Vec3& worldAnchor,
                   const Math::Vec3& worldAxisA, const Math::Vec3& worldAxisB);
    UniversalJoint(RigidBody& a, const Math::Vec3& localAnchorA, const Math::Vec3& localAxisA,
                   RigidBody& b, const Math::Vec3& localAnchorB, const Math::Vec3& localAxisB);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    void setLimitsA(f32 minAngle, f32 maxAngle);
    void setLimitsB(f32 minAngle, f32 maxAngle);
    f32 currentAngleA() const;
    f32 currentAngleB() const;
    void setMotorA(f32 targetAngularVelocity, f32 maxTorque);
    void setMotorB(f32 targetAngularVelocity, f32 maxTorque);
    void disableMotorA();
    void disableMotorB();

private:
    void calculatePositionProperties();
    void calculatePerpendicularityProperties();
    void calculateAngles();
    void calculateLimitProperties(bool hasLimits, f32 theta, f32 minAngle, f32 maxAngle,
                                  const Math::Vec3& axis, bool& active, f32& effectiveMass);
    void calculateMotorProperties(bool enabled, const Math::Vec3& axis, f32& effectiveMass);
    void applyLinearImpulse(const Math::Vec3& impulse);
    void applyAngularImpulse(const Math::Vec3& impulse);

    RigidBody* mBodyA;
    RigidBody* mBodyB;
    Math::Vec3 mLocalAnchorA;
    Math::Vec3 mLocalAnchorB;
    Math::Vec3 mLocalAxisA;
    Math::Vec3 mLocalAxisB;
    Math::Quaternion mInverseInitialOrientation;

    Math::Vec3 mArmA{0.0f};
    Math::Vec3 mArmB{0.0f};
    Math::Mat3 mPositionEffectiveMass{0.0f};
    Math::Vec3 mTotalPositionImpulse{0.0f};

    Math::Vec3 mAxisA{1.0f, 0.0f, 0.0f};
    Math::Vec3 mAxisB{0.0f, 1.0f, 0.0f};
    Math::Vec3 mPerpendicularAxis{0.0f, 0.0f, 1.0f};
    f32 mPerpendicularity = 0.0f;
    f32 mPerpendicularEffectiveMass = 0.0f;
    f32 mTotalPerpendicularImpulse = 0.0f;

    f32 mThetaA = 0.0f;
    f32 mThetaB = 0.0f;

    f32 mLimitsMinA = -glm::pi<f32>();
    f32 mLimitsMaxA = glm::pi<f32>();
    bool mHasLimitsA = false;
    bool mLimitActiveA = false;
    f32 mLimitEffectiveMassA = 0.0f;
    f32 mTotalLimitImpulseA = 0.0f;

    f32 mLimitsMinB = -glm::pi<f32>();
    f32 mLimitsMaxB = glm::pi<f32>();
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
