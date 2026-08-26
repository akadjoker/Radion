#ifndef RADION_PHYSICS_DYNAMICS_PISTONJOINT_H
#define RADION_PHYSICS_DYNAMICS_PISTONJOINT_H

#include "Math.h"
#include "dynamics/Joint.h"

namespace Radion::Physics
{

// Prismatic and revolute freedom along the same axis - a shock absorber or
// MacPherson strut, where SliderJoint or HingeJoint alone would only give
// one of the two. The DOF split (2 translation + 2 rotation locked, 1
// translation + 1 rotation free, each independently limited and motored)
// follows ODE's Piston joint (fisica/ODE/ode/src/joints/piston.cpp); the
// constraint math itself is our own, reusing the rotation-lock this file
// shares with HingeJoint and the position-lock it shares with SliderJoint.
class PistonJoint final : public Joint
{
public:
    PistonJoint(RigidBody& a, RigidBody& b, const Math::Vec3& worldAnchor,
                const Math::Vec3& worldAxis);
    PistonJoint(RigidBody& a, const Math::Vec3& localAnchorA, const Math::Vec3& localAxisA,
                const Math::Vec3& localNormalAxisA, RigidBody& b, const Math::Vec3& localAnchorB,
                const Math::Vec3& localAxisB, const Math::Vec3& localNormalAxisB);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    void setLinearLimits(f32 minDistance, f32 maxDistance);
    void setAngularLimits(f32 minAngle, f32 maxAngle);
    f32 currentPosition() const;
    f32 currentAngle() const;
    void setLinearMotor(f32 targetVelocity, f32 maxForce);
    void disableLinearMotor();
    void setAngularMotor(f32 targetAngularVelocity, f32 maxTorque);
    void disableAngularMotor();

private:
    void calculateArmsAndOffset();
    void calculatePositionLockProperties();
    void calculateRotationLockProperties();
    void calculateAxisAndPosition();
    void calculateLinearLimitProperties();
    void calculateAngularLimitProperties(f32 duration);
    void calculateLinearMotorProperties();
    void calculateAngularMotorProperties();
    void applyLinearImpulse(const Math::Vec3& impulse);
    void applyAngularImpulse(const Math::Vec3& impulse);

    RigidBody* mBodyA;
    RigidBody* mBodyB;
    Math::Vec3 mLocalAnchorA;
    Math::Vec3 mLocalAnchorB;
    Math::Vec3 mLocalAxisA;
    Math::Vec3 mLocalAxisB;
    Math::Vec3 mLocalNormalAxisA;
    Math::Vec3 mLocalNormalAxisA2;
    Math::Quaternion mInverseInitialOrientation;

    Math::Vec3 mArmA{0.0f};
    Math::Vec3 mArmB{0.0f};
    Math::Vec3 mOffset{0.0f};

    Math::Vec3 mN1{1.0f, 0.0f, 0.0f};
    Math::Vec3 mN2{0.0f, 0.0f, 1.0f};
    glm::mat2 mPositionLockEffectiveMass{0.0f};
    Math::Vec2 mTotalPositionLockImpulse{0.0f};

    Math::Vec3 mA1{0.0f, 1.0f, 0.0f};
    Math::Vec3 mB2{1.0f, 0.0f, 0.0f};
    Math::Vec3 mC2{0.0f, 0.0f, 1.0f};
    Math::Vec3 mB2xA1{0.0f};
    Math::Vec3 mC2xA1{0.0f};
    glm::mat2 mRotationLockEffectiveMass{0.0f};
    Math::Vec2 mTotalRotationLockImpulse{0.0f};

    f32 mSlidePosition = 0.0f;
    f32 mTheta = 0.0f;

    f32 mLinearLimitsMin = -M_INFINITY;
    f32 mLinearLimitsMax = M_INFINITY;
    bool mHasLinearLimits = false;
    bool mLinearLimitActive = false;
    f32 mLinearLimitEffectiveMass = 0.0f;
    f32 mTotalLinearLimitImpulse = 0.0f;

    f32 mAngularLimitsMin = -glm::pi<f32>();
    f32 mAngularLimitsMax = glm::pi<f32>();
    bool mHasAngularLimits = false;
    bool mAngularLimitActive = false;
    f32 mAngularLimitEffectiveMass = 0.0f;
    f32 mTotalAngularLimitImpulse = 0.0f;

    f32 mLinearMotorTargetVelocity = 0.0f;
    f32 mLinearMotorMaxForce = 0.0f;
    bool mLinearMotorEnabled = false;
    f32 mLinearMotorEffectiveMass = 0.0f;
    f32 mLinearMotorMaxImpulse = 0.0f;
    f32 mTotalLinearMotorImpulse = 0.0f;

    f32 mAngularMotorTargetVelocity = 0.0f;
    f32 mAngularMotorMaxTorque = 0.0f;
    bool mAngularMotorEnabled = false;
    f32 mAngularMotorEffectiveMass = 0.0f;
    f32 mAngularMotorMaxImpulse = 0.0f;
    f32 mTotalAngularMotorImpulse = 0.0f;

    f32 mPreviousDuration = 0.0f;
};

}

#endif
