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
    PistonJoint();
    PistonJoint(RigidBody& a, RigidBody& b, const glm::vec3& worldAnchor,
                const glm::vec3& worldAxis);
    PistonJoint(RigidBody& a, const glm::vec3& localAnchorA, const glm::vec3& localAxisA,
                const glm::vec3& localNormalAxisA, RigidBody& b, const glm::vec3& localAnchorB,
                const glm::vec3& localAxisB, const glm::vec3& localNormalAxisB);
    void configure(RigidBody& a, RigidBody& b, const glm::vec3& worldAnchor,
                  const glm::vec3& worldAxis);
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

    void setLinearLimits(f32 minDistance, f32 maxDistance);
    f32 minLinearDistance() const;
    f32 maxLinearDistance() const;
    void setAngularLimits(f32 minAngle, f32 maxAngle);
    f32 minAngularAngle() const;
    f32 maxAngularAngle() const;
    f32 currentPosition() const;
    f32 currentAngle() const;
    void setLinearMotor(f32 targetVelocity, f32 maxForce);
    void disableLinearMotor();
    f32 linearMotorTargetVelocity() const;
    f32 linearMotorMaxForce() const;
    void setAngularMotor(f32 targetAngularVelocity, f32 maxTorque);
    void disableAngularMotor();
    f32 angularMotorTargetVelocity() const;
    f32 angularMotorMaxTorque() const;

    void setAuthoredAxis(const glm::vec3& axis);
    const glm::vec3& authoredAxis() const;

private:
    void calculateArmsAndOffset();
    void calculatePositionLockProperties();
    void calculateRotationLockProperties();
    void calculateAxisAndPosition();
    void calculateLinearLimitProperties();
    void calculateAngularLimitProperties(f32 duration);
    void calculateLinearMotorProperties();
    void calculateAngularMotorProperties();
    void applyLinearImpulse(const glm::vec3& impulse);
    void applyAngularImpulse(const glm::vec3& impulse);

    RigidBody* mBodyA;
    RigidBody* mBodyB;
    glm::vec3 mLocalAnchorA;
    glm::vec3 mLocalAnchorB;
    glm::vec3 mLocalAxisA;
    glm::vec3 mLocalAxisB;
    glm::vec3 mLocalNormalAxisA;
    glm::vec3 mLocalNormalAxisA2;
    glm::quat mInverseInitialOrientation;
    glm::vec3 mAuthoredAxis{0.0f, 1.0f, 0.0f};

    glm::vec3 mArmA{0.0f};
    glm::vec3 mArmB{0.0f};
    glm::vec3 mOffset{0.0f};

    glm::vec3 mN1{1.0f, 0.0f, 0.0f};
    glm::vec3 mN2{0.0f, 0.0f, 1.0f};
    glm::mat2 mPositionLockEffectiveMass{0.0f};
    glm::vec2 mTotalPositionLockImpulse{0.0f};

    glm::vec3 mA1{0.0f, 1.0f, 0.0f};
    glm::vec3 mB2{1.0f, 0.0f, 0.0f};
    glm::vec3 mC2{0.0f, 0.0f, 1.0f};
    glm::vec3 mB2xA1{0.0f};
    glm::vec3 mC2xA1{0.0f};
    glm::mat2 mRotationLockEffectiveMass{0.0f};
    glm::vec2 mTotalRotationLockImpulse{0.0f};

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
