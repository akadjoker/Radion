#ifndef RADION_PHYSICS_DYNAMICS_POINTJOINT_H
#define RADION_PHYSICS_DYNAMICS_POINTJOINT_H

#include "Math.h"
#include "dynamics/Joint.h"

namespace Radion::Physics
{

class PointJoint final : public Joint
{
public:
    PointJoint(RigidBody& a, RigidBody& b, const Math::Vec3& worldAnchor);
    PointJoint(RigidBody& a, const Math::Vec3& localAnchorA, RigidBody& b,
               const Math::Vec3& localAnchorB);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    const Math::Vec3& localAnchorA() const;
    const Math::Vec3& localAnchorB() const;
    Math::Vec3 worldAnchorA() const;
    Math::Vec3 worldAnchorB() const;
    void setMotor(const Math::Vec3& localAxisA, f32 targetVelocity, f32 maxTorque);
    void disableMotor();

private:
    void calculateProperties();
    void applyVelocityImpulse(const Math::Vec3& impulse);
    void applyMotorImpulse(f32 impulse);

    RigidBody* mBodyA;
    RigidBody* mBodyB;
    Math::Vec3 mLocalAnchorA;
    Math::Vec3 mLocalAnchorB;
    Math::Vec3 mArmA{0.0f};
    Math::Vec3 mArmB{0.0f};
    Math::Mat3 mEffectiveMass{0.0f};
    Math::Vec3 mTotalImpulse{0.0f};
    Math::Vec3 mMotorLocalAxisA{1.0f, 0.0f, 0.0f};
    Math::Vec3 mMotorWorldAxis{1.0f, 0.0f, 0.0f};
    f32 mMotorTargetVelocity = 0.0f;
    f32 mMotorMaxTorque = 0.0f;
    f32 mMotorEffectiveMass = 0.0f;
    f32 mMotorTotalImpulse = 0.0f;
    f32 mMotorMaxImpulse = 0.0f;
    bool mMotorEnabled = false;
    f32 mPreviousDuration = 0.0f;
};

}

#endif
