#ifndef RADION_PHYSICS_DYNAMICS_FIXEDJOINT_H
#define RADION_PHYSICS_DYNAMICS_FIXEDJOINT_H

#include "Math.h"
#include "dynamics/Joint.h"

namespace Radion::Physics
{

class FixedJoint final : public Joint
{
public:
    FixedJoint(RigidBody& a, RigidBody& b, const Math::Vec3& worldAnchor);
    FixedJoint(RigidBody& a, const Math::Vec3& localAnchorA, RigidBody& b,
               const Math::Vec3& localAnchorB);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

private:
    void calculatePositionProperties();
    void calculateRotationProperties();
    void applyVelocityImpulse(const Math::Vec3& impulse);
    void applyAngularVelocityImpulse(const Math::Vec3& impulse);

    RigidBody* mBodyA;
    RigidBody* mBodyB;
    Math::Vec3 mLocalAnchorA;
    Math::Vec3 mLocalAnchorB;
    Math::Quaternion mInverseInitialOrientation;
    Math::Vec3 mArmA{0.0f};
    Math::Vec3 mArmB{0.0f};
    Math::Mat3 mPositionEffectiveMass{0.0f};
    Math::Mat3 mRotationEffectiveMass{0.0f};
    Math::Vec3 mTotalPositionImpulse{0.0f};
    Math::Vec3 mTotalRotationImpulse{0.0f};
    f32 mPreviousDuration = 0.0f;
};

}

#endif
