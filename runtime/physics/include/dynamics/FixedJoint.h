#ifndef RADION_PHYSICS_DYNAMICS_FIXEDJOINT_H
#define RADION_PHYSICS_DYNAMICS_FIXEDJOINT_H

#include "Math.h"
#include "dynamics/Joint.h"

namespace Radion::Physics
{

class FixedJoint final : public Joint
{
public:
    FixedJoint(RigidBody& a, RigidBody& b, const Math::vec3& worldAnchor);
    FixedJoint(RigidBody& a, const Math::vec3& localAnchorA, RigidBody& b,
               const Math::vec3& localAnchorB);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

private:
    void calculatePositionProperties();
    void calculateRotationProperties();
    void applyVelocityImpulse(const Math::vec3& impulse);
    void applyAngularVelocityImpulse(const Math::vec3& impulse);

    RigidBody* mBodyA;
    RigidBody* mBodyB;
    Math::vec3 mLocalAnchorA;
    Math::vec3 mLocalAnchorB;
    Math::quat mInverseInitialOrientation;
    Math::vec3 mArmA{0.0f};
    Math::vec3 mArmB{0.0f};
    Math::mat3 mPositionEffectiveMass{0.0f};
    Math::mat3 mRotationEffectiveMass{0.0f};
    Math::vec3 mTotalPositionImpulse{0.0f};
    Math::vec3 mTotalRotationImpulse{0.0f};
    f32 mPreviousDuration = 0.0f;
};

}

#endif
