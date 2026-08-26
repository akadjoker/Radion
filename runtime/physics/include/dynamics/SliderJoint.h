#ifndef RADION_PHYSICS_DYNAMICS_SLIDERJOINT_H
#define RADION_PHYSICS_DYNAMICS_SLIDERJOINT_H

#include "Math.h"
#include "dynamics/Joint.h"

namespace Radion::Physics
{

class SliderJoint final : public Joint
{
public:
    SliderJoint(RigidBody& a, RigidBody& b, const Math::Vec3& worldAnchor,
                const Math::Vec3& worldSliderAxis);
    SliderJoint(RigidBody& a, const Math::Vec3& localAnchorA, const Math::Vec3& localSliderAxisA,
                const Math::Vec3& localNormalAxisA, RigidBody& b, const Math::Vec3& localAnchorB,
                const Math::Vec3& localSliderAxisB, const Math::Vec3& localNormalAxisB);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    void setLimits(f32 minDistance, f32 maxDistance);
    f32 currentPosition() const;
    void setMotor(f32 targetVelocity, f32 maxForce);
    void disableMotor();

private:
    void calculateArmsAndOffset();
    void calculatePositionLockProperties();
    void calculateRotationProperties();
    void calculateSlideAxisAndPosition();
    void calculateLimitProperties();
    void calculateMotorProperties();
    void applyVelocityImpulse(const Math::Vec3& impulse);
    void applyAngularVelocityImpulse(const Math::Vec3& impulse);

    RigidBody* mBodyA;
    RigidBody* mBodyB;
    Math::Vec3 mLocalAnchorA;
    Math::Vec3 mLocalAnchorB;
    Math::Vec3 mLocalSliderAxisA;
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

    Math::Mat3 mRotationEffectiveMass{0.0f};
    Math::Vec3 mTotalRotationImpulse{0.0f};

    Math::Vec3 mWorldSliderAxis{1.0f, 0.0f, 0.0f};
    f32 mSlidePosition = 0.0f;

    f32 mLimitsMin = -M_INFINITY;
    f32 mLimitsMax = M_INFINITY;
    bool mHasLimits = false;
    bool mLimitActive = false;
    f32 mLimitEffectiveMass = 0.0f;
    f32 mTotalLimitImpulse = 0.0f;

    f32 mMotorTargetVelocity = 0.0f;
    f32 mMotorMaxForce = 0.0f;
    bool mMotorEnabled = false;
    f32 mMotorEffectiveMass = 0.0f;
    f32 mMotorMaxImpulse = 0.0f;
    f32 mTotalMotorImpulse = 0.0f;

    f32 mPreviousDuration = 0.0f;
};

}

#endif
