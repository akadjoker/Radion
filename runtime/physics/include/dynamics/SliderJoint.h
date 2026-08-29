#ifndef RADION_PHYSICS_DYNAMICS_SLIDERJOINT_H
#define RADION_PHYSICS_DYNAMICS_SLIDERJOINT_H

#include "Math.h"
#include "dynamics/Joint.h"

namespace Radion::Physics
{

class SliderJoint final : public Joint
{
public:
    SliderJoint();
    SliderJoint(RigidBody& a, RigidBody& b, const glm::vec3& worldAnchor,
                const glm::vec3& worldSliderAxis);
    SliderJoint(RigidBody& a, const glm::vec3& localAnchorA, const glm::vec3& localSliderAxisA,
                const glm::vec3& localNormalAxisA, RigidBody& b, const glm::vec3& localAnchorB,
                const glm::vec3& localSliderAxisB, const glm::vec3& localNormalAxisB);
    void configure(RigidBody& a, RigidBody& b, const glm::vec3& worldAnchor,
                  const glm::vec3& worldSliderAxis);
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

    void setLimits(f32 minDistance, f32 maxDistance);
    f32 minDistance() const;
    f32 maxDistance() const;
    f32 currentPosition() const;
    void setMotor(f32 targetVelocity, f32 maxForce);
    void disableMotor();
    f32 motorTargetVelocity() const;
    f32 motorMaxForce() const;
    bool motorEnabled() const;

    void setAuthoredAxis(const glm::vec3& axis);
    const glm::vec3& authoredAxis() const;

private:
    void calculateArmsAndOffset();
    void calculatePositionLockProperties();
    void calculateRotationProperties();
    void calculateSlideAxisAndPosition();
    void calculateLimitProperties();
    void calculateMotorProperties();
    void applyVelocityImpulse(const glm::vec3& impulse);
    void applyAngularVelocityImpulse(const glm::vec3& impulse);

    RigidBody* mBodyA;
    RigidBody* mBodyB;
    glm::vec3 mLocalAnchorA;
    glm::vec3 mLocalAnchorB;
    glm::vec3 mLocalSliderAxisA;
    glm::vec3 mLocalNormalAxisA;
    glm::vec3 mLocalNormalAxisA2;
    glm::quat mInverseInitialOrientation;
    glm::vec3 mAuthoredAxis{1.0f, 0.0f, 0.0f};

    glm::vec3 mArmA{0.0f};
    glm::vec3 mArmB{0.0f};
    glm::vec3 mOffset{0.0f};

    glm::vec3 mN1{1.0f, 0.0f, 0.0f};
    glm::vec3 mN2{0.0f, 0.0f, 1.0f};
    glm::mat2 mPositionLockEffectiveMass{0.0f};
    glm::vec2 mTotalPositionLockImpulse{0.0f};

    glm::mat3 mRotationEffectiveMass{0.0f};
    glm::vec3 mTotalRotationImpulse{0.0f};

    glm::vec3 mWorldSliderAxis{1.0f, 0.0f, 0.0f};
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
