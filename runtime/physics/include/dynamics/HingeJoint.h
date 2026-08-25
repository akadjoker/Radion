#ifndef RADION_PHYSICS_DYNAMICS_HINGEJOINT_H
#define RADION_PHYSICS_DYNAMICS_HINGEJOINT_H

#include "Math.h"
#include "dynamics/Joint.h"

namespace Radion::Physics
{

class HingeJoint final : public Joint
{
public:
    HingeJoint(RigidBody& a, RigidBody& b, const glm::vec3& worldAnchor,
               const glm::vec3& worldHingeAxis);
    HingeJoint(RigidBody& a, const glm::vec3& localAnchorA, const glm::vec3& localHingeAxisA,
               const glm::vec3& localNormalAxisA, RigidBody& b, const glm::vec3& localAnchorB,
               const glm::vec3& localHingeAxisB, const glm::vec3& localNormalAxisB);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    void setLimits(f32 minAngle, f32 maxAngle);
    f32 currentAngle() const;
    void setMotor(f32 targetAngularVelocity, f32 maxTorque);
    void disableMotor();

private:
    void calculatePositionProperties();
    void calculateHingeRotationProperties();
    void calculateAxisAndAngle();
    void calculateLimitProperties(f32 duration);
    void calculateMotorProperties();
    f32 smallestAngleToLimit() const;
    bool minLimitClosest() const;
    void applyVelocityImpulse(const glm::vec3& impulse);
    void applyAngularVelocityImpulse(const glm::vec3& impulse);

    RigidBody* mBodyA;
    RigidBody* mBodyB;
    glm::vec3 mLocalAnchorA;
    glm::vec3 mLocalAnchorB;
    glm::vec3 mLocalHingeAxisA;
    glm::vec3 mLocalHingeAxisB;
    glm::vec3 mLocalNormalAxisA;
    glm::vec3 mLocalNormalAxisB;
    glm::quat mInverseInitialOrientation;

    glm::vec3 mArmA{0.0f};
    glm::vec3 mArmB{0.0f};
    glm::mat3 mPositionEffectiveMass{0.0f};
    glm::vec3 mTotalPositionImpulse{0.0f};

    glm::vec3 mA1{0.0f, 1.0f, 0.0f};
    glm::vec3 mB2{1.0f, 0.0f, 0.0f};
    glm::vec3 mC2{0.0f, 0.0f, 1.0f};
    glm::vec3 mB2xA1{0.0f};
    glm::vec3 mC2xA1{0.0f};
    glm::mat2 mHingeRotationEffectiveMass{0.0f};
    glm::vec2 mTotalHingeRotationImpulse{0.0f};

    f32 mTheta = 0.0f;
    f32 mLimitsMin = -glm::pi<f32>();
    f32 mLimitsMax = glm::pi<f32>();
    bool mHasLimits = false;
    f32 mLimitEffectiveMass = 0.0f;
    f32 mTotalLimitImpulse = 0.0f;
    bool mLimitActive = false;

    f32 mMotorTargetVelocity = 0.0f;
    f32 mMotorMaxTorque = 0.0f;
    bool mMotorEnabled = false;
    f32 mMotorEffectiveMass = 0.0f;
    f32 mMotorMaxImpulse = 0.0f;
    f32 mTotalMotorImpulse = 0.0f;

    f32 mPreviousDuration = 0.0f;
};

}

#endif
