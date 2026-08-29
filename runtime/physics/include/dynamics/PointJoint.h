#ifndef RADION_PHYSICS_DYNAMICS_POINTJOINT_H
#define RADION_PHYSICS_DYNAMICS_POINTJOINT_H

#include "Math.h"
#include "dynamics/Joint.h"

namespace Radion::Physics
{

class PointJoint final : public Joint
{
public:
    PointJoint();
    PointJoint(RigidBody& a, RigidBody& b, const glm::vec3& worldAnchor);
    PointJoint(RigidBody& a, const glm::vec3& localAnchorA, RigidBody& b,
               const glm::vec3& localAnchorB);
    PointJoint(PointJoint&& other) noexcept;
    void configure(RigidBody& a, RigidBody& b, const glm::vec3& worldAnchor);
    void rebuild() override;

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    glm::vec3 anchorWorldA() const override;
    glm::vec3 anchorWorldB() const override;

    const glm::vec3& localAnchorA() const;
    const glm::vec3& localAnchorB() const;
    glm::vec3 worldAnchorA() const;
    glm::vec3 worldAnchorB() const;
    void setMotor(const glm::vec3& localAxisA, f32 targetVelocity, f32 maxTorque);
    void disableMotor();

private:
    void calculateProperties();
    void applyVelocityImpulse(const glm::vec3& impulse);
    void applyMotorImpulse(f32 impulse);

    // Null until rebuild() resolves them - see HingeJoint.
    RigidBody* mBodyA = nullptr;
    RigidBody* mBodyB = nullptr;
    glm::vec3 mLocalAnchorA;
    glm::vec3 mLocalAnchorB;
    glm::vec3 mArmA{0.0f};
    glm::vec3 mArmB{0.0f};
    glm::mat3 mEffectiveMass{0.0f};
    glm::vec3 mTotalImpulse{0.0f};
    glm::vec3 mMotorLocalAxisA{1.0f, 0.0f, 0.0f};
    glm::vec3 mMotorWorldAxis{1.0f, 0.0f, 0.0f};
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
