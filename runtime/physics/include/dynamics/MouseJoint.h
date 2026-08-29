#ifndef RADION_PHYSICS_DYNAMICS_MOUSEJOINT_H
#define RADION_PHYSICS_DYNAMICS_MOUSEJOINT_H

#include "Math.h"
#include "dynamics/Joint.h"

namespace Radion::Physics
{

// Drags one anchor point on a body towards a moving world target through a
// soft spring, capped by a maximum force - picking a body up with the cursor
// without teleporting it. Single-body: both joint endpoints report the
// dragged body.
class MouseJoint final : public Joint
{
public:
    MouseJoint(RigidBody& body, const glm::vec3& worldGrabPoint);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    bool singleBody() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;
    void rebuild() override
    {
    }

    glm::vec3 anchorWorldA() const override;
    glm::vec3 anchorWorldB() const override;

    // Wakes the body: a sleeping body under a moving cursor must follow.
    void setTarget(const glm::vec3& target);
    const glm::vec3& target() const;
    void setMaxForce(f32 force);
    void setStiffness(f32 stiffness);
    void setDamping(f32 damping);
    // The usual spring parametrisation on top of raw stiffness: frequency in
    // hertz and a damping ratio, scaled by the body's mass so the feel does
    // not change with what is being dragged.
    void tuneSpring(f32 frequencyHz, f32 dampingRatio);

private:
    RigidBody* mBody;
    glm::vec3 mLocalAnchor;
    glm::vec3 mTarget;
    f32 mMaxForce = 500.0f;
    f32 mStiffness = 0.0f;
    f32 mDamping = 0.0f;

    glm::vec3 mArm{0.0f};
    glm::mat3 mEffectiveMass{0.0f};
    glm::vec3 mSoftBias{0.0f};
    f32 mGamma = 0.0f;
    glm::vec3 mTotalImpulse{0.0f};
    f32 mMaxImpulse = 0.0f;
    f32 mPreviousDuration = 0.0f;
};

}

#endif
