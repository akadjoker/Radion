#include "PCH.h"

#include "dynamics/MouseJoint.h"

#include "dynamics/RigidBody.h"

namespace Radion::Physics
{

MouseJoint::MouseJoint(RigidBody& body, const glm::vec3& worldGrabPoint)
    : mBody(&body), mLocalAnchor(body.pointToLocal(worldGrabPoint)), mTarget(worldGrabPoint)
{
    tuneSpring(5.0f, 0.7f);
}

RigidBody* MouseJoint::bodyA() const
{
    return mBody;
}

RigidBody* MouseJoint::bodyB() const
{
    return mBody;
}

bool MouseJoint::singleBody() const
{
    return true;
}

void MouseJoint::setTarget(const glm::vec3& target)
{
    if (!std::isfinite(target.x) || !std::isfinite(target.y) || !std::isfinite(target.z))
        return;
    if (target != mTarget)
    {
        mBody->setAwake(true);
        mTarget = target;
    }
}

const glm::vec3& MouseJoint::target() const
{
    return mTarget;
}

void MouseJoint::setMaxForce(f32 force)
{
    if (std::isfinite(force))
        mMaxForce = glm::max(force, 0.0f);
}

void MouseJoint::setStiffness(f32 stiffness)
{
    if (std::isfinite(stiffness))
        mStiffness = glm::max(stiffness, 0.0f);
}

void MouseJoint::setDamping(f32 damping)
{
    if (std::isfinite(damping))
        mDamping = glm::max(damping, 0.0f);
}

void MouseJoint::tuneSpring(f32 frequencyHz, f32 dampingRatio)
{
    const f32 mass = mBody->inverseMass() > 0.0f ? 1.0f / mBody->inverseMass() : 0.0f;
    const f32 omega = glm::two_pi<f32>() * glm::max(frequencyHz, 0.0f);
    setStiffness(mass * omega * omega);
    setDamping(2.0f * mass * glm::max(dampingRatio, 0.0f) * omega);
}

void MouseJoint::setup(f32 duration)
{
    const f32 h = duration;
    mGamma = h * (mDamping + h * mStiffness);
    if (mGamma != 0.0f)
        mGamma = 1.0f / mGamma;
    const f32 beta = h * mStiffness * mGamma;

    mArm = mBody->directionToWorld(mLocalAnchor);

    glm::mat3 inverseEffectiveMass(0.0f);
    const glm::vec3 axes[] = {glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f),
                              glm::vec3(0.0f, 0.0f, 1.0f)};
    for (u32 axis = 0; axis < 3; ++axis)
    {
        glm::vec3 response = axes[axis] * (mBody->inverseMass() + mGamma);
        response += glm::cross(
            mBody->inverseInertiaTensorWorld() * glm::cross(mArm, axes[axis]), mArm);
        inverseEffectiveMass[axis] = response;
    }
    const f32 determinant = glm::determinant(inverseEffectiveMass);
    if (std::abs(determinant) > 1.0e-9f && std::isfinite(determinant))
        mEffectiveMass = glm::inverse(inverseEffectiveMass);
    else
    {
        mEffectiveMass = glm::mat3(0.0f);
        mTotalImpulse = glm::vec3(0.0f);
    }

    const glm::vec3 error = mBody->position() + mArm - mTarget;
    mSoftBias = error * beta;
    mMaxImpulse = mMaxForce * h;

    // A tensioned spring is not a body at rest: without this, sleep puts the
    // grabbed body back down the frame after the spring wakes it, and the
    // cursor drags nothing.
    if (glm::dot(error, error) > 1.0e-6f)
        mBody->setAwake(true);

    // The reference bleeds a little angular velocity so a grabbed body does
    // not spin forever around the cursor.
    mBody->setAngularVelocity(mBody->angularVelocity() *
                              glm::max(0.0f, 1.0f - 0.02f * (60.0f * h)));

    if (mPreviousDuration > 0.0f)
        mTotalImpulse *= duration / mPreviousDuration;
    else
        mTotalImpulse = glm::vec3(0.0f);
    mPreviousDuration = duration;
}

void MouseJoint::warmStart()
{
    if (!mBody->isDynamic())
        return;
    mBody->setVelocity(mBody->velocity() + mTotalImpulse * mBody->inverseMass());
    mBody->setAngularVelocity(mBody->angularVelocity() +
                              mBody->inverseInertiaTensorWorld() * glm::cross(mArm, mTotalImpulse));
}

void MouseJoint::solveVelocity()
{
    if (!mBody->isDynamic())
        return;

    const glm::vec3 pointVelocity =
        mBody->velocity() + glm::cross(mBody->angularVelocity(), mArm);
    const glm::vec3 impulse =
        mEffectiveMass * (-(pointVelocity + mSoftBias + mGamma * mTotalImpulse));

    const glm::vec3 previous = mTotalImpulse;
    mTotalImpulse += impulse;
    const f32 lengthSquared = glm::dot(mTotalImpulse, mTotalImpulse);
    if (lengthSquared > mMaxImpulse * mMaxImpulse && lengthSquared > 0.0f)
        mTotalImpulse *= mMaxImpulse / std::sqrt(lengthSquared);
    const glm::vec3 applied = mTotalImpulse - previous;

    mBody->setVelocity(mBody->velocity() + applied * mBody->inverseMass());
    mBody->setAngularVelocity(mBody->angularVelocity() +
                              mBody->inverseInertiaTensorWorld() * glm::cross(mArm, applied));
}

void MouseJoint::solvePosition(f32)
{
    // Soft constraint: the spring bias in the velocity pass is the whole
    // correction, exactly as in the reference.
}

}
