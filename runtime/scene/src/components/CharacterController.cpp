#include "PCH.h"

#include "CharacterController.h"

#include "GameObject.h"

#include <algorithm>
#include <cmath>

namespace Radion
{

CharacterController::CharacterController()
    : Component(ComponentType::CharacterController, ComponentEventUpdate)
{
}

// ------------------------------------------------------------- configuration

void CharacterController::setOctree(const TriangleOctree* octree)
{
    mOctree = octree;
}
const TriangleOctree* CharacterController::octree() const
{
    return mOctree;
}

void CharacterController::setRadius(f32 radius)
{
    mRadius = std::max(0.01f, radius);
}
f32 CharacterController::radius() const
{
    return mRadius;
}

void CharacterController::setHeight(f32 height)
{
    mHeight = std::max(0.05f, height);
}
f32 CharacterController::height() const
{
    return mHeight;
}

void CharacterController::setStepOffset(f32 offset)
{
    mStepOffset = std::max(0.0f, offset);
}
f32 CharacterController::stepOffset() const
{
    return mStepOffset;
}

void CharacterController::setSlopeLimit(f32 degrees)
{
    mSlopeLimitDegrees = std::clamp(degrees, 0.0f, 89.0f);
}
f32 CharacterController::slopeLimit() const
{
    return mSlopeLimitDegrees;
}

void CharacterController::setSkinWidth(f32 width)
{
    mSkinWidth = std::max(0.0f, width);
}
f32 CharacterController::skinWidth() const
{
    return mSkinWidth;
}

void CharacterController::setGravity(f32 gravity)
{
    mGravity = gravity;
}
f32 CharacterController::gravity() const
{
    return mGravity;
}

void CharacterController::setMaxFallSpeed(f32 speed)
{
    mMaxFallSpeed = std::min(0.0f, speed);
}
f32 CharacterController::maxFallSpeed() const
{
    return mMaxFallSpeed;
}

void CharacterController::setMaxIterations(u32 count)
{
    mMaxIterations = std::max(1u, count);
}
u32 CharacterController::maxIterations() const
{
    return mMaxIterations;
}

// ---------------------------------------------------------------- movement

void CharacterController::setMoveInput(const glm::vec3& moveSpeed)
{
    mMoveInput = glm::vec3(moveSpeed.x, 0.0f, moveSpeed.z);
}
const glm::vec3& CharacterController::moveInput() const
{
    return mMoveInput;
}

void CharacterController::jump(f32 speed)
{
    mVerticalVelocity.y = speed;
    mGrounded = false;
}

void CharacterController::teleport(const glm::vec3& worldPosition)
{
    if (GameObject* owner = this->owner())
        owner->setPosition(worldPosition);
    mVerticalVelocity = glm::vec3(0.0f);
    mVelocity = glm::vec3(0.0f);
    mGrounded = false;
    mGroundNormal = glm::vec3(0.0f, 1.0f, 0.0f);
}

bool CharacterController::isGrounded() const
{
    return mGrounded;
}
const glm::vec3& CharacterController::groundNormal() const
{
    return mGroundNormal;
}
const glm::vec3& CharacterController::velocity() const
{
    return mVelocity;
}

f32 CharacterController::slopeAngle() const
{
    return glm::degrees(
        std::acos(glm::clamp(glm::dot(mGroundNormal, glm::vec3(0.0f, 1.0f, 0.0f)), -1.0f, 1.0f)));
}

// ---------------------------------------------------------------- CollideAndSlide
//
// Port of CCollision::CollideEllipsoid from the study project, working in
// world space: the octree's sweepEllipsoid() does the ellipsoid transform
// internally and hands back a fraction-of-velocity t plus a world normal, so
// every step of the loop below is the study's algorithm with the ellipsoid
// bookkeeping moved into the query.

CharacterController::Slide CharacterController::slide(const glm::vec3& startCenter,
                                                      const glm::vec3& displacement) const
{
    Slide out;
    out.center = startCenter;
    out.velocity = displacement;

    if (!mOctree)
    {
        out.center = startCenter + displacement;
        return out;
    }

    const glm::vec3 radii(mRadius, mHeight * 0.5f, mRadius);
    const glm::vec3 up(0.0f, 1.0f, 0.0f);
    const f32 cosLimit = std::cos(glm::radians(mSlopeLimitDegrees));
    glm::vec3 outputVelocity = displacement;
    glm::vec3 endpoint = startCenter + displacement;

    for (u32 i = 0; i < mMaxIterations; ++i)
    {
        if (glm::dot(out.velocity, out.velocity) < 1e-12f)
            break;

        TriangleOctree::SweepHit hit;
        if (!mOctree->sweepEllipsoid(out.center, radii, out.velocity, hit))
        {
            out.center = out.center + out.velocity;
            break;
        }

        out.collided = true;
        const glm::vec3 n = hit.normal;
        const f32 upDot = glm::dot(n, up);
        if (upDot >= cosLimit)
        {
            out.grounded = true;
            out.groundNormal = n;
        }
        else
        {
            out.steepBlock = true;
        }

        // Move to the contact position (embedded hits push out along the
        // normal instead of forward), then nudge off the surface by the skin.
        out.center = (hit.t > 0.0f) ? out.center + out.velocity * hit.t : out.center - n * hit.t;
        out.center = out.center + n * mSkinWidth;

        // Slide the output velocity off the normal. A wall steeper than the
        // slope limit cannot be climbed: drop its upward component so the
        // character slides down it instead of walking up it.
        outputVelocity = outputVelocity - n * glm::dot(outputVelocity, n);
        if (upDot < cosLimit && outputVelocity.y > 0.0f)
            outputVelocity.y = 0.0f;

        // Project the movement endpoint onto the sliding plane through the
        // new center (the study's eTo adjustment) and continue from there.
        endpoint = endpoint - n * glm::dot(endpoint - out.center, n);
        out.velocity = endpoint - out.center;
    }

    out.velocity = outputVelocity;
    return out;
}

CharacterController::MoveResult CharacterController::move(const glm::vec3& displacement)
{
    MoveResult result;
    GameObject* owner = this->owner();
    if (!owner)
        return result;

    if (!mOctree)
    {
        owner->setPosition(owner->position() + displacement);
        result.displacement = displacement;
        return result;
    }

    const f32 halfHeight = mHeight * 0.5f;
    const glm::vec3 radii(mRadius, halfHeight, mRadius);
    const glm::vec3 up(0.0f, 1.0f, 0.0f);
    const f32 cosLimit = std::cos(glm::radians(mSlopeLimitDegrees));

    const glm::vec3 start = owner->position();
    const glm::vec3 center = start + glm::vec3(0.0f, halfHeight, 0.0f);
    const glm::vec3 horizontal(displacement.x, 0.0f, displacement.z);
    const glm::vec3 vertical(0.0f, displacement.y, 0.0f);

    // Primary pass: slide the whole displacement.
    Slide primary = slide(center, displacement);
    glm::vec3 finalCenter = primary.center;
    glm::vec3 finalDisplacement = primary.velocity;
    result.collided = primary.collided;
    result.grounded = primary.grounded;
    result.normal = primary.groundNormal;

    // Step-up: a steep wall blocked the horizontal move - try climbing over
    // it. Only accept the climb when the raised horizontal path is clear and
    // we land back on something within the step height (no floating).
    if (mStepOffset > 0.0f && primary.steepBlock && glm::dot(horizontal, horizontal) > 1e-10f)
    {
        const glm::vec3 upVec(0.0f, mStepOffset, 0.0f);
        TriangleOctree::SweepHit upHit;
        if (!mOctree->sweepEllipsoid(center, radii, upVec, upHit))
        {
            Slide across = slide(center + upVec, horizontal);
            if (!across.collided)
            {
                const glm::vec3 downVec(0.0f, -mStepOffset, 0.0f);
                TriangleOctree::SweepHit downHit;
                if (mOctree->sweepEllipsoid(across.center, radii, downVec, downHit))
                {
                    const glm::vec3 landed =
                        across.center + downVec * downHit.t + downHit.normal * mSkinWidth;
                    // Keep the vertical (gravity / jump) movement from the
                    // primary move, resolved at the stepped position.
                    Slide final = slide(landed, vertical);
                    finalCenter = final.center;
                    finalDisplacement = vertical;
                    result.collided = true;
                    if (final.grounded || glm::dot(downHit.normal, up) >= cosLimit)
                    {
                        result.grounded = true;
                        result.normal = glm::dot(downHit.normal, up) >= cosLimit
                                            ? downHit.normal
                                            : final.groundNormal;
                    }
                }
            }
        }
    }

    owner->setPosition(finalCenter - glm::vec3(0.0f, halfHeight, 0.0f));
    result.displacement = finalDisplacement;
    return result;
}

// ------------------------------------------------------------------- update

void CharacterController::onUpdate(f32 deltaTime)
{
    if (!mOctree)
        return;
    if (deltaTime <= 0.0f)
        return;

    // Stay planted while grounded instead of accumulating tiny negative drift.
    if (mGrounded && mVerticalVelocity.y < 0.0f)
        mVerticalVelocity.y = 0.0f;

    mVerticalVelocity.y += mGravity * deltaTime;
    if (mVerticalVelocity.y < mMaxFallSpeed)
        mVerticalVelocity.y = mMaxFallSpeed;

    const glm::vec3 displacement = mMoveInput * deltaTime + mVerticalVelocity * deltaTime;

    const MoveResult result = move(displacement);

    mGrounded = result.grounded;
    if (result.grounded)
        mGroundNormal = result.normal;
    mVelocity = deltaTime > 0.0f ? result.displacement / deltaTime : glm::vec3(0.0f);
    // Fold the post-collision residual back in so gravity keeps the velocity
    // honest after a landing.
    mVerticalVelocity.y = mVelocity.y;
    if (mGrounded && mVerticalVelocity.y < 0.0f)
        mVerticalVelocity.y = 0.0f;
}

} // namespace Radion
