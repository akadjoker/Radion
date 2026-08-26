#include "PCH.h"

#include "character/CharacterBody.h"

#include "collision/CollisionShape.h"

namespace Radion::Physics
{

namespace
{
constexpr f32 kEpsilon = 1.0e-6f;
} // namespace

void CharacterBody::setShape(f32 radius, f32 height)
{
    mRadius = Math::max(radius, 0.01f);
    mHeight = Math::max(height, 0.0f);
}

void CharacterBody::setSlopeLimit(f32 degrees)
{
    mSlopeLimitDegrees = Math::clamp(degrees, 0.0f, 89.0f);
    mSlopeLimitCosine = std::cos(Math::radians(mSlopeLimitDegrees));
}

Math::mat4 CharacterBody::transform() const
{
    Math::mat4 result(1.0f);
    result[3] = Math::vec4(mPosition, 1.0f);
    return result;
}

void CharacterBody::jump(f32 speed)
{
    if (!mGrounded)
        return;
    mVerticalSpeed = speed;
    mGrounded = false;
}

void CharacterBody::teleport(const Math::vec3& position)
{
    mPosition = position;
    mVerticalSpeed = 0.0f;
    mVelocity = Math::vec3(0.0f);
    mGrounded = false;
    mOnWall = false;
    mOnCeiling = false;
    mGroundNormal = mUpDirection;
    mWallNormal = Math::vec3(0.0f);
    mCeilingNormal = Math::vec3(0.0f);
}

f32 CharacterBody::slopeAngle() const
{
    if (!mGrounded)
        return 0.0f;
    return Math::degrees(std::acos(Math::clamp(Math::dot(mGroundNormal, mUpDirection), -1.0f, 1.0f)));
}

Math::vec3 CharacterBody::radii() const
{
    const f32 half = mHeight * 0.5f + mRadius;
    return Math::vec3(mRadius, half, mRadius);
}

CharacterBody::Slide CharacterBody::slide(const Math::vec3& startCentre,
                                          const Math::vec3& displacement, const TrimeshShape& mesh,
                                          const Math::mat4& meshTransform) const
{
    Slide out;
    out.centre = startCentre;
    out.velocity = displacement;

    const Math::mat3 rotation(meshTransform);
    const Math::mat3 inverseRotation = Math::transpose(rotation);
    const Math::vec3 meshOrigin(meshTransform[3]);
    const Math::vec3 extents = radii();
    const Math::vec3 up = mUpDirection;

    // World-space bookkeeping only; projecting the destination with the
    // ellipsoid-space normal leaves a residual whose vertical component gets
    // amplified on the way back and launches the character off walls.
    Math::vec3 center = startCentre;
    Math::vec3 velocity = displacement;
    Math::vec3 outputVelocity = displacement;
    Math::vec3 endpoint = startCentre + displacement;
    const Math::vec3 inputHorizontal = displacement - up * Math::dot(displacement, up);

    bool resolvedOverlap = false;
    u32 iteration = 0;
    for (; iteration < mMaxIterations; ++iteration)
    {
        if (Math::dot(velocity, velocity) < 1e-12f)
            break;

        TrimeshShape::SweepHit hit;
        if (!mesh.sweepEllipsoid(inverseRotation * (center - meshOrigin), extents,
                                 inverseRotation * velocity, hit))
        {
            center = center + velocity;
            break;
        }

        out.collided = true;
        const Math::vec3 n = Math::normalize(rotation * hit.normal);
        const f32 upDot = Math::dot(n, up);

        // Floor within the slope limit, ceiling when facing along -up, wall
        // otherwise; each keeps the most extreme normal seen for the getters.
        if (upDot >= mSlopeLimitCosine)
        {
            out.grounded = true;
            if (upDot > Math::dot(out.groundNormal, up))
                out.groundNormal = n;
        }
        else if (upDot <= -mSlopeLimitCosine)
        {
            out.onCeiling = true;
            if (!out.onCeiling || upDot < Math::dot(out.ceilingNormal, up))
                out.ceilingNormal = n;
        }
        else
        {
            out.steepBlock = true;
            if (!out.onWall || upDot < Math::dot(out.wallNormal, up))
                out.wallNormal = n;
            out.onWall = true;
        }

        // One push-out per move: iterating an embedded shove is how a wedged
        // character is thrown across the level in a single frame.
        if (hit.t < 0.0f)
        {
            if (resolvedOverlap)
                break;
            resolvedOverlap = true;
        }

        center = (hit.t > 0.0f) ? center + velocity * hit.t : center - n * hit.t;
        center = center + n * mSkinWidth;

        // A wall steeper than the limit cannot be climbed: drop the upward
        // component so the slide goes down it, not up.
        outputVelocity = outputVelocity - n * Math::dot(outputVelocity, n);
        if (upDot < mSlopeLimitCosine)
        {
            const f32 upSpeed = Math::dot(outputVelocity, up);
            if (upSpeed > 0.0f)
                outputVelocity -= up * upSpeed;
        }
        else if (upDot <= -mSlopeLimitCosine && !mSlideOnCeiling)
        {
            const f32 upSpeed = Math::dot(outputVelocity, up);
            if (upSpeed > 0.0f)
                outputVelocity -= up * upSpeed;
        }

        // Motion straight along -up onto a slope stops instead of sliding
        // down its face.
        if (mFloorStopOnSlope && out.grounded && Math::length(outputVelocity) > 1e-5f &&
            Math::length(Math::normalize(outputVelocity) + up) < 0.02f)
        {
            outputVelocity = Math::vec3(0.0f);
            endpoint = center;
            break;
        }

        // On the floor, pressing into a wall within this angle of head-on
        // stops instead of sliding along it. Zero keeps the plain slide.
        if (mWallMinSlideAngleDegrees > 0.0f && out.onWall && out.grounded &&
            Math::length(inputHorizontal) > 1e-6f)
        {
            const Math::vec3 wallHorizontal = out.wallNormal - up * Math::dot(out.wallNormal, up);
            if (Math::length(wallHorizontal) > 1e-6f)
            {
                const f32 angle = std::acos(Math::clamp(
                    -Math::dot(Math::normalize(wallHorizontal), Math::normalize(inputHorizontal)),
                    -1.0f, 1.0f));
                if (angle < mWallMinSlideAngleDegrees)
                {
                    const Math::vec3 horizontal = outputVelocity - up * Math::dot(outputVelocity, up);
                    outputVelocity -= horizontal;
                }
            }
        }

        // Destination projected onto the sliding plane, not the leftover
        // velocity - the projection of the leftover gives a shorter slide.
        endpoint = endpoint - n * Math::dot(endpoint - center, n);
        velocity = endpoint - center;
    }

    out.centre = center;
    out.velocity = outputVelocity;
    return out;
}

bool CharacterBody::stepUp(const Math::vec3& startCentre, const Math::vec3& horizontal,
                           const TrimeshShape& mesh, const Math::mat4& meshTransform,
                           Slide& out) const
{
    if (mStepOffset <= 0.0f || Math::dot(horizontal, horizontal) < kEpsilon)
        return false;

    const Math::mat3 rotation(meshTransform);
    const Math::mat3 inverseRotation = Math::transpose(rotation);
    const Math::vec3 meshOrigin(meshTransform[3]);
    const Math::vec3 extents = radii();

    // Anything overhead means there is no room to climb.
    const Math::vec3 up = mUpDirection * mStepOffset;
    TrimeshShape::SweepHit upHit;
    if (mesh.sweepEllipsoid(inverseRotation * (startCentre - meshOrigin), extents,
                            inverseRotation * up, upHit))
        return false;

    // The raised path has to be clear, or a wall taller than the step lets
    // the climb through and the character ratchets up it a step per frame.
    const Slide across = slide(startCentre + up, horizontal, mesh, meshTransform);
    if (across.collided)
        return false;

    // Nothing underneath is a gap, not a step.
    const Math::vec3 down = mUpDirection * -(mStepOffset + mSkinWidth);
    TrimeshShape::SweepHit downHit;
    if (!mesh.sweepEllipsoid(inverseRotation * (across.centre - meshOrigin), extents,
                             inverseRotation * down, downHit))
        return false;

    const Math::vec3 normal = Math::normalize(rotation * downHit.normal);
    if (Math::dot(normal, mUpDirection) < mSlopeLimitCosine)
        return false;

    out = across;
    out.centre = across.centre + down * Math::max(downHit.t, 0.0f) + normal * mSkinWidth;
    out.collided = true;
    out.grounded = true;
    out.groundNormal = normal;
    return true;
}

bool CharacterBody::snapToGround(const TrimeshShape& mesh, const Math::mat4& meshTransform)
{
    if (mGroundSnapDistance <= 0.0f)
        return false;

    const Math::mat3 rotation(meshTransform);
    const Math::mat3 inverseRotation = Math::transpose(rotation);
    const Math::vec3 meshOrigin(meshTransform[3]);
    const Math::vec3 extents = radii();
    const Math::vec3 down = mUpDirection * -mGroundSnapDistance;

    TrimeshShape::SweepHit hit;
    if (!mesh.sweepEllipsoid(inverseRotation * (mPosition - meshOrigin), extents,
                             inverseRotation * down, hit))
        return false;

    const Math::vec3 normal = Math::normalize(rotation * hit.normal);
    if (Math::dot(normal, mUpDirection) < mSlopeLimitCosine)
        return false;

    // Only downward hits ahead of us; a negative t is an overlap the slide
    // has just dealt with.
    if (hit.t < 0.0f || hit.t > 1.0f)
        return false;

    mPosition += down * hit.t + normal * mSkinWidth;
    mGrounded = true;
    mGroundNormal = normal;
    return true;
}

bool CharacterBody::isNearGround(const TrimeshShape& mesh, const Math::mat4& meshTransform,
                                 f32 maxDistance) const
{
    if (maxDistance <= 0.0f)
        return false;

    const Math::mat3 rotation(meshTransform);
    const Math::mat3 inverseRotation = Math::transpose(rotation);
    const Math::vec3 meshOrigin(meshTransform[3]);
    const Math::vec3 extents = radii();
    const Math::vec3 down = mUpDirection * -maxDistance;

    TrimeshShape::SweepHit hit;
    if (!mesh.sweepEllipsoid(inverseRotation * (mPosition - meshOrigin), extents,
                             inverseRotation * down, hit))
        return false;
    if (hit.t < 0.0f || hit.t > 1.0f)
        return false;

    const Math::vec3 normal = Math::normalize(rotation * hit.normal);
    return Math::dot(normal, mUpDirection) >= mSlopeLimitCosine;
}

CharacterBody::MoveResult CharacterBody::move(const Math::vec3& displacement,
                                              const TrimeshShape& mesh,
                                              const Math::mat4& meshTransform)
{
    MoveResult result;
    const Math::vec3 start = mPosition;
    const Math::vec3 up = mUpDirection;
    const Math::vec3 horizontal = displacement - up * Math::dot(displacement, up);

    Slide primary = slide(start, displacement, mesh, meshTransform);

    // Climb only when the horizontal move was actually stopped: a platform
    // seam is an edge, an edge contact reads steep even on level floor, and
    // firing the climb there is a hop at every junction.
    const Math::vec3 achieved = primary.centre - start;
    const Math::vec3 achievedHorizontal = achieved - up * Math::dot(achieved, up);
    const f32 wanted = Math::dot(horizontal, horizontal);
    const bool reallyBlocked =
        wanted > kEpsilon && Math::dot(achievedHorizontal, horizontal) < wanted * 0.5f;
    if (primary.steepBlock && reallyBlocked)
    {
        Slide climbed;
        if (stepUp(start, horizontal, mesh, meshTransform, climbed))
            primary = climbed;
    }

    mPosition = primary.centre;
    mGrounded = primary.grounded;
    mGroundNormal = primary.grounded ? primary.groundNormal : up;
    mOnWall = primary.onWall;
    mOnCeiling = primary.onCeiling;
    mWallNormal = primary.onWall ? primary.wallNormal : Math::vec3(0.0f);
    mCeilingNormal = primary.onCeiling ? primary.ceilingNormal : Math::vec3(0.0f);

    result.collided = primary.collided;
    result.grounded = primary.grounded;
    result.blocked = primary.steepBlock;
    result.onWall = primary.onWall;
    result.onCeiling = primary.onCeiling;
    result.groundNormal = mGroundNormal;
    result.wallNormal = mWallNormal;
    result.ceilingNormal = mCeilingNormal;
    result.remaining = primary.velocity;
    return result;
}

bool CharacterBody::applyFloorSnap(const TrimeshShape& mesh, const Math::mat4& meshTransform)
{
    return snapToGround(mesh, meshTransform);
}

CharacterBody::MoveResult CharacterBody::moveAndSlide(f32 deltaTime, const TrimeshShape& mesh,
                                                      const Math::mat4& meshTransform)
{
    MoveResult result;
    if (!(deltaTime > 0.0f))
        return result;

    // The caller owns velocity, gravity and jump included; this displaces by
    // velocity*dt and slides.
    const Math::vec3 start = mPosition;
    const Math::vec3 up = mUpDirection;
    const bool wasGrounded = mGrounded;
    const bool velFacingUp = Math::dot(mVelocity, up) > 0.0f;

    result = move(mVelocity * deltaTime, mesh, meshTransform);

    // If the slide just lost a floor the body was standing on, probing down
    // and settling keeps resting contact from flickering.
    if (!result.grounded && wasGrounded && !velFacingUp &&
        Math::dot(mPosition - start, up) <= kEpsilon && snapToGround(mesh, meshTransform))
    {
        result.grounded = true;
        result.groundNormal = mGroundNormal;
    }

    mVelocity = result.remaining / deltaTime;
    mVerticalSpeed = Math::dot(mVelocity, up);
    if (result.grounded && mVerticalSpeed < 0.0f)
        mVerticalSpeed = 0.0f;
    return result;
}

CharacterBody::MoveResult CharacterBody::update(f32 deltaTime, const TrimeshShape& mesh,
                                                const Math::mat4& meshTransform)
{
    MoveResult result;
    if (!(deltaTime > 0.0f))
        return result;

    const Math::vec3 start = mPosition;
    const Math::vec3 up = mUpDirection;

    // Planted while grounded, so standing still does not accumulate downward
    // speed frame after frame.
    if (mGrounded && mVerticalSpeed < 0.0f)
        mVerticalSpeed = 0.0f;

    mVerticalSpeed += mGravity * deltaTime;
    if (mVerticalSpeed < mMaxFallSpeed)
        mVerticalSpeed = mMaxFallSpeed;

    const bool wasGrounded = mGrounded;
    result = move(mMoveInput * deltaTime + up * (mVerticalSpeed * deltaTime), mesh, meshTransform);

    // Only going down, only if standing a moment ago, and only if this frame
    // gained no height: a jump must leave the floor, a cliff walk-off must
    // fall, and snapping mid-climb undoes the climb.
    if (!result.grounded && wasGrounded && mVerticalSpeed <= 0.0f &&
        Math::dot(mPosition - start, up) <= kEpsilon && snapToGround(mesh, meshTransform))
    {
        result.grounded = true;
        result.groundNormal = mGroundNormal;
    }

    // The slid velocity, never the position delta: the delta includes the
    // push-out, and reading that back as speed launches the character off a
    // surface he only brushed.
    mVelocity = result.remaining / deltaTime;
    mVerticalSpeed = Math::dot(mVelocity, up);
    if (result.grounded && mVerticalSpeed < 0.0f)
        mVerticalSpeed = 0.0f;
    return result;
}

} // namespace Radion::Physics
