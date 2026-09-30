#ifndef RADION_PHYSICS_CHARACTER_BODY_H
#define RADION_PHYSICS_CHARACTER_BODY_H

#include "Math.h"
#include "Types.h"

#include <vector>

namespace Radion::Physics
{

class TrimeshShape;
struct ContactManifold;

// A player or an NPC, moved by collide-and-slide against static world
// geometry.

class CharacterBody
{
public:
    struct MoveResult
    {
        bool collided = false;
        bool grounded = false;
        bool blocked = false;
        bool onWall = false;
        bool onCeiling = false;
        // Flattest ground touched this move; up when airborne.
        Math::vec3 groundNormal{0.0f, 1.0f, 0.0f};
        Math::vec3 wallNormal{0.0f};
        Math::vec3 ceilingNormal{0.0f};
        Math::vec3 remaining{0.0f};
    };

    // `height` is the INNER segment: total height is 2*(radius + height/2); the position is the centre.
    void setShape(f32 radius, f32 height);
    f32 radius() const
    {
        return mRadius;
    }
    f32 height() const
    {
        return mHeight;
    }

    void setPosition(const Math::vec3& position)
    {
        mPosition = position;
    }
    const Math::vec3& position() const
    {
        return mPosition;
    }
    Math::mat4 transform() const;

    // Ground steeper than this is a wall: it blocks instead of carrying.
    void setSlopeLimit(f32 degrees);
    f32 slopeLimit() const
    {
        return mSlopeLimitDegrees;
    }
    // Kept this far off every surface so resting contact does not flicker.
    void setSkinWidth(f32 width)
    {
        mSkinWidth = Math::max(width, 0.0f);
    }
    // Slide passes per move; a corner needs more than one.
    void setMaxIterations(u32 iterations)
    {
        mMaxIterations = Math::max(iterations, 1u);
    }

    // Continuous sweep, so no substeps: a fast move cannot pass through a wall.
    MoveResult move(const Math::vec3& displacement, const TrimeshShape& mesh,
                    const Math::mat4& meshTransform);

    MoveResult update(f32 deltaTime, const TrimeshShape& mesh, const Math::mat4& meshTransform);

    // Caller-owned drive: set the full velocity (gravity included) and call moveAndSlide().
    void setVelocity(const Math::vec3& velocity)
    {
        mVelocity = velocity;
    }
    MoveResult moveAndSlide(f32 deltaTime, const TrimeshShape& mesh,
                            const Math::mat4& meshTransform);
    // No grounded check, so a double jump is the caller's call.
    void setVerticalSpeed(f32 speed)
    {
        mVerticalSpeed = speed;
    }

    // Up axis for the slope limit, ground snap and ceiling test; default (0,1,0). A rotated world sets its own.
    void setUpDirection(const Math::vec3& up)
    {
        mUpDirection = Math::normalize(up);
    }
    const Math::vec3& upDirection() const
    {
        return mUpDirection;
    }

    // Stop when motion is straight along -up on a slope instead of sliding down. On by default.
    void setFloorStopOnSlope(bool on)
    {
        mFloorStopOnSlope = on;
    }
    // On the floor, pressing into a wall within this angle of head-on stops horizontal motion instead of sliding. 0 disables.
    void setWallMinSlideAngle(f32 degrees)
    {
        mWallMinSlideAngleDegrees = Math::clamp(degrees, 0.0f, 90.0f);
        mWallMinSlideAngleCosine = std::cos(Math::radians(mWallMinSlideAngleDegrees));
    }
    f32 wallMinSlideAngle() const
    {
        return mWallMinSlideAngleDegrees;
    }
    void setSlideOnCeiling(bool on)
    {
        mSlideOnCeiling = on;
    }

    // Horizontal velocity to hold, world units/s; Y is ignored (gravity and jump own it).
    void setMoveInput(const Math::vec3& velocity)
    {
        mMoveInput = Math::vec3(velocity.x, 0.0f, velocity.z);
    }
    const Math::vec3& moveInput() const
    {
        return mMoveInput;
    }

    void setGravity(f32 gravity)
    {
        mGravity = gravity;
    }
    f32 gravity() const
    {
        return mGravity;
    }
    void setMaxFallSpeed(f32 speed)
    {
        mMaxFallSpeed = speed;
    }

    // How far below the feet still counts as standing; must exceed one frame of fall or `grounded` flickers at rest.
    void setGroundSnapDistance(f32 distance)
    {
        mGroundSnapDistance = Math::max(distance, 0.0f);
    }
    f32 groundSnapDistance() const
    {
        return mGroundSnapDistance;
    }

    void setStepOffset(f32 offset)
    {
        mStepOffset = Math::max(offset, 0.0f);
    }
    f32 stepOffset() const
    {
        return mStepOffset;
    }

    // Ignored while airborne; setVerticalSpeed() is the no-check alternative.
    void jump(f32 speed);
    void teleport(const Math::vec3& position);

    // Probes down by the snap distance and settles onto the floor without moving sideways; update() calls it when the slide loses the floor.
    bool applyFloorSnap(const TrimeshShape& mesh, const Math::mat4& meshTransform);

    // Read-only applyFloorSnap() probe: is a walkable floor within `maxDistance` below. grounded() flickers false for a frame on
    // small steps, so use this for jump/fall animation switches.
    bool isNearGround(const TrimeshShape& mesh, const Math::mat4& meshTransform,
                      f32 maxDistance) const;

    bool isOnFloor() const
    {
        return mGrounded;
    }
    bool isOnWall() const
    {
        return mOnWall;
    }
    bool isOnCeiling() const
    {
        return mOnCeiling;
    }
    const Math::vec3& floorNormal() const
    {
        return mGroundNormal;
    }
    const Math::vec3& wallNormal() const
    {
        return mWallNormal;
    }
    const Math::vec3& ceilingNormal() const
    {
        return mCeilingNormal;
    }

    bool grounded() const
    {
        return mGrounded;
    }
    const Math::vec3& groundNormal() const
    {
        return mGroundNormal;
    }
    f32 slopeAngle() const;
    const Math::vec3& velocity() const
    {
        return mVelocity;
    }
    f32 verticalSpeed() const
    {
        return mVerticalSpeed;
    }

private:
    // Pushes the capsule out of overlaps, reporting the steepest floor it came off.
    struct Slide
    {
        Math::vec3 centre{0.0f};
        Math::vec3 velocity{0.0f};
        Math::vec3 groundNormal{0.0f, 0.0f, 0.0f};
        Math::vec3 wallNormal{0.0f};
        Math::vec3 ceilingNormal{0.0f};
        bool collided = false;
        bool grounded = false;
        bool onWall = false;
        bool onCeiling = false;
        bool steepBlock = false;
    };

    Math::vec3 radii() const;

    Slide slide(const Math::vec3& startCentre, const Math::vec3& displacement,
                const TrimeshShape& mesh, const Math::mat4& meshTransform) const;
    // Up, across, down phase order; false when the raised path is blocked or there is nothing to land on.
    bool stepUp(const Math::vec3& startCentre, const Math::vec3& horizontal, const TrimeshShape& mesh,
                const Math::mat4& meshTransform, Slide& out) const;
    bool snapToGround(const TrimeshShape& mesh, const Math::mat4& meshTransform);

    Math::vec3 mPosition{0.0f};
    Math::vec3 mUpDirection{0.0f, 1.0f, 0.0f};
    Math::vec3 mGroundNormal{0.0f, 1.0f, 0.0f};
    Math::vec3 mWallNormal{0.0f};
    Math::vec3 mCeilingNormal{0.0f};
    Math::vec3 mMoveInput{0.0f};
    Math::vec3 mVelocity{0.0f};
    f32 mRadius = 0.4f;
    f32 mHeight = 1.2f;
    f32 mSlopeLimitDegrees = 45.0f;
    f32 mSlopeLimitCosine = 0.70710678f;
    f32 mSkinWidth = 0.02f;
    // More than one frame of fall and more than the skin so resting contact never lapses; small enough not to pull onto ledges stepped off.
    f32 mGroundSnapDistance = 0.15f;
    f32 mStepOffset = 0.35f;
    f32 mGravity = -20.0f;
    f32 mMaxFallSpeed = -50.0f;
    f32 mVerticalSpeed = 0.0f;
    // wall-min-slide 0 = off, so the plain slide is unchanged until a caller opts in.
    f32 mWallMinSlideAngleDegrees = 0.0f;
    f32 mWallMinSlideAngleCosine = 1.0f;
    bool mFloorStopOnSlope = true;
    bool mSlideOnCeiling = true;
    bool mOnWall = false;
    bool mOnCeiling = false;
    // Eight leaves residual penetration in corners (sliding off one wall drives into the next).
    u32 mMaxIterations = 16;
    bool mGrounded = false;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_CHARACTER_BODY_H
