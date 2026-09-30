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
        // A surface too steep to walk on stopped the horizontal move.
        bool blocked = false;
        bool onWall = false;
        bool onCeiling = false;
        // Flattest ground found this move - the one the character is really
        // standing on when a move touches two. Up when airborne.
        Math::vec3 groundNormal{0.0f, 1.0f, 0.0f};
        // The steepest wall and the ceiling touched this move, if any.
        Math::vec3 wallNormal{0.0f};
        Math::vec3 ceilingNormal{0.0f};
        // What the slide could not deliver, in world units.
        Math::vec3 remaining{0.0f};
    };

    // `radius` and `height` describe a capsule: height is the INNER segment,
    // so the whole thing stands 2*(radius + height/2) tall. The position is
    // its centre.
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
    // World transform of the capsule, for a contact query or a debug draw.
    Math::mat4 transform() const;

    // Ground steeper than this is a wall: it blocks instead of carrying.
    void setSlopeLimit(f32 degrees);
    f32 slopeLimit() const
    {
        return mSlopeLimitDegrees;
    }
    // Kept this far off every surface, so a capsule resting on the floor is
    // never exactly touching it and the contact does not flicker.
    void setSkinWidth(f32 width)
    {
        mSkinWidth = Math::max(width, 0.0f);
    }
    // Slide passes per move. A corner needs more than one: sliding off the
    // first wall drives into the second.
    void setMaxIterations(u32 iterations)
    {
        mMaxIterations = Math::max(iterations, 1u);
    }

    // Moves by `displacement`, sliding along whatever it meets. The sweep is
    // continuous, so there is nothing to split into substeps - a fast move
    // cannot pass through a wall the way a push-out would let it.
    MoveResult move(const Math::vec3& displacement, const TrimeshShape& mesh,
                    const Math::mat4& meshTransform);

    // Applies gravity, then moves by the horizontal input. This is what a
    // caller drives every frame; move() is the raw one underneath.
    MoveResult update(f32 deltaTime, const TrimeshShape& mesh, const Math::mat4& meshTransform);

    // Caller-owned drive: set the full velocity yourself (gravity included)
    // and call moveAndSlide(). update() is the wrapper for callers who want
    // the controller to own gravity and input.
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

    // The up axis the slope limit, the ground snap and the ceiling test are
    // measured against. Default (0,1,0); a rotated world sets it to its own
    // "up" so the whole controller works on its side or upside down.
    void setUpDirection(const Math::vec3& up)
    {
        mUpDirection = Math::normalize(up);
    }
    const Math::vec3& upDirection() const
    {
        return mUpDirection;
    }

    // Stop the body when its motion is straight along -up on a slope, instead
    // of letting the slide carry it down the face. On by default.
    void setFloorStopOnSlope(bool on)
    {
        mFloorStopOnSlope = on;
    }
    // On the floor, pressing into a wall within this angle of head-on stops
    // the horizontal motion instead of sliding along the wall. Zero (the
    // default) disables it.
    void setWallMinSlideAngle(f32 degrees)
    {
        mWallMinSlideAngleDegrees = Math::clamp(degrees, 0.0f, 90.0f);
        mWallMinSlideAngleCosine = std::cos(Math::radians(mWallMinSlideAngleDegrees));
    }
    f32 wallMinSlideAngle() const
    {
        return mWallMinSlideAngleDegrees;
    }
    // Keep sliding along a ceiling instead of stopping against it. On by
    // default.
    void setSlideOnCeiling(bool on)
    {
        mSlideOnCeiling = on;
    }

    // Horizontal velocity the character is trying to hold, world units per
    // second. Y is ignored - gravity and jump own that axis.
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

    // How far below his feet the character still counts as standing. One
    // frame of falling covers less than the skin width, so without this
    // `grounded` flickers at rest; it also keeps him on the ground walking
    // down stairs and ramps.
    void setGroundSnapDistance(f32 distance)
    {
        mGroundSnapDistance = Math::max(distance, 0.0f);
    }
    f32 groundSnapDistance() const
    {
        return mGroundSnapDistance;
    }

    // How high a ledge the character climbs instead of being stopped by it.
    void setStepOffset(f32 offset)
    {
        mStepOffset = Math::max(offset, 0.0f);
    }
    f32 stepOffset() const
    {
        return mStepOffset;
    }

    // Ignored while airborne, so jump cannot be repeated in mid-air;
    // setVerticalSpeed() is the no-check alternative.
    void jump(f32 speed);
    // Straight there, clearing the fall.
    void teleport(const Math::vec3& position);

    // Probe down by the snap distance and settle onto whatever is there,
    // without moving sideways. Called every update() when the slide loses the
    // floor; exposed so a caller can re-snap after moving the body itself.
    bool applyFloorSnap(const TrimeshShape& mesh, const Math::mat4& meshTransform);

    // Same probe as applyFloorSnap(), but read-only - reports whether a
    // walkable floor sits within `maxDistance` below right now without
    // moving the character. grounded() alone flickers false for one frame
    // on a small step or a threshold the slide's own sweep barely clears
    // before landing back on it; a caller that only wants "close enough to
    // the ground to not look airborne" (a jump/fall animation switch, say)
    // should use this instead of reacting to that flicker.
    bool isNearGround(const TrimeshShape& mesh, const Math::mat4& meshTransform,
                      f32 maxDistance) const;

    // Contact state, kept up to date by update() and moveAndSlide().
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
    // Degrees between the ground and horizontal; 0 when airborne.
    f32 slopeAngle() const;
    // What the last update actually achieved, per second.
    const Math::vec3& velocity() const
    {
        return mVelocity;
    }
    f32 verticalSpeed() const
    {
        return mVerticalSpeed;
    }

private:
    // Pushes the capsule out of everything it overlaps at its current
    // position, reporting the steepest floor it came off.
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

    // The ellipsoid half-extents this capsule is swept as.
    Math::vec3 radii() const;

    Slide slide(const Math::vec3& startCentre, const Math::vec3& displacement,
                const TrimeshShape& mesh, const Math::mat4& meshTransform) const;
    // Up, across, down - the phase order both references use. False when the
    // raised path is blocked or there is nothing to land on.
    bool stepUp(const Math::vec3& startCentre, const Math::vec3& horizontal, const TrimeshShape& mesh,
                const Math::mat4& meshTransform, Slide& out) const;
    // Probes down by the snap distance and settles onto whatever it finds.
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
    // Comfortably more than one frame of falling and more than the skin, so
    // resting contact never lapses. Small enough not to pull him onto ledges
    // he stepped off deliberately.
    f32 mGroundSnapDistance = 0.15f;
    f32 mStepOffset = 0.35f;
    f32 mGravity = -20.0f;
    f32 mMaxFallSpeed = -50.0f;
    f32 mVerticalSpeed = 0.0f;
    // wall-min-slide is 0 = off, so the plain slide is unchanged until a
    // caller opts in.
    f32 mWallMinSlideAngleDegrees = 0.0f;
    f32 mWallMinSlideAngleCosine = 1.0f;
    bool mFloorStopOnSlope = true;
    bool mSlideOnCeiling = true;
    bool mOnWall = false;
    bool mOnCeiling = false;
    // Sixteen, matching the controller this follows. Eight leaves residual
    // penetration in a corner, where sliding off one wall drives into the
    // next and the next.
    u32 mMaxIterations = 16;
    bool mGrounded = false;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_CHARACTER_BODY_H
