#ifndef RADION_CHARACTER_CONTROLLER_H
#define RADION_CHARACTER_CONTROLLER_H

#include "Component.h"
#include "Octree.h"

#include <glm/glm.hpp>

namespace Radion
{


class CharacterController final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::CharacterController;

    void setOctree(const TriangleOctree* octree);
    const TriangleOctree* octree() const;

    void setRadius(f32 radius);
    f32 radius() const;
    void setHeight(f32 height); // total height; vertical radius = height / 2
    f32 height() const;
    void setStepOffset(f32 offset); // how high a step it will climb
    f32 stepOffset() const;
    void setSlopeLimit(f32 degrees); // max walkable slope
    f32 slopeLimit() const;
    void setSkinWidth(f32 width); // pushed this far off a surface at contact
    f32 skinWidth() const;
    void setGravity(f32 gravity); // units/s^2; negative pulls down
    f32 gravity() const;
    void setMaxFallSpeed(f32 speed);
    f32 maxFallSpeed() const;
    void setMaxIterations(u32 count);
    u32 maxIterations() const;

    // Desired horizontal movement per second (XZ). onUpdate() integrates it
    // with deltaTime; the vertical axis is driven by gravity and jump().
    void setMoveInput(const glm::vec3& moveSpeed);
    const glm::vec3& moveInput() const;

    void jump(f32 speed);
    void teleport(const glm::vec3& worldPosition);

    bool isGrounded() const;
    const glm::vec3& groundNormal() const;
    f32 slopeAngle() const; // degrees of the current ground
    // Residual per-second velocity after the last move (what the slide left).
    const glm::vec3& velocity() const;

    struct MoveResult
    {
        bool collided = false;
        bool grounded = false;
        glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
        glm::vec3 displacement = glm::vec3(0.0f); // residual after slides
    };

    // CollideAndSlide `displacement` against the octree, moving the owner.
    // This is what onUpdate() calls internally; expose it for a caller that
    // wants to drive the controller by hand instead of through moveInput.
    MoveResult move(const glm::vec3& displacement);

private:
    friend class GameObject;

    CharacterController();
    void onUpdate(f32 deltaTime) override;

    struct Slide
    {
        glm::vec3 center = glm::vec3(0.0f);
        glm::vec3 velocity = glm::vec3(0.0f);
        bool collided = false;
        bool grounded = false;
        glm::vec3 groundNormal = glm::vec3(0.0f, 1.0f, 0.0f);
        bool steepBlock = false; // a wall (steeper than slopeLimit) stopped us
    };

    // One CollideAndSlide pass: sweep the ellipsoid, slide the velocity off
    // every contact, up to mMaxIterations times. Returns the surviving center
    // and velocity plus grounding info.
    Slide slide(const glm::vec3& startCenter, const glm::vec3& displacement) const;

    const TriangleOctree* mOctree = nullptr;
    f32 mRadius = 0.5f;
    f32 mHeight = 2.0f;
    f32 mStepOffset = 0.35f;
    f32 mSlopeLimitDegrees = 45.0f;
    f32 mSkinWidth = 0.01f;
    f32 mGravity = -20.0f;
    f32 mMaxFallSpeed = -50.0f;
    u32 mMaxIterations = 16;

    glm::vec3 mMoveInput = glm::vec3(0.0f);
    glm::vec3 mVerticalVelocity = glm::vec3(0.0f);
    glm::vec3 mVelocity = glm::vec3(0.0f);
    bool mGrounded = false;
    glm::vec3 mGroundNormal = glm::vec3(0.0f, 1.0f, 0.0f);
};

} // namespace Radion

#endif // RADION_CHARACTER_CONTROLLER_H
