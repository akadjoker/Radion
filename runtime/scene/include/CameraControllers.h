#ifndef RADION_CAMERA_CONTROLLERS_H
#define RADION_CAMERA_CONTROLLERS_H

#include "Component.h"
#include "Input.h"

#include <glm/glm.hpp>

namespace Radion
{

class TriangleOctree;

class FreeFly final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::FreeFly;

    enum class Action : u8
    {
        Forward,
        Back,
        Left,
        Right,
        Up,
        Down,
        Sprint,
        Count
    };

    void setMoveSpeed(f32 unitsPerSecond);
    f32 moveSpeed() const;
    // Held Action::Sprint multiplies moveSpeed by this for as long as it is
    // down - Shift+W (the default Sprint key) moving faster than plain W.
    void setSprintMultiplier(f32 multiplier);
    f32 sprintMultiplier() const;
    void setLookSpeed(f32 degreesPerPixel);
    f32 lookSpeed() const;
    void setPitchLimit(f32 degrees);
    f32 pitchLimit() const;
    void setKey(Action action, KeyCode key);
    KeyCode key(Action action) const;
    void setLookButton(MouseButton button);
    MouseButton lookButton() const;
    void setRequireLookButton(bool required);
    bool requiresLookButton() const;
    void setInvertY(bool invert);
    bool invertY() const;

private:
    friend class GameObject;

    FreeFly();
    void onUpdate(f32 deltaTime) override;

    KeyCode mKeys[static_cast<u8>(Action::Count)];
    MouseButton mLookButton = RIGHT;
    f32 mMoveSpeed = 5.0f;
    f32 mSprintMultiplier = 2.0f;
    f32 mLookSpeed = 0.15f;
    f32 mPitchLimit = 89.0f;
    f32 mPitch = 0.0f;
    bool mRequireLookButton = true;
    bool mInvertY = false;
};

class FPS final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::FPS;

    enum class Action : u8
    {
        Forward,
        Back,
        Left,
        Right,
        Up,
        Down,
        Sprint,
        Count
    };

    void setMoveSpeed(f32 unitsPerSecond);
    f32 moveSpeed() const;
    // Held Action::Sprint multiplies moveSpeed by this for as long as it is
    // down - Shift+W (the default Sprint key) moving faster than plain W.
    void setSprintMultiplier(f32 multiplier);
    f32 sprintMultiplier() const;
    void setLookSpeed(f32 degreesPerPixel);
    f32 lookSpeed() const;
    void setPitchLimit(f32 degrees);
    f32 pitchLimit() const;
    void setKey(Action action, KeyCode key);
    KeyCode key(Action action) const;
    void setLookButton(MouseButton button);
    MouseButton lookButton() const;
    void setRequireLookButton(bool required);
    bool requiresLookButton() const;
    void setInvertY(bool invert);
    bool invertY() const;

private:
    friend class GameObject;

    FPS();
    void onUpdate(f32 deltaTime) override;

    KeyCode mKeys[static_cast<u8>(Action::Count)];
    MouseButton mLookButton = RIGHT;
    f32 mMoveSpeed = 5.0f;
    f32 mSprintMultiplier = 2.0f;
    f32 mLookSpeed = 0.15f;
    f32 mPitchLimit = 89.0f;
    f32 mPitch = 0.0f;
    bool mRequireLookButton = true;
    bool mInvertY = false;
};

class Orbit final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Orbit;

    void setTarget(GameObject* target);
    GameObject* target() const;
    void setTargetPoint(const Math::Vec3& point);
    const Math::Vec3& targetPoint() const;
    void setDistance(f32 distance);
    f32 distance() const;
    void setDistanceLimits(f32 minimum, f32 maximum);
    void setYawPitch(f32 yawDegrees, f32 pitchDegrees);
    f32 yaw() const;
    f32 pitch() const;
    void setPitchLimit(f32 degrees);
    f32 pitchLimit() const;
    void setOrbitSpeed(f32 degreesPerPixel);
    f32 orbitSpeed() const;
    void setZoomSpeed(f32 unitsPerStep);
    f32 zoomSpeed() const;
    void setOrbitButton(MouseButton button);
    MouseButton orbitButton() const;
    void setRequireOrbitButton(bool required);
    bool requiresOrbitButton() const;
    void setInvertY(bool invert);
    bool invertY() const;

private:
    friend class GameObject;

    Orbit();
    void onUpdate(f32 deltaTime) override;
    Math::Vec3 currentTarget() const;
    void clampDistance();

    GameObject* mTarget = nullptr;
    Math::Vec3 mTargetPoint = Math::Vec3(0.0f);
    MouseButton mOrbitButton = LEFT;
    f32 mYaw = 0.0f;
    f32 mPitch = 17.0f;
    f32 mPitchLimit = 89.0f;
    f32 mDistance = 10.0f;
    f32 mMinDistance = 1.0f;
    f32 mMaxDistance = 100.0f;
    f32 mOrbitSpeed = 0.3f;
    f32 mZoomSpeed = 1.0f;
    bool mRequireOrbitButton = true;
    bool mInvertY = false;
};

class Maya final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Maya;

    void setTarget(GameObject* target);
    GameObject* target() const;
    void setTargetPoint(const Math::Vec3& point);
    const Math::Vec3& targetPoint() const;
    void setDistance(f32 distance);
    f32 distance() const;
    void setDistanceLimits(f32 minimum, f32 maximum);
    void setYawPitch(f32 yawDegrees, f32 pitchDegrees);
    f32 yaw() const;
    f32 pitch() const;
    void setPitchLimit(f32 degrees);
    f32 pitchLimit() const;
    void setOrbitSpeed(f32 degreesPerPixel);
    f32 orbitSpeed() const;
    void setPanSpeed(f32 unitsPerPixel);
    f32 panSpeed() const;
    void setZoomSpeed(f32 unitsPerStep);
    f32 zoomSpeed() const;
    void setModifierKey(KeyCode key);
    KeyCode modifierKey() const;
    void setOrbitButton(MouseButton button);
    MouseButton orbitButton() const;
    void setPanButton(MouseButton button);
    MouseButton panButton() const;
    void setDollyButton(MouseButton button);
    MouseButton dollyButton() const;
    void setInvertY(bool invert);
    bool invertY() const;

private:
    friend class GameObject;

    Maya();
    void onUpdate(f32 deltaTime) override;
    Math::Vec3 currentTarget() const;
    void clampDistance();

    GameObject* mTarget = nullptr;
    Math::Vec3 mTargetPoint = Math::Vec3(0.0f);
    KeyCode mModifierKey = KEY_LEFT_ALT;
    MouseButton mOrbitButton = LEFT;
    MouseButton mPanButton = MIDDLE;
    MouseButton mDollyButton = RIGHT;
    f32 mYaw = 0.0f;
    f32 mPitch = 17.0f;
    f32 mPitchLimit = 89.0f;
    f32 mDistance = 10.0f;
    f32 mMinDistance = 1.0f;
    f32 mMaxDistance = 100.0f;
    f32 mOrbitSpeed = 0.3f;
    f32 mPanSpeed = 0.01f;
    f32 mZoomSpeed = 1.0f;
    bool mInvertY = false;
};

class ThirdPerson final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::ThirdPerson;

    void setTarget(GameObject* target);
    GameObject* target() const;
    void setLookSpeed(f32 degreesPerPixel);
    f32 lookSpeed() const;
    void setPitchLimits(f32 minDegrees, f32 maxDegrees);
    f32 minPitch() const;
    f32 maxPitch() const;
    void setDistance(f32 distance);
    f32 distance() const;
    // Mouse-wheel zoom, the same setZoomSpeed()/setDistanceLimits() pattern
    // Orbit/Maya already use - wheel input is read every onUpdate()
    // regardless, so these only need setting once to take effect.
    void setZoomSpeed(f32 speed);
    f32 zoomSpeed() const;
    void setDistanceLimits(f32 minDistance, f32 maxDistance);
    f32 minDistance() const;
    f32 maxDistance() const;
    void setHeightOffset(f32 height);
    f32 heightOffset() const;
    void setShoulderOffset(f32 offset);
    f32 shoulderOffset() const;
    void setSmoothTime(f32 seconds);
    f32 smoothTime() const;
    void setInvertY(bool invert);
    bool invertY() const;
    void setYawPitch(f32 yawDegrees, f32 pitchDegrees);
    f32 yaw() const;
    f32 pitch() const;
    void snap();

    // Sphere-swept collision against a level's collision mesh, the same
    // primitive CharacterController::setOctree() already uses - nullptr (the
    // default) keeps the orbit uncollided, same as before this existed. Set
    // once after the octree is built; the component only ever reads it.
    void setCollisionOctree(const TriangleOctree* octree);
    const TriangleOctree* collisionOctree() const;
    void setCollisionRadius(f32 radius);
    f32 collisionRadius() const;
    void setCollisionMargin(f32 margin);
    f32 collisionMargin() const;

private:
    friend class GameObject;

    ThirdPerson();
    void onUpdate(f32 deltaTime) override;
    Math::Vec3 desiredPosition() const;
    Math::Vec3 aimPoint() const;
    // Pulls `desired` back toward `anchor` when the sphere swept between them
    // hits the collision octree - see slideCamera() in
    // collision/CollisionShape.cpp for the physics-world twin this mirrors.
    Math::Vec3 collide(const Math::Vec3& anchor, const Math::Vec3& desired) const;

    GameObject* mTarget = nullptr;
    Math::Vec3 mVelocity = Math::Vec3(0.0f);
    f32 mYaw = 0.0f;
    f32 mPitch = 20.0f;
    f32 mMinPitch = -30.0f;
    f32 mMaxPitch = 70.0f;
    f32 mDistance = 5.0f;
    f32 mMinDistance = 1.0f;
    f32 mMaxDistance = 20.0f;
    f32 mZoomSpeed = 1.0f;
    f32 mHeightOffset = 1.6f;
    f32 mShoulderOffset = 0.0f;
    f32 mLookSpeed = 0.15f;
    f32 mSmoothTime = 0.35f;
    bool mInvertY = false;
    const TriangleOctree* mCollisionOctree = nullptr;
    f32 mCollisionRadius = 0.25f;
    f32 mCollisionMargin = 0.05f;
};

} // namespace Radion

#endif // RADION_CAMERA_CONTROLLERS_H
