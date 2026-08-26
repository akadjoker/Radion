#include "PCH.h"

#include "CameraControllers.h"

#include "GameObject.h"
#include "Octree.h"
#include "Scene.h"

namespace Radion
{

namespace
{
void configureMovementKeys(KeyCode* keys)
{
    keys[static_cast<u8>(FPS::Action::Forward)] = KEY_W;
    keys[static_cast<u8>(FPS::Action::Back)] = KEY_S;
    keys[static_cast<u8>(FPS::Action::Left)] = KEY_A;
    keys[static_cast<u8>(FPS::Action::Right)] = KEY_D;
    keys[static_cast<u8>(FPS::Action::Up)] = KEY_SPACE;
    keys[static_cast<u8>(FPS::Action::Down)] = KEY_LEFT_CONTROL;
    keys[static_cast<u8>(FPS::Action::Sprint)] = KEY_LEFT_SHIFT;
}

Math::Vec3 orbitOffset(f32 yaw, f32 pitch)
{
    const f32 yawRadians = glm::radians(yaw);
    const f32 pitchRadians = glm::radians(pitch);
    return Math::Vec3(glm::cos(pitchRadians) * glm::sin(yawRadians), glm::sin(pitchRadians),
                     glm::cos(pitchRadians) * glm::cos(yawRadians));
}

Math::Vec3 shoulderRight(f32 yaw)
{
    const f32 yawRadians = glm::radians(yaw);
    return Math::Vec3(glm::cos(yawRadians), 0.0f, -glm::sin(yawRadians));
}

Math::Vec3 springDamp(const Math::Vec3& current, const Math::Vec3& target, Math::Vec3& velocity,
                     f32 smoothTime, f32 dt)
{
    const f32 omega = 2.0f / smoothTime;
    const f32 x = omega * dt;
    const f32 exponent = 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
    const Math::Vec3 change = current - target;
    const Math::Vec3 temp = (velocity + omega * change) * dt;
    velocity = (velocity - omega * temp) * exponent;
    return target + (change + temp) * exponent;
}
} // namespace



FreeFly::FreeFly() : Component(Type, ComponentEventUpdate)
{
    mKeys[static_cast<u8>(Action::Forward)] = KEY_W;
    mKeys[static_cast<u8>(Action::Back)] = KEY_S;
    mKeys[static_cast<u8>(Action::Left)] = KEY_A;
    mKeys[static_cast<u8>(Action::Right)] = KEY_D;
    mKeys[static_cast<u8>(Action::Up)] = KEY_SPACE;
    mKeys[static_cast<u8>(Action::Down)] = KEY_LEFT_CONTROL;
    mKeys[static_cast<u8>(Action::Sprint)] = KEY_LEFT_SHIFT;
}

// Outside Play the editor's gizmo owns the camera pose.
static bool editingBlocksController(const GameObject* object)
{
    return object->scene() && object->scene()->runningInEditor();
}

void FreeFly::onUpdate(f32 deltaTime)
{
    GameObject* object = owner();
    if (editingBlocksController(object))
        return;
    const f32 sprint =
        Input::isKeyDown(mKeys[static_cast<u8>(Action::Sprint)]) ? mSprintMultiplier : 1.0f;
    const f32 distance = mMoveSpeed * sprint * deltaTime;
    const f32 forward = (Input::isKeyDown(mKeys[static_cast<u8>(Action::Forward)]) ? 1.0f : 0.0f) -
                        (Input::isKeyDown(mKeys[static_cast<u8>(Action::Back)]) ? 1.0f : 0.0f);
    const f32 right = (Input::isKeyDown(mKeys[static_cast<u8>(Action::Right)]) ? 1.0f : 0.0f) -
                      (Input::isKeyDown(mKeys[static_cast<u8>(Action::Left)]) ? 1.0f : 0.0f);
    const f32 up = (Input::isKeyDown(mKeys[static_cast<u8>(Action::Up)]) ? 1.0f : 0.0f) -
                   (Input::isKeyDown(mKeys[static_cast<u8>(Action::Down)]) ? 1.0f : 0.0f);

    object->moveForward(forward * distance);
    object->moveRight(right * distance);
    object->moveUp(up * distance);

    if (mRequireLookButton && !Input::isMouseDown(mLookButton))
        return;

    const Math::Vec2 mouseDelta = Input::getMouseDelta();
    const f32 yawDelta = -mouseDelta.x * mLookSpeed;
    f32 pitchDelta = (mInvertY ? mouseDelta.y : -mouseDelta.y) * mLookSpeed;
    const f32 nextPitch = glm::clamp(mPitch + pitchDelta, -mPitchLimit, mPitchLimit);
    pitchDelta = nextPitch - mPitch;
    mPitch = nextPitch;

    object->yaw(yawDelta, TransformSpace::Parent);
    object->pitch(pitchDelta, TransformSpace::Local);
}

void FreeFly::setMoveSpeed(f32 speed)
{
    mMoveSpeed = glm::max(0.0f, speed);
}
f32 FreeFly::moveSpeed() const
{
    return mMoveSpeed;
}
void FreeFly::setSprintMultiplier(f32 multiplier)
{
    mSprintMultiplier = glm::max(0.0f, multiplier);
}
f32 FreeFly::sprintMultiplier() const
{
    return mSprintMultiplier;
}
void FreeFly::setLookSpeed(f32 speed)
{
    mLookSpeed = glm::max(0.0f, speed);
}
f32 FreeFly::lookSpeed() const
{
    return mLookSpeed;
}
void FreeFly::setPitchLimit(f32 degrees)
{
    mPitchLimit = glm::clamp(degrees, 0.0f, 89.9f);
    mPitch = glm::clamp(mPitch, -mPitchLimit, mPitchLimit);
}
f32 FreeFly::pitchLimit() const
{
    return mPitchLimit;
}
void FreeFly::setKey(Action action, KeyCode keyCode)
{
    const u8 index = static_cast<u8>(action);
    if (index < static_cast<u8>(Action::Count))
        mKeys[index] = keyCode;
}
KeyCode FreeFly::key(Action action) const
{
    const u8 index = static_cast<u8>(action);
    return index < static_cast<u8>(Action::Count) ? mKeys[index] : KEY_NULL;
}
void FreeFly::setLookButton(MouseButton button)
{
    mLookButton = button;
}
MouseButton FreeFly::lookButton() const
{
    return mLookButton;
}
void FreeFly::setRequireLookButton(bool required)
{
    mRequireLookButton = required;
}
bool FreeFly::requiresLookButton() const
{
    return mRequireLookButton;
}
void FreeFly::setInvertY(bool invert)
{
    mInvertY = invert;
}
bool FreeFly::invertY() const
{
    return mInvertY;
}


FPS::FPS() : Component(Type, ComponentEventUpdate)
{
    configureMovementKeys(mKeys);
}

void FPS::onUpdate(f32 deltaTime)
{
    GameObject* object = owner();
    if (editingBlocksController(object))
        return;
    Math::Vec3 forward = object->forward();
    Math::Vec3 right = object->right();
    forward.y = 0.0f;
    right.y = 0.0f;
    if (glm::dot(forward, forward) > 0.000001f)
        forward = glm::normalize(forward);
    if (glm::dot(right, right) > 0.000001f)
        right = glm::normalize(right);

    const f32 forwardInput =
        (Input::isKeyDown(mKeys[static_cast<u8>(Action::Forward)]) ? 1.0f : 0.0f) -
        (Input::isKeyDown(mKeys[static_cast<u8>(Action::Back)]) ? 1.0f : 0.0f);
    const f32 rightInput = (Input::isKeyDown(mKeys[static_cast<u8>(Action::Right)]) ? 1.0f : 0.0f) -
                           (Input::isKeyDown(mKeys[static_cast<u8>(Action::Left)]) ? 1.0f : 0.0f);
    const f32 upInput = (Input::isKeyDown(mKeys[static_cast<u8>(Action::Up)]) ? 1.0f : 0.0f) -
                        (Input::isKeyDown(mKeys[static_cast<u8>(Action::Down)]) ? 1.0f : 0.0f);
    const f32 sprint =
        Input::isKeyDown(mKeys[static_cast<u8>(Action::Sprint)]) ? mSprintMultiplier : 1.0f;
    object->translate((forward * forwardInput + right * rightInput + Math::Vec3(0, upInput, 0)) *
                          (mMoveSpeed * sprint * deltaTime),
                      TransformSpace::World);

    if (mRequireLookButton && !Input::isMouseDown(mLookButton))
        return;
    const Math::Vec2 delta = Input::getMouseDelta();
    const f32 yawDelta = -delta.x * mLookSpeed;
    const f32 requestedPitch = (mInvertY ? delta.y : -delta.y) * mLookSpeed;
    const f32 nextPitch = glm::clamp(mPitch + requestedPitch, -mPitchLimit, mPitchLimit);
    object->yaw(yawDelta, TransformSpace::Parent);
    object->pitch(nextPitch - mPitch, TransformSpace::Local);
    mPitch = nextPitch;
}

void FPS::setMoveSpeed(f32 speed)
{
    mMoveSpeed = glm::max(0.0f, speed);
}
f32 FPS::moveSpeed() const
{
    return mMoveSpeed;
}
void FPS::setSprintMultiplier(f32 multiplier)
{
    mSprintMultiplier = glm::max(0.0f, multiplier);
}
f32 FPS::sprintMultiplier() const
{
    return mSprintMultiplier;
}
void FPS::setLookSpeed(f32 speed)
{
    mLookSpeed = glm::max(0.0f, speed);
}
f32 FPS::lookSpeed() const
{
    return mLookSpeed;
}
void FPS::setPitchLimit(f32 degrees)
{
    mPitchLimit = glm::clamp(degrees, 0.0f, 89.9f);
    mPitch = glm::clamp(mPitch, -mPitchLimit, mPitchLimit);
}
f32 FPS::pitchLimit() const
{
    return mPitchLimit;
}
void FPS::setKey(Action action, KeyCode keyCode)
{
    const u8 index = static_cast<u8>(action);
    if (index < static_cast<u8>(Action::Count))
        mKeys[index] = keyCode;
}
KeyCode FPS::key(Action action) const
{
    const u8 index = static_cast<u8>(action);
    return index < static_cast<u8>(Action::Count) ? mKeys[index] : KEY_NULL;
}
void FPS::setLookButton(MouseButton button)
{
    mLookButton = button;
}
MouseButton FPS::lookButton() const
{
    return mLookButton;
}
void FPS::setRequireLookButton(bool required)
{
    mRequireLookButton = required;
}
bool FPS::requiresLookButton() const
{
    return mRequireLookButton;
}
void FPS::setInvertY(bool invert)
{
    mInvertY = invert;
}
bool FPS::invertY() const
{
    return mInvertY;
}

Orbit::Orbit() : Component(Type, ComponentEventUpdate)
{
}

void Orbit::onUpdate(f32)
{
    if (editingBlocksController(owner()))
        return;
    if (!mRequireOrbitButton || Input::isMouseDown(mOrbitButton))
    {
        const Math::Vec2 delta = Input::getMouseDelta();
        mYaw += delta.x * mOrbitSpeed;
        mPitch = glm::clamp(mPitch + (mInvertY ? -delta.y : delta.y) * mOrbitSpeed, -mPitchLimit,
                            mPitchLimit);
    }
    mDistance -= Input::getMouseWheelMoveV() * mZoomSpeed;
    clampDistance();
    const Math::Vec3 point = currentTarget();
    owner()->setGlobalPosition(point + orbitOffset(mYaw, mPitch) * mDistance);
    owner()->lookAt(point);
}
void Orbit::setTarget(GameObject* target)
{
    mTarget = target;
}
GameObject* Orbit::target() const
{
    return mTarget;
}
void Orbit::setTargetPoint(const Math::Vec3& point)
{
    mTarget = nullptr;
    mTargetPoint = point;
}
const Math::Vec3& Orbit::targetPoint() const
{
    return mTargetPoint;
}
void Orbit::setDistance(f32 distance)
{
    mDistance = distance;
    clampDistance();
}
f32 Orbit::distance() const
{
    return mDistance;
}
void Orbit::setDistanceLimits(f32 minimum, f32 maximum)
{
    mMinDistance = glm::max(0.0f, minimum);
    mMaxDistance = glm::max(mMinDistance, maximum);
    clampDistance();
}
void Orbit::setYawPitch(f32 yaw, f32 pitch)
{
    mYaw = yaw;
    mPitch = glm::clamp(pitch, -mPitchLimit, mPitchLimit);
}
f32 Orbit::yaw() const
{
    return mYaw;
}
f32 Orbit::pitch() const
{
    return mPitch;
}
void Orbit::setPitchLimit(f32 degrees)
{
    mPitchLimit = glm::clamp(degrees, 0.0f, 89.9f);
    mPitch = glm::clamp(mPitch, -mPitchLimit, mPitchLimit);
}
f32 Orbit::pitchLimit() const
{
    return mPitchLimit;
}
void Orbit::setOrbitSpeed(f32 speed)
{
    mOrbitSpeed = glm::max(0.0f, speed);
}
f32 Orbit::orbitSpeed() const
{
    return mOrbitSpeed;
}
void Orbit::setZoomSpeed(f32 speed)
{
    mZoomSpeed = glm::max(0.0f, speed);
}
f32 Orbit::zoomSpeed() const
{
    return mZoomSpeed;
}
void Orbit::setOrbitButton(MouseButton button)
{
    mOrbitButton = button;
}
MouseButton Orbit::orbitButton() const
{
    return mOrbitButton;
}
void Orbit::setRequireOrbitButton(bool required)
{
    mRequireOrbitButton = required;
}
bool Orbit::requiresOrbitButton() const
{
    return mRequireOrbitButton;
}
void Orbit::setInvertY(bool invert)
{
    mInvertY = invert;
}
bool Orbit::invertY() const
{
    return mInvertY;
}
Math::Vec3 Orbit::currentTarget() const
{
    return mTarget ? mTarget->globalPosition() : mTargetPoint;
}
void Orbit::clampDistance()
{
    mDistance = glm::clamp(mDistance, mMinDistance, mMaxDistance);
}

Maya::Maya() : Component(Type, ComponentEventUpdate)
{
}

void Maya::onUpdate(f32)
{
    if (editingBlocksController(owner()))
        return;
    Math::Vec3 point = currentTarget();
    if (Input::isKeyDown(mModifierKey) && Input::isMouseDown(mOrbitButton))
    {
        const Math::Vec2 delta = Input::getMouseDelta();
        mYaw += delta.x * mOrbitSpeed;
        mPitch = glm::clamp(mPitch + (mInvertY ? -delta.y : delta.y) * mOrbitSpeed, -mPitchLimit,
                            mPitchLimit);
    }
    else if (Input::isKeyDown(mModifierKey) && Input::isMouseDown(mPanButton))
    {
        const Math::Vec2 delta = Input::getMouseDelta();
        point += owner()->right() * (-delta.x * mPanSpeed) + owner()->up() * (delta.y * mPanSpeed);
        setTargetPoint(point);
    }
    else if (Input::isKeyDown(mModifierKey) && Input::isMouseDown(mDollyButton))
        mDistance -= Input::getMouseDelta().x * mZoomSpeed * 0.05f;

    mDistance -= Input::getMouseWheelMoveV() * mZoomSpeed;
    clampDistance();
    owner()->setGlobalPosition(point + orbitOffset(mYaw, mPitch) * mDistance);
    owner()->lookAt(point);
}
void Maya::setTarget(GameObject* target)
{
    mTarget = target;
}
GameObject* Maya::target() const
{
    return mTarget;
}
void Maya::setTargetPoint(const Math::Vec3& point)
{
    mTarget = nullptr;
    mTargetPoint = point;
}
const Math::Vec3& Maya::targetPoint() const
{
    return mTargetPoint;
}
void Maya::setDistance(f32 distance)
{
    mDistance = distance;
    clampDistance();
}
f32 Maya::distance() const
{
    return mDistance;
}
void Maya::setDistanceLimits(f32 minimum, f32 maximum)
{
    mMinDistance = glm::max(0.0f, minimum);
    mMaxDistance = glm::max(mMinDistance, maximum);
    clampDistance();
}
void Maya::setYawPitch(f32 yaw, f32 pitch)
{
    mYaw = yaw;
    mPitch = glm::clamp(pitch, -mPitchLimit, mPitchLimit);
}
f32 Maya::yaw() const
{
    return mYaw;
}
f32 Maya::pitch() const
{
    return mPitch;
}
void Maya::setPitchLimit(f32 degrees)
{
    mPitchLimit = glm::clamp(degrees, 0.0f, 89.9f);
    mPitch = glm::clamp(mPitch, -mPitchLimit, mPitchLimit);
}
f32 Maya::pitchLimit() const
{
    return mPitchLimit;
}
void Maya::setOrbitSpeed(f32 speed)
{
    mOrbitSpeed = glm::max(0.0f, speed);
}
f32 Maya::orbitSpeed() const
{
    return mOrbitSpeed;
}
void Maya::setPanSpeed(f32 speed)
{
    mPanSpeed = glm::max(0.0f, speed);
}
f32 Maya::panSpeed() const
{
    return mPanSpeed;
}
void Maya::setZoomSpeed(f32 speed)
{
    mZoomSpeed = glm::max(0.0f, speed);
}
f32 Maya::zoomSpeed() const
{
    return mZoomSpeed;
}
void Maya::setModifierKey(KeyCode key)
{
    mModifierKey = key;
}
KeyCode Maya::modifierKey() const
{
    return mModifierKey;
}
void Maya::setOrbitButton(MouseButton button)
{
    mOrbitButton = button;
}
MouseButton Maya::orbitButton() const
{
    return mOrbitButton;
}
void Maya::setPanButton(MouseButton button)
{
    mPanButton = button;
}
MouseButton Maya::panButton() const
{
    return mPanButton;
}
void Maya::setDollyButton(MouseButton button)
{
    mDollyButton = button;
}
MouseButton Maya::dollyButton() const
{
    return mDollyButton;
}
void Maya::setInvertY(bool invert)
{
    mInvertY = invert;
}
bool Maya::invertY() const
{
    return mInvertY;
}
Math::Vec3 Maya::currentTarget() const
{
    return mTarget ? mTarget->globalPosition() : mTargetPoint;
}
void Maya::clampDistance()
{
    mDistance = glm::clamp(mDistance, mMinDistance, mMaxDistance);
}

ThirdPerson::ThirdPerson() : Component(Type, ComponentEventUpdate)
{
}

void ThirdPerson::onUpdate(f32 deltaTime)
{
    if (editingBlocksController(owner()))
        return;
    if (!mTarget)
        return;

    // Mouse right turns the view right, mouse up looks up (mInvertY off).
    // Yaw matches FreeFly::onUpdate() above (both rotate the same way), but
    // pitch does not: FreeFly rotates its own object directly, while
    // ThirdPerson orbits a position around the target and looks back at it -
    // orbitOffset()'s sin(pitch) term puts the camera ABOVE the target as
    // pitch rises, which looks DOWN, the opposite of FreeFly's own pitch()
    // call. Matching FreeFly's raw sign here (as an earlier pass did) matched
    // the input math but inverted the actual view.
    const Math::Vec2 delta = Input::getMouseDelta();
    mYaw -= delta.x * mLookSpeed;
    mPitch = glm::clamp(mPitch + (mInvertY ? -delta.y : delta.y) * mLookSpeed, mMinPitch, mMaxPitch);
    mDistance = glm::clamp(mDistance - Input::getMouseWheelMoveV() * mZoomSpeed, mMinDistance,
                           mMaxDistance);

    const Math::Vec3 anchor = aimPoint();
    const Math::Vec3 target = mCollisionOctree ? collide(anchor, desiredPosition()) : desiredPosition();
    const Math::Vec3 current = owner()->globalPosition();
    owner()->setGlobalPosition(springDamp(current, target, mVelocity, mSmoothTime, deltaTime));
    owner()->lookAt(aimPoint());
}
void ThirdPerson::setTarget(GameObject* target)
{
    mTarget = target;
}
GameObject* ThirdPerson::target() const
{
    return mTarget;
}
void ThirdPerson::setLookSpeed(f32 speed)
{
    mLookSpeed = glm::max(0.0f, speed);
}
f32 ThirdPerson::lookSpeed() const
{
    return mLookSpeed;
}
void ThirdPerson::setPitchLimits(f32 minDegrees, f32 maxDegrees)
{
    mMinPitch = glm::min(minDegrees, maxDegrees);
    mMaxPitch = glm::max(minDegrees, maxDegrees);
    mPitch = glm::clamp(mPitch, mMinPitch, mMaxPitch);
}
f32 ThirdPerson::minPitch() const
{
    return mMinPitch;
}
f32 ThirdPerson::maxPitch() const
{
    return mMaxPitch;
}
void ThirdPerson::setDistance(f32 distance)
{
    mDistance = glm::max(0.0f, distance);
}
f32 ThirdPerson::distance() const
{
    return mDistance;
}
void ThirdPerson::setZoomSpeed(f32 speed)
{
    mZoomSpeed = glm::max(0.0f, speed);
}
f32 ThirdPerson::zoomSpeed() const
{
    return mZoomSpeed;
}
void ThirdPerson::setDistanceLimits(f32 minDistance, f32 maxDistance)
{
    mMinDistance = glm::max(0.0f, glm::min(minDistance, maxDistance));
    mMaxDistance = glm::max(mMinDistance, maxDistance);
    mDistance = glm::clamp(mDistance, mMinDistance, mMaxDistance);
}
f32 ThirdPerson::minDistance() const
{
    return mMinDistance;
}
f32 ThirdPerson::maxDistance() const
{
    return mMaxDistance;
}
void ThirdPerson::setHeightOffset(f32 height)
{
    mHeightOffset = height;
}
f32 ThirdPerson::heightOffset() const
{
    return mHeightOffset;
}
void ThirdPerson::setShoulderOffset(f32 offset)
{
    mShoulderOffset = offset;
}
f32 ThirdPerson::shoulderOffset() const
{
    return mShoulderOffset;
}
void ThirdPerson::setSmoothTime(f32 seconds)
{
    mSmoothTime = glm::max(0.0001f, seconds);
}
f32 ThirdPerson::smoothTime() const
{
    return mSmoothTime;
}
void ThirdPerson::setInvertY(bool invert)
{
    mInvertY = invert;
}
bool ThirdPerson::invertY() const
{
    return mInvertY;
}
void ThirdPerson::setYawPitch(f32 yawDegrees, f32 pitchDegrees)
{
    mYaw = yawDegrees;
    mPitch = glm::clamp(pitchDegrees, mMinPitch, mMaxPitch);
}
f32 ThirdPerson::yaw() const
{
    return mYaw;
}
f32 ThirdPerson::pitch() const
{
    return mPitch;
}
void ThirdPerson::snap()
{
    const Math::Vec3 anchor = aimPoint();
    owner()->setGlobalPosition(mCollisionOctree ? collide(anchor, desiredPosition())
                                                : desiredPosition());
    mVelocity = Math::Vec3(0.0f);
}
Math::Vec3 ThirdPerson::desiredPosition() const
{
    return aimPoint() + orbitOffset(mYaw, mPitch) * mDistance + shoulderRight(mYaw) * mShoulderOffset;
}
Math::Vec3 ThirdPerson::aimPoint() const
{
    return mTarget ? mTarget->globalPosition() + Math::Vec3(0.0f, mHeightOffset, 0.0f) : Math::Vec3(0.0f);
}

// One swept sphere from the anchor to the desired position - same shape as
// TrimeshShape::slideCamera() in collision/CollisionShape.cpp, ported onto
// TriangleOctree::sweepSphere() so this stays in radion_scene without a new
// link edge to radion_physics. hit.t is the fraction of the way there; a
// negative t means the anchor itself is already inside something, and the
// camera stays at the anchor. The contact normal points away from the
// surface, so nudging along it by the margin keeps the sphere clear instead
// of grazing it.
Math::Vec3 ThirdPerson::collide(const Math::Vec3& anchor, const Math::Vec3& desired) const
{
    const Math::Vec3 delta = desired - anchor;
    if (glm::dot(delta, delta) < 1.0e-6f)
        return desired;

    TriangleOctree::SweepHit hit;
    if (mCollisionOctree->sweepSphere(anchor, mCollisionRadius, delta, hit))
    {
        const f32 t = glm::clamp(hit.t, 0.0f, 1.0f);
        Math::Vec3 position = anchor + delta * t;
        if (mCollisionMargin > 0.0f)
            position += hit.normal * mCollisionMargin;
        return position;
    }
    return desired;
}

void ThirdPerson::setCollisionOctree(const TriangleOctree* octree)
{
    mCollisionOctree = octree;
}
const TriangleOctree* ThirdPerson::collisionOctree() const
{
    return mCollisionOctree;
}
void ThirdPerson::setCollisionRadius(f32 radius)
{
    mCollisionRadius = glm::max(0.01f, radius);
}
f32 ThirdPerson::collisionRadius() const
{
    return mCollisionRadius;
}
void ThirdPerson::setCollisionMargin(f32 margin)
{
    mCollisionMargin = glm::max(0.0f, margin);
}
f32 ThirdPerson::collisionMargin() const
{
    return mCollisionMargin;
}

} // namespace Radion
