#include "PCH.h"

#include "Camera.h"

#include "Math.h"

namespace Radion
{

Camera::Camera() : Component(Type)
{
}

void Camera::setPerspective(f32 fieldOfViewDegrees, f32 aspect, f32 nearPlane, f32 farPlane)
{
    if (!std::isfinite(fieldOfViewDegrees) || !std::isfinite(aspect) || !std::isfinite(nearPlane) ||
        !std::isfinite(farPlane) || fieldOfViewDegrees <= 0.0f || fieldOfViewDegrees >= 180.0f ||
        aspect <= 0.0f || nearPlane <= 0.0f || farPlane <= nearPlane)
    {
        Log::warning("Camera: rejected invalid perspective");
        return;
    }
    mProjection = CameraProjection::Perspective;
    mFieldOfView = fieldOfViewDegrees;
    mAspect = aspect;
    mNearPlane = nearPlane;
    mFarPlane = farPlane;
}

void Camera::setOrthographic(f32 size, f32 aspect, f32 nearPlane, f32 farPlane)
{
    if (!std::isfinite(size) || !std::isfinite(aspect) || !std::isfinite(nearPlane) ||
        !std::isfinite(farPlane) || size <= 0.0f || aspect <= 0.0f || farPlane <= nearPlane)
    {
        Log::warning("Camera: rejected invalid orthographic projection");
        return;
    }
    mProjection = CameraProjection::Orthographic;
    mOrthographicSize = size;
    mAspect = aspect;
    mNearPlane = nearPlane;
    mFarPlane = farPlane;
}

void Camera::setAspect(f32 aspect)
{
    if (!std::isfinite(aspect) || aspect <= 0.0f)
    {
        Log::warning("Camera: rejected invalid aspect ratio");
        return;
    }
    mAspect = aspect;
}

CameraProjection Camera::projectionMode() const
{
    return mProjection;
}

f32 Camera::fieldOfView() const
{
    return mFieldOfView;
}

f32 Camera::orthographicSize() const
{
    return mOrthographicSize;
}

f32 Camera::aspect() const
{
    return mAspect;
}

f32 Camera::nearPlane() const
{
    return mNearPlane;
}

f32 Camera::farPlane() const
{
    return mFarPlane;
}

Math::mat4 Camera::viewMatrix() const
{
    const Math::quat inverseRotation = Math::inverse(owner()->globalRotation());
    return Math::mat4_cast(inverseRotation) *
           Math::translate(Math::mat4(1.0f), -owner()->globalPosition());
}

Math::mat4 Camera::projectionMatrix() const
{
    if (mProjection == CameraProjection::Perspective)
        return Math::perspective(Math::radians(mFieldOfView), mAspect, mNearPlane, mFarPlane);

    const f32 halfHeight = mOrthographicSize * 0.5f;
    const f32 halfWidth = halfHeight * mAspect;
    return Math::ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, mNearPlane, mFarPlane);
}

Math::mat4 Camera::viewProjectionMatrix() const
{
    return projectionMatrix() * viewMatrix();
}

Ray Camera::rayFromMouse(f32 mouseX, f32 mouseY, const FloatRect& viewport) const
{
    if (!std::isfinite(mouseX) || !std::isfinite(mouseY) || !std::isfinite(viewport.x) ||
        !std::isfinite(viewport.y) || !std::isfinite(viewport.width) ||
        !std::isfinite(viewport.height) || viewport.width <= 0.0f || viewport.height <= 0.0f)
    {
        Log::warning("Camera: rejected invalid picking viewport");
        return Ray();
    }

    return Radion::rayFromScreen(mouseX - viewport.x, mouseY - viewport.y, viewport.width,
                                 viewport.height, Math::inverse(viewProjectionMatrix()));
}

} // namespace Radion
