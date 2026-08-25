#ifndef RADION_CAMERA_H
#define RADION_CAMERA_H

#include "Component.h"
#include "GameObject.h"
#include "Math.h"

namespace Radion
{

enum class CameraProjection : u8
{
    Perspective,
    Orthographic
};

class Camera final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Camera;
    void setPerspective(f32 fieldOfViewDegrees, f32 aspect, f32 nearPlane, f32 farPlane);
    void setOrthographic(f32 size, f32 aspect, f32 nearPlane, f32 farPlane);
    void setAspect(f32 aspect);

    CameraProjection projectionMode() const;
    f32 fieldOfView() const;
    f32 orthographicSize() const;
    f32 aspect() const;
    f32 nearPlane() const;
    f32 farPlane() const;

    glm::mat4 viewMatrix() const;
    glm::mat4 projectionMatrix() const;
    glm::mat4 viewProjectionMatrix() const;

    // Builds a world-space picking ray from window mouse coordinates. The
    // viewport is supplied by the render pass, so split-screen and editor
    // previews use the same camera API.
    Ray rayFromMouse(f32 mouseX, f32 mouseY, const FloatRect& viewport) const;

private:
    friend class GameObject;

    Camera();

    CameraProjection mProjection = CameraProjection::Perspective;
    f32 mFieldOfView = 60.0f;
    f32 mOrthographicSize = 10.0f;
    f32 mAspect = 16.0f / 9.0f;
    f32 mNearPlane = 0.1f;
    f32 mFarPlane = 1000.0f;
};

} // namespace Radion

#endif // RADION_CAMERA_H
