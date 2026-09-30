#include "PCH.h"
#include "ViewportCamera.h"

#include "Math.h"

namespace Radion
{

void computeCameraMatrices(const CameraState& camera, CameraView view, f32 aspect, Math::mat4& viewMatrix,
                           Math::mat4& projectionMatrix, Math::vec3& cameraPos)
{
    constexpr f32 kNear = 0.05f;
    constexpr f32 kFar = 1000.0f;

    if (view == CameraView::Perspective)
    {
        const Math::vec3 forward(Math::sin(camera.yaw) * Math::cos(camera.pitch), Math::sin(camera.pitch),
                                -Math::cos(camera.yaw) * Math::cos(camera.pitch));
        cameraPos = camera.target - forward * camera.distance;
        viewMatrix = Math::lookAt(cameraPos, camera.target, Math::vec3(0.0f, 1.0f, 0.0f));
        projectionMatrix = Math::perspective(Math::radians(60.0f), aspect, kNear, kFar);
        return;
    }

    Math::vec3 offset(0.0f);
    Math::vec3 up(0.0f, 1.0f, 0.0f);
    switch (view)
    {
    case CameraView::Top:
        offset = Math::vec3(0.0f, camera.distance, 0.0f);
        up = Math::vec3(0.0f, 0.0f, -1.0f);
        break;
    case CameraView::Bottom:
        offset = Math::vec3(0.0f, -camera.distance, 0.0f);
        up = Math::vec3(0.0f, 0.0f, 1.0f);
        break;
    case CameraView::Front: offset = Math::vec3(0.0f, 0.0f, camera.distance); break;
    case CameraView::Back: offset = Math::vec3(0.0f, 0.0f, -camera.distance); break;
    case CameraView::Left: offset = Math::vec3(-camera.distance, 0.0f, 0.0f); break;
    case CameraView::Right: offset = Math::vec3(camera.distance, 0.0f, 0.0f); break;
    case CameraView::Perspective: break;
    }
    cameraPos = camera.target + offset;
    viewMatrix = Math::lookAt(cameraPos, camera.target, up);
    const f32 halfHeight = camera.distance * 0.5f;
    const f32 halfWidth = halfHeight * aspect;
    projectionMatrix = Math::ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, kNear, kFar);
}

} // namespace Radion
