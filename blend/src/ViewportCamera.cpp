#include "PCH.h"
#include "ViewportCamera.h"

#include <glm/gtc/matrix_transform.hpp>

namespace Radion
{

void computeCameraMatrices(const CameraState& camera, CameraView view, f32 aspect, glm::mat4& viewMatrix,
                           glm::mat4& projectionMatrix, glm::vec3& cameraPos)
{
    constexpr f32 kNear = 0.05f;
    constexpr f32 kFar = 1000.0f;

    if (view == CameraView::Perspective)
    {
        const glm::vec3 forward(glm::sin(camera.yaw) * glm::cos(camera.pitch), glm::sin(camera.pitch),
                                -glm::cos(camera.yaw) * glm::cos(camera.pitch));
        cameraPos = camera.target - forward * camera.distance;
        viewMatrix = glm::lookAt(cameraPos, camera.target, glm::vec3(0.0f, 1.0f, 0.0f));
        projectionMatrix = glm::perspective(glm::radians(60.0f), aspect, kNear, kFar);
        return;
    }

    glm::vec3 offset(0.0f);
    glm::vec3 up(0.0f, 1.0f, 0.0f);
    switch (view)
    {
    case CameraView::Top:
        offset = glm::vec3(0.0f, camera.distance, 0.0f);
        up = glm::vec3(0.0f, 0.0f, -1.0f);
        break;
    case CameraView::Bottom:
        offset = glm::vec3(0.0f, -camera.distance, 0.0f);
        up = glm::vec3(0.0f, 0.0f, 1.0f);
        break;
    case CameraView::Front: offset = glm::vec3(0.0f, 0.0f, camera.distance); break;
    case CameraView::Back: offset = glm::vec3(0.0f, 0.0f, -camera.distance); break;
    case CameraView::Left: offset = glm::vec3(-camera.distance, 0.0f, 0.0f); break;
    case CameraView::Right: offset = glm::vec3(camera.distance, 0.0f, 0.0f); break;
    case CameraView::Perspective: break;
    }
    cameraPos = camera.target + offset;
    viewMatrix = glm::lookAt(cameraPos, camera.target, up);
    const f32 halfHeight = camera.distance * 0.5f;
    const f32 halfWidth = halfHeight * aspect;
    projectionMatrix = glm::ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, kNear, kFar);
}

} // namespace Radion
