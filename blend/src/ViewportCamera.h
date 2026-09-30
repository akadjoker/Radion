#ifndef RADION_BLENDER_VIEWPORT_CAMERA_H
#define RADION_BLENDER_VIEWPORT_CAMERA_H

#include "Types.h"

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

namespace Radion
{

enum class CameraView : u8
{
    Perspective,
    Top,
    Bottom,
    Front,
    Back,
    Left,
    Right
};

// Camera orbiting `target` at (yaw, pitch, distance). The fixed views look
// straight down an axis and ignore yaw/pitch; `distance` is then the height of
// the orthographic frustum.
struct CameraState
{
    glm::vec3 target = glm::vec3(0.0f);
    f32 yaw = 0.0f;   // radians
    f32 pitch = 0.3f; // radians
    f32 distance = 6.0f;
};

// One place for the camera maths, shared by the docked viewports and the
// offscreen capture the API takes screenshots with - so a screenshot frames
// the model exactly as the same camera would on screen.
void computeCameraMatrices(const CameraState& camera, CameraView view, f32 aspect, glm::mat4& viewMatrix,
                           glm::mat4& projectionMatrix, glm::vec3& cameraPos);

} // namespace Radion

#endif // RADION_BLENDER_VIEWPORT_CAMERA_H
