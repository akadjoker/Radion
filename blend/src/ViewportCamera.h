#ifndef RADION_BLENDER_VIEWPORT_CAMERA_H
#define RADION_BLENDER_VIEWPORT_CAMERA_H

#include "Types.h"

#include "Math.h"

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
    Math::vec3 target = Math::vec3(0.0f);
    f32 yaw = 0.0f;   // radians
    f32 pitch = 0.3f; // radians
    f32 distance = 6.0f;
};

// Camera maths shared by the docked viewports and the screenshot capture, so both frame identically.
void computeCameraMatrices(const CameraState& camera, CameraView view, f32 aspect, Math::mat4& viewMatrix,
                           Math::mat4& projectionMatrix, Math::vec3& cameraPos);

} // namespace Radion

#endif // RADION_BLENDER_VIEWPORT_CAMERA_H
