#ifndef RADION_CAMERA_H
#define RADION_CAMERA_H

#include "Component.h"
#include "GPU.h" // TextureHandle, for the recording target
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

    // ---- recording -----------------------------------------------------
    //
    // A recording camera renders its own view into its own texture every
    // frame, at its own resolution - nothing to do with the window's. That
    // is what a camera sensor is, and it is equally what feeds a monitor in
    // the scene, a rear-view mirror or a security screen.
    //
    // The resolution is the point: a sensor wants 320x240 or 640x480, not
    // whatever the window happens to be. A small target costs a small
    // render.
    void setRecording(bool recording);
    bool recording() const
    {
        return mRecording;
    }
    // Clamped to at least 1x1. Changing it drops the current texture, so a
    // handle taken before the call must not be kept across it.
    void setRecordSize(u32 width, u32 height);
    u32 recordWidth() const
    {
        return mRecordWidth;
    }
    u32 recordHeight() const
    {
        return mRecordHeight;
    }

    // Last frame's picture, or an invalid handle before the first one is
    // rendered (and whenever recording is off). Valid to sample, draw, or
    // read back.
    TextureHandle recordTexture() const
    {
        return mRecordTexture;
    }
    TextureHandle recordDepthTexture() const
    {
        return mRecordDepthTexture;
    }

private:
    friend class GameObject;
    friend class Engine;

    Camera();

    CameraProjection mProjection = CameraProjection::Perspective;
    f32 mFieldOfView = 60.0f;
    f32 mOrthographicSize = 10.0f;
    f32 mAspect = 16.0f / 9.0f;
    f32 mNearPlane = 0.1f;
    f32 mFarPlane = 1000.0f;

    bool mRecording = false;
    u32 mRecordWidth = 640;
    u32 mRecordHeight = 480;
    TextureHandle mRecordTexture;
    TextureHandle mRecordDepthTexture;
};

} // namespace Radion

#endif // RADION_CAMERA_H
