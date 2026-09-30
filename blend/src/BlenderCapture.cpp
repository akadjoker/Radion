#include "PCH.h"
#include "BlenderApplication.h"
#include "Mesh.h"

#include <glad.h>
#include "Math.h"

using namespace Radion;

namespace
{
constexpr s32 kMinCaptureSize = 16;
constexpr s32 kMaxCaptureSize = 4096;

// Colour+depth target for one capture; cheaper than keeping a second full-size target.
struct CaptureTarget
{
    GLuint fbo = 0;
    GLuint color = 0;
    GLuint depth = 0;

    bool create(s32 width, s32 height)
    {
        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);

        glGenTextures(1, &color);
        glBindTexture(GL_TEXTURE_2D, color);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                     nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color, 0);

        glGenRenderbuffers(1, &depth);
        glBindRenderbuffer(GL_RENDERBUFFER, depth);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER,
                                  depth);

        return glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    }

    ~CaptureTarget()
    {
        if (fbo)
            glDeleteFramebuffers(1, &fbo);
        if (color)
            glDeleteTextures(1, &color);
        if (depth)
            glDeleteRenderbuffers(1, &depth);
    }
};

// Perspective from fov, orthographic from frustum height (which must also cover the width).
f32 framingDistance(CameraView view, f32 radius, f32 aspect)
{
    constexpr f32 kMargin = 1.05f;
    if (view == CameraView::Perspective)
    {
        const f32 halfFov = Math::radians(60.0f) * 0.5f;
        // The narrower axis decides: a tall image has a smaller horizontal fov.
        const f32 halfAxis = aspect >= 1.0f ? halfFov : Math::atan(Math::tan(halfFov) * aspect);
        return radius / Math::sin(halfAxis) * kMargin;
    }
    return 2.0f * radius * kMargin * (aspect >= 1.0f ? 1.0f : 1.0f / aspect);
}
} // namespace

bool BlenderApplication::captureViewport(const CaptureParams& params, std::vector<u8>& rgba)
{
    if (!mMeshData || mMeshData->positions.empty())
        return false;
    if (params.width < kMinCaptureSize || params.height < kMinCaptureSize ||
        params.width > kMaxCaptureSize || params.height > kMaxCaptureSize)
        return false;

    Assets().computeBounds(*mMeshData);
    const f32 aspect = static_cast<f32>(params.width) / static_cast<f32>(params.height);

    CameraState camera = params.camera;
    if (params.frame)
    {
        // Farthest vertex, not the box corner: a long thin model would otherwise be framed too far out.
        const Math::vec3 center = mMeshData->bounds.center();
        f32 farthest = 0.0f;
        for (const Math::vec3& position : mMeshData->positions)
            farthest = Math::max(farthest, Math::distance(position, center));
        camera.target = center;
        camera.distance = framingDistance(params.view, Math::max(farthest, 0.01f), aspect);
    }

    Math::mat4 view(1.0f);
    Math::mat4 projection(1.0f);
    Math::vec3 cameraPos(0.0f);
    computeCameraMatrices(camera, params.view, aspect, view, projection, cameraPos);

    CaptureTarget target;
    GLint previousFbo = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &previousFbo);
    GLint previousViewport[4] = {0, 0, 0, 0};
    glGetIntegerv(GL_VIEWPORT, previousViewport);

    const bool ready = target.create(params.width, params.height);
    if (ready)
    {
        glViewport(0, 0, params.width, params.height);
        const Math::vec3& background = mSettings.viewport().backgroundColor;
        glClearColor(background.x, background.y, background.z, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        MiniDrawParams draw;
        draw.mode = params.shading;
        draw.colorBySubmesh = params.colorBySubmesh;
        draw.vertexColors = params.vertexColors;
        draw.showWireframeOverlay = params.wireframeOverlay;
        if (mHasSkeleton && !mBonePalette.empty())
        {
            draw.bonePalette = mBonePalette.data();
            draw.boneCount = static_cast<u32>(mBonePalette.size());
        }

        std::vector<u8> submeshVisible;
        submeshVisible.reserve(mMeshData->submeshes.size());
        for (u32 i = 0; i < static_cast<u32>(mMeshData->submeshes.size()); ++i)
            submeshVisible.push_back(isSubmeshVisible(i) ? 1 : 0);
        draw.submeshVisible = submeshVisible.data();
        draw.submeshVisibleCount = static_cast<u32>(submeshVisible.size());

        mRenderer.renderViewport(mMeshData, view, projection, cameraPos, draw);

        if (params.grid)
        {
            mBatch.begin();
            mBatch.grid(0.0f, 20, 1.0f, true);
            mBatch.flush(projection * view);
        }

        rgba.resize(static_cast<usize>(params.width) * static_cast<usize>(params.height) * 4u);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, params.width, params.height, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());

        // GL's first row is the bottom of the image; files want the top first.
        const usize rowBytes = static_cast<usize>(params.width) * 4u;
        std::vector<u8> row(rowBytes);
        for (s32 y = 0; y < params.height / 2; ++y)
        {
            u8* top = rgba.data() + static_cast<usize>(y) * rowBytes;
            u8* bottom = rgba.data() + static_cast<usize>(params.height - 1 - y) * rowBytes;
            std::copy(top, top + rowBytes, row.begin());
            std::copy(bottom, bottom + rowBytes, top);
            std::copy(row.begin(), row.end(), bottom);
        }
    }

    glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(previousFbo));
    glViewport(previousViewport[0], previousViewport[1], previousViewport[2], previousViewport[3]);

    if (!ready)
        Log::error("BlenderApplication: capture target %dx%d failed", params.width, params.height);
    return ready;
}
