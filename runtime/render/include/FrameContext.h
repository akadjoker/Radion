#ifndef RADION_FRAME_CONTEXT_H
#define RADION_FRAME_CONTEXT_H

#include "GPU.h"
#include "RenderList.h"

namespace Radion
{

struct SkySettings;

enum class AspectMode : u8
{
    Window,
    Ratio16x9,
    Ratio16x10,
    Ratio4x3,
    Ratio21x9,
    Ratio1x1
};

enum class FitMode : u8
{
    Stretch,
    Letterbox,
    Crop
};

struct PresentationSettings
{
    AspectMode aspect = AspectMode::Window;
    FitMode fit = FitMode::Letterbox;
};

struct PresentationView
{
    Rect rect;
    f32 aspect = 1.0f;
};

// Size of the scene buffers, independent of the window: the last post pass stretches to the presentation rect, so changing them costs sharpness, not geometry.
// width/height 0 follow the presentation rect with `scale`; setting both pins the size and `scale` is ignored.
struct RenderResolution
{
    u32 width = 0;
    u32 height = 0;
    f32 scale = 1.0f;
};

inline f32 aspectValue(AspectMode mode, u32 windowWidth, u32 windowHeight)
{
    switch (mode)
    {
    case AspectMode::Ratio16x9:
        return 16.0f / 9.0f;
    case AspectMode::Ratio16x10:
        return 16.0f / 10.0f;
    case AspectMode::Ratio4x3:
        return 4.0f / 3.0f;
    case AspectMode::Ratio21x9:
        return 21.0f / 9.0f;
    case AspectMode::Ratio1x1:
        return 1.0f;
    case AspectMode::Window:
    default:
        return static_cast<f32>(windowWidth) / static_cast<f32>(windowHeight ? windowHeight : 1);
    }
}

inline PresentationView computePresentation(const PresentationSettings& settings, u32 windowWidth,
                                            u32 windowHeight)
{
    PresentationView view;
    view.aspect = aspectValue(settings.aspect, windowWidth, windowHeight);
    view.rect.width = static_cast<s32>(windowWidth);
    view.rect.height = static_cast<s32>(windowHeight);

    if (windowWidth == 0 || windowHeight == 0 || settings.fit == FitMode::Stretch ||
        settings.aspect == AspectMode::Window)
        return view;

    const f32 windowAspect = static_cast<f32>(windowWidth) / static_cast<f32>(windowHeight);
    const bool fitHeight = settings.fit == FitMode::Letterbox ? windowAspect > view.aspect
                                                              : windowAspect < view.aspect;
    if (fitHeight)
    {
        view.rect.height = static_cast<s32>(windowHeight);
        view.rect.width = static_cast<s32>(windowHeight * view.aspect + 0.5f);
    }
    else
    {
        view.rect.width = static_cast<s32>(windowWidth);
        view.rect.height = static_cast<s32>(windowWidth / view.aspect + 0.5f);
    }
    view.rect.x = (static_cast<s32>(windowWidth) - view.rect.width) / 2;
    view.rect.y = (static_cast<s32>(windowHeight) - view.rect.height) / 2;
    return view;
}

struct FrameContext
{
    const RenderList* list = nullptr;

    Math::mat4 view = Math::mat4(1.0f);
    Math::mat4 projection = Math::mat4(1.0f);
    Math::mat4 viewProjection = Math::mat4(1.0f);
    Math::mat4 projectionNoJitter = Math::mat4(1.0f);
    Math::mat4 viewProjectionNoJitter = Math::mat4(1.0f);
    Math::mat4 prevView = Math::mat4(1.0f);
    Math::mat4 prevProjectionNoJitter = Math::mat4(1.0f);
    Math::mat4 prevViewProjectionNoJitter = Math::mat4(1.0f);
    Math::vec2 jitter = Math::vec2(0.0f);
    Math::vec2 prevJitter = Math::vec2(0.0f);
    Math::vec3 cameraPosition = Math::vec3(0.0f);

    // (0,0,0,0) = no clip: a world point is kept when dot(vec4(pos,1), clipPlane) >= 0.
    Math::vec4 clipPlane = Math::vec4(0.0f);

    // Where the technique draws; an invalid target is the screen. Each technique binds its own viewport.
    TargetHandle target;
    TextureHandle ambientOcclusion;
    TextureHandle directionalShadow;
    SamplerHandle directionalShadowSampler;
    SamplerHandle directionalShadowRawSampler;
    BufferHandle directionalShadowBlock;

    // Filled by Lighting::prepare()/cull(), read by ForwardPass (see Lighting.h); the entity buffer holds the sun at index 0, then local lights.
    BufferHandle entityBuffer;
    BufferHandle entityMatrixBuffer;
    BufferHandle lightTileBuffer;
    BufferHandle lightingBlock;
    TextureHandle shadowAtlas;
    SamplerHandle shadowAtlasSampler;

    // Set by the DecalSystem owner; left invalid, ForwardPass binds a neutral placeholder since Lit pipelines declare these arrays statically.
    TextureHandle decalAlbedo;
    TextureHandle decalNormal;
    TextureHandle decalSurface;

    // Colour and depth textures behind `target`. A pass reading the scene cannot sample them directly (still attached, a feedback loop) and must copy into a same-format texture; a mismatched depth blit fails silently.
    TextureHandle sceneColor;
    TextureHandle sceneDepth;

    // Probe cubemap and capture position; left invalid, ForwardPass binds a neutral cube (BindingEnvironmentCube). Zero extents = infinitely distant, no parallax.
    TextureHandle environmentCube;
    SamplerHandle environmentCubeSampler;
    Math::vec3 environmentProbePosition = Math::vec3(0.0f);
    Math::vec3 environmentProbeExtents = Math::vec3(0.0f);
    u32 environmentProbeMips = 1;
    f32 environmentProbeIntensity = 1.0f;

    Viewport viewport;
    u32 width = 0;
    u32 height = 0;
    bool temporalAA = true;
    f32 fieldOfView = 60.0f;
    f32 aspect = 16.0f / 9.0f;
    f32 nearPlane = 0.1f;

    f32 time = 0.0f;
    f32 deltaTime = 0.0f;
    const SkySettings* sky = nullptr;

    // Only WaterPass reads this: water projects its vertices through the reflection camera (see water.vert), unrelated to `viewProjection`.
    Math::mat4 reflectionViewProj = Math::mat4(1.0f);
};

} // namespace Radion

#endif // RADION_FRAME_CONTEXT_H
