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

// Size of the buffers the scene is rendered into, which is independent of the
// window: the last post pass stretches the result to the presentation rect,
// and the camera's aspect comes from that rect rather than from these numbers,
// so changing them costs sharpness and never distorts geometry.
//
// width/height of 0 follow the presentation rect, and `scale` then applies -
// which is what a demo wants while the window is resizable. Setting both pins
// the buffers to exactly that size no matter how large the window is, and
// `scale` is ignored.
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

// What every technique gets for the frame. Passed by const reference, rebuilt
// each frame by whoever drives the loop - today a demo, later the scene.
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

    // (0,0,0,0) = no clip: a world point is kept when
    // dot(vec4(pos,1), clipPlane) >= 0. Set by whoever builds this camera - a
    // planar reflection's mirrored one, e.g. - never toggled elsewhere.
    Math::vec4 clipPlane = Math::vec4(0.0f);

    // Where the technique should draw, and how much of it. An invalid target
    // is the screen. Each technique binds this itself - viewport is not
    // global GL state one setViewport() call covers for the whole frame,
    // it belongs to the pass, the way a camera owns its own width/height/
    // scissor. A ReflectionPass rendering into a smaller RTT needs its own.
    TargetHandle target;
    TextureHandle ambientOcclusion;
    TextureHandle directionalShadow;
    SamplerHandle directionalShadowSampler;
    SamplerHandle directionalShadowRawSampler;
    BufferHandle directionalShadowBlock;

    // Filled by Lighting::prepare()/cull(), read by ForwardPass. See
    // Lighting.h - the entity buffer holds the sun (index 0, by convention)
    // and every local light, the matrix buffer their shadow view-projections,
    // the tile buffer the cull dispatch's per-tile bitmasks.
    BufferHandle entityBuffer;
    BufferHandle entityMatrixBuffer;
    BufferHandle lightTileBuffer;
    BufferHandle lightingBlock;
    TextureHandle shadowAtlas;
    SamplerHandle shadowAtlasSampler;

    // Set by whoever owns a DecalSystem, alongside submitDecals() into the
    // entity buffer above. Left invalid, ForwardPass binds its own neutral
    // placeholder instead - a Lit pipeline declares these arrays statically,
    // so something has to be bound whether or not any decal exists this frame.
    TextureHandle decalAlbedo;
    TextureHandle decalNormal;
    TextureHandle decalSurface;

    // The colour and depth textures behind `target`, when the frame renders
    // into one it owns. A pass that has to READ what the scene already drew
    // cannot sample these directly - they are still attached to the bound
    // target, which is a feedback loop - but it needs them to copy out a match:
    // a copy whose format differs is a silently failed blit, since a depth blit
    // between unlike formats is an error GL reports nowhere the frame can see.
    TextureHandle sceneColor;
    TextureHandle sceneDepth;

    // The environment probe's cubemap and where it was captured from, for
    // image-based reflections. Left invalid, ForwardPass binds a neutral cube
    // instead - see BindingEnvironmentCube. Extents all zero means the
    // reflection is treated as infinitely distant (no parallax correction).
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

    // Only WaterPass reads this: a water surface projects its own vertices
    // through the reflection camera too (see water.vert), which has its own
    // projection unrelated to `viewProjection` above. Whoever renders the
    // reflection target sets this to that camera's matrix.
    Math::mat4 reflectionViewProj = Math::mat4(1.0f);
};

} // namespace Radion

#endif // RADION_FRAME_CONTEXT_H
