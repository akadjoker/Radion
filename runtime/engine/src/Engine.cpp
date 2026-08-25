#include "PCH.h"

#include "Engine.h"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define MSF_GIF_IMPL
#include "msf_gif.h"
#include <GL/gl.h>

#include "AssetManager.h"
#include "AsyncTextureLoader.h"
#include "Batch.h"
#include "DebugDraw3D.h"
#include "DefaultPack.h"
#include "EngineSettings.h"
#include "FileSystem.h"
#include "GPUProfiler.h"
#include "GrassRender.h"
#include "HairRender.h"
#include "Input.h"
#include "Log.h"
#include "MaterialManager.h"
#include "OceanRender.h"
#include "ParticlePass.h"
#include "PostProcess.h"
#include "Profiler.h"
#include "RenderList.h"
#include "Renderer.h"
#include "Scene.h"
#include "TrailRender.h"
#include "TreeRender.h"

namespace Radion
{

struct Engine::GifRecorder
{
    MsfGifState state{};
    FILE* file = nullptr;
    std::vector<unsigned char> pixels;
    int width = 0;
    int height = 0;
    int frameCount = 0;
    int sampleCounter = 0;
    bool recording = false;
    std::string filename;

    bool start(Platform::Window& window)
    {
        if (recording)
            return false;
        window.getDrawableSize(width, height);
        if (width <= 0 || height <= 0)
            return false;

        std::filesystem::create_directories("recordings");
        for (int index = 1;; ++index)
        {
            filename = "recordings/radion_" + std::to_string(index) + ".gif";
            if (!std::filesystem::exists(filename))
                break;
        }

        file = std::fopen(filename.c_str(), "wb");
        if (!file || !msf_gif_begin_to_file(&state, width, height,
                                             reinterpret_cast<MsfGifFileWriteFunc>(std::fwrite),
                                             file))
        {
            if (file)
                std::fclose(file);
            file = nullptr;
            return false;
        }

        pixels.resize(static_cast<size_t>(width) * height * 4);
        frameCount = 0;
        sampleCounter = 0;
        recording = true;
        Log::info("GIF recording started: %s", filename.c_str());
        return true;
    }

    void frame()
    {
        if (!recording || ++sampleCounter % 4 != 0)
            return;

        glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
        const size_t rowBytes = static_cast<size_t>(width) * 4;
        for (int y = 0; y < height / 2; ++y)
        {
            unsigned char* top = pixels.data() + static_cast<size_t>(y) * rowBytes;
            unsigned char* bottom =
                pixels.data() + static_cast<size_t>(height - 1 - y) * rowBytes;
            for (size_t x = 0; x < rowBytes; ++x)
                std::swap(top[x], bottom[x]);
        }
        if (msf_gif_frame_to_file(&state, pixels.data(), 7, 8, width * 4))
            ++frameCount;
    }

    void stop()
    {
        if (!recording)
            return;
        msf_gif_end_to_file(&state);
        std::fclose(file);
        file = nullptr;
        recording = false;
        Log::info("GIF recording finished: %s (%d frames)", filename.c_str(), frameCount);
    }
};

Engine::Engine() = default;

Engine::~Engine()
{
    shutdown();
}

bool Engine::initialize(const EngineConfig& config)
{
    if (mInitialized)
        return true;

    // Ahead of everything: addDefaultPasses() below asks for its first shader
    // before this function returns, and the fallback has to already be there.
    DefaultPack::mount(FileSystem::getSingleton());

#if defined(RADION_DEBUG)
    mWindow.setDebugContext(true);
#else
    mWindow.setDebugContext(config.debugContext);
#endif

    const char* title = config.title ? config.title : "Radion";
    if (!mWindow.create(title, config.width, config.height, config.monitor, config.resizable,
                        config.fullscreen, config.visible))
        return false;

    mGpu = GPU::createOpenGL(mWindow);
    if (!mGpu)
    {
        mWindow.destroy();
        return false;
    }

    if (!mImGui.initialize(mWindow))
    {
        GPU::destroyDevice(mGpu);
        mGpu = nullptr;
        mWindow.destroy();
        return false;
    }

    mRenderer = new Renderer();
    mRenderList = new RenderList();
    mPostProcess = new PostProcessStack();
    if (!mPostProcess->initialize() || !mRenderer->addDefaultPasses())
    {
        mProbe.shutdown();
        mPostProcess->shutdown();
        delete mPostProcess;
        delete mRenderList;
        delete mRenderer;
        mRenderList = nullptr;
        mPostProcess = nullptr;
        mRenderer = nullptr;
        mImGui.shutdown();
        GPU::destroyDevice(mGpu);
        mGpu = nullptr;
        mWindow.destroy();
        return false;
    }

    // Fixed internal resolution, not the window's own. On a HiDPI display the
    // drawable size is twice the window (2560x1408 for a 1280x720 window), and
    // every post-process target was being allocated at that size - four times
    // the fill rate for a demo nobody asked to run at 4K. A demo that wants
    // native can still call setRenderResolution({0, 0, 1.0f}).
    RenderResolution defaultResolution;
    defaultResolution.width = 1280;
    defaultResolution.height = 720;
    setRenderResolution(defaultResolution);

    // Off by default: a demo that wants the look opts in, rather than every
    // demo paying the cost and fighting the softening/glow to see its own
    // change clearly.
    // 128 is the reference's own middle ground for a realtime probe: six
    // faces of it is a quarter of the pixels of one 720p frame, and the
    // reflection is blurred by roughness anyway.
    if (!mProbe.create(128))
        Log::warning("Engine: no environment probe, reflections will be flat");

    mPostProcess->add(PostEffect::Bloom);
    mPostProcess->setEnabled(PostEffect::Bloom, false);
    mPostProcess->add(PostEffect::ToneMap);
    mPostProcess->add(PostEffect::FXAA);
    mPostProcess->setEnabled(PostEffect::FXAA, false);

    // So a scene file also carries shadow/post-process settings, not just
    // the object graph - Renderer/Lighting/PostProcessStack all exist by
    // this point, and every scene loaded/saved from here on shares the same
    // three structs (they are Engine-wide, not per-Scene).
    mSceneManager.bindRenderSettings(mRenderer->cascadeSettings(),
                                     mRenderer->lighting()
                                         ? &mRenderer->lighting()->atlasSettings()
                                         : nullptr,
                                     mPostProcess, mRenderer->lensFlare(), &mProbe,
                                     mRenderer->lighting(), mRenderer->volumetric(), &mSky,
                                     &mRenderResolution, &ParticleDraws());

    // Same reasoning as Bloom/FXAA above: a light still has to opt in with
    // its own setVolumetric(true), but these master switches are what let a
    // demo skip the cost across the board instead of per light.
    VolumetricPass* volumetric = mRenderer->volumetric();
    volumetric->sunEnabled = false;
    volumetric->spotEnabled = false;
    volumetric->pointEnabled = false;
    volumetric->rectEnabled = false;

    mInitialized = true;
    return true;
}

void Engine::shutdown()
{
    if (!mInitialized)
        return;

    stopGifRecording();

    if (mFrameActive)
    {
        Log::warning("Engine::shutdown called during an active frame");
        mFrameActive = false;
    }

    // Window position/size (and everything else EngineSettings tracks) as
    // they stand right before teardown - a plain window-close should not
    // lose them just because nobody pressed "Guardar settings".
    if (!mSettingsFile.empty())
        EngineSettings::save(*this, mSettingsFile);

    mSceneManager.unload();

    if (mLoadingBatch)
    {
        mLoadingBatch->shutdown();
        delete mLoadingBatch;
        mLoadingBatch = nullptr;
    }

    mRenderer->shutdown();
    mPostProcess->shutdown();
    delete mPostProcess;
    delete mRenderList;
    delete mRenderer;
    mRenderList = nullptr;
    mPostProcess = nullptr;
    mRenderer = nullptr;
    mImGui.shutdown();

    // Everything that owns GPU resources, released here in a fixed order
    // while the context is still alive: assets (meshes, textures, samplers)
    // first, then the pipelines they referenced, then whatever the device
    // still has live. Leaving any of it to a destructor would put GL calls
    // at whatever point the owner happened to die.
    GPUProfiler::getSingleton().shutdown();
    // Joins the worker thread before anything it might still be touching
    // (FileSystem, the texture cache below) goes away.
    AsyncTextureLoader::getSingleton().shutdown();
    Assets().shutdown();
    MaterialManager::getSingleton().destroyAllPipelines();
    mGpu->shutdown();

    GPU::destroyDevice(mGpu);
    mGpu = nullptr;
    mWindow.destroy();
    mInitialized = false;
}

Scene* Engine::createScene()
{
    return mSceneManager.create();
}

bool Engine::setActiveScene(Scene* scene)
{
    return mSceneManager.activate(scene);
}

bool Engine::update()
{
    if (!mInitialized || mFrameActive)
        return false;

    Profiler::getSingleton().beginFrame();
    // The previous frame's tail, one frame late - see flip().
    Profiler::getSingleton().addSample("ImGui (frame anterior)", mOverlayMilliseconds);
    Profiler::getSingleton().addSample("Present (frame anterior)", mPresentMilliseconds);
    DebugDraw().clear();
    TrailDraws().clear();
    ParticleDraws().clear();
    GrassDraws().clear();
    HairDraws().clear();
    OceanDraws().clear();
    TreeDraws().clear();
    RADION_PROFILE_SCOPE("Engine update");
    mWindow.update();
    if (!mWindow.isOpen())
        return false;

    if (Input::isKeyPressed(KEY_F10))
    {
        if (isGifRecording())
            stopGifRecording();
        else if (!startGifRecording())
            Log::error("Engine: could not start GIF recording");
    }

    mGpu->beginFrame();
    // GL context is current from here on - the one place in the frame that
    // is both safe (main thread, no draw in flight yet) and early enough for
    // a texture that finished streaming to render correctly this same frame.
    Assets().processAsyncTextureLoads();
    if (Assets().processAsyncMeshLoads() > 0)
        if (Scene* active = mSceneManager.active())
            active->reapplyHiddenSubmeshes();
    GPUProfiler::getSingleton().beginFrame();
    mImGui.update();
    mFrameActive = true;
    return true;
}

CascadeShadowSettings* Engine::cascadeSettings()
{
    return mRenderer ? mRenderer->cascadeSettings() : nullptr;
}

f32 Engine::cascadeHalfExtent(u32 cascade) const
{
    return mRenderer ? mRenderer->cascadeHalfExtent(cascade) : 0.0f;
}

f32 Engine::cascadeSplit(u32 cascade) const
{
    return mRenderer ? mRenderer->cascadeSplit(cascade) : 0.0f;
}

const RenderListStats* Engine::shadowListStats() const
{
    return mRenderer ? mRenderer->shadowListStats() : nullptr;
}

const char* Engine::sunShadowStatus() const
{
    return mRenderer ? mRenderer->sunShadowStatus() : "no renderer";
}

const RenderListStats* Engine::mainRenderListStats() const
{
    return mRenderList ? &mRenderList->stats() : nullptr;
}

TextureHandle Engine::directionalShadowTexture() const
{
    return mRenderer ? mRenderer->directionalShadowTexture() : TextureHandle();
}

void Engine::debugDrawTexture(TextureHandle texture, bool isArray, u32 layer, TargetHandle target,
                              u32 width, u32 height, const glm::vec4& sourceRect)
{
    if (mRenderer && texture.valid() && width > 0 && height > 0)
        mRenderer->debugDrawTexture(texture, isArray, layer, target, width, height, sourceRect);
    ClearValue noClear;
    noClear.bits = 0;
    mGpu->setTarget(TargetHandle(), noClear);
}

void Engine::debugDrawCubemap(TextureHandle texture, u32 face, u32 mip, TargetHandle target,
                              u32 width, u32 height)
{
    if (mRenderer && texture.valid() && width > 0 && height > 0)
        mRenderer->debugDrawCubemap(texture, face, mip, target, width, height);
}

Lighting* Engine::lighting()
{
    return mRenderer ? mRenderer->lighting() : nullptr;
}

VolumetricPass* Engine::volumetric()
{
    return mRenderer ? mRenderer->volumetric() : nullptr;
}

void Engine::setSunShadows(bool enabled)
{
    CascadeShadowSettings* settings = cascadeSettings();
    if (settings)
        settings->enabled = enabled;
}

bool Engine::sunShadows()
{
    CascadeShadowSettings* settings = cascadeSettings();
    return settings ? settings->enabled : false;
}

void Engine::setPointShadows(bool enabled)
{
    Lighting* lit = lighting();
    if (lit)
        lit->atlasSettings().point = enabled;
}

bool Engine::pointShadows()
{
    Lighting* lit = lighting();
    return lit ? lit->atlasSettings().point : false;
}

void Engine::setSpotShadows(bool enabled)
{
    Lighting* lit = lighting();
    if (lit)
        lit->atlasSettings().spot = enabled;
}

bool Engine::spotShadows()
{
    Lighting* lit = lighting();
    return lit ? lit->atlasSettings().spot : false;
}

LensFlarePass* Engine::lensFlare()
{
    return mRenderer ? mRenderer->lensFlare() : nullptr;
}

const char* Engine::reflectionSource() const
{
    return mRenderer ? mRenderer->reflectionSource() : "none";
}

void Engine::setEnabledPasses(u32 mask)
{
    mEnabledPasses = mask;
}

u32 Engine::enabledPasses() const
{
    return mEnabledPasses;
}

void Engine::setPassEnabled(u32 bit, bool enabled)
{
    if (enabled)
        mEnabledPasses |= bit;
    else
        mEnabledPasses &= ~bit;
}

bool Engine::passEnabled(u32 bit) const
{
    return (mEnabledPasses & bit) != 0;
}

DecalSystem* Engine::decals()
{
    return mRenderer ? mRenderer->decals() : nullptr;
}

void Engine::setSkyCubemap(const std::string& baseName)
{
    // Switching the mode is this function's job and not loadSkyCubemap()'s:
    // asking for a sky by name is a request to see it, whereas restoring
    // settings resolves the same name without disturbing the saved mode.
    if (loadSkyCubemap(mSky, baseName) && !baseName.empty())
        mSky.mode = SkyMode::Cubemap;
}

void Engine::drawSkyContents()
{
    mImGui.drawSkyContents(mSky);
}

void Engine::drawPostProcessContents()
{
    mImGui.drawPostProcessContents(*mPostProcess, mRenderResolution, mRenderer->volumetric());
}

void Engine::drawProfilerContents()
{
    const RenderListStats empty;
    const RenderListStats* stats = mainRenderListStats();
    mImGui.drawProfilerContents(mGpu->stats(), stats ? *stats : empty);
}

void Engine::setRenderResolution(const RenderResolution& resolution)
{
    mRenderResolution = resolution;
    mRenderResolution.scale = glm::clamp(resolution.scale, 0.25f, 2.0f);
}

bool Engine::startGifRecording()
{
    if (!mInitialized)
        return false;
    if (!mGifRecorder)
        mGifRecorder = std::make_unique<GifRecorder>();
    return mGifRecorder->start(mWindow);
}

void Engine::stopGifRecording()
{
    if (mGifRecorder)
        mGifRecorder->stop();
}

bool Engine::isGifRecording() const
{
    return mGifRecorder && mGifRecorder->recording;
}

int Engine::gifRecordingFrameCount() const
{
    return mGifRecorder ? mGifRecorder->frameCount : 0;
}

const std::string& Engine::gifRecordingFilename() const
{
    static const std::string empty;
    return mGifRecorder ? mGifRecorder->filename : empty;
}

namespace
{

// The scene buffers are rounded up to this. Two things downstream want it and
// a raw window size gives neither:
//
//  - bloom and SSAO run at half resolution, and an odd height makes the halves
//    cover one row less than the full target. Every upsample from them is then
//    half a pixel off, which reads as shimmer along edges rather than as the
//    misalignment it is.
//  - the tiled light cull dispatches one group per 32x32 tile, so a size that
//    is not a multiple of 32 leaves the last row of tiles hanging outside.
//
// Rounding up rather than down keeps the whole image: the final post pass
// stretches the buffer to the presentation rect, and the camera takes its
// aspect from that rect, so a slightly larger buffer only changes sampling
// density - it cannot distort geometry.
constexpr u32 kRenderAlignment = 32;

u32 alignUp(u32 value)
{
    return ((glm::max(value, 1u) + kRenderAlignment - 1) / kRenderAlignment) * kRenderAlignment;
}

} // namespace

void Engine::resolveRenderSize(const Rect& rect, u32& width, u32& height) const
{
    if (mRenderResolution.width != 0 && mRenderResolution.height != 0)
    {
        width = alignUp(mRenderResolution.width);
        height = alignUp(mRenderResolution.height);
        return;
    }

    const f32 scale = mRenderResolution.scale;
    width =
        alignUp(static_cast<u32>(glm::max(1.0f, glm::round(static_cast<f32>(rect.width) * scale))));
    height = alignUp(
        static_cast<u32>(glm::max(1.0f, glm::round(static_cast<f32>(rect.height) * scale))));
}

bool Engine::render(Scene& scene)
{
    RADION_PROFILE_SCOPE("Engine render");
    if (!mFrameActive || !scene.activeCamera())
        return false;

    int width = 0;
    int height = 0;
    mWindow.getDrawableSize(width, height);
    if (width <= 0 || height <= 0)
        return false;

    Camera* camera = scene.activeCamera();
    const PresentationView presentation =
        computePresentation(mPresentation, static_cast<u32>(width), static_cast<u32>(height));
    camera->setAspect(presentation.aspect);
    u32 renderWidth = 0;
    u32 renderHeight = 0;
    resolveRenderSize(presentation.rect, renderWidth, renderHeight);
    return renderInternal(scene, renderWidth, renderHeight, &presentation.rect, nullptr);
}

bool Engine::renderToTexture(Scene& scene, u32 width, u32 height, RenderTextureOutput& output,
                             const RenderTextureSettings& settings)
{
    output = RenderTextureOutput();
    if (!mFrameActive || width == 0 || height == 0 || settings.outputIndex >= 2 ||
        !scene.activeCamera())
        return false;

    const u32 renderWidth = alignUp(width);
    const u32 renderHeight = alignUp(height);
    Camera* camera = scene.activeCamera();
    camera->setAspect(static_cast<f32>(width) / static_cast<f32>(height));
    if (!renderInternal(scene, renderWidth, renderHeight, nullptr, &output, &settings))
    {
        output = RenderTextureOutput();
        return false;
    }
    // The offscreen resolve leaves its target bound. ImGui is submitted after
    // the scene and must return to the window framebuffer without clearing it.
    ClearValue noClear;
    noClear.bits = 0;
    mGpu->setTarget(TargetHandle(), noClear);
    return true;
}

bool Engine::renderToTexture(Scene& scene, const RenderView& view, u32 width, u32 height,
                             RenderTextureOutput& output, const RenderTextureSettings& settings)
{
    output = RenderTextureOutput();
    if (!mFrameActive || width == 0 || height == 0 || settings.outputIndex >= 2)
        return false;

    const u32 renderWidth = alignUp(width);
    const u32 renderHeight = alignUp(height);
    if (!renderInternal(scene, renderWidth, renderHeight, nullptr, &output, &settings, &view))
    {
        output = RenderTextureOutput();
        return false;
    }
    ClearValue noClear;
    noClear.bits = 0;
    mGpu->setTarget(TargetHandle(), noClear);
    return true;
}

bool Engine::renderInternal(Scene& scene, u32 renderWidth, u32 renderHeight,
                            const Rect* presentRect, RenderTextureOutput* output,
                            const RenderTextureSettings* textureSettings,
                            const RenderView* explicitView)
{
    const RenderTextureSettings defaultTextureSettings;
    const RenderTextureSettings& settings =
        textureSettings ? *textureSettings : defaultTextureSettings;

    u32 passes = mEnabledPasses;
    if (!settings.shadows)
        passes &= ~RenderPassShadows;
    if (!settings.planarReflections)
        passes &= ~RenderPassPlanarReflections;
    if (!settings.postProcess)
        passes &= ~RenderPassPostProcess;
    if (!settings.ambientOcclusion)
        passes &= ~RenderPassAmbientOcclusion;
    if (!settings.volumetrics)
        passes &= ~RenderPassVolumetrics;
    if (!settings.lensFlares)
        passes &= ~RenderPassLensFlares;
    if (!settings.temporalAA)
        passes &= ~RenderPassTemporalAA;
    // TAA resolves inside the post-process stack and jitters the projection
    // to feed it. With post-process off there is nothing to resolve into, so
    // leaving the jitter on would only shake the image.
    if (!(passes & RenderPassPostProcess))
        passes &= ~(RenderPassTemporalAA | RenderPassAmbientOcclusion | RenderPassVolumetrics);
    const bool wantShadows = (passes & RenderPassShadows) != 0;
    const bool wantPostProcess = (passes & RenderPassPostProcess) != 0;

    // Here rather than in render(): every path into this function needs the
    // sun current, and an editor that only ever renders through
    // renderToTexture() never calls render() at all. updateSun() recomputes
    // from timeOfDay with no accumulation, so running it per render is safe.
    mSky.updateSun();
    if (mSky.enabled)
    {
        if (DirectionalLight* sun = scene.electedSunLight())
        {
            if (mSky.sunFromSky)
                sun->owner()->lookAt(sun->owner()->globalPosition() - mSky.sunDirection);
            else
                mSky.sunDirection = -sun->owner()->forward();
        }
    }

    FrameContext frame;
    frame.list = mRenderList;
    // RenderView is commonly assembled on the caller's stack each frame, so
    // its address cannot identify a temporal stream. The output object is
    // persistent for the editor/game view that owns this history instead.
    const void* viewIdentity = explicitView ? static_cast<const void*>(output) : nullptr;
    if (explicitView)
    {
        // An editor observer's own render: nothing here is scene.activeCamera(),
        // not even to read it - buildRenderList()'s explicit-view overload
        // does not touch it either, and everything past this point only
        // ever reads frame.*.
        RADION_PROFILE_SCOPE("Scene build list");
        frame.view = explicitView->view;
        frame.projectionNoJitter = explicitView->projection;
        frame.viewProjectionNoJitter = frame.projectionNoJitter * frame.view;
        if (!scene.buildRenderList(*mRenderList, frame.viewProjectionNoJitter,
                                   explicitView->position, 0, false,
                                   settings.previewOcclusionCulling))
            return false;
        frame.projection = frame.projectionNoJitter;
        frame.viewProjection = frame.viewProjectionNoJitter;
        frame.cameraPosition = explicitView->position;
        frame.fieldOfView = explicitView->fieldOfView;
        frame.aspect = explicitView->aspect;
        frame.nearPlane = explicitView->nearPlane;
    }
    else
    {
        Camera* camera = scene.activeCamera();
        if (!camera)
            return false;
        viewIdentity = camera;
        frame.view = camera->viewMatrix();
        frame.projectionNoJitter = camera->projectionMatrix();
        frame.viewProjectionNoJitter = frame.projectionNoJitter * frame.view;
        {
            RADION_PROFILE_SCOPE("Scene build list");
            if (!scene.buildRenderList(*mRenderList, frame.viewProjectionNoJitter,
                                       camera->owner()->globalPosition()))
                return false;
        }
        frame.projection = frame.projectionNoJitter;
        frame.viewProjection = frame.viewProjectionNoJitter;
        frame.cameraPosition = camera->owner()->globalPosition();
        frame.fieldOfView = camera->fieldOfView();
        frame.aspect = camera->aspect();
        frame.nearPlane = camera->nearPlane();
    }
    const u32 temporalIndex = output ? settings.outputIndex + 1 : 0;
    TemporalState& temporal = mTemporal[temporalIndex];
    frame.temporalAA = (passes & RenderPassTemporalAA) != 0 && mPostProcess->taaEnabled;
    const bool resetTemporal = !frame.temporalAA || !temporal.valid ||
                               temporal.width != renderWidth ||
                               temporal.height != renderHeight ||
                               temporal.viewIdentity != viewIdentity;
    if (resetTemporal)
    {
        temporal.valid = false;
        temporal.viewIdentity = viewIdentity;
        temporal.width = renderWidth;
        temporal.height = renderHeight;
        temporal.jitterPhase = 0;
    }
    frame.prevView = temporal.valid ? temporal.prevView : frame.view;
    frame.prevProjectionNoJitter =
        temporal.valid ? temporal.prevProjectionNoJitter : frame.projectionNoJitter;
    frame.prevViewProjectionNoJitter =
        temporal.valid ? temporal.prevViewProjectionNoJitter : frame.viewProjectionNoJitter;
    frame.prevJitter = temporal.valid ? temporal.prevJitter : glm::vec2(0.0f);
    const auto halton = [](u32 index, u32 base)
    {
        f32 result = 0.0f;
        f32 fraction = 1.0f;
        while (index > 0)
        {
            fraction /= static_cast<f32>(base);
            result += fraction * static_cast<f32>(index % base);
            index /= base;
        }
        return result;
    };
    frame.projection = frame.projectionNoJitter;
    if (frame.temporalAA)
    {
        const u32 jitterSample = temporal.jitterPhase % 8u + 1u;
        frame.jitter = glm::vec2((halton(jitterSample, 2u) - 0.5f) * 2.0f /
                                     static_cast<f32>(renderWidth),
                                 (halton(jitterSample, 3u) - 0.5f) * 2.0f /
                                     static_cast<f32>(renderHeight));
        frame.projection[2][0] += frame.jitter.x;
        frame.projection[2][1] += frame.jitter.y;
    }
    frame.viewProjection = frame.projection * frame.view;
    frame.viewport.x = 0.0f;
    frame.viewport.y = 0.0f;
    frame.viewport.width = static_cast<f32>(renderWidth);
    frame.viewport.height = static_cast<f32>(renderHeight);
    frame.width = renderWidth;
    frame.height = renderHeight;
    frame.deltaTime = scene.deltaTime();
    frame.time = static_cast<f32>(mWindow.getTime());
    frame.sky = &mSky;
    if (mProbe.ready())
    {
        frame.environmentCube = mProbe.cubemap();
        frame.environmentCubeSampler = mProbe.sampler();
        frame.environmentProbePosition = mProbe.position;
        frame.environmentProbeExtents = mProbe.extents;
        frame.environmentProbeMips = mProbe.mipCount();
        frame.environmentProbeIntensity = mProbe.intensity;
    }

    if (!mPostProcess->begin(frame.width, frame.height, frame, temporalIndex, resetTemporal))
        return false;

    // Before anything else this frame: the capture renders into its own
    // targets and the reflection has to exist before the surfaces that read
    // it are drawn.
    if (mProbe.consumeCapture(frame.deltaTime, mProbeCaptureDeferred))
    {
        const auto captureStart = std::chrono::steady_clock::now();
        RADION_PROFILE_SCOPE("Environment probe");
        RADION_GPU_PROFILE_SCOPE("Environment probe");
        if (mProbe.content == EnvironmentProbe::Content::FaceColors)
            mProbe.captureFaceColors();
        else
            mRenderer->captureEnvironment(mProbe, scene, frame);
        const auto captureEnd = std::chrono::steady_clock::now();
        mProbe.recordCapture(
            std::chrono::duration<f32, std::milli>(captureEnd - captureStart).count(), frame.time);
    }

    // Every placeable ReflectionProbe the scene owns, with the same
    // invalidation/defer gating as Engine's global probe above.
    for (ReflectionProbe* reflectionProbe : scene.reflectionProbes())
    {
        EnvironmentProbe& probe = reflectionProbe->probe();
        if (!probe.consumeCapture(frame.deltaTime, mProbeCaptureDeferred))
            continue;
        const auto captureStart = std::chrono::steady_clock::now();
        RADION_PROFILE_SCOPE("Reflection probe");
        RADION_GPU_PROFILE_SCOPE("Reflection probe");
        if (probe.content == EnvironmentProbe::Content::FaceColors)
            probe.captureFaceColors();
        else
            mRenderer->captureEnvironment(probe, scene, frame);
        const auto captureEnd = std::chrono::steady_clock::now();
        probe.recordCapture(
            std::chrono::duration<f32, std::milli>(captureEnd - captureStart).count(), frame.time);
    }

    if (wantShadows)
    {
        RADION_GPU_PROFILE_SCOPE("Shadows");
        mRenderer->executeShadows(scene, frame);
    }
    {
        RADION_GPU_PROFILE_SCOPE("Lighting atlas");
        mRenderer->executeLightingPrepare(scene, frame, wantShadows);
        mRenderer->submitDecals(frame);
    }

    // Before the scene's own target is bound: the mirrored view renders into a
    // target of its own, and the surfaces that read it are drawn below.
    if (passes & RenderPassPlanarReflections)
        mRenderer->executeReflection(scene, frame);
    else
        Assets().publishRenderTarget(kReflectionTargetName, TextureHandle());
    mRenderer->executeRefraction(scene, frame);

    ClearValue clear;
    clear.bits = ClearColor | ClearDepth;
    clear.color[0] = 0.05f;
    clear.color[1] = 0.06f;
    clear.color[2] = 0.08f;
    mGpu->setTarget(frame.target, clear);
    const f32 velocityClear[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // Any pass that has not opted into velocity MRT (water, particles,
    // vegetation) retains this value and therefore uses current colour only
    // in TAA. A wrong vector is substantially worse than no history.
    const f32 reactiveClear[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    mGpu->clearColorAttachment(frame.target, 1, velocityClear);
    mGpu->clearColorAttachment(frame.target, 2, reactiveClear);
    const bool ambientOcclusionActive =
        (passes & RenderPassAmbientOcclusion) != 0 && mPostProcess->enabled &&
        mPostProcess->ssaoEnabled;
    {
        RADION_GPU_PROFILE_SCOPE("Depth prepass");
        mRenderer->executeDepth(frame);
    }
    if (scene.occlusionQueryEnabled() && !explicitView)
    {
        RADION_PROFILE_SCOPE("Occlusion queries");
        RADION_GPU_PROFILE_SCOPE("Occlusion queries");
        // Reads last frame's verdict for every entry this frame's
        // buildRenderList() already saw, then launches this frame's query
        // against the depth prepass just above - the result of THIS one
        // only ever gets read next frame, from the top of the next
        // buildRenderList() call.
        scene.updateOcclusionQueries(frame.target, frame.viewProjection, frame.cameraPosition);
    }
    {
        RADION_PROFILE_SCOPE("SSAO");
        RADION_GPU_PROFILE_SCOPE("SSAO");
        if (ambientOcclusionActive)
            frame.ambientOcclusion = mPostProcess->computeSSAO(frame.projection);
    }
    {
        RADION_PROFILE_SCOPE("Tiled light cull");
        RADION_GPU_PROFILE_SCOPE("Tiled light cull");
        mRenderer->executeLightingCull(frame, mPostProcess->sceneDepth(), frame.width,
                                       frame.height);
    }
    {
        RADION_GPU_PROFILE_SCOPE("Forward");
        mRenderer->execute(frame);
    }
    if ((passes & RenderPassVolumetrics) && mPostProcess->enabled)
    {
        RADION_PROFILE_SCOPE("Volumetric");
        RADION_GPU_PROFILE_SCOPE("Volumetric");
        mRenderer->executeVolumetric(frame, *mPostProcess);
    }
    if (passes & RenderPassLensFlares)
    {
        RADION_PROFILE_SCOPE("Lens flare");
        RADION_GPU_PROFILE_SCOPE("Lens flare");
        mRenderer->executeLensFlare(frame, *mPostProcess);
    }
    {
        RADION_PROFILE_SCOPE("Post process");
        RADION_GPU_PROFILE_SCOPE("Post process");
        if (output)
        {
            output->color =
                mPostProcess->resolveToTexture(settings.outputIndex, wantPostProcess);
            output->depth = mPostProcess->sceneDepth();
            output->width = mPostProcess->sceneWidth();
            output->height = mPostProcess->sceneHeight();
            if (!output->valid())
                return false;
        }
        else
        {
            int windowWidth = 0;
            int windowHeight = 0;
            mWindow.getDrawableSize(windowWidth, windowHeight);
            mPostProcess->resolve(*presentRect, static_cast<u32>(windowWidth),
                                  static_cast<u32>(windowHeight));
        }
    }
    if (!output && (debugShowShadowCascades || debugShowShadowAtlas))
    {
        int width = 0;
        int height = 0;
        mWindow.getDrawableSize(width, height);
        mRenderer->debugDrawShadows(debugShowShadowCascades, debugShowShadowAtlas,
                                    static_cast<u32>(width), static_cast<u32>(height));
    }
    if (frame.temporalAA)
    {
        temporal.prevView = frame.view;
        temporal.prevProjectionNoJitter = frame.projectionNoJitter;
        temporal.prevViewProjectionNoJitter = frame.viewProjectionNoJitter;
        temporal.prevJitter = frame.jitter;
        ++temporal.jitterPhase;
        temporal.valid = true;
    }
    else
    {
        temporal.valid = false;
        temporal.jitterPhase = 0;
    }
    return true;
}

bool Engine::presentLoadingFrame(const char* stage, f32 progress)
{
    if (!update())
        return false;

    const s32 width = static_cast<s32>(getWindow().getWidth());
    const s32 height = static_cast<s32>(getWindow().getHeight());

    if (!mLoadingBatch)
    {
        mLoadingBatch = new BatchRenderer();
        BatchRenderer::Config config;
        config.maxVertices = 4096;
        config.maxDrawCalls = 8;
        config.enableProfiling = false;
        if (!mLoadingBatch->init(config))
        {
            delete mLoadingBatch;
            mLoadingBatch = nullptr;
            flip();
            return true;
        }
    }

    // Straight to the backbuffer, cleared to black: the level behind this is
    // half-built, and a loading screen showing pieces of it arriving is worse
    // than showing nothing.
    ClearValue clear;
    clear.bits = ClearColor | ClearDepth;
    clear.color[0] = 0.0f;
    clear.color[1] = 0.0f;
    clear.color[2] = 0.0f;
    clear.color[3] = 1.0f;
    mGpu->setTarget(TargetHandle(), clear);
    mGpu->setViewport(Viewport{0.0f, 0.0f, static_cast<f32>(width), static_cast<f32>(height)});

    mLoadingBatch->resize(width, height);
    mLoadingBatch->update();
    mLoadingBatch->loadIdentity();
    mLoadingBatch->setDepthTest(false);
    mLoadingBatch->setDepthWrite(false);
    mLoadingBatch->setCullFace(false);
    mLoadingBatch->setBlend(true);
    mLoadingBatch->setBlendMode(BatchRenderer::BlendMode::Alpha);

    constexpr f32 kTextSize = 16.0f;
    const f32 centerX = static_cast<f32>(width) * 0.5f;
    const f32 centerY = static_cast<f32>(height) * 0.5f;
    const f32 textWidth = mLoadingBatch->textWidth(kTextSize, stage);

    // Amber rather than white: white on black is the one pairing that reads
    // as a blown-out gap in the picture rather than as text put there on
    // purpose.
    mLoadingBatch->setColor(static_cast<unsigned char>(255), static_cast<unsigned char>(200),
                            static_cast<unsigned char>(60));
    mLoadingBatch->drawText(centerX - textWidth * 0.5f, centerY - kTextSize, kTextSize, stage);

    if (progress >= 0.0f)
    {
        constexpr f32 kBarWidth = 320.0f;
        constexpr f32 kBarHeight = 6.0f;
        const f32 left = centerX - kBarWidth * 0.5f;
        const f32 top = centerY + kTextSize;
        mLoadingBatch->setTexture(TextureHandle());
        mLoadingBatch->setColor(static_cast<unsigned char>(40), static_cast<unsigned char>(30),
                                static_cast<unsigned char>(12));
        mLoadingBatch->drawRect(left, top, kBarWidth, kBarHeight, true);
        mLoadingBatch->setColor(static_cast<unsigned char>(255), static_cast<unsigned char>(170),
                                static_cast<unsigned char>(40));
        mLoadingBatch->drawRect(left, top, kBarWidth * glm::clamp(progress, 0.0f, 1.0f), kBarHeight,
                                true);
    }

    mLoadingBatch->flip();

    flip();
    return true;
}

bool Engine::waitForAsyncLoads(const char* stage)
{
    // The totals only ever grow while work is outstanding - a mesh landing
    // can queue the textures its materials need - so the denominator is the
    // high-water mark, not the count at entry, or the bar would jump back.
    u32 peak = 0;
    char label[256];

    for (;;)
    {
        const u32 meshes = Assets().pendingAsyncMeshLoads();
        const AsyncTextureLoader& loader = AsyncTextureLoader::getSingleton();
        const u32 textures = loader.pendingCount() + loader.completedCount();
        const u32 remaining = meshes + textures;
        if (remaining == 0)
            break;

        peak = glm::max(peak, remaining);
        std::snprintf(label, sizeof(label), "%s  %u mesh / %u texture", stage, meshes, textures);
        const f32 progress =
            peak > 0 ? 1.0f - static_cast<f32>(remaining) / static_cast<f32>(peak) : 0.0f;
        if (!presentLoadingFrame(label, progress))
            return false;
    }

    return true;
}

void Engine::flip(const RenderList* profileList)
{
    if (!mFrameActive)
        return;

    Profiler::getSingleton().endFrame();
 
    const u64 overlayStart = SDL_GetPerformanceCounter();
    const RenderList& displayedList = profileList ? *profileList : *mRenderList;
    if (mBuiltinPanelsVisible)
    {
        mImGui.drawProfiler(mGpu->stats(), displayedList.stats());
        RenderResolution resolution = mRenderResolution;
        mImGui.drawPostProcess(*mPostProcess, resolution, mRenderer->volumetric());
        setRenderResolution(resolution);
        mImGui.drawSky(mSky);
    }
    if (mImGuiVisible)
    {
        ExternalGLScope scope(*mGpu);
        mImGui.flip();
    }
    else
    {
        // update() always starts an ImGui frame. A standalone runner hides
        // it for a clean game window, but Dear ImGui still requires that
        // frame to be closed before the next NewFrame().
        mImGui.endFrame();
    }

    GPUProfiler::getSingleton().endFrame();
    mGpu->endFrame();
    mOverlayMilliseconds = static_cast<f32>((SDL_GetPerformanceCounter() - overlayStart) * 1000.0 /
                                            SDL_GetPerformanceFrequency());

 
    const u64 presentStart = SDL_GetPerformanceCounter();
    mGpu->present();
    mPresentMilliseconds = static_cast<f32>((SDL_GetPerformanceCounter() - presentStart) * 1000.0 /
                                            SDL_GetPerformanceFrequency());
    if (mGifRecorder)
        mGifRecorder->frame();
    mFrameActive = false;
}

} // namespace Radion
