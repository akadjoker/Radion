#ifndef RADION_ENGINE_H
#define RADION_ENGINE_H

#include "EnvironmentProbe.h"
#include "FrameContext.h"
#include "GPU.h"
#include "GPUCaps.h"
#include "ImGuiLayer.h"
#include "Sky.h"
#include "SceneManager.h"
#include "Window.h"

#include <string>
#include <memory>

namespace Radion
{

class Scene;
class Renderer;
class RenderList;
class BatchRenderer;
class ScreenDrawPass;
struct RenderListStats;
class PostProcessStack;
class Lighting;
class VolumetricPass;
class LensFlarePass;
class DecalSystem;

struct RenderTextureOutput
{
    TextureHandle color;
    TextureHandle depth;
    u32 width = 0;
    u32 height = 0;

    bool valid() const
    {
        return color.valid() && width > 0 && height > 0;
    }
};

enum RenderPassBits : u32
{
    RenderPassShadows = 1 << 0,
    RenderPassPlanarReflections = 1 << 1,
    RenderPassPostProcess = 1 << 2,
    RenderPassAmbientOcclusion = 1 << 3,
    RenderPassVolumetrics = 1 << 4,
    RenderPassLensFlares = 1 << 5,
    RenderPassTemporalAA = 1 << 6,
    RenderPassAll = 0xFFFFFFFFu
};

struct RenderTextureSettings
{
    u32 outputIndex = 0;
    bool shadows = true;
    bool planarReflections = true;
    bool postProcess = true;
    bool ambientOcclusion = true;
    bool volumetrics = true;
    bool lensFlares = true;
    // Temporal accumulation also jitters the projection. Editor/navigation
    // views can disable it without affecting the Game View or main render.
    bool temporalAA = true;
    // Explicit-view renders only: show the game camera's occlusion verdicts.
    bool previewOcclusionCulling = false;
};

// A viewpoint to render from that is not backed by any Scene entity - an
// editor's own free-look observer camera, chiefly, which has no business
// owning a GameObject just to hold a view/projection pair. Every field
// renderInternal() would otherwise have read off a Camera component,
// gathered by hand instead.
struct RenderView
{
    glm::mat4 view = glm::mat4(1.0f);
    glm::mat4 projection = glm::mat4(1.0f);
    glm::vec3 position = glm::vec3(0.0f);
    f32 fieldOfView = 60.0f;
    f32 aspect = 1.0f;
    f32 nearPlane = 0.1f;
};

struct EngineConfig
{
    const char* title = "Radion";
    int width = 1280;
    int height = 720;
    int monitor = 0;
    bool resizable = true;
    bool fullscreen = false;

    // Off for offscreen tools (the lightmap baker CLI, say) - still a real
    // window with a real GL context underneath, SDL just never shows it.
    bool visible = true;

    // On in Debug builds: the driver's own diagnostics cost nothing to have
    // and a demo that runs without them hides its mistakes until something
    // looks wrong on screen, which is the expensive way to find them.
#ifdef RADION_DEBUG
    bool debugContext = true;
#else
    bool debugContext = false;
#endif
};

class Engine
{
public:
    Engine();
    ~Engine();

    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    bool initialize(const EngineConfig& config = {});
    void shutdown();

    bool update();
    Scene* createScene();
    bool setActiveScene(Scene* scene);
    Scene* activeScene()
    {
        return mSceneManager.active();
    }
    const Scene* activeScene() const
    {
        return mSceneManager.active();
    }
    SceneManager& sceneManager()
    {
        return mSceneManager;
    }
    bool render(Scene& scene);
    bool renderToTexture(Scene& scene, u32 width, u32 height, RenderTextureOutput& output,
                         const RenderTextureSettings& settings = RenderTextureSettings());
    // Same, from a RenderView instead of scene.activeCamera() - an editor
    // observer's own render, so its every frame does not mean saving and
    // restoring every field of whatever Camera happens to be the scene's
    // real one (position, rotation, FOV, ortho/perspective, ...) around a
    // borrowed render, the fragile shape that kept re-breaking in one field
    // or another. Nothing past FrameContext setup ever reads
    // scene.activeCamera() again - shadows/lighting/decals/everything else
    // only ever reads frame.* - so this path does not touch the scene's
    // real camera at all, not even to read it.
    // Renders every Camera in the scene that has recording() on into its own
    // texture, at its own resolution, and leaves the handle on the camera.
    // Called at the top of render(); public so a custom render loop that does
    // not use render() can still drive the sensors. Returns how many were
    // rendered.
    //
    // Each one is a full render of the scene from that camera, so the cost is
    // per recording camera - which is why the resolution is the camera's own
    // and usually small.
    u32 renderRecordingCameras(Scene& scene);

    bool renderToTexture(Scene& scene, const RenderView& view, u32 width, u32 height,
                         RenderTextureOutput& output,
                         const RenderTextureSettings& settings = RenderTextureSettings());
    // Custom render loops can provide their list so the built-in profiler
    // displays the same counters they rendered. Scene-driven loops omit it.
    void flip(const RenderList* profileList = nullptr);

    // Presents one black frame carrying `stage` centred on it, and returns
    // false once the window has been asked to close. Everything a caller
    // loading a large level needs between its own steps, so the window keeps
    // drawing instead of freezing while a mesh streams in - the async loads
    // update() pumps are what finishes during these frames.
    // `progress` in [0,1] draws a bar under the text; negative draws none.
    //
    // Deliberately draws nothing but itself: no scene is rendered, because
    // during a load the scene is half of one, and the passes have their own
    // render state to set up and tear down that a loading frame has no
    // business walking through.
    bool presentLoadingFrame(const char* stage, f32 progress = -1.0f);

    // Pumps loading frames until every async mesh and texture queued by a
    // scene load has landed, reporting how far along it is. False when the
    // window closed part-way through, which leaves the scene incomplete and
    // is the caller's cue to quit rather than carry on.
    bool waitForAsyncLoads(const char* stage);

    // Captures the presented backbuffer as an animated GIF. Recording is
    // sampled by flip(), so callers only need to toggle it around their loop.
    bool startGifRecording();
    void stopGifRecording();
    bool isGifRecording() const;
    int gifRecordingFrameCount() const;
    const std::string& gifRecordingFilename() const;
    void setBuiltinPanelsVisible(bool visible)
    {
        mBuiltinPanelsVisible = visible;
    }
    // Whether flip() composites ImGui's draw data onto the backbuffer at
    // all - unlike setBuiltinPanelsVisible() (which only skips the engine's
    // own profiler/post-process/sky panels), this hides everything ImGui
    // drew this frame, a demo's own windows included, with no cooperation
    // needed from the demo itself: NewFrame() still runs every update() so
    // ImGui::Begin()/SliderFloat()/etc. stay perfectly safe to call, only
    // ImGui::Render() and the actual draw call are skipped. A clean shot of
    // the game with no UI at all.
    void setImGuiVisible(bool visible)
    {
        mImGuiVisible = visible;
    }
    bool imGuiVisible() const
    {
        return mImGuiVisible;
    }
    // False while ImGui is hidden, since a panel nobody can see must not eat
    // the game's input. Game code that picks with the mouse or reads keys
    // asks these before acting.
    bool uiWantsMouse() const
    {
        return mImGuiVisible && mImGui.wantsMouse();
    }
    bool uiWantsKeyboard() const
    {
        return mImGuiVisible && mImGui.wantsKeyboard();
    }
    void drawSkyContents();
    void drawPostProcessContents();
    void drawProfilerContents();

    bool isInitialized() const
    {
        return mInitialized;
    }
    bool isRunning() const
    {
        return mWindow.isOpen();
    }
    void requestClose()
    {
        mWindow.requestClose();
    }

    // Set once by EngineSettings::load() so shutdown() can save back to the
    // same file without every demo remembering to call it on the way out -
    // that is how window position/size survives a plain window-close instead
    // of only the "Guardar settings" button.
    void setSettingsFile(const std::string& filename)
    {
        mSettingsFile = filename;
    }
    const std::string& settingsFile() const
    {
        return mSettingsFile;
    }

    Platform::Window& getWindow()
    {
        return mWindow;
    }
    const Platform::Window& getWindow() const
    {
        return mWindow;
    }
    GPU& getGPU()
    {
        return *mGpu;
    }
    const GPUCaps& getCaps() const
    {
        return mGpu->caps();
    }
    PostProcessStack& postProcess()
    {
        return *mPostProcess;
    }
    void setPresentation(const PresentationSettings& settings)
    {
        mPresentation = settings;
    }
    const PresentationSettings& presentation() const
    {
        return mPresentation;
    }
    struct CascadeShadowSettings* cascadeSettings();
    f32 cascadeHalfExtent(u32 cascade) const;
    f32 cascadeSplit(u32 cascade) const;
    const RenderListStats* shadowListStats() const;
    const char* sunShadowStatus() const;
    const RenderListStats* mainRenderListStats() const;
    TextureHandle directionalShadowTexture() const;
    void debugDrawTexture(TextureHandle texture, bool isArray, u32 layer, TargetHandle target,
                          u32 width, u32 height,
                          const glm::vec4& sourceRect = glm::vec4(1.0f, 1.0f, 0.0f, 0.0f));
    void debugDrawCubemap(TextureHandle texture, u32 face, u32 mip, TargetHandle target,
                          u32 width, u32 height);
    Lighting* lighting();
    VolumetricPass* volumetric();
    LensFlarePass* lensFlare();
    const char* reflectionSource() const;
    void setEnabledPasses(u32 mask);
    u32 enabledPasses() const;
    void setPassEnabled(u32 bit, bool enabled);
    bool passEnabled(u32 bit) const;
    DecalSystem* decals();
    void setSunShadows(bool enabled);
    bool sunShadows();
    void setPointShadows(bool enabled);
    bool pointShadows();
    void setSpotShadows(bool enabled);
    bool spotShadows();

    // The environment probe, for image-based reflections. Owned here for the
    // same reason the sky is: it is frame-wide state every lit surface reads,
    // not something a single object carries.
    EnvironmentProbe& environmentProbe()
    {
        return mProbe;
    }
    void setProbeCaptureDeferred(bool deferred) { mProbeCaptureDeferred = deferred; }

    SkySettings& sky()
    {
        return mSky;
    }

    // Loads the six faces through AssetManager::loadCubemap() and, on
    // success, points mSky at the result and switches its mode to Cubemap.
    // An empty name clears the handle and the name instead of loading
    // anything. Kept on Engine (not just AssetManager) because it also owns
    // where in SkySettings the result lands and which mode follows it.
    void setSkyCubemap(const std::string& baseName);

    // What the scene is rendered into, see RenderResolution. Lowering it buys
    // back fill rate and bandwidth - the two things a full-screen effect
    // spends. The UI is drawn afterwards at window resolution, so it stays
    // crisp whatever this is set to.
    void setRenderResolution(const RenderResolution& resolution);
    const RenderResolution& renderResolution() const
    {
        return mRenderResolution;
    }
    RenderResolution& renderResolution()
    {
        return mRenderResolution;
    }

    // Raw shadow textures blitted into the backbuffer's corners - cascades
    // bottom-left, atlas bottom-right. See ShadowDebugView.
    bool debugShowShadowCascades = false;
    bool debugShowShadowAtlas = false;

    bool debugShowPhysicsShapes = false;
    bool debugShowPhysicsContacts = false;
    bool debugShowPhysicsJoints = false;

    // Every Radion::Obstacle's shape and seenFrom() arrow - Scene::
    // debugDrawObstacles(), same family as the physics flags above. The
    // selected object's own Obstacle draws regardless, in ViewportPanel.cpp.
    bool debugShowAIObstacles = false;

private:
    struct GifRecorder;

    struct TemporalState
    {
        glm::mat4 prevView = glm::mat4(1.0f);
        glm::mat4 prevProjectionNoJitter = glm::mat4(1.0f);
        glm::mat4 prevViewProjectionNoJitter = glm::mat4(1.0f);
        glm::vec2 prevJitter = glm::vec2(0.0f);
        const void* viewIdentity = nullptr;
        u32 width = 0;
        u32 height = 0;
        u32 jitterPhase = 0;
        bool valid = false;
    };

    bool renderInternal(Scene& scene, u32 renderWidth, u32 renderHeight, const Rect* presentRect,
                        RenderTextureOutput* output,
                        const RenderTextureSettings* textureSettings = nullptr,
                        const RenderView* explicitView = nullptr);
    // The pixel size the scene buffers get this frame.
    void resolveRenderSize(const Rect& rect, u32& width, u32& height) const;

    Platform::Window mWindow;
    std::string mSettingsFile;
    GPU* mGpu = nullptr;
    Render::ImGuiLayer mImGui;
    Renderer* mRenderer = nullptr;
    u32 mEnabledPasses = RenderPassAll;
    RenderList* mRenderList = nullptr;
    PostProcessStack* mPostProcess = nullptr;
    SceneManager mSceneManager;
    PresentationSettings mPresentation;
    SkySettings mSky;
    EnvironmentProbe mProbe;
    bool mProbeCaptureDeferred = false;
    RenderResolution mRenderResolution;
    TemporalState mTemporal[3];
    // How long the last flip() spent on the overlay and on present() - both
    // reported to the profiler from update(), since the frame they belong to
    // is already closed by the time they run.
    f32 mOverlayMilliseconds = 0.0f;
    f32 mPresentMilliseconds = 0.0f;
    // Built on the first loading frame and kept for the rest of the run: a
    // level reload wants it again, and it is a few kilobytes.
    BatchRenderer* mLoadingBatch = nullptr;
    // Draws the ScreenDraw queue over the resolved window frame - built on
    // first use, same reasoning as mLoadingBatch.
    ScreenDrawPass* mScreenDrawPass = nullptr;
    bool mInitialized = false;
    bool mFrameActive = false;
    bool mBuiltinPanelsVisible = true;
    bool mImGuiVisible = true;
    std::unique_ptr<GifRecorder> mGifRecorder;
};

} // namespace Radion

#endif // RADION_ENGINE_H
