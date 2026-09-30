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
    // Temporal accumulation jitters the projection; editor views can disable it.
    bool temporalAA = true;
    // Explicit-view renders only: show the game camera's occlusion verdicts.
    bool previewOcclusionCulling = false;
};

// A viewpoint not backed by a Scene entity (an editor's free-look camera).
struct RenderView
{
    Math::mat4 view = Math::mat4(1.0f);
    Math::mat4 projection = Math::mat4(1.0f);
    Math::vec3 position = Math::vec3(0.0f);
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

    // Off for offscreen tools; still a real window and GL context, just never shown.
    bool visible = true;

    // On in Debug builds: driver diagnostics are free and expose mistakes early.
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
    // Same, from a RenderView: never touches scene.activeCamera(), so no save/restore of its fields.
    // Renders every recording() Camera into its own texture; called at the top of render().
    // Returns how many; cost is per camera.
    u32 renderRecordingCameras(Scene& scene);

    bool renderToTexture(Scene& scene, const RenderView& view, u32 width, u32 height,
                         RenderTextureOutput& output,
                         const RenderTextureSettings& settings = RenderTextureSettings());
    // Custom render loops pass their list so the profiler shows their counters.
    void flip(const RenderList* profileList = nullptr);

    // Presents one black frame with `stage` centred; false once the window is asked to close.
    // `progress` in [0,1] draws a bar; negative draws none. Draws no scene: it is half loaded during a load.
    bool presentLoadingFrame(const char* stage, f32 progress = -1.0f);

    // Pumps loading frames until async mesh/texture loads land. False if the window closed part-way,
    // leaving the scene incomplete: the caller should quit.
    bool waitForAsyncLoads(const char* stage);

    // Recording is sampled by flip().
    bool startGifRecording();
    void stopGifRecording();
    bool isGifRecording() const;
    int gifRecordingFrameCount() const;
    const std::string& gifRecordingFilename() const;
    void setBuiltinPanelsVisible(bool visible)
    {
        mBuiltinPanelsVisible = visible;
    }
    // Whether flip() composites ImGui's draw data; hides all ImGui including a demo's own windows.
    // NewFrame() still runs each update(), so ImGui calls stay safe.
    void setImGuiVisible(bool visible)
    {
        mImGuiVisible = visible;
    }
    bool imGuiVisible() const
    {
        return mImGuiVisible;
    }
    // False while ImGui is hidden, so an invisible panel does not eat game input.
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

    // Set by EngineSettings::load() so shutdown() saves back to the same file.
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
                          const Math::vec4& sourceRect = Math::vec4(1.0f, 1.0f, 0.0f, 0.0f));
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

    EnvironmentProbe& environmentProbe()
    {
        return mProbe;
    }
    void setProbeCaptureDeferred(bool deferred) { mProbeCaptureDeferred = deferred; }

    SkySettings& sky()
    {
        return mSky;
    }

    // Loads the six faces and switches the sky to Cubemap; an empty name clears the handle.
    void setSkyCubemap(const std::string& baseName);

    // What the scene is rendered into; the UI is drawn afterwards at window resolution.
    void setRenderResolution(const RenderResolution& resolution);
    const RenderResolution& renderResolution() const
    {
        return mRenderResolution;
    }
    RenderResolution& renderResolution()
    {
        return mRenderResolution;
    }

    // Shadow textures blitted into the backbuffer's corners (cascades left, atlas right).
    bool debugShowShadowCascades = false;
    bool debugShowShadowAtlas = false;

    bool debugShowPhysicsShapes = false;
    bool debugShowPhysicsContacts = false;
    bool debugShowPhysicsJoints = false;

    // Draws every Obstacle's shape and seenFrom() arrow; the selected one always draws in ViewportPanel.cpp.
    bool debugShowAIObstacles = false;

private:
    struct GifRecorder;

    struct TemporalState
    {
        Math::mat4 prevView = Math::mat4(1.0f);
        Math::mat4 prevProjectionNoJitter = Math::mat4(1.0f);
        Math::mat4 prevViewProjectionNoJitter = Math::mat4(1.0f);
        Math::vec2 prevJitter = Math::vec2(0.0f);
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
    // Reported to the profiler from update(): the frame they belong to is already closed.
    f32 mOverlayMilliseconds = 0.0f;
    f32 mPresentMilliseconds = 0.0f;
    // Built on first loading frame, kept for level reloads.
    BatchRenderer* mLoadingBatch = nullptr;
    ScreenDrawPass* mScreenDrawPass = nullptr;
    bool mInitialized = false;
    bool mFrameActive = false;
    bool mBuiltinPanelsVisible = true;
    bool mImGuiVisible = true;
    std::unique_ptr<GifRecorder> mGifRecorder;
};

} // namespace Radion

#endif // RADION_ENGINE_H
