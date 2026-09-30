#ifndef RADION_IMGUI_LAYER_H
#define RADION_IMGUI_LAYER_H

namespace Radion
{
struct GPUStats;
struct RenderListStats;
struct RenderResolution;
class PostProcessStack;
class VolumetricPass;
struct SkySettings;

namespace Platform
{
class Window;
}

namespace Render
{

class ImGuiLayer
{
public:
    bool initialize(Platform::Window& window);
    void shutdown();
    void update();
    void drawProfiler(const GPUStats& gpu, const RenderListStats& renderList);
    void drawProfilerContents(const GPUStats& gpu, const RenderListStats& renderList);
    // resolution is read and written in place; the caller clamps and applies it.
    // volumetric may be null; the panel then skips that section.
    void drawPostProcess(PostProcessStack& post, RenderResolution& resolution,
                         VolumetricPass* volumetric);
    void drawPostProcessContents(PostProcessStack& post, RenderResolution& resolution,
                                 VolumetricPass* volumetric);
    void drawSky(SkySettings& sky);
    void drawSkyContents(SkySettings& sky);
    // Whether a panel is under the pointer or has keyboard focus; game input must check first.
    bool wantsMouse() const;
    bool wantsKeyboard() const;
    // A hidden frame must still be ended, or the next NewFrame() asserts.
    void endFrame();
    void flip();

private:
    Platform::Window* mWindow = nullptr;
    // Own hold so the header refreshes on the same cadence as the tables (see ProfileSample::display).
    f64 mHeaderRefresh = 0.0;
    f32 mHeaderFrameMilliseconds = 0.0f;
    f32 mHeaderGpuMilliseconds = 0.0f;
    int mHeaderFps = 0;
};

} // namespace Render
} // namespace Radion

#endif // RADION_IMGUI_LAYER_H
