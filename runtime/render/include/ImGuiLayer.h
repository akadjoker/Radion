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
    // resolution is read and written in place; the caller clamps and applies
    // it, since it belongs to the engine and not to the stack.
    // volumetric is null until the renderer that owns it exists; the panel
    // just skips the section in that case.
    void drawPostProcess(PostProcessStack& post, RenderResolution& resolution,
                         VolumetricPass* volumetric);
    void drawPostProcessContents(PostProcessStack& post, RenderResolution& resolution,
                                 VolumetricPass* volumetric);
    void drawSky(SkySettings& sky);
    void drawSkyContents(SkySettings& sky);
    // NewFrame() still runs when Engine hides ImGui so user code may safely
    // build optional UI. A hidden frame must nevertheless be ended, or the
    // next NewFrame() asserts before the game gets to its second update.
    void endFrame();
    void flip();

private:
    Platform::Window* mWindow = nullptr;
    // The header line is read off the window and the device rather than the
    // profiler's samples, so it needs its own hold to change on the same
    // cadence as the tables below it - see ProfileSample::display.
    f64 mHeaderRefresh = 0.0;
    f32 mHeaderFrameMilliseconds = 0.0f;
    f32 mHeaderGpuMilliseconds = 0.0f;
    int mHeaderFps = 0;
};

} // namespace Render
} // namespace Radion

#endif // RADION_IMGUI_LAYER_H
