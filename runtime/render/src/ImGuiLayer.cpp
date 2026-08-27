#include "PCH.h"

#include "ImGuiLayer.h"

#include "AssetManager.h"
#include "GPUProfiler.h"
#include "Log.h"
#include "PostProcess.h"
#include "Profiler.h"
#include "RenderList.h"
#include "Sky.h"
#include "VolumetricPass.h"
#include "Window.h"

#include <backends/imgui_impl_opengl3.h>
#include <backends/imgui_impl_sdl2.h>
#include <imgui.h>

namespace Radion
{
namespace Render
{

namespace
{

int SDLCALL processEvent(void*, SDL_Event* event)
{
    ImGui_ImplSDL2_ProcessEvent(event);
    return 0;
}

} // namespace

bool ImGuiLayer::initialize(Platform::Window& window)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight();

    if (!ImGui_ImplSDL2_InitForOpenGL(window.getNativeWindow(), window.getGLContext()) ||
        !ImGui_ImplOpenGL3_Init("#version 410 core"))
    {
        Log::error("Dear ImGui backend initialization failed");
        ImGui::DestroyContext();
        return false;
    }

    mWindow = &window;
    SDL_AddEventWatch(processEvent, nullptr);
    return true;
}

void ImGuiLayer::shutdown()
{
    if (!mWindow)
        return;

    SDL_DelEventWatch(processEvent, nullptr);
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    mWindow = nullptr;
}

void ImGuiLayer::update()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();
}

bool ImGuiLayer::wantsMouse() const
{
    return mWindow && ImGui::GetCurrentContext() && ImGui::GetIO().WantCaptureMouse;
}

bool ImGuiLayer::wantsKeyboard() const
{
    return mWindow && ImGui::GetCurrentContext() && ImGui::GetIO().WantCaptureKeyboard;
}

void ImGuiLayer::endFrame()
{
    ImGui::EndFrame();
}

void ImGuiLayer::drawProfiler(const GPUStats& gpu, const RenderListStats& renderList)
{
    if (!ImGui::Begin("Profiler"))
    {
        ImGui::End();
        return;
    }

    drawProfilerContents(gpu, renderList);
    ImGui::End();
}

void ImGuiLayer::drawProfilerContents(const GPUStats& gpu, const RenderListStats& renderList)
{
    const Profiler& profiler = Profiler::getSingleton();
    const f64 now = mWindow ? mWindow->getTime() : 0.0;
    if (now - mHeaderRefresh > Profiler::RefreshSeconds)
    {
        mHeaderRefresh = now;
        mHeaderFps = mWindow ? mWindow->getFPS() : 0;
        mHeaderFrameMilliseconds = mWindow ? mWindow->getDeltaTime() * 1000.0f : 0.0f;
        mHeaderGpuMilliseconds = gpu.gpuMilliseconds;
    }
    ImGui::Text("%d FPS   Frame %.2f ms", mHeaderFps, mHeaderFrameMilliseconds);
    ImGui::Text("CPU %.2f ms   GPU %.2f ms", profiler.frameMilliseconds(), mHeaderGpuMilliseconds);
    ImGui::Text("Draws %u   Triangles %u   Dispatches %u", gpu.drawCalls, gpu.triangles,
                gpu.dispatches);
    ImGui::Text("Pipelines %u   Targets %u   Textures %u", gpu.pipelineSwitches, gpu.targetSwitches,
                gpu.textureBinds);
    ImGui::Text("Submitted %u   Culled %u/%u   Packets %u", renderList.submitted,
                renderList.culledMeshes, renderList.culledSubmeshes, renderList.packets);
    ImGui::Text("Lights %u   Dropped %u", renderList.lights, renderList.droppedLights);

    const auto sampleTable = [](const char* id, const ProfileSample* samples, u32 count)
    {
        if (!ImGui::BeginTable(id, 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV))
            return;
        ImGui::TableSetupColumn("Scope");
        ImGui::TableSetupColumn("ms");
        ImGui::TableSetupColumn("avg");
        ImGui::TableSetupColumn("max");
        ImGui::TableHeadersRow();
        for (u32 i = 0; i < count; ++i)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(samples[i].name.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", samples[i].display);
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", samples[i].average);
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", samples[i].maximum);
        }
        ImGui::EndTable();
    };

    if (ImGui::CollapsingHeader("CPU", ImGuiTreeNodeFlags_DefaultOpen))
        sampleTable("profile.cpu", profiler.samples(), profiler.sampleCount());

    const GPUProfiler& gpuProfiler = GPUProfiler::getSingleton();
    if (ImGui::CollapsingHeader("GPU", ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (!gpuProfiler.available())
            ImGui::TextUnformatted("timer queries unavailable");
        else
        {
            // Lags the CPU numbers by a few frames, which is what keeps reading
            // them from stalling the frame that recorded them.
            ImGui::Text("Measured %.2f ms of %.2f ms", gpuProfiler.frameMilliseconds(),
                        mHeaderGpuMilliseconds);
            sampleTable("profile.gpu", gpuProfiler.samples(), gpuProfiler.sampleCount());
        }
    }
}

void ImGuiLayer::drawPostProcess(PostProcessStack& post, RenderResolution& resolution,
                                 VolumetricPass* volumetric)
{
    if (!ImGui::Begin("Post Process"))
    {
        ImGui::End();
        return;
    }

    drawPostProcessContents(post, resolution, volumetric);
    ImGui::End();
}

void ImGuiLayer::drawPostProcessContents(PostProcessStack& post, RenderResolution& resolution,
                                         VolumetricPass* volumetric)
{
    ImGui::Checkbox("Enabled", &post.enabled);
    // Presets first, because picking a number is the common case and typing
    // one is not. "Window" is the only entry that tracks a resizing window;
    // everything else pins the buffers and stops caring how big it gets.
    struct ResolutionPreset
    {
        const char* label;
        u32 width;
        u32 height;
    };
    static const ResolutionPreset presets[] = {{"Window", 0, 0},          {"2560x1440", 2560, 1440},
                                               {"1920x1080", 1920, 1080}, {"1600x900", 1600, 900},
                                               {"1280x720", 1280, 720},   {"960x540", 960, 540},
                                               {"640x360", 640, 360}};

    int current = 0;
    for (int i = 1; i < static_cast<int>(IM_ARRAYSIZE(presets)); ++i)
    {
        if (resolution.width == presets[i].width && resolution.height == presets[i].height)
        {
            current = i;
            break;
        }
    }
    if (ImGui::BeginCombo("Resolution", presets[current].label))
    {
        for (int i = 0; i < static_cast<int>(IM_ARRAYSIZE(presets)); ++i)
        {
            if (ImGui::Selectable(presets[i].label, i == current))
            {
                resolution.width = presets[i].width;
                resolution.height = presets[i].height;
            }
        }
        ImGui::EndCombo();
    }
    if (current == 0)
        ImGui::SliderFloat("Render scale", &resolution.scale, 0.25f, 2.0f, "%.2f");
    ImGui::TextDisabled("Scene -> HDR%s", post.taaEnabled ? " -> TAA" : "");
    ImGui::SameLine();
    for (usize i = 0; i < post.layers().size(); ++i)
    {
        const PostEffect effect = post.layers()[i].effect;
        const char* name = effect == PostEffect::Bloom     ? "Bloom"
                           : effect == PostEffect::ToneMap ? "Tone map"
                                                           : "FXAA";
        ImGui::TextDisabled("-> %s", name);
        if (i + 1 < post.layers().size())
            ImGui::SameLine();
    }
    ImGui::Separator();

    const std::vector<PostLayer>& layers = post.layers();
    for (usize i = 0; i < layers.size(); ++i)
    {
        const PostEffect effect = layers[i].effect;
        const char* name = effect == PostEffect::Bloom     ? "Bloom"
                           : effect == PostEffect::ToneMap ? "Tone map"
                                                           : "FXAA";
        ImGui::PushID(static_cast<int>(i));
        bool active = layers[i].enabled;
        if (ImGui::Checkbox(name, &active))
            post.setEnabled(effect, active);
        ImGui::SameLine();
        ImGui::BeginDisabled(i == 0);
        if (ImGui::ArrowButton("up", ImGuiDir_Up))
        {
            post.move(i, i - 1);
            ImGui::EndDisabled();
            ImGui::PopID();
            break;
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(i + 1 == layers.size());
        if (ImGui::ArrowButton("down", ImGuiDir_Down))
        {
            post.move(i, i + 1);
            ImGui::EndDisabled();
            ImGui::PopID();
            break;
        }
        ImGui::EndDisabled();
        ImGui::PopID();
    }

    ImGui::Separator();
    if (ImGui::CollapsingHeader("Tone map", ImGuiTreeNodeFlags_DefaultOpen))
    {
        int curve = static_cast<int>(post.toneMap);
        const char* curves[] = {"None (show clipping)", "Reinhard", "ACES"};
        if (ImGui::Combo("Curve", &curve, curves, 3))
            post.toneMap = static_cast<ToneMapMode>(curve);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("None exposes clipped highlights; Reinhard is a simple reference; "
                              "ACES preserves highlight colour and contrast.");
        ImGui::SliderFloat("Exposure", &post.exposure, 0.05f, 5.0f, "%.2f");
    }
    if (ImGui::CollapsingHeader("Bloom", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::SliderFloat("Threshold", &post.bloomThreshold, 0.0f, 5.0f, "%.2f");
        ImGui::SliderFloat("Soft knee", &post.bloomSoftKnee, 0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat("Strength", &post.bloomStrength, 0.0f, 2.0f, "%.2f");
    }
    if (ImGui::CollapsingHeader("SSAO", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Active##ssao", &post.ssaoEnabled);
        ImGui::SameLine();
        ImGui::Checkbox("Debug AO##ssao", &post.ssaoDebug);
        ImGui::SameLine();
        ImGui::Checkbox("Blur##ssao", &post.ssaoBlur);
        ImGui::SliderFloat("Radius##ssao", &post.ssaoRadius, 0.05f, 20.0f, "%.2f");
        ImGui::SliderFloat("Bias##ssao", &post.ssaoBias, 0.0f, 1.0f, "%.3f");
        ImGui::SliderFloat("Intensity##ssao", &post.ssaoIntensity, 0.0f, 3.0f, "%.2f");
        int samples = static_cast<int>(post.ssaoSamples);
        if (ImGui::SliderInt("Samples##ssao", &samples, 4, 64))
            post.ssaoSamples = static_cast<u32>(samples);
        ImGui::SliderFloat("Depth rejection##ssao", &post.ssaoDepthSigma, 1.0f, 300.0f, "%.0f");
    }
    if (ImGui::CollapsingHeader("Temporal anti-aliasing", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Active##taa", &post.taaEnabled);
        ImGui::SliderFloat("Still feedback##taa", &post.taaFeedback, 0.0f, 0.98f, "%.3f");
        ImGui::SliderFloat("Motion feedback##taa", &post.taaMotionFeedback, 0.0f, 0.98f, "%.3f");
        ImGui::SliderFloat("History clip width##taa", &post.taaClipWidth, 0.25f, 4.0f, "%.2f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Standard deviations of the 3x3 neighbourhood the history may sit "
                              "outside before it is pulled back in. Lower rejects more history: "
                              "less ghosting, more residual jitter.");
        ImGui::SliderFloat("Sharpen##taa", &post.taaSharpness, 0.0f, 1.0f, "%.2f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Unsharp on the resolved luminance. Accumulating sub-pixel samples "
                              "is softer than the raw image by construction; this puts the high "
                              "frequency back. Past ~0.5 it rings on high-contrast edges.");
        ImGui::TextDisabled("HDR resolve before Bloom and Tone map; history resets per view.");
    }
    if (ImGui::CollapsingHeader("FXAA", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::SliderFloat("Subpixel", &post.fxaaSubpixel, 0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat("Edge threshold", &post.fxaaEdgeThreshold, 0.01f, 0.5f, "%.3f");
        ImGui::SliderFloat("Minimum threshold", &post.fxaaEdgeThresholdMin, 0.0f, 0.2f, "%.3f");
    }
    // Faithful port of the reference's "Volumetric light" panel (same
    // sections, order, sliders and ranges). One forced deviation: the
    // reference disables the sun row while the CSM panel's own sun master
    // switch is off (BeginDisabled(!pp.sunMasterEnabled)) - this panel has no
    // access to that state, only to the VolumetricPass itself, so the row
    // stays enabled here and the demo is responsible for not calling
    // setSunEnabled while its own sun is off.
    if (volumetric && ImGui::CollapsingHeader("Volumetric light", ImGuiTreeNodeFlags_DefaultOpen))
    {
        ImGui::Checkbox("Sol (directional)", &volumetric->sunEnabled);
        ImGui::SliderFloat("Densidade sol", &volumetric->sunDensity, 0.0f, 0.2f);

        ImGui::Separator();
        ImGui::Checkbox("Spot lights", &volumetric->spotEnabled);
        ImGui::SliderFloat("Densidade spots", &volumetric->spotDensity, 0.0f, 0.3f);
        ImGui::SliderFloat("Forca spots", &volumetric->spotStrength, 0.0f, 4.0f);

        ImGui::Separator();
        ImGui::Checkbox("Point lights (proxy rasterizado)", &volumetric->pointEnabled);
        const char* proxies[] = {"Esfera (menos pixels)", "Cubo (menos triangulos)"};
        int proxy = volumetric->pointProxyIsCube ? 1 : 0;
        if (ImGui::Combo("Proxy", &proxy, proxies, 2))
            volumetric->pointProxyIsCube = proxy != 0;
        ImGui::SliderFloat("Densidade points", &volumetric->pointDensity, 0.0f, 0.3f);
        ImGui::SliderFloat("Forca points", &volumetric->pointStrength, 0.0f, 4.0f);

        ImGui::Separator();
        ImGui::Checkbox("Rect lights (luz de area)", &volumetric->rectEnabled);
        ImGui::SliderFloat("Densidade rect", &volumetric->rectDensity, 0.0f, 0.3f);
        ImGui::SliderFloat("Forca rect", &volumetric->rectStrength, 0.0f, 4.0f);

        ImGui::Separator();
        ImGui::Checkbox("Blur##vol", &volumetric->blurEnabled);
        ImGui::SliderFloat("Scattering (g)", &volumetric->scattering, -0.9f, 0.95f);
        ImGui::SliderFloat("Forca##vol", &volumetric->strength, 0.0f, 4.0f);
        int samples = static_cast<int>(volumetric->samples);
        if (ImGui::SliderInt("Passos", &samples, 4, 64))
            volumetric->samples = static_cast<u32>(samples);
        ImGui::SliderFloat("Distancia maxima", &volumetric->maxDistance, 100.0f, 6000.0f);
        ImGui::Checkbox("Ver fallback das cascatas", &volumetric->debugFallback);
    }
}

void ImGuiLayer::drawSky(SkySettings& sky)
{
    if (!ImGui::Begin("Sky"))
    {
        ImGui::End();
        return;
    }
    drawSkyContents(sky);
    ImGui::End();
}

void ImGuiLayer::drawSkyContents(SkySettings& sky)
{
    ImGui::Checkbox("Enabled", &sky.enabled);
    int mode = static_cast<int>(sky.mode);
    const char* modes[] = {"Gradient", "Rayleigh + Mie", "Cubemap"};
    if (ImGui::Combo("Model", &mode, modes, 3))
        sky.mode = static_cast<SkyMode>(mode);
    if (sky.mode == SkyMode::Cubemap)
    {
        // Scanned once and kept: listing walks the filesystem, and this runs
        // every frame the panel is open.
        static std::vector<std::string> available;
        static bool scanned = false;
        if (!scanned)
        {
            available = Assets().listCubemaps(sky.cubemapDirectory);
            scanned = true;
        }

        if (available.empty())
        {
            ImGui::TextDisabled("No cubemaps found under '%s'", sky.cubemapDirectory.c_str());
        }
        else
        {
            int selected = -1;
            for (usize i = 0; i < available.size(); ++i)
                if (available[i] == sky.cubemapName)
                    selected = static_cast<int>(i);

            // The stem is what tells the skies apart; the shared directory
            // in front of it is noise in a list this narrow.
            std::vector<const char*> labels;
            labels.reserve(available.size());
            for (const std::string& name : available)
            {
                const usize slash = name.find_last_of('/');
                labels.push_back(name.c_str() + (slash == std::string::npos ? 0 : slash + 1));
            }

            if (ImGui::Combo("Cubemap", &selected, labels.data(),
                             static_cast<int>(labels.size())) &&
                selected >= 0)
            {
                loadSkyCubemap(sky, available[static_cast<usize>(selected)]);
            }
        }
        if (sky.cubemapName.empty())
            ImGui::TextDisabled("Nothing loaded - showing the gradient sky instead");
    }
    ImGui::Separator();
    ImGui::Checkbox("Sky drives the sun light", &sky.sunFromSky);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("On: the directional light is aimed from the azimuth and elevation "
                          "below, so the lighting always matches the sun drawn in the sky. Off: "
                          "the light is aimed by hand (or by its gizmo) and the sky follows it.");
    ImGui::Checkbox("Time controls sun", &sky.automaticSun);
    if (sky.automaticSun)
    {
        ImGui::SliderFloat("Time", &sky.timeOfDay, 0.0f, 24.0f, "%05.2f h");
        if (ImGui::Button("Dawn"))
            sky.timeOfDay = 6.5f;
        ImGui::SameLine();
        if (ImGui::Button("Noon"))
            sky.timeOfDay = 12.0f;
        ImGui::SameLine();
        if (ImGui::Button("Sunset"))
            sky.timeOfDay = 18.0f;
        ImGui::SameLine();
        if (ImGui::Button("Night"))
            sky.timeOfDay = 23.0f;
        ImGui::SliderFloat("North offset", &sky.northOffset, -180.0f, 180.0f, "%.0f deg");
        ImGui::SliderFloat("Maximum elevation", &sky.maximumElevation, 10.0f, 89.0f, "%.0f deg");
    }
    else
    {
        ImGui::SliderFloat("Azimuth", &sky.sunAzimuth, 0.0f, 360.0f, "%.1f deg");
        ImGui::SliderFloat("Elevation", &sky.sunElevation, -89.0f, 89.0f, "%.1f deg");
    }
    ImGui::TextDisabled("Az %.1f  El %.1f", sky.sunAzimuth, sky.sunElevation);
    ImGui::ColorButton("Sun transmittance", ImVec4(sky.sunTransmittance.r, sky.sunTransmittance.g,
                                                   sky.sunTransmittance.b, 1.0f));
    ImGui::SameLine();
    ImGui::TextDisabled("atmospheric sun color");
    ImGui::ColorEdit3("Ambient", &sky.ambient.x);
    ImGui::SliderFloat("Ambient strength", &sky.ambientStrength, 0.0f, 4.0f, "%.2f");
    if (sky.mode == SkyMode::Gradient)
        ImGui::SliderFloat("Intensity", &sky.intensity, 0.0f, 3.0f, "%.2f");
    else if (sky.mode == SkyMode::Atmosphere)
    {
        ImGui::SliderFloat("Sun intensity", &sky.sunIntensity, 0.0f, 60.0f, "%.1f");
        ImGui::SliderFloat("Exposure", &sky.atmosphereExposure, 0.0f, 4.0f, "%.2f");
        ImGui::SliderFloat("Rayleigh", &sky.rayleigh, 0.0f, 4.0f, "%.2f");
        ImGui::SliderFloat("Mie", &sky.mie, 0.0f, 4.0f, "%.2f");
        ImGui::SliderFloat("Mie g", &sky.mieG, 0.0f, 0.95f, "%.2f");
        int viewSteps = static_cast<int>(sky.viewSteps);
        int lightSteps = static_cast<int>(sky.lightSteps);
        if (ImGui::SliderInt("View steps", &viewSteps, 4, 48))
            sky.viewSteps = static_cast<u32>(viewSteps);
        if (ImGui::SliderInt("Light steps", &lightSteps, 2, 16))
            sky.lightSteps = static_cast<u32>(lightSteps);
        ImGui::TextDisabled("Cost per sky pixel: %u x %u", sky.viewSteps, sky.lightSteps);
    }
    ImGui::Separator();
    if (ImGui::CollapsingHeader("Clouds"))
    {
        ImGui::Checkbox("Enabled##clouds", &sky.cloudsEnabled);
        ImGui::SliderFloat("Height", &sky.cloudHeight, 200.0f, 8000.0f, "%.0f");
        ImGui::SliderFloat("Scale", &sky.cloudScale, 0.0001f, 0.01f, "%.4f");
        ImGui::SliderFloat("Coverage", &sky.cloudCoverage, 0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat("Density", &sky.cloudDensity, 0.01f, 1.0f, "%.2f");
        ImGui::SliderFloat("Speed", &sky.cloudSpeed, 0.0f, 20.0f, "%.2f");
        ImGui::SliderFloat2("Direction", &sky.cloudDirection.x, -1.0f, 1.0f, "%.2f");
        ImGui::ColorEdit3("Color##clouds", &sky.cloudColor.x);
    }
}

void ImGuiLayer::flip()
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

} // namespace Render
} // namespace Radion
