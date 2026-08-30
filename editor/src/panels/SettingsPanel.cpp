#include "PCH.h"

#include "panels/SettingsPanel.h"

#include "EditorApplication.h"
#include "Engine.h"
#include "LensFlarePass.h"
#include "Light.h"
#include "Lighting.h"
#include "ParticlePass.h"
#include "PostProcess.h"
#include "RenderList.h"
#include "Scene.h"
#include "Shadows.h"
#include "panels/AssetsPanel.h"

#include <IconsMaterialDesignIcons.h>
#include <imgui.h>

namespace Radion
{

SettingsPanel::SettingsPanel(EditorApplication& app) : EditorPanel("Settings", app)
{
}

void SettingsPanel::onImGui()
{
    Engine& engine = app().engine();
    if (ImGui::CollapsingHeader("Editor Preview", ImGuiTreeNodeFlags_DefaultOpen))
    {
        EditorSettings& settings = app().settings();
        if (ImGui::Button("Full"))
        {
            settings.previewShadows = true;
            settings.previewSSAO = true;
            settings.previewVolumetrics = true;
            settings.previewLensFlares = true;
            settings.previewPlanarReflections = true;
            settings.previewNavigationScale = 1.0f;
        }
        ImGui::SameLine();
        if (ImGui::Button("Balanced"))
        {
            settings.previewShadows = true;
            settings.previewSSAO = true;
            settings.previewVolumetrics = false;
            settings.previewLensFlares = false;
            settings.previewPlanarReflections = false;
            settings.previewNavigationScale = 0.5f;
        }
        ImGui::SameLine();
        if (ImGui::Button("Fast"))
        {
            settings.previewShadows = false;
            settings.previewSSAO = false;
            settings.previewVolumetrics = false;
            settings.previewLensFlares = false;
            settings.previewPlanarReflections = false;
            settings.previewNavigationScale = 0.5f;
        }
        ImGui::Checkbox("Preview shadows", &settings.previewShadows);
        ImGui::Checkbox("Preview SSAO", &settings.previewSSAO);
        ImGui::Checkbox("Preview volumetrics", &settings.previewVolumetrics);
        ImGui::Checkbox("Preview lens flares", &settings.previewLensFlares);
        ImGui::Checkbox("Preview planar reflections", &settings.previewPlanarReflections);
        ImGui::Checkbox("Preview occlusion culling", &settings.previewOcclusionCulling);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Hide in the Viewport whatever the GAME camera's occlusion queries\n"
                              "hid - watch the culler work from outside. Needs occlusion queries\n"
                              "enabled on the scene and a Game view rendering.");
        int navigationPercent = static_cast<int>(settings.previewNavigationScale * 100.0f + 0.5f);
        if (ImGui::SliderInt("Navigation resolution", &navigationPercent, 25, 100, "%d%%",
                             ImGuiSliderFlags_AlwaysClamp))
            settings.previewNavigationScale = static_cast<f32>(navigationPercent) * 0.01f;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Internal Viewport resolution while orbiting, panning or looking.");
        ImGui::TextDisabled("Editor-only: scene and Game rendering are unchanged.");
    }

    if (ImGui::CollapsingHeader("Particles"))
    {
        ParticleRenderQueue& particles = ParticleDraws();
        ImGui::TextUnformatted("Global particle texture");
        ImGui::Button(particles.textureFile().empty() ? "Drop texture asset here"
                                                      : particles.textureFile().c_str(),
                      ImVec2(-FLT_MIN, 0.0f));
        if (ImGui::BeginDragDropTarget())
        {
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(kAssetFileDragPayload))
            {
                particles.setTextureFile(
                    std::string(static_cast<const char*>(payload->Data), payload->DataSize));
                app().markDirty();
            }
            ImGui::EndDragDropTarget();
        }
        if (ImGui::Button("Clear particle texture"))
        {
            particles.setTextureFile(std::string());
            app().markDirty();
        }
    }

    if (ImGui::CollapsingHeader("Culling", ImGuiTreeNodeFlags_DefaultOpen))
    {
        Scene& scene = app().scene();
        bool staticCulling = scene.staticCullingEnabled();
        if (ImGui::Checkbox("Static BVH", &staticCulling))
        {
            scene.setStaticCullingEnabled(staticCulling);
            app().markDirty();
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(%zu hit)", scene.lastStaticHitCount());

        bool dynamicCulling = scene.dynamicCullingEnabled();
        if (ImGui::Checkbox("Dynamic Octree", &dynamicCulling))
        {
            scene.setDynamicCullingEnabled(dynamicCulling);
            app().markDirty();
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(%zu hit)", scene.lastDynamicHitCount());

        bool occlusion = scene.occlusionQueryEnabled();
        if (ImGui::Checkbox("Occlusion Query", &occlusion))
        {
            scene.setOcclusionQueryEnabled(occlusion);
            app().markDirty();
        }
        ImGui::SameLine();
        ImGui::TextDisabled("(%zu candidate)", scene.lastOcclusionCandidateCount());
    }

    // Theme moved to Windows > Theme - not a scene/render setting, it has
    // nothing to do with anything else on this panel.
    if (ImGui::CollapsingHeader("Sky"))
    {
        // drawSkyContents()/drawPostProcessContents() below are also each
        // their own standalone ImGui::Begin("Sky")/Begin("Post Process")
        // window elsewhere (Engine::flip()'s built-in overlay) - fine there,
        // since a window name is its own ID scope, but embedded side by side
        // in this one panel their identical "Enabled" checkboxes collide
        // without a push of their own.
        ImGui::PushID("Sky");
        engine.drawSkyContents();
        ImGui::PopID();
    }

    if (ImGui::CollapsingHeader("Environment Probe", ImGuiTreeNodeFlags_DefaultOpen))
    {
        EnvironmentProbe& probe = engine.environmentProbe();
        static const int resolutions[] = {32, 64, 128, 256, 512};
        static const char* resolutionNames[] = {"32", "64", "128", "256", "512"};
        int resolutionIndex = 1;
        for (int index = 0; index < IM_ARRAYSIZE(resolutions); ++index)
            if (static_cast<int>(probe.resolution()) == resolutions[index])
                resolutionIndex = index;
        if (ImGui::Combo("Resolution##probe", &resolutionIndex, resolutionNames,
                         IM_ARRAYSIZE(resolutionNames)))
        {
            if (probe.create(static_cast<u32>(resolutions[resolutionIndex])))
                probe.invalidate();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Cubemap size per face. Capture cost grows with six faces.");

        int mode = 0;
        if (probe.enabled)
        {
            switch (probe.refresh)
            {
            case EnvironmentProbe::Refresh::Automatic:
                mode = 1;
                break;
            case EnvironmentProbe::Refresh::Manual:
                mode = 2;
                break;
            case EnvironmentProbe::Refresh::Timed:
                mode = 3;
                break;
            }
        }
        const char* modes[] = {"Off", "Automatic", "Manual", "Timed"};
        if (ImGui::Combo("Mode", &mode, modes, IM_ARRAYSIZE(modes)))
        {
            probe.enabled = mode != 0;
            if (mode == 1)
            {
                probe.refresh = EnvironmentProbe::Refresh::Automatic;
                probe.invalidate();
            }
            else if (mode == 2)
                probe.refresh = EnvironmentProbe::Refresh::Manual;
            else if (mode == 3)
            {
                probe.refresh = EnvironmentProbe::Refresh::Timed;
                probe.requestCapture();
            }
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Automatic captures once after scene changes. Manual captures only "
                              "on request. Timed captures repeatedly at the interval below.");

        if (probe.enabled && probe.refresh == EnvironmentProbe::Refresh::Timed)
            ImGui::SliderFloat("Interval", &probe.interval, 0.1f, 10.0f, "%.1f s");

        if (probe.enabled && probe.refresh == EnvironmentProbe::Refresh::Manual)
            if (ImGui::Button("Capture now", ImVec2(-FLT_MIN, 0.0f)))
                probe.requestCapture();

        if (!probe.enabled)
            ImGui::TextDisabled("Probe disabled");
        else if (probe.captureCount() == 0)
            ImGui::TextDisabled("Not captured yet");
        else
        {
            const f32 age = glm::max(static_cast<f32>(engine.getWindow().getTime()) -
                                         probe.lastCaptureTimeSeconds(),
                                     0.0f);
            ImGui::TextDisabled("Last capture: %.1f s ago | %.2f ms | #%llu",
                                static_cast<double>(age),
                                static_cast<double>(probe.lastCaptureCostMilliseconds()),
                                static_cast<unsigned long long>(probe.captureCount()));
        }
    }

    if (ImGui::CollapsingHeader("Lens flare"))
    {
        if (LensFlarePass* lensFlare = engine.lensFlare())
        {
            ImGui::Checkbox("Enabled", &lensFlare->enabled);
            if (lensFlare->enabled)
            {
                ImGui::SliderFloat("Occlusion radius", &lensFlare->occlusionRadius, 0.0f, 32.0f);
                ImGui::SliderFloat("Sun distance", &lensFlare->sunDistance, 1000.0f, 200000.0f,
                                   "%.0f");
                ImGui::Checkbox("Debug occlusion", &lensFlare->debugOcclusion);
            }
        }
        else
            ImGui::TextDisabled("Lens flare unavailable");
    }

    if (ImGui::CollapsingHeader("Shadows"))
    {
        CascadeShadowSettings* shadows = engine.cascadeSettings();
        Lighting* lighting = engine.lighting();
        if (!shadows || !lighting)
            ImGui::TextDisabled("Shadow settings unavailable");
        else
        {
            bool sunEnabled = engine.sunShadows();
            if (ImGui::Checkbox("Sun shadows", &sunEnabled))
                engine.setSunShadows(sunEnabled);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Master switch for the directional cascade pass. Off costs "
                                  "nothing: no shadow map, no draws. Every light keeps its own "
                                  "Cast Shadows setting.");
            if (!app().scene().sunLight())
                ImGui::TextDisabled("No directional light in this scene");

            bool pointEnabled = engine.pointShadows();
            if (ImGui::Checkbox("Point shadows", &pointEnabled))
                engine.setPointShadows(pointEnabled);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Master switch for point light shadows. Off costs nothing: no "
                                  "atlas tiles, no draws. Every light keeps its own Cast Shadows "
                                  "setting.");

            bool spotEnabled = engine.spotShadows();
            if (ImGui::Checkbox("Spot shadows", &spotEnabled))
                engine.setSpotShadows(spotEnabled);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Master switch for spot and rectangle light shadows. Off costs "
                                  "nothing: no atlas tiles, no draws. Every light keeps its own "
                                  "Cast Shadows setting.");

            if (!engine.sunShadows())
                ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.4f, 1.0f),
                                   "Sun shadows are off - the cascade pass is not running, so "
                                   "these settings have no visible effect yet.");

            auto applyDirectionalProfile =
                [shadows](u32 count, u32 resolution, f32 distance, u32 quality)
            {
                shadows->enabled = true;
                shadows->count = count;
                shadows->resolution = resolution;
                shadows->distance = distance;
                shadows->quality = quality;
                shadows->splitOffset[0] = 0.1f;
                shadows->splitOffset[1] = 0.2f;
                shadows->splitOffset[2] = 0.5f;
                shadows->bias = 0.1f;
                shadows->normalBias = 2.0f;
                shadows->pancakeSize = 20.0f;
                shadows->blur = 1.0f;
                shadows->fadeStart = 0.8f;
                shadows->opacity = 1.0f;
                shadows->angularDiameter = 0.0f;
                shadows->blend = true;
            };

            ImGui::TextUnformatted("Directional quality");
            // ##shadowQuality: CollapsingHeader does not push an ID scope
            // for its own contents (unlike TreeNode) - "Fast"/"Balanced" here
            // and "Editor Preview"'s own Fast/Balanced buttons above land in
            // the same window-level ID stack and collide the moment both
            // headers are open at once.
            if (ImGui::Button("Fast##shadowQuality"))
            {
                applyDirectionalProfile(2, 1024, 75.0f, 0);
                app().markDirty();
            }
            ImGui::SameLine();
            if (ImGui::Button("Balanced##shadowQuality"))
            {
                applyDirectionalProfile(4, 1024, 100.0f, 2);
                app().markDirty();
            }
            ImGui::SameLine();
            if (ImGui::Button("High"))
            {
                applyDirectionalProfile(4, 2048, 100.0f, 3);
                app().markDirty();
            }
            ImGui::TextDisabled("High: 4 cascades at 2048, Soft Medium filtering");

            if (ImGui::CollapsingHeader("Advanced directional shadows"))
            {
                if (ImGui::TreeNode("Cascade layout"))
                {
                    // Three shadow modes, no others (light_3d.cpp:583, and
                    // renderer_scene_cull.cpp:2167-2177): Orthogonal, 2 splits,
                    // 4 splits. A free 1..4 slider offered a 3 that the
                    // calculator turns back into 2 without saying so.
                    static const u32 kSplitCounts[3] = {1, 2, 4};
                    const char* splitNames[] = {"Orthogonal (1)", "2 Splits", "4 Splits"};
                    int splitIndex = shadows->count >= 4 ? 2 : (shadows->count <= 1 ? 0 : 1);
                    if (ImGui::Combo("Shadow mode", &splitIndex, splitNames, 3))
                        shadows->count = kSplitCounts[splitIndex];

                    static const u32 resolutionValues[] = {512, 1024, 2048, 4096};
                    const char* resolutionNames[] = {"512", "1024", "2048", "4096"};
                    int resolutionIndex = 0;
                    for (int i = 0; i < 4; ++i)
                    {
                        if (shadows->resolution == resolutionValues[i])
                            resolutionIndex = i;
                    }
                    if (ImGui::Combo("Resolution##cascades", &resolutionIndex, resolutionNames, 4))
                        shadows->resolution = resolutionValues[resolutionIndex];

                    // Only the offsets this mode actually reads are shown, as in
                    // DirectionalLight3D::_validate_property (light_3d.cpp:559-
                    // 570): one split reads none of them, two reads only the
                    // first, and distances[splits] = max_distance overwrites the
                    // rest. Showing a slider that changes nothing is what makes
                    // these hard to set.
                    static const char* kSplitLabels[3] = {"Split 1", "Split 2", "Split 3"};
                    const u32 usedSplits =
                        shadows->count >= 4 ? 3u : (shadows->count <= 1 ? 0u : 1u);
                    for (u32 i = 0; i < usedSplits; ++i)
                    {
                        ImGui::SliderFloat(kSplitLabels[i], &shadows->splitOffset[i], 0.0f, 1.0f,
                                           "%.3f");
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Fraction of the shadow distance where cascade %u "
                                              "ends.", i);
                    }
                    if (const Camera* cam = app().scene().activeCamera())
                    {
                        const f32 farPlane = glm::max(cam->farPlane(), 1.0f);
                        ImGui::SliderFloat(
                            "Shadow distance", &shadows->distance, 1.0f, farPlane, "%.0f",
                            ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_NoRoundToFormat |
                                ImGuiSliderFlags_AlwaysClamp);
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Max follows the active camera's far plane (%.0f).",
                                              farPlane);
                    }
                    else
                    {
                        ImGui::SliderFloat(
                            "Shadow distance", &shadows->distance, 1.0f, 1000.0f, "%.0f",
                            ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_NoRoundToFormat);
                    }
                    // Hidden with a single split, for the same reason and in the
                    // same place as split 1 (light_3d.cpp:561-564): there is no
                    // second cascade to blend into.
                    if (shadows->count > 1)
                        ImGui::Checkbox("Blend cascades", &shadows->blend);

                    if (ImGui::Button("Fit to Scene"))
                        app().fitShadowsToScene();
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Solves distance/extrusion so the near cascade reaches about 20 "
                            "texels per unit, using the scene camera's field of view, the sun's "
                            "angle and the settings below. Everything else is left untouched.");

                    ImGui::Separator();
                    ImGui::TextDisabled("Cascade coverage");
                    for (u32 cascade = 0; cascade < shadows->count; ++cascade)
                    {
                        const f32 extent = engine.cascadeHalfExtent(cascade) * 2.0f;
                        const f32 texelsPerUnit =
                            extent > 0.001f ? static_cast<f32>(shadows->resolution) / extent : 0.0f;
                        const ImVec4 color = texelsPerUnit > 20.0f ? ImVec4(0.5f, 0.9f, 0.5f, 1.0f)
                                             : texelsPerUnit > 6.0f
                                                 ? ImVec4(0.9f, 0.8f, 0.4f, 1.0f)
                                                 : ImVec4(0.9f, 0.45f, 0.35f, 1.0f);
                        // The band each cascade actually covers, so the three
                        // offsets above read as distances rather than as
                        // fractions to guess at.
                        const f32 bandNear = cascade == 0 ? 0.0f : engine.cascadeSplit(cascade - 1);
                        const f32 bandFar = engine.cascadeSplit(cascade);
                        ImGui::TextColored(color, "%u: %.1f-%.1f m, %.1f units, %.1f texels/unit",
                                           cascade, static_cast<double>(bandNear),
                                           static_cast<double>(bandFar),
                                           static_cast<double>(extent),
                                           static_cast<double>(texelsPerUnit));
                    }
                    ImGui::TreePop();
                }

                if (ImGui::TreeNode("Sampling and bias"))
                {
                    static const char* qualityNames[] = {"Hard",        "Soft Very Low",
                                                         "Soft Low",    "Soft Medium",
                                                         "Soft High",   "Soft Ultra"};
                    int qualityIndex = static_cast<int>(glm::min(shadows->quality, 5u));
                    if (ImGui::Combo("Quality", &qualityIndex, qualityNames, 6))
                        shadows->quality = static_cast<u32>(qualityIndex);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Filter sample count: 0/4/8 up to 32 rotated taps. "
                                          "Also sets the blocker search size when Sun size is "
                                          "above zero.");

                    ImGui::SliderFloat("Sun size", &shadows->angularDiameter, 0.0f, 5.0f,
                                       "%.2f deg");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Angular diameter of the sun. Above zero the penumbra "
                                          "widens with distance from the caster; zero keeps "
                                          "plain PCF.");
                    ImGui::SliderFloat("Blur", &shadows->blur, 0.0f, 4.0f);
                    ImGui::SliderFloat("Bias", &shadows->bias, 0.0f, 1.0f, "%.3f");
                    ImGui::SliderFloat("Normal bias", &shadows->normalBias, 0.0f, 8.0f);
                    ImGui::SliderFloat("Pancake size", &shadows->pancakeSize, 0.0f, 100.0f);
                    ImGui::SliderFloat("Fade start", &shadows->fadeStart, 0.0f, 1.0f);
                    ImGui::SliderFloat("Opacity", &shadows->opacity, 0.0f, 1.0f);
                    ImGui::TreePop();
                }
            }

            if (ImGui::CollapsingHeader("Local-light shadow atlas"))
            {
                ShadowAtlasSettings& atlas = lighting->atlasSettings();
                ImGui::Text("Tiles used: %u", lighting->tilesUsed());
                ImGui::Text("Lights without tile: %u", lighting->lightsWithoutTile());
                ImGui::Text("Atlas scale: %.2f", static_cast<double>(lighting->atlasScale()));

                static const u32 atlasSizes[] = {1024, 2048, 4096, 8192};
                const char* atlasSizeNames[] = {"1024", "2048", "4096", "8192"};
                int atlasSizeIndex = 2;
                for (int i = 0; i < 4; ++i)
                    if (atlas.size == atlasSizes[i])
                        atlasSizeIndex = i;
                if (ImGui::Combo("Atlas size", &atlasSizeIndex, atlasSizeNames, 4))
                    atlas.size = atlasSizes[atlasSizeIndex];

                static const u32 tileValues[] = {256, 512, 1024, 2048};
                const char* tileNames[] = {"256", "512", "1024", "2048"};
                int maximumIndex = 0;
                for (int i = 0; i < 4; ++i)
                {
                    if (atlas.maximumTileSize == tileValues[i])
                        maximumIndex = i;
                }
                if (ImGui::Combo("Maximum tile", &maximumIndex, tileNames, 4))
                    atlas.maximumTileSize = tileValues[maximumIndex];

                static const u32 minimumValues[] = {32, 64, 128, 256};
                const char* minimumNames[] = {"32", "64", "128", "256"};
                int minimumIndex = 0;
                for (int i = 0; i < 4; ++i)
                {
                    if (atlas.minimumTileSize == minimumValues[i])
                        minimumIndex = i;
                }
                if (ImGui::Combo("Minimum tile", &minimumIndex, minimumNames, 4))
                    atlas.minimumTileSize = minimumValues[minimumIndex];

                ImGui::SliderFloat("Volumetric priority", &atlas.volumetricPriority, 1.0f, 16.0f);
                ImGui::SliderFloat("Point priority", &atlas.pointPriority, 1.0f, 8.0f);
                ImGui::SliderFloat("Point bias", &atlas.pointBias, 0.0f, 0.02f, "%.4f");
                ImGui::SliderFloat("Atlas slope bias", &atlas.biasSlope, 0.0f, 8.0f);
                ImGui::SliderFloat("Atlas constant bias", &atlas.biasConstant, 0.0f, 16.0f);
            }

            if (ImGui::CollapsingHeader("Debug and lighting"))
            {
                static const char* shadingModes[] = {
                    "Lit",           "Cascade index", "Shadow factor",      "Normals",
                    "Ambient+shadow", "Roughness",    "Albedo",             "Ambient occlusion",
                    "Env reflection", "Fast shaded"};
                int shadingMode = static_cast<int>(lighting->debugMode);
                if (shadingMode >= IM_ARRAYSIZE(shadingModes))
                    shadingMode = 0;
                if (ImGui::Combo("Shading view", &shadingMode, shadingModes,
                                 IM_ARRAYSIZE(shadingModes)))
                    lighting->debugMode = static_cast<LightingDebugMode>(shadingMode);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Cascade index tints each pixel by the cascade it samples; "
                                      "Shadow factor shows the raw sun shadow term in greyscale. "
                                      "Fast shaded drops out before every texture fetch, shadow "
                                      "lookup and light loop - shapes only, for a scene too heavy "
                                      "to navigate lit.");
                ImGui::Checkbox("Show cascades", &engine.debugShowShadowCascades);
                ImGui::Checkbox("Show atlas", &engine.debugShowShadowAtlas);
                ImGui::Checkbox("Tiled light culling", &lighting->tiled);
                ImGui::SameLine();
                ImGui::Checkbox("Tile heatmap", &lighting->debugTiles);
                ImGui::Checkbox("2.5D culling", &lighting->use25D);
                ImGui::Checkbox("Decals", &lighting->decalsEnabled);

                const RenderListStats* stats = engine.shadowListStats();
                if (stats)
                    ImGui::TextDisabled("Last cascade: %u submitted, %u mesh culled, %u packets",
                                        stats->submitted, stats->culledMeshes, stats->packets);
            }
        }
    }

    if (ImGui::CollapsingHeader("Post Process"))
    {
        ImGui::PushID("PostProcess");
        engine.drawPostProcessContents();
        ImGui::PopID();
    }

    if (ImGui::CollapsingHeader("Search Paths"))
    {
        ImGui::TextWrapped("Extra folders to look in when a mesh's own texture references do "
                           "not resolve under this project's Assets/ - useful for something "
                           "imported straight from wherever it was exported to, textures and "
                           "all, without copying them in first.");
        if (!app().hasProject())
            ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.4f, 1.0f),
                               "No project is open - paths added below work for this session "
                               "only and are lost on restart. Create or open a project to keep "
                               "them.");
        const std::vector<std::string>& paths = app().projectSearchPaths();
        int removeIndex = -1;
        for (int i = 0; i < static_cast<int>(paths.size()); ++i)
        {
            ImGui::PushID(i);
            if (ImGui::Button(ICON_MDI_CLOSE))
                removeIndex = i;
            ImGui::SameLine();
            ImGui::TextUnformatted(paths[static_cast<usize>(i)].c_str());
            ImGui::PopID();
        }
        if (removeIndex >= 0)
            app().removeProjectSearchPath(static_cast<usize>(removeIndex));
        if (ImGui::Button(ICON_MDI_FOLDER_PLUS " Add Folder..."))
            app().browseAddSearchPath();
    }

    if (ImGui::Button("Reset to Defaults"))
    {
        engine.sky() = SkySettings();
        engine.postProcess().exposure = 1.0f;
        engine.postProcess().toneMap = ToneMapMode::ACES;
        engine.postProcess().bloomThreshold = 1.0f;
        engine.postProcess().bloomSoftKnee = 0.5f;
        engine.postProcess().bloomStrength = 0.35f;
        engine.setRenderResolution(RenderResolution());
        if (CascadeShadowSettings* shadows = engine.cascadeSettings())
            *shadows = CascadeShadowSettings();
        app().markDirty();
    }
}

} // namespace Radion
