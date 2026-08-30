#include "PCH.h"

#include "panels/DebugPanel.h"

#include "AssetManager.h"
#include "EditorApplication.h"
#include "Engine.h"
#include "GameObject.h"
#include "Hash.h"
#include "Lighting.h"
#include "Material.h"
#include "MeshRenderer.h"
#include "PostProcess.h"
#include "ReflectionProbe.h"
#include "Scene.h"
#include "Shadows.h"

#include <imgui.h>

namespace Radion
{

DebugPanel::DebugPanel(EditorApplication& app) : EditorPanel("Debug", app)
{
    mEnabled = app.settings().debugPreviewEnabled;
    mView = glm::clamp(app.settings().debugPreviewView, 0, 6);
}

DebugPanel::~DebugPanel()
{
    mPreview.destroy();
    for (OffscreenTarget& target : mCascadePreviews)
        target.destroy();
    for (OffscreenTarget& target : mProbePreviews)
        target.destroy();
}

void DebugPanel::drawSpatialDebugSection()
{
    Scene& scene = app().scene();

    if (!ImGui::CollapsingHeader("Spatial Debug", ImGuiTreeNodeFlags_DefaultOpen))
        return;

    bool dynamicCulling = scene.dynamicCullingEnabled();
    bool occlusion = scene.occlusionQueryEnabled();

    bool showDynamicIndex = app().showDynamicIndexDebug();
    if (ImGui::Checkbox("Draw dynamic octree", &showDynamicIndex))
        app().setShowDynamicIndexDebug(showDynamicIndex);
    if (showDynamicIndex && !dynamicCulling)
        ImGui::TextDisabled("  dynamic octree culling is off - the tree is never built, nothing "
                            "to draw");
    else if (showDynamicIndex)
    {
        const Scene::DynamicIndexStats stats = scene.dynamicIndexStats();
        ImGui::TextDisabled("  nodes %u, entries %u, depth %u, visited %u, accepted %u",
                            stats.nodeCount, stats.entryCount, stats.depth, stats.nodesVisited,
                            stats.entriesAccepted);
    }

    bool showOcclusion = app().showOcclusionDebug();
    if (ImGui::Checkbox("Draw occlusion boxes", &showOcclusion))
        app().setShowOcclusionDebug(showOcclusion);
    if (showOcclusion && !occlusion)
        ImGui::TextDisabled("  occlusion queries are off - boxes default to visible (green)");

    bool showSubmeshBounds = app().showSubmeshBounds();
    if (ImGui::Checkbox("Draw submesh bounds", &showSubmeshBounds))
        app().setShowSubmeshBounds(showSubmeshBounds);
    drawSubmeshBoundsReadout();

    ImGui::Checkbox("Surface probe", &app().surfaceProbe());
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("While on, clicking the viewport reports the surface under the "
                          "cursor: submesh, material, shadow flags and the cascade that "
                          "point samples.");
    if (app().surfaceProbe() || app().pickedSurface().valid)
        drawSurfaceProbeReadout();

    ImGui::Separator();
}

void DebugPanel::drawSurfaceProbeReadout()
{
    const EditorApplication::PickedSurface& probe = app().pickedSurface();
    if (!probe.valid)
    {
        ImGui::TextDisabled("  click a surface in the viewport");
        return;
    }

    GameObject* object = app().scene().findGameObject(probe.object);
    ImGui::TextDisabled("  object: %s", object ? object->name().c_str() : "(gone)");
    ImGui::TextDisabled("  position (%.2f, %.2f, %.2f)  normal (%.2f, %.2f, %.2f)",
                        probe.position.x, probe.position.y, probe.position.z, probe.normal.x,
                        probe.normal.y, probe.normal.z);
    ImGui::TextDisabled("  submesh %d, material slot %d", probe.submesh, probe.materialSlot);

    MeshRenderer* renderer = object ? object->getComponent<MeshRenderer>() : nullptr;
    const Mesh* mesh = renderer ? Assets().getMesh(renderer->mesh()) : nullptr;
    const Material* material = nullptr;
    if (renderer && probe.materialSlot >= 0)
    {
        if (const Material* overrides = renderer->materialOverrides();
            overrides && static_cast<u32>(probe.materialSlot) < renderer->materialOverrideCount())
            material = &overrides[probe.materialSlot];
        else if (mesh && static_cast<usize>(probe.materialSlot) < mesh->materials.size())
            material = &mesh->materials[probe.materialSlot];
    }
    if (material)
    {
        ImGui::TextDisabled("  material '%s'%s", material->name.c_str(),
                            material->pipeline.valid() ? "" : "  [NO PIPELINE - never drawn]");
        const bool casts = (material->flags & MaterialCastShadow) != 0;
        const bool receives = (material->flags & MaterialReceiveShadow) != 0;
        ImGui::TextColored(casts ? ImVec4(0.5f, 0.9f, 0.5f, 1.0f) : ImVec4(0.9f, 0.45f, 0.35f, 1.0f),
                           "  cast shadow: %s", casts ? "yes" : "NO");
        ImGui::TextColored(receives ? ImVec4(0.5f, 0.9f, 0.5f, 1.0f)
                                    : ImVec4(0.9f, 0.45f, 0.35f, 1.0f),
                           "  receive shadow: %s", receives ? "yes" : "NO");
    }
    else
        ImGui::TextDisabled("  material: (none resolved)");

    CascadeShadowSettings* shadows = app().engine().cascadeSettings();
    const u32 count = shadows ? glm::clamp(shadows->count, 1u, MaxShadowCascades) : 0;
    if (count > 0)
    {
        u32 cascade = count - 1;
        for (u32 i = 0; i < count; ++i)
            if (probe.viewDepth < app().engine().cascadeSplit(i))
            {
                cascade = i;
                break;
            }
        ImGui::TextDisabled("  view depth %.2f -> cascade %u (splits %.1f / %.1f / %.1f / %.1f)",
                            probe.viewDepth, cascade, app().engine().cascadeSplit(0),
                            app().engine().cascadeSplit(1), app().engine().cascadeSplit(2),
                            app().engine().cascadeSplit(3));
    }
}

// Turns "why is nothing being culled" into a number. Each submesh's box is
// measured against the whole model's box by diagonal length rather than by
// volume - a floor slab spanning the building is flat, so its volume is near
// zero while it is still entirely un-cullable, and only the diagonal catches
// that. Anything near 1.0 covers the model and will intersect every frustum
// the model itself does.
void DebugPanel::drawSubmeshBoundsReadout()
{
    GameObject* selected = app().selection().resolve(app().scene());
    MeshRenderer* renderer = selected ? selected->getComponent<MeshRenderer>() : nullptr;
    const Mesh* mesh = renderer ? Assets().getMesh(renderer->mesh()) : nullptr;
    if (!mesh || mesh->submeshes.empty())
    {
        ImGui::TextDisabled("  select an object with a mesh to measure its submeshes");
        return;
    }

    const f32 meshRadius = mesh->bounds.radius();
    if (meshRadius <= 0.0001f)
        return;

    u32 spanning = 0;
    f32 total = 0.0f;
    for (const SubMesh& submesh : mesh->submeshes)
    {
        const f32 ratio = submesh.bounds.radius() / meshRadius;
        total += ratio;
        if (ratio > 0.5f)
            ++spanning;
    }

    const usize count = mesh->submeshes.size();
    const f32 average = total / static_cast<f32>(count);
    const ImVec4 color = spanning * 2 > count ? ImVec4(0.9f, 0.45f, 0.35f, 1.0f)
                                              : ImVec4(0.5f, 0.9f, 0.5f, 1.0f);
    ImGui::TextColored(color, "  %u of %zu submeshes span >50%% of the model (avg %.2f)", spanning,
                       count, static_cast<double>(average));
    if (spanning * 2 > count)
        ImGui::TextDisabled("  grouped by material, not by region - every box contains the\n"
                            "  camera, so frustum culling can never reject them");
}

void DebugPanel::drawPhysicsDebugSection()
{
    if (!ImGui::CollapsingHeader("Physics Debug", ImGuiTreeNodeFlags_DefaultOpen))
        return;

    Scene& scene = app().scene();
    Engine& engine = app().engine();

    ImGui::Checkbox("Draw body shapes", &engine.debugShowPhysicsShapes);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Every RigidBody's collision shape, coloured by body type and shape "
                          "kind, dimmed to a third of its brightness while asleep.");

    ImGui::Checkbox("Draw contacts", &engine.debugShowPhysicsContacts);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Every active contact point and its normal, the line length scaled by "
                          "the impulse it is carrying.");
    if (engine.debugShowPhysicsContacts && scene.runningInEditor())
        ImGui::TextDisabled("  physics only simulates in Play - nothing to draw yet");

    ImGui::Checkbox("Draw joint direction", &engine.debugShowPhysicsJoints);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Every Joint's anchor pair as a yellow line, and its free axis - "
                          "hinge, slider, piston, universal, wheel - as a cyan segment from the "
                          "first anchor.");
    if (engine.debugShowPhysicsJoints && scene.jointCount() == 0)
        ImGui::TextDisabled("  no joints in this scene - Joint is not a scene component yet, "
                            "only reachable from code");

    ImGui::Separator();
}

void DebugPanel::drawAIDebugSection()
{
    if (!ImGui::CollapsingHeader("AI Debug", ImGuiTreeNodeFlags_DefaultOpen))
        return;

    Scene& scene = app().scene();
    Engine& engine = app().engine();

    ImGui::Checkbox("Draw obstacles", &engine.debugShowAIObstacles);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Every Radion::Obstacle's shape, with an arrow on its solid side "
                          "(seenFrom) - two arrows for Both. The selected object's own Obstacle "
                          "always draws, flag or not.");
    if (engine.debugShowAIObstacles && scene.obstacleCount() == 0)
        ImGui::TextDisabled("  no Obstacle components in this scene");

    ImGui::Separator();
}

void DebugPanel::onImGui()
{
    drawSpatialDebugSection();
    drawPhysicsDebugSection();
    drawAIDebugSection();

    if (!ImGui::CollapsingHeader("GPU Resource Inspector", ImGuiTreeNodeFlags_DefaultOpen))
        return;

    ImGui::Checkbox("Enable resource preview", &mEnabled);
    app().settings().debugPreviewEnabled = mEnabled;
    if (!mEnabled)
    {
        ImGui::TextDisabled("Debug preview is disabled.");
        return;
    }

    static const char* views[] = {"Scene color (HDR)", "Scene depth", "SSAO", "Shadow atlas",
                                  "Sun cascade", "Mirror reflection", "Environment probes"};
    ImGui::SetNextItemWidth(180.0f);
    ImGui::Combo("Resource", &mView, views, IM_ARRAYSIZE(views));
    app().settings().debugPreviewView = mView;

    if (mView == 4)
    {
        ImGui::Separator();
        drawCascadePreviews();
        return;
    }
    if (mView == 6)
    {
        ImGui::Separator();
        drawProbePreviews();
        return;
    }

    Engine& engine = app().engine();
    TextureHandle texture;
    bool rawDepth = false;
    bool arrayTexture = false;
    u32 layer = 0;

    if (mView == 0)
        texture = engine.postProcess().sceneColor();
    else if (mView == 1)
    {
        texture = engine.postProcess().sceneDepth();
        rawDepth = true;
    }
    else if (mView == 2)
        texture = engine.postProcess().ssaoTexture();
    else if (mView == 3)
    {
        Lighting* lighting = engine.lighting();
        texture = lighting ? lighting->atlasTexture() : TextureHandle();
        rawDepth = true;
    }
    else
    {
        // Renderer::executeReflection() publishes this each frame it runs a
        // mirror/water pass, under the same name lit.frag's uMirrorReflectionTex
        // resolves - viewing it raw here (before the shader's own UV/bump lookup)
        // tells apart a hole in the captured render from a mapping bug in the
        // sampling.
        texture = AssetManager::getSingleton().resolveRenderTarget(hashName(kReflectionTargetName));
    }

    ImGui::Separator();
    if (!texture.valid())
    {
        ImGui::TextDisabled("This resource has not been rendered yet.");
        return;
    }

    ImVec2 available = ImGui::GetContentRegionAvail();
    const f32 previewSize = glm::max(1.0f, glm::min(available.x, available.y));
    TextureHandle previewTexture = texture;
    if (rawDepth)
    {
        const u32 size = static_cast<u32>(previewSize);
        if (mPreview.width != size || mPreview.height != size)
        {
            mPreview.destroy();
            if (!mPreview.create(size, size, Format::RGBA8, Format::Unknown,
                                 "editor.debug.preview"))
            {
                ImGui::TextDisabled("Could not create debug preview target.");
                return;
            }
        }
        engine.debugDrawTexture(texture, arrayTexture, layer, mPreview.target, size, size);
        previewTexture = mPreview.color;
    }

    const u32 nativeId = engine.getGPU().nativeTextureId(previewTexture);
    ImGui::Image(static_cast<ImTextureID>(static_cast<uintptr_t>(nativeId)),
                 ImVec2(previewSize, previewSize), ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
}

void DebugPanel::drawProbePreviews()
{
    Engine& engine = app().engine();
    const std::vector<ReflectionProbe*>& localProbes = app().scene().reflectionProbes();
    const int probeCount = 1 + static_cast<int>(localProbes.size());
    mProbeIndex = glm::clamp(mProbeIndex, 0, probeCount - 1);

    auto probeAt = [&](int index) -> EnvironmentProbe*
    {
        return index == 0 ? &engine.environmentProbe()
                          : &localProbes[static_cast<usize>(index - 1)]->probe();
    };
    auto probeName = [&](int index) -> std::string
    {
        if (index == 0)
            return "Global";
        ReflectionProbe* component = localProbes[static_cast<usize>(index - 1)];
        GameObject* owner = component ? component->owner() : nullptr;
        return owner ? owner->name() + " (#" + std::to_string(owner->id()) + ")"
                     : "Local probe";
    };

    const std::string selectedName = probeName(mProbeIndex);
    ImGui::SetNextItemWidth(220.0f);
    if (ImGui::BeginCombo("Probe", selectedName.c_str()))
    {
        for (int index = 0; index < probeCount; ++index)
        {
            const std::string name = probeName(index);
            if (ImGui::Selectable(name.c_str(), index == mProbeIndex))
            {
                mProbeIndex = index;
                mProbeMip = 0;
            }
        }
        ImGui::EndCombo();
    }

    EnvironmentProbe* probe = probeAt(mProbeIndex);
    if (!probe || !probe->ready())
    {
        ImGui::TextDisabled("This probe has no cubemap yet.");
        return;
    }

    if (!probe->enabled)
        ImGui::TextDisabled("This probe is disabled in Settings.");

    if (probe->captureCount() == 0)
    {
        ImGui::TextDisabled("This probe has not been captured yet.");
        if (probe->enabled)
        {
            if (ImGui::Button("Capture now"))
                probe->requestCapture();
        }
        else
        {
            ImGui::TextDisabled("Enable it before requesting a capture.");
        }
        return;
    }

    const int maximumMip = glm::max(static_cast<int>(probe->mipCount()) - 1, 0);
    mProbeMip = glm::clamp(mProbeMip, 0, maximumMip);
    ImGui::SetNextItemWidth(220.0f);
    ImGui::SliderInt("Mip", &mProbeMip, 0, maximumMip);
    ImGui::SameLine();
    if (ImGui::Button("Capture now") && probe->enabled)
        probe->requestCapture();

    ImGui::TextDisabled("%ux%u | %u mip(s) | capture #%llu | %.2f ms", probe->resolution(),
                        probe->resolution(), probe->mipCount(),
                        static_cast<unsigned long long>(probe->captureCount()),
                        static_cast<double>(probe->lastCaptureCostMilliseconds()));

    static const char* faceNames[EnvironmentProbe::FaceCount] = {
        "+X Right", "-X Left", "+Y Up", "-Y Down", "+Z Back", "-Z Forward"};

    const ImVec2 available = ImGui::GetContentRegionAvail();
    constexpr int columns = 3;
    constexpr f32 gap = 8.0f;
    const f32 faceSize = glm::clamp((available.x - gap * (columns - 1)) / columns,
                                    64.0f, 256.0f);
    const u32 targetSize = static_cast<u32>(faceSize);
    bool refreshPreviews = mRenderedProbeIndex != mProbeIndex ||
                           mRenderedProbeMip != mProbeMip ||
                           mRenderedProbeCapture != probe->captureCount() ||
                           mRenderedProbeSize != targetSize;

    for (OffscreenTarget& preview : mProbePreviews)
    {
        if (preview.width != targetSize || preview.height != targetSize)
        {
            preview.destroy();
            if (!preview.create(targetSize, targetSize, Format::RGBA8, Format::Unknown,
                                "editor.debug.probe"))
            {
                ImGui::TextDisabled("Could not create face previews.");
                return;
            }
            refreshPreviews = true;
        }
    }

    if (refreshPreviews)
    {
        for (u32 face = 0; face < EnvironmentProbe::FaceCount; ++face)
            engine.debugDrawCubemap(probe->cubemap(), face, static_cast<u32>(mProbeMip),
                                    mProbePreviews[face].target, targetSize, targetSize);
    }

    for (u32 face = 0; face < EnvironmentProbe::FaceCount; ++face)
    {
        ImGui::BeginGroup();
        ImGui::TextUnformatted(faceNames[face]);
        const u32 nativeId = engine.getGPU().nativeTextureId(mProbePreviews[face].color);
        ImGui::Image(static_cast<ImTextureID>(static_cast<uintptr_t>(nativeId)),
                     ImVec2(faceSize, faceSize), ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
        ImGui::EndGroup();
        if (face % columns != columns - 1)
            ImGui::SameLine(0.0f, gap);
    }

    if (refreshPreviews)
    {
        mRenderedProbeIndex = mProbeIndex;
        mRenderedProbeMip = mProbeMip;
        mRenderedProbeCapture = probe->captureCount();
        mRenderedProbeSize = targetSize;
    }
}

// Every cascade laid out in one row instead of one at a time behind a
// slider - comparing texel density between the near and far cascade (the
// whole reason to look at this view at all) needs both on screen together,
// not a flip back and forth trying to remember what the other one looked
// like.
void DebugPanel::drawCascadePreviews()
{
    Engine& engine = app().engine();
    const TextureHandle texture = engine.directionalShadowTexture();
    if (!texture.valid())
    {
        ImGui::TextDisabled("This resource has not been rendered yet.");
        return;
    }

    CascadeShadowSettings* shadows = engine.cascadeSettings();
    const u32 count =
        glm::clamp(shadows ? shadows->count : 1u, 1u, static_cast<u32>(MaxShadowCascades));

    const ImVec2 available = ImGui::GetContentRegionAvail();
    const f32 spacing = ImGui::GetStyle().ItemSpacing.x;
    const f32 cell = glm::max(
        1.0f, glm::min((available.x - spacing * static_cast<f32>(count - 1)) /
                           static_cast<f32>(count),
                       available.y - ImGui::GetTextLineHeightWithSpacing()));
    const u32 size = static_cast<u32>(cell);

    for (u32 i = 0; i < count; ++i)
    {
        OffscreenTarget& preview = mCascadePreviews[i];
        if (preview.width != size || preview.height != size)
        {
            preview.destroy();
            if (!preview.create(size, size, Format::RGBA8, Format::Unknown,
                                "editor.debug.cascade"))
            {
                ImGui::TextDisabled("Could not create debug preview target.");
                return;
            }
        }
        const DirectionalShadowRegion region = directionalShadowRegion(2, count, i);
        engine.debugDrawTexture(texture, false, 0, preview.target, size, size,
                                glm::vec4(static_cast<f32>(region.width) * 0.5f,
                                          static_cast<f32>(region.height) * 0.5f,
                                          static_cast<f32>(region.x) * 0.5f,
                                          static_cast<f32>(region.y) * 0.5f));

        ImGui::BeginGroup();
        ImGui::Text("Cascade %u", i);
        const u32 nativeId = engine.getGPU().nativeTextureId(preview.color);
        ImGui::Image(static_cast<ImTextureID>(static_cast<uintptr_t>(nativeId)),
                     ImVec2(cell, cell), ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
        ImGui::EndGroup();
        if (i + 1 < count)
            ImGui::SameLine();
    }
}

} // namespace Radion
