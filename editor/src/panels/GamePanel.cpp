#include "PCH.h"

#include "panels/GamePanel.h"

#include "Camera.h"
#include "EditorApplication.h"
#include "GameObject.h"
#include "Scene.h"

#include <imgui.h>
namespace Radion
{
GamePanel::GamePanel(EditorApplication& app) : EditorPanel("Game", app)
{
    mResolutionPreset = Math::clamp(app.settings().gameResolutionPreset, 0, 6);
}
void GamePanel::onImGui()
{
    struct ResolutionPreset
    {
        const char* label;
        u32 width;
        u32 height;
    };
    static constexpr ResolutionPreset presets[] = {
        {"Panel", 0, 0},           {"2560 x 1440", 2560, 1440}, {"1920 x 1080", 1920, 1080},
        {"1600 x 900", 1600, 900}, {"1280 x 720", 1280, 720},   {"960 x 540", 960, 540},
        {"640 x 360", 640, 360}};

    Scene& scene = app().scene();
    Camera* activeCamera = scene.activeCamera();
    std::string activeCameraName = activeCamera && activeCamera->owner()
                                       ? activeCamera->owner()->name()
                                       : std::string("No active camera");
    ImGui::SetNextItemWidth(180.0f);
    if (ImGui::BeginCombo("Camera", activeCameraName.c_str()))
    {
        for (Camera* camera : scene.cameras())
        {
            if (!camera || !camera->owner())
                continue;
            GameObject* owner = camera->owner();
            ImGui::PushID(owner);
            if (ImGui::Selectable(owner->name().c_str(), camera == activeCamera))
            {
                app().recordUndo();
                scene.setActiveCamera(camera);
                activeCamera = camera;
                activeCameraName = owner->name();
                app().markDirty();
            }
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("The active scene camera saved as scene.activeCamera.");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(140.0f);
    if (ImGui::BeginCombo("Resolution", presets[mResolutionPreset].label))
    {
        for (int i = 0; i < static_cast<int>(IM_ARRAYSIZE(presets)); ++i)
        {
            if (ImGui::Selectable(presets[i].label, i == mResolutionPreset))
                mResolutionPreset = i;
        }
        ImGui::EndCombo();
    }
    app().settings().gameResolutionPreset = mResolutionPreset;
    // Only the live view submits the scene (EditorApplication::ViewMode) -
    // while Scene is live this panel keeps showing the last frame it drew
    // rather than paying for a second full render of everything.
    const bool live = app().viewMode() == EditorApplication::ViewMode::Game;
    if (!live)
    {
        ImGui::SameLine();
        if (ImGui::Button("Go Live"))
            app().setViewMode(EditorApplication::ViewMode::Game);
    }
    else
    {
        ImGui::SameLine();
        ImGui::TextDisabled(app().playing() ? "LIVE / PLAY" : "LIVE / EDIT PREVIEW");
    }
    ImGui::Separator();

    ImVec2 available = ImGui::GetContentRegionAvail();
    available.x = Math::max(available.x, 1.0f);
    available.y = Math::max(available.y, 1.0f);
    const u32 width = presets[mResolutionPreset].width != 0 ? presets[mResolutionPreset].width
                                                            : static_cast<u32>(available.x);
    const u32 height = presets[mResolutionPreset].height != 0 ? presets[mResolutionPreset].height
                                                              : static_cast<u32>(available.y);
    RenderTextureSettings settings;
    settings.outputIndex = 1;
    if (live && scene.activeCamera() &&
        app().engine().renderToTexture(scene, width, height, mOutput, settings))
    {
        app().notifySceneRendered();
        app().publishGameTexture(mOutput.color);
    }

    if (mOutput.valid())
    {
        // The stale frame keeps the size it was rendered at, which is not
        // necessarily the panel's current one - scale from the texture's own
        // dimensions so a resize while paused does not stretch it.
        const f32 textureWidth = static_cast<f32>(mOutput.width);
        const f32 textureHeight = static_cast<f32>(mOutput.height);
        const f32 scale = Math::min(available.x / textureWidth, available.y / textureHeight);
        const ImVec2 size(textureWidth * scale, textureHeight * scale);
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (available.x - size.x) * 0.5f);
        const u32 nativeId = app().engine().getGPU().nativeTextureId(mOutput.color);
        const ImVec2 imageMin = ImGui::GetCursorScreenPos();
        ImGui::Image(static_cast<ImTextureID>(static_cast<uintptr_t>(nativeId)), size,
                     ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
        const ImVec2 imageMax(imageMin.x + size.x, imageMin.y + size.y);
        ImDrawList* overlay = ImGui::GetWindowDrawList();
        overlay->AddRect(imageMin, imageMax, IM_COL32(90, 165, 255, 255), 0.0f, 0, 1.0f);
        const std::string label =
            activeCameraName + "  |  " + std::to_string(mOutput.width) + " x " +
            std::to_string(mOutput.height) + "  |  Final";
        const ImVec2 textSize = ImGui::CalcTextSize(label.c_str());
        overlay->AddRectFilled(imageMin, ImVec2(imageMin.x + textSize.x + 12.0f,
                                                imageMin.y + textSize.y + 8.0f),
                               IM_COL32(12, 16, 24, 210));
        overlay->AddText(ImVec2(imageMin.x + 6.0f, imageMin.y + 4.0f),
                         IM_COL32(225, 238, 255, 255), label.c_str());
    }
    else
        ImGui::TextDisabled("No active scene camera");
}
} // namespace Radion
