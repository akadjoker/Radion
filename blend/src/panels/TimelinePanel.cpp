#include "PCH.h"
#include "TimelinePanel.h"
#include "../BlenderApplication.h"
#include "Skeleton.h"

#include <cstdio>
#include <IconsMaterialDesignIcons.h>
#include <imgui.h>

using namespace Radion;

TimelinePanel::TimelinePanel(BlenderApplication& app)
    : BlenderPanel("Timeline", app), mZoom(1.0f), mScrollOffset(0.0f)
{
}

TimelinePanel::~TimelinePanel()
{
}

void TimelinePanel::onImGui()
{
    if (ImGui::Begin(title().c_str()))
    {
        const MeshData* meshData = app().currentMeshData();
        const bool hasSkin = meshData && !meshData->skin.empty();

        ImGui::BeginDisabled(!hasSkin);
        drawAnimationClips();
        ImGui::Separator();
        drawPlaybackControls();
        ImGui::Separator();
        drawTimelineRuler();
        drawKeyframeTrack();
        ImGui::EndDisabled();
    }
    ImGui::End();
}

void TimelinePanel::drawAnimationClips()
{
    BlenderApplication& blender = app();

    ImGui::TextUnformatted("Animation Clips");
    ImGui::SameLine(ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize(ICON_MDI_PLUS).x -
                    ImGui::GetStyle().FramePadding.x * 2.0f);
    ImGui::BeginDisabled(!blender.hasSkeleton());
    if (ImGui::SmallButton(ICON_MDI_PLUS))
        blender.requestAppendAnimation();
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Append Animation... (needs a skeleton loaded first)");

    if (blender.animationClipCount() == 0)
    {
        ImGui::TextDisabled("No clips loaded");
        return;
    }

    s32 pendingDelete = -1;
    for (usize i = 0; i < blender.animationClipCount(); ++i)
    {
        ImGui::PushID(static_cast<int>(i));
        const AnimationClip& clip = blender.animationClip(i);
        const bool active = blender.activeAnimationClip() == static_cast<s32>(i);
        std::string label = clip.name().empty() ? "Clip " + std::to_string(i) : clip.name();
        char durationLabel[32];
        snprintf(durationLabel, sizeof(durationLabel), " (%.2fs)", clip.duration());
        label += durationLabel;

        if (ImGui::Selectable(label.c_str(), active))
            blender.setActiveAnimationClip(static_cast<s32>(i));

        ImGui::SameLine(ImGui::GetContentRegionAvail().x + ImGui::GetCursorPosX() -
                        ImGui::GetFrameHeight());
        if (ImGui::SmallButton(ICON_MDI_TRASH_CAN_OUTLINE))
            pendingDelete = static_cast<s32>(i);
        ImGui::PopID();
    }

    if (pendingDelete >= 0)
        blender.removeAnimationClip(static_cast<u32>(pendingDelete));
}

void TimelinePanel::drawPlaybackControls()
{
    BlenderApplication& blender = app();

    const char* playLabel = blender.isPlaying() ? "||" : ">";
    if (ImGui::Button(playLabel, ImVec2(30, 0)))
    {
        if (blender.isPlaying())
            blender.stop();
        else
            blender.play();
    }
    ImGui::SetItemTooltip(blender.isPlaying() ? "Pause" : "Play");

    ImGui::SameLine();
    u32 currentFrame = blender.currentFrame();
    u32 totalFrames = blender.totalFrames();
    const u32 minFrame = 0;

    if (ImGui::SliderScalar("##frameSlider", ImGuiDataType_U32, &currentFrame, &minFrame, &totalFrames,
                            "%d"))
    {
        blender.setCurrentFrame(currentFrame);
    }

    ImGui::SameLine();
    ImGui::Text("%u / %u", blender.currentFrame(), blender.totalFrames());

    ImGui::SameLine();
    bool loop = blender.settings().animation().autoLoop;
    if (ImGui::Checkbox("##loop", &loop))
    {
        blender.settings().animation().autoLoop = loop;
    }
    ImGui::SetItemTooltip("Loop Playback");
}

void TimelinePanel::drawTimelineRuler()
{
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    ImVec2 cursorPos = ImGui::GetCursorScreenPos();
    ImVec2 contentSize = ImGui::GetContentRegionAvail();

    f32 pixelsPerFrame = 10.0f * mZoom;
    u32 totalFrames = app().totalFrames();

    drawList->AddRectFilled(cursorPos, ImVec2(cursorPos.x + contentSize.x, cursorPos.y + 20),
                            IM_COL32(40, 40, 40, 255));

    for (u32 i = 0; i <= totalFrames; i += 10)
    {
        f32 x = cursorPos.x + mScrollOffset + i * pixelsPerFrame;
        if (x > cursorPos.x && x < cursorPos.x + contentSize.x)
        {
            drawList->AddLine(ImVec2(x, cursorPos.y), ImVec2(x, cursorPos.y + 10), IM_COL32(100, 100, 100, 255), 1.0f);
            char frameLabel[16];
            snprintf(frameLabel, sizeof(frameLabel), "%u", i);
            drawList->AddText(ImVec2(x + 2, cursorPos.y + 10), IM_COL32(200, 200, 200, 255), frameLabel);
        }
    }

    f32 playheadX = cursorPos.x + mScrollOffset + app().currentFrame() * pixelsPerFrame;
    drawList->AddLine(ImVec2(playheadX, cursorPos.y), ImVec2(playheadX, cursorPos.y + 20), IM_COL32(255, 100, 0, 255),
                      2.0f);

    ImGui::Dummy(ImVec2(contentSize.x, 25));
}

void TimelinePanel::drawKeyframeTrack()
{
    ImGui::Text("Keyframe Track");
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    ImVec2 cursorPos = ImGui::GetCursorScreenPos();
    ImVec2 contentSize = ImGui::GetContentRegionAvail();

    drawList->AddRectFilled(cursorPos, ImVec2(cursorPos.x + contentSize.x, cursorPos.y + contentSize.y),
                            IM_COL32(30, 30, 30, 255));

    ImGui::Dummy(contentSize);
}
