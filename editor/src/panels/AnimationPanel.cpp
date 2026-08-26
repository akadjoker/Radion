#include "PCH.h"

#include "panels/AnimationPanel.h"

#include "Animation.h"
#include "EditorApplication.h"
#include "GameObject.h"
#include "Log.h"

#include <IconsMaterialDesignIcons.h>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <imgui.h>

namespace Radion
{

AnimationPanel::AnimationPanel(EditorApplication& app) : EditorPanel("Animation", app)
{
}

void AnimationPanel::onImGui()
{
    GameObject* object = app().selection().resolve(app().scene());
    Animator* animator = object ? object->getComponent<Animator>() : nullptr;

    // Selection moved off whatever this panel had in pose-edit mode - drop
    // it back to normal playback rather than leaving it frozen off-screen
    // with no UI left pointed at it.
    if (mEditingAnimator && mEditingAnimator != animator)
    {
        mEditingAnimator->setPoseEditMode(false);
        mEditingAnimator = nullptr;
        app().animationPoseTarget() = EditorApplication::AnimationPoseTarget();
    }

    if (!animator)
    {
        ImGui::TextDisabled("Select an object with an Animator component to scrub or pose its "
                            "animation.");
        return;
    }
    if (!animator->bound())
    {
        ImGui::TextDisabled("Animator has no animation set bound.");
        return;
    }

    drawPlayback(*animator);
    ImGui::Separator();
    drawPoseEditor(*animator);
}

void AnimationPanel::drawPlayback(Animator& animator)
{
    const AnimationSet* set = Animations().get(animator.animationSet());
    if (!set)
        return;

    // Pick-a-clip first, transport controls (Pause/Loop/Scrub) for whatever
    // ends up playing second, right below each other rather than Loop
    // sitting above the list it has no effect on until something is
    // actually playing.
    AnimationLayer* layer = animator.layerCount() > 0 ? &animator.layer(0) : nullptr;

    // Animator::update() re-samples bind pose every frame before layering
    // clips on top, so stopping every layer (and dropping pose-edit if it
    // was on) IS the bind pose - no separate "reset" path needed.
    bool anyPlaying = animator.poseEditMode();
    for (u32 i = 0; i < animator.layerCount() && !anyPlaying; ++i)
        anyPlaying = !animator.layer(i).current().empty();
    ImGui::BeginDisabled(!anyPlaying);
    if (ImGui::Button(ICON_MDI_HUMAN " Bind Pose"))
    {
        for (u32 i = 0; i < animator.layerCount(); ++i)
            animator.layer(i).stop();
        if (animator.poseEditMode())
        {
            animator.setPoseEditMode(false);
            mEditingAnimator = nullptr;
            app().animationPoseTarget() = EditorApplication::AnimationPoseTarget();
        }
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Stops every layer (and pose editing) - the model returns to its "
                          "skeleton's bind pose.");

    ImGui::TextUnformatted("Clips");
    ImGui::BeginChild("AnimationPanelClips", ImVec2(0.0f, 100.0f), true);
    for (const AnimationClip& clip : set->clips)
    {
        ImGui::PushID(&clip);
        // Play/Stop, one button - the same toggle any media player uses,
        // rather than a separate always-there Play per row plus one lone
        // Stop below that only ever talks about layer 0's clip.
        const bool isThisClip = layer && layer->current() == clip.name();
        if (ImGui::Button(isThisClip ? ICON_MDI_STOP : ICON_MDI_PLAY))
        {
            if (isThisClip)
                layer->stop();
            else
                animator.play(clip.name(), mLoop ? PlayMode::Loop : PlayMode::Once);
        }
        ImGui::SameLine();
        ImGui::TextUnformatted(clip.name().c_str());
        ImGui::PopID();
    }
    ImGui::EndChild();

    if (!layer || layer->current().empty())
        return;
    bool paused = layer->paused();
    if (ImGui::Checkbox(ICON_MDI_PAUSE " Pause", &paused))
        layer->setPaused(paused);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Freezes this clip exactly where it is (unlike Stop, it stays "
                          "selected) - Scrub below only holds still while this is on, "
                          "otherwise the next frame just keeps playing past wherever you "
                          "dragged it.");
    ImGui::SameLine();
    ImGui::Checkbox("Loop", &mLoop);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Off plays the clip once and holds its last frame instead of "
                          "restarting - applies next time you click Play, not to what is "
                          "already playing.");

    // wrappedTime(), not time() - time() is the raw elapsed seconds
    // Animator::update() blends against, and keeps climbing for as long as a
    // Loop clip keeps looping (16s into a clip that lasts 2s, e.g.) instead
    // of resetting each pass, which reads as broken here and puts the scrub
    // slider below permanently pinned past its own max.
    float time = layer->wrappedTime();
    ImGui::Text("%s  %.2f / %.2fs", layer->current().c_str(), static_cast<double>(time),
               static_cast<double>(layer->duration()));
    if (ImGui::SliderFloat("Scrub", &time, 0.0f, glm::max(layer->duration(), 0.001f)))
    {
        layer->seek(time);
        // Dragging without Pause on would just have the next frame march
        // straight past the dragged position - scrubbing implies "hold this
        // frame still while I look at it", so turn Pause on for them rather
        // than make them find the checkbox first.
        layer->setPaused(true);
    }

    ImGui::SetNextItemWidth(70.0f);
    ImGui::DragFloat("FPS", &mFps, 0.5f, 1.0f, 240.0f, "%.0f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Only converts the Frame field to/from seconds for display - clips "
                          "are always sampled by seconds regardless, this changes nothing about "
                          "playback itself.");
    ImGui::SameLine();
    const int frameCount = static_cast<int>(std::lround(layer->duration() * mFps));
    int frame = static_cast<int>(std::lround(time * mFps));
    ImGui::SetNextItemWidth(120.0f);
    if (ImGui::DragInt("Frame", &frame, 1.0f, 0, glm::max(frameCount, 0)))
    {
        layer->seek(static_cast<f32>(frame) / glm::max(mFps, 0.0001f));
        layer->setPaused(true);
    }
}

// The one always-visible control here, same job as Lumix's own "Preview"
// checkbox on its Animable property grid (animation_plugins.cpp) - primary
// action up front, everything it unlocks tucked behind CollapsingHeaders
// below rather than flowing loose in one long column.
void AnimationPanel::drawPoseEditor(Animator& animator)
{
    const Skeleton* skeleton = animator.skeleton();
    if (!skeleton)
        return;

    bool editing = animator.poseEditMode();
    if (ImGui::Checkbox("Edit Pose", &editing))
    {
        animator.setPoseEditMode(editing);
        mEditingAnimator = editing ? &animator : nullptr;
        EditorApplication::AnimationPoseTarget& target = app().animationPoseTarget();
        target = EditorApplication::AnimationPoseTarget();
        target.active = editing;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Freezes clip playback on this Animator so a bone rotated (or an IK "
                          "chain's target dragged) in the Viewport sticks instead of being "
                          "overwritten by the next frame of playback.");
    if (!editing)
        return;

    EditorApplication::AnimationPoseTarget& target = app().animationPoseTarget();

    if (ImGui::CollapsingHeader("Bones && IK Chains", ImGuiTreeNodeFlags_DefaultOpen))
        drawBonesAndChains(animator, *skeleton, target);

    // Closed by default - authoring a new clip is the rarer, more deliberate
    // action of the two (posing/scrubbing is what most sessions here are
    // for), same reasoning Lumix's own Transformation dump collapses by
    // default under its Preview/Time pair.
    if (ImGui::CollapsingHeader("Author Clip"))
        drawClipAuthoring(animator, *skeleton, target);
}

void AnimationPanel::drawBonesAndChains(Animator& animator, const Skeleton& skeleton,
                                        EditorApplication::AnimationPoseTarget& target)
{
    ImGui::TextUnformatted("Bones");
    ImGui::BeginChild("AnimationPanelBones", ImVec2(0.0f, 150.0f), true);
    for (u32 i = 0; i < skeleton.boneCount(); ++i)
    {
        ImGui::PushID(static_cast<int>(i));
        const bool selected = target.bone == static_cast<s32>(i);
        if (ImGui::Selectable(skeleton.bone(i).name.c_str(), selected))
        {
            target.bone = static_cast<s32>(i);
            target.ikChain = -1;
        }
        ImGui::PopID();
    }
    ImGui::EndChild();

    ImGui::TextUnformatted("IK Chains");
    ImGui::BeginDisabled(target.bone < 0);
    if (ImGui::Button(ICON_MDI_PLUS " Add Chain From Selected Bone"))
    {
        IKChain chain;
        chain.tipBone = target.bone;
        Math::Vec3 modelPosition;
        if (animator.boneGlobalPosition(target.bone, modelPosition) && animator.owner())
            chain.target =
                Math::Vec3(animator.owner()->globalTransform() * Math::Vec4(modelPosition, 1.0f));
        target.ikChain = static_cast<s32>(animator.addIKChain(chain));
        target.bone = -1;
    }
    ImGui::EndDisabled();
    ImGui::BeginChild("AnimationPanelChains", ImVec2(0.0f, 90.0f), true);
    for (u32 i = 0; i < animator.ikChainCount(); ++i)
    {
        IKChain* chain = animator.ikChain(i);
        if (!chain)
            continue;
        char label[96];
        const char* tipName = chain->tipBone >= 0 &&
                                     static_cast<u32>(chain->tipBone) < skeleton.boneCount()
                                 ? skeleton.bone(static_cast<u32>(chain->tipBone)).name.c_str()
                                 : "?";
        std::snprintf(label, sizeof(label), "Chain %u - tip: %s, length: %u", i, tipName,
                     chain->length);
        ImGui::PushID(static_cast<int>(i));
        const bool selected = target.ikChain == static_cast<s32>(i);
        if (ImGui::Selectable(label, selected))
        {
            target.ikChain = static_cast<s32>(i);
            target.bone = -1;
        }
        ImGui::PopID();
    }
    ImGui::EndChild();
}

void AnimationPanel::drawClipAuthoring(Animator& animator, const Skeleton& skeleton,
                                       EditorApplication::AnimationPoseTarget& target)
{
    ImGui::InputText("Name", mClipName, sizeof(mClipName));
    if (ImGui::Button(ICON_MDI_FILE_PLUS " New Clip"))
    {
        mClip = AnimationClip();
        mClip.setName(mClipName);
        mHasClip = true;
        mTime = 0.0f;
    }

    if (!mHasClip)
        return;

    ImGui::Text("%d bone track(s), %.2fs", static_cast<int>(mClip.tracks().size()),
               static_cast<double>(mClip.duration()));
    ImGui::DragFloat("Time (s)", &mTime, 0.05f, 0.0f, 3600.0f, "%.2f");

    ImGui::BeginDisabled(target.bone < 0 && target.ikChain < 0);
    if (ImGui::Button(ICON_MDI_KEY_PLUS " Set Key"))
    {
        if (target.bone >= 0)
            mClip.setKeyframe(target.bone, mTime, animator.localPose()[target.bone]);
        else if (target.ikChain >= 0)
        {
            IKChain* chain = animator.ikChain(static_cast<u32>(target.ikChain));
            if (chain && chain->tipBone >= 0)
            {
                s32 bone = skeleton.bone(static_cast<u32>(chain->tipBone)).parent;
                for (u32 link = 0; link < chain->length && bone >= 0; ++link)
                {
                    mClip.setKeyframe(bone, mTime, animator.localPose()[static_cast<u32>(bone)]);
                    bone = skeleton.bone(static_cast<u32>(bone)).parent;
                }
            }
        }
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Bakes the selected bone's current pose - or every bone the selected "
                          "IK chain touches - into this clip at Time.");

    ImGui::SameLine();
    if (ImGui::Button(ICON_MDI_CONTENT_SAVE " Save As..."))
    {
        mSaveDialog.Open(ImGuiFileDialog::Mode::SaveFile, RADION_ASSET_DIR,
                         std::string(mClipName) + ".ranim");
        mSavePending = true;
    }
    if (mSavePending && mSaveDialog.Render(RADION_ASSET_DIR, RADION_ASSET_DIR, RADION_ASSET_DIR))
    {
        const ImGuiFileDialog::Result result = mSaveDialog.ConsumeResult();
        mSavePending = false;
        if (result.accepted)
        {
            mClip.setName(mClipName);
            if (RadionSkeletonIO::saveAnimation(result.path.string(), skeleton, mClip))
                Log::info("AnimationPanel: saved '%s'", result.path.string().c_str());
            else
                Log::error("AnimationPanel: could not save '%s'", result.path.string().c_str());
        }
    }
}

} // namespace Radion
