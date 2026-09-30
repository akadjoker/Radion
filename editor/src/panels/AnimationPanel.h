#ifndef RADION_ANIMATION_PANEL_H
#define RADION_ANIMATION_PANEL_H

#include "EditorApplication.h"
#include "EditorPanel.h"
#include "ImGuiFileDialog.h"
#include "Skeleton.h"

namespace Radion
{
class Animator;

 
class AnimationPanel final : public EditorPanel
{
public:
    explicit AnimationPanel(EditorApplication& app);
    void onImGui() override;

private:
    void drawPlayback(Animator& animator);
    void drawPoseEditor(Animator& animator);
    void drawBonesAndChains(Animator& animator, const Skeleton& skeleton,
                            EditorApplication::AnimationPoseTarget& target);
    void drawClipAuthoring(Animator& animator, const Skeleton& skeleton,
                           EditorApplication::AnimationPoseTarget& target);

    // Animator left in pose-edit mode, so a selection change can take it back out.
    Animator* mEditingAnimator = nullptr;

    // Applies to the next Play click; AnimationLayer has no mode() getter to read it back.
    bool mLoop = true;
    // Only converts the Frame field to/from wrappedTime() seconds; clips are always sampled in seconds.
    f32 mFps = 30.0f;

    bool mHasClip = false;
    AnimationClip mClip;
    char mClipName[64] = "NewClip";
    f32 mTime = 0.0f;

    ImGuiFileDialog mSaveDialog;
    bool mSavePending = false;
};
} // namespace Radion

#endif
