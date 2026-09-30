#ifndef RADION_ANIMATION_H
#define RADION_ANIMATION_H

#include "Component.h"
#include "Skeleton.h"

#include "Math.h"
#include <string>
#include <vector>

namespace Radion
{

class Animator final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Animator;
    void bind(AnimationSetHandle animations);
    bool bound() const;
    AnimationSetHandle animationSet() const;
    AnimationLayer& layer(u32 index);
    u32 layerCount() const;
    void play(const std::string& clip, PlayMode mode = PlayMode::Loop, f32 blendTime = 0.2f);
    void update(f32 deltaTime);

    const Skeleton* skeleton() const;
    const std::vector<LocalPose>& localPose() const;
    const std::vector<Math::mat4>& globalPose() const;
    const std::vector<Math::mat4>& palette() const;
    const std::vector<Math::mat4>& prevPalette() const;

    // Solved every update() after pose evaluation and before the skinning palette is built.
    u32 addIKChain(const IKChain& chain);
    IKChain* ikChain(u32 index);
    u32 ikChainCount() const;
    void clearIKChains();

    bool boneGlobalPosition(s32 bone, Math::vec3& out) const;

    // While on, update() leaves mLocalPose as written; IK chains and the skinning palette are still rebuilt.
    void setPoseEditMode(bool enabled);
    bool poseEditMode() const;

    // Only meaningful in pose edit mode; playback would overwrite it otherwise.
    void setBoneLocalPose(u32 bone, const LocalPose& pose);

private:
    friend class GameObject;

    Animator();

    const AnimationClip* findClip(const std::string& name) const;

    AnimationSetHandle mAnimations;
    std::vector<AnimationLayer> mLayers;
    std::vector<LocalPose> mLocalPose;
    std::vector<LocalPose> mScratch;
    std::vector<Math::mat4> mGlobalPose;
    std::vector<Math::mat4> mPalette;
    std::vector<Math::mat4> mPrevPalette;
    std::vector<IKChain> mIKChains;
    bool mPoseEditMode = false;
};

} // namespace Radion

#endif // RADION_ANIMATION_H
