#ifndef RADION_ANIMATION_H
#define RADION_ANIMATION_H

#include "Component.h"
#include "Skeleton.h"

#include <glm/glm.hpp>
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
    // What bind() was last called with - null/invalid if never bound. What a
    // saved scene resolves back to a filename through
    // AnimationManager::sourceFile(), the same way MeshRenderer resolves its
    // MeshHandle through AssetManager::meshDesc().
    AnimationSetHandle animationSet() const;
    AnimationLayer& layer(u32 index);
    u32 layerCount() const;
    void play(const std::string& clip, PlayMode mode = PlayMode::Loop, f32 blendTime = 0.2f);
    void update(f32 deltaTime);

    const Skeleton* skeleton() const;
    const std::vector<LocalPose>& localPose() const;
    const std::vector<glm::mat4>& globalPose() const;
    const std::vector<glm::mat4>& palette() const;
    const std::vector<glm::mat4>& prevPalette() const;

    // IK chains, solved every update() right after the pose is evaluated and
    // before the skinning palette is built - see IKSolver. Held by value and
    // handed back by pointer so a caller can move a target every frame (foot
    // placement) without re-adding the chain.
    u32 addIKChain(const IKChain& chain);
    IKChain* ikChain(u32 index);
    u32 ikChainCount() const;
    void clearIKChains();

    // Model-space position of a bone in the pose as it stands, for a caller
    // that needs to know where a foot IS before deciding where to put it.
    // Returns false for an out-of-range bone rather than an identity matrix
    // that would read as a real position at the origin.
    bool boneGlobalPosition(s32 bone, glm::vec3& out) const;

    // Editor hand-posing: while on, update() leaves mLocalPose exactly as
    // last written instead of resampling bind pose + clip layers into it, so
    // a bone set through setBoneLocalPose() (or an IK chain's target, moved
    // by the caller through ikChain()) is not overwritten by playback next
    // frame. IK chains and the skinning palette are still rebuilt every
    // frame either way, so a dragged IK target keeps solving live.
    void setPoseEditMode(bool enabled);
    bool poseEditMode() const;

    // Writes one bone's local pose directly. Only meaningful in pose edit
    // mode - a bound clip's next update() would recompute over it otherwise.
    // Out-of-range bone is a no-op.
    void setBoneLocalPose(u32 bone, const LocalPose& pose);

private:
    friend class GameObject;

    Animator();

    const AnimationClip* findClip(const std::string& name) const;

    AnimationSetHandle mAnimations;
    std::vector<AnimationLayer> mLayers;
    std::vector<LocalPose> mLocalPose;
    std::vector<LocalPose> mScratch;
    std::vector<glm::mat4> mGlobalPose;
    std::vector<glm::mat4> mPalette;
    std::vector<glm::mat4> mPrevPalette;
    std::vector<IKChain> mIKChains;
    bool mPoseEditMode = false;
};

} // namespace Radion

#endif // RADION_ANIMATION_H
