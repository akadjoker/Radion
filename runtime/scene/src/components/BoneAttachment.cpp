#include "PCH.h"

#include "BoneAttachment.h"

#include "GameObject.h"

#define GLM_ENABLE_EXPERIMENTAL
#include "Math.h"

namespace Radion
{

BoneAttachment::BoneAttachment() : Component(Type)
{
}

bool BoneAttachment::bind(Animator* animator, const std::string& boneName)
{
    const Skeleton* skeleton = animator ? animator->skeleton() : nullptr;
    return bind(animator, skeleton ? skeleton->findBone(boneName.c_str()) : -1);
}

bool BoneAttachment::bind(Animator* animator, s32 boneIndex)
{
    const Skeleton* skeleton = animator ? animator->skeleton() : nullptr;
    if (!skeleton || boneIndex < 0 || boneIndex >= static_cast<s32>(skeleton->boneCount()))
    {
        mAnimator = nullptr;
        mBoneIndex = -1;
        return false;
    }
    mAnimator = animator;
    mBoneIndex = boneIndex;
    return true;
}

void BoneAttachment::unbind()
{
    mAnimator = nullptr;
    mBoneIndex = -1;
}

Animator* BoneAttachment::animator() const
{
    return mAnimator;
}
s32 BoneAttachment::boneIndex() const
{
    return mBoneIndex;
}

void BoneAttachment::update()
{
    if (!active() || !mAnimator || mBoneIndex < 0 || !owner())
        return;
    const std::vector<Math::mat4>& pose = mAnimator->globalPose();
    if (mBoneIndex >= static_cast<s32>(pose.size()))
        return;

    Math::vec3 scale;
    Math::quat rotation;
    Math::vec3 position;
    Math::vec3 skew;
    Math::vec4 perspective;
    if (!Math::decompose(pose[static_cast<usize>(mBoneIndex)], scale, rotation, position, skew,
                        perspective))
        return;
    owner()->setPosition(position);
    owner()->setRotation(Math::normalize(rotation));
    owner()->setScale(scale);
}

} // namespace Radion
