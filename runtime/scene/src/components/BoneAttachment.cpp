#include "PCH.h"

#include "BoneAttachment.h"

#include "GameObject.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/matrix_decompose.hpp>

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
    const std::vector<glm::mat4>& pose = mAnimator->globalPose();
    if (mBoneIndex >= static_cast<s32>(pose.size()))
        return;

    glm::vec3 scale;
    glm::quat rotation;
    glm::vec3 position;
    glm::vec3 skew;
    glm::vec4 perspective;
    if (!glm::decompose(pose[static_cast<usize>(mBoneIndex)], scale, rotation, position, skew,
                        perspective))
        return;
    owner()->setPosition(position);
    owner()->setRotation(glm::normalize(rotation));
    owner()->setScale(scale);
}

} // namespace Radion
