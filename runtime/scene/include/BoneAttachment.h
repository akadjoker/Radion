#ifndef RADION_BONE_ATTACHMENT_H
#define RADION_BONE_ATTACHMENT_H

#include "Animation.h"
#include "Component.h"

#include <string>

namespace Radion
{

class BoneAttachment final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::BoneAttachment;

    bool bind(Animator* animator, const std::string& boneName);
    bool bind(Animator* animator, s32 boneIndex);
    void unbind();
    Animator* animator() const;
    s32 boneIndex() const;

private:
    friend class GameObject;
    friend class Scene;

    BoneAttachment();
    void update();

    Animator* mAnimator = nullptr;
    s32 mBoneIndex = -1;
};

} // namespace Radion

#endif // RADION_BONE_ATTACHMENT_H
