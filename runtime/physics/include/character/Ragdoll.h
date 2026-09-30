#ifndef RADION_PHYSICS_CHARACTER_RAGDOLL_H
#define RADION_PHYSICS_CHARACTER_RAGDOLL_H

#include "Types.h"
#include "collision/CollisionFilter.h"
#include "collision/CollisionShape.h"
#include "dynamics/HingeJoint.h"
#include "dynamics/PointJoint.h"
#include "dynamics/RigidBody.h"

#include <array>
#include <deque>
#include "Math.h"
#include <vector>

namespace Radion
{
class Scene;
class Skeleton;
struct LocalPose;
} // namespace Radion

namespace Radion::Physics
{

// Ten simulated bones: torso is one body (no spine) and limbs stop at wrist/ankle; hands and feet just follow the animation.
enum class RagdollPart : u8
{
    Hips,
    Head,
    LeftUpperArm,
    LeftLowerArm,
    RightUpperArm,
    RightLowerArm,
    LeftUpperLeg,
    LeftLowerLeg,
    RightUpperLeg,
    RightLowerLeg,
    Count
};

// Builds a physical ragdoll from a humanoid skeleton and feeds RigidBody transforms back into its pose. build() resolves
// joints by name suffix, so any armature prefix works.
// Use: build() once; activate() on death; writePose() each frame after Scene::stepPhysics(); deactivate() to stand up.
class Ragdoll
{
public:
    // Resolves tracked bones by name suffix and their simulated parents; false (unusable) if any is missing.
    // Shapes are sized in activate() from the starting pose, not here.
    bool build(const Skeleton& skeleton);

    bool valid() const
    {
        return mValid;
    }
    bool active() const
    {
        return mActive;
    }

    // Spawns bodies and joints from `globalPose` (model space) seen through `ownerWorld` (assumed fixed while active);
    // `localPose` is only read, to freeze non-simulated bones. `scene` must outlive the ragdoll until deactivate().
    void activate(Radion::Scene& scene, const std::vector<Math::mat4>& globalPose,
                 const std::vector<LocalPose>& localPose, const Math::mat4& ownerWorld);

    // Safe to call when not active.
    void deactivate();

    // Writes the physics-driven local pose into the ten tracked bones of `localPose` (sized to the bone count); others untouched.
    // No-op when inactive; call after Scene::stepPhysics().
    void writePose(std::vector<LocalPose>& localPose) const;

    RigidBody* body(RagdollPart part)
    {
        return &mParts[static_cast<usize>(part)].rigidBody;
    }

    // Base mask each part's filter starts from before per-pair exclusions are subtracted (group is ignored, see activate()).
    // Defaults to colliding with everything.
    void setCollisionMask(u32 mask)
    {
        mFilter.mask = mask;
    }

private:
    struct Part
    {
        s32 boneIndex = -1;
        s32 farBoneIndex = -1; // reference bone the segment/size is measured against
        RagdollPart parentPart = RagdollPart::Count; // Count: parented to the owner root
        std::vector<s32> parentChain; // non-simulated bones between parentPart and this one,
                                      // nearest-to-this-bone first (build() order, see .cpp)
        Math::mat4 parentChainLocal{1.0f}; // parentChain composed against the live pose, set by activate()

        enum class ShapeKind : u8
        {
            Capsule,
            Box,
            Sphere
        } shapeKind = ShapeKind::Capsule;
        CapsuleShape capsule{0.08f, 0.10f};
        BoxShape box{Math::vec3(0.15f)};
        SphereShape sphere{0.10f};
        f32 mass = 1.0f;

        RigidBody rigidBody;
        Math::mat4 attachmentLocal{1.0f};
    };

    void resolveHierarchy(const Skeleton& skeleton);
    Part& part(RagdollPart p)
    {
        return mParts[static_cast<usize>(p)];
    }
    const Part& part(RagdollPart p) const
    {
        return mParts[static_cast<usize>(p)];
    }

    std::array<Part, static_cast<usize>(RagdollPart::Count)> mParts;
    std::deque<PointJoint> mPointJoints;
    std::deque<HingeJoint> mHingeJoints;
    Radion::Scene* mScene = nullptr;
    Math::mat4 mOwnerWorld{1.0f};
    CollisionFilter mFilter{1u, 0xFFFFFFFFu};
    bool mValid = false;
    bool mActive = false;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_CHARACTER_RAGDOLL_H
