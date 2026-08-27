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
#include "Math.h"
#include <vector>

namespace Radion
{
class Skeleton;
struct LocalPose;
} // namespace Radion

namespace Radion::Physics
{

class PhysicsWorld;

// The ten bones a ragdoll actually simulates. Torso is one rigid body for
// the whole Hips-to-chest span (no independent spine), and every limb stops
// at the wrist/ankle - hands, feet, fingers and toes are never their own
// body, they just hang wherever the animation left them relative to the arm
// or leg that IS simulated.
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

// Auto-builds a physical ragdoll from a humanoid skeleton and, once active,
// feeds the resulting RigidBody transforms back into that same skeleton's
// pose - the skinned mesh itself goes limp. No separate proxy meshes, no
// hardcoded bone names: build() resolves the standard humanoid joints by
// name suffix (findBone() ignores whatever armature prefix the rig uses,
// "mixamorig:" or otherwise), so the same Ragdoll works for any similarly
// named biped skeleton.
//
// Typical use, once per character:
//   Ragdoll ragdoll;
//   if (ragdoll.build(*animator->skeleton()))
//   {
//       // on death:
//       ragdoll.activate(world, animator->globalPose(), animator->localPose(),
//                        doll->globalTransform());
//       animator->setPoseEditMode(true);
//       // every frame while active, after world.step():
//       std::vector<LocalPose> pose = animator->localPose();
//       ragdoll.writePose(pose);
//       for (u32 i = 0; i < pose.size(); ++i)
//           animator->setBoneLocalPose(i, pose[i]);
//       // to stand back up:
//       ragdoll.deactivate();
//       animator->setPoseEditMode(false);
//   }
class Ragdoll
{
public:
    // Resolves the tracked bones against `skeleton` by name suffix and works
    // out which of them are each other's simulated parent. False (and
    // unusable) if any is missing - a skeleton this sparse is not a biped
    // this class knows how to ragdoll. Shapes are not sized here: that
    // happens in activate(), from whatever pose the ragdoll actually starts
    // from, the same way the reference demo sizes its capsules at the
    // moment a doll dies rather than from the bind pose.
    bool build(const Skeleton& skeleton);

    bool valid() const
    {
        return mValid;
    }
    bool active() const
    {
        return mActive;
    }

    // Spawns the physics bodies and the joints between them, positioned and
    // oriented from `globalPose` (model space, e.g. Animator::globalPose())
    // as seen through `ownerWorld` - the doll's own GameObject transform,
    // assumed fixed for as long as the ragdoll stays active. `localPose` is
    // the same frame's local pose (e.g. Animator::localPose()); it is only
    // read, to freeze the non-simulated bones (spine, shoulders, neck,
    // wrists, ankles) exactly where the animation left them relative to
    // whichever simulated bone they hang off. Adds every body/joint to
    // `world`, which must outlive the ragdoll until deactivate().
    void activate(PhysicsWorld& world, const std::vector<Math::mat4>& globalPose,
                 const std::vector<LocalPose>& localPose, const Math::mat4& ownerWorld);

    // Removes every body and joint this ragdoll added to its world. Safe to
    // call when not active.
    void deactivate();

    // Writes this frame's physics-driven local pose into the ten tracked
    // bones of `localPose` (already sized to the skeleton's bone count,
    // e.g. a copy of Animator::localPose()) - every other bone is left
    // untouched. No-op when not active. Call after PhysicsWorld::step(),
    // before handing the result to Animator::setBoneLocalPose().
    void writePose(std::vector<LocalPose>& localPose) const;

    RigidBody* body(RagdollPart part)
    {
        return &mParts[static_cast<usize>(part)].rigidBody;
    }

    // Base mask every part's filter starts from before this ragdoll's own
    // per-pair exclusions are subtracted out of it (group is ignored - each
    // part gets its own bit, see activate()). Defaults to "collide with
    // everything"; a caller that wants the whole ragdoll kept off some
    // external category (e.g. a "corpse" layer other queries skip) clears
    // that bit here before activate().
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
        u32 bodyId = 0xFFFFFFFFu;
        Math::mat4 attachmentLocal{1.0f}; // bone, expressed in the body's own local frame
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
    PhysicsWorld* mWorld = nullptr;
    Math::mat4 mOwnerWorld{1.0f};
    CollisionFilter mFilter{1u, 0xFFFFFFFFu};
    bool mValid = false;
    bool mActive = false;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_CHARACTER_RAGDOLL_H
