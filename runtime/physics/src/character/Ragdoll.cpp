#include "character/Ragdoll.h"

#include "Scene.h"
#include "Skeleton.h"

#include <cctype>
#include <cmath>
#include <cstring>
#include "Math.h"

namespace Radion::Physics
{

namespace
{

// Case-insensitive tail match only across a name-part boundary (':' / '_' / start), so armature prefixes do not matter
// and "LeftArm" does not match "...LeftArmTwist".
bool boneNameEndsWith(const std::string& name, const char* suffix)
{
    const usize suffixLength = std::strlen(suffix);
    if (name.size() < suffixLength)
        return false;
    const usize offset = name.size() - suffixLength;
    for (usize i = 0; i < suffixLength; ++i)
        if (std::tolower(static_cast<unsigned char>(name[offset + i])) !=
            std::tolower(static_cast<unsigned char>(suffix[i])))
            return false;
    return offset == 0 || name[offset - 1] == ':' || name[offset - 1] == '_';
}

s32 findBoneBySuffix(const Skeleton& skeleton, const char* suffix)
{
    for (u32 i = 0; i < skeleton.boneCount(); ++i)
        if (boneNameEndsWith(skeleton.bone(i).name, suffix))
            return static_cast<s32>(i);
    return -1;
}

s32 findBoneAnyOf(const Skeleton& skeleton, std::initializer_list<const char*> suffixes)
{
    for (const char* suffix : suffixes)
    {
        const s32 bone = findBoneBySuffix(skeleton, suffix);
        if (bone >= 0)
            return bone;
    }
    return -1;
}

// Same composition as Skeleton::evaluate(), duplicated because Skeleton keeps it private.
Math::mat4 composeLocal(const Radion::LocalPose& pose)
{
    return Math::translate(Math::mat4(1.0f), pose.position) * Math::mat4_cast(pose.rotation) *
          Math::scale(Math::mat4(1.0f), pose.scale);
}

Math::quat rotationBetween(const Math::vec3& from, const Math::vec3& to)
{
    const f32 cosine = Math::clamp(Math::dot(from, to), -1.0f, 1.0f);
    if (cosine > 0.9999f)
        return Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    if (cosine < -0.9999f)
        return Math::angleAxis(Math::pi<f32>(), Math::vec3(1.0f, 0.0f, 0.0f));
    const Math::vec3 axis = Math::cross(from, to);
    const f32 scale = std::sqrt((1.0f + cosine) * 2.0f);
    return Math::normalize(Math::quat(scale * 0.5f, axis / scale));
}

// Bits start well above the low bits scene/demo filters use, so a ragdoll never collides with itself or depends on the scene's layers.
constexpr u32 kFirstPartBit = 8;

bool isLowerLimb(RagdollPart part)
{
    return part == RagdollPart::LeftLowerArm || part == RagdollPart::RightLowerArm ||
          part == RagdollPart::LeftLowerLeg || part == RagdollPart::RightLowerLeg;
}

bool isLeg(RagdollPart part)
{
    return part == RagdollPart::LeftUpperLeg || part == RagdollPart::LeftLowerLeg ||
          part == RagdollPart::RightUpperLeg || part == RagdollPart::RightLowerLeg;
}

} // namespace

bool Ragdoll::build(const Skeleton& skeleton)
{
    mValid = false;
    if (skeleton.empty())
        return false;

    const s32 hips = findBoneBySuffix(skeleton, "Hips");
    const s32 spine = findBoneAnyOf(skeleton, {"Spine2", "Spine1", "Spine", "Chest"});
    const s32 neck = findBoneBySuffix(skeleton, "Neck");
    const s32 head = findBoneBySuffix(skeleton, "Head");
    const s32 leftArm = findBoneBySuffix(skeleton, "LeftArm");
    const s32 leftForeArm = findBoneBySuffix(skeleton, "LeftForeArm");
    const s32 leftHand = findBoneBySuffix(skeleton, "LeftHand");
    const s32 rightArm = findBoneBySuffix(skeleton, "RightArm");
    const s32 rightForeArm = findBoneBySuffix(skeleton, "RightForeArm");
    const s32 rightHand = findBoneBySuffix(skeleton, "RightHand");
    const s32 leftUpLeg = findBoneBySuffix(skeleton, "LeftUpLeg");
    const s32 leftLeg = findBoneBySuffix(skeleton, "LeftLeg");
    const s32 leftFoot = findBoneBySuffix(skeleton, "LeftFoot");
    const s32 rightUpLeg = findBoneBySuffix(skeleton, "RightUpLeg");
    const s32 rightLeg = findBoneBySuffix(skeleton, "RightLeg");
    const s32 rightFoot = findBoneBySuffix(skeleton, "RightFoot");

    const s32 required[] = {hips,     spine,    neck,     head,        leftArm,   leftForeArm,
                            leftHand, rightArm, rightForeArm, rightHand, leftUpLeg, leftLeg,
                            leftFoot, rightUpLeg, rightLeg, rightFoot};
    for (s32 bone : required)
        if (bone < 0)
            return false;

    part(RagdollPart::Hips).boneIndex = hips;
    part(RagdollPart::Hips).farBoneIndex = spine;
    part(RagdollPart::Hips).shapeKind = Part::ShapeKind::Box;

    part(RagdollPart::Head).boneIndex = head;
    part(RagdollPart::Head).farBoneIndex = neck;
    part(RagdollPart::Head).shapeKind = Part::ShapeKind::Sphere;

    part(RagdollPart::LeftUpperArm).boneIndex = leftArm;
    part(RagdollPart::LeftUpperArm).farBoneIndex = leftForeArm;
    part(RagdollPart::LeftLowerArm).boneIndex = leftForeArm;
    part(RagdollPart::LeftLowerArm).farBoneIndex = leftHand;
    part(RagdollPart::RightUpperArm).boneIndex = rightArm;
    part(RagdollPart::RightUpperArm).farBoneIndex = rightForeArm;
    part(RagdollPart::RightLowerArm).boneIndex = rightForeArm;
    part(RagdollPart::RightLowerArm).farBoneIndex = rightHand;

    part(RagdollPart::LeftUpperLeg).boneIndex = leftUpLeg;
    part(RagdollPart::LeftUpperLeg).farBoneIndex = leftLeg;
    part(RagdollPart::LeftLowerLeg).boneIndex = leftLeg;
    part(RagdollPart::LeftLowerLeg).farBoneIndex = leftFoot;
    part(RagdollPart::RightUpperLeg).boneIndex = rightUpLeg;
    part(RagdollPart::RightUpperLeg).farBoneIndex = rightLeg;
    part(RagdollPart::RightLowerLeg).boneIndex = rightLeg;
    part(RagdollPart::RightLowerLeg).farBoneIndex = rightFoot;

    resolveHierarchy(skeleton);

    mValid = true;
    return true;
}

void Ragdoll::resolveHierarchy(const Skeleton& skeleton)
{
    for (usize i = 0; i < mParts.size(); ++i)
    {
        Part& p = mParts[i];
        p.parentPart = RagdollPart::Count;
        p.parentChain.clear();

        s32 walk = skeleton.bone(static_cast<u32>(p.boneIndex)).parent;
        while (walk >= 0)
        {
            usize matched = mParts.size();
            for (usize j = 0; j < mParts.size(); ++j)
                if (j != i && mParts[j].boneIndex == walk)
                {
                    matched = j;
                    break;
                }
            if (matched != mParts.size())
            {
                p.parentPart = static_cast<RagdollPart>(matched);
                break;
            }
            p.parentChain.push_back(walk);
            walk = skeleton.bone(static_cast<u32>(walk)).parent;
        }
    }
}

void Ragdoll::activate(Radion::Scene& scene, const std::vector<Math::mat4>& globalPose,
                       const std::vector<Radion::LocalPose>& localPose, const Math::mat4& ownerWorld)
{
    if (!mValid || mActive)
        return;

    mScene = &scene;
    mOwnerWorld = ownerWorld;
    mPointJoints.clear();
    mHingeJoints.clear();

    auto worldMatrix = [&](s32 bone) -> Math::mat4
    {
        return ownerWorld * globalPose[static_cast<usize>(bone)];
    };
    auto worldPosition = [&](s32 bone) -> Math::vec3
    {
        return Math::vec3(worldMatrix(bone)[3]);
    };

    std::array<Math::vec3, static_cast<usize>(RagdollPart::Count)> jointPosition;
    std::array<Math::vec3, static_cast<usize>(RagdollPart::Count)> segmentAxis;
    std::array<CollisionShape*, static_cast<usize>(RagdollPart::Count)> shapeOf{};

    for (usize i = 0; i < mParts.size(); ++i)
    {
        Part& p = mParts[i];
        const Math::vec3 nearPosition = worldPosition(p.boneIndex);
        const Math::vec3 farPosition = worldPosition(p.farBoneIndex);
        const Math::vec3 segment = farPosition - nearPosition;
        const f32 length = Math::max(Math::length(segment), 0.02f);
        const Math::vec3 axis = Math::length(segment) > 1.0e-5f ? segment / Math::length(segment)
                                                              : Math::vec3(0.0f, 1.0f, 0.0f);
        const Math::quat orientation = rotationBetween(Math::vec3(0.0f, 1.0f, 0.0f), axis);

        jointPosition[i] = nearPosition;
        segmentAxis[i] = axis;

        CollisionShape* shape = nullptr;
        Math::vec3 centre = (nearPosition + farPosition) * 0.5f;
        switch (p.shapeKind)
        {
        case Part::ShapeKind::Box:
            p.box = BoxShape(Math::vec3(Math::max(length * 0.32f, 0.05f),
                                       Math::max(length * 0.5f, 0.05f),
                                       Math::max(length * 0.22f, 0.05f)));
            shape = &p.box;
            p.mass = 18.0f;
            break;
        case Part::ShapeKind::Sphere:
            p.sphere = SphereShape(Math::max(length * 0.55f, 0.03f));
            shape = &p.sphere;
            centre = nearPosition;
            p.mass = 4.0f;
            break;
        case Part::ShapeKind::Capsule:
        default:
        {
            const f32 radius = Math::max(length * 0.18f, 0.015f);
            const f32 halfHeight = Math::max(length * 0.5f - radius, 0.02f);
            p.capsule = CapsuleShape(radius, halfHeight);
            shape = &p.capsule;
            p.mass = isLeg(static_cast<RagdollPart>(i)) ? 4.0f : 2.0f;
            break;
        }
        }

        RigidBody& body = p.rigidBody;
        body.setBodyType(BodyType::Dynamic);
        body.setMass(p.mass);
        body.setInertiaTensor(shape->inertia(p.mass));
        body.setPosition(centre);
        body.setOrientation(orientation);
        body.setVelocity(Math::vec3(0.0f));
        body.setAngularVelocity(Math::vec3(0.0f));
        body.setDamping(0.999f, 0.995f);
        body.calculateDerivedData();
        body.setAwake(true);

        shapeOf[i] = shape;

        body.setShape(shape);
        body.setFriction(0.7f);
        body.setRestitution(0.05f);
        body.setFilter(mFilter); // fixed up below, once every part's shape/pose is known
        scene.addBody(body);

        // Fixed for the simulated life: where the animated bone sat relative to the body frame.
        p.attachmentLocal = Math::inverse(body.transform()) * worldMatrix(p.boneIndex);

        Math::mat4 chain(1.0f);
        for (auto it = p.parentChain.rbegin(); it != p.parentChain.rend(); ++it)
            chain = chain * composeLocal(localPose[static_cast<usize>(*it)]);
        p.parentChainLocal = chain;
    }

    // Self-collision as in Jolt's RagdollSettings::Stabilize(): jointed pairs never collide (capsules meet at the joint, so contact is a false
    // positive fighting it) and neither does any pair already overlapping at spawn (AABB test is enough for ten parts).
    std::array<u32, static_cast<usize>(RagdollPart::Count)> excludeBit{};
    for (usize i = 0; i < mParts.size(); ++i)
        if (mParts[i].parentPart != RagdollPart::Count)
        {
            const usize parentIndex = static_cast<usize>(mParts[i].parentPart);
            excludeBit[i] |= (1u << (kFirstPartBit + parentIndex));
            excludeBit[parentIndex] |= (1u << (kFirstPartBit + i));
        }
    for (usize i = 0; i < mParts.size(); ++i)
    {
        const AABB boundsI = shapeOf[i]->bounds(mParts[i].rigidBody.transform());
        for (usize j = i + 1; j < mParts.size(); ++j)
        {
            const AABB boundsJ = shapeOf[j]->bounds(mParts[j].rigidBody.transform());
            if (!boundsI.intersects(boundsJ))
                continue;
            excludeBit[i] |= (1u << (kFirstPartBit + j));
            excludeBit[j] |= (1u << (kFirstPartBit + i));
        }
    }
    for (usize i = 0; i < mParts.size(); ++i)
        mParts[i].rigidBody.setFilter({1u << (kFirstPartBit + i), mFilter.mask & ~excludeBit[i]});

    const Math::vec3 lateral = part(RagdollPart::Hips).rigidBody.directionToWorld(Math::vec3(1.0f, 0.0f, 0.0f));

    for (usize i = 0; i < mParts.size(); ++i)
    {
        Part& p = mParts[i];
        if (p.parentPart == RagdollPart::Count)
            continue;
        Part& parentP = part(p.parentPart);
        const RagdollPart thisPart = static_cast<RagdollPart>(i);

        if (isLowerLimb(thisPart))
        {
            Math::vec3 axis = Math::cross(segmentAxis[static_cast<usize>(p.parentPart)], segmentAxis[i]);
            axis = Math::length(axis) > 1.0e-4f ? Math::normalize(axis) : lateral;
            mHingeJoints.emplace_back(parentP.rigidBody, p.rigidBody, jointPosition[i], axis);
            if (isLeg(thisPart))
                mHingeJoints.back().setLimits(-2.4f, 0.0f);
            else
                mHingeJoints.back().setLimits(0.0f, 2.4f);
            mScene->addJoint(&mHingeJoints.back());
        }
        else
        {
            const Math::vec3 anchor =
                thisPart == RagdollPart::Head ? worldPosition(p.farBoneIndex) : jointPosition[i];
            mPointJoints.emplace_back(parentP.rigidBody, p.rigidBody, anchor);
            mScene->addJoint(&mPointJoints.back());
        }
    }

    mActive = true;
}

void Ragdoll::deactivate()
{
    if (mActive && mScene)
    {
        for (Part& p : mParts)
            if (p.rigidBody.scene())
                mScene->removeBody(p.rigidBody);
        for (PointJoint& joint : mPointJoints)
            mScene->removeJoint(&joint);
        for (HingeJoint& joint : mHingeJoints)
            mScene->removeJoint(&joint);
    }
    mPointJoints.clear();
    mHingeJoints.clear();
    mScene = nullptr;
    mActive = false;
}

void Ragdoll::writePose(std::vector<Radion::LocalPose>& localPose) const
{
    if (!mActive)
        return;

    std::array<Math::mat4, static_cast<usize>(RagdollPart::Count)> modelOf;
    const Math::mat4 invOwner = Math::inverse(mOwnerWorld);

    for (usize i = 0; i < mParts.size(); ++i)
    {
        const Part& p = mParts[i];
        if (p.boneIndex < 0 || static_cast<usize>(p.boneIndex) >= localPose.size())
            continue;

        const Math::mat4 boneWorld = p.rigidBody.transform() * p.attachmentLocal;
        const Math::mat4 boneModel = invOwner * boneWorld;
        modelOf[i] = boneModel;

        const Math::mat4 parentModel =
            p.parentPart == RagdollPart::Count ? Math::mat4(1.0f) : modelOf[static_cast<usize>(p.parentPart)];
        const Math::mat4 local = Math::inverse(p.parentChainLocal) * Math::inverse(parentModel) * boneModel;

        Radion::LocalPose pose;
        Math::vec3 skew;
        Math::vec4 perspective;
        if (!Math::decompose(local, pose.scale, pose.rotation, pose.position, skew, perspective))
            continue;
        pose.rotation = Math::normalize(pose.rotation);
        localPose[static_cast<usize>(p.boneIndex)] = pose;
    }
}

} // namespace Radion::Physics
