#include "character/Ragdoll.h"

#include "Skeleton.h"
#include "dynamics/PhysicsWorld.h"

#include <cctype>
#include <cmath>
#include <cstring>
#include <glm/gtc/matrix_transform.hpp>

#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#endif
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

namespace Radion::Physics
{

namespace
{

// Tail match on a bone name, case-insensitive, only across a name-part
// boundary (":"/"_" or the start of the string) - what makes an armature
// prefix like "mixamorig:" (or none at all) not matter, without also
// letting "LeftArm" match inside some other bone's "...LeftArmTwist".
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

// Same composition Skeleton::evaluate() uses to turn a LocalPose into a
// matrix - duplicated here rather than shared because it is three lines and
// Skeleton keeps it private (IKSolver does the same thing for the same
// reason).
glm::mat4 composeLocal(const Radion::LocalPose& pose)
{
    return glm::translate(glm::mat4(1.0f), pose.position) * glm::mat4_cast(pose.rotation) *
          glm::scale(glm::mat4(1.0f), pose.scale);
}

glm::quat rotationBetween(const glm::vec3& from, const glm::vec3& to)
{
    const f32 cosine = glm::clamp(glm::dot(from, to), -1.0f, 1.0f);
    if (cosine > 0.9999f)
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    if (cosine < -0.9999f)
        return glm::angleAxis(glm::pi<f32>(), glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::vec3 axis = glm::cross(from, to);
    const f32 scale = std::sqrt((1.0f + cosine) * 2.0f);
    return glm::normalize(glm::quat(scale * 0.5f, axis / scale));
}

// Ten parts, ten bits - starting well above any low bit a scene's own
// default CollisionFilter{1, ...} or the reference demo's own ragdoll
// (group 2) already use, so a ragdoll never collides with itself by
// construction and never has to know what layer numbers the rest of a
// scene happens to be using.
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

void Ragdoll::activate(PhysicsWorld& world, const std::vector<glm::mat4>& globalPose,
                       const std::vector<Radion::LocalPose>& localPose, const glm::mat4& ownerWorld)
{
    if (!mValid || mActive)
        return;

    mWorld = &world;
    mOwnerWorld = ownerWorld;
    mPointJoints.clear();
    mHingeJoints.clear();

    auto worldMatrix = [&](s32 bone) -> glm::mat4
    {
        return ownerWorld * globalPose[static_cast<usize>(bone)];
    };
    auto worldPosition = [&](s32 bone) -> glm::vec3
    {
        return glm::vec3(worldMatrix(bone)[3]);
    };

    std::array<glm::vec3, static_cast<usize>(RagdollPart::Count)> jointPosition;
    std::array<glm::vec3, static_cast<usize>(RagdollPart::Count)> segmentAxis;
    std::array<CollisionShape*, static_cast<usize>(RagdollPart::Count)> shapeOf{};

    for (usize i = 0; i < mParts.size(); ++i)
    {
        Part& p = mParts[i];
        const glm::vec3 nearPosition = worldPosition(p.boneIndex);
        const glm::vec3 farPosition = worldPosition(p.farBoneIndex);
        const glm::vec3 segment = farPosition - nearPosition;
        const f32 length = glm::max(glm::length(segment), 0.02f);
        const glm::vec3 axis = glm::length(segment) > 1.0e-5f ? segment / glm::length(segment)
                                                              : glm::vec3(0.0f, 1.0f, 0.0f);
        const glm::quat orientation = rotationBetween(glm::vec3(0.0f, 1.0f, 0.0f), axis);

        jointPosition[i] = nearPosition;
        segmentAxis[i] = axis;

        CollisionShape* shape = nullptr;
        glm::vec3 centre = (nearPosition + farPosition) * 0.5f;
        switch (p.shapeKind)
        {
        case Part::ShapeKind::Box:
            p.box = BoxShape(glm::vec3(glm::max(length * 0.32f, 0.05f),
                                       glm::max(length * 0.5f, 0.05f),
                                       glm::max(length * 0.22f, 0.05f)));
            shape = &p.box;
            p.mass = 18.0f;
            break;
        case Part::ShapeKind::Sphere:
            p.sphere = SphereShape(glm::max(length * 0.55f, 0.03f));
            shape = &p.sphere;
            centre = nearPosition;
            p.mass = 4.0f;
            break;
        case Part::ShapeKind::Capsule:
        default:
        {
            const f32 radius = glm::max(length * 0.18f, 0.015f);
            const f32 halfHeight = glm::max(length * 0.5f - radius, 0.02f);
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
        body.setVelocity(glm::vec3(0.0f));
        body.setAngularVelocity(glm::vec3(0.0f));
        body.setDamping(0.999f, 0.995f);
        body.calculateDerivedData();
        body.setAwake(true);

        shapeOf[i] = shape;

        BodyEntry entry;
        entry.body = &body;
        entry.shape = shape;
        entry.friction = 0.7f;
        entry.restitution = 0.05f;
        entry.filter = mFilter; // fixed up below, once every part's shape/pose is known
        p.bodyId = world.addBody(entry);

        // Fixed for the body's whole simulated life: where the animated
        // bone sat relative to the body frame we just chose for it.
        p.attachmentLocal = glm::inverse(body.transform()) * worldMatrix(p.boneIndex);

        glm::mat4 chain(1.0f);
        for (auto it = p.parentChain.rbegin(); it != p.parentChain.rend(); ++it)
            chain = chain * composeLocal(localPose[static_cast<usize>(*it)]);
        p.parentChainLocal = chain;
    }

    // Self-collision, the way Jolt's RagdollSettings::Stabilize() does it
    // (Jolt/Physics/Ragdoll/Ragdoll.cpp): a jointed pair never collides with
    // itself - the capsules meet exactly at the joint by construction, so
    // that contact is never anything but a false positive fighting the
    // joint every step - and neither does any OTHER pair that already
    // overlaps in the pose the ragdoll spawns into (Jolt tests the actual
    // shapes; an AABB test is enough here, this only ever runs on ten
    // parts). Everything else - left arm vs right arm, an arm swung across
    // the torso later, ends up in the same island - keeps colliding.
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
    {
        if (BodyEntry* entry = world.body(mParts[i].bodyId))
        {
            entry->filter.group = 1u << (kFirstPartBit + i);
            entry->filter.mask = mFilter.mask & ~excludeBit[i];
        }
    }

    const glm::vec3 lateral = part(RagdollPart::Hips).rigidBody.directionToWorld(glm::vec3(1.0f, 0.0f, 0.0f));

    for (usize i = 0; i < mParts.size(); ++i)
    {
        Part& p = mParts[i];
        if (p.parentPart == RagdollPart::Count)
            continue;
        Part& parentP = part(p.parentPart);
        const RagdollPart thisPart = static_cast<RagdollPart>(i);

        if (isLowerLimb(thisPart))
        {
            glm::vec3 axis = glm::cross(segmentAxis[static_cast<usize>(p.parentPart)], segmentAxis[i]);
            axis = glm::length(axis) > 1.0e-4f ? glm::normalize(axis) : lateral;
            mHingeJoints.emplace_back(parentP.rigidBody, p.rigidBody, jointPosition[i], axis);
            if (isLeg(thisPart))
                mHingeJoints.back().setLimits(-2.4f, 0.0f);
            else
                mHingeJoints.back().setLimits(0.0f, 2.4f);
            mWorld->addJoint(&mHingeJoints.back());
        }
        else
        {
            const glm::vec3 anchor =
                thisPart == RagdollPart::Head ? worldPosition(p.farBoneIndex) : jointPosition[i];
            mPointJoints.emplace_back(parentP.rigidBody, p.rigidBody, anchor);
            mWorld->addJoint(&mPointJoints.back());
        }
    }

    mActive = true;
}

void Ragdoll::deactivate()
{
    if (mActive && mWorld)
    {
        for (Part& p : mParts)
            if (p.bodyId != 0xFFFFFFFFu)
            {
                mWorld->removeBody(p.bodyId);
                p.bodyId = 0xFFFFFFFFu;
            }
        for (PointJoint& joint : mPointJoints)
            mWorld->removeJoint(&joint);
        for (HingeJoint& joint : mHingeJoints)
            mWorld->removeJoint(&joint);
    }
    mPointJoints.clear();
    mHingeJoints.clear();
    mWorld = nullptr;
    mActive = false;
}

void Ragdoll::writePose(std::vector<Radion::LocalPose>& localPose) const
{
    if (!mActive)
        return;

    std::array<glm::mat4, static_cast<usize>(RagdollPart::Count)> modelOf;
    const glm::mat4 invOwner = glm::inverse(mOwnerWorld);

    for (usize i = 0; i < mParts.size(); ++i)
    {
        const Part& p = mParts[i];
        if (p.boneIndex < 0 || static_cast<usize>(p.boneIndex) >= localPose.size())
            continue;

        const glm::mat4 boneWorld = p.rigidBody.transform() * p.attachmentLocal;
        const glm::mat4 boneModel = invOwner * boneWorld;
        modelOf[i] = boneModel;

        const glm::mat4 parentModel =
            p.parentPart == RagdollPart::Count ? glm::mat4(1.0f) : modelOf[static_cast<usize>(p.parentPart)];
        const glm::mat4 local = glm::inverse(p.parentChainLocal) * glm::inverse(parentModel) * boneModel;

        Radion::LocalPose pose;
        glm::vec3 skew;
        glm::vec4 perspective;
        if (!glm::decompose(local, pose.scale, pose.rotation, pose.position, skew, perspective))
            continue;
        pose.rotation = glm::normalize(pose.rotation);
        localPose[static_cast<usize>(p.boneIndex)] = pose;
    }
}

} // namespace Radion::Physics
