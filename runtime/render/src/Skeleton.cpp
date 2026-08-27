#include "PCH.h"

#include "Skeleton.h"

#include "ByteArray.h"
#include "FileSystem.h"
#include "RadionFormat.h"

#include <cstring>
#include <limits>

#define GLM_ENABLE_EXPERIMENTAL
#include "Math.h"
#include "Math.h"

namespace Radion
{

namespace
{

Math::quat nlerp(const Math::quat& from, Math::quat to, f32 amount)
{
    if (Math::dot(from, to) < 0.0f)
        to = -to;
    return Math::normalize(from * (1.0f - amount) + to * amount);
}

// Same "match by name suffix" convention Ragdoll::build() uses to find a
// humanoid rig's Hips regardless of the exporter's own prefix
// ("mixamorig:Hips", "Hips", ...).
s32 findBoneBySuffix(const Skeleton& skeleton, const char* suffix)
{
    const usize suffixLength = std::strlen(suffix);
    for (u32 i = 0; i < skeleton.boneCount(); ++i)
    {
        const std::string& name = skeleton.bone(i).name;
        if (name.size() >= suffixLength &&
            name.compare(name.size() - suffixLength, suffixLength, suffix) == 0)
            return static_cast<s32>(i);
    }
    return -1;
}

// A locomotion/action clip's Hips track should sit at roughly the same
// height as every other clip sharing this skeleton - that is what the
// FbxImporter in-place conversion pins it to (FbxImporter.cpp, the
// "!keepRootMotion" block). A clip whose Hips baseline drifts far from the
// rest of the set is either missing that conversion (imported with root
// motion kept by mistake) or ends on a pose far off the character's own
// standing height - either way the character visibly steps up or down when
// switching into that clip, so this is caught here once, for every set,
// instead of an artist spotting a floating/sunken character per demo.
constexpr f32 kHipsHeightMismatchThreshold = 8.0f; // asset-space units (cm for a Mixamo rig)

f32 lowestTrackHeight(const AnimationClip& clip, s32 boneIndex)
{
    for (const BoneTrack& track : clip.tracks())
    {
        if (track.bone != boneIndex || track.positions.empty())
            continue;
        f32 lowest = track.positions.front().y;
        for (const Math::vec3& position : track.positions)
            lowest = Math::min(lowest, position.y);
        return lowest;
    }
    return std::numeric_limits<f32>::quiet_NaN();
}

void warnOnHipsHeightMismatch(const std::string& skeletonFile, const Skeleton& skeleton,
                              const std::vector<std::string>& animationFiles,
                              const std::vector<AnimationClip>& clips)
{
    const s32 hips = findBoneBySuffix(skeleton, "Hips");
    if (hips < 0)
        return;

    f32 referenceHeight = std::numeric_limits<f32>::quiet_NaN();
    std::string referenceFile;
    for (usize i = 0; i < clips.size(); ++i)
    {
        const f32 height = lowestTrackHeight(clips[i], hips);
        if (std::isnan(height))
            continue;
        if (std::isnan(referenceHeight))
        {
            referenceHeight = height;
            referenceFile = animationFiles[i];
            continue;
        }
        if (Math::abs(height - referenceHeight) > kHipsHeightMismatchThreshold)
            Log::warning(
                "AnimationManager: '%s' Hips baseline (%.2f) differs from '%s' (%.2f) by "
                "more than %.1f in skeleton '%s' - one of these clips likely was not "
                "generated \"in place\", the character will visibly step up/down "
                "switching into it",
                animationFiles[i].c_str(), height, referenceFile.c_str(), referenceHeight,
                kHipsHeightMismatchThreshold, skeletonFile.c_str());
    }
}

} // namespace

bool Skeleton::empty() const
{
    return mBones.empty();
}
u32 Skeleton::boneCount() const
{
    return static_cast<u32>(mBones.size());
}
const Bone& Skeleton::bone(u32 index) const
{
    return mBones[index];
}

s32 Skeleton::findBone(const char* name) const
{
    if (!name)
        return -1;
    for (usize i = 0; i < mBones.size(); ++i)
        if (mBones[i].name == name)
            return static_cast<s32>(i);
    return -1;
}

bool Skeleton::addBone(const std::string& name, s32 parent, const Math::mat4& bindLocal,
                       const Math::mat4& inverseBind)
{
    if (name.empty() || parent < -1 || parent >= 65535 || findBone(name.c_str()) >= 0)
        return false;
    mBones.push_back({name, parent, bindLocal, inverseBind});
    return true;
}

bool Skeleton::finalize()
{
    mOrder.clear();
    mOrder.reserve(mBones.size());
    std::vector<bool> placed(mBones.size(), false);
    bool progress = true;
    while (mOrder.size() < mBones.size() && progress)
    {
        progress = false;
        for (usize i = 0; i < mBones.size(); ++i)
        {
            if (placed[i])
                continue;
            const s32 parent = mBones[i].parent;
            if (parent >= static_cast<s32>(mBones.size()) || parent == static_cast<s32>(i))
                return false;
            if (parent < 0 || placed[static_cast<usize>(parent)])
            {
                placed[i] = true;
                mOrder.push_back(static_cast<u16>(i));
                progress = true;
            }
        }
    }
    return mOrder.size() == mBones.size();
}

void Skeleton::bindPose(std::vector<LocalPose>& pose) const
{
    pose.resize(mBones.size());
    for (usize i = 0; i < mBones.size(); ++i)
    {
        Math::vec3 skew;
        Math::vec4 perspective;
        if (!Math::decompose(mBones[i].bindLocal, pose[i].scale, pose[i].rotation, pose[i].position,
                            skew, perspective))
        {
            pose[i] = LocalPose();
            continue;
        }
        pose[i].rotation = Math::normalize(pose[i].rotation);
    }
}

void Skeleton::evaluate(const std::vector<LocalPose>& localPose, std::vector<Math::mat4>& globalPose,
                        std::vector<Math::mat4>& palette) const
{
    if (localPose.size() != mBones.size() || mOrder.size() != mBones.size())
        return;
    globalPose.resize(mBones.size());
    palette.resize(mBones.size());
    for (u16 index : mOrder)
    {
        const LocalPose& pose = localPose[index];
        const Math::mat4 local = Math::translate(Math::mat4(1.0f), pose.position) *
                                Math::mat4_cast(pose.rotation) *
                                Math::scale(Math::mat4(1.0f), pose.scale);
        const s32 parent = mBones[index].parent;
        globalPose[index] = parent >= 0 ? globalPose[static_cast<usize>(parent)] * local : local;
        palette[index] = globalPose[index] * mBones[index].inverseBind;
    }
}

// ------------------------------------------------------------ inverse kinematics

namespace
{

Math::mat4 composeLocal(const LocalPose& pose)
{
    // The same composition Skeleton::evaluate() uses - the solver writes back
    // into localPose, so it has to rebuild world matrices exactly the way the
    // evaluate step would, or the pose it hands back would not match itself.
    return Math::translate(Math::mat4(1.0f), pose.position) * Math::mat4_cast(pose.rotation) *
           Math::scale(Math::mat4(1.0f), pose.scale);
}

bool decomposeLocal(const Math::mat4& matrix, LocalPose& out)
{
    Math::vec3 skew;
    Math::vec4 perspective;
    if (!Math::decompose(matrix, out.scale, out.rotation, out.position, skew, perspective))
        return false;
    out.rotation = Math::normalize(out.rotation);
    return true;
}

Math::vec3 translationOf(const Math::mat4& matrix)
{
    return Math::vec3(matrix[3]);
}

// acos of the dot product, clamped - the reference's XMScalarACos does the
// clamp itself, and without it a dot product a hair past 1.0 from rounding
// returns NaN and poisons the whole chain.
f32 angleBetweenNormals(const Math::vec3& a, const Math::vec3& b)
{
    return std::acos(Math::clamp(Math::dot(a, b), -1.0f, 1.0f));
}

} // namespace

IKConstraint IKConstraint::thigh()
{
    // wiScene.cpp:3486-3491, verbatim.
    IKConstraint constraint;
    constraint.enabled = true;
    constraint.minimum = Math::vec3(Math::pi<f32>() * 0.6f, Math::pi<f32>() * 0.1f,
                                   Math::pi<f32>() * 0.1f);
    constraint.maximum = Math::vec3(Math::pi<f32>() * 0.1f, Math::pi<f32>() * 0.1f,
                                   Math::pi<f32>() * 0.1f);
    return constraint;
}

IKConstraint IKConstraint::knee()
{
    // wiScene.cpp:3492-3497, verbatim: zero on two axes is what makes a knee
    // a hinge instead of a ball joint.
    IKConstraint constraint;
    constraint.enabled = true;
    constraint.minimum = Math::vec3(0.0f);
    constraint.maximum = Math::vec3(Math::pi<f32>() * 0.8f, 0.0f, 0.0f);
    return constraint;
}

IKConstraint IKConstraint::inverted() const
{
    IKConstraint flipped = *this;
    flipped.minimum = maximum;
    flipped.maximum = minimum;
    return flipped;
}

void IKSolver::solve(const Skeleton& skeleton, const IKChain& chain,
                     const Math::mat4& ownerTransform, std::vector<LocalPose>& localPose,
                     std::vector<Math::mat4>& globalPose)
{
    const u32 boneCount = skeleton.boneCount();
    if (!chain.enabled || chain.tipBone < 0 || static_cast<u32>(chain.tipBone) >= boneCount)
        return;
    if (localPose.size() != boneCount || globalPose.size() != boneCount)
        return;
    if (chain.length == 0 || chain.iterations == 0)
        return;

    // The pose lives in the owner's space, the target arrives in world space -
    // one conversion here rather than at every call site.
    const Math::mat4 toPoseSpace = Math::inverse(ownerTransform);
    const Math::vec3 target = Math::vec3(toPoseSpace * Math::vec4(chain.target, 1.0f));

    const u32 linkCount = Math::min(chain.length, IKChain::MaxLinks);
    s32 stack[IKChain::MaxLinks] = {};

    for (u32 iteration = 0; iteration < chain.iterations; ++iteration)
    {
        s32 childBone = chain.tipBone;
        s32 parentBone = skeleton.bone(static_cast<u32>(chain.tipBone)).parent;

        for (u32 link = 0; link < linkCount; ++link)
        {
            stack[link] = childBone;

            // No parent left to rotate: the chain root, the reference's own
            // "chain root reached, exit".
            if (parentBone < 0 || static_cast<u32>(parentBone) >= boneCount)
                break;

            const Math::mat4& parentWorld = globalPose[static_cast<usize>(parentBone)];
            const Math::vec3 parentPosition = translationOf(parentWorld);

            // The TIP, not the current child - CCD always aims the end
            // effector at the target, whichever link is being rotated.
            const Math::vec3 tipPosition =
                translationOf(globalPose[static_cast<usize>(chain.tipBone)]);

            const Math::vec3 toTip = tipPosition - parentPosition;
            const Math::vec3 toTarget = target - parentPosition;
            if (Math::dot(toTip, toTip) <= 1e-12f || Math::dot(toTarget, toTarget) <= 1e-12f)
                break; // the tip sits on the joint: no direction to rotate along
            const Math::vec3 directionToTip = Math::normalize(toTip);
            const Math::vec3 directionToTarget = Math::normalize(toTarget);

            const IKConstraint& constraint = chain.constraints[link];
            Math::quat rotation;
            if (constraint.enabled)
            {
                // Constrained: one rotation PER AXIS rather than a single
                // shortest one, each limited and each divided by the
                // iteration count so the limit is a total spread over all
                // the passes, not a per-pass allowance.
                rotation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
                Math::mat4 axisFrame = parentWorld;
                const f32 iterationReciprocal = 1.0f / static_cast<f32>(chain.iterations);
                for (u32 axisIndex = 0; axisIndex < 3; ++axisIndex)
                {
                    Math::vec3 localAxis(0.0f);
                    localAxis[static_cast<int>(axisIndex)] = 1.0f;
                    const f32 axisMinimum = constraint.minimum[static_cast<int>(axisIndex)] *
                                            iterationReciprocal;
                    const f32 axisMaximum = constraint.maximum[static_cast<int>(axisIndex)] *
                                            iterationReciprocal;

                    const Math::vec3 axisWorld = Math::mat3(axisFrame) * localAxis;
                    if (Math::dot(axisWorld, axisWorld) <= 1e-12f)
                        continue;
                    const Math::vec3 axis = Math::normalize(axisWorld);

                    // Both directions flattened onto the plane the axis is
                    // normal to: what is left is the part of the rotation
                    // this axis is allowed to answer for.
                    const Math::vec3 flatTip =
                        directionToTip - axis * Math::dot(axis, directionToTip);
                    const Math::vec3 flatTarget =
                        directionToTarget - axis * Math::dot(axis, directionToTarget);
                    if (Math::dot(flatTip, flatTip) <= 1e-12f ||
                        Math::dot(flatTarget, flatTarget) <= 1e-12f)
                        continue;
                    const Math::vec3 projectedTip = Math::normalize(flatTip);
                    const Math::vec3 projectedTarget = Math::normalize(flatTarget);

                    f32 angle = angleBetweenNormals(projectedTip, projectedTarget);
                    if (Math::dot(Math::cross(projectedTip, projectedTarget), axis) < 0.0f)
                        angle = Math::two_pi<f32>() - Math::min(angle, axisMinimum);
                    else
                        angle = Math::min(angle, axisMaximum);

                    const Math::quat axisRotation = Math::normalize(Math::angleAxis(angle, axis));
                    // The reference's own order (`W = R(Q1) * W`, row-vector,
                    // so R applies before W) written for Mathc's column-vector
                    // convention, where that same order is `W * R`. Left as
                    // the reference has it on purpose: it reads like a
                    // local-space rotation where a world one would be
                    // expected, and changing it would change the result.
                    axisFrame = axisFrame * Math::mat4_cast(axisRotation);
                    rotation = rotation * axisRotation;
                }
                rotation = Math::normalize(rotation);
            }
            else
            {
                // Unconstrained: the shortest rotation that takes the tip
                // direction onto the target direction.
                const Math::vec3 axis = Math::cross(directionToTip, directionToTarget);
                if (Math::dot(axis, axis) <= 1e-12f)
                    break; // already aligned, or exactly opposed
                rotation = Math::normalize(
                    Math::angleAxis(angleBetweenNormals(directionToTip, directionToTarget),
                                   Math::normalize(axis)));
            }

            // Rotate the parent about its OWN position, keeping its
            // translation and scale: what the reference gets out of
            // ApplyTransform() + Rotate() + UpdateTransform(). Mathc's product
            // order is reversed from the reference's XMQuaternionMultiply
            // (which reads "local first, then Q"), so the world rotation
            // goes on the left here.
            Math::vec3 parentScale;
            Math::quat parentRotation;
            Math::vec3 parentTranslation;
            Math::vec3 skew;
            Math::vec4 perspective;
            if (!Math::decompose(parentWorld, parentScale, parentRotation, parentTranslation, skew,
                                perspective))
                break;
            parentRotation = Math::normalize(rotation * Math::normalize(parentRotation));

            const Math::mat4 rotatedWorld = Math::translate(Math::mat4(1.0f), parentTranslation) *
                                           Math::mat4_cast(parentRotation) *
                                           Math::scale(Math::mat4(1.0f), parentScale);

            // Back to local, against the grandparent - the reference's
            // MatrixTransform(inverse(parent_of_parent.world)) step, which it
            // skips when the parent is a root.
            const s32 grandParent = skeleton.bone(static_cast<u32>(parentBone)).parent;
            const Math::mat4 newLocal =
                grandParent >= 0
                    ? Math::inverse(globalPose[static_cast<usize>(grandParent)]) * rotatedWorld
                    : rotatedWorld;

            LocalPose updated;
            if (!decomposeLocal(newLocal, updated))
                break;
            localPose[static_cast<usize>(parentBone)] = updated;
            globalPose[static_cast<usize>(parentBone)] = rotatedWorld;

            // Push the change back down every link already traversed, from
            // the rotated parent to the tip, so the next link up measures
            // against a pose that is consistent again.
            s32 propagationParent = parentBone;
            for (s32 index = static_cast<s32>(link); index >= 0; --index)
            {
                const s32 bone = stack[index];
                globalPose[static_cast<usize>(bone)] =
                    globalPose[static_cast<usize>(propagationParent)] *
                    composeLocal(localPose[static_cast<usize>(bone)]);
                propagationParent = bone;
            }

            if (grandParent < 0)
                break; // chain root reached

            childBone = parentBone;
            parentBone = grandParent;
        }
    }
}

const std::string& AnimationClip::name() const
{
    return mName;
}
void AnimationClip::setName(const std::string& name)
{
    mName = name;
}
f32 AnimationClip::duration() const
{
    return mDuration;
}
void AnimationClip::setDuration(f32 duration)
{
    mDuration = Math::max(duration, 0.0f);
}
std::vector<BoneTrack>& AnimationClip::tracks()
{
    return mTracks;
}
const std::vector<BoneTrack>& AnimationClip::tracks() const
{
    return mTracks;
}

void AnimationClip::sample(f32 time, std::vector<LocalPose>& pose) const
{
    for (const BoneTrack& track : mTracks)
    {
        if (track.bone < 0 || static_cast<usize>(track.bone) >= pose.size() || track.times.empty())
            continue;
        const usize count = Math::min(
            track.times.size(), Math::min(track.positions.size(),
                                         Math::min(track.rotations.size(), track.scales.size())));
        if (count == 0)
            continue;
        LocalPose& result = pose[static_cast<usize>(track.bone)];
        if (count == 1)
        {
            result.position = track.positions[0];
            result.rotation = Math::normalize(track.rotations[0]);
            result.scale = track.scales[0];
            continue;
        }
        usize next = static_cast<usize>(
            std::upper_bound(track.times.begin(), track.times.begin() + count, time) -
            track.times.begin());
        if (next == 0)
            next = 1;
        if (next >= count)
            next = count - 1;
        const usize previous = next > 0 ? next - 1 : 0;
        const f32 span = track.times[next] - track.times[previous];
        const f32 amount =
            span > 0.000001f ? Math::clamp((time - track.times[previous]) / span, 0.0f, 1.0f) : 0.0f;
        result.position = Math::mix(track.positions[previous], track.positions[next], amount);
        result.rotation = nlerp(track.rotations[previous], track.rotations[next], amount);
        result.scale = Math::mix(track.scales[previous], track.scales[next], amount);
    }
}

void AnimationClip::setKeyframe(s32 bone, f32 time, const LocalPose& pose)
{
    if (bone < 0)
        return;

    BoneTrack* track = nullptr;
    for (BoneTrack& candidate : mTracks)
        if (candidate.bone == bone)
        {
            track = &candidate;
            break;
        }
    if (!track)
    {
        mTracks.push_back(BoneTrack());
        track = &mTracks.back();
        track->bone = bone;
    }

    const usize index = static_cast<usize>(
        std::lower_bound(track->times.begin(), track->times.end(), time) - track->times.begin());
    if (index < track->times.size() && Math::abs(track->times[index] - time) < 0.0001f)
    {
        track->positions[index] = pose.position;
        track->rotations[index] = pose.rotation;
        track->scales[index] = pose.scale;
    }
    else
    {
        track->times.insert(track->times.begin() + static_cast<ptrdiff_t>(index), time);
        track->positions.insert(track->positions.begin() + static_cast<ptrdiff_t>(index),
                                pose.position);
        track->rotations.insert(track->rotations.begin() + static_cast<ptrdiff_t>(index),
                                pose.rotation);
        track->scales.insert(track->scales.begin() + static_cast<ptrdiff_t>(index), pose.scale);
    }
    mDuration = Math::max(mDuration, time);
}

void AnimationClip::removeKeyframe(s32 bone, f32 time)
{
    for (BoneTrack& track : mTracks)
    {
        if (track.bone != bone)
            continue;
        const usize index = static_cast<usize>(
            std::lower_bound(track.times.begin(), track.times.end(), time) - track.times.begin());
        if (index >= track.times.size() || Math::abs(track.times[index] - time) >= 0.0001f)
            return;
        track.times.erase(track.times.begin() + static_cast<ptrdiff_t>(index));
        track.positions.erase(track.positions.begin() + static_cast<ptrdiff_t>(index));
        track.rotations.erase(track.rotations.begin() + static_cast<ptrdiff_t>(index));
        track.scales.erase(track.scales.begin() + static_cast<ptrdiff_t>(index));
        return;
    }
}

void AnimationLayer::play(const std::string& clip, PlayMode mode, f32 blendTime)
{
    if (mCurrentName == clip && mMode == mode)
        return;
    mPrevious = mCurrent;
    mPreviousTime = mTime;
    mPreviousMode = mMode;
    mCurrent = nullptr;
    mCurrentName = clip;
    mReturnTo.clear();
    mTime = 0.0f;
    mMode = mode;
    mBlendDuration = Math::max(blendTime, 0.0f);
    mBlend = mBlendDuration > 0.0f ? 0.0f : 1.0f;
}

void AnimationLayer::crossFade(const std::string& clip, f32 duration)
{
    play(clip, mMode, duration);
}
void AnimationLayer::playOneShot(const std::string& clip, const std::string& returnTo,
                                 f32 blendTime)
{
    play(clip, PlayMode::Once, blendTime);
    mReturnTo = returnTo;
}
void AnimationLayer::stop()
{
    mCurrent = mPrevious = nullptr;
    mCurrentName.clear();
    mReturnTo.clear();
    mPaused = false;
}
void AnimationLayer::setPaused(bool paused)
{
    mPaused = paused;
}
bool AnimationLayer::paused() const
{
    return mPaused;
}
void AnimationLayer::setSpeed(f32 speed)
{
    mSpeed = std::isfinite(speed) ? speed : 1.0f;
}
void AnimationLayer::setMask(const std::vector<f32>& weights)
{
    mMask = weights;
}
void AnimationLayer::maskAll(const Skeleton& skeleton, f32 weight)
{
    mMask.assign(skeleton.boneCount(), Math::clamp(weight, 0.0f, 1.0f));
}

void AnimationLayer::maskFromBone(const Skeleton& skeleton, const char* rootBone, f32 weight)
{
    const s32 root = skeleton.findBone(rootBone);
    if (root < 0)
        return;
    if (mMask.size() != skeleton.boneCount())
        mMask.assign(skeleton.boneCount(), 0.0f);
    for (u32 i = 0; i < skeleton.boneCount(); ++i)
        for (s32 boneIndex = static_cast<s32>(i); boneIndex >= 0;
             boneIndex = skeleton.bone(static_cast<u32>(boneIndex)).parent)
            if (boneIndex == root)
            {
                mMask[i] = Math::clamp(weight, 0.0f, 1.0f);
                break;
            }
}

bool AnimationLayer::isPlaying(const std::string& clip) const
{
    return mCurrentName == clip;
}
const std::string& AnimationLayer::current() const
{
    return mCurrentName;
}
f32 AnimationLayer::time() const
{
    return mTime;
}
f32 AnimationLayer::duration() const
{
    return mCurrent ? mCurrent->duration() : 0.0f;
}
f32 AnimationLayer::normalizedTime() const
{
    return duration() > 0.0f ? mTime / duration() : 0.0f;
}
f32 AnimationLayer::wrappedTime() const
{
    const f32 length = duration();
    if (!(length > 0.0f))
        return 0.0f;
    if (mMode == PlayMode::Once)
        return Math::clamp(mTime, 0.0f, length);
    const f32 cycle = mMode == PlayMode::PingPong ? length * 2.0f : length;
    f32 wrapped = std::fmod(mTime, cycle);
    if (wrapped < 0.0f)
        wrapped += cycle;
    return mMode == PlayMode::PingPong && wrapped > length ? cycle - wrapped : wrapped;
}
bool AnimationLayer::finished() const
{
    return mCurrent && mMode == PlayMode::Once && mTime >= duration();
}
void AnimationLayer::seek(f32 time)
{
    mTime = Math::max(time, 0.0f);
    mPrevious = nullptr;
    mBlend = 1.0f;
}

AnimationManager& AnimationManager::getSingleton()
{
    static AnimationManager manager;
    return manager;
}

AnimationSetHandle AnimationManager::create(const Skeleton& skeleton,
                                            const std::vector<AnimationClip>& clips)
{
    AnimationSet set;
    set.skeleton = skeleton;
    set.clips = clips;
    return mSets.add(set);
}

namespace
{
std::string animationSetCacheKey(const std::string& skeletonFile,
                                 const std::vector<std::string>& animationFiles)
{
    std::string key = skeletonFile;
    for (const std::string& file : animationFiles)
    {
        key += '|';
        key += file;
    }
    return key;
}
} // namespace

bool AnimationManager::destroy(AnimationSetHandle handle)
{
    AnimationSet removed;
    if (!mSets.remove(handle, removed))
        return false;
    // The pool recycles the slot, so a stale entry left behind would end up
    // naming whatever set lands there next.
    const auto entry = mSourceByHandle.find(packHandle(handle));
    if (entry != mSourceByHandle.end())
    {
        mLoadedByKey.erase(animationSetCacheKey(entry->second.skeletonFile,
                                               entry->second.animationFiles));
        mSourceByHandle.erase(entry);
    }
    return true;
}

const AnimationSet* AnimationManager::get(AnimationSetHandle handle) const
{
    return mSets.get(handle);
}

void AnimationManager::clear()
{
    mSets.clear();
    mLoadedByKey.clear();
    mSourceByHandle.clear();
}

AnimationSetHandle AnimationManager::loadFromFiles(const std::string& skeletonFile,
                                                   const std::vector<std::string>& animationFiles)
{
    const std::string key = animationSetCacheKey(skeletonFile, animationFiles);
    const auto cached = mLoadedByKey.find(key);
    if (cached != mLoadedByKey.end() && mSets.get(cached->second))
        return cached->second;

    Skeleton skeleton;
    if (!RadionSkeletonIO::loadSkeleton(skeletonFile, skeleton))
        return AnimationSetHandle();

    std::vector<AnimationClip> clips;
    clips.reserve(animationFiles.size());
    for (const std::string& file : animationFiles)
    {
        AnimationClip clip;
        if (!RadionSkeletonIO::loadAnimation(file, skeleton, clip))
            return AnimationSetHandle();
        clips.push_back(std::move(clip));
    }

    warnOnHipsHeightMismatch(skeletonFile, skeleton, animationFiles, clips);

    const AnimationSetHandle handle = create(skeleton, clips);
    if (!handle.valid())
        return handle;
    mLoadedByKey[key] = handle;
    mSourceByHandle[packHandle(handle)] = FileSource{skeletonFile, animationFiles};
    return handle;
}

const std::string& AnimationManager::skeletonSourceFile(AnimationSetHandle handle) const
{
    static const std::string empty;
    const auto entry = mSourceByHandle.find(packHandle(handle));
    return entry != mSourceByHandle.end() ? entry->second.skeletonFile : empty;
}

const std::vector<std::string>& AnimationManager::animationSourceFiles(AnimationSetHandle handle) const
{
    static const std::vector<std::string> empty;
    const auto entry = mSourceByHandle.find(packHandle(handle));
    return entry != mSourceByHandle.end() ? entry->second.animationFiles : empty;
}

AnimationManager& Animations()
{
    return AnimationManager::getSingleton();
}

namespace
{

bool readMatrix(AssetFormat::Reader& reader, Math::mat4& matrix)
{
    for (u32 column = 0; column < 4; ++column)
        for (u32 row = 0; row < 4; ++row)
            if (!reader.readF32(matrix[column][row]))
                return false;
    return true;
}

void writeMatrix(AssetFormat::Writer& writer, const Math::mat4& matrix)
{
    for (u32 column = 0; column < 4; ++column)
        for (u32 row = 0; row < 4; ++row)
            writer.writeF32(matrix[column][row]);
}

} // namespace

bool RadionSkeletonIO::loadSkeleton(const std::string& filename, Skeleton& skeleton)
{
    ByteArray data = FileSystem::getSingleton().readBinary(filename);
    if (data.empty())
        return false;
    AssetFormat::Reader reader(data);
    if (!reader.header(AssetFormat::SkeletonMagic))
        return false;
    Skeleton loaded;
    AssetFormat::ChunkHeader chunk;
    bool found = false;
    while (reader.remaining() >= 12 && reader.next(chunk))
    {
        if (!reader.enter(chunk))
            return false;
        if (chunk.id == AssetFormat::Bones)
        {
            u32 count = 0;
            if (!reader.readU32(count) || count == 0 || count > 65535)
                return false;
            for (u32 i = 0; i < count; ++i)
            {
                std::string name;
                s32 parent = -1;
                Math::mat4 local(1.0f), inverse(1.0f);
                if (!reader.string(name) || !reader.readS32(parent) || !readMatrix(reader, local) ||
                    !readMatrix(reader, inverse) || !loaded.addBone(name, parent, local, inverse))
                    return false;
            }
            found = true;
        }
        reader.leave();
    }
    if (!found || !loaded.finalize())
        return false;
    skeleton = std::move(loaded);
    return true;
}

bool RadionSkeletonIO::saveSkeleton(const std::string& filename, const Skeleton& skeleton)
{
    if (skeleton.empty())
        return false;
    ByteArray data;
    AssetFormat::Writer writer(data);
    writer.header(AssetFormat::SkeletonMagic);
    const u64 chunk = writer.beginChunk(AssetFormat::Bones);
    writer.writeU32(skeleton.boneCount());
    for (u32 i = 0; i < skeleton.boneCount(); ++i)
    {
        const Bone& bone = skeleton.bone(i);
        writer.string(bone.name);
        writer.writeS32(bone.parent);
        writeMatrix(writer, bone.bindLocal);
        writeMatrix(writer, bone.inverseBind);
    }
    writer.endChunk(chunk);
    return FileSystem::getSingleton().writeBinary(filename, data);
}

bool RadionSkeletonIO::loadAnimation(const std::string& filename, const Skeleton& skeleton,
                                     AnimationClip& clip)
{
    ByteArray data = FileSystem::getSingleton().readBinary(filename);
    if (data.empty())
        return false;
    AssetFormat::Reader reader(data);
    if (!reader.header(AssetFormat::AnimationMagic))
        return false;
    AnimationClip loaded;
    AssetFormat::ChunkHeader chunk;
    bool found = false;
    while (reader.remaining() >= 12 && reader.next(chunk))
    {
        if (!reader.enter(chunk))
            return false;
        if (chunk.id == AssetFormat::Clip)
        {
            std::string clipName;
            f32 duration = 0.0f;
            u32 trackCount = 0;
            if (!reader.string(clipName) || !reader.readF32(duration) ||
                !reader.readU32(trackCount) || trackCount > skeleton.boneCount())
                return false;
            loaded.setName(clipName);
            loaded.setDuration(duration);
            loaded.tracks().reserve(trackCount);
            for (u32 trackIndex = 0; trackIndex < trackCount; ++trackIndex)
            {
                std::string boneName;
                u32 keyCount = 0;
                if (!reader.string(boneName) || !reader.readU32(keyCount) || keyCount == 0 ||
                    keyCount > 1000000)
                    return false;
                BoneTrack track;
                track.bone = skeleton.findBone(boneName.c_str());
                if (track.bone < 0)
                    return false;
                track.times.resize(keyCount);
                track.positions.resize(keyCount);
                track.rotations.resize(keyCount);
                track.scales.resize(keyCount);
                for (u32 key = 0; key < keyCount; ++key)
                {
                    Math::vec3& p = track.positions[key];
                    Math::quat& q = track.rotations[key];
                    Math::vec3& s = track.scales[key];
                    if (!reader.readF32(track.times[key]) || !reader.readF32(p.x) ||
                        !reader.readF32(p.y) || !reader.readF32(p.z) || !reader.readF32(q.x) ||
                        !reader.readF32(q.y) || !reader.readF32(q.z) || !reader.readF32(q.w) ||
                        !reader.readF32(s.x) || !reader.readF32(s.y) || !reader.readF32(s.z))
                        return false;
                    q = Math::normalize(q);
                }
                loaded.tracks().push_back(std::move(track));
            }
            found = true;
        }
        reader.leave();
    }
    if (!found)
        return false;
    clip = std::move(loaded);
    return true;
}

bool RadionSkeletonIO::saveAnimation(const std::string& filename, const Skeleton& skeleton,
                                     const AnimationClip& clip)
{
    ByteArray data;
    AssetFormat::Writer writer(data);
    writer.header(AssetFormat::AnimationMagic);
    const u64 chunk = writer.beginChunk(AssetFormat::Clip);
    writer.string(clip.name());
    writer.writeF32(clip.duration());
    writer.writeU32(static_cast<u32>(clip.tracks().size()));
    for (const BoneTrack& track : clip.tracks())
    {
        if (track.bone < 0 || static_cast<u32>(track.bone) >= skeleton.boneCount())
            return false;
        writer.string(skeleton.bone(static_cast<u32>(track.bone)).name);
        const u32 count = static_cast<u32>(track.times.size());
        writer.writeU32(count);
        for (u32 key = 0; key < count; ++key)
        {
            writer.writeF32(track.times[key]);
            const Math::vec3& p = track.positions[key];
            const Math::quat& q = track.rotations[key];
            const Math::vec3& s = track.scales[key];
            writer.writeF32(p.x);
            writer.writeF32(p.y);
            writer.writeF32(p.z);
            writer.writeF32(q.x);
            writer.writeF32(q.y);
            writer.writeF32(q.z);
            writer.writeF32(q.w);
            writer.writeF32(s.x);
            writer.writeF32(s.y);
            writer.writeF32(s.z);
        }
    }
    writer.endChunk(chunk);
    return FileSystem::getSingleton().writeBinary(filename, data);
}

} // namespace Radion
