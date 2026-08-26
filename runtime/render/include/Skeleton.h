#ifndef RADION_SKELETON_H
#define RADION_SKELETON_H

#include "Containers.h"
#include "Types.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <vector>

namespace Radion
{

struct LocalPose
{
    Math::Vec3 position = Math::Vec3(0.0f);
    Math::Quaternion rotation = Math::Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    Math::Vec3 scale = Math::Vec3(1.0f);
};

struct Bone
{
    std::string name;
    s32 parent = -1;
    Math::Mat4 bindLocal = Math::Mat4(1.0f);
    Math::Mat4 inverseBind = Math::Mat4(1.0f);
};

class Skeleton
{
public:
    bool empty() const;
    u32 boneCount() const;
    const Bone& bone(u32 index) const;
    s32 findBone(const char* name) const;

    bool addBone(const std::string& name, s32 parent, const Math::Mat4& bindLocal,
                 const Math::Mat4& inverseBind);
    bool finalize();
    void bindPose(std::vector<LocalPose>& pose) const;
    void evaluate(const std::vector<LocalPose>& localPose, std::vector<Math::Mat4>& globalPose,
                  std::vector<Math::Mat4>& palette) const;

private:
    std::vector<Bone> mBones;
    std::vector<u16> mOrder;
};

// ------------------------------------------------------------ inverse kinematics

// Per-axis rotation limits for one link of an IK chain, in radians. Only the
// axes that matter are non-zero: a knee has min = max = 0 on two of the three,
// which is what makes it bend on one axis only.
struct IKConstraint
{
    bool enabled = false;
    Math::Vec3 minimum = Math::Vec3(0.0f);
    Math::Vec3 maximum = Math::Vec3(0.0f);

    // The reference's own numbers, kept verbatim - see IKSolver's comment for
    // why the bone-to-preset mapping is the caller's job here and not a table
    // inside the solver.
    static IKConstraint thigh();
    static IKConstraint knee();

    // For skeletons whose knee bends the other way: the reference swaps min
    // with max rather than negating anything.
    IKConstraint inverted() const;
};

// One CCD chain. The tip bone is what gets pulled towards the target; the
// solver walks up `length` parents from it, rotating each to close the gap.
struct IKChain
{
    // The traversal stack is a fixed 32 links, no allocation - the same bound
    // the reference uses. A chain longer than this is clamped, not grown.
    static constexpr u32 MaxLinks = 32;

    s32 tipBone = -1;

    // WORLD space. The solver converts it into the pose's own space once,
    // through the owning transform - a foot-placement raycast gives a world
    // hit, so taking world here is what avoids every caller doing that
    // conversion by hand.
    Math::Vec3 target = Math::Vec3(0.0f);

    u32 length = 2;         // how many parents up from the tip
    u32 iterations = 10;    // CCD passes
    bool enabled = true;

    // Index 0 constrains the tip's own parent, 1 its grandparent, and so on -
    // the same order the solver walks. Left disabled, that link takes the
    // plain shortest rotation instead.
    IKConstraint constraints[MaxLinks];
};

class IKSolver
{
public:
    // Rewrites localPose and globalPose in place. globalPose must already be
    // what Skeleton::evaluate() produced for localPose, and stays consistent
    // with it on the way out.
    static void solve(const Skeleton& skeleton, const IKChain& chain,
                      const Math::Mat4& ownerTransform, std::vector<LocalPose>& localPose,
                      std::vector<Math::Mat4>& globalPose);
};

struct BoneTrack
{
    s32 bone = -1;
    std::vector<f32> times;
    std::vector<Math::Vec3> positions;
    std::vector<Math::Quaternion> rotations;
    std::vector<Math::Vec3> scales;
};

class AnimationClip
{
public:
    const std::string& name() const;
    void setName(const std::string& name);
    f32 duration() const;
    void setDuration(f32 duration);
    std::vector<BoneTrack>& tracks();
    const std::vector<BoneTrack>& tracks() const;
    void sample(f32 time, std::vector<LocalPose>& pose) const;

    // Authoring: writes one bone's pose at `time` into its track, creating
    // the track if this is the bone's first key, overwriting an existing key
    // at (nearly) the same time rather than adding a duplicate. Keeps every
    // track's times sorted ascending, which sample()'s upper_bound search
    // assumes. Extends duration() to cover `time` if it did not already.
    void setKeyframe(s32 bone, f32 time, const LocalPose& pose);
    // No-op if `bone` has no track or no key within epsilon of `time`.
    void removeKeyframe(s32 bone, f32 time);

private:
    std::string mName;
    f32 mDuration = 0.0f;
    std::vector<BoneTrack> mTracks;
};

struct AnimationSet
{
    Skeleton skeleton;
    std::vector<AnimationClip> clips;
};

using AnimationSetHandle = Handle<struct AnimationSetTag>;

class AnimationManager
{
public:
    static AnimationManager& getSingleton();

    AnimationSetHandle create(const Skeleton& skeleton, const std::vector<AnimationClip>& clips);
    bool destroy(AnimationSetHandle handle);
    const AnimationSet* get(AnimationSetHandle handle) const;
    void clear();

    // Radion's own skeleton + clip files (RadionSkeletonIO, the .rskel/.ranim
    // counterpart to .rmesh) -> AnimationSetHandle, cached by the exact
    // (skeleton, clips) pair so binding the same character to several
    // Animators (Sinbad on four GameObjects, e.g.) loads and decodes the
    // files once. What a saved scene's Animator component uses to restore
    // its animation set by name instead of a handle that means nothing next
    // run - the same role AssetManager::loadMesh() plays for meshes.
    // Returns an invalid handle if the skeleton or any clip fails to load.
    AnimationSetHandle loadFromFiles(const std::string& skeletonFile,
                                     const std::vector<std::string>& animationFiles);

    // The files loadFromFiles() cached `handle` under, or empty/none for a
    // handle created through create() directly (no files behind it) or an
    // invalid/stale one.
    const std::string& skeletonSourceFile(AnimationSetHandle handle) const;
    const std::vector<std::string>& animationSourceFiles(AnimationSetHandle handle) const;

private:
    struct FileSource
    {
        std::string skeletonFile;
        std::vector<std::string> animationFiles;
    };

    Pool<AnimationSet, AnimationSetHandle> mSets;
    HashMap<std::string, AnimationSetHandle> mLoadedByKey; // skeleton|clip,clip,... -> handle
    HashMap<u64, FileSource> mSourceByHandle;              // packHandle() -> files
};

AnimationManager& Animations();

class Animator;

enum class PlayMode : u8
{
    Loop,
    Once,
    PingPong
};

class AnimationLayer
{
public:
    void play(const std::string& clip, PlayMode mode = PlayMode::Loop, f32 blendTime = 0.2f);
    void crossFade(const std::string& clip, f32 duration = 0.2f);
    void playOneShot(const std::string& clip, const std::string& returnTo, f32 blendTime = 0.2f);
    void stop();
    // Freezes time() (and any crossfade in progress) exactly where it stands
    // - unlike stop(), the clip stays selected (current() keeps its name,
    // wrappedTime() keeps reading the held frame), which is what a scrub bar
    // wants: seek() always works regardless, but without this the very next
    // update() just keeps advancing past wherever a drag left it.
    void setPaused(bool paused);
    bool paused() const;
    void setSpeed(f32 speed);
    void setMask(const std::vector<f32>& weights);
    void maskFromBone(const Skeleton& skeleton, const char* rootBone, f32 weight = 1.0f);
    void maskAll(const Skeleton& skeleton, f32 weight = 1.0f);

    bool isPlaying(const std::string& clip) const;
    const std::string& current() const;
    // Raw elapsed seconds since play()/crossFade() - never wraps, keeps
    // climbing for as long as a Loop clip keeps looping. Right for
    // Animator::update()'s own blend-timing math; wrong for anything asking
    // "how far into the clip AM I RIGHT NOW" - use wrappedTime() for that.
    f32 time() const;
    f32 duration() const;
    f32 normalizedTime() const;
    // time(), folded back into [0, duration) the same way Once/Loop/PingPong
    // sample the clip - what a UI scrub bar or "time remaining" readout
    // wants, since time() alone reads as nonsense once a looping clip has
    // gone around more than once (16s into a clip that lasts 2s, e.g.).
    f32 wrappedTime() const;
    bool finished() const;
    void seek(f32 time);

private:
    friend class Animator;

    const AnimationClip* mCurrent = nullptr;
    const AnimationClip* mPrevious = nullptr;
    std::string mCurrentName;
    std::string mReturnTo;
    f32 mTime = 0.0f;
    f32 mPreviousTime = 0.0f;
    f32 mBlend = 1.0f;
    f32 mBlendDuration = 0.0f;
    f32 mSpeed = 1.0f;
    PlayMode mMode = PlayMode::Loop;
    PlayMode mPreviousMode = PlayMode::Loop;
    std::vector<f32> mMask;
    bool mPaused = false;
};

// Radion's own skeleton/animation file format - the counterpart to
// RadionMeshImporter, which owns mesh geometry the same way.
class RadionSkeletonIO
{
public:
    static bool loadSkeleton(const std::string& filename, Skeleton& skeleton);
    static bool saveSkeleton(const std::string& filename, const Skeleton& skeleton);
    static bool loadAnimation(const std::string& filename, const Skeleton& skeleton,
                              AnimationClip& clip);
    static bool saveAnimation(const std::string& filename, const Skeleton& skeleton,
                              const AnimationClip& clip);
};

} // namespace Radion

#endif // RADION_SKELETON_H
