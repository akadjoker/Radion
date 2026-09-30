#ifndef RADION_SKELETON_H
#define RADION_SKELETON_H

#include "Containers.h"
#include "Types.h"

#include "Math.h"
#include <string>
#include <vector>

namespace Radion
{

struct LocalPose
{
    Math::vec3 position = Math::vec3(0.0f);
    Math::quat rotation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    Math::vec3 scale = Math::vec3(1.0f);
};

struct Bone
{
    std::string name;
    s32 parent = -1;
    Math::mat4 bindLocal = Math::mat4(1.0f);
    Math::mat4 inverseBind = Math::mat4(1.0f);
};

class Skeleton
{
public:
    bool empty() const;
    u32 boneCount() const;
    const Bone& bone(u32 index) const;
    s32 findBone(const char* name) const;

    bool addBone(const std::string& name, s32 parent, const Math::mat4& bindLocal,
                 const Math::mat4& inverseBind);
    bool finalize();
    void bindPose(std::vector<LocalPose>& pose) const;
    void evaluate(const std::vector<LocalPose>& localPose, std::vector<Math::mat4>& globalPose,
                  std::vector<Math::mat4>& palette) const;

private:
    std::vector<Bone> mBones;
    std::vector<u16> mOrder;
};

// Per-axis rotation limits (radians) for one IK link; a knee has min = max = 0 on two axes so it bends on one.
struct IKConstraint
{
    bool enabled = false;
    Math::vec3 minimum = Math::vec3(0.0f);
    Math::vec3 maximum = Math::vec3(0.0f);

    // The reference's own numbers, verbatim; the bone-to-preset mapping is the caller's job (see IKSolver).
    static IKConstraint thigh();
    static IKConstraint knee();

    // For skeletons whose knee bends the other way: swaps min with max.
    IKConstraint inverted() const;
};

// One CCD chain: the tip bone is pulled toward the target by rotating up to `length` parents.
struct IKChain
{
    // Fixed 32-link traversal stack, no allocation; longer chains are clamped.
    static constexpr u32 MaxLinks = 32;

    s32 tipBone = -1;

    // WORLD space; the solver converts it into pose space once, so callers (e.g. foot-placement raycasts) need not.
    Math::vec3 target = Math::vec3(0.0f);

    u32 length = 2;         // how many parents up from the tip
    u32 iterations = 10;    // CCD passes
    bool enabled = true;

    // Index 0 constrains the tip's parent, 1 its grandparent, etc.; a disabled link takes the plain shortest rotation.
    IKConstraint constraints[MaxLinks];
};

class IKSolver
{
public:
    // Rewrites localPose and globalPose in place; globalPose must already match Skeleton::evaluate() of localPose.
    static void solve(const Skeleton& skeleton, const IKChain& chain,
                      const Math::mat4& ownerTransform, std::vector<LocalPose>& localPose,
                      std::vector<Math::mat4>& globalPose);
};

struct BoneTrack
{
    s32 bone = -1;
    std::vector<f32> times;
    std::vector<Math::vec3> positions;
    std::vector<Math::quat> rotations;
    std::vector<Math::vec3> scales;
};

// One named moment in a clip; time in seconds from the clip start (as AnimationClip::duration()).
struct AnimationEvent
{
    f32 time = 0.0f;
    std::string name;
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

    // Authoring: writes one bone's pose at `time`, creating the track if needed and overwriting a key at (nearly) the same time. Keeps track times sorted (sample()'s upper_bound needs it) and extends duration() to cover `time`.
    void setKeyframe(s32 bone, f32 time, const LocalPose& pose);
    // No-op if `bone` has no track or no key within epsilon of `time`.
    void removeKeyframe(s32 bone, f32 time);

    // A named moment in the clip (footstep, weapon fire); belongs to the clip, not the player. Kept sorted by time so a layer fires all crossed events in one walk.
    void addEvent(f32 time, const std::string& name);
    void removeEvent(const std::string& name);
    void clearEvents();
    const std::vector<AnimationEvent>& events() const;

private:
    std::string mName;
    f32 mDuration = 0.0f;
    std::vector<BoneTrack> mTracks;
    std::vector<AnimationEvent> mEvents;
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

    // Loads Radion's .rskel/.ranim files into an AnimationSetHandle, cached by the exact (skeleton, clips) pair so repeated bindings decode once. Lets a saved scene restore an animation set by name.
    // Returns an invalid handle if the skeleton or any clip fails to load.
    AnimationSetHandle loadFromFiles(const std::string& skeletonFile,
                                     const std::vector<std::string>& animationFiles);

    // The files loadFromFiles() cached `handle` under; empty for create()-made, invalid or stale handles.
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
    // Freezes time() (and any crossfade) where it stands; unlike stop() the clip stays selected. For scrub bars: without it update() advances past where a drag left it.
    void setPaused(bool paused);
    bool paused() const;

    // Events crossed by this frame's advance, in order. Polled after Animator::update() rather than a callback, which would run user code with the pose half-built.
    // Cleared each update(). A loop wrap fires the clip tail then head; a long frame fires every skipped event, in order.
    const std::vector<const AnimationEvent*>& firedEvents() const;
    void setSpeed(f32 speed);
    void setMask(const std::vector<f32>& weights);
    void maskFromBone(const Skeleton& skeleton, const char* rootBone, f32 weight = 1.0f);
    void maskAll(const Skeleton& skeleton, f32 weight = 1.0f);

    bool isPlaying(const std::string& clip) const;
    const std::string& current() const;
    // Raw elapsed seconds since play()/crossFade(); never wraps. For blend timing; use wrappedTime() for the position in the clip.
    f32 time() const;
    f32 duration() const;
    f32 normalizedTime() const;
    // time() folded into [0, duration) the way Once/Loop/PingPong sample the clip; for scrub bars and time-remaining readouts.
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
    // Points into the clip's event list, which outlives the frame (the bound AnimationSet owns it).
    std::vector<const AnimationEvent*> mFiredEvents;
};

// Radion's skeleton/animation file format; counterpart to RadionMeshImporter.
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
