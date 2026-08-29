#include "PCH.h"

#include "Animation.h"

#include "GameObject.h"

namespace Radion
{

namespace
{

f32 playbackTime(f32 time, f32 duration, PlayMode mode)
{
    if (!(duration > 0.0f))
        return 0.0f;
    if (mode == PlayMode::Once)
        return glm::clamp(time, 0.0f, duration);

    const f32 cycle = mode == PlayMode::PingPong ? duration * 2.0f : duration;
    f32 wrapped = std::fmod(time, cycle);
    if (wrapped < 0.0f)
        wrapped += cycle;
    return mode == PlayMode::PingPong && wrapped > duration ? cycle - wrapped : wrapped;
}

// Everything in [from, to) along the clip, appended in order. The interval
// is half-open at the end so an event exactly on the boundary fires once,
// on the frame that reaches it, and not again on the next.
void collectEvents(const AnimationClip& clip, f32 from, f32 to,
                   std::vector<const AnimationEvent*>& out)
{
    for (const AnimationEvent& event : clip.events())
        if (event.time >= from && event.time < to)
            out.push_back(&event);
}

// The events a single advance crossed, given where playback was and where it
// landed. Handles the two cases a naive `from < t < to` gets wrong: a loop
// that wrapped past the end (fire the tail, then the head), and a frame long
// enough to cover the whole clip more than once - which fires each event
// once rather than as many times as it lapped, because a stutter should not
// spawn ten footsteps.
void collectFiredEvents(const AnimationClip& clip, f32 previous, f32 current, PlayMode mode,
                        f32 advance, std::vector<const AnimationEvent*>& out)
{
    const f32 duration = clip.duration();
    if (clip.events().empty() || !(duration > 0.0f) || advance <= 0.0f)
        return;

    if (mode == PlayMode::Once || advance >= duration)
    {
        // Either it cannot wrap, or it covered everything: one pass over the
        // range, clamped to the clip.
        if (advance >= duration)
            collectEvents(clip, 0.0f, duration, out);
        else
            collectEvents(clip, previous, current, out);
        return;
    }

    if (current >= previous)
        collectEvents(clip, previous, current, out);
    else
    {
        // Wrapped: the tail of the clip, then the head of it.
        collectEvents(clip, previous, duration, out);
        collectEvents(clip, 0.0f, current, out);
    }
}

glm::quat nlerp(const glm::quat& from, glm::quat to, f32 amount)
{
    if (glm::dot(from, to) < 0.0f)
        to = -to;
    return glm::normalize(from * (1.0f - amount) + to * amount);
}

void applyClip(const AnimationClip* clip, f32 time, f32 weight, const std::vector<f32>& mask,
               std::vector<LocalPose>& pose, std::vector<LocalPose>& scratch)
{
    if (!clip || weight <= 0.001f)
        return;
    scratch = pose;
    clip->sample(time, scratch);
    for (usize i = 0; i < pose.size(); ++i)
    {
        const f32 amount = weight * (mask.empty() || i >= mask.size() ? 1.0f : mask[i]);
        if (amount <= 0.001f)
            continue;
        pose[i].position = glm::mix(pose[i].position, scratch[i].position, amount);
        pose[i].rotation = nlerp(pose[i].rotation, scratch[i].rotation, amount);
        pose[i].scale = glm::mix(pose[i].scale, scratch[i].scale, amount);
    }
}

} // namespace

Animator::Animator() : Component(Type)
{
}

void Animator::bind(AnimationSetHandle animations)
{
    mAnimations = animations;
    mLayers.clear();
    const AnimationSet* set = Animations().get(mAnimations);
    if (set)
    {
        set->skeleton.bindPose(mLocalPose);
        mScratch.resize(set->skeleton.boneCount());
        set->skeleton.evaluate(mLocalPose, mGlobalPose, mPalette);
        mPrevPalette = mPalette;
    }
    else
    {
        mLocalPose.clear();
        mScratch.clear();
        mGlobalPose.clear();
        mPalette.clear();
        mPrevPalette.clear();
    }
}

bool Animator::bound() const
{
    const AnimationSet* set = Animations().get(mAnimations);
    return set && !set->skeleton.empty();
}

AnimationSetHandle Animator::animationSet() const
{
    return mAnimations;
}
AnimationLayer& Animator::layer(u32 index)
{
    if (mLayers.size() <= index)
        mLayers.resize(index + 1);
    return mLayers[index];
}
u32 Animator::layerCount() const
{
    return static_cast<u32>(mLayers.size());
}
void Animator::play(const std::string& clip, PlayMode mode, f32 blendTime)
{
    layer(0).play(clip, mode, blendTime);
}

const AnimationClip* Animator::findClip(const std::string& name) const
{
    const AnimationSet* set = Animations().get(mAnimations);
    if (!set)
        return nullptr;
    for (const AnimationClip& clip : set->clips)
        if (clip.name() == name)
            return &clip;
    return nullptr;
}

void Animator::update(f32 deltaTime)
{
    const AnimationSet* set = Animations().get(mAnimations);
    if (!set || set->skeleton.empty())
        return;

    mPrevPalette = mPalette;

    if (!mPoseEditMode)
    {
        set->skeleton.bindPose(mLocalPose);
        const f32 dt = std::isfinite(deltaTime) ? glm::max(deltaTime, 0.0f) : 0.0f;
        for (AnimationLayer& layer : mLayers)
        {
            if (!layer.mCurrent && !layer.mCurrentName.empty())
                layer.mCurrent = findClip(layer.mCurrentName);
            if (!layer.mCurrent)
                continue;
            // Paused freezes advancement (time, crossfade) but not sampling -
            // seek()'s mTime write still shows up below, which is the whole
            // point: a scrub bar drags the frozen frame around instead of
            // the next update() immediately marching past it.
            const f32 layerDt = layer.mPaused ? 0.0f : dt;
            // Where playback stood before this frame moved it - the other
            // end of the interval the clip's events are tested against.
            const f32 timeBefore =
                playbackTime(layer.mTime, layer.mCurrent->duration(), layer.mMode);
            layer.mFiredEvents.clear();
            layer.mTime += layerDt * layer.mSpeed;
            if (layer.mBlendDuration > 0.0f && layer.mBlend < 1.0f)
                layer.mBlend = glm::min(layer.mBlend + layerDt / layer.mBlendDuration, 1.0f);
            if (layer.mBlend >= 1.0f)
                layer.mPrevious = nullptr;
            if (layer.finished() && !layer.mReturnTo.empty())
            {
                const std::string returnTo = layer.mReturnTo;
                layer.play(returnTo, PlayMode::Loop, 0.2f);
                layer.mCurrent = findClip(returnTo);
            }
            const f32 currentTime =
                playbackTime(layer.mTime, layer.mCurrent->duration(), layer.mMode);
            // After the return-to switch above, so a one-shot that just
            // handed over does not fire the clip it left behind.
            collectFiredEvents(*layer.mCurrent, timeBefore, currentTime, layer.mMode,
                               std::abs(layerDt * layer.mSpeed), layer.mFiredEvents);
            if (layer.mPrevious)
            {
                layer.mPreviousTime += layerDt * layer.mSpeed;
                applyClip(layer.mPrevious,
                         playbackTime(layer.mPreviousTime, layer.mPrevious->duration(),
                                      layer.mPreviousMode),
                         1.0f, layer.mMask, mLocalPose, mScratch);
            }
            applyClip(layer.mCurrent, currentTime, layer.mPrevious ? layer.mBlend : 1.0f,
                     layer.mMask, mLocalPose, mScratch);
        }
    }
    set->skeleton.evaluate(mLocalPose, mGlobalPose, mPalette);

    // IK last, on the finished pose, then evaluate again - the reference does
    // the same by setting recompute_hierarchy: it is a correction applied to a
    // pose that has already been built, not a step inside building it. The
    // second evaluate is also what rebuilds the skinning palette from the
    // corrected pose rather than the pre-IK one.
    if (!mIKChains.empty())
    {
        const glm::mat4 ownerTransform = owner() ? owner()->globalTransform() : glm::mat4(1.0f);
        bool solved = false;
        for (const IKChain& chain : mIKChains)
        {
            if (!chain.enabled)
                continue;
            IKSolver::solve(set->skeleton, chain, ownerTransform, mLocalPose, mGlobalPose);
            solved = true;
        }
        if (solved)
            set->skeleton.evaluate(mLocalPose, mGlobalPose, mPalette);
    }
}

u32 Animator::addIKChain(const IKChain& chain)
{
    mIKChains.push_back(chain);
    return static_cast<u32>(mIKChains.size() - 1);
}

IKChain* Animator::ikChain(u32 index)
{
    return index < mIKChains.size() ? &mIKChains[index] : nullptr;
}

u32 Animator::ikChainCount() const
{
    return static_cast<u32>(mIKChains.size());
}

void Animator::clearIKChains()
{
    mIKChains.clear();
}

void Animator::setPoseEditMode(bool enabled)
{
    mPoseEditMode = enabled;
}

bool Animator::poseEditMode() const
{
    return mPoseEditMode;
}

void Animator::setBoneLocalPose(u32 bone, const LocalPose& pose)
{
    if (bone < mLocalPose.size())
        mLocalPose[bone] = pose;
}

bool Animator::boneGlobalPosition(s32 bone, glm::vec3& out) const
{
    if (bone < 0 || static_cast<usize>(bone) >= mGlobalPose.size())
        return false;
    out = glm::vec3(mGlobalPose[static_cast<usize>(bone)][3]);
    return true;
}

const Skeleton* Animator::skeleton() const
{
    const AnimationSet* set = Animations().get(mAnimations);
    return set ? &set->skeleton : nullptr;
}
const std::vector<LocalPose>& Animator::localPose() const
{
    return mLocalPose;
}
const std::vector<glm::mat4>& Animator::globalPose() const
{
    return mGlobalPose;
}
const std::vector<glm::mat4>& Animator::palette() const
{
    return mPalette;
}

const std::vector<glm::mat4>& Animator::prevPalette() const
{
    return mPrevPalette;
}

} // namespace Radion
