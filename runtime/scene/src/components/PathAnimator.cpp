#include "PCH.h"

#include "PathAnimator.h"

#include "GameObject.h"

namespace Radion
{

void PathTrack::addKeyframe(f32 time, const Math::vec3& position, const Math::quat& rotation,
                            const Math::vec3& scale)
{
    mKeyframes.push_back({time, position, rotation, scale});
}

void PathTrack::clear()
{
    mKeyframes.clear();
}

bool PathTrack::empty() const
{
    return mKeyframes.empty();
}

usize PathTrack::keyframeCount() const
{
    return mKeyframes.size();
}

f32 PathTrack::duration() const
{
    return mKeyframes.empty() ? 0.0f : mKeyframes.back().time;
}

PathPose PathTrack::evaluate(f32 time) const
{
    const usize count = mKeyframes.size();
    if (count == 0)
        return PathPose();
    if (count == 1)
        return PathPose{mKeyframes[0].position, mKeyframes[0].rotation, mKeyframes[0].scale};

    const f32 length = duration();
    f32 t = length > 0.0f ? Math::mod(time, length) : 0.0f;
    if (t < 0.0f)
        t += length;

    // Linear scan for the bracketing pair - tracks are a handful of
    // keyframes (a light or camera flythrough), never dense enough to need
    // a binary search.
    usize segment = 0;
    while (segment + 1 < count && mKeyframes[segment + 1].time <= t)
        ++segment;

    const PathKeyframe& a = mKeyframes[segment];
    const PathKeyframe& b = mKeyframes[(segment + 1) % count];
    const f32 span = b.time - a.time;
    const f32 localT = span > 0.0f ? Math::clamp((t - a.time) / span, 0.0f, 1.0f) : 0.0f;

    // Position: Catmull-Rom through the closed loop, using the point before
    // `a` and the one after `b`, wrapping around the keyframe list either
    // way - the same shape a patrol or a camera flythrough closes with.
    const Math::vec3& p0 = mKeyframes[(segment + count - 1) % count].position;
    const Math::vec3& p1 = a.position;
    const Math::vec3& p2 = b.position;
    const Math::vec3& p3 = mKeyframes[(segment + 2) % count].position;

    const f32 t2 = localT * localT;
    const f32 t3 = t2 * localT;
    PathPose pose;
    pose.position = 0.5f * ((2.0f * p1) + (-p0 + p2) * localT +
                            (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                            (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);

    // Rotation and scale: plain slerp/lerp between the bracketing pair. A
    // spline through orientations needs its own (squad) blend to stay
    // smooth across a keyframe, which is more than this track set out to
    // do - two adjacent keyframes are usually close enough that slerp reads
    // the same as one.
    pose.rotation = Math::slerp(a.rotation, b.rotation, localT);
    pose.scale = Math::mix(a.scale, b.scale, localT);
    return pose;
}

PathAnimator::PathAnimator() : Component(Type, ComponentEventUpdate)
{
}

void PathAnimator::setTrack(const PathTrack* track)
{
    mTrack = track;
    mTime = 0.0f;
}

const PathTrack* PathAnimator::track() const
{
    return mTrack;
}

void PathAnimator::play(bool loop)
{
    mPlaying = true;
    mLoop = loop;
}

void PathAnimator::pause()
{
    mPlaying = false;
}

bool PathAnimator::playing() const
{
    return mPlaying;
}

f32 PathAnimator::time() const
{
    return mTime;
}

void PathAnimator::setTime(f32 time)
{
    mTime = time;
    apply();
}

void PathAnimator::setSpeed(f32 speed)
{
    mSpeed = speed;
}

f32 PathAnimator::speed() const
{
    return mSpeed;
}

void PathAnimator::onUpdate(f32 deltaTime)
{
    if (!mPlaying || !mTrack || mTrack->empty())
        return;

    const f32 length = mTrack->duration();
    mTime += deltaTime * mSpeed;
    if (length > 0.0f && mTime >= length)
    {
        if (mLoop)
            mTime = Math::mod(mTime, length);
        else
        {
            mTime = length;
            mPlaying = false;
        }
    }

    apply();
}

void PathAnimator::apply()
{
    if (!mTrack || mTrack->empty())
        return;
    const PathPose pose = mTrack->evaluate(mTime);
    owner()->setPosition(pose.position);
    owner()->setRotation(pose.rotation);
    owner()->setScale(pose.scale);
}

} // namespace Radion
