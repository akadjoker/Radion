#ifndef RADION_PATH_ANIMATOR_H
#define RADION_PATH_ANIMATOR_H

#include "Component.h"

#include "Math.h"
#include "Math.h"
#include <vector>

namespace Radion
{

// One full transform keyframe on a path, at an explicit time rather than an
// index - legs do not have to be equal length. Ordered by `time` as they
// are added. Rotation and scale default to identity/one, so a track that
// only ever calls addKeyframe(time, position) still behaves exactly like a
// position-only path.
struct PathKeyframe
{
    f32 time = 0.0f;
    Math::vec3 position = Math::vec3(0.0f);
    Math::quat rotation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    Math::vec3 scale = Math::vec3(1.0f);
};

// What evaluate() hands back: a full transform, not just a point, so a
// PathAnimator can turn a camera to face along its own flight the same way
// it flies it.
struct PathPose
{
    Math::vec3 position = Math::vec3(0.0f);
    Math::quat rotation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    Math::vec3 scale = Math::vec3(1.0f);
};

// A named set of keyframes, built once and bound to as many PathAnimator
// components as need to fly the same route - shared the way an AnimationClip
// is. Always a closed loop: the track wraps from the last keyframe back to
// the first, so a caller that wants a true loop (first and last keyframe at
// the same pose, the way a patrol closes) adds that last point itself.
class PathTrack
{
public:
    void addKeyframe(f32 time, const Math::vec3& position,
                     const Math::quat& rotation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f),
                     const Math::vec3& scale = Math::vec3(1.0f));
    void clear();
    bool empty() const;
    usize keyframeCount() const;

    // The last keyframe's time - where the loop wraps back to the first.
    f32 duration() const;

    // Position on a Catmull-Rom through the closed loop, rotation slerped
    // and scale lerped between the two bracketing keyframes - at `time`,
    // wrapped into [0, duration()). Two keyframes just oscillates between
    // them; one holds still; none returns identity.
    PathPose evaluate(f32 time) const;

private:
    std::vector<PathKeyframe> mKeyframes;
};

// Plays a PathTrack onto its owner's transform, the way Animator plays a
// clip onto a skeleton - a light, a camera, anything a GameObject carries
// can be flown along a route without one-off spline code at every call site.
class PathAnimator final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::PathAnimator;

    void setTrack(const PathTrack* track);
    const PathTrack* track() const;

    void play(bool loop = true);
    void pause();
    bool playing() const;

    f32 time() const;
    void setTime(f32 time);
    void setSpeed(f32 speed);
    f32 speed() const;

private:
    friend class GameObject;

    PathAnimator();
    void onUpdate(f32 deltaTime) override;
    void apply();

    const PathTrack* mTrack = nullptr;
    f32 mTime = 0.0f;
    f32 mSpeed = 1.0f;
    bool mPlaying = false;
    bool mLoop = true;
};

} // namespace Radion

#endif // RADION_PATH_ANIMATOR_H
