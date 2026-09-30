#ifndef RADION_PATH_ANIMATOR_H
#define RADION_PATH_ANIMATOR_H

#include "Component.h"

#include "Math.h"
#include <vector>

namespace Radion
{

// Explicit time, so legs need not be equal length.
struct PathKeyframe
{
    f32 time = 0.0f;
    Math::vec3 position = Math::vec3(0.0f);
    Math::quat rotation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    Math::vec3 scale = Math::vec3(1.0f);
};

struct PathPose
{
    Math::vec3 position = Math::vec3(0.0f);
    Math::quat rotation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f);
    Math::vec3 scale = Math::vec3(1.0f);
};

// Always a closed loop: wraps last to first; add a matching last point for a true loop.
class PathTrack
{
public:
    void addKeyframe(f32 time, const Math::vec3& position,
                     const Math::quat& rotation = Math::quat(1.0f, 0.0f, 0.0f, 0.0f),
                     const Math::vec3& scale = Math::vec3(1.0f));
    void clear();
    bool empty() const;
    usize keyframeCount() const;

    f32 duration() const;

    // Catmull-Rom position, slerped rotation, lerped scale; time wrapped into [0, duration()).
    // One keyframe holds still; none returns identity.
    PathPose evaluate(f32 time) const;

private:
    std::vector<PathKeyframe> mKeyframes;
};

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
