#ifndef RADION_MORPH_ANIMATION_H
#define RADION_MORPH_ANIMATION_H

#include "Types.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace Radion
{

// Per-vertex keyframes: one position and normal per vertex per frame, in the
// same order and count as the mesh's own vertex buffer. There is no skeleton
// here - the "bone" is the frame index.
struct MorphKeyframes
{
    std::vector<std::vector<glm::vec3>> framePositions;
    std::vector<std::vector<glm::vec3>> frameNormals;

    u32 frameCount() const
    {
        return static_cast<u32>(framePositions.size());
    }
    usize vertexCount() const
    {
        return framePositions.empty() ? 0 : framePositions[0].size();
    }
};

// One frame of one tag: a named attachment point that moves with the
// animation. What glues separate parts together - legs to torso to head, or
// a weapon onto a hand.
struct MorphTagFrame
{
    glm::vec3 origin{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
};

struct MorphTags
{
    std::vector<std::string> names;
    std::vector<std::vector<MorphTagFrame>> perFrame; // [frame][tag]

    bool empty() const
    {
        return names.empty();
    }
    // Index of `name`, or -1.
    s32 find(const std::string& name) const;
};

// A named frame range: "run" is frames 40 to 47 at 10 fps, looping. The
// Quake animation.cfg model, given directly instead of parsed - which is
// what lets a caller say play("run") instead of remembering numbers.
struct MorphClip
{
    std::string name;
    s32 first = 0;
    s32 last = 0; // inclusive
    f32 fps = 10.0f;
    bool loop = true;
};

// Drives a MorphKeyframes/MorphTags pair over time: advances the current
// clip, resolves it to a frame pair plus a blend factor, and crossfades from
// whatever was playing into a newly requested clip - two clips blended
// together, each interpolating its own two keyframes. That crossfade is the
// one real step beyond plain frame interpolation.
//
// Pure CPU maths, with no mesh or GPU behind it: the caller owns the mesh
// and pushes the result.
class MorphAnimator
{
public:
    void addClip(const std::string& name, s32 first, s32 last, f32 fps, bool loop = true);
    bool hasClip(const std::string& name) const;
    void removeClip(const std::string& name);
    // For scene persistence and for an inspector's clip list - nothing else
    // can enumerate what is registered.
    const std::unordered_map<std::string, MorphClip>& clips() const
    {
        return mClips;
    }

    // Crossfades from the current clip into `name` over blendTime seconds.
    // False when there is no such clip.
    bool play(const std::string& name, f32 blendTime = 0.15f);
    const std::string& currentClip() const;

    void update(f32 deltaTime);

    // Forces the current clip to an exact time, dropping any crossfade in
    // progress first - a scrub bar wants this exact pose, not "70% faded
    // into whatever was playing before". Does nothing when nothing is
    // playing.
    void seek(f32 time);
    f32 time() const
    {
        return mCurrent.time;
    }

    // Blends the keyframes into `positions` and `normals`, which must
    // already be sized to keyframes.vertexCount(). Only those two: uvs,
    // tangents and colours are the base mesh's and are never touched.
    void writeVertices(const MorphKeyframes& keyframes, std::vector<glm::vec3>& positions,
                       std::vector<glm::vec3>& normals) const;

    // Local transform of tag `tagIndex` at the current blended pose.
    // Identity when the tag set has no frames.
    glm::mat4 tagTransform(const MorphTags& tags, s32 tagIndex) const;

private:
    struct PlayState
    {
        const MorphClip* clip = nullptr;
        f32 time = 0.0f; // seconds into the clip
    };

    // Resolves a PlayState to frame a, frame b and the blend between them,
    // handling loop wraparound and one-shot clamping.
    void framePair(const PlayState& state, s32& a, s32& b, f32& t) const;

    // Clips are keyed by name because PlayState holds a pointer into this
    // map: a vector would invalidate it on the next addClip().
    std::unordered_map<std::string, MorphClip> mClips;
    PlayState mCurrent, mPrevious;
    f32 mBlend = 1.0f; // 0 = fully previous, 1 = fully current
    f32 mBlendDuration = 0.0f;
};

} // namespace Radion

#endif // RADION_MORPH_ANIMATION_H
