#include "PCH.h"

#include "MorphAnimation.h"

namespace Radion
{

namespace
{
const std::string kEmptyClipName;

// Frame indices are clamped: a shared clip table reused across models with different frame counts would index past the end.
s32 clampFrame(s32 frame, s32 maxFrame)
{
    if (maxFrame < 0)
        return 0;
    if (frame < 0)
        return 0;
    return frame > maxFrame ? maxFrame : frame;
}
} // namespace

s32 MorphTags::find(const std::string& name) const
{
    for (usize i = 0; i < names.size(); ++i)
        if (names[i] == name)
            return static_cast<s32>(i);
    return -1;
}

void MorphAnimator::addClip(const std::string& name, s32 first, s32 last, f32 fps, bool loop)
{
    MorphClip clip;
    clip.name = name;
    clip.first = first;
    clip.last = last;
    clip.fps = fps;
    clip.loop = loop;
    mClips[name] = clip;
}

bool MorphAnimator::hasClip(const std::string& name) const
{
    return mClips.find(name) != mClips.end();
}

void MorphAnimator::removeClip(const std::string& name)
{
    const auto found = mClips.find(name);
    if (found == mClips.end())
        return;
    // Play states point into the map; erasing would dangle them.
    if (mCurrent.clip == &found->second)
        mCurrent.clip = nullptr;
    if (mPrevious.clip == &found->second)
        mPrevious.clip = nullptr;
    mClips.erase(found);
}

const std::string& MorphAnimator::currentClip() const
{
    return mCurrent.clip ? mCurrent.clip->name : kEmptyClipName;
}

bool MorphAnimator::play(const std::string& name, f32 blendTime)
{
    const auto found = mClips.find(name);
    if (found == mClips.end())
        return false;

    // Restart is skipped only for a looping clip already playing; a one-shot freezes on its last frame and must restart every time.
    const bool sameClip = mCurrent.clip == &found->second;
    if (sameClip && found->second.loop)
        return true;

    mPrevious = mCurrent;
    mCurrent.clip = &found->second;
    mCurrent.time = 0.0f;
    mBlendDuration = blendTime;
    // Snap, do not crossfade: a retriggered one-shot would blend against an offset copy of itself.
    mBlend = (sameClip || blendTime <= 0.0f || !mPrevious.clip) ? 1.0f : 0.0f;
    return true;
}

void MorphAnimator::update(f32 deltaTime)
{
    if (mCurrent.clip)
        mCurrent.time += deltaTime;
    if (mPrevious.clip)
        mPrevious.time += deltaTime;

    if (mBlend < 1.0f)
    {
        mBlend += (mBlendDuration > 0.0f) ? deltaTime / mBlendDuration : 1.0f;
        if (mBlend >= 1.0f)
        {
            mBlend = 1.0f;
            mPrevious.clip = nullptr;
        }
    }
}

void MorphAnimator::seek(f32 time)
{
    if (mCurrent.clip)
        mCurrent.time = time;
    mPrevious.clip = nullptr;
    mBlend = 1.0f;
}

void MorphAnimator::framePair(const PlayState& state, s32& a, s32& b, f32& t) const
{
    if (!state.clip)
    {
        a = b = 0;
        t = 0.0f;
        return;
    }
    const MorphClip& clip = *state.clip;
    const s32 span = clip.last - clip.first;
    if (span <= 0)
    {
        a = b = clip.first;
        t = 0.0f;
        return;
    }

    const f32 framesElapsed = state.time * clip.fps;
    if (clip.loop)
    {
        f32 wrapped = std::fmod(framesElapsed, static_cast<f32>(span + 1));
        if (wrapped < 0.0f)
            wrapped += static_cast<f32>(span + 1);
        const s32 index = static_cast<s32>(std::floor(wrapped));
        t = wrapped - static_cast<f32>(index);
        a = clip.first + index;
        b = clip.first + ((index + 1) % (span + 1));
    }
    else
    {
        f32 clamped = framesElapsed;
        if (clamped < 0.0f)
            clamped = 0.0f;
        if (clamped > static_cast<f32>(span))
            clamped = static_cast<f32>(span);
        const s32 index = static_cast<s32>(std::floor(clamped));
        t = clamped - static_cast<f32>(index);
        a = clip.first + index;
        b = clip.first + ((index < span) ? index + 1 : index);
    }
}

void MorphAnimator::writeVertices(const MorphKeyframes& keyframes, std::vector<Math::vec3>& positions,
                                  std::vector<Math::vec3>& normals) const
{
    const usize count = keyframes.vertexCount();
    if (count == 0 || positions.size() < count || !mCurrent.clip)
        return;

    const s32 maxFrame = static_cast<s32>(keyframes.frameCount()) - 1;

    s32 currentA = 0, currentB = 0;
    f32 currentT = 0.0f;
    framePair(mCurrent, currentA, currentB, currentT);
    currentA = clampFrame(currentA, maxFrame);
    currentB = clampFrame(currentB, maxFrame);
    const std::vector<Math::vec3>& positionsA = keyframes.framePositions[currentA];
    const std::vector<Math::vec3>& positionsB = keyframes.framePositions[currentB];

    const bool hasNormals = !keyframes.frameNormals.empty() && normals.size() >= count;
    const bool blending = mBlend < 1.0f && mPrevious.clip;

    s32 previousA = 0, previousB = 0;
    f32 previousT = 0.0f;
    if (blending)
    {
        framePair(mPrevious, previousA, previousB, previousT);
        previousA = clampFrame(previousA, maxFrame);
        previousB = clampFrame(previousB, maxFrame);
    }
    const std::vector<Math::vec3>* previousPositionsA =
        blending ? &keyframes.framePositions[previousA] : nullptr;
    const std::vector<Math::vec3>* previousPositionsB =
        blending ? &keyframes.framePositions[previousB] : nullptr;

    for (usize i = 0; i < count; ++i)
    {
        Math::vec3 position = Math::mix(positionsA[i], positionsB[i], currentT);
        Math::vec3 normal = hasNormals ? Math::mix(keyframes.frameNormals[currentA][i],
                                                 keyframes.frameNormals[currentB][i], currentT)
                                      : Math::vec3(0.0f, 1.0f, 0.0f);
        if (blending)
        {
            const Math::vec3 previousPosition =
                Math::mix((*previousPositionsA)[i], (*previousPositionsB)[i], previousT);
            position = Math::mix(previousPosition, position, mBlend);
            if (hasNormals)
            {
                const Math::vec3 previousNormal =
                    Math::mix(keyframes.frameNormals[previousA][i],
                             keyframes.frameNormals[previousB][i], previousT);
                normal = Math::mix(previousNormal, normal, mBlend);
            }
        }
        positions[i] = position;
        if (hasNormals)
            normals[i] = Math::dot(normal, normal) > 1.0e-12f ? Math::normalize(normal)
                                                             : Math::vec3(0.0f, 1.0f, 0.0f);
    }
}

Math::mat4 MorphAnimator::tagTransform(const MorphTags& tags, s32 tagIndex) const
{
    if (tags.empty() || tagIndex < 0 || tagIndex >= static_cast<s32>(tags.names.size()) ||
        tags.perFrame.empty())
        return Math::mat4(1.0f);

    // Nothing playing: use the tag's frame 0 rather than identity, so models need no dummy clip.
    if (!mCurrent.clip)
    {
        const MorphTagFrame& first = tags.perFrame[0][tagIndex];
        return Math::translate(Math::mat4(1.0f), first.origin) * Math::mat4_cast(first.rotation);
    }

    const s32 maxFrame = static_cast<s32>(tags.perFrame.size()) - 1;

    s32 currentA = 0, currentB = 0;
    f32 currentT = 0.0f;
    framePair(mCurrent, currentA, currentB, currentT);
    currentA = clampFrame(currentA, maxFrame);
    currentB = clampFrame(currentB, maxFrame);
    const MorphTagFrame& frameA = tags.perFrame[currentA][tagIndex];
    const MorphTagFrame& frameB = tags.perFrame[currentB][tagIndex];
    Math::vec3 origin = Math::mix(frameA.origin, frameB.origin, currentT);
    Math::quat rotation = Math::normalize(Math::lerp(frameA.rotation, frameB.rotation, currentT));

    if (mBlend < 1.0f && mPrevious.clip)
    {
        s32 previousA = 0, previousB = 0;
        f32 previousT = 0.0f;
        framePair(mPrevious, previousA, previousB, previousT);
        previousA = clampFrame(previousA, maxFrame);
        previousB = clampFrame(previousB, maxFrame);
        const MorphTagFrame& previousFrameA = tags.perFrame[previousA][tagIndex];
        const MorphTagFrame& previousFrameB = tags.perFrame[previousB][tagIndex];
        const Math::vec3 previousOrigin =
            Math::mix(previousFrameA.origin, previousFrameB.origin, previousT);
        const Math::quat previousRotation = Math::normalize(
            Math::lerp(previousFrameA.rotation, previousFrameB.rotation, previousT));
        origin = Math::mix(previousOrigin, origin, mBlend);
        rotation = Math::normalize(Math::lerp(previousRotation, rotation, mBlend));
    }

    return Math::translate(Math::mat4(1.0f), origin) * Math::mat4_cast(rotation);
}

} // namespace Radion
