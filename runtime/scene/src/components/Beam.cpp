#include "PCH.h"

#include "Beam.h"

namespace Radion
{

namespace
{
// U spans the bolt's WIDTH, V its LENGTH (tail at V=1, head at V=0) - most
// spark/beam sprites (Particles/spark1.png included) are tall and narrow,
// drawn bright at the top fading to nothing at the bottom, so the image's
// own long axis is V, not U. Mapping length to U instead runs the sprite
// sideways across the bolt's width rather than along its length.
void buildBeamQuad(const Math::vec3& tail, const Math::vec3& head, const Math::vec3& perp,
                   Color colorTail, Color colorHead, TrailVertex* out)
{
    const Math::vec3 p0 = tail - perp, p1 = tail + perp;
    const Math::vec3 p2 = head - perp, p3 = head + perp;
    out[0] = {p0, Math::vec2(0.0f, 1.0f), colorTail};
    out[1] = {p1, Math::vec2(1.0f, 1.0f), colorTail};
    out[2] = {p2, Math::vec2(0.0f, 0.0f), colorHead};
    out[3] = {p1, Math::vec2(1.0f, 1.0f), colorTail};
    out[4] = {p3, Math::vec2(1.0f, 0.0f), colorHead};
    out[5] = {p2, Math::vec2(0.0f, 0.0f), colorHead};
}
} // namespace

Beam::Beam() : Component(Type, ComponentEventLateUpdate)
{
    mColorTail = Color(255, 255, 255, 0);
}

void Beam::setPoints(const Math::vec3& start, const Math::vec3& end)
{
    mStart = start;
    mEnd = end;
}
const Math::vec3& Beam::start() const
{
    return mStart;
}
const Math::vec3& Beam::end() const
{
    return mEnd;
}
void Beam::setWidth(f32 width)
{
    mWidth = Math::max(0.001f, width);
}
f32 Beam::width() const
{
    return mWidth;
}
void Beam::setSegmentLength(f32 length)
{
    mSegmentLength = Math::max(0.001f, length);
}
f32 Beam::segmentLength() const
{
    return mSegmentLength;
}
void Beam::setColor(Color colorHead, Color colorTail)
{
    mColorHead = colorHead;
    mColorTail = colorTail;
}
Color Beam::colorHead() const
{
    return mColorHead;
}
Color Beam::colorTail() const
{
    return mColorTail;
}
void Beam::setTravelTime(f32 seconds)
{
    mTravelTime = Math::max(0.001f, seconds);
}
f32 Beam::travelTime() const
{
    return mTravelTime;
}
void Beam::setTexture(TextureHandle texture)
{
    mTexture = texture;
}
TextureHandle Beam::texture() const
{
    return mTexture;
}
void Beam::setAdditive(bool additive)
{
    mAdditive = additive;
}
bool Beam::additive() const
{
    return mAdditive;
}
void Beam::setDepthTest(bool enabled)
{
    mDepthTest = enabled;
}
bool Beam::depthTest() const
{
    return mDepthTest;
}

void Beam::fire()
{
    mFiring = true;
    mElapsed = 0.0f;
}
bool Beam::isFiring() const
{
    return mFiring;
}

void Beam::onLateUpdate(f32 deltaTime)
{
    if (!mFiring)
        return;

    mElapsed += deltaTime;
    if (mElapsed >= mTravelTime)
    {
        mFiring = false;
        return;
    }

    const Math::vec3 axis = mEnd - mStart;
    const f32 totalLength = Math::length(axis);
    const Math::vec3 dir = totalLength > 0.0001f ? axis / totalLength : Math::vec3(0.0f, 0.0f, 1.0f);

    const f32 t = mElapsed / mTravelTime;
    const Math::vec3 head = mStart + axis * t;
    // The tail trails mSegmentLength behind the head, but never past start -
    // a bolt that has only travelled half its own segment length yet is
    // shorter than usual, not spawning part of itself behind where it began.
    const f32 travelled = totalLength * t;
    const f32 tailDistance = Math::min(mSegmentLength, travelled);
    const Math::vec3 tail = head - dir * tailDistance;

    const Math::vec3 worldUp(0.0f, 1.0f, 0.0f);
    Math::vec3 perpA = Math::cross(dir, worldUp);
    if (Math::length(perpA) < 0.0001f)
        perpA = Math::cross(dir, Math::vec3(1.0f, 0.0f, 0.0f));
    perpA = Math::normalize(perpA) * (mWidth * 0.5f);
    const Math::vec3 perpB = Math::normalize(Math::cross(dir, perpA)) * (mWidth * 0.5f);

    buildBeamQuad(tail, head, perpA, mColorTail, mColorHead, &mVertices[0]);
    buildBeamQuad(tail, head, perpB, mColorTail, mColorHead, &mVertices[6]);

    TrailDraws().submit(
        {mVertices, 12, mTexture,
         mAdditive ? BatchRenderer::BlendMode::Additive : BatchRenderer::BlendMode::Alpha,
         mDepthTest});
}

} // namespace Radion
