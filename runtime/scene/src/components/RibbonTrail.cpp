#include "PCH.h"

#include "RibbonTrail.h"

#include "GameObject.h"

namespace Radion
{

RibbonTrail::RibbonTrail() : Component(Type, ComponentEventLateUpdate)
{
    reserveVertices();
}

void RibbonTrail::reserveVertices()
{
    // Sections times subdivisions times strips across the blade, six
    // vertices each. Reserved up front so a swing never reallocates in the
    // middle of a frame.
    const u32 strips = mBladeCount > 1 ? mBladeCount - 1 : 1;
    mVertices.reserve(static_cast<usize>(MaxSamples - 1) * 6u * mSubdivisions * strips);
}

bool RibbonTrail::setBlade(GameObject* base, GameObject* tip)
{
    GameObject* pair[2] = {base, tip};
    return setBladePoints(pair, 2);
}

bool RibbonTrail::setBladePoints(GameObject* const* objects, u32 count)
{
    if (!objects || count < 2 || count > MaxBladePoints)
        return false;
    for (u32 i = 0; i < count; ++i)
    {
        if (!objects[i])
            return false;
        // The same object twice would give the ribbon a zero-width strip
        // between them, which renders as nothing and hides the mistake.
        for (u32 j = 0; j < i; ++j)
            if (objects[i] == objects[j])
                return false;
    }
    for (u32 i = 0; i < count; ++i)
        mBlade[i] = objects[i];
    mBladeCount = count;
    reserveVertices();
    clear();
    return true;
}

Math::Vec3 RibbonTrail::centreOf(const Sample& value) const
{
    Math::Vec3 sum(0.0f);
    for (u32 i = 0; i < mBladeCount; ++i)
        sum += value.points[i];
    return mBladeCount ? sum / static_cast<f32>(mBladeCount) : sum;
}

void RibbonTrail::setEmitting(bool emitting)
{
    if (mEmitting == emitting)
        return;
    if (!emitting && mHasCurrent && mCount != 0)
    {
        const Sample& last = sample(mCount - 1);
        f32 moved = 0.0f;
        for (u32 i = 0; i < mBladeCount; ++i)
            moved = glm::max(moved, glm::length(mCurrentPoints[i] - last.points[i]));
        if (moved > 0.0001f)
            push(mCurrentPoints, mCurrentDistance);
    }
    mEmitting = emitting;
    mSeeded = false;
    mHasCurrent = false;
}
bool RibbonTrail::emitting() const
{
    return mEmitting;
}
GameObject* RibbonTrail::base() const
{
    return mBladeCount ? mBlade[0] : nullptr;
}
GameObject* RibbonTrail::tip() const
{
    return mBladeCount ? mBlade[mBladeCount - 1] : nullptr;
}
f32 RibbonTrail::lifetime() const
{
    return mLifetime;
}
f32 RibbonTrail::minDistance() const
{
    return mMinDistance;
}
u32 RibbonTrail::smoothness() const
{
    return mSubdivisions;
}
Color RibbonTrail::startColor() const
{
    return mStartColor;
}
Color RibbonTrail::endColor() const
{
    return mEndColor;
}
bool RibbonTrail::additive() const
{
    return mAdditive;
}
bool RibbonTrail::depthTest() const
{
    return mDepthTest;
}
void RibbonTrail::clear()
{
    mFirst = 0;
    mCount = 0;
    mDistance = 0.0f;
    mCurrentDistance = 0.0f;
    mSeeded = false;
    mHasCurrent = false;
    mVertices.clear();
}
void RibbonTrail::setLifetime(f32 seconds)
{
    mLifetime = glm::max(0.01f, seconds);
}
void RibbonTrail::setMinDistance(f32 distance)
{
    mMinDistance = glm::max(0.001f, distance);
}
void RibbonTrail::setSmoothness(u32 subdivisions)
{
    // Eight was too few to be worth evaluating a spline for: the curve was
    // computed and then drawn as eight straight chords, and on a fast swing -
    // where consecutive samples are furthest apart, which is exactly where
    // the curve matters - those chords are plainly visible facets.
    mSubdivisions = glm::clamp(subdivisions, 1u, 64u);
    reserveVertices();
}
void RibbonTrail::setColor(Color start, Color end)
{
    mStartColor = start;
    mEndColor = end;
}
void RibbonTrail::setTexture(TextureHandle texture)
{
    mTexture = texture;
}
void RibbonTrail::setAdditive(bool additive)
{
    mAdditive = additive;
}
void RibbonTrail::setDepthTest(bool enabled)
{
    mDepthTest = enabled;
}
usize RibbonTrail::sampleCount() const
{
    return mCount;
}

void RibbonTrail::onLateUpdate(f32 deltaTime)
{
    for (usize i = 0; i < mCount; ++i)
        sample(i).age += deltaTime;
    expire();

    bool bladeAlive = mBladeCount >= 2;
    for (u32 i = 0; i < mBladeCount && bladeAlive; ++i)
        bladeAlive = mBlade[i] && !mBlade[i]->disposed();

    if (mEmitting && bladeAlive)
    {
        Math::Vec3 current[MaxBladePoints];
        for (u32 i = 0; i < mBladeCount; ++i)
        {
            current[i] = mBlade[i]->globalPosition();
            mCurrentPoints[i] = current[i];
        }
        mHasCurrent = true;
        if (!mSeeded || mCount == 0)
        {
            push(current, mDistance);
            for (u32 i = 0; i < mBladeCount; ++i)
                mLastPoints[i] = current[i];
            mCurrentDistance = mDistance;
            mSeeded = true;
        }
        else
        {
            // The furthest-travelled point sets the sampling rate: the tip of
            // a swung blade covers far more ground than its base, and pacing
            // by anything slower leaves the fast end visibly faceted.
            f32 travel = 0.0f;
            for (u32 i = 0; i < mBladeCount; ++i)
                travel = glm::max(travel, glm::length(current[i] - mLastPoints[i]));
            const u32 steps = travel > 0.0f ? static_cast<u32>(travel / mMinDistance) : 0u;
            for (u32 step = 1; step <= steps; ++step)
            {
                const f32 amount = glm::min((mMinDistance * step) / travel, 1.0f);
                Math::Vec3 stepped[MaxBladePoints];
                f32 advanced = 0.0f;
                for (u32 i = 0; i < mBladeCount; ++i)
                {
                    stepped[i] = glm::mix(mLastPoints[i], current[i], amount);
                    advanced = glm::max(advanced,
                                        glm::length(stepped[i] - sample(mCount - 1).points[i]));
                }
                mDistance += advanced;
                push(stepped, mDistance);
            }
            if (steps > 0)
            {
                const f32 amount = glm::min((mMinDistance * steps) / travel, 1.0f);
                for (u32 i = 0; i < mBladeCount; ++i)
                    mLastPoints[i] = glm::mix(mLastPoints[i], current[i], amount);
            }
            const Sample& last = sample(mCount - 1);
            f32 remaining = 0.0f;
            for (u32 i = 0; i < mBladeCount; ++i)
                remaining = glm::max(remaining, glm::length(current[i] - last.points[i]));
            mCurrentDistance = last.distance + remaining;
        }
    }

    buildVertices();
    if (!mVertices.empty())
        TrailDraws().submit(
            {mVertices.data(), static_cast<u32>(mVertices.size()), mTexture,
             mAdditive ? BatchRenderer::BlendMode::Additive : BatchRenderer::BlendMode::Alpha,
             mDepthTest});
}

void RibbonTrail::push(const Math::Vec3* points, f32 distance)
{
    if (mCount == MaxSamples)
    {
        mFirst = (mFirst + 1) % MaxSamples;
        --mCount;
    }
    Sample& value = mSamples[(mFirst + mCount) % MaxSamples];
    for (u32 i = 0; i < mBladeCount; ++i)
        value.points[i] = points[i];
    value.age = 0.0f;
    value.distance = distance;
    ++mCount;
}
RibbonTrail::Sample& RibbonTrail::sample(usize index)
{
    return mSamples[(mFirst + index) % MaxSamples];
}
const RibbonTrail::Sample& RibbonTrail::sample(usize index) const
{
    return mSamples[(mFirst + index) % MaxSamples];
}
void RibbonTrail::expire()
{
    while (mCount && sample(0).age >= mLifetime)
    {
        mFirst = (mFirst + 1) % MaxSamples;
        --mCount;
    }
}
void RibbonTrail::buildVertices()
{
    mVertices.clear();
    if (mCount == 0)
        return;

    usize renderCount = mCount;
    if (mHasCurrent)
    {
        const Sample& last = sample(mCount - 1);
        f32 moved = 0.0f;
        for (u32 i = 0; i < mBladeCount; ++i)
            moved = glm::max(moved, glm::length(mCurrentPoints[i] - last.points[i]));
        if (moved > 0.0001f)
            ++renderCount;
    }
    if (renderCount < 2)
        return;

    const f32 firstDistance = renderSample(0).distance;
    const f32 span = glm::max(renderSample(renderCount - 1).distance - firstDistance, 0.0001f);

    for (usize segment = 0; segment + 1 < renderCount; ++segment)
    {
        const Sample before = renderSample(segment > 0 ? segment - 1 : segment);
        const Sample from = renderSample(segment);
        const Sample to = renderSample(segment + 1);
        const Sample after = renderSample(segment + 2 < renderCount ? segment + 2 : segment + 1);
        Sample previous = from;
        for (u32 division = 1; division <= mSubdivisions; ++division)
        {
            const f32 amount = static_cast<f32>(division) / mSubdivisions;
            const Sample current = interpolate(before, from, to, after, amount);
            appendSection(previous, current, firstDistance, span);
            previous = current;
        }
    }
}

void RibbonTrail::appendSection(const Sample& a, const Sample& b, f32 firstDistance, f32 span)
{
    const f32 fadeA = glm::clamp(1.0f - a.age / mLifetime, 0.0f, 1.0f);
    const f32 fadeB = glm::clamp(1.0f - b.age / mLifetime, 0.0f, 1.0f);
    const Color colorA = Color::lerp(mEndColor, mStartColor, fadeA);
    const Color colorB = Color::lerp(mEndColor, mStartColor, fadeB);
    const f32 vA = (a.distance - firstDistance) / span;
    const f32 vB = (b.distance - firstDistance) / span;
    // One quad per gap between blade points, so a blade sampled at more than
    // two places becomes a sheet of strips rather than one flat quad. u runs
    // 0..1 across the whole blade whatever the count, so a texture does not
    // change scale when a point is added.
    const f32 spanU = static_cast<f32>(mBladeCount - 1);
    for (u32 i = 0; i + 1 < mBladeCount; ++i)
    {
        const f32 u0 = static_cast<f32>(i) / spanU;
        const f32 u1 = static_cast<f32>(i + 1) / spanU;
        mVertices.push_back({a.points[i], Math::Vec2(u0, vA), colorA});
        mVertices.push_back({a.points[i + 1], Math::Vec2(u1, vA), colorA});
        mVertices.push_back({b.points[i], Math::Vec2(u0, vB), colorB});
        mVertices.push_back({b.points[i + 1], Math::Vec2(u1, vB), colorB});
        mVertices.push_back({b.points[i], Math::Vec2(u0, vB), colorB});
        mVertices.push_back({a.points[i + 1], Math::Vec2(u1, vA), colorA});
    }
}

RibbonTrail::Sample RibbonTrail::renderSample(usize index) const
{
    if (index < mCount)
        return sample(index);
    Sample current;
    for (u32 i = 0; i < mBladeCount; ++i)
        current.points[i] = mCurrentPoints[i];
    current.distance = mCurrentDistance;
    return current;
}

RibbonTrail::Sample RibbonTrail::interpolate(const Sample& before, const Sample& from,
                                             const Sample& to, const Sample& after,
                                             f32 amount) const
{
    auto limitedTangent =
        [](const Math::Vec3& previous, const Math::Vec3& point, const Math::Vec3& next)
    {
        const Math::Vec3 incoming = point - previous;
        const Math::Vec3 outgoing = next - point;
        const f32 incomingLength = glm::length(incoming);
        const f32 outgoingLength = glm::length(outgoing);
        if (incomingLength < 0.0001f || outgoingLength < 0.0001f ||
            glm::dot(incoming, outgoing) <= 0.0f)
            return Math::Vec3(0.0f);
        const Math::Vec3 direction = incoming / incomingLength + outgoing / outgoingLength;
        const f32 directionLength = glm::length(direction);
        if (directionLength < 0.0001f)
            return Math::Vec3(0.0f);
        return direction / directionLength * glm::min(incomingLength, outgoingLength);
    };

    auto hermite = [](const Math::Vec3& fromPoint, const Math::Vec3& fromTangent,
                      const Math::Vec3& toPoint, const Math::Vec3& toTangent, f32 t)
    {
        const f32 t2 = t * t;
        const f32 t3 = t2 * t;
        return (2.0f * t3 - 3.0f * t2 + 1.0f) * fromPoint +
               (t3 - 2.0f * t2 + t) * fromTangent + (-2.0f * t3 + 3.0f * t2) * toPoint +
               (t3 - t2) * toTangent;
    };

    const Math::Vec3 beforeCenter = centreOf(before);
    const Math::Vec3 fromCenter = centreOf(from);
    const Math::Vec3 toCenter = centreOf(to);
    const Math::Vec3 afterCenter = centreOf(after);
    const Math::Vec3 fromTangent = limitedTangent(beforeCenter, fromCenter, toCenter);
    const Math::Vec3 toTangent = limitedTangent(fromCenter, toCenter, afterCenter);
    const Math::Vec3 center = hermite(fromCenter, fromTangent, toCenter, toTangent, amount);

    // ONE curve, through the centre, with every blade point carried along as
    // an offset from it. Giving each point its own spline lets two of them
    // cross on a fast reversal and folds the ribbon into a sail - with more
    // than two points the odds only get worse.
    //
    // The offsets are blended, not squared up against the curve's tangent: a
    // swept ribbon is the ruled surface between successive blade positions,
    // and forcing the cross-section perpendicular to the direction of travel
    // is neither what the geometry is nor what any reference does. Tried
    // once, and where the blade ran nearly along its own path the
    // perpendicular part collapsed to numerical noise which then got rescaled
    // back to full width - a full-width strip pointing nowhere in particular.
    const f32 blend = amount * amount * (3.0f - 2.0f * amount);

    Sample result;
    for (u32 i = 0; i < mBladeCount; ++i)
    {
        const Math::Vec3 fromOffset = from.points[i] - fromCenter;
        const Math::Vec3 toOffset = to.points[i] - toCenter;
        result.points[i] = center + glm::mix(fromOffset, toOffset, blend);
    }
    result.age = glm::mix(from.age, to.age, amount);
    result.distance = glm::mix(from.distance, to.distance, amount);
    return result;
}

} // namespace Radion
