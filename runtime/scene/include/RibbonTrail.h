#ifndef RADION_RIBBON_TRAIL_H
#define RADION_RIBBON_TRAIL_H

#include "Color.h"
#include "Component.h"
#include "GPU.h"
#include "TrailRender.h"

#include <vector>

namespace Radion
{

class RibbonTrail final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::RibbonTrail;

    // Widest the blade may be sampled. Two is a flat strip; more makes the
    // ribbon's cross-section a curve of its own, which is what a real sword
    // trail looks like along a curved edge.
    static constexpr u32 MaxBladePoints = 8;

    bool setBlade(GameObject* base, GameObject* tip);
    // Two to MaxBladePoints objects strung along the blade, base first. Each
    // one traces its own line through the trail and the ribbon is the surface
    // between them, so a curved blade gives a curved sheet instead of the
    // flat quad two points can describe.
    bool setBladePoints(GameObject* const* objects, u32 count);
    u32 bladePointCount() const
    {
        return mBladeCount;
    }
    void setEmitting(bool emitting);
    bool emitting() const;
    void clear();

    void setLifetime(f32 seconds);
    void setMinDistance(f32 distance);
    void setSmoothness(u32 subdivisions);
    void setColor(Color start, Color end = Color::Transparent);
    void setTexture(TextureHandle texture);
    void setAdditive(bool additive);
    void setDepthTest(bool enabled);
    usize sampleCount() const;
    GameObject* base() const;
    GameObject* tip() const;
    f32 lifetime() const;
    f32 minDistance() const;
    u32 smoothness() const;
    Color startColor() const;
    Color endColor() const;
    bool additive() const;
    bool depthTest() const;

private:
    friend class GameObject;

    struct Sample
    {
        Math::Vec3 points[MaxBladePoints] = {};
        f32 age = 0.0f;
        f32 distance = 0.0f;
    };

    RibbonTrail();
    void onLateUpdate(f32 deltaTime) override;
    void push(const Math::Vec3* points, f32 distance);
    // Average of a sample's blade points. The curve is run through THIS and
    // each point keeps its offset from it - two edges given independent
    // splines cross on a fast reversal and fold the ribbon into a sail, and
    // with more than two the odds only get worse.
    Math::Vec3 centreOf(const Sample& sample) const;
    Sample& sample(usize index);
    const Sample& sample(usize index) const;
    void expire();
    void reserveVertices();
    void buildVertices();
    // Two triangles of the ribbon quad between two samples, faded by age and
    // textured along the strip's own distance (firstDistance/span cover the
    // whole strip, not just this section, so the UV stays continuous).
    void appendSection(const Sample& a, const Sample& b, f32 firstDistance, f32 span);
    Sample renderSample(usize index) const;
    Sample interpolate(const Sample& before, const Sample& from, const Sample& to,
                       const Sample& after, f32 amount) const;

    static constexpr usize MaxSamples = 48;
    GameObject* mBlade[MaxBladePoints] = {};
    u32 mBladeCount = 0;
    Sample mSamples[MaxSamples];
    usize mFirst = 0;
    usize mCount = 0;
    Math::Vec3 mLastPoints[MaxBladePoints] = {};
    Math::Vec3 mCurrentPoints[MaxBladePoints] = {};
    f32 mDistance = 0.0f;
    f32 mCurrentDistance = 0.0f;
    f32 mLifetime = 0.35f;
    f32 mMinDistance = 0.04f;
    u32 mSubdivisions = 12;
    Color mStartColor = Color::Cyan;
    Color mEndColor = Color::Transparent;
    TextureHandle mTexture;
    std::vector<TrailVertex> mVertices;
    bool mEmitting = false;
    bool mSeeded = false;
    bool mHasCurrent = false;
    bool mAdditive = true;
    bool mDepthTest = true;
};

} // namespace Radion

#endif // RADION_RIBBON_TRAIL_H
