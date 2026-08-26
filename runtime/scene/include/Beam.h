#ifndef RADION_BEAM_H
#define RADION_BEAM_H

#include "Color.h"
#include "Component.h"
#include "GPU.h"
#include "TrailRender.h" // TrailVertex

#include <glm/glm.hpp>

namespace Radion
{

// A bolt travelling from one world-space point to another over setSpeed() -
// a plasma shot, an arrow, anything that visibly crosses the distance
// instead of appearing along its whole length at once. Not a history trail
// like RibbonTrail (which tracks two moving points as a blade's own edges):
// fire() picks a fixed start/end and the bolt's own head travels the line
// between them, a fixed-length tail following behind it. Two quads sharing
// the head-tail centerline, crossed 90 degrees apart (the classic
// tree-impostor/beam X shape), so it never disappears end-on the way a
// single flat quad would from some viewing angles. Both perpendiculars come
// from the world-up axis, not camera-facing, so it needs nothing at render
// time. Renders through TrailDraws(), the same queue RibbonTrail/Billboard/
// Text3D submit to - no render pass of its own.
class Beam final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Beam;

    void setPoints(const Math::Vec3& start, const Math::Vec3& end);
    const Math::Vec3& start() const;
    const Math::Vec3& end() const;
    void setWidth(f32 width);
    f32 width() const;
    // Visible length of the bolt itself - independent of the start/end
    // distance, which is usually much longer. Clamped so the tail never
    // reaches past start before the bolt has travelled that far.
    void setSegmentLength(f32 length);
    f32 segmentLength() const;
    // Gradient along the bolt's own length at any instant, not a fade over
    // its lifetime: head (colorHead) is where it currently is, tail
    // (colorTail) trails behind it - a comet, not a dissolve.
    void setColor(Color colorHead, Color colorTail = Color::Transparent);
    Color colorHead() const;
    Color colorTail() const;
    // Seconds to cross the whole start-to-end distance. The bolt stops
    // (isFiring() goes false) the instant its head reaches end.
    void setTravelTime(f32 seconds);
    f32 travelTime() const;
    void setTexture(TextureHandle texture);
    TextureHandle texture() const;
    void setAdditive(bool additive);
    bool additive() const;
    void setDepthTest(bool enabled);
    bool depthTest() const;

    // Starts (or restarts) the bolt from setPoints()'s current start,
    // travelling toward its current end. Call setPoints() first if the bolt
    // should launch somewhere new.
    void fire();
    bool isFiring() const;

private:
    friend class GameObject;

    Beam();
    void onLateUpdate(f32 deltaTime) override;

    Math::Vec3 mStart = Math::Vec3(0.0f);
    Math::Vec3 mEnd = Math::Vec3(0.0f, 0.0f, 1.0f);
    f32 mWidth = 0.05f;
    f32 mSegmentLength = 1.0f;
    Color mColorHead;
    Color mColorTail;
    f32 mTravelTime = 0.3f;
    f32 mElapsed = 0.0f;
    bool mFiring = false;
    TextureHandle mTexture;
    bool mAdditive = true;
    bool mDepthTest = false;
    TrailVertex mVertices[12]; // two crossed quads, 6 vertices each
};

} // namespace Radion

#endif // RADION_BEAM_H
