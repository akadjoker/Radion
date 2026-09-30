#ifndef RADION_BEAM_H
#define RADION_BEAM_H

#include "Color.h"
#include "Component.h"
#include "GPU.h"
#include "TrailRender.h"

#include "Math.h"

namespace Radion
{

// Two quads crossed 90 degrees apart, perpendiculars from world-up, so nothing camera-facing is needed at render time.
class Beam final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Beam;

    void setPoints(const Math::vec3& start, const Math::vec3& end);
    const Math::vec3& start() const;
    const Math::vec3& end() const;
    void setWidth(f32 width);
    f32 width() const;
    // Clamped so the tail never reaches past start.
    void setSegmentLength(f32 length);
    f32 segmentLength() const;
    // Head-to-tail gradient at any instant, not a fade over lifetime.
    void setColor(Color colorHead, Color colorTail = Color::Transparent);
    Color colorHead() const;
    Color colorTail() const;
    void setTravelTime(f32 seconds);
    f32 travelTime() const;
    void setTexture(TextureHandle texture);
    TextureHandle texture() const;
    void setAdditive(bool additive);
    bool additive() const;
    void setDepthTest(bool enabled);
    bool depthTest() const;

    // Call setPoints() first to launch somewhere new.
    void fire();
    bool isFiring() const;

private:
    friend class GameObject;

    Beam();
    void onLateUpdate(f32 deltaTime) override;

    Math::vec3 mStart = Math::vec3(0.0f);
    Math::vec3 mEnd = Math::vec3(0.0f, 0.0f, 1.0f);
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
    TrailVertex mVertices[12];
};

} // namespace Radion

#endif // RADION_BEAM_H
