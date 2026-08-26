#include "PCH.h"

#include "Light.h"

namespace Radion
{

namespace
{
f32 validRange(f32 range)
{
    return std::isfinite(range) ? glm::max(0.01f, range) : 10.0f;
}
} // namespace

Light::Light(LightType type) : Component(Type), mType(type)
{
}

LightType Light::lightType() const
{
    return mType;
}
void Light::setColor(const Math::Vec3& color)
{
    if (std::isfinite(color.x) && std::isfinite(color.y) && std::isfinite(color.z))
        mColor = glm::max(color, Math::Vec3(0.0f));
}
const Math::Vec3& Light::color() const
{
    return mColor;
}
void Light::setIntensity(f32 intensity)
{
    if (std::isfinite(intensity))
        mIntensity = glm::max(0.0f, intensity);
}
f32 Light::intensity() const
{
    return mIntensity;
}
DirectionalLight::DirectionalLight() : Light(LightType::Directional)
{
    setCastShadows(true);
}

PointLight::PointLight() : Light(LightType::Point)
{
}
void PointLight::setRange(f32 range)
{
    if (std::isfinite(range))
        mRange = validRange(range);
}
f32 PointLight::range() const
{
    return mRange;
}

SpotLight::SpotLight() : Light(LightType::Spot)
{
}
void SpotLight::setRange(f32 range)
{
    if (std::isfinite(range))
        mRange = validRange(range);
}
f32 SpotLight::range() const
{
    return mRange;
}
void SpotLight::setAngles(f32 innerDegrees, f32 outerDegrees)
{
    if (!std::isfinite(innerDegrees) || !std::isfinite(outerDegrees))
        return;
    mOuterAngle = glm::clamp(outerDegrees, 0.1f, 89.9f);
    mInnerAngle = glm::clamp(innerDegrees, 0.0f, mOuterAngle);
}
f32 SpotLight::innerAngle() const
{
    return mInnerAngle;
}
f32 SpotLight::outerAngle() const
{
    return mOuterAngle;
}
RectangleLight::RectangleLight() : Light(LightType::Rectangle)
{
}
void RectangleLight::setRange(f32 range)
{
    if (std::isfinite(range))
        mRange = validRange(range);
}
f32 RectangleLight::range() const
{
    return mRange;
}
void RectangleLight::setSize(f32 width, f32 height)
{
    if (std::isfinite(width) && std::isfinite(height))
    {
        mWidth = glm::max(0.01f, width);
        mHeight = glm::max(0.01f, height);
    }
}
f32 RectangleLight::width() const
{
    return mWidth;
}
f32 RectangleLight::height() const
{
    return mHeight;
}
void Light::setCastShadows(bool enabled)
{
    mCastShadows = enabled;
}
bool Light::castsShadows() const
{
    return mCastShadows;
}
void Light::setVolumetric(bool enabled)
{
    mVolumetric = enabled;
}
bool Light::volumetric() const
{
    return mVolumetric;
}

} // namespace Radion
