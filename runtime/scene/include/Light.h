#ifndef RADION_LIGHT_H
#define RADION_LIGHT_H

#include "Component.h"

#include <glm/glm.hpp>

namespace Radion
{

enum class LightType : u8
{
    Directional,
    Point,
    Spot,
    Rectangle
};

class Light : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Light;

    LightType lightType() const;
    void setColor(const glm::vec3& color);
    const glm::vec3& color() const;
    void setIntensity(f32 intensity);
    f32 intensity() const;
    void setCastShadows(bool enabled);
    bool castsShadows() const;
    void setVolumetric(bool enabled);
    bool volumetric() const;

protected:
    explicit Light(LightType type);

private:
    LightType mType;
    glm::vec3 mColor = glm::vec3(1.0f);
    f32 mIntensity = 1.0f;
    bool mCastShadows = false;
    bool mVolumetric = false;
};

class DirectionalLight final : public Light
{
public:
    static constexpr ComponentType Type = ComponentType::Light;

private:
    friend class GameObject;
    DirectionalLight();
};

class PointLight final : public Light
{
public:
    static constexpr ComponentType Type = ComponentType::Light;

    void setRange(f32 range);
    f32 range() const;

private:
    friend class GameObject;
    PointLight();

    f32 mRange = 10.0f;
};

class SpotLight final : public Light
{
public:
    static constexpr ComponentType Type = ComponentType::Light;

    void setRange(f32 range);
    f32 range() const;
    void setAngles(f32 innerDegrees, f32 outerDegrees);
    f32 innerAngle() const;
    f32 outerAngle() const;

private:
    friend class GameObject;
    SpotLight();

    f32 mRange = 10.0f;
    f32 mInnerAngle = 25.0f;
    f32 mOuterAngle = 35.0f;
};

class RectangleLight final : public Light
{
public:
    static constexpr ComponentType Type = ComponentType::Light;

    void setRange(f32 range);
    f32 range() const;
    void setSize(f32 width, f32 height);
    f32 width() const;
    f32 height() const;

private:
    friend class GameObject;
    RectangleLight();

    f32 mRange = 10.0f;
    f32 mWidth = 1.0f;
    f32 mHeight = 1.0f;
};

// All four register as ComponentType::Light and share one slot on the object,
// so the slot says a light is there but never which one. lightType() is the
// only thing that does - these are what let is<T>()/as<T>() ask it.
#define RADION_LIGHT_MATCH(Class, Kind)                                                            \
    template <> struct ComponentMatch<Class>                                                       \
    {                                                                                              \
        static bool test(const Component* component)                                               \
        {                                                                                          \
            return static_cast<const Light*>(component)->lightType() == LightType::Kind;            \
        }                                                                                          \
    };

RADION_LIGHT_MATCH(DirectionalLight, Directional)
RADION_LIGHT_MATCH(PointLight, Point)
RADION_LIGHT_MATCH(SpotLight, Spot)
RADION_LIGHT_MATCH(RectangleLight, Rectangle)

#undef RADION_LIGHT_MATCH

} // namespace Radion

#endif // RADION_LIGHT_H
