#ifndef RADION_COLOR_H
#define RADION_COLOR_H

#include "Types.h"

namespace Radion
{

// 32-bit color packed as 0xAARRGGBB (ARGB)
class Color
{
public:
    static const u32 White;
    static const u32 Black;
    static const u32 Red;
    static const u32 Green;
    static const u32 Blue;
    static const u32 Yellow;
    static const u32 Cyan;
    static const u32 Magenta;
    static const u32 Orange;
    static const u32 Gray;
    static const u32 Transparent;

    Color() : mValue(0xFFFFFFFF)
    {
    }
    Color(u32 argb) : mValue(argb)
    {
    }
    Color(u8 r, u8 g, u8 b, u8 a = 255);

    u32 value() const
    {
        return mValue;
    }
    void setValue(u32 v)
    {
        mValue = v;
    }

    u8 r() const
    {
        return (u8)((mValue >> 16) & 0xFF);
    }
    u8 g() const
    {
        return (u8)((mValue >> 8) & 0xFF);
    }
    u8 b() const
    {
        return (u8)(mValue & 0xFF);
    }
    u8 a() const
    {
        return (u8)((mValue >> 24) & 0xFF);
    }

    float red() const
    {
        return r() / 255.0f;
    }
    float green() const
    {
        return g() / 255.0f;
    }
    float blue() const
    {
        return b() / 255.0f;
    }
    float alpha() const
    {
        return a() / 255.0f;
    }

    void setRGB(u8 R, u8 G, u8 B);
    void setRGBA(u8 R, u8 G, u8 B, u8 A);
    Color withAlpha(float a) const;

    float getHue() const;
    float getSaturation() const;
    float getValue() const;
    float getLuminance() const;

    Color lerp(const Color& to, float t) const;
    Color multiply(const Color& other) const;

    u32 toARGB(float alpha) const;

    static Color fromRGB(u8 r, u8 g, u8 b, u8 a = 255);
    static Color fromRGBFloat(float r, float g, float b, float a = 1.0f);
    static Color fromHSV(float h, float s, float v, float a = 1.0f);
    static Color lerp(Color from, Color to, float t);

    bool operator==(const Color& o) const
    {
        return mValue == o.mValue;
    }
    bool operator!=(const Color& o) const
    {
        return mValue != o.mValue;
    }
    operator u32() const
    {
        return mValue;
    }

private:
    u32 mValue;
};

} // namespace Radion

#endif // RADION_COLOR_H
