#include "PCH.h"

#include "Color.h"

#include <algorithm>
#include <cmath>

namespace Radion
{

const u32 Color::White = 0xFFFFFFFF;
const u32 Color::Black = 0xFF000000;
const u32 Color::Red = 0xFFFF0000;
const u32 Color::Green = 0xFF00FF00;
const u32 Color::Blue = 0xFF0000FF;
const u32 Color::Yellow = 0xFFFFFF00;
const u32 Color::Cyan = 0xFF00FFFF;
const u32 Color::Magenta = 0xFFFF00FF;
const u32 Color::Orange = 0xFFFFA500;
const u32 Color::Gray = 0xFF808080;
const u32 Color::Transparent = 0x00000000;

static inline int iclamp(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float fclamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

Color::Color(u8 r, u8 g, u8 b, u8 a)
{
    mValue = (u32(a) << 24) | (u32(r) << 16) | (u32(g) << 8) | u32(b);
}

void Color::setRGB(u8 R, u8 G, u8 B)
{
    mValue = (mValue & 0xFF000000) | (u32(R) << 16) | (u32(G) << 8) | u32(B);
}

void Color::setRGBA(u8 R, u8 G, u8 B, u8 A)
{
    mValue = (u32(A) << 24) | (u32(R) << 16) | (u32(G) << 8) | u32(B);
}

Color Color::withAlpha(float a) const
{
    u32 alpha;
    if (a <= 0.0f)
        alpha = 0;
    else if (a >= 1.0f)
        alpha = 0xFF;
    else
        alpha = u32(0xFF * a);

    return Color((mValue & 0x00FFFFFF) | (alpha << 24));
}

float Color::getHue() const
{
    int h = r();
    int s = g();
    int v = b();

    int maxV = std::max(h, std::max(s, v));
    int minV = std::min(h, std::min(s, v));

    float hue = 0.0f;

    if (maxV == minV)
        hue = 0.0f;
    else if (maxV == h)
        hue = std::fmod(60.0f * (s - v) / (maxV - minV) + 360.0f, 360.0f);
    else if (maxV == s)
        hue = 60.0f * (v - h) / (maxV - minV) + 120.0f;
    else if (maxV == v)
        hue = 60.0f * (h - s) / (maxV - minV) + 240.0f;

    return hue / 360.0f;
}

float Color::getSaturation() const
{
    int h = r();
    int s = g();
    int v = b();

    int maxV = std::max(h, std::max(s, v));
    if (maxV == 0)
        return 0.0f;

    int minV = std::min(h, std::min(s, v));
    return float(maxV - minV) / float(maxV);
}

float Color::getValue() const
{
    int h = r();
    int s = g();
    int v = b();
    return std::max(h, std::max(s, v)) / 255.0f;
}

float Color::getLuminance() const
{
    return 0.2126f * red() + 0.7152f * green() + 0.0722f * blue();
}

Color Color::lerp(const Color& to, float t) const
{
    return Color::lerp(*this, to, t);
}

Color Color::multiply(const Color& other) const
{
    return Color::fromRGBFloat(red() * other.red(), green() * other.green(), blue() * other.blue(),
                               alpha());
}

u32 Color::toARGB(float alpha) const
{
    alpha = fclamp(alpha, 0.0f, 1.0f);
    return (u32(0xFF * alpha) << 24) | (mValue & 0x00FFFFFF);
}

Color Color::fromRGB(u8 r, u8 g, u8 b, u8 a)
{
    return Color(r, g, b, a);
}

Color Color::fromRGBFloat(float r, float g, float b, float a)
{
    auto intColor = [](float v) -> u8
    {
        return (u8)iclamp((int)(v * 256.0f), 0, 255);
    };
    return Color(intColor(r), intColor(g), intColor(b), intColor(a));
}

Color Color::fromHSV(float h, float s, float v, float a)
{
    h = std::fmod(h * 360.0f, 360.0f);
    if (h < 0.0f)
        h += 360.0f;

    int hi = (int)(h / 60.0f) % 6;
    float f = h / 60.0f - std::floor(h / 60.0f);
    float p = v * (1.0f - s);
    float q = v * (1.0f - f * s);
    float t = v * (1.0f - (1.0f - f) * s);

    float rf = 0, gf = 0, bf = 0;
    switch (hi)
    {
    case 0:
        rf = v;
        gf = t;
        bf = p;
        break;
    case 1:
        rf = q;
        gf = v;
        bf = p;
        break;
    case 2:
        rf = p;
        gf = v;
        bf = t;
        break;
    case 3:
        rf = p;
        gf = q;
        bf = v;
        break;
    case 4:
        rf = t;
        gf = p;
        bf = v;
        break;
    case 5:
        rf = v;
        gf = p;
        bf = q;
        break;
    }

    u8 R = (u8)(rf * 255.0f);
    u8 G = (u8)(gf * 255.0f);
    u8 B = (u8)(bf * 255.0f);
    u8 A = (u8)(fclamp(a, 0.0f, 1.0f) * 255.0f);
    return Color(R, G, B, A);
}

Color Color::lerp(Color from, Color to, float t)
{
    if (t <= 0.0f)
        return from;
    if (t >= 1.0f)
        return to;

    int a = from.a();
    int r = from.r();
    int g = from.g();
    int b = from.b();

    int dA = (int)to.a() - a;
    int dR = (int)to.r() - r;
    int dG = (int)to.g() - g;
    int dB = (int)to.b() - b;

    a += (int)(dA * t);
    r += (int)(dR * t);
    g += (int)(dG * t);
    b += (int)(dB * t);

    return Color((u8)r, (u8)g, (u8)b, (u8)a);
}

} // namespace Radion