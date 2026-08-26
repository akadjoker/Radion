#include "PCH.h"

#include "Noise.h"

#include <cmath>

namespace Radion
{
namespace Noise
{

void Perlin::initialise(u32 seed)
{
    // A xorshift32 rather than a general-purpose RNG: what matters is filling
    // the table deterministically, not the exact distribution.
    u32 s = seed ? seed : 0x9E3779B9u;
    for (u32 i = 0; i < 256; ++i)
    {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        state[i] = static_cast<u8>(s);
    }
}

namespace
{

f32 fade(f32 t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

f32 gradient(u8 hash, f32 x, f32 y, f32 z)
{
    const u8 h = hash & 15;
    const f32 u = h < 8 ? x : y;
    const f32 v = h < 4 ? y : (h == 12 || h == 14) ? x : z;
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

} // namespace

f32 Perlin::compute(f32 x, f32 y, f32 z) const
{
    const f32 flooredX = std::floor(x);
    const f32 flooredY = std::floor(y);
    const f32 flooredZ = std::floor(z);
    const s32 ix = static_cast<s32>(flooredX) & 255;
    const s32 iy = static_cast<s32>(flooredY) & 255;
    const s32 iz = static_cast<s32>(flooredZ) & 255;
    const f32 fx = x - flooredX;
    const f32 fy = y - flooredY;
    const f32 fz = z - flooredZ;
    const f32 u = fade(fx);
    const f32 v = fade(fy);
    const f32 w = fade(fz);

    const u8 a = static_cast<u8>((state[ix & 255] + iy) & 255);
    const u8 b = static_cast<u8>((state[(ix + 1) & 255] + iy) & 255);

    const u8 aa = static_cast<u8>((state[a] + iz) & 255);
    const u8 ab = static_cast<u8>((state[(a + 1) & 255] + iz) & 255);
    const u8 ba = static_cast<u8>((state[b] + iz) & 255);
    const u8 bb = static_cast<u8>((state[(b + 1) & 255] + iz) & 255);

    const f32 p0 = gradient(state[aa], fx, fy, fz);
    const f32 p1 = gradient(state[ba], fx - 1.0f, fy, fz);
    const f32 p2 = gradient(state[ab], fx, fy - 1.0f, fz);
    const f32 p3 = gradient(state[bb], fx - 1.0f, fy - 1.0f, fz);
    const f32 p4 = gradient(state[(aa + 1) & 255], fx, fy, fz - 1.0f);
    const f32 p5 = gradient(state[(ba + 1) & 255], fx - 1.0f, fy, fz - 1.0f);
    const f32 p6 = gradient(state[(ab + 1) & 255], fx, fy - 1.0f, fz - 1.0f);
    const f32 p7 = gradient(state[(bb + 1) & 255], fx - 1.0f, fy - 1.0f, fz - 1.0f);

    const f32 q0 = Math::mix(p0, p1, u);
    const f32 q1 = Math::mix(p2, p3, u);
    const f32 q2 = Math::mix(p4, p5, u);
    const f32 q3 = Math::mix(p6, p7, u);

    const f32 r0 = Math::mix(q0, q1, v);
    const f32 r1 = Math::mix(q2, q3, v);

    return Math::mix(r0, r1, w);
}

f32 Perlin::compute(f32 x, f32 y, f32 z, u32 octaves, f32 persistence) const
{
    f32 result = 0.0f;
    f32 amplitude = 1.0f;
    for (u32 i = 0; i < octaves; ++i)
    {
        result += compute(x, y, z) * amplitude;
        x *= 2.0f;
        y *= 2.0f;
        z *= 2.0f;
        amplitude *= persistence;
    }
    return result;
}

namespace Voronoi
{

f32 computeSin(f32 x)
{
    constexpr f32 kPi = 3.141592654f;
    constexpr f32 kHalfPi = 1.570796327f;
    constexpr f32 c1 = -0.16666667f;
    constexpr f32 c2 = 0.0083333310f;
    constexpr f32 c3 = -0.00019840874f;
    constexpr f32 c4 = 2.7525562e-06f;
    constexpr f32 c5 = -2.3889859e-08f;

    // Reduce the angle to [-pi, pi].
    const f32 turns = std::round(x * 0.159154943f);
    x = x - turns * 6.283185307f;

    const f32 signedPi = std::copysign(kPi, x);
    const f32 absolute = std::abs(x);
    const f32 reflected = signedPi - x;
    x = (absolute <= kHalfPi) ? x : reflected;

    const f32 x2 = x * x;
    f32 r = c5 * x2 + c4;
    r = r * x2 + c3;
    r = r * x2 + c2;
    r = r * x2 + c1;
    r = r * x2 + 1.0f;
    return r * x;
}

namespace
{

Math::vec2 hash(Math::vec2 p)
{
    Math::vec2 r(Math::dot(p, Math::vec2(127.1f, 311.7f)),
                Math::dot(p, Math::vec2(269.5f, 183.3f)));
    r.x = computeSin(r.x);
    r.y = computeSin(r.y);
    r *= 18.5453f;
    return Math::vec2(r.x - std::floor(r.x), r.y - std::floor(r.y));
}

} // namespace

Result compute(f32 x, f32 y, f32 seed)
{
    const Math::vec2 p(x, y);
    const Math::vec2 n(std::floor(p.x), std::floor(p.y));
    const Math::vec2 f = p - n;

    Math::vec3 best(8.0f, 0.0f, 0.0f); // x = best squared distance
    for (s32 j = -1; j <= 1; ++j)
    {
        for (s32 i = -1; i <= 1; ++i)
        {
            const Math::vec2 g{static_cast<f32>(i), static_cast<f32>(j)};
            const Math::vec2 o = hash(n + g);
            // The cell's point is not at its centre: it is displaced by the
            // hash, and the seed enters HERE, inside the sine, not in the hash.
            // That is what makes the same pattern of cells give different
            // terrains.
            const Math::vec2 r(g.x - f.x + (0.5f + 0.5f * computeSin(seed * o.x)),
                              g.y - f.y + (0.5f + 0.5f * computeSin(seed * o.y)));
            const f32 d = Math::dot(r, r);
            if (d < best.x)
                best = Math::vec3(d, o.x, o.y);
        }
    }

    Result result;
    result.distance = std::sqrt(best.x);
    result.cellId = best.y + best.z;
    return result;
}

} // namespace Voronoi

} // namespace Noise
} // namespace Radion
