#ifndef RADION_NOISE_H
#define RADION_NOISE_H

#include "Types.h"

#include "Math.h"

namespace Radion
{

// Independent of optimised math libraries for cross-platform determinism: one seed must give the
// same world everywhere, so the sine is hand-written instead of std::sin.
namespace Noise
{

struct Perlin
{
    u8 state[256] = {};

    void initialise(u32 seed);

    // In [-1, 1].
    f32 compute(f32 x, f32 y, f32 z) const;

    // fBm: octaves at doubling frequency, amplitude falling by persistence. NOT normalised by the
    // amplitude sum; downstream terrain is tuned to that.
    f32 compute(f32 x, f32 y, f32 z, u32 octaves, f32 persistence = 0.5f) const;
};

// F1 Voronoi: the nearest cell among the nine neighbours.
namespace Voronoi
{

// Hand-written sine for determinism (std::sin can differ in the last bit across compilers). Degree-11 polynomial.
f32 computeSin(f32 x);

struct Result
{
    f32 distance = 0.0f;
    f32 cellId = 0.0f;
};

Result compute(f32 x, f32 y, f32 seed);

} // namespace Voronoi

} // namespace Noise

} // namespace Radion

#endif // RADION_NOISE_H
