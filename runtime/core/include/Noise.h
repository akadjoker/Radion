#ifndef RADION_NOISE_H
#define RADION_NOISE_H

#include "Types.h"

#include "Math.h"

namespace Radion
{

// Deliberately independent of any optimised math library, so terrain
// generation is deterministic across platforms. Otherwise a world grown from
// one seed comes out different on another machine - and that is not a subtle
// difference, because these feed a cell hash where one bit changes everything
// downstream. It is also why the sine below is written out by hand rather than
// calling std::sin.
namespace Noise
{

// Classic Perlin over a 256-byte permutation table.
struct Perlin
{
    u8 state[256] = {};

    void initialise(u32 seed);

    // In [-1, 1].
    f32 compute(f32 x, f32 y, f32 z) const;

    // fBm: octaves at doubling frequency, amplitude falling by persistence.
    //
    // NOT normalised by the sum of the amplitudes, so with many octaves the
    // result can leave [-1, 1] - which is assumed by the saturate/smoothstep
    // downstream, and is why changing it here would change every terrain tuned
    // against these numbers.
    f32 compute(f32 x, f32 y, f32 z, u32 octaves, f32 persistence = 0.5f) const;
};

// F1 Voronoi: the nearest cell among the nine neighbours.
namespace Voronoi
{

// The sine, written out. Not for speed - for the determinism above: std::sin
// can differ in the last bit between compilers, and one bit is enough to move
// every cell. Degree-11 polynomial approximation, same coefficients as the
// reference.
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
