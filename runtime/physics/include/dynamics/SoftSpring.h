#ifndef RADION_PHYSICS_DYNAMICS_SOFTSPRING_H
#define RADION_PHYSICS_DYNAMICS_SOFTSPRING_H

#include "Types.h"

namespace Radion::Physics
{

// One constraint row turned into a spring, solved inside the solver instead
// of pushed with an external force.
//
// The difference is stability, and it is not a matter of degree: an external
// -k*x - c*v is integrated explicitly, so it only holds while k*dt^2/m stays
// small - a stiff spring at a coarse step gains energy every step until it
// blows up. Folding the spring into the constraint row solves it implicitly,
// which is unconditionally stable, so the stiffness can be whatever the
// machine actually is. It also means a spring costs nothing extra: it is the
// row the joint was already solving, made soft.
//
// The price, and it is worth naming: implicit integration carries damping of
// its own, so even a damping of zero does not oscillate forever.
//
// Usage, per row, per step:
//   setup:   calculate(...) -> effective mass for the row
//   solve:   lambda = -effectiveMass * (Jv + bias(totalLambda))
class SoftSpring
{
public:
    // `positionError` is the constraint equation C - how far the row is from
    // where the spring wants it. `stiffness` is k in N/m (or N*m/rad), and
    // `damping` is c in the same equation: F = -k*x - c*v. A stiffness of
    // zero leaves the row hard, and `damping` is then ignored.
    //
    // `inverseEffectiveMass` is J*M^-1*J^T for the row; `bias` is whatever
    // bias the row already had, which this adds to rather than replaces.
    void calculate(f32 duration, f32 inverseEffectiveMass, f32 bias, f32 positionError,
                   f32 stiffness, f32 damping, f32& outEffectiveMass);

    // Same row with no spring: hard, carrying only the bias it was given.
    void calculateHard(f32 inverseEffectiveMass, f32 bias, f32& outEffectiveMass);

    bool active() const
    {
        return mSoftness != 0.0f;
    }

    // The full bias for this row, given the impulse accumulated so far. The
    // softness term is what makes the row yield instead of holding: each
    // iteration acknowledges the impulse already applied rather than
    // pretending the row starts clean.
    f32 bias(f32 totalImpulse) const
    {
        return mSoftness * totalImpulse + mBias;
    }

private:
    f32 mBias = 0.0f;
    f32 mSoftness = 0.0f;
};

} // namespace Radion::Physics

#endif // RADION_PHYSICS_DYNAMICS_SOFTSPRING_H
