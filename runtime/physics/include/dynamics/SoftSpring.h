#ifndef RADION_PHYSICS_DYNAMICS_SOFTSPRING_H
#define RADION_PHYSICS_DYNAMICS_SOFTSPRING_H

#include "Types.h"

namespace Radion::Physics
{

// One constraint row turned into a spring and solved implicitly inside the solver: unconditionally stable, unlike an external -k*x - c*v
// (explicit; holds only while k*dt^2/m is small, else it gains energy until it blows up), and it costs nothing extra.
// Implicit integration carries damping of its own, so even zero damping does not oscillate forever.
// Usage, per row, per step:
//   setup:   calculate(...) -> effective mass for the row
//   solve:   lambda = -effectiveMass * (Jv + bias(totalLambda))
class SoftSpring
{
public:
    // `positionError` is the constraint C (distance from where the spring wants it); `stiffness` is k in N/m (or N*m/rad), `damping` is c: F = -k*x - c*v.
    // Zero stiffness leaves the row hard and ignores damping. `inverseEffectiveMass` is J*M^-1*J^T; `bias` is the row's existing bias, which this adds to.
    void calculate(f32 duration, f32 inverseEffectiveMass, f32 bias, f32 positionError,
                   f32 stiffness, f32 damping, f32& outEffectiveMass);

    void calculateHard(f32 inverseEffectiveMass, f32 bias, f32& outEffectiveMass);

    bool active() const
    {
        return mSoftness != 0.0f;
    }

    // Full bias given the impulse accumulated so far; the softness term makes the row yield, each iteration acknowledging the impulse already applied.
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
