#include "PCH.h"

#include "dynamics/SoftSpring.h"

namespace Radion::Physics
{

void SoftSpring::calculate(f32 duration, f32 inverseEffectiveMass, f32 bias, f32 positionError,
                           f32 stiffness, f32 damping, f32& outEffectiveMass)
{
    if (stiffness <= 0.0f)
    {
        calculateHard(inverseEffectiveMass, bias, outEffectiveMass);
        return;
    }

    // Softness is gamma: divided by the step because this solver works in
    // impulses, not forces.
    mSoftness = 1.0f / (duration * (damping + duration * stiffness));

    // beta = dt*k / (c + dt*k), and the bias is beta/dt * C, which reduces
    // to dt * k * softness * C.
    mBias = bias + duration * stiffness * mSoftness * positionError;

    // The velocity constraint is J*v + softness*lambda + bias = 0, so
    // collecting lambda gives (J*M^-1*J^T + softness) * lambda = -(J*v + bias),
    // and the effective mass is the inverse of that.
    outEffectiveMass = 1.0f / (inverseEffectiveMass + mSoftness);
}

void SoftSpring::calculateHard(f32 inverseEffectiveMass, f32 bias, f32& outEffectiveMass)
{
    mSoftness = 0.0f;
    mBias = bias;
    outEffectiveMass = inverseEffectiveMass > 0.0f ? 1.0f / inverseEffectiveMass : 0.0f;
}

} // namespace Radion::Physics
