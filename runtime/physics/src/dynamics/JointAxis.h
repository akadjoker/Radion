#ifndef RADION_PHYSICS_DYNAMICS_JOINTAXIS_H
#define RADION_PHYSICS_DYNAMICS_JOINTAXIS_H

#include "Math.h"
#include "Types.h"

namespace Radion::Physics::detail
{

// Math::normalize(vec3(0)) is NaN, which spreads through frames and impulses into every touching body, silently.
// A zero axis is a caller mistake; fall back to a sane axis to keep it local to that joint (setters already refuse it).
inline Math::vec3 normalizedAxisOr(const Math::vec3& axis, const Math::vec3& fallback)
{
    const f32 length = Math::length(axis);
    if (length > 1.0e-6f && std::isfinite(length))
        return axis / length;
    return fallback;
}

} // namespace Radion::Physics::detail

#endif // RADION_PHYSICS_DYNAMICS_JOINTAXIS_H
