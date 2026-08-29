#ifndef RADION_PHYSICS_DYNAMICS_JOINTAXIS_H
#define RADION_PHYSICS_DYNAMICS_JOINTAXIS_H

// JointAxis.h - one place to turn an authored axis into a unit vector.
// Internal to the joint implementations, not a public header.

#include "Math.h"
#include "Types.h"

namespace Radion::Physics::detail
{

// glm::normalize(vec3(0)) is NaN, and a NaN axis does not stay put: it goes
// into the joint's frames, out through the impulses into both bodies, and
// from there into every body they touch - the whole simulation, silently,
// with no error anywhere and nothing to point at afterwards.
//
// The setters already refused a degenerate axis; the constructors and
// configure() did not, so the C++ path had a hole the editor path did not.
// This closes it in the one direction that matters: a zero axis is a caller
// mistake, and falling back to a sane one keeps it a mistake in that one
// joint instead of the end of the simulation.
inline glm::vec3 normalizedAxisOr(const glm::vec3& axis, const glm::vec3& fallback)
{
    const f32 length = glm::length(axis);
    if (length > 1.0e-6f && std::isfinite(length))
        return axis / length;
    return fallback;
}

} // namespace Radion::Physics::detail

#endif // RADION_PHYSICS_DYNAMICS_JOINTAXIS_H
