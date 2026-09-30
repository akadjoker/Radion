#ifndef RADION_AI_INTERNAL_H
#define RADION_AI_INTERNAL_H

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include "Math.h"

namespace Radion::AI::detail
{

inline Math::vec3 safeNormalize(const Math::vec3& v)
{
    float len = Math::length(v);
    if (len <= 0.0f)
        return Math::vec3(0.0f);
    return v / len;
}

inline Math::vec3 parallelComponent(const Math::vec3& v, const Math::vec3& unitBasis)
{
    return unitBasis * Math::dot(v, unitBasis);
}

inline Math::vec3 perpendicularComponent(const Math::vec3& v, const Math::vec3& unitBasis)
{
    return v - parallelComponent(v, unitBasis);
}

inline Math::vec3 truncateLength(const Math::vec3& v, float maxLength)
{
    float maxLengthSquared = maxLength * maxLength;
    float vecLengthSquared = Math::dot(v, v);
    if (vecLengthSquared <= maxLengthSquared)
        return v;
    return v * (maxLength / std::sqrt(vecLengthSquared));
}

inline float frandom01()
{
    return static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
}

inline float clip(float x, float min, float max)
{
    if (x < min)
        return min;
    if (x > max)
        return max;
    return x;
}

// Classify x relative to the interval [lowerBound, upperBound]:
// -1 below, 0 inside, +1 above.
inline int intervalComparison(float x, float lowerBound, float upperBound)
{
    if (x < lowerBound)
        return -1;
    if (x > upperBound)
        return +1;
    return 0;
}

inline float scalarRandomWalk(float initial, float walkspeed, float min, float max)
{
    const float next = initial + (((frandom01() * 2.0f) - 1.0f) * walkspeed);
    return std::clamp(next, min, max);
}

} // namespace Radion::AI::detail

#endif // RADION_AI_INTERNAL_H
