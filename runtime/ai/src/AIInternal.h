#ifndef RADION_AI_INTERNAL_H
#define RADION_AI_INTERNAL_H

// AIInternal.h - small math helpers shared by the AI implementation files.
// Not part of the public API (not installed with the public headers).

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <glm/glm.hpp>

namespace Radion::AI::detail
{

inline glm::vec3 safeNormalize(const glm::vec3& v)
{
    float len = glm::length(v);
    if (len <= 0.0f)
        return glm::vec3(0.0f);
    return v / len;
}

// Component of v parallel to a unit basis vector.
inline glm::vec3 parallelComponent(const glm::vec3& v, const glm::vec3& unitBasis)
{
    return unitBasis * glm::dot(v, unitBasis);
}

// Component of v perpendicular to a unit basis vector.
inline glm::vec3 perpendicularComponent(const glm::vec3& v, const glm::vec3& unitBasis)
{
    return v - parallelComponent(v, unitBasis);
}

// Clamps the length of v to maxLength.
inline glm::vec3 truncateLength(const glm::vec3& v, float maxLength)
{
    float maxLengthSquared = maxLength * maxLength;
    float vecLengthSquared = glm::dot(v, v);
    if (vecLengthSquared <= maxLengthSquared)
        return v;
    return v * (maxLength / std::sqrt(vecLengthSquared));
}

// Uniform random float in [0,1].
inline float frandom01()
{
    return static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
}

// Constrain x to [min, max].
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

// Random walk of `initial` by at most `walkspeed`, clamped to [min, max].
inline float scalarRandomWalk(float initial, float walkspeed, float min, float max)
{
    const float next = initial + (((frandom01() * 2.0f) - 1.0f) * walkspeed);
    return std::clamp(next, min, max);
}

} // namespace Radion::AI::detail

#endif // RADION_AI_INTERNAL_H
