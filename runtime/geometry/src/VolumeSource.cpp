#include "PCH.h"

#include "VolumeSource.h"

 

namespace Radion::Volume
{

Sample Source::sample(const glm::vec3& position) const
{
    constexpr f32 epsilon = 0.0005f;
    Sample result;
    result.density = sampleDensity(position);
    result.gradient = glm::vec3(
        sampleDensity(position + glm::vec3(epsilon, 0, 0)) - sampleDensity(position - glm::vec3(epsilon, 0, 0)),
        sampleDensity(position + glm::vec3(0, epsilon, 0)) - sampleDensity(position - glm::vec3(0, epsilon, 0)),
        sampleDensity(position + glm::vec3(0, 0, epsilon)) - sampleDensity(position - glm::vec3(0, 0, epsilon)));
    result.gradient /= 2.0f * epsilon;
    return result;
}

} // namespace Radion::Volume
