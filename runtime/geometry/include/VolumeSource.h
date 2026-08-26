#ifndef RADION_VOLUME_SOURCE_H
#define RADION_VOLUME_SOURCE_H

#include "Math.h"
#include "Types.h"

namespace Radion::Volume
{

// Positive density is inside the solid; the gradient points outwards.
struct Sample
{
    Math::Vec3 gradient{0.0f};
    f32 density = 0.0f;
};

class Source
{
public:
    virtual ~Source() = default;
    virtual f32 sampleDensity(const Math::Vec3& position) const = 0;
    virtual Sample sample(const Math::Vec3& position) const;
};

} // namespace Radion::Volume

#endif // RADION_VOLUME_SOURCE_H
