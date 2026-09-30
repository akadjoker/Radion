#ifndef RADION_REFLECTION_PROBE_H
#define RADION_REFLECTION_PROBE_H

#include "Component.h"
#include "EnvironmentProbe.h"

namespace Radion
{

// Everything but position lives on EnvironmentProbe's public fields, reached through probe().
class ReflectionProbe final : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::ReflectionProbe;

    bool create(u32 resolution = 128);

    EnvironmentProbe& probe();
    const EnvironmentProbe& probe() const;

private:
    friend class GameObject;

    ReflectionProbe();
    ~ReflectionProbe() override;

    void onLateUpdate(f32 deltaTime) override;
    void onDestroy() override;

    // Also called from create(): a probe captured in its first frame would otherwise capture from the world origin.
    void syncToOwner();

    EnvironmentProbe mProbe;
};

} // namespace Radion

#endif // RADION_REFLECTION_PROBE_H
