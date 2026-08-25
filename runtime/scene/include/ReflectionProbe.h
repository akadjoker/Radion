#ifndef RADION_REFLECTION_PROBE_H
#define RADION_REFLECTION_PROBE_H

#include "Component.h"
#include "EnvironmentProbe.h"

namespace Radion
{

// A placeable local reflection cubemap - what lets two cars in different
// parts of a scene (a showroom, a street) each pick up the reflections
// actually around them instead of sharing Engine's one global environment
// probe. Owns an EnvironmentProbe (render/) and keeps its position synced to
// this component's own GameObject; everything else (extents, intensity,
// refresh policy, content) is that EnvironmentProbe's own public fields -
// see its header - reached through probe() rather than mirrored here.
//
// Scene registers every live one (mirrors how it tracks lights/cameras) so
// buildRenderList() can pick the nearest per object and Engine::render() can
// drive every registered probe's capture, not just Engine's own mProbe.
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

    // Position and self-exclusion, taken from the GameObject this is on.
    // Called from create() as well as every late update: a probe attached
    // and captured inside the same frame would otherwise be captured before
    // its first late update ever ran - from the world origin, excluding
    // nothing - and an Automatic probe could otherwise keep that bad capture
    // until the next scene invalidation.
    void syncToOwner();

    EnvironmentProbe mProbe;
};

} // namespace Radion

#endif // RADION_REFLECTION_PROBE_H
