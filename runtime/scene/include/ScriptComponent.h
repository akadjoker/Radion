#ifndef RADION_SCRIPT_COMPONENT_H
#define RADION_SCRIPT_COMPONENT_H

#include "Component.h"

namespace Radion
{

// Base class for behaviour written in C++ and attached to a GameObject: the
// game's own rules live in a subclass of this, not in the demo's main loop.
// Subclasses override whichever of Component's onStart/onUpdate/onLateUpdate/
// onEnable/onDisable/onDestroy hooks they need and reach their object through
// owner().
//
// One per GameObject. Every subclass registers under the same
// ComponentType::Script slot, so addComponent<T>() on an object that already
// carries any script returns null. A behaviour that needs several independent
// pieces splits them across child objects.
//
// For the same reason getComponent<T>() is unsafe here - it casts on the slot
// alone and would hand back a pointer to whatever script is attached, as the
// wrong type. Keep the pointer addComponent() returned.
class ScriptComponent : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Script;

    // Which per-frame hooks the subclass wants called. Asking for an event
    // it does not override only costs an empty call; not asking for one it
    // does override means the override never runs.
    explicit ScriptComponent(u8 events = ComponentEventUpdate);

    // Runtime discriminator for the shared Script slot, mirroring how
    // Light.h tells its four light kinds apart: several ScriptComponent
    // subclasses can occupy it, and this is what ComponentMatch<ZenBehaviour>
    // uses to find the Zen-driven one among them, without RTTI.
    virtual bool isZenBehaviour() const;
};

} // namespace Radion

#endif // RADION_SCRIPT_COMPONENT_H
