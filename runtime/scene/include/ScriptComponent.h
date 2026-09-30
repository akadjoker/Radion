#ifndef RADION_SCRIPT_COMPONENT_H
#define RADION_SCRIPT_COMPONENT_H

#include "Component.h"

namespace Radion
{

// One per GameObject: every subclass registers under ComponentType::Script, so addComponent<T>() on an object with any script returns null.
// getComponent<T>() is unsafe for the same reason (casts on the slot alone); keep the pointer addComponent() returned.
class ScriptComponent : public Component
{
public:
    static constexpr ComponentType Type = ComponentType::Script;

    // Not requesting an event the subclass overrides means the override never runs.
    explicit ScriptComponent(u8 events = ComponentEventUpdate);

    // Runtime discriminator for the shared Script slot (like Light.h), without RTTI.
    virtual bool isZenBehaviour() const;
};

} // namespace Radion

#endif // RADION_SCRIPT_COMPONENT_H
