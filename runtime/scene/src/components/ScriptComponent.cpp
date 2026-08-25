#include "PCH.h"

#include "ScriptComponent.h"

namespace Radion
{

ScriptComponent::ScriptComponent(u8 events) : Component(Type, events)
{
}

bool ScriptComponent::isZenBehaviour() const
{
    return false;
}

} // namespace Radion
