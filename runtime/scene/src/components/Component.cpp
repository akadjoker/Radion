#include "PCH.h"

#include "Component.h"

namespace Radion
{

Component::Component(ComponentType type, u8 events) : mType(type), mEvents(events)
{
}
GameObject* Component::owner() const
{
    return mOwner;
}
ComponentType Component::type() const
{
    return mType;
}
u32 Component::id() const
{
    return mLocalId;
}
bool Component::active() const
{
    return mActive;
}
void Component::setActive(bool active)
{
    if (mActive == active)
        return;
    mActive = active;
    if (mOwner)
        active ? onEnable() : onDisable();
}
void Component::attached()
{
    onAwake();
    if (mActive)
        onEnable();
}
void Component::detached()
{
    if (mActive)
        onDisable();
    onDestroy();
}
void Component::onAwake()
{
}
void Component::onStart()
{
}
void Component::onEnable()
{
}
void Component::onDisable()
{
}
void Component::onUpdate(f32)
{
}
void Component::onLateUpdate(f32)
{
}
void Component::onDestroy()
{
}

} // namespace Radion
