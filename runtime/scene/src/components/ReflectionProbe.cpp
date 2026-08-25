#include "PCH.h"

#include "ReflectionProbe.h"

#include "GameObject.h"

namespace Radion
{

ReflectionProbe::ReflectionProbe() : Component(Type, ComponentEventLateUpdate)
{
}

ReflectionProbe::~ReflectionProbe()
{
    mProbe.shutdown();
}

bool ReflectionProbe::create(u32 resolution)
{
    syncToOwner();
    return mProbe.create(resolution);
}

void ReflectionProbe::syncToOwner()
{
    GameObject* object = owner();
    if (!object)
        return;
    mProbe.position = object->globalPosition();
    // No automatic excludeObjectId: a probe is its own placeable object now
    // (Hierarchy > Create > Special Nodes > Reflection Probe), not
    // necessarily riding on the reflective mesh it serves, so there is no
    // single "self" this could safely guess at - a probe sitting inside a
    // room has nothing of its own to exclude. What keeps a mirror out of a
    // capture that would show its own back is MeshRenderer::
    // visibleInReflections on THAT object, checked once for every probe
    // (Scene::buildShadowList), not something this component owns.
}

EnvironmentProbe& ReflectionProbe::probe()
{
    return mProbe;
}
const EnvironmentProbe& ReflectionProbe::probe() const
{
    return mProbe;
}

void ReflectionProbe::onLateUpdate(f32)
{
    syncToOwner();
}

void ReflectionProbe::onDestroy()
{
    mProbe.shutdown();
}

} // namespace Radion
