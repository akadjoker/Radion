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
    // No automatic excludeObjectId: a probe is its own placeable object, with no single "self" to guess. A mirror stays out of a capture via MeshRenderer::visibleInReflections on that object (checked in Scene::buildShadowList).
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
