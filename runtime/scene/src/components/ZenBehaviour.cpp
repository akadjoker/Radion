#include "PCH.h"

#include "ZenBehaviour.h"

#include "GameObject.h"
#include "Scene.h"
#include "SceneScriptBindings.h"

#include "zen/object.h"
#include "zen/vm.h"

#include <cstring>

namespace Radion
{

static std::string currentErrorMessage(zen::VM& vm)
{
    zen::ObjFiber* fiber = vm.current_fiber();
    return (fiber && fiber->error) ? fiber->error->chars : "zen: runtime error";
}

ZenBehaviour::ZenBehaviour() : ScriptComponent(ComponentEventUpdate)
{
}

ZenBehaviour::~ZenBehaviour()
{
    releaseInstance();
}

void ZenBehaviour::releaseInstance()
{
    // At process exit the ScriptCache singleton may already be gone: it owns
    // the VM, so the instance this was holding died with it and there is
    // nothing left to unprotect.
    if (zen::is_instance(mInstance) && ScriptCache::alive())
        ScriptCache::getSingleton().unprotectInstance(mInstance);
    mInstance = zen::val_nil();
    mBoundVersion = 0;
    mStartCalled = false;
}

bool ZenBehaviour::loadFile(const std::string& path)
{
    releaseInstance();
    mScriptPath = path;
    mFromFile = true;

    std::string error;
    mEntry = ScriptCache::getSingleton().loadFile(path, error);
    mLoaded = mEntry != nullptr;
    if (!mLoaded)
    {
        fail(error);
        return false;
    }
    mFailed = false;
    mLastError.clear();
    return true;
}

bool ZenBehaviour::loadSource(const std::string& source)
{
    releaseInstance();
    mScriptPath.clear();
    mFromFile = false;

    std::string error;
    mEntry = ScriptCache::getSingleton().loadSource(source, error);
    mLoaded = mEntry != nullptr;
    if (!mLoaded)
    {
        fail(error);
        return false;
    }
    mFailed = false;
    mLastError.clear();
    return true;
}

bool ZenBehaviour::reload()
{
    if (!mFromFile || mScriptPath.empty())
        return false;

    std::string error;
    if (!ScriptCache::getSingleton().reloadFile(mScriptPath, error))
    {
        fail(error);
        return false;
    }
    mFailed = false;
    mLastError.clear();
    // The cache entry's version just bumped - the next ensureInstance()
    // (this component's or any sibling's sharing the same path) notices the
    // mismatch and re-instantiates on its own.
    return true;
}

void ZenBehaviour::fail(const std::string& error)
{
    mFailed = true;
    mLastError = error;
}

const std::string& ZenBehaviour::scriptPath() const
{
    return mScriptPath;
}

bool ZenBehaviour::hasError() const
{
    return mFailed;
}

const std::string& ZenBehaviour::lastError() const
{
    return mLastError;
}

bool ZenBehaviour::isZenBehaviour() const
{
    return true;
}

usize ZenBehaviour::declaredPropertyCount() const
{
    return mEntry ? mEntry->properties.size() : 0;
}

const ScriptProperty* ZenBehaviour::declaredPropertyAt(usize index) const
{
    if (!mEntry || index >= mEntry->properties.size())
        return nullptr;
    return &mEntry->properties[index];
}

const ScriptProperty* ZenBehaviour::declaredProperty(const std::string& name) const
{
    if (!mEntry)
        return nullptr;
    for (usize i = 0; i < mEntry->properties.size(); ++i)
        if (mEntry->properties[i].name == name)
            return &mEntry->properties[i];
    return nullptr;
}

usize ZenBehaviour::overrideCount() const
{
    return mOverrides.size();
}

const ScriptProperty* ZenBehaviour::overrideAt(usize index) const
{
    return index < mOverrides.size() ? &mOverrides[index] : nullptr;
}

const ScriptProperty* ZenBehaviour::findOverride(const std::string& name) const
{
    for (usize i = 0; i < mOverrides.size(); ++i)
        if (mOverrides[i].name == name)
            return &mOverrides[i];
    return nullptr;
}

ScriptProperty& ZenBehaviour::overrideSlot(const std::string& name)
{
    for (usize i = 0; i < mOverrides.size(); ++i)
        if (mOverrides[i].name == name)
            return mOverrides[i];

    ScriptProperty added;
    added.name = name;
    mOverrides.push_back(added);
    return mOverrides.back();
}

void ZenBehaviour::setNumberOverride(const std::string& name, f64 value, bool integer)
{
    if (name.empty())
        return;
    ScriptProperty& property = overrideSlot(name);
    property.kind = ScriptProperty::Kind::Number;
    property.number = value;
    property.integer = integer;
    applyOverrides();
}

void ZenBehaviour::setStringOverride(const std::string& name, const std::string& value)
{
    if (name.empty())
        return;
    ScriptProperty& property = overrideSlot(name);
    property.kind = ScriptProperty::Kind::String;
    property.text = value;
    applyOverrides();
}

void ZenBehaviour::setBoolOverride(const std::string& name, bool value)
{
    if (name.empty())
        return;
    ScriptProperty& property = overrideSlot(name);
    property.kind = ScriptProperty::Kind::Bool;
    property.flag = value;
    applyOverrides();
}

void ZenBehaviour::clearOverride(const std::string& name)
{
    bool removed = false;
    for (usize i = 0; i < mOverrides.size(); ++i)
    {
        if (mOverrides[i].name == name)
        {
            mOverrides.erase(mOverrides.begin() + (std::ptrdiff_t)i);
            removed = true;
            break;
        }
    }
    if (!removed)
        return;

    // Putting the declared default back is enough for a property the script
    // still declares. For one it no longer does, the field may not even
    // exist any more, so the instance is dropped and rebuilt from __init__.
    if (const ScriptProperty* declared = declaredProperty(name))
        writeProperty(*declared);
    else
        releaseInstance();
}

void ZenBehaviour::clearOverrides()
{
    if (mOverrides.empty())
        return;

    const std::vector<ScriptProperty> cleared(mOverrides);
    mOverrides.clear();

    for (usize i = 0; i < cleared.size(); ++i)
    {
        if (const ScriptProperty* declared = declaredProperty(cleared[i].name))
            writeProperty(*declared);
        else
            releaseInstance();
    }
}

usize ZenBehaviour::applyOverrides()
{
    usize applied = 0;
    for (usize i = 0; i < mOverrides.size(); ++i)
        if (writeProperty(mOverrides[i]))
            ++applied;
    return applied;
}

bool ZenBehaviour::writeProperty(const ScriptProperty& property)
{
    if (!zen::is_instance(mInstance) || !ScriptCache::alive())
        return false;

    zen::ObjInstance* instance = zen::as_instance(mInstance);
    if (!instance || !instance->klass || !instance->klass->field_names)
        return false;

    zen::ObjClass* klass = instance->klass;
    int field = -1;
    for (int i = 0; i < klass->num_fields; ++i)
    {
        if (klass->field_names[i] &&
            std::strcmp(klass->field_names[i]->chars, property.name.c_str()) == 0)
        {
            field = i;
            break;
        }
    }
    if (field < 0 || field >= instance->num_fields)
        return false;

    switch (property.kind)
    {
    case ScriptProperty::Kind::Number:
        instance->fields[field] = property.integer ? zen::val_int((s64)property.number)
                                                   : zen::val_float(property.number);
        break;
    case ScriptProperty::Kind::String:
    {
        zen::VM& vm = ScriptCache::getSingleton().vm();
        zen::ObjString* text = vm.make_string(property.text.c_str(), (int)property.text.size());
        instance->fields[field] = zen::val_obj((zen::Obj*)text);
        break;
    }
    case ScriptProperty::Kind::Bool:
        instance->fields[field] = zen::val_bool(property.flag);
        break;
    }
    return true;
}

bool ZenBehaviour::ensureInstance()
{
    GameObject* object = owner();
    if (!object || !mEntry)
        return false;

    if (zen::is_instance(mInstance) && mBoundVersion == mEntry->version)
        return true;

    if (zen::is_instance(mInstance))
        ScriptCache::getSingleton().unprotectInstance(mInstance);

    zen::VM& vm = ScriptCache::getSingleton().vm();
    mInstance = vm.make_instance(zen::as_class(mEntry->classValue));
    ScriptCache::getSingleton().protectInstance(mInstance);

    // make_instance() deliberately does not run __init__, so it is invoked
    // here through its own vtable slot. Binding the owner first is the one
    // departure from how the script is otherwise driven: the owner is a
    // field on the instance, not an argument, so "self.owner" has to already
    // be there for __init__ to be able to use it. Then the component's own
    // overrides go on top of the defaults __init__ just wrote.
    SceneScriptBindings::bindOwner(vm, mInstance, object);
    if (mEntry->initSlot >= 0)
    {
        vm.invoke(mInstance, mEntry->initSlot, nullptr, 0);
        if (vm.had_error())
        {
            fail(currentErrorMessage(vm));
            return false;
        }
    }
    applyOverrides();

    mBoundVersion = mEntry->version;
    mStartCalled = false;
    return true;
}

void ZenBehaviour::onUpdate(f32 deltaTime)
{
    if (!mLoaded || mFailed)
        return;

    GameObject* object = owner();
    if (!object || !object->scene() || object->scene()->runningInEditor())
        return;

    if (!ensureInstance())
        return;

    zen::VM& vm = ScriptCache::getSingleton().vm();

    if (!mStartCalled)
    {
        mStartCalled = true;
        if (mEntry->onStartSlot >= 0)
        {
            vm.invoke(mInstance, mEntry->onStartSlot, nullptr, 0);
            if (vm.had_error())
            {
                fail(currentErrorMessage(vm));
                return;
            }
        }
    }

    if (mEntry->onUpdateSlot < 0)
        return;

    zen::Value arg = zen::val_float(static_cast<f64>(deltaTime));
    vm.invoke(mInstance, mEntry->onUpdateSlot, &arg, 1);
    if (vm.had_error())
        fail(currentErrorMessage(vm));
}

void ZenBehaviour::onCollision(GameObject* other)
{
    if (!mLoaded || mFailed || !other)
        return;

    GameObject* object = owner();
    if (!object || !object->scene() || object->scene()->runningInEditor())
        return;

    if (!ensureInstance())
        return;

    if (mEntry->onCollisionSlot < 0)
        return;

    zen::VM& vm = ScriptCache::getSingleton().vm();
    zen::Value arg = SceneScriptBindings::wrapGameObject(vm, other);
    vm.invoke(mInstance, mEntry->onCollisionSlot, &arg, 1);
    if (vm.had_error())
        fail(currentErrorMessage(vm));
}

void ZenBehaviour::onDestroy()
{
    if (mLoaded && !mFailed && mStartCalled && mEntry && zen::is_instance(mInstance) &&
        mEntry->onDestroySlot >= 0 && ScriptCache::alive())
    {
        zen::VM& vm = ScriptCache::getSingleton().vm();
        vm.invoke(mInstance, mEntry->onDestroySlot, nullptr, 0);
        if (vm.had_error())
            fail(currentErrorMessage(vm));
    }
    releaseInstance();
}

} // namespace Radion
