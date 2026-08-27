#include "PCH.h"

#include "SceneScriptBindings.h"

#include "Camera.h"
#include "Component.h"
#include "GameObject.h"
#include "Light.h"
#include "Scene.h"
#include "ScriptCache.h"

#include "zen/memory.h"
#include "zen/module.h"
#include "zen/object.h"
#include "zen/value.h"
#include "zen/vm.h"

#include <glm/gtc/quaternion.hpp>

namespace Radion
{

static zen::ObjClass* findClass(zen::VM* vm, const char* name)
{
    const zen::Value value = vm->get_global(name);
    return zen::is_class(value) ? zen::as_class(value) : nullptr;
}

static zen::Value makeVec3(zen::VM* vm, const glm::vec3& v)
{
    zen::ObjClass* klass = findClass(vm, "Vec3");
    if (!klass)
        return zen::val_nil();
    const zen::Value instance = vm->make_instance(klass);
    zen::ObjInstance* inst = zen::as_instance(instance);
    inst->fields[0] = zen::val_float(v.x);
    inst->fields[1] = zen::val_float(v.y);
    inst->fields[2] = zen::val_float(v.z);
    return instance;
}

static glm::vec3 readVec3(zen::Value instance)
{
    zen::ObjInstance* inst = zen::as_instance(instance);
    return glm::vec3(static_cast<f32>(zen::to_number(inst->fields[0])),
                     static_cast<f32>(zen::to_number(inst->fields[1])),
                     static_cast<f32>(zen::to_number(inst->fields[2])));
}

static zen::Value makeGameObjectValue(zen::VM* vm, GameObject* object)
{
    zen::ObjClass* klass = findClass(vm, "GameObject");
    if (!klass || !object)
        return zen::val_nil();
    const zen::Value instance = vm->make_instance(klass);
    zen::as_instance(instance)->native_data = object;
    return instance;
}

// Native methods called through a script dot-call ("self.set_position(...)")
// follow the ClassBuilder convention: self sits one slot before the args
// array the VM hands the native function, i.e. args[-1].
static GameObject* selfGameObject(zen::Value* args)
{
    return static_cast<GameObject*>(zen::as_instance(args[-1])->native_data);
}

static Scene* selfScene(zen::Value* args)
{
    return static_cast<Scene*>(zen::as_instance(args[-1])->native_data);
}

// Component classes (Camera, Light, ...) are handed to instances through
// ScriptCache::instanceFor() rather than makeGameObjectValue()'s bare "new
// wrapper every call" - see registerComponentClasses() - so their self is a
// zen_instance_data<T> cast, matching componentFromSelf in the reference.
static Component* selfComponent(zen::Value* args)
{
    return zen::zen_instance_data<Component>(args[-1]);
}

static Camera* selfCamera(zen::Value* args)
{
    return zen::zen_instance_data<Camera>(args[-1]);
}

static Light* selfLight(zen::Value* args)
{
    return zen::zen_instance_data<Light>(args[-1]);
}

static int vec3Init(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    const f32 x = nargs > 0 ? static_cast<f32>(zen::to_number(args[0])) : 0.0f;
    const f32 y = nargs > 1 ? static_cast<f32>(zen::to_number(args[1])) : 0.0f;
    const f32 z = nargs > 2 ? static_cast<f32>(zen::to_number(args[2])) : 0.0f;
    zen::ObjInstance* inst = zen::as_instance(args[-1]);
    inst->fields[0] = zen::val_float(x);
    inst->fields[1] = zen::val_float(y);
    inst->fields[2] = zen::val_float(z);
    return 0;
}

static int vec3Length(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    const glm::vec3 v = readVec3(args[-1]);
    args[0] = zen::val_float(static_cast<f64>(glm::length(v)));
    return 1;
}

// Operator overloads (__add__/__sub__/__mul__) are dispatched through
// VM::invoke_operator(), not OP_INVOKE - there self sits at args[0] and the
// other operand at args[1], not the args[-1] convention above.
static int vec3Add(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    const glm::vec3 a = readVec3(args[0]);
    const glm::vec3 b = readVec3(args[1]);
    args[0] = makeVec3(vm, a + b);
    return 1;
}

static int vec3Sub(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    const glm::vec3 a = readVec3(args[0]);
    const glm::vec3 b = readVec3(args[1]);
    args[0] = makeVec3(vm, a - b);
    return 1;
}

static int vec3Mul(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    const glm::vec3 a = readVec3(args[0]);
    const f32 scalar = static_cast<f32>(zen::to_number(args[1]));
    args[0] = makeVec3(vm, a * scalar);
    return 1;
}

static int componentGetActive(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    Component* component = selfComponent(args);
    args[0] = zen::val_bool(component && component->active());
    return 1;
}

static int componentSetActive(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (Component* component = selfComponent(args))
        if (nargs >= 1)
            component->setActive(zen::is_truthy(args[0]));
    return 0;
}

static int goGetName(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    const std::string& name = selfGameObject(args)->name();
    args[0] = zen::val_obj((zen::Obj*)vm->make_string(name.c_str(), (int)name.size()));
    return 1;
}

static int goSetName(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (nargs >= 1 && zen::is_string(args[0]))
        selfGameObject(args)->setName(
            std::string(zen::safe_string_chars(args[0]), (usize)zen::safe_string_len(args[0])));
    return 0;
}

static int goGetActive(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    args[0] = zen::val_bool(selfGameObject(args)->active());
    return 1;
}

static int goSetActive(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (nargs >= 1 && zen::is_bool(args[0]))
        selfGameObject(args)->setActive(args[0].as.boolean);
    return 0;
}

static int goGetPosition(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    args[0] = makeVec3(vm, selfGameObject(args)->position());
    return 1;
}

static int goSetPosition(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (nargs >= 1 && zen::is_instance(args[0]))
        selfGameObject(args)->setPosition(readVec3(args[0]));
    return 0;
}

static int goGetScale(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    args[0] = makeVec3(vm, selfGameObject(args)->scale());
    return 1;
}

static int goSetScale(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (nargs >= 1 && zen::is_instance(args[0]))
        selfGameObject(args)->setScale(readVec3(args[0]));
    return 0;
}

static int goGetRotation(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    const glm::vec3 degrees = glm::degrees(glm::eulerAngles(selfGameObject(args)->rotation()));
    args[0] = makeVec3(vm, degrees);
    return 1;
}

static int goSetRotation(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (nargs >= 1 && zen::is_instance(args[0]))
        selfGameObject(args)->setRotationDegrees(readVec3(args[0]));
    return 0;
}

static int goYaw(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (nargs >= 1)
        selfGameObject(args)->yaw(static_cast<f32>(zen::to_number(args[0])));
    return 0;
}

static int goPitch(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (nargs >= 1)
        selfGameObject(args)->pitch(static_cast<f32>(zen::to_number(args[0])));
    return 0;
}

static int goRoll(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (nargs >= 1)
        selfGameObject(args)->roll(static_cast<f32>(zen::to_number(args[0])));
    return 0;
}

// The Zen compiler lowers node.get_component<Camera>() to node.get_component(Camera),
// so the first native argument is the requested host class, not a string -
// matching natNodeGetComponent in the reference exactly.
static int goGetComponent(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    GameObject* object = selfGameObject(args);
    if (!object || !ScriptCache::alive() || nargs < 1 || !zen::is_class(args[0]))
    {
        args[0] = zen::val_nil();
        return 1;
    }

    ScriptCache& cache = ScriptCache::getSingleton();
    zen::ObjClass* requested = zen::as_class(args[0]);
    const char* name = requested->name ? requested->name->chars : "";

    if (!std::strcmp(name, "Camera"))
        args[0] = cache.instanceFor(cache.cameraClass(), object->getComponent<Camera>());
    else if (!std::strcmp(name, "Light") || !std::strcmp(name, "DirectionalLight"))
        args[0] = cache.instanceFor(cache.lightClass(), object->getComponent<Light>());
    else
        args[0] = zen::val_nil();

    return 1;
}

static int sceneFind(zen::VM* vm, zen::Value* args, int nargs)
{
    Scene* scene = selfScene(args);
    if (!scene || nargs < 1 || !zen::is_string(args[0]))
    {
        args[0] = zen::val_nil();
        return 1;
    }
    const std::string name(zen::safe_string_chars(args[0]), (usize)zen::safe_string_len(args[0]));
    args[0] = makeGameObjectValue(vm, scene->findGameObject(name));
    return 1;
}

static int sceneCreate(zen::VM* vm, zen::Value* args, int nargs)
{
    Scene* scene = selfScene(args);
    if (!scene)
    {
        args[0] = zen::val_nil();
        return 1;
    }
    const std::string name = (nargs >= 1 && zen::is_string(args[0]))
        ? std::string(zen::safe_string_chars(args[0]), (usize)zen::safe_string_len(args[0]))
        : std::string();
    args[0] = makeGameObjectValue(vm, scene->createGameObject(name));
    return 1;
}

static void sceneScriptBindingsInit(zen::VM* vm)
{
    // Match the script-side shape used by Kinetix2D without importing its
    // 2D API: Radion scripts derive from ScriptComponent and receive their
    // 3D GameObject through self.node. Concrete Radion component handles
    // (Camera, Light, ...) derive from Component in
    // registerComponentClasses(), each with its native_data pointing at the
    // owning C++ component - the is_active/set_active pair below is theirs
    // for free.
    auto component = vm->def_class("Component");
    component.method("is_active", componentGetActive, 0);
    component.method("set_active", componentSetActive, 1);
    component.persistent(true).constructable(false).end();

    auto scriptComponent = vm->def_class("ScriptComponent");
    scriptComponent.parent("Component");
    scriptComponent.field("node");
    scriptComponent.constructable(false).persistent(false).end();

    vm->def_class("Vec3")
        .field("x")
        .field("y")
        .field("z")
        .method("__init__", vec3Init, 3)
        .method("length", vec3Length, 0)
        .method("__add__", vec3Add, 1)
        .method("__sub__", vec3Sub, 1)
        .method("__mul__", vec3Mul, 1)
        .constructable(true)
        .persistent(false)
        .end();

    // The GameObject/Scene wrappers carry nothing but a raw pointer and
    // define no native destructor, so they stay ordinary GC objects: a script
    // calling scene.find() on every frame drops one wrapper per frame and the
    // collector takes them. A persistent class would arena-allocate each one,
    // keep it out of the GC list and never free it.
    vm->def_class("GameObject")
        .method("get_name", goGetName, 0)
        .method("set_name", goSetName, 1)
        .method("get_active", goGetActive, 0)
        .method("set_active", goSetActive, 1)
        .method("get_position", goGetPosition, 0)
        .method("set_position", goSetPosition, 1)
        .method("get_scale", goGetScale, 0)
        .method("set_scale", goSetScale, 1)
        .method("get_rotation", goGetRotation, 0)
        .method("set_rotation", goSetRotation, 1)
        .method("yaw", goYaw, 1)
        .method("pitch", goPitch, 1)
        .method("roll", goRoll, 1)
        .method("get_component", goGetComponent, 1)
        .constructable(false)
        .persistent(false)
        .end();

    vm->def_class("Scene")
        .method("find", sceneFind, 1)
        .method("create", sceneCreate, 1)
        .constructable(false)
        .persistent(false)
        .end();
}

static int cameraGetFieldOfView(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    Camera* camera = selfCamera(args);
    args[0] = zen::val_float(camera ? (f64)camera->fieldOfView() : 0.0);
    return 1;
}

static int cameraSetPerspective(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (Camera* camera = selfCamera(args))
        if (nargs >= 4)
            camera->setPerspective((f32)zen::to_number(args[0]), (f32)zen::to_number(args[1]),
                                   (f32)zen::to_number(args[2]), (f32)zen::to_number(args[3]));
    return 0;
}

static int cameraGetOrthographicSize(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    Camera* camera = selfCamera(args);
    args[0] = zen::val_float(camera ? (f64)camera->orthographicSize() : 0.0);
    return 1;
}

static int cameraSetOrthographic(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (Camera* camera = selfCamera(args))
        if (nargs >= 4)
            camera->setOrthographic((f32)zen::to_number(args[0]), (f32)zen::to_number(args[1]),
                                    (f32)zen::to_number(args[2]), (f32)zen::to_number(args[3]));
    return 0;
}

static int cameraGetAspect(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    Camera* camera = selfCamera(args);
    args[0] = zen::val_float(camera ? (f64)camera->aspect() : 0.0);
    return 1;
}

static int cameraSetAspect(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (Camera* camera = selfCamera(args))
        if (nargs >= 1)
            camera->setAspect((f32)zen::to_number(args[0]));
    return 0;
}

static int cameraGetNearPlane(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    Camera* camera = selfCamera(args);
    args[0] = zen::val_float(camera ? (f64)camera->nearPlane() : 0.0);
    return 1;
}

static int cameraGetFarPlane(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    Camera* camera = selfCamera(args);
    args[0] = zen::val_float(camera ? (f64)camera->farPlane() : 0.0);
    return 1;
}

static int lightGetColor(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    Light* light = selfLight(args);
    args[0] = light ? makeVec3(vm, light->color()) : zen::val_nil();
    return 1;
}

static int lightSetColor(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (Light* light = selfLight(args))
        if (nargs >= 1 && zen::is_instance(args[0]))
            light->setColor(readVec3(args[0]));
    return 0;
}

static int lightGetIntensity(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    Light* light = selfLight(args);
    args[0] = zen::val_float(light ? (f64)light->intensity() : 0.0);
    return 1;
}

static int lightSetIntensity(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (Light* light = selfLight(args))
        if (nargs >= 1)
            light->setIntensity((f32)zen::to_number(args[0]));
    return 0;
}

static int lightGetCastsShadows(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    Light* light = selfLight(args);
    args[0] = zen::val_bool(light && light->castsShadows());
    return 1;
}

static int lightSetCastShadows(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (Light* light = selfLight(args))
        if (nargs >= 1)
            light->setCastShadows(zen::is_truthy(args[0]));
    return 0;
}

static int lightGetVolumetric(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    Light* light = selfLight(args);
    args[0] = zen::val_bool(light && light->volumetric());
    return 1;
}

static int lightSetVolumetric(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (Light* light = selfLight(args))
        if (nargs >= 1)
            light->setVolumetric(zen::is_truthy(args[0]));
    return 0;
}

void SceneScriptBindings::registerComponentClasses(ScriptCache& cache)
{
    zen::VM& vm = cache.vm();

    auto camera = vm.def_class("Camera");
    camera.parent("Component");
    camera.method("get_field_of_view", cameraGetFieldOfView, 0);
    camera.method("set_perspective", cameraSetPerspective, 4);
    camera.method("get_orthographic_size", cameraGetOrthographicSize, 0);
    camera.method("set_orthographic", cameraSetOrthographic, 4);
    camera.method("get_aspect", cameraGetAspect, 0);
    camera.method("set_aspect", cameraSetAspect, 1);
    camera.method("get_near_plane", cameraGetNearPlane, 0);
    camera.method("get_far_plane", cameraGetFarPlane, 0);
    camera.persistent(true).constructable(false);
    cache.setCameraClass(camera.end());

    auto light = vm.def_class("Light");
    light.parent("Component");
    light.method("get_color", lightGetColor, 0);
    light.method("set_color", lightSetColor, 1);
    light.method("get_intensity", lightGetIntensity, 0);
    light.method("set_intensity", lightSetIntensity, 1);
    light.method("get_casts_shadows", lightGetCastsShadows, 0);
    light.method("set_cast_shadows", lightSetCastShadows, 1);
    light.method("get_volumetric", lightGetVolumetric, 0);
    light.method("set_volumetric", lightSetVolumetric, 1);
    light.persistent(true).constructable(false);
    cache.setLightClass(light.end());
}

static const zen::NativeLib kSceneScriptLib = {
    "radion_scene",
    nullptr,
    0,
    nullptr,
    0,
    sceneScriptBindingsInit,
};

const zen::NativeLib& SceneScriptBindings::library()
{
    return kSceneScriptLib;
}

// Mirrors OP_SETFIELD's instance branch in vm_dispatch.cpp: search this
// instance's own fields first, then the class's known field names, and
// grow both the class's field_names and this instance's fields array by one
// the first time a name is seen. There is no public "set a script instance
// field from C++" call in the VM, so this is the smallest faithful copy of
// what the bytecode itself does for "self.owner = ...".
static void setInstanceField(zen::VM* vm, zen::Value instance, const char* name, zen::Value value)
{
    zen::ObjInstance* inst = zen::as_instance(instance);
    zen::ObjClass* klass = inst->klass;
    zen::ObjString* key = vm->make_string(name);

    for (int32_t i = 0; i < inst->num_fields; ++i)
    {
        if (klass->field_names[i] == key)
        {
            inst->fields[i] = value;
            return;
        }
    }

    int32_t classIndex = -1;
    for (int32_t i = 0; i < klass->num_fields; ++i)
    {
        if (klass->field_names[i] == key)
        {
            classIndex = i;
            break;
        }
    }
    if (classIndex < 0)
    {
        classIndex = klass->num_fields++;
        klass->field_names = (zen::ObjString**)zen::zen_realloc(
            &vm->get_gc(), klass->field_names, sizeof(zen::ObjString*) * classIndex,
            sizeof(zen::ObjString*) * (classIndex + 1));
        klass->field_names[classIndex] = key;
    }

    const int32_t oldCount = inst->num_fields;
    const int32_t newCount = classIndex + 1;
    inst->fields = (zen::Value*)zen::zen_realloc(&vm->get_gc(), inst->fields,
                                                 sizeof(zen::Value) * oldCount,
                                                 sizeof(zen::Value) * newCount);
    for (int32_t i = oldCount; i < newCount; ++i)
        inst->fields[i] = zen::val_nil();
    inst->fields[classIndex] = value;
    inst->num_fields = newCount;
}

void SceneScriptBindings::bindOwner(zen::VM& vm, zen::Value instance, GameObject* owner)
{
    if (!owner || !zen::is_instance(instance))
        return;

    zen::ObjClass* gameObjectClass = findClass(&vm, "GameObject");
    zen::ObjClass* sceneClass = findClass(&vm, "Scene");
    if (!gameObjectClass || !sceneClass)
        return;

    const zen::Value nodeValue = vm.make_instance(gameObjectClass);
    zen::as_instance(nodeValue)->native_data = owner;
    // `node` is the public behaviour API, matching Kinetix2D. `owner` is
    // retained as an alias for existing Radion scripts, and both fields point
    // at the same lightweight wrapper rather than allocating two every
    // script instance.
    setInstanceField(&vm, instance, "node", nodeValue);
    setInstanceField(&vm, instance, "owner", nodeValue);

    const zen::Value sceneValue = vm.make_instance(sceneClass);
    zen::as_instance(sceneValue)->native_data = owner->scene();
    setInstanceField(&vm, instance, "scene", sceneValue);
}

zen::Value SceneScriptBindings::wrapGameObject(zen::VM& vm, GameObject* object)
{
    return makeGameObjectValue(&vm, object);
}

} // namespace Radion
