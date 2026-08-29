#include "PCH.h"

#include "SceneScriptBindings.h"

#include "Animation.h"
#include "AssetManager.h"
#include "Camera.h"
#include "CharacterController.h"
#include "Component.h"
#include "GameObject.h"
#include "GPU.h"
#include "Light.h"
#include "MeshRenderer.h"
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

// Every component handle and wrapper class registered here declares zero
// fields, and new_instance() leaves the field array null for those, so an
// is_instance() check alone is not enough to read x/y/z off a value: a script
// passing self.node where a Vec3 belongs would dereference null.
static bool isVec3Instance(zen::Value value)
{
    return zen::is_instance(value) && zen::as_instance(value)->num_fields >= 3;
}

static glm::vec3 readVec3(zen::Value instance)
{
    if (!isVec3Instance(instance))
        return glm::vec3(0.0f);
    zen::ObjInstance* inst = zen::as_instance(instance);
    return glm::vec3(static_cast<f32>(zen::to_number(inst->fields[0])),
                     static_cast<f32>(zen::to_number(inst->fields[1])),
                     static_cast<f32>(zen::to_number(inst->fields[2])));
}

// CharacterController::move() returns its four fields by value rather than
// through the component's own getters (those only change in onUpdate()), so
// the whole result has to cross into script as one instance.
static zen::Value makeMoveResult(zen::VM* vm, const CharacterController::MoveResult& result)
{
    zen::ObjClass* klass = findClass(vm, "MoveResult");
    if (!klass)
        return zen::val_nil();
    const zen::Value instance = vm->make_instance(klass);
    zen::ObjInstance* inst = zen::as_instance(instance);
    inst->fields[0] = zen::val_bool(result.collided);
    inst->fields[1] = zen::val_bool(result.grounded);
    inst->fields[2] = makeVec3(vm, result.normal);
    inst->fields[3] = makeVec3(vm, result.displacement);
    return instance;
}

// A GameObject id, resolved against a Scene fresh on every call rather than
// cached as a pointer - id 0 is the reserved sentinel for the scene root
// (GameObject::id() never returns 0 for a real object; Scene.h:73), and any
// other id that Scene::findGameObject() no longer knows is an object that has
// been destroyed. Shared by every GameObject handle's self-resolution below
// and by readGameObject(), so a destroyed object simply stops resolving
// everywhere at once, the instant Scene::forgetIdBranch() drops its id -
// before the delete that follows it (Scene.cpp:1884-1885) - with no separate
// invalidation step required.
static GameObject* resolveGameObjectById(Scene* scene, u64 id)
{
    if (!scene)
        return nullptr;
    return id == 0 ? &scene->root() : scene->findGameObject(id);
}

// A GameObject handle carries the Scene it belongs to (as native_data) and
// the object's own id (its one field) rather than a raw GameObject* - see
// resolveGameObjectById() above for why. `scene` is passed in explicitly
// (not read off `object`) so a handle can still be built for the scene root,
// whose own GameObject::scene() is set but which callers may not always have
// resolved through an existing object first.
static zen::Value makeGameObjectValue(zen::VM* vm, Scene* scene, GameObject* object)
{
    zen::ObjClass* klass = findClass(vm, "GameObject");
    if (!klass || !scene || !object)
        return zen::val_nil();
    const zen::Value instance = vm->make_instance(klass);
    zen::ObjInstance* inst = zen::as_instance(instance);
    inst->native_data = scene;
    inst->fields[0] = zen::val_int((s64)object->id());
    return instance;
}

// Recorded once, when sceneScriptBindingsInit() defines "GameObject" - what
// readGameObject() below compares an argument's class against. Every
// component handle (Camera, ...) also carries a native_data pointer, so
// is_instance() alone cannot tell a GameObject handle from one of those; the
// class identity check is what makes readGameObject() safe to call on any
// argument a script hands in.
static zen::ObjClass* gGameObjectClass = nullptr;

// The inverse of makeGameObjectValue(): nullptr unless `value` is an
// instance of exactly the GameObject class (see gGameObjectClass above), or
// its id no longer resolves in its own Scene - a handle to an object that
// has since been destroyed reads as absent here, not as whatever now
// occupies its old address.
static GameObject* readGameObject(zen::Value value)
{
    if (!zen::is_instance(value) || zen::as_instance(value)->klass != gGameObjectClass)
        return nullptr;
    zen::ObjInstance* inst = zen::as_instance(value);
    Scene* scene = static_cast<Scene*>(inst->native_data);
    return resolveGameObjectById(scene, (u64)zen::to_integer(inst->fields[0]));
}

// AnimationLayer handles never keep an AnimationLayer* directly: Animator::layer()
// resizes mLayers on demand (Animation.cpp:90-95), so a pointer taken before a
// later get_layer() call could dangle once the vector reallocates. Nor do they
// keep the owning Animator* - the Animator itself can be removed out from
// under the handle - so the handle instead carries the Scene (as native_data),
// the owning GameObject's id and the layer index, the same by-id resolution as
// a GameObject handle, built fresh here instead of through
// ScriptCache::instanceFor().
static zen::Value makeAnimationLayerHandle(zen::VM* vm, GameObject* object, u32 index)
{
    zen::ObjClass* klass = findClass(vm, "AnimationLayer");
    if (!klass || !object || !object->scene())
        return zen::val_nil();
    const zen::Value instance = vm->make_instance(klass);
    zen::ObjInstance* inst = zen::as_instance(instance);
    inst->native_data = object->scene();
    inst->fields[0] = zen::val_int((s64)object->id());
    inst->fields[1] = zen::val_int((s64)index);
    return instance;
}

// Native methods called through a script dot-call ("self.set_position(...)")
// follow the ClassBuilder convention: self sits one slot before the args
// array the VM hands the native function, i.e. args[-1]. Resolves by id every
// call (see resolveGameObjectById) and returns nullptr once the object no
// longer exists - every go*() native below has to treat that as the normal
// "empty" case, never a reason to crash.
static GameObject* selfGameObject(zen::Value* args)
{
    zen::ObjInstance* inst = zen::as_instance(args[-1]);
    Scene* scene = static_cast<Scene*>(inst->native_data);
    return resolveGameObjectById(scene, (u64)zen::to_integer(inst->fields[0]));
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

static MeshRenderer* selfMeshRenderer(zen::Value* args)
{
    return zen::zen_instance_data<MeshRenderer>(args[-1]);
}

static CharacterController* selfCharacterController(zen::Value* args)
{
    return zen::zen_instance_data<CharacterController>(args[-1]);
}

static Animator* selfAnimator(zen::Value* args)
{
    return zen::zen_instance_data<Animator>(args[-1]);
}

// Resolves an AnimationLayer handle's owning GameObject by id, then asks it
// for its current Animator - not the Animator the handle was originally built
// from, since that one may since have been removed. Used both by
// selfAnimationLayer() below and by the two mask natives, which need the
// Animator itself (for its skeleton) rather than a specific layer - the same
// selfAnimator(args) trick the reference this is ported from used no longer
// applies once an AnimationLayer's native_data is a Scene*, not an Animator*.
static Animator* animatorForLayerHandle(zen::Value* args)
{
    zen::ObjInstance* inst = zen::as_instance(args[-1]);
    Scene* scene = static_cast<Scene*>(inst->native_data);
    GameObject* object = resolveGameObjectById(scene, (u64)zen::to_integer(inst->fields[0]));
    return object ? object->getComponent<Animator>() : nullptr;
}

// Resolves an AnimationLayer handle's index (its second field) against
// whatever Animator its owning GameObject currently has, at the moment of the
// call - see makeAnimationLayerHandle for why neither the Animator nor the
// AnimationLayer itself is ever cached on the handle.
static AnimationLayer* selfAnimationLayer(zen::Value* args)
{
    Animator* animator = animatorForLayerHandle(args);
    if (!animator)
        return nullptr;
    zen::ObjInstance* inst = zen::as_instance(args[-1]);
    const u32 index = (u32)zen::to_integer(inst->fields[1]);
    return &animator->layer(index);
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
    GameObject* object = selfGameObject(args);
    const std::string empty;
    const std::string& name = object ? object->name() : empty;
    args[0] = zen::val_obj((zen::Obj*)vm->make_string(name.c_str(), (int)name.size()));
    return 1;
}

static int goSetName(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (GameObject* object = selfGameObject(args))
        if (nargs >= 1 && zen::is_string(args[0]))
            object->setName(
                std::string(zen::safe_string_chars(args[0]), (usize)zen::safe_string_len(args[0])));
    return 0;
}

static int goGetActive(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    GameObject* object = selfGameObject(args);
    args[0] = zen::val_bool(object && object->active());
    return 1;
}

static int goSetActive(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (GameObject* object = selfGameObject(args))
        if (nargs >= 1 && zen::is_bool(args[0]))
            object->setActive(args[0].as.boolean);
    return 0;
}

static int goGetPosition(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    GameObject* object = selfGameObject(args);
    args[0] = makeVec3(vm, object ? object->position() : glm::vec3(0.0f));
    return 1;
}

static int goSetPosition(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (GameObject* object = selfGameObject(args))
        if (nargs >= 1 && isVec3Instance(args[0]))
            object->setPosition(readVec3(args[0]));
    return 0;
}

static int goGetScale(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    GameObject* object = selfGameObject(args);
    args[0] = makeVec3(vm, object ? object->scale() : glm::vec3(0.0f));
    return 1;
}

static int goSetScale(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (GameObject* object = selfGameObject(args))
        if (nargs >= 1 && isVec3Instance(args[0]))
            object->setScale(readVec3(args[0]));
    return 0;
}

static int goGetRotation(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    GameObject* object = selfGameObject(args);
    const glm::vec3 degrees =
        object ? glm::degrees(glm::eulerAngles(object->rotation())) : glm::vec3(0.0f);
    args[0] = makeVec3(vm, degrees);
    return 1;
}

static int goSetRotation(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (GameObject* object = selfGameObject(args))
        if (nargs >= 1 && isVec3Instance(args[0]))
            object->setRotationDegrees(readVec3(args[0]));
    return 0;
}

static int goYaw(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (GameObject* object = selfGameObject(args))
        if (nargs >= 1)
            object->yaw(static_cast<f32>(zen::to_number(args[0])));
    return 0;
}

static int goPitch(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (GameObject* object = selfGameObject(args))
        if (nargs >= 1)
            object->pitch(static_cast<f32>(zen::to_number(args[0])));
    return 0;
}

static int goRoll(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (GameObject* object = selfGameObject(args))
        if (nargs >= 1)
            object->roll(static_cast<f32>(zen::to_number(args[0])));
    return 0;
}

static int goGetParent(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    GameObject* object = selfGameObject(args);
    args[0] = object ? makeGameObjectValue(vm, object->scene(), object->parent()) : zen::val_nil();
    return 1;
}

static int goGetRoot(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    GameObject* object = selfGameObject(args);
    args[0] = object ? makeGameObjectValue(vm, object->scene(), object->root()) : zen::val_nil();
    return 1;
}

static int goGetChildCount(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    GameObject* object = selfGameObject(args);
    args[0] = zen::val_int(object ? (s64)object->childCount() : 0);
    return 1;
}

// childCount() is checked here, before child() ever runs, rather than
// relying on GameObject::child()'s own bounds check - keeps the C++ side
// from ever being asked to index past mChildren from a script-picked value.
static int goGetChild(zen::VM* vm, zen::Value* args, int nargs)
{
    GameObject* object = selfGameObject(args);
    if (!object)
    {
        args[0] = zen::val_nil();
        return 1;
    }
    const s64 index = nargs >= 1 ? zen::to_integer(args[0]) : -1;
    GameObject* child = (index >= 0 && (usize)index < object->childCount())
        ? object->child((usize)index)
        : nullptr;
    args[0] = makeGameObjectValue(vm, object->scene(), child);
    return 1;
}

static int goFindChild(zen::VM* vm, zen::Value* args, int nargs)
{
    GameObject* object = selfGameObject(args);
    if (!object || nargs < 1 || !zen::is_string(args[0]))
    {
        args[0] = zen::val_nil();
        return 1;
    }
    const std::string name(zen::safe_string_chars(args[0]), (usize)zen::safe_string_len(args[0]));
    const bool recursive = nargs >= 2 ? zen::is_truthy(args[1]) : true;
    args[0] = makeGameObjectValue(vm, object->scene(), object->findChild(name, recursive));
    return 1;
}

// dispose() only raises a flag (GameObject::dispose()) - the object is still
// fully alive right after this call returns. Scene::update() sweeps every
// disposed object into its destroy queue at the end of the frame
// (Scene.cpp:522-529), and only the flushChanges() that follows actually
// deletes it - which is why disposed() can come back true on an object a
// script can still otherwise reach.
static int goDispose(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    if (GameObject* object = selfGameObject(args))
        object->dispose();
    return 0;
}

static int goIsDisposed(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    GameObject* object = selfGameObject(args);
    args[0] = zen::val_bool(object && object->disposed());
    return 1;
}

static int goGetGlobalPosition(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    GameObject* object = selfGameObject(args);
    args[0] = makeVec3(vm, object ? object->globalPosition() : glm::vec3(0.0f));
    return 1;
}

static int goSetGlobalPosition(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (GameObject* object = selfGameObject(args))
        if (nargs >= 1 && isVec3Instance(args[0]))
            object->setGlobalPosition(readVec3(args[0]));
    return 0;
}

static int goGetGlobalRotation(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    GameObject* object = selfGameObject(args);
    const glm::vec3 degrees =
        object ? glm::degrees(glm::eulerAngles(object->globalRotation())) : glm::vec3(0.0f);
    args[0] = makeVec3(vm, degrees);
    return 1;
}

// Same Euler-degrees-to-quaternion conversion as GameObject::setRotationDegrees()
// (GameObject.cpp:457-465), applied to the global rotation instead of the
// local one.
static int goSetGlobalRotation(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (GameObject* object = selfGameObject(args))
        if (nargs >= 1 && isVec3Instance(args[0]))
            object->setGlobalRotation(glm::quat(glm::radians(readVec3(args[0]))));
    return 0;
}

// Always TransformSpace::Local, GameObject::translate()'s own default -
// scripts have no way to ask for Parent or World space here.
static int goTranslate(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (GameObject* object = selfGameObject(args))
        if (nargs >= 1 && isVec3Instance(args[0]))
            object->translate(readVec3(args[0]));
    return 0;
}

static int goMoveForward(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (GameObject* object = selfGameObject(args))
        if (nargs >= 1)
            object->moveForward(static_cast<f32>(zen::to_number(args[0])));
    return 0;
}

static int goMoveRight(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (GameObject* object = selfGameObject(args))
        if (nargs >= 1)
            object->moveRight(static_cast<f32>(zen::to_number(args[0])));
    return 0;
}

static int goMoveUp(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (GameObject* object = selfGameObject(args))
        if (nargs >= 1)
            object->moveUp(static_cast<f32>(zen::to_number(args[0])));
    return 0;
}

static int goLookAt(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    GameObject* object = selfGameObject(args);
    if (!object || nargs < 1 || !isVec3Instance(args[0]))
        return 0;
    const glm::vec3 up =
        (nargs >= 2 && isVec3Instance(args[1])) ? readVec3(args[1]) : glm::vec3(0, 1, 0);
    object->lookAt(readVec3(args[0]), up);
    return 0;
}

static int goGetStatic(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    GameObject* object = selfGameObject(args);
    args[0] = zen::val_bool(object && object->isStatic());
    return 1;
}

static int goSetStatic(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (GameObject* object = selfGameObject(args))
        if (nargs >= 1)
            object->setStatic(zen::is_truthy(args[0]));
    return 0;
}

static int goIsActiveInHierarchy(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    GameObject* object = selfGameObject(args);
    args[0] = zen::val_bool(object && object->isActiveInHierarchy());
    return 1;
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
    else if (!std::strcmp(name, "MeshRenderer"))
        args[0] = cache.instanceFor(cache.meshRendererClass(), object->getComponent<MeshRenderer>());
    else if (!std::strcmp(name, "CharacterController"))
        args[0] = cache.instanceFor(cache.characterControllerClass(),
                                    object->getComponent<CharacterController>());
    else if (!std::strcmp(name, "Animator"))
        args[0] = cache.instanceFor(cache.animatorClass(), object->getComponent<Animator>());
    else
        args[0] = zen::val_nil();

    return 1;
}

// Same class dispatch as goGetComponent(), over the four component classes
// that can actually be constructed from script. Light is left out on
// purpose: Light is only the script-facing base class for four concrete C++
// types (DirectionalLight/PointLight/SpotLight/RectangleLight), none of
// which has a script class of its own to instantiate - add_component(Light)
// would have nothing to build.
static int goAddComponent(zen::VM* vm, zen::Value* args, int nargs)
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
        args[0] = cache.instanceFor(cache.cameraClass(), object->addComponent<Camera>());
    else if (!std::strcmp(name, "MeshRenderer"))
        args[0] = cache.instanceFor(cache.meshRendererClass(), object->addComponent<MeshRenderer>());
    else if (!std::strcmp(name, "CharacterController"))
        args[0] = cache.instanceFor(cache.characterControllerClass(),
                                    object->addComponent<CharacterController>());
    else if (!std::strcmp(name, "Animator"))
        args[0] = cache.instanceFor(cache.animatorClass(), object->addComponent<Animator>());
    else
        args[0] = zen::val_nil();

    return 1;
}

static int goRemoveComponent(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    GameObject* object = selfGameObject(args);
    if (!object || nargs < 1 || !zen::is_class(args[0]))
    {
        args[0] = zen::val_bool(false);
        return 1;
    }

    zen::ObjClass* requested = zen::as_class(args[0]);
    const char* name = requested->name ? requested->name->chars : "";

    bool removed = false;
    if (!std::strcmp(name, "Camera"))
        removed = object->removeComponent<Camera>();
    else if (!std::strcmp(name, "MeshRenderer"))
        removed = object->removeComponent<MeshRenderer>();
    else if (!std::strcmp(name, "CharacterController"))
        removed = object->removeComponent<CharacterController>();
    else if (!std::strcmp(name, "Animator"))
        removed = object->removeComponent<Animator>();

    args[0] = zen::val_bool(removed);
    return 1;
}

static int goHasComponent(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    GameObject* object = selfGameObject(args);
    if (!object || nargs < 1 || !zen::is_class(args[0]))
    {
        args[0] = zen::val_bool(false);
        return 1;
    }

    zen::ObjClass* requested = zen::as_class(args[0]);
    const char* name = requested->name ? requested->name->chars : "";

    bool has = false;
    if (!std::strcmp(name, "Camera"))
        has = object->getComponent<Camera>() != nullptr;
    else if (!std::strcmp(name, "MeshRenderer"))
        has = object->getComponent<MeshRenderer>() != nullptr;
    else if (!std::strcmp(name, "CharacterController"))
        has = object->getComponent<CharacterController>() != nullptr;
    else if (!std::strcmp(name, "Animator"))
        has = object->getComponent<Animator>() != nullptr;

    args[0] = zen::val_bool(has);
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
    args[0] = makeGameObjectValue(vm, scene, scene->findGameObject(name));
    return 1;
}

// create(name, parent) - parent is optional; without it (or with anything
// that is not a real GameObject handle) createGameObject() gets nullptr,
// which puts the new object under the scene root exactly as it already did
// before this second argument existed.
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
    GameObject* parent = nargs >= 2 ? readGameObject(args[1]) : nullptr;
    args[0] = makeGameObjectValue(vm, scene, scene->createGameObject(name, parent));
    return 1;
}

// destroy() is queued exactly like GameObject.dispose() - see goDispose()
// above and Scene::update()/flushChanges() (Scene.cpp:522-534) - the object
// only actually goes away at the end of the frame this is called in.
static int sceneDestroy(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    Scene* scene = selfScene(args);
    GameObject* object = nargs >= 1 ? readGameObject(args[0]) : nullptr;
    args[0] = zen::val_bool(scene && object && scene->destroy(object));
    return 1;
}

// Unlike destroy(), reparent() is immediate: Scene::reparent() (Scene.cpp:375-388)
// moves the object into its new parent's children right here, not through the
// pending queues flushChanges() drains at the end of the frame.
static int sceneReparent(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    Scene* scene = selfScene(args);
    GameObject* object = nargs >= 1 ? readGameObject(args[0]) : nullptr;
    GameObject* parent = nargs >= 2 ? readGameObject(args[1]) : nullptr;
    args[0] = zen::val_bool(scene && object && scene->reparent(object, parent));
    return 1;
}

static int sceneGetRoot(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    Scene* scene = selfScene(args);
    args[0] = scene ? makeGameObjectValue(vm, scene, &scene->root()) : zen::val_nil();
    return 1;
}

// AnimationLayer.play(clip, mode, blend_time) - mode and blend_time are
// optional, defaulting the same way Animator::play()/AnimationLayer::play()
// do in C++ (PlayMode::Loop, 0.2s), the same optional-tail pattern vec3Init
// uses above for Vec3(x, y, z).
static int animationLayerPlay(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    AnimationLayer* layer = selfAnimationLayer(args);
    if (!layer || nargs < 1 || !zen::is_string(args[0]))
        return 0;
    const std::string clip(zen::safe_string_chars(args[0]), (usize)zen::safe_string_len(args[0]));
    const PlayMode mode =
        nargs >= 2 ? static_cast<PlayMode>(zen::to_integer(args[1])) : PlayMode::Loop;
    const f32 blendTime = nargs >= 3 ? (f32)zen::to_number(args[2]) : 0.2f;
    layer->play(clip, mode, blendTime);
    return 0;
}

static int animationLayerCrossFade(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    AnimationLayer* layer = selfAnimationLayer(args);
    if (!layer || nargs < 1 || !zen::is_string(args[0]))
        return 0;
    const std::string clip(zen::safe_string_chars(args[0]), (usize)zen::safe_string_len(args[0]));
    const f32 duration = nargs >= 2 ? (f32)zen::to_number(args[1]) : 0.2f;
    layer->crossFade(clip, duration);
    return 0;
}

static int animationLayerPlayOneShot(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    AnimationLayer* layer = selfAnimationLayer(args);
    if (!layer || nargs < 2 || !zen::is_string(args[0]) || !zen::is_string(args[1]))
        return 0;
    const std::string clip(zen::safe_string_chars(args[0]), (usize)zen::safe_string_len(args[0]));
    const std::string returnTo(zen::safe_string_chars(args[1]),
                               (usize)zen::safe_string_len(args[1]));
    const f32 blendTime = nargs >= 3 ? (f32)zen::to_number(args[2]) : 0.2f;
    layer->playOneShot(clip, returnTo, blendTime);
    return 0;
}

static int animationLayerStop(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    if (AnimationLayer* layer = selfAnimationLayer(args))
        layer->stop();
    return 0;
}

static int animationLayerIsPaused(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    AnimationLayer* layer = selfAnimationLayer(args);
    args[0] = zen::val_bool(layer && layer->paused());
    return 1;
}

static int animationLayerSetPaused(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (AnimationLayer* layer = selfAnimationLayer(args))
        if (nargs >= 1)
            layer->setPaused(zen::is_truthy(args[0]));
    return 0;
}

static int animationLayerSetSpeed(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (AnimationLayer* layer = selfAnimationLayer(args))
        if (nargs >= 1)
            layer->setSpeed((f32)zen::to_number(args[0]));
    return 0;
}

static int animationLayerIsPlaying(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    AnimationLayer* layer = selfAnimationLayer(args);
    if (!layer || nargs < 1 || !zen::is_string(args[0]))
    {
        args[0] = zen::val_bool(false);
        return 1;
    }
    const std::string clip(zen::safe_string_chars(args[0]), (usize)zen::safe_string_len(args[0]));
    args[0] = zen::val_bool(layer->isPlaying(clip));
    return 1;
}

static int animationLayerGetCurrent(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    AnimationLayer* layer = selfAnimationLayer(args);
    if (!layer)
    {
        args[0] = zen::val_obj((zen::Obj*)vm->make_string("", 0));
        return 1;
    }
    const std::string& current = layer->current();
    args[0] = zen::val_obj((zen::Obj*)vm->make_string(current.c_str(), (int)current.size()));
    return 1;
}

static int animationLayerGetTime(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    AnimationLayer* layer = selfAnimationLayer(args);
    args[0] = zen::val_float(layer ? (f64)layer->time() : 0.0);
    return 1;
}

static int animationLayerGetDuration(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    AnimationLayer* layer = selfAnimationLayer(args);
    args[0] = zen::val_float(layer ? (f64)layer->duration() : 0.0);
    return 1;
}

static int animationLayerGetNormalizedTime(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    AnimationLayer* layer = selfAnimationLayer(args);
    args[0] = zen::val_float(layer ? (f64)layer->normalizedTime() : 0.0);
    return 1;
}

static int animationLayerGetWrappedTime(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    AnimationLayer* layer = selfAnimationLayer(args);
    args[0] = zen::val_float(layer ? (f64)layer->wrappedTime() : 0.0);
    return 1;
}

static int animationLayerIsFinished(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    AnimationLayer* layer = selfAnimationLayer(args);
    args[0] = zen::val_bool(layer && layer->finished());
    return 1;
}

static int animationLayerSeek(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (AnimationLayer* layer = selfAnimationLayer(args))
        if (nargs >= 1)
            layer->seek((f32)zen::to_number(args[0]));
    return 0;
}

// The skeleton a mask needs comes from the handle's own Animator, not from
// an argument - a no-op (rather than a crash) when the Animator has none.
static int animationLayerMaskAll(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    AnimationLayer* layer = selfAnimationLayer(args);
    const Animator* animator = animatorForLayerHandle(args);
    const Skeleton* skeleton = animator ? animator->skeleton() : nullptr;
    if (layer && skeleton && nargs >= 1)
        layer->maskAll(*skeleton, (f32)zen::to_number(args[0]));
    return 0;
}

static int animationLayerMaskFromBone(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    AnimationLayer* layer = selfAnimationLayer(args);
    const Animator* animator = animatorForLayerHandle(args);
    const Skeleton* skeleton = animator ? animator->skeleton() : nullptr;
    if (!layer || !skeleton || nargs < 2 || !zen::is_string(args[0]))
        return 0;
    const std::string rootBone(zen::safe_string_chars(args[0]),
                               (usize)zen::safe_string_len(args[0]));
    layer->maskFromBone(*skeleton, rootBone.c_str(), (f32)zen::to_number(args[1]));
    return 0;
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
    // PlayMode crosses into script as a plain int global, the same style as
    // a KEY_* constant - there is no PlayMode handle class, only the numbers
    // Animator::play()/AnimationLayer::play() already accept.
    const struct
    {
        const char* name;
        PlayMode mode;
    } playModeConstants[] = {
        {"PLAY_LOOP", PlayMode::Loop},
        {"PLAY_ONCE", PlayMode::Once},
        {"PLAY_PINGPONG", PlayMode::PingPong},
    };
    for (const auto& entry : playModeConstants)
        vm->def_global(entry.name, zen::val_int(static_cast<int>(entry.mode)));

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

    // Built only from C++ (makeMoveResult) as the return value of
    // CharacterController.move() - a script never constructs one directly.
    vm->def_class("MoveResult")
        .field("collided")
        .field("grounded")
        .field("normal")
        .field("displacement")
        .constructable(false)
        .persistent(false)
        .end();

    // Not a Component - an Animator layer slot, addressed by the owning
    // GameObject's id and an index rather than by its own identity (see
    // makeAnimationLayerHandle/selfAnimationLayer). native_data is set to the
    // Scene, exactly like a GameObject handle, not to an Animator*.
    vm->def_class("AnimationLayer")
        .field("object_id")
        .field("index")
        .method("play", animationLayerPlay, 3)
        .method("cross_fade", animationLayerCrossFade, 2)
        .method("play_one_shot", animationLayerPlayOneShot, 3)
        .method("stop", animationLayerStop, 0)
        .method("is_paused", animationLayerIsPaused, 0)
        .method("set_paused", animationLayerSetPaused, 1)
        .method("set_speed", animationLayerSetSpeed, 1)
        .method("is_playing", animationLayerIsPlaying, 1)
        .method("get_current", animationLayerGetCurrent, 0)
        .method("get_time", animationLayerGetTime, 0)
        .method("get_duration", animationLayerGetDuration, 0)
        .method("get_normalized_time", animationLayerGetNormalizedTime, 0)
        .method("get_wrapped_time", animationLayerGetWrappedTime, 0)
        .method("is_finished", animationLayerIsFinished, 0)
        .method("seek", animationLayerSeek, 1)
        .method("mask_all", animationLayerMaskAll, 1)
        .method("mask_from_bone", animationLayerMaskFromBone, 2)
        .constructable(false)
        .persistent(false)
        .end();

    // The GameObject/Scene wrappers carry nothing but a raw pointer and
    // define no native destructor, so they stay ordinary GC objects: a script
    // calling scene.find() on every frame drops one wrapper per frame and the
    // collector takes them. A persistent class would arena-allocate each one,
    // keep it out of the GC list and never free it.
    //
    // "id" is the one field: a GameObject handle resolves the object fresh
    // from its Scene (native_data) on every call (selfGameObject/
    // resolveGameObjectById above) rather than keeping a raw GameObject*, so
    // an object destroyed after the handle was made simply stops resolving -
    // it never leaves the handle pointing at freed memory or, worse, at
    // whatever a later allocation reuses that address for.
    gGameObjectClass = vm->def_class("GameObject")
        .field("id")
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
        .method("get_parent", goGetParent, 0)
        .method("get_root", goGetRoot, 0)
        .method("get_child_count", goGetChildCount, 0)
        .method("get_child", goGetChild, 1)
        .method("find_child", goFindChild, 2)
        .method("dispose", goDispose, 0)
        .method("is_disposed", goIsDisposed, 0)
        .method("get_global_position", goGetGlobalPosition, 0)
        .method("set_global_position", goSetGlobalPosition, 1)
        .method("get_global_rotation", goGetGlobalRotation, 0)
        .method("set_global_rotation", goSetGlobalRotation, 1)
        .method("translate", goTranslate, 1)
        .method("move_forward", goMoveForward, 1)
        .method("move_right", goMoveRight, 1)
        .method("move_up", goMoveUp, 1)
        .method("look_at", goLookAt, 2)
        .method("get_static", goGetStatic, 0)
        .method("set_static", goSetStatic, 1)
        .method("is_active_in_hierarchy", goIsActiveInHierarchy, 0)
        .method("get_component", goGetComponent, 1)
        .method("add_component", goAddComponent, 1)
        .method("remove_component", goRemoveComponent, 1)
        .method("has_component", goHasComponent, 1)
        .constructable(false)
        .persistent(false)
        .end();

    vm->def_class("Scene")
        .method("find", sceneFind, 1)
        .method("create", sceneCreate, 2)
        .method("destroy", sceneDestroy, 1)
        .method("reparent", sceneReparent, 2)
        .method("get_root", sceneGetRoot, 0)
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
        if (nargs >= 1 && isVec3Instance(args[0]))
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

static int meshRendererGetVisibleInReflections(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    MeshRenderer* renderer = selfMeshRenderer(args);
    args[0] = zen::val_bool(renderer && renderer->visibleInReflections());
    return 1;
}

static int meshRendererSetVisibleInReflections(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (MeshRenderer* renderer = selfMeshRenderer(args))
        if (nargs >= 1)
            renderer->setVisibleInReflections(zen::is_truthy(args[0]));
    return 0;
}

static int meshRendererIsSubmeshVisible(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    MeshRenderer* renderer = selfMeshRenderer(args);
    args[0] = zen::val_bool(renderer && nargs >= 1 &&
                            renderer->submeshVisible((u32)zen::to_integer(args[0])));
    return 1;
}

static int meshRendererSetSubmeshVisible(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (MeshRenderer* renderer = selfMeshRenderer(args))
        if (nargs >= 2)
            renderer->setSubmeshVisible((u32)zen::to_integer(args[0]), zen::is_truthy(args[1]));
    return 0;
}

static int meshRendererGetSubmeshCount(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    MeshRenderer* renderer = selfMeshRenderer(args);
    args[0] = zen::val_int(renderer ? (s64)renderer->submeshCount() : 0);
    return 1;
}

static int meshRendererGetHiddenSubmeshCount(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    MeshRenderer* renderer = selfMeshRenderer(args);
    args[0] = zen::val_int(renderer ? (s64)renderer->hiddenSubmeshes().size() : 0);
    return 1;
}

static int meshRendererGetMaterialOverrideCount(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    MeshRenderer* renderer = selfMeshRenderer(args);
    args[0] = zen::val_int(renderer ? (s64)renderer->materialOverrideCount() : 0);
    return 1;
}

static int meshRendererClearMaterialOverrides(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    if (MeshRenderer* renderer = selfMeshRenderer(args))
        renderer->clearMaterialOverrides();
    return 0;
}

static int meshRendererHasMesh(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    MeshRenderer* renderer = selfMeshRenderer(args);
    args[0] = zen::val_bool(renderer && renderer->mesh().valid());
    return 1;
}

// Shared tail of every set_* below. A desc with the same recipe resolves to
// the mesh already uploaded for it, so a script handing the same box to a
// hundred objects uploads one. The material comes from upload(), which fills
// a lit default in when the recipe carries none - without one the submesh
// would reach emitSubmesh() with no pipeline and be dropped in silence.
bool assignMesh(MeshRenderer* renderer, const MeshDesc& desc)
{
    // createMesh() uploads, so it needs a device. A script can reach here
    // without one - a headless test, a scene torn down after the GPU is gone -
    // and getSingleton() treats that as a caller bug rather than answering.
    if (!renderer || !GPU::tryGet())
        return false;
    const MeshHandle mesh = Assets().createMesh(desc);
    if (!mesh.valid())
        return false;
    renderer->setMesh(mesh);
    return true;
}

f32 argFloat(zen::Value* args, int nargs, int index, f32 fallback)
{
    return index < nargs ? static_cast<f32>(zen::to_number(args[index])) : fallback;
}

static int meshRendererSetBox(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    const glm::vec3 size(argFloat(args, nargs, 0, 1.0f), argFloat(args, nargs, 1, 1.0f),
                         argFloat(args, nargs, 2, 1.0f));
    const bool assigned = assignMesh(selfMeshRenderer(args), MeshDesc::box(size));
    args[0] = zen::val_bool(assigned);
    return 1;
}

static int meshRendererSetSphere(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    const bool assigned =
        assignMesh(selfMeshRenderer(args), MeshDesc::sphere(argFloat(args, nargs, 0, 0.5f), 16, 24));
    args[0] = zen::val_bool(assigned);
    return 1;
}

static int meshRendererSetPlane(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    const bool assigned =
        assignMesh(selfMeshRenderer(args), MeshDesc::plane(argFloat(args, nargs, 0, 1.0f),
                                                           argFloat(args, nargs, 1, 1.0f), 1, 1,
                                                           1.0f));
    args[0] = zen::val_bool(assigned);
    return 1;
}

static int meshRendererSetCylinder(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    const bool assigned =
        assignMesh(selfMeshRenderer(args), MeshDesc::cylinder(argFloat(args, nargs, 0, 0.5f),
                                                              argFloat(args, nargs, 1, 1.0f), 24));
    args[0] = zen::val_bool(assigned);
    return 1;
}

static int meshRendererSetCone(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    const bool assigned =
        assignMesh(selfMeshRenderer(args), MeshDesc::cone(argFloat(args, nargs, 0, 0.5f),
                                                          argFloat(args, nargs, 1, 1.0f), 24));
    args[0] = zen::val_bool(assigned);
    return 1;
}

static int meshRendererSetCapsule(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    const bool assigned =
        assignMesh(selfMeshRenderer(args), MeshDesc::capsule(argFloat(args, nargs, 0, 0.4f),
                                                             argFloat(args, nargs, 1, 1.0f), 8, 24));
    args[0] = zen::val_bool(assigned);
    return 1;
}

static int meshRendererSetTorus(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    const bool assigned =
        assignMesh(selfMeshRenderer(args), MeshDesc::torus(argFloat(args, nargs, 0, 1.0f),
                                                           argFloat(args, nargs, 1, 0.25f), 32, 16));
    args[0] = zen::val_bool(assigned);
    return 1;
}

static int meshRendererSetMeshFile(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    MeshRenderer* renderer = selfMeshRenderer(args);
    if (!renderer || nargs < 1 || !zen::is_string(args[0]))
    {
        args[0] = zen::val_bool(false);
        return 1;
    }
    const std::string file(zen::safe_string_chars(args[0]), (usize)zen::safe_string_len(args[0]));
    args[0] = zen::val_bool(assignMesh(renderer, MeshDesc::fromFile(file)));
    return 1;
}

static int characterControllerGetRadius(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    CharacterController* controller = selfCharacterController(args);
    args[0] = zen::val_float(controller ? (f64)controller->radius() : 0.0);
    return 1;
}

static int characterControllerSetRadius(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (CharacterController* controller = selfCharacterController(args))
        if (nargs >= 1)
            controller->setRadius((f32)zen::to_number(args[0]));
    return 0;
}

static int characterControllerGetHeight(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    CharacterController* controller = selfCharacterController(args);
    args[0] = zen::val_float(controller ? (f64)controller->height() : 0.0);
    return 1;
}

static int characterControllerSetHeight(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (CharacterController* controller = selfCharacterController(args))
        if (nargs >= 1)
            controller->setHeight((f32)zen::to_number(args[0]));
    return 0;
}

static int characterControllerGetStepOffset(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    CharacterController* controller = selfCharacterController(args);
    args[0] = zen::val_float(controller ? (f64)controller->stepOffset() : 0.0);
    return 1;
}

static int characterControllerSetStepOffset(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (CharacterController* controller = selfCharacterController(args))
        if (nargs >= 1)
            controller->setStepOffset((f32)zen::to_number(args[0]));
    return 0;
}

static int characterControllerGetSlopeLimit(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    CharacterController* controller = selfCharacterController(args);
    args[0] = zen::val_float(controller ? (f64)controller->slopeLimit() : 0.0);
    return 1;
}

static int characterControllerSetSlopeLimit(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (CharacterController* controller = selfCharacterController(args))
        if (nargs >= 1)
            controller->setSlopeLimit((f32)zen::to_number(args[0]));
    return 0;
}

static int characterControllerGetSkinWidth(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    CharacterController* controller = selfCharacterController(args);
    args[0] = zen::val_float(controller ? (f64)controller->skinWidth() : 0.0);
    return 1;
}

static int characterControllerSetSkinWidth(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (CharacterController* controller = selfCharacterController(args))
        if (nargs >= 1)
            controller->setSkinWidth((f32)zen::to_number(args[0]));
    return 0;
}

static int characterControllerGetGravity(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    CharacterController* controller = selfCharacterController(args);
    args[0] = zen::val_float(controller ? (f64)controller->gravity() : 0.0);
    return 1;
}

static int characterControllerSetGravity(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (CharacterController* controller = selfCharacterController(args))
        if (nargs >= 1)
            controller->setGravity((f32)zen::to_number(args[0]));
    return 0;
}

static int characterControllerGetMaxFallSpeed(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    CharacterController* controller = selfCharacterController(args);
    args[0] = zen::val_float(controller ? (f64)controller->maxFallSpeed() : 0.0);
    return 1;
}

static int characterControllerSetMaxFallSpeed(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (CharacterController* controller = selfCharacterController(args))
        if (nargs >= 1)
            controller->setMaxFallSpeed((f32)zen::to_number(args[0]));
    return 0;
}

static int characterControllerGetMaxIterations(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    CharacterController* controller = selfCharacterController(args);
    args[0] = zen::val_int(controller ? (s64)controller->maxIterations() : 0);
    return 1;
}

static int characterControllerSetMaxIterations(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (CharacterController* controller = selfCharacterController(args))
        if (nargs >= 1)
            controller->setMaxIterations((u32)zen::to_integer(args[0]));
    return 0;
}

static int characterControllerGetMoveInput(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    CharacterController* controller = selfCharacterController(args);
    args[0] = controller ? makeVec3(vm, controller->moveInput()) : zen::val_nil();
    return 1;
}

static int characterControllerSetMoveInput(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (CharacterController* controller = selfCharacterController(args))
        if (nargs >= 1 && isVec3Instance(args[0]))
            controller->setMoveInput(readVec3(args[0]));
    return 0;
}

static int characterControllerJump(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (CharacterController* controller = selfCharacterController(args))
        if (nargs >= 1)
            controller->jump((f32)zen::to_number(args[0]));
    return 0;
}

static int characterControllerTeleport(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (CharacterController* controller = selfCharacterController(args))
        if (nargs >= 1 && isVec3Instance(args[0]))
            controller->teleport(readVec3(args[0]));
    return 0;
}

static int characterControllerIsGrounded(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    CharacterController* controller = selfCharacterController(args);
    args[0] = zen::val_bool(controller && controller->isGrounded());
    return 1;
}

static int characterControllerGetGroundNormal(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    CharacterController* controller = selfCharacterController(args);
    args[0] = controller ? makeVec3(vm, controller->groundNormal()) : zen::val_nil();
    return 1;
}

static int characterControllerGetSlopeAngle(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    CharacterController* controller = selfCharacterController(args);
    args[0] = zen::val_float(controller ? (f64)controller->slopeAngle() : 0.0);
    return 1;
}

static int characterControllerGetVelocity(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)nargs;
    CharacterController* controller = selfCharacterController(args);
    args[0] = controller ? makeVec3(vm, controller->velocity()) : zen::val_nil();
    return 1;
}

// move()'s result is not readable back through is_grounded()/get_velocity()/
// get_ground_normal() - those only update in onUpdate() - so the whole
// MoveResult has to be handed back here instead.
static int characterControllerMove(zen::VM* vm, zen::Value* args, int nargs)
{
    CharacterController* controller = selfCharacterController(args);
    if (!controller || nargs < 1 || !isVec3Instance(args[0]))
    {
        args[0] = zen::val_nil();
        return 1;
    }
    const CharacterController::MoveResult result = controller->move(readVec3(args[0]));
    args[0] = makeMoveResult(vm, result);
    return 1;
}

static int animatorIsBound(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    Animator* animator = selfAnimator(args);
    args[0] = zen::val_bool(animator && animator->bound());
    return 1;
}

// mode and blend_time are optional, the same defaults Animator::play() gives
// them in C++ (PlayMode::Loop, 0.2s).
static int animatorPlay(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    Animator* animator = selfAnimator(args);
    if (!animator || nargs < 1 || !zen::is_string(args[0]))
        return 0;
    const std::string clip(zen::safe_string_chars(args[0]), (usize)zen::safe_string_len(args[0]));
    const PlayMode mode =
        nargs >= 2 ? static_cast<PlayMode>(zen::to_integer(args[1])) : PlayMode::Loop;
    const f32 blendTime = nargs >= 3 ? (f32)zen::to_number(args[2]) : 0.2f;
    animator->play(clip, mode, blendTime);
    return 0;
}

static int animatorGetLayerCount(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    Animator* animator = selfAnimator(args);
    args[0] = zen::val_int(animator ? (s64)animator->layerCount() : 0);
    return 1;
}

// Calling animator->layer(index) here is what grows mLayers when index is
// past the current count - the handle handed back only ever stores the
// index, never the resulting reference (see selfAnimationLayer).
static int animatorGetLayer(zen::VM* vm, zen::Value* args, int nargs)
{
    Animator* animator = selfAnimator(args);
    if (!animator || nargs < 1)
    {
        args[0] = zen::val_nil();
        return 1;
    }
    const u32 index = (u32)zen::to_integer(args[0]);
    animator->layer(index);
    args[0] = makeAnimationLayerHandle(vm, animator->owner(), index);
    return 1;
}

static int animatorGetBoneCount(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    Animator* animator = selfAnimator(args);
    const Skeleton* skeleton = animator ? animator->skeleton() : nullptr;
    args[0] = zen::val_int(skeleton ? (s64)skeleton->boneCount() : 0);
    return 1;
}

static int animatorFindBone(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    Animator* animator = selfAnimator(args);
    const Skeleton* skeleton = animator ? animator->skeleton() : nullptr;
    if (!skeleton || nargs < 1 || !zen::is_string(args[0]))
    {
        args[0] = zen::val_int(-1);
        return 1;
    }
    const std::string name(zen::safe_string_chars(args[0]), (usize)zen::safe_string_len(args[0]));
    args[0] = zen::val_int((s64)skeleton->findBone(name.c_str()));
    return 1;
}

static int animatorGetBonePosition(zen::VM* vm, zen::Value* args, int nargs)
{
    Animator* animator = selfAnimator(args);
    glm::vec3 position;
    if (!animator || nargs < 1 ||
        !animator->boneGlobalPosition((s32)zen::to_integer(args[0]), position))
    {
        args[0] = zen::val_nil();
        return 1;
    }
    args[0] = makeVec3(vm, position);
    return 1;
}

static int animatorGetPoseEditMode(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    Animator* animator = selfAnimator(args);
    args[0] = zen::val_bool(animator && animator->poseEditMode());
    return 1;
}

static int animatorSetPoseEditMode(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    if (Animator* animator = selfAnimator(args))
        if (nargs >= 1)
            animator->setPoseEditMode(zen::is_truthy(args[0]));
    return 0;
}

static int animatorGetIKChainCount(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    Animator* animator = selfAnimator(args);
    args[0] = zen::val_int(animator ? (s64)animator->ikChainCount() : 0);
    return 1;
}

static int animatorClearIKChains(zen::VM* vm, zen::Value* args, int nargs)
{
    (void)vm;
    (void)nargs;
    if (Animator* animator = selfAnimator(args))
        animator->clearIKChains();
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

    auto meshRenderer = vm.def_class("MeshRenderer");
    meshRenderer.parent("Component");
    meshRenderer.method("get_visible_in_reflections", meshRendererGetVisibleInReflections, 0);
    meshRenderer.method("set_visible_in_reflections", meshRendererSetVisibleInReflections, 1);
    meshRenderer.method("is_submesh_visible", meshRendererIsSubmeshVisible, 1);
    meshRenderer.method("set_submesh_visible", meshRendererSetSubmeshVisible, 2);
    meshRenderer.method("get_submesh_count", meshRendererGetSubmeshCount, 0);
    meshRenderer.method("get_hidden_submesh_count", meshRendererGetHiddenSubmeshCount, 0);
    meshRenderer.method("get_material_override_count", meshRendererGetMaterialOverrideCount, 0);
    meshRenderer.method("clear_material_overrides", meshRendererClearMaterialOverrides, 0);
    meshRenderer.method("has_mesh", meshRendererHasMesh, 0);
    meshRenderer.method("set_box", meshRendererSetBox, 3);
    meshRenderer.method("set_sphere", meshRendererSetSphere, 1);
    meshRenderer.method("set_plane", meshRendererSetPlane, 2);
    meshRenderer.method("set_cylinder", meshRendererSetCylinder, 2);
    meshRenderer.method("set_cone", meshRendererSetCone, 2);
    meshRenderer.method("set_capsule", meshRendererSetCapsule, 2);
    meshRenderer.method("set_torus", meshRendererSetTorus, 2);
    meshRenderer.method("set_mesh_file", meshRendererSetMeshFile, 1);
    meshRenderer.persistent(true).constructable(false);
    cache.setMeshRendererClass(meshRenderer.end());

    auto characterController = vm.def_class("CharacterController");
    characterController.parent("Component");
    characterController.method("get_radius", characterControllerGetRadius, 0);
    characterController.method("set_radius", characterControllerSetRadius, 1);
    characterController.method("get_height", characterControllerGetHeight, 0);
    characterController.method("set_height", characterControllerSetHeight, 1);
    characterController.method("get_step_offset", characterControllerGetStepOffset, 0);
    characterController.method("set_step_offset", characterControllerSetStepOffset, 1);
    characterController.method("get_slope_limit", characterControllerGetSlopeLimit, 0);
    characterController.method("set_slope_limit", characterControllerSetSlopeLimit, 1);
    characterController.method("get_skin_width", characterControllerGetSkinWidth, 0);
    characterController.method("set_skin_width", characterControllerSetSkinWidth, 1);
    characterController.method("get_gravity", characterControllerGetGravity, 0);
    characterController.method("set_gravity", characterControllerSetGravity, 1);
    characterController.method("get_max_fall_speed", characterControllerGetMaxFallSpeed, 0);
    characterController.method("set_max_fall_speed", characterControllerSetMaxFallSpeed, 1);
    characterController.method("get_max_iterations", characterControllerGetMaxIterations, 0);
    characterController.method("set_max_iterations", characterControllerSetMaxIterations, 1);
    characterController.method("get_move_input", characterControllerGetMoveInput, 0);
    characterController.method("set_move_input", characterControllerSetMoveInput, 1);
    characterController.method("jump", characterControllerJump, 1);
    characterController.method("teleport", characterControllerTeleport, 1);
    characterController.method("is_grounded", characterControllerIsGrounded, 0);
    characterController.method("get_ground_normal", characterControllerGetGroundNormal, 0);
    characterController.method("get_slope_angle", characterControllerGetSlopeAngle, 0);
    characterController.method("get_velocity", characterControllerGetVelocity, 0);
    characterController.method("move", characterControllerMove, 1);
    characterController.persistent(true).constructable(false);
    cache.setCharacterControllerClass(characterController.end());

    // IK only exposes counting/clearing here: addIKChain()/ikChain() hand
    // back an IKChain& a caller mutates in place (moving a foot target every
    // frame), which needs its own script-facing class - not yet ported.
    auto animator = vm.def_class("Animator");
    animator.parent("Component");
    animator.method("is_bound", animatorIsBound, 0);
    animator.method("play", animatorPlay, 3);
    animator.method("get_layer_count", animatorGetLayerCount, 0);
    animator.method("get_layer", animatorGetLayer, 1);
    animator.method("get_bone_count", animatorGetBoneCount, 0);
    animator.method("find_bone", animatorFindBone, 1);
    animator.method("get_bone_position", animatorGetBonePosition, 1);
    animator.method("get_pose_edit_mode", animatorGetPoseEditMode, 0);
    animator.method("set_pose_edit_mode", animatorSetPoseEditMode, 1);
    animator.method("get_ik_chain_count", animatorGetIKChainCount, 0);
    animator.method("clear_ik_chains", animatorClearIKChains, 0);
    animator.persistent(true).constructable(false);
    cache.setAnimatorClass(animator.end());
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
static void setInstanceFieldUnpaused(zen::VM* vm, zen::Value instance, const char* name,
                                     zen::Value value)
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
        // num_fields is bumped only once field_names has actually been grown
        // to match and the new slot holds a real pointer: the GC's own class
        // marking walks field_names[0, num_fields) (memory.cpp's OBJ_CLASS
        // case), and zen_realloc() may call gc_collect() before it resizes
        // anything (memory.cpp's threshold check runs first) - bumping the
        // count first left a window where that walk read one slot past the
        // still-old, smaller array.
        classIndex = klass->num_fields;
        klass->field_names = (zen::ObjString**)zen::zen_realloc(
            &vm->get_gc(), klass->field_names, sizeof(zen::ObjString*) * classIndex,
            sizeof(zen::ObjString*) * (classIndex + 1));
        klass->field_names[classIndex] = key;
        klass->num_fields = classIndex + 1;
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

// The GC stays off for the whole of the above. Two things in there are
// unreachable from any root while it runs: the interned key returned by
// make_string(), until it is stored into field_names, and `value` itself,
// until it is stored into the instance. Both zen_realloc() calls can collect
// before they resize (memory.cpp checks the threshold first), and a
// collection there frees the unmarked key - gc_rebuild_intern_table() keeps
// only marked strings - leaving the class holding a dangling field name that
// no later lookup matches, which surfaces as "self.node is nil" long
// afterwards, on whichever script happens to be compiled next.
// new_instance() pauses across its own field allocation for the same reason.
static void setInstanceField(zen::VM* vm, zen::Value instance, const char* name, zen::Value value)
{
    zen::gc_pause(&vm->get_gc());
    setInstanceFieldUnpaused(vm, instance, name, value);
    zen::gc_resume(&vm->get_gc());
}

void SceneScriptBindings::bindOwner(zen::VM& vm, zen::Value instance, GameObject* owner)
{
    if (!owner || !zen::is_instance(instance))
        return;

    zen::ObjClass* sceneClass = findClass(&vm, "Scene");
    if (!sceneClass)
        return;

    // `node` is the public behaviour API, matching Kinetix2D. `owner` is
    // retained as an alias for existing Radion scripts, and both fields point
    // at the same lightweight wrapper rather than allocating two every
    // script instance.
    const zen::Value nodeValue = makeGameObjectValue(&vm, owner->scene(), owner);
    setInstanceField(&vm, instance, "node", nodeValue);
    setInstanceField(&vm, instance, "owner", nodeValue);

    const zen::Value sceneValue = vm.make_instance(sceneClass);
    zen::as_instance(sceneValue)->native_data = owner->scene();
    setInstanceField(&vm, instance, "scene", sceneValue);
}

zen::Value SceneScriptBindings::wrapGameObject(zen::VM& vm, GameObject* object)
{
    return makeGameObjectValue(&vm, object ? object->scene() : nullptr, object);
}

} // namespace Radion
