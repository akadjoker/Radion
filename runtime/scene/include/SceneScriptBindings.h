#ifndef RADION_SCENE_SCRIPT_BINDINGS_H
#define RADION_SCENE_SCRIPT_BINDINGS_H

#include "zen/value.h"

namespace zen
{
class VM;
struct NativeLib;
}

namespace Radion
{

class GameObject;
class ScriptCache;

// Registers the Vec3/GameObject/Scene native classes a Zen script sees, and
// binds a script instance to the GameObject it belongs to. One entry point
// per concern, instead of native callbacks scattered loose through the
// zen:: namespace.
class SceneScriptBindings
{
public:
    // Hand this to ScriptVM::registerModule() once per VM - its init_fn
    // defines the Vec3/GameObject/Scene classes as globals immediately, with
    // no "import" required from the script.
    static const zen::NativeLib& library();

    // Sets "owner" (a GameObject wrapper) and "scene" (a Scene wrapper) as
    // FIELDS on a script instance - not globals, since one zen::VM is now
    // shared by every scripted object in the process (see ScriptCache) and
    // a global "self"/"scene" would just be whichever behaviour bound it
    // last. Call once right after creating the instance, before invoking
    // its on_start().
    static void bindOwner(zen::VM& vm, zen::Value instance, GameObject* owner);

    // A standalone GameObject wrapper, the same kind bindOwner()'s "owner"
    // field and Scene.find()/Scene.create() hand back - for a native call
    // that needs to pass one as a plain argument instead (a collision
    // hook's "other").
    static zen::Value wrapGameObject(zen::VM& vm, GameObject* object);

    // Registers one native class per script-facing component (Camera, Light,
    // ...), each deriving "Component", and records its ObjClass pointer on
    // the given ScriptCache - the pointer GameObject's typed getters and
    // get_component() hand to ScriptCache::instanceFor(). Called once, by
    // ScriptCache's own constructor, with a direct reference to the cache
    // being constructed rather than through ScriptCache::getSingleton().
    static void registerComponentClasses(ScriptCache& cache);
};

} // namespace Radion

#endif // RADION_SCENE_SCRIPT_BINDINGS_H
