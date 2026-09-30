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

// One entry point per concern for the Zen script-facing classes.
class SceneScriptBindings
{
public:
    // Once per VM; defines Vec3/GameObject/Scene as globals with no script import.
    static const zen::NativeLib& library();

    // Sets "owner" and "scene" as fields on the instance, not globals: one zen::VM is shared by every scripted object (see ScriptCache).
    // Call right after creating the instance, before on_start().
    static void bindOwner(zen::VM& vm, zen::Value instance, GameObject* owner);

    static zen::Value wrapGameObject(zen::VM& vm, GameObject* object);

    // Called once by ScriptCache's constructor, with a direct reference rather than via getSingleton().
    static void registerComponentClasses(ScriptCache& cache);
};

} // namespace Radion

#endif // RADION_SCENE_SCRIPT_BINDINGS_H
