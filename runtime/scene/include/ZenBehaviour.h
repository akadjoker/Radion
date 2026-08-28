#ifndef RADION_ZEN_BEHAVIOUR_H
#define RADION_ZEN_BEHAVIOUR_H

#include "ScriptCache.h"
#include "ScriptComponent.h"

#include "zen/value.h"

#include <string>

namespace Radion
{

class GameObject;

// Runs a Zen script class against the GameObject it is attached to. The
// script defines one class with any of on_start(self)/on_update(self, dt)/
// on_destroy(self)/on_collision(self, other, began)/on_event(self, event,
// value); ZenBehaviour creates one instance of it per component
// (through the shared ScriptCache) and reaches the owning GameObject
// through the instance's own "node" field (see SceneScriptBindings) - not a
// VM global, since the underlying zen::VM is shared by every scripted object
// in the process. `owner` remains an alias for older Radion scripts. Two
// scripted objects sharing the same script file share the compiled class but
// never share instance state.
class ZenBehaviour : public ScriptComponent
{
public:
    ZenBehaviour();
    ~ZenBehaviour() override;

    // Looks the source up in ScriptCache (compiling it there on first use),
    // replacing whatever was loaded before. Neither call creates the script
    // instance or invokes on_start() by itself - onUpdate() does both,
    // lazily, the first time the scene is not running in the editor.
    bool loadFile(const std::string& path);
    bool loadSource(const std::string& source);

    // Recompiles the cache entry for the path from the last loadFile() call
    // - every ZenBehaviour using that same path, not just this one, picks
    // up the change and re-instantiates on its next update. False (no-op)
    // if this behaviour was loaded from a source string instead of a file.
    bool reload();

    // Re-reads the file this behaviour was loaded from and calls reload()
    // only if its on-disk timestamp moved since the last load/reload - the
    // per-object hot-reload check. False (no-op) for a behaviour loaded from
    // a source string, or when the file has not changed.
    bool reloadIfChanged();
    // The source file's last-write time as of the last successful load or
    // reload, read straight off the shared ScriptCache entry - so it moves
    // for every component sharing this path, not only the one that called
    // reload(). 0 for a behaviour loaded from a source string.
    s64 sourceTimestamp() const;

    const std::string& scriptPath() const;
    bool hasError() const;
    const std::string& lastError() const;

    // Runs a named event hook - on_event(self, event, value) - if the
    // script's class defines one. False (no-op) when it does not.
    bool callEvent(const std::string& event, f64 value = 0.0);

    // Calls any function the script's class defines, by name, with one f64
    // argument - the general escape hatch beside the fixed on_start/
    // on_update/on_destroy/on_collision/on_event hooks. False (and
    // hasError()) if the class defines no such name.
    bool callFunction(const std::string& name, f64 value = 0.0);

    // Whether the script's class defines a method by this name - checked
    // against the compiled class's own vtable, no instance touched and no
    // call made.
    bool hasFunction(const std::string& name) const;

    // What the loaded script declares in its class body (or optionally in
    // __init__), collected by ScriptCache and shared by every component
    // running it. Empty until something loads successfully.
    usize declaredPropertyCount() const;
    const ScriptProperty* declaredPropertyAt(usize index) const;
    const ScriptProperty* declaredProperty(const std::string& name) const;

    // This component's own values, written over the script's defaults right
    // after __init__ runs. These are what the inspector edits and what the
    // scene file stores - two objects on the same script differ only here.
    usize overrideCount() const;
    const ScriptProperty* overrideAt(usize index) const;
    const ScriptProperty* findOverride(const std::string& name) const;
    void setNumberOverride(const std::string& name, f64 value, bool integer = false);
    void setStringOverride(const std::string& name, const std::string& value);
    void setBoolOverride(const std::string& name, bool value);
    void clearOverride(const std::string& name);
    void clearOverrides();
    usize applyOverrides();

    bool isZenBehaviour() const override;

    // Runs on_collision(self, other, began), once per contact - called by
    // CollisionWorld, not through Component's own per-frame events. A no-op
    // when the script defines no on_collision, and gated in editor mode the
    // same as onUpdate(). CollisionWorld re-detects every touching pair from
    // scratch each step with no enter/exit state of its own, so this always
    // passes began=true; see callCollision() for the began-aware call.
    void onCollision(GameObject* other);

    // The began-aware form onCollision() forwards to - exposed directly for
    // a caller (or a future CollisionWorld) that does know whether a contact
    // just started. A script written against the older on_collision(self,
    // other) still works unchanged: zen does not check argument count on a
    // native-invoked call (VM::invoke(instance, slot, args, nargs) writes
    // the extra register and never reads it back), so the added `began`
    // argument is simply ignored by a class that never declared it.
    bool callCollision(GameObject* other, bool began);

protected:
    void onUpdate(f32 deltaTime) override;
    void onDestroy() override;

private:
    bool ensureInstance();
    void releaseInstance();
    void fail(const std::string& error);
    ScriptProperty& overrideSlot(const std::string& name);
    bool writeProperty(const ScriptProperty& property);

    std::string mScriptPath;
    std::string mLastError;
    // Handed over by ScriptCache at load time and held from there on: it
    // stays valid for the cache's whole life, so the update path never pays
    // for a lookup. A reload rebuilds it in place and bumps its version,
    // which is what onUpdate() watches to re-instantiate.
    const ScriptCache::Entry* mEntry = nullptr;
    std::vector<ScriptProperty> mOverrides;
    zen::Value mInstance = zen::val_nil();
    int mBoundVersion = 0;
    bool mLoaded = false;
    bool mFailed = false;
    bool mStartCalled = false;
    bool mFromFile = false;
};

// Several ScriptComponent subclasses can occupy ComponentType::Script - this
// is what tells a Zen-driven behaviour apart from a hand-written C++ one on
// that shared slot, the same way Light.h's ComponentMatch specializations
// tell its four light kinds apart, without RTTI.
template <> struct ComponentMatch<ZenBehaviour>
{
    static bool test(const Component* component)
    {
        return static_cast<const ScriptComponent*>(component)->isZenBehaviour();
    }
};

} // namespace Radion

#endif // RADION_ZEN_BEHAVIOUR_H
