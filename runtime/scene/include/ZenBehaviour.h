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
// on_destroy(self); ZenBehaviour creates one instance of it per component
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

    const std::string& scriptPath() const;
    bool hasError() const;
    const std::string& lastError() const;

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

    // Runs on_collision(self, other), once per contact - called by
    // CollisionWorld, not through Component's own per-frame events. A no-op
    // when the script defines no on_collision, and gated in editor mode the
    // same as onUpdate().
    void onCollision(GameObject* other);

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
